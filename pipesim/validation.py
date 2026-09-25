"""Design unit tests: references, sockets, reach, clearances, collisions and support."""
from __future__ import annotations
import copy
import itertools
import numpy as np
import pybullet as pb
from scipy.spatial import ConvexHull
from .document import Assembly, DocumentError, joint_kind, fingerprint
from .math3d import point, unit, UnionFind
from .geometry import bounds, mesh_for_part
from .physics import World

class CollisionWorld:
    """Separate static bodies retain collisions even within one rigid subassembly."""
    def __init__(self,assembly):
        separate=copy.copy(assembly)
        separate.joints=[]; separate.anchors=[]
        separate.doc={**assembly.doc,'environment':{'ground':False},'drives':[]}
        self.world=World(separate,static=True)
        self.assembly=assembly
        self.boxes={pid:bounds(p) for pid,p in assembly.parts.items()}
        self.positions={pid:pb.getBasePositionAndOrientation(entry[0],physicsClientId=self.world.client) for pid,entry in self.world.part_map.items()}
    def move(self,pid,delta):
        body=self.world.part_map[pid][0]; pos,quat=self.positions[pid]
        pb.resetBasePositionAndOrientation(body,np.array(pos)+np.array(delta)/1000,quat,physicsClientId=self.world.client)
    def contacts(self,a,b,tolerance_mm=.5,delta=None):
        ba=self.boxes[a]+(np.array(delta) if delta is not None else 0); bb=self.boxes[b]
        if np.any(ba[1]<bb[0]-tolerance_mm) or np.any(bb[1]<ba[0]-tolerance_mm): return []
        aa=self.world.part_map[a][0]; ab=self.world.part_map[b][0]
        return pb.getClosestPoints(aa,ab,distance=0,physicsClientId=self.world.client)
    def intersect(self,a,b,tolerance_mm=.5,delta=None):
        return any(c[8]*1000 < -tolerance_mm for c in self.contacts(a,b,tolerance_mm,delta))
    def close(self): self.world.close()
    def __enter__(self): return self
    def __exit__(self,*args): self.close()

def support(assembly,subset=None,tolerance_mm=1.):
    """Quasi-static gravity support polygon for each currently rigid component.

    Mounting fixtures are declared external constraints. Ground contacts are actual
    lowest mesh vertices. This checks gravity only, not friction or disturbances.
    """
    subset=set(assembly.parts if subset is None else subset)
    groups=UnionFind(subset)
    for j in assembly.joints:
        a,b=j['a']['part'],j['b']['part']
        if a in subset and b in subset and joint_kind(j)=='fixed': groups.union(a,b)
    anchored={a['part'] for a in assembly.anchors}
    ground=assembly.doc.get('environment',{}).get('ground',True)
    z=assembly.doc.get('environment',{}).get('ground_z_mm',0)
    results=[]
    for group in groups.groups():
        masses=[assembly.parts[p].mass for p in group]
        com=np.average([point(assembly.parts[p].matrix,assembly.parts[p].center_of_mass) for p in group],axis=0,weights=masses)
        if set(group)&anchored:
            results.append({'parts':group,'stable':True,'basis':'declared world anchor','center_of_mass_mm':com.tolist()}); continue
        contacts=[]
        if ground:
            for pid in group:
                mesh=mesh_for_part(assembly.parts[pid],True)
                contacts.extend(mesh.vertices[np.abs(mesh.vertices[:,2]-z)<=tolerance_mm,:2].tolist())
        points=np.unique(np.round(contacts,4),axis=0) if contacts else np.empty((0,2))
        stable=False; margin=None
        if len(points)>=3 and np.linalg.matrix_rank(points-points.mean(axis=0))==2:
            hull=ConvexHull(points)
            distances=-(hull.equations[:,:2]@com[:2]+hull.equations[:,2])
            margin=float(distances.min()); stable=margin>=-tolerance_mm
        results.append({'parts':group,'stable':bool(stable),'basis':'ground support polygon' if len(points) else 'no support','center_of_mass_mm':com.tolist(),'margin_mm':margin,'contacts_xy_mm':points.tolist()})
    return {'stable':all(g['stable'] for g in results),'components':results,'assumptions':['Gravity acts along -Z','Declared anchors have adequate capacity','No dynamic disturbances or friction analysis']}

def validate(assembly,collisions=True,build=False):
    issues=[]
    def issue(code,message,parts=(),severity='error',**details):
        issues.append({'code':code,'severity':severity,'message':message,'parts':list(parts),**details})
    occupied={}; member_ends={}
    for p in assembly.parts.values():
        try:
            if p.mass<=0: issue('MASS','Part mass must be positive; use an anchor for a fixed body',[p.id])
            if p.kind=='member' and p.length<=0: issue('LENGTH','Cut length must be positive',[p.id])
            for shape in p.shapes:
                if shape['type']=='tube' and not 0<shape['wall_mm']<shape['diameter_mm']/2:
                    issue('WALL','Tube wall must lie between zero and its radius',[p.id])
                for key in ('length_mm','diameter_mm','radius_mm'):
                    if key in shape and shape[key]<=0: issue('DIMENSION',f'{key} must be positive',[p.id])
                if 'size_mm' in shape and any(v<=0 for v in shape['size_mm']): issue('DIMENSION','Box extents must be positive',[p.id])
                if shape['type']=='mesh' and 'mass_kg' not in p.definition:
                    issue('MESH_MASS','Imported meshes need an explicit mass',[p.id])
            mesh_for_part(p)
        except (ValueError,KeyError,TypeError) as exc: issue('GEOMETRY',str(exc),[p.id])
    for j in assembly.joints:
        a,b=assembly.parts[j['a']['part']],assembly.parts[j['b']['part']]
        kind=joint_kind(j)
        for endpoint,p in ((j['a'],a),(j['b'],b)):
            if 'port' in endpoint:
                key=(p.id,endpoint['port']); occupied.setdefault(key,[]).append(j['id'])
            if 'end' in endpoint:
                key=(p.id,endpoint['end']); member_ends.setdefault(key,[]).append(j['id'])
            if 'at_mm' in endpoint and endpoint['at_mm']>p.length:
                issue('STATION_OUTSIDE_MEMBER','Attachment station exceeds the cut length',[p.id],joint=j['id'])
        pa,pb_,axis=assembly.joint_frames(j)
        error=pb_-pa
        # Free slide coordinates are allowed to offset the two frame origins axially.
        if kind in ('prismatic','cylindrical'): error=error-axis*np.dot(error,axis)
        tolerance=j.get('fit_tolerance_mm',1.1) if j.get('type')=='socket' else 1.1
        if kind!='distance' and np.linalg.norm(error)>tolerance+1e-7:
            issue('UNREACHABLE_JOINT',f'Connection frames miss by {np.linalg.norm(error):.2f} mm',[a.id,b.id],joint=j['id'],gap_mm=float(np.linalg.norm(error)))
        elif j.get('type')=='socket' and np.linalg.norm(error)>1.1:
            issue('ASSEMBLY_FIT_ALLOWANCE',f'Connection uses {np.linalg.norm(error):.2f} mm of its {tolerance:g} mm assembly fit allowance',[a.id,b.id],'warning',joint=j['id'],gap_mm=float(np.linalg.norm(error)))
        if j.get('type')=='socket':
            port=a.ports.get(j['a'].get('port',''),{})
            if port.get('type')!='socket' or b.kind!='member':
                issue('SOCKET_ENDPOINTS','Socket a must be a connector socket and b a member',[a.id,b.id],joint=j['id']); continue
            section=b.section; diameter=section.get('diameter_mm')
            if diameter is None or abs(diameter-port.get('diameter_mm',0))>.6:
                issue('PROFILE_MISMATCH','Member cross-section does not match the socket bore',[a.id,b.id],joint=j['id'])
            _,member_axis=b.frame(j['b'])
            alignment=float(axis@member_axis)
            if (abs(alignment) if port.get('through') else alignment)<.999:
                issue('AXIS_MISMATCH','Member is not aligned with the socket axis',[a.id,b.id],joint=j['id'])
            insertion=j.get('insertion_mm',0)
            if j.get('assembly')=='radial' and port.get('assembly')!='radial':
                issue('CLOSED_SOCKET_RADIAL','A closed socket cannot be installed radially; select an actual split fitting',[a.id,b.id],joint=j['id'])
            if not port.get('through'):
                if 'end' not in j['b']: issue('SOCKET_NEEDS_END','An end socket must terminate a pipe end',[a.id,b.id],joint=j['id'])
                if kind!='fixed' or 'fit_tolerance_mm' in j:
                    mouth,_=a.frame(j['a']); end,_=b.frame(j['b'])
                    insertion=float((mouth-end)@axis)
                if insertion<port.get('min_engagement_mm',0)-.1: issue('ENGAGEMENT_SHORT','Pipe does not reach the required screw engagement',[a.id,b.id],joint=j['id'])
                if insertion>port.get('engagement_mm',0)+.1: issue('SOCKET_BOTTOMED','Insertion exceeds the available socket depth',[a.id,b.id],joint=j['id'])
            else:
                if insertion: issue('THROUGH_INSERTION','Through sockets use at_mm and zero insertion_mm',[a.id,b.id],joint=j['id'])
                mouth,_=a.frame(j['a'])
                station=float((mouth-b.matrix[:3,3])@b.matrix[:3,2]+b.length/2)
                declared_station=j['b'].get('at_mm',0 if j['b'].get('end')=='start' else b.length)
                half=port.get('engagement_mm',0)/2
                if min(station,declared_station)<half or max(station,declared_station)>b.length-half:
                    issue('THROUGH_ENGAGEMENT','Pipe must span the full through socket',[a.id,b.id],joint=j['id'])
            if not j.get('locked',False): issue('LOOSE_SCREW','Socket can slide and rotate because its screw is loose',[a.id,b.id],'warning',joint=j['id'])
        for key,values in j.get('limits',{}).items():
            pairs=values if key=='rotation_deg' else [values]
            if any(v[0]>=v[1] for v in pairs): issue('JOINT_LIMIT','Joint limit lower bound must be smaller than upper bound',[a.id,b.id],joint=j['id'])
        state=assembly.doc.get('state',{}).get('joints',{}).get(j['id'],{})
        for key,coord in state.items():
            limitkey='angle_deg' if key=='twist_deg' else key
            limits=j.get('limits',{}).get(limitkey)
            if limits:
                outside=any(not lo<=v<=hi for v,(lo,hi) in zip(coord,limits)) if key=='rotation_deg' else not limits[0]<=coord<=limits[1]
                if outside: issue('STATE_OUTSIDE_LIMIT','Authored motion coordinate exceeds the joint limits',[a.id,b.id],joint=j['id'])
    for (pid,port),jids in occupied.items():
        capacity=assembly.parts[pid].ports[port].get('capacity',1)
        if len(jids)>capacity: issue('SOCKET_OCCUPIED',f'{pid}/{port} has {len(jids)} connections but capacity {capacity}',[pid],joints=jids)
        for other in assembly.parts[pid].ports[port].get('excludes',[]):
            if (pid,other) in occupied and port<other:
                issue('SHARED_BORE_OCCUPIED',f'{pid}/{port} and {other} use the same bore; choose continuous pipe or two pipe ends',[pid],joints=jids+occupied[pid,other])
    for (pid,end),jids in member_ends.items():
        if len(jids)>1: issue('END_OCCUPIED',f'{pid} {end} is terminated more than once',[pid],joints=jids)
    for load in assembly.doc.get('loads',[]):
        if load['part'] not in assembly.parts: issue('LOAD_REFERENCE','Load references an unknown part',[load['part']])
        elif load.get('at_mm',0)>assembly.parts[load['part']].length: issue('LOAD_STATION','Load station lies outside the member',[load['part']])
    knownj={j['id']:j for j in assembly.joints}
    for drive in assembly.doc.get('drives',[]):
        for key in ('driver','follower'):
            if drive[key] not in knownj: issue('DRIVE_REFERENCE',f"{drive['id']} references unknown joint {drive[key]}")
        if drive.get('type')=='gt2' and abs(drive.get('pitch_mm',2)-2)>1e-8: issue('BELT_PITCH','GT2 drive must use 2 mm pitch')
    components=support(assembly) if not any(i['code'] in ('MASS','GEOMETRY','DIMENSION','LENGTH','WALL') for i in issues) else {'stable':False,'components':[]}
    if not components['stable']: issue('UNSUPPORTED','One or more rigid components lacks a gravity support polygon or world anchor',severity='warning')
    if collisions and not any(i['severity']=='error' for i in issues):
        rigid={pid:i for i,g in enumerate(assembly.rigid_groups()) for pid in g}
        joints_by_pair={}
        for j in assembly.joints: joints_by_pair.setdefault(frozenset([j['a']['part'],j['b']['part']]),[]).append(j)
        with CollisionWorld(assembly) as world:
            for a,b in itertools.combinations(assembly.parts,2):
                contacts=world.contacts(a,b)
                overlaps=[c for c in contacts if c[8]*1000 < -1.]
                if not overlaps: continue
                pair=joints_by_pair.get(frozenset([a,b]),[])
                # Adjacent humanoid capsules overlap at anatomical joint caps by design.
                if pair and assembly.parts[a].kind=='human' and assembly.parts[b].kind=='human': continue
                # Explicit bolt/glue mounting interfaces may overlap only near the declared frame.
                allowed=False
                for j in pair:
                    if j.get('type')=='socket': continue
                    pa,_,_=assembly.joint_frames(j)
                    if all(np.linalg.norm(np.array(c[5])*1000-pa)<12 for c in overlaps): allowed=True
                if allowed: continue
                severity='error' if rigid[a]==rigid[b] else 'warning'
                issue('INTERSECTION',f'Parts penetrate by {max(-c[8]*1000 for c in overlaps):.2f} mm',[a,b],severity,depth_mm=max(-c[8]*1000 for c in overlaps))
    sources=[p.id for p in assembly.parts.values() if p.definition.get('source',{}).get('geometry_status') in ('dimension-derived approximation','generic envelope; supply actual wheel mount dimensions')]
    if sources: issue('APPROXIMATE_GEOMETRY','Some clearances depend on inferred dimensions; inspect source assumptions',sources,'warning')
    result={'valid':not any(i['severity']=='error' for i in issues),'input_sha256':assembly.input_hash,'issues':issues,'rigid_groups':assembly.rigid_groups(),'support':components,'summary':{'parts':len(assembly.parts),'joints':len(assembly.joints),'rigid_bodies':len(assembly.rigid_groups()),'mass_kg':sum(p.mass for p in assembly.parts.values()),'errors':sum(i['severity']=='error' for i in issues),'warnings':sum(i['severity']=='warning' for i in issues)}}
    if build and result['valid']:
        from .planning import plan_build
        result['build']=plan_build(assembly)
        result['valid']=result['valid'] and result['build']['status']=='buildable'
    return result
