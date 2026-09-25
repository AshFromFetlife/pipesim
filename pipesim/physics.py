"""Bullet rigid-body dynamics with native articulated joints and SI-unit conversion.

Locked connections collapse into compound rigid bodies. Free connections become
URDF joints, so Bullet (rather than an animation script) integrates their motion.
Cylindrical sockets have a translation joint followed by a rotation joint.
Spherical joints have three bounded rotational coordinates, with inertialess
intermediate links. Closed loops use Bullet constraints; sliding closures retain
their native joints through welded, inertia-sharing rigid-body representations.
Large articulations are partitioned at rigid links and welded back together,
keeping each Bullet body inside its shared-memory joint-state capacity.
"""
from __future__ import annotations
import copy
import math
import tempfile
import time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pybullet as pb
from scipy.spatial.transform import Rotation
from .document import Assembly, DocumentError, joint_kind, fingerprint
from .math3d import transform, pose_of, point, unit, UnionFind
from .geometry import collision_primitives, mesh_for_part, shape_mesh
from .simulation_control import Progress, SimulationCancelled

# Bullet's shared-memory state has 128 slots, including the base's seven pose
# coordinates. Leave room for the base, even for a fixed-root articulation.
MAX_ARTICULATION_JOINTS = 120


def unsupported_loop(assembly,joint):
    """Explain the unsupported connection without implying a physical failure."""
    from .naming import part_name
    a,b=(part_name(assembly,joint[end]['part']) for end in ('a','b'))
    kind=joint_kind(joint)
    if joint.get('motor') or joint.get('limits'):
        reason=(f'The connection between "{a}" and "{b}" has a motor or travel limits and is part of a closed connection loop. '
                'The simulator cannot arrange this loop while preserving all its motors and limits. '
                'Review the connection settings in Design → Properties. '
                'This rotational loop needs a passive hinge or ball joint without travel limits to close it; '
                'keep any limits or motors that the real mechanism needs.')
    else:
        reason=(f'The {kind} connection between "{a}" and "{b}" is part of a closed connection loop '
                'that the current simulator cannot solve. Review its settings in Design → Properties → Connections.')
    return DocumentError(f'Simulation could not start. {reason} This is a simulator limitation; gravity has not been applied. Connection: {joint["id"]}.')


def _partition_tree(root,tree):
    """Keep each logical joint's axes together; overlap only rigid groups."""
    chunks=[{'root':root,'groups':[root],'tree':[],'size':0}]
    owner={root:0}; overflow={}
    for edge in tree:
        parent,child,j,_=edge
        size={'spherical':3,'cylindrical':2}.get(joint_kind(j),1)
        index=owner[parent]
        if chunks[index]['size']+size>MAX_ARTICULATION_JOINTS:
            index=overflow.get(parent,-1)
            if index<0 or chunks[index]['size']+size>MAX_ARTICULATION_JOINTS:
                index=len(chunks); overflow[parent]=index
                chunks.append({'root':parent,'groups':[parent],'tree':[],'size':0})
        chunk=chunks[index]
        chunk['groups'].append(child); chunk['tree'].append(edge); chunk['size']+=size
        owner[child]=index
    return chunks

def _floats(values): return ' '.join(f'{float(v):.12g}' for v in values)
def _origin(parent,matrix):
    angles=np.deg2rad(pose_of(matrix)['rotation_deg'])
    ET.SubElement(parent,'origin',xyz=_floats(matrix[:3,3]/1000),rpy=_floats(angles))
def _matrix(pos,quat):
    m=np.eye(4); m[:3,:3]=Rotation.from_quat(quat).as_matrix(); m[:3,3]=np.array(pos)*1000
    return m

def _inertia(parts,reference):
    inv=np.linalg.inv(reference)
    masses=np.array([p.mass for p in parts])
    if np.any(masses<=0): raise DocumentError('Dynamic parts need positive mass; use anchors for fixed bodies')
    centers=np.array([point(inv@p.matrix,p.center_of_mass) for p in parts])/1000
    mass=masses.sum(); com=np.average(centers,axis=0,weights=masses)
    inertia=np.zeros((3,3))
    for p,m,center in zip(parts,masses,centers):
        if 'inertia_kg_m2' in p.definition:
            diagonal=np.array(p.definition['inertia_kg_m2'])
            if np.any(diagonal<=0) or max(diagonal)>sum(diagonal)-max(diagonal)+1e-12:
                raise DocumentError(f'{p.id}: principal moments violate rigid-body inertia inequalities')
            local=np.diag(diagonal)
        else:
            mesh=mesh_for_part(p)
            props=mesh.mass_properties
            if props['mass']>1e-8 and np.linalg.eigvalsh(props['inertia']).min()>0:
                local=props['inertia']*(m/props['mass'])*1e-6
            else:
                x,y,z=mesh.extents/1000
                local=np.diag([m*(y*y+z*z)/12,m*(x*x+z*z)/12,m*(x*x+y*y)/12])
        r=(inv@p.matrix)[:3,:3]; offset=center-com
        inertia+=r@local@r.T + m*((offset@offset)*np.eye(3)-np.outer(offset,offset))
    inertia+=np.eye(3)*1e-12
    return float(mass),com,inertia

class World:
    def __init__(self,assembly,dt=1/240,static=False,*,progress=None):
        self.assembly=assembly
        self.progress=progress or Progress()
        self.dt=dt
        self.client=pb.connect(pb.DIRECT)
        self.tmp=tempfile.TemporaryDirectory(prefix='pipesim-physics-')
        self.directory=Path(self.tmp.name)
        self.static=static
        self.part_map={}; self.joint_map={}; self.body_ids=[]; self.constraints=[]
        self.events=[]; self.elapsed=0.; self._mesh_number=0
        self.broken=set(); self.reference_coordinates={}
        try:
            self._configure()
            self._build(assembly)
        except BaseException:
            self.close(); raise

    def _configure(self):
        self.solver_iterations=120; self.substeps=1
        # Refresh contacts at the scale of socket clearance (0.5 mm radial).
        # Bullet's default persistence threshold can retain bore/axle contacts
        # after they separate, producing spurious friction that arrests a swing.
        pb.setPhysicsEngineParameter(fixedTimeStep=self.dt,numSolverIterations=self.solver_iterations,numSubSteps=self.substeps,contactBreakingThreshold=.0001,deterministicOverlappingPairs=1,enableConeFriction=1,enableFileCaching=0,physicsClientId=self.client)
        env=self.assembly.doc.get('environment',{})
        pb.setGravity(*env.get('gravity_m_s2',[0,0,-9.81]),physicsClientId=self.client)
        if env.get('ground',True):
            shape=pb.createCollisionShape(pb.GEOM_PLANE,physicsClientId=self.client)
            visual=pb.createVisualShape(pb.GEOM_BOX,halfExtents=[50,50,.01],visualFramePosition=[0,0,-.011],rgbaColor=[.88,.9,.91,1],physicsClientId=self.client)
            self.ground=pb.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,baseVisualShapeIndex=visual,basePosition=[0,0,env.get('ground_z_mm',0)/1000],physicsClientId=self.client)
            pb.changeDynamics(self.ground,-1,lateralFriction=env.get('friction',.7),physicsClientId=self.client)
        else: self.ground=None

    def _mesh(self,mesh):
        name=f'mesh-{self._mesh_number}.obj'; self._mesh_number+=1
        path=self.directory/name
        mesh=mesh.copy(); mesh.apply_scale(.001); mesh.export(path)
        return str(path).replace('\\','/')

    def _link(self,robot,name,parts,reference,replica=False):
        self.progress.update('building',f'Building rigid body geometry ({self._built_parts}/{len(self.assembly.parts)} parts)',
                             completed_parts=self._built_parts,total_parts=len(self.assembly.parts))
        link=ET.SubElement(robot,'link',name=name)
        mass,com,inertia=_inertia(parts,reference)
        inertial=ET.SubElement(link,'inertial')
        ET.SubElement(inertial,'origin',xyz=_floats(com),rpy='0 0 0')
        ET.SubElement(inertial,'mass',value=str(mass))
        ET.SubElement(inertial,'inertia',ixx=str(inertia[0,0]),iyy=str(inertia[1,1]),izz=str(inertia[2,2]),ixy=str(inertia[0,1]),ixz=str(inertia[0,2]),iyz=str(inertia[1,2]))
        if replica: return com
        inv=np.linalg.inv(reference)
        for p in parts:
            self.progress.check()
            visual=ET.SubElement(link,'visual')
            _origin(visual,inv@p.matrix)
            geom=ET.SubElement(visual,'geometry')
            ET.SubElement(geom,'mesh',filename=self._mesh(mesh_for_part(p)))
            from PIL import ImageColor
            rgb=np.array(ImageColor.getrgb(p.definition.get('color','#8d9ba5')))/255
            mat=ET.SubElement(visual,'material',name=p.id.replace('/','_'))
            ET.SubElement(mat,'color',rgba=_floats([*rgb,1]))
            for s,local in collision_primitives(p):
                collision=ET.SubElement(link,'collision')
                _origin(collision,inv@p.matrix@local)
                geom=ET.SubElement(collision,'geometry')
                kind=s['type']; radius=s.get('radius_mm',s.get('diameter_mm',0)/2)/1000
                if kind=='box': ET.SubElement(geom,'box',size=_floats(np.array(s['size_mm'])/1000))
                elif kind=='sphere': ET.SubElement(geom,'sphere',radius=str(radius))
                elif kind=='cylinder': ET.SubElement(geom,'cylinder',radius=str(radius),length=str(s['length_mm']/1000))
                elif kind=='capsule':
                    # URDF has no standard capsule element; a convex mesh gives the same contact envelope.
                    clean={k:v for k,v in s.items() if k not in ('position_mm','rotation_deg','axis')}
                    ET.SubElement(geom,'mesh',filename=self._mesh(shape_mesh(clean,p.base)))
                elif kind=='mesh':
                    clean={k:v for k,v in s.items() if k not in ('position_mm','rotation_deg','axis')}
                    mesh=shape_mesh(clean,p.base)
                    if s.get('collision')!='static_mesh': mesh=mesh.convex_hull
                    elif not self.static: raise DocumentError(f'{p.id}: concave static_mesh cannot be a dynamic body; provide convex collision_geometry')
                    ET.SubElement(geom,'mesh',filename=self._mesh(mesh))
                else: raise DocumentError(f'Unsupported collision shape {kind}')
            self._built_parts+=1
            self.progress.update('building',f'Building rigid body geometry ({self._built_parts}/{len(self.assembly.parts)} parts)',
                                 completed_parts=self._built_parts,total_parts=len(self.assembly.parts))
        return com

    def _dummy(self,robot,name):
        link=ET.SubElement(robot,'link',name=name)
        inertial=ET.SubElement(link,'inertial')
        ET.SubElement(inertial,'mass',value='0.000001')
        ET.SubElement(inertial,'inertia',ixx='0.000000001',iyy='0.000000001',izz='0.000000001',ixy='0',ixz='0',iyz='0')

    def _build(self,assembly):
        self._built_parts=0
        self.progress.update('building','Preparing rigid bodies and joints')
        self._partition_welds=[]; self._loop_welds=[]; self._body_roots={}
        for anchor in assembly.anchors:
            if set(anchor.get('dofs',['x','y','z','rx','ry','rz'])) != {'x','y','z','rx','ry','rz'}:
                raise DocumentError('Rigid simulation requires fixed world anchors. For a moving world support, anchor a fixture and attach it with a joint; partial DOFs are supported by FEA only.')
        rigid=UnionFind(assembly.parts)
        for j in assembly.joints:
            if joint_kind(j)=='fixed' and not (j.get('break_force_n') or j.get('break_torque_nm')):
                rigid.union(j['a']['part'],j['b']['part'])
        groups=rigid.groups()
        group_for={pid:i for i,group in enumerate(groups) for pid in group}
        fixed={group_for[a['part']] for a in assembly.anchors}
        adjacency={i:[] for i in range(len(groups))}
        edges=[]; loops=[]; sliding_loops=[]
        self._group_original={i:i for i in adjacency}
        for j in assembly.joints:
            ga,gb=group_for[j['a']['part']],group_for[j['b']['part']]
            if ga==gb or joint_kind(j)=='distance' or j['id'] in self.broken: continue
            edges.append(j)
        # Keep anatomical limits and muscle motors in the articulation tree.
        # In a two-hand grip, the passive hand/bar joint can close the loop;
        # a breadth-first choice alone could strand a shoulder in that loop.
        tree_groups=UnionFind(adjacency)
        def can_close(j): return joint_kind(j) in ('revolute','spherical') and not j.get('limits') and not j.get('motor')
        for j in sorted(edges,key=lambda j:2 if can_close(j) else 1 if joint_kind(j) in ('prismatic','cylindrical') else 0):
            ga,gb=group_for[j['a']['part']],group_for[j['b']['part']]
            if tree_groups.find(ga)==tree_groups.find(gb):
                if joint_kind(j) in ('prismatic','cylindrical'):
                    sliding_loops.append(j); continue
                if not can_close(j): raise unsupported_loop(assembly,j)
                loops.append(j); continue
            tree_groups.union(ga,gb)
            adjacency[ga].append((gb,j,False)); adjacency[gb].append((ga,j,True))
        # Cut each sliding loop at a rigid body, retain the native joint, then
        # weld that body's two representations together. Split its inertia and
        # keep collision geometry only once: this adds no physical mass or lock
        # to the mechanism and preserves native slide/twist limits and motors.
        for j in sliding_loops:
            ga,gb=group_for[j['a']['part']],group_for[j['b']['part']]
            reverse=gb in fixed
            parent,original=(gb,ga) if reverse else (ga,gb)
            replica=len(groups);groups.append(groups[original])
            self._group_original[replica]=original
            adjacency[replica]=[(parent,j,not reverse)]
            adjacency[parent].append((replica,j,reverse))
        visited=set()
        for root in sorted(adjacency,key=lambda g:(g not in fixed,g)):
            if root in visited: continue
            frames={root:assembly.parts[groups[root][0]].matrix.copy()}
            robot=ET.Element('robot',name='pipesim')
            coms={root:self._link(robot,f'g{root}',[assembly.parts[k] for k in groups[root]],frames[root])}
            queue=[root]; visited.add(root); tree=[]; joint_specs={}
            for parent in queue:
                for child,j,reversed_edge in adjacency[parent]:
                    if child in visited: continue
                    visited.add(child); queue.append(child); tree.append((parent,child,j,reversed_edge))
                    pa,pb_,axis=assembly.joint_frames(j)
                    child_ref=assembly.parts[groups[child][0]].matrix.copy()
                    child_endpoint=j['a'] if reversed_edge else j['b']
                    pivot=assembly.parts[child_endpoint['part']].frame(child_endpoint)[0]
                    if reversed_edge and j.get('type')=='socket': pivot=pa
                    child_ref[:3,3]=pivot
                    frames[child]=child_ref
                    coms[child]=self._link(robot,f'g{child}',[assembly.parts[k] for k in groups[child]],child_ref,replica=self._group_original[child]!=child)
                    kind=joint_kind(j)
                    if kind=='fixed': axes=[('fixed','fixed',axis)]
                    elif kind=='cylindrical': axes=[('slide','prismatic',axis),('angle','revolute',axis)]
                    elif kind=='prismatic': axes=[('slide','prismatic',axis)]
                    elif kind=='revolute': axes=[('angle','revolute',axis)]
                    elif kind=='spherical':
                        basis=assembly.parts[j['a']['part']].matrix[:3,:3]@Rotation.from_euler('xyz',j['a'].get('frame',{}).get('rotation_deg',[0,0,0]),degrees=True).as_matrix()
                        axes=[(label,'revolute',basis[:,i]) for i,label in enumerate(('rx','ry','rz'))]
                        # Inverting an XYZ rotation requires inverse Z, Y, X.
                        # Anchoring a hand traverses this anatomical chain backwards.
                        if reversed_edge: axes.reverse()
                    else: raise DocumentError(f'Unsupported dynamic joint {kind}')
                    last=f'g{parent}'
                    for i,(coordinate,jtype,world_axis) in enumerate(axes):
                        target=f'g{child}' if i==len(axes)-1 else f'helper_{child}_{i}'
                        if i!=len(axes)-1: self._dummy(robot,target)
                        jname=f'joint_{child}_{i}'
                        joint=ET.SubElement(robot,'joint',name=jname,type=jtype)
                        ET.SubElement(joint,'parent',link=last); ET.SubElement(joint,'child',link=target)
                        _origin(joint,np.linalg.inv(frames[parent])@child_ref if i==0 else np.eye(4))
                        local_axis=child_ref[:3,:3].T@world_axis
                        if reversed_edge: local_axis=-local_axis
                        ET.SubElement(joint,'axis',xyz=_floats(local_axis))
                        limits=j.get('limits',{})
                        if coordinate=='slide': lo,hi=np.array(limits.get('slide_mm',[-1e6,1e6]))/1000
                        elif coordinate.startswith('r'): lo,hi=np.deg2rad(limits.get('rotation_deg',[[-180,180]]*3)[('rx','ry','rz').index(coordinate)])
                        else: lo,hi=np.deg2rad(limits.get('angle_deg',[-1e8,1e8]))
                        offset=self.reference_coordinates.get(j['id'],{}).get(coordinate,0)
                        if lo>=hi: raise DocumentError(f"{j['id']}: lower joint limit must be smaller than upper limit")
                        ET.SubElement(joint,'limit',lower=str(lo-offset),upper=str(hi-offset),effort='1000000',velocity='10000')
                        # URDF viscous damping is integrated explicitly. It can inject
                        # energy into light intermediate links; use an implicit
                        # velocity constraint with a bounded damping impulse below.
                        ET.SubElement(joint,'dynamics',damping='0',friction='0')
                        joint_specs[jname]=(j,coordinate)
                        last=target
            self._load_articulation(robot,root,tree,groups,frames,fixed,joint_specs)
            for _,_,j,_ in tree:
                aa=self.part_map[j['a']['part']]; bb=self.part_map[j['b']['part']]
                if j.get('type')!='socket' and joint_kind(j)!='cylindrical':
                    pb.setCollisionFilterPair(aa[0],bb[0],aa[1],bb[1],0,physicsClientId=self.client)
        if loops or self._partition_welds or self._loop_welds:
            # Long muscle-driven loops need more constraint iterations than a
            # free articulation; otherwise solver error looks like weak muscles.
            # Substeps also resolve the impact when passive elbows reach their
            # extension stops, avoiding energy injection at the limit.
            self.solver_iterations=1000; self.substeps=max(1,math.ceil(self.dt*480-1e-9))
            pb.setPhysicsEngineParameter(numSolverIterations=self.solver_iterations,numSubSteps=self.substeps,physicsClientId=self.client)
        for j in loops: self._loop(j)
        # Distance-only links remain separate bodies and exchange equal/opposite forces.

    def _load_articulation(self,robot,root,tree,groups,frames,fixed,joint_specs):
        chunks=_partition_tree(root,tree)
        original=self._group_original
        copies={original[g]:sum(original[h]==original[g] for c in chunks for h in c['groups']) for c in chunks for g in c['groups']}
        links={link.get('name'):link for link in robot.findall('link')}
        joints={joint.get('name'):joint for joint in robot.findall('joint')}
        owned={}
        for chunk in chunks:
            self.progress.update('loading',f'Loading physics articulation {len(self.body_ids)+1}',force=True)
            chunk_root=chunk['root']
            names={f'g{g}' for g in chunk['groups']}; joint_names=[]
            for _,child,_,_ in chunk['tree']:
                for i in range(3):
                    name=f'joint_{child}_{i}'
                    if name not in joints: break
                    joint_names.append(name)
                    names.add(joints[name].find('child').get('link'))
            section=ET.Element('robot',name='pipesim')
            for name in sorted(names):
                link=copy.deepcopy(links[name])
                if name.startswith('g'):
                    group=int(name[1:]); share=copies[original[group]]
                    # A seam has two representations of ONE rigid link. Share
                    # its inertia, and keep its geometry on its original owner.
                    inertial=link.find('inertial')
                    mass=inertial.find('mass'); mass.set('value',str(float(mass.get('value'))/share))
                    inertia=inertial.find('inertia')
                    for key,value in list(inertia.attrib.items()): inertia.set(key,str(float(value)/share))
                    if group in owned:
                        for shape in link.findall('visual')+link.findall('collision'): link.remove(shape)
                section.append(link)
            for name in joint_names: section.append(copy.deepcopy(joints[name]))
            path=self.directory/f'body-{len(self.body_ids)}.urdf'
            ET.indent(section)
            ET.ElementTree(section).write(path,encoding='utf-8',xml_declaration=True)
            ref=frames[chunk_root]
            body=pb.loadURDF(str(path),ref[:3,3]/1000,Rotation.from_matrix(ref[:3,:3]).as_quat(),useFixedBase=self.static or chunk_root in fixed,flags=pb.URDF_USE_INERTIA_FROM_FILE|pb.URDF_USE_SELF_COLLISION|pb.URDF_USE_IMPLICIT_CYLINDER,physicsClientId=self.client)
            self.body_ids.append(body); self._body_roots[body]=groups[chunk_root]
            link_for={chunk_root:-1}
            for index in range(pb.getNumJoints(body,physicsClientId=self.client)):
                info=pb.getJointInfo(body,index,physicsClientId=self.client)
                linkname=info[12].decode(); jname=info[1].decode()
                if linkname.startswith('g'): link_for[int(linkname[1:])]=index
                j,coordinate=joint_specs[jname]
                self.joint_map.setdefault(j['id'],{})[coordinate]=(body,index)
                if info[2]!=pb.JOINT_FIXED:
                    pb.setJointMotorControl2(body,index,pb.VELOCITY_CONTROL,force=0,physicsClientId=self.client)
                pb.enableJointForceTorqueSensor(body,index,True,physicsClientId=self.client)
            for group in chunk['groups']:
                link=link_for[group]
                if group in owned:
                    parent,parent_link=owned[group]
                    self._weld_partition(parent,parent_link,body,frames[group])
                else:
                    owned[group]=(body,link)
                    if original[group]==group:
                        for pid in groups[group]:
                            self.part_map[pid]=(body,link,np.linalg.inv(frames[group])@self.assembly.parts[pid].matrix)
                friction=np.mean([self.assembly.parts[pid].definition.get('friction',.6) for pid in groups[group]])
                restitution=max(self.assembly.parts[pid].definition.get('restitution',.02) for pid in groups[group])
                pb.changeDynamics(body,link,lateralFriction=float(friction),restitution=restitution,linearDamping=.015,angularDamping=.015,physicsClientId=self.client)
                if group in fixed and group!=chunk_root:
                    pos,quat=self.link_pose(body,link)
                    self.constraints.append(pb.createConstraint(body,link,-1,-1,pb.JOINT_FIXED,[0,0,0],[0,0,0],pos,parentFrameOrientation=[0,0,0,1],childFrameOrientation=quat,physicsClientId=self.client))
        for group,(body,link) in owned.items():
            if original[group]==group: continue
            parent,parent_link=owned[original[group]]
            frame=frames[original[group]]
            a=np.linalg.inv(_matrix(*self.link_pose(parent,parent_link)))@frame
            b=np.linalg.inv(_matrix(*self.link_pose(body,link)))@frame
            constraint=pb.createConstraint(parent,parent_link,body,link,pb.JOINT_FIXED,[0,0,0],a[:3,3]/1000,b[:3,3]/1000,
                parentFrameOrientation=Rotation.from_matrix(a[:3,:3]).as_quat(),childFrameOrientation=Rotation.from_matrix(b[:3,:3]).as_quat(),physicsClientId=self.client)
            pb.changeConstraint(constraint,maxForce=1e9,physicsClientId=self.client)
            self.constraints.append(constraint);self._loop_welds.append(constraint)

    def _weld_partition(self,parent,link,child,frame):
        a=np.linalg.inv(_matrix(*self.link_pose(parent,link)))@frame
        b=np.linalg.inv(_matrix(*self.link_pose(child,-1)))@frame
        constraint=pb.createConstraint(parent,link,child,-1,pb.JOINT_FIXED,[0,0,0],a[:3,3]/1000,b[:3,3]/1000,
            parentFrameOrientation=Rotation.from_matrix(a[:3,:3]).as_quat(),childFrameOrientation=Rotation.from_matrix(b[:3,:3]).as_quat(),physicsClientId=self.client)
        pb.changeConstraint(constraint,maxForce=1e9,physicsClientId=self.client)
        self.constraints.append(constraint); self._partition_welds.append((parent,link,child))

    def _loop(self,j):
        kind=joint_kind(j)
        if kind not in ('revolute','spherical') or j.get('limits') or j.get('motor'):
            raise unsupported_loop(self.assembly,j)
        pa,_,axis=self.assembly.joint_frames(j)
        a=self.part_map[j['a']['part']]; b=self.part_map[j['b']['part']]
        for pivot in ([pa,pa+axis*100] if kind=='revolute' else [pa]):
            # Bullet user-constraint pivots are in COM frames, unlike URDF
            # attachment frames. This matters for asymmetric fittings and loads.
            ma=_matrix(*self.link_pose(a[0],a[1])); mb=_matrix(*self.link_pose(b[0],b[1]))
            la=point(np.linalg.inv(ma),pivot)/1000; lb=point(np.linalg.inv(mb),pivot)/1000
            constraint=pb.createConstraint(a[0],a[1],b[0],b[1],pb.JOINT_POINT2POINT,[0,0,0],la,lb,physicsClientId=self.client)
            pb.changeConstraint(constraint,maxForce=j.get('max_force_n',1000000),physicsClientId=self.client)
            self.constraints.append(constraint)
        pb.setCollisionFilterPair(a[0],b[0],a[1],b[1],0,physicsClientId=self.client)

    def link_pose(self,body,link):
        if link<0:
            return pb.getBasePositionAndOrientation(body,physicsClientId=self.client)
        s=pb.getLinkState(body,link,computeForwardKinematics=True,physicsClientId=self.client)
        return s[0],s[1]

    def link_matrix(self,body,link):
        if link>=0:
            s=pb.getLinkState(body,link,computeForwardKinematics=True,physicsClientId=self.client)
            return _matrix(s[4],s[5])
        pos,quat=pb.getBasePositionAndOrientation(body,physicsClientId=self.client)
        dyn=pb.getDynamicsInfo(body,-1,physicsClientId=self.client)
        return _matrix(pos,quat)@np.linalg.inv(_matrix(dyn[3],dyn[4]))

    def part_matrix(self,pid):
        body,link,local=self.part_map[pid]
        return self.link_matrix(body,link)@local

    def part_velocity(self,pid):
        body,link,_=self.part_map[pid]
        if link<0: return pb.getBaseVelocity(body,physicsClientId=self.client)
        s=pb.getLinkState(body,link,computeLinkVelocity=True,physicsClientId=self.client)
        return s[6],s[7]

    def _joint_samples(self):
        indices={}
        for mapping in self.joint_map.values():
            for body,index in mapping.values(): indices.setdefault(body,[]).append(index)
        return {(body,index):state for body,links in indices.items()
                for index,state in zip(links,pb.getJointStates(body,links,physicsClientId=self.client))}

    def joint_state(self,jid,coordinate=None,samples=None):
        mapping=self.joint_map.get(jid,{})
        if not mapping: return (0.,0.,[0]*6,0.)
        coordinate=coordinate or ('slide' if 'slide' in mapping else 'angle' if 'angle' in mapping else next(iter(mapping)))
        body,index=mapping[coordinate]
        state=list(samples[(body,index)] if samples is not None else pb.getJointState(body,index,physicsClientId=self.client))
        state[0]+=self.reference_coordinates.get(jid,{}).get(coordinate,0)
        return state

    def set_coordinates(self,coordinates):
        for jid,values in coordinates.items():
            if jid not in self.joint_map: continue
            converted={'slide':values.get('slide_mm',0)/1000,'angle':math.radians(values.get('angle_deg',values.get('twist_deg',0)))}
            converted.update({k:math.radians(v) for k,v in zip(('rx','ry','rz'),values.get('rotation_deg',[0,0,0]))})
            for coordinate,(body,index) in self.joint_map[jid].items():
                if coordinate!='fixed': pb.resetJointState(body,index,converted.get(coordinate,0),physicsClientId=self.client)
        # Initial coordinates move downstream section bases as well as their
        # native axes, before Bullet takes the first step or records a frame.
        for parent,link,child in self._partition_welds:
            dyn=pb.getDynamicsInfo(child,-1,physicsClientId=self.client)
            com=self.link_matrix(parent,link)@_matrix(dyn[3],dyn[4])
            pb.resetBasePositionAndOrientation(child,com[:3,3]/1000,Rotation.from_matrix(com[:3,:3]).as_quat(),physicsClientId=self.client)

    def _detach(self,ids):
        """Rebuild changed topology at the current pose, preserving momentum and coordinates."""
        updated=copy.deepcopy(self.assembly)
        velocities={}
        for pid,p in updated.parts.items():
            matrix=self.part_matrix(pid); p.matrix=matrix
            body,link,_=self.part_map[pid]
            linear,angular=self.part_velocity(pid); center,_=self.link_pose(body,link)
            com=point(matrix,p.center_of_mass)/1000
            linear=np.array(linear)+np.cross(angular,com-np.array(center))
            velocities[pid]=(linear,np.array(angular))
        saved={}
        for jid,mapping in self.joint_map.items():
            if jid in ids: continue
            for coordinate in mapping:
                q,v,*_=self.joint_state(jid,coordinate)
                saved.setdefault(jid,{})[coordinate]=(q,v)
                self.reference_coordinates.setdefault(jid,{})[coordinate]=q
        self.broken.update(ids)
        updated.joints=[j for j in updated.joints if j['id'] not in self.broken]
        updated.doc['drives']=[d for d in updated.doc.get('drives',[]) if d['driver'] not in self.broken and d['follower'] not in self.broken]
        self.assembly=updated
        pb.resetSimulation(physicsClientId=self.client)
        self.part_map={}; self.joint_map={}; self.body_ids=[]; self.constraints=[]
        self._configure(); self._build(updated)
        for body in self.body_ids:
            roots=self._body_roots[body]
            masses=[updated.parts[pid].mass for pid in roots]
            if roots and pb.getDynamicsInfo(body,-1,physicsClientId=self.client)[0]>0:
                linear=np.average([velocities[pid][0] for pid in roots],axis=0,weights=masses)
                angular=np.average([velocities[pid][1] for pid in roots],axis=0,weights=masses)
                pb.resetBaseVelocity(body,linear,angular,physicsClientId=self.client)
        for jid,mapping in self.joint_map.items():
            for coordinate,(body,index) in mapping.items():
                velocity=saved.get(jid,{}).get(coordinate,(0,0))[1]
                if coordinate!='fixed': pb.resetJointState(body,index,0,velocity,physicsClientId=self.client)

    def _break_events(self):
        detach=[]
        for j in self.assembly.joints:
            if j['id'] in self.broken: continue
            mapping=self.joint_map.get(j['id'],{})
            if j.get('type')=='socket' and joint_kind(j)!='fixed':
                connector=self.assembly.parts[j['a']['part']]; member=self.assembly.parts[j['b']['part']]
                port=connector.ports[j['a']['port']]
                socket_matrix=self.part_matrix(connector.id); member_matrix=self.part_matrix(member.id)
                mouth=point(socket_matrix,port['position_mm']); axis=socket_matrix[:3,:3]@unit(port['axis'])
                endpoints=[float((point(member_matrix,[0,0,z])-mouth)@axis) for z in (-member.length/2,member.length/2)]
                depth=port.get('engagement_mm',30)
                lo,hi=(-depth/2,depth/2) if port.get('through') else (-depth,0)
                if min(endpoints)>hi+1 or max(endpoints)<lo-1:
                    detach.append(j['id']); self.events.append({'time_s':self.elapsed,'type':'socket_disengaged','joint':j['id'],'continued_as':'detached rigid body'})
            if mapping and (j.get('break_force_n') or j.get('break_torque_nm')):
                reactions=[self.joint_state(j['id'],c) for c in mapping]
                force=max(np.linalg.norm(s[2][:3]) for s in reactions)
                torque=max(np.linalg.norm(s[2][3:])+abs(s[3]) for s in reactions)
                if force>j.get('break_force_n',math.inf) or torque>j.get('break_torque_nm',math.inf):
                    detach.append(j['id']); self.events.append({'time_s':self.elapsed,'type':'joint_break','joint':j['id'],'force_n':float(force),'torque_nm':float(torque)})
        if detach: self._detach(set(detach))

    def _target(self,motor,t):
        schedule=motor.get('schedule',[])
        if not schedule: return motor.get('target',0)
        return float(np.interp(t,[k['time_s'] for k in schedule],[k['target'] for k in schedule]))

    def _force(self,pid,force,world_point):
        body,link,_=self.part_map[pid]
        pb.applyExternalForce(body,link,force,world_point,pb.WORLD_FRAME,physicsClientId=self.client)

    def _joint_effort(self,key,effort):
        body,index=key
        info=pb.getJointInfo(body,index,physicsClientId=self.client)
        matrix=self.link_matrix(body,index)
        axis=matrix[:3,:3]@np.array(info[13]); parent=info[16]
        if info[2]==pb.JOINT_PRISMATIC:
            pivot=matrix[:3,3]/1000
            pb.applyExternalForce(body,index,axis*effort,pivot,pb.WORLD_FRAME,physicsClientId=self.client)
            pb.applyExternalForce(body,parent,-axis*effort,pivot,pb.WORLD_FRAME,physicsClientId=self.client)
        else:
            pb.applyExternalTorque(body,index,axis*effort,pb.WORLD_FRAME,physicsClientId=self.client)
            pb.applyExternalTorque(body,parent,-axis*effort,pb.WORLD_FRAME,physicsClientId=self.client)

    def step(self):
        torques={key:0. for mapping in self.joint_map.values() for key in mapping.values()}
        # A scalar query transfers a body's entire state. Batch both reads and
        # passive motor updates so long chains do not repeat that per axis.
        samples=self._joint_samples(); passive={}
        for j in self.assembly.joints:
            if j['id'] not in self.joint_map: continue
            for coordinate,(body,index) in self.joint_map[j['id']].items():
                if coordinate=='fixed': continue
                _,velocity,*_=self.joint_state(j['id'],coordinate,samples)
                resistance=j.get('friction',0)+j.get('damping',.03)*abs(velocity)
                passive.setdefault(body,[]).append((index,resistance))
        for body,values in passive.items():
            pb.setJointMotorControlArray(body,[v[0] for v in values],pb.VELOCITY_CONTROL,
                targetVelocities=[0.]*len(values),forces=[v[1] for v in values],physicsClientId=self.client)
        for j in self.assembly.joints:
            if j['id'] not in self.joint_map: continue
            motor=j.get('motor')
            if motor:
                if 'rotation_deg' in motor:
                    for coordinate,target in zip(('rx','ry','rz'),motor['rotation_deg']):
                        if coordinate not in self.joint_map[j['id']]: continue
                        body,index=self.joint_map[j['id']][coordinate]
                        offset=self.reference_coordinates.get(j['id'],{}).get(coordinate,0)
                        pb.setJointMotorControl2(body,index,pb.POSITION_CONTROL,targetPosition=math.radians(target)-offset,force=motor.get('max_torque_nm',2),positionGain=min(1,motor.get('kp',20)*self.dt/self.substeps),velocityGain=min(1,motor.get('kd',1)),physicsClientId=self.client)
                    continue
                coordinate='slide' if 'slide' in self.joint_map[j['id']] else 'angle'
                if coordinate not in self.joint_map[j['id']]: continue
                q,v,_,_=self.joint_state(j['id'],coordinate,samples)
                target=self._target(motor,self.elapsed)
                target=target/1000 if coordinate=='slide' else math.radians(target)
                limit=motor.get('max_force_n',100) if coordinate=='slide' else motor.get('max_torque_nm',2)
                body,index=self.joint_map[j['id']][coordinate]
                if motor.get('mode')=='velocity':
                    pb.setJointMotorControl2(body,index,pb.VELOCITY_CONTROL,targetVelocity=target,force=limit,physicsClientId=self.client)
                else:
                    # Native constraint motors remain stable for tiny pulley inertia;
                    # explicit high-gain torque servos chatter at practical timesteps.
                    offset=self.reference_coordinates.get(j['id'],{}).get(coordinate,0)
                    pb.setJointMotorControl2(body,index,pb.POSITION_CONTROL,targetPosition=target-offset,force=limit,positionGain=min(1,motor.get('kp',20)*self.dt/self.substeps),velocityGain=min(1,motor.get('kd',1)),physicsClientId=self.client)
        for drive in self.assembly.doc.get('drives',[]):
            driver,follower=drive['driver'],drive['follower']
            if driver not in self.joint_map or follower not in self.joint_map: raise DocumentError(f"{drive['id']}: drive joints are missing or locked")
            if 'angle' not in self.joint_map[driver]: raise DocumentError('Drive input must be a revolute joint')
            output='slide' if 'slide' in self.joint_map[follower] else 'angle'
            q,w,*_=self.joint_state(driver,'angle',samples); x,v,*_=self.joint_state(follower,output,samples)
            ratio=drive.get('ratio',1)
            if drive['type'] in ('gt2','rack') and output=='slide': ratio=drive.get('pitch_mm',2)*drive.get('teeth',20)/(2*math.pi*1000)
            ratio*=drive.get('sign',1)
            offset=drive.get('offset_mm',0)/1000 if output=='slide' else math.radians(drive.get('offset_deg',0))
            effort=drive.get('stiffness_n_m',10000)*(ratio*q+offset-x)+drive.get('damping_ns_m',50)*(ratio*w-v)
            effort=float(np.clip(effort,-drive.get('max_force_n',200),drive.get('max_force_n',200)))
            torques[self.joint_map[follower][output]]+=effort
            torques[self.joint_map[driver]['angle']]-=effort*ratio/drive.get('efficiency',1)
        for key,effort in torques.items():
            if effort: self._joint_effort(key,effort)
        for load in self.assembly.doc.get('loads',[]):
            if not load.get('start_s',0)<=self.elapsed<=load.get('end_s',math.inf): continue
            pid=load['part']; p=self.assembly.parts[pid]; matrix=self.part_matrix(pid)
            local=load.get('point_mm',[0,0,load.get('at_mm',p.length/2)-p.length/2])
            location=point(matrix,local)/1000
            self._force(pid,load.get('force_n',[0,0,0]),location)
            body,link,_=self.part_map[pid]
            pb.applyExternalTorque(body,link,load.get('moment_nm',[0,0,0]),pb.WORLD_FRAME,physicsClientId=self.client)
        for j in self.assembly.joints:
            if joint_kind(j)!='distance' or j['id'] in self.broken: continue
            a,b=j['a']['part'],j['b']['part']
            la,_=self.assembly.parts[a].local_frame(j['a']); lb,_=self.assembly.parts[b].local_frame(j['b'])
            pa=point(self.part_matrix(a),la)/1000; pb_=point(self.part_matrix(b),lb)/1000
            delta=pb_-pa; length=np.linalg.norm(delta)
            if length<1e-8: continue
            axis=delta/length; rest=j.get('rest_length_mm',0)/1000
            speed=np.dot(np.array(self.part_velocity(b)[0])-np.array(self.part_velocity(a)[0]),axis)
            force=20000*(length-rest)+j.get('damping',20)*speed
            if j.get('tension_only',False): force=max(0,force)
            limit=j.get('break_force_n')
            if limit and abs(force)>limit:
                self.broken.add(j['id']); self.events.append({'time_s':self.elapsed,'type':'joint_break','joint':j['id'],'force_n':float(abs(force))}); continue
            self._force(a,axis*force,pa); self._force(b,-axis*force,pb_)
        pb.stepSimulation(physicsClientId=self.client)
        self.elapsed+=self.dt
        self._break_events()

    def snapshot(self):
        state={}
        reactions={}
        samples=self._joint_samples()
        motor_efforts={}; driven={j['id'] for j in self.assembly.joints if j.get('motor')}
        for jid,mapping in self.joint_map.items():
            values={}
            for coordinate in mapping:
                q,v,forces,effort=self.joint_state(jid,coordinate,samples)
                if jid in driven:
                    motor_efforts.setdefault(jid,{})[coordinate]={'force_n' if coordinate=='slide' else 'torque_nm':effort}
                if coordinate=='slide': values['slide_mm']=q*1000
                elif coordinate=='angle': values['angle_deg']=math.degrees(q)
                elif coordinate in ('rx','ry','rz'): values.setdefault('rotation_deg',[0,0,0])[('rx','ry','rz').index(coordinate)]=math.degrees(q)
                reactions[jid]={'force_n':list(forces[:3]),'moment_nm':list(forces[3:])}
            state[jid]=values
        contacts=[]
        bylink={}
        for pid,(body,link,_) in self.part_map.items(): bylink.setdefault((body,link),[]).append(pid)
        for c in pb.getContactPoints(physicsClientId=self.client):
            if c[9]<=0: continue
            force=np.array(c[7])*c[9]+np.array(c[11])*c[10]+np.array(c[13])*c[12]
            contacts.append({'a':bylink.get((c[1],c[3]),['world']),'b':bylink.get((c[2],c[4]),['world']),'position_a_mm':(np.array(c[5])*1000).tolist(),'position_b_mm':(np.array(c[6])*1000).tolist(),'force_on_a_n':force.tolist()})
        return {'time_s':round(self.elapsed,8),'parts':{pid:pose_of(self.part_matrix(pid)) for pid in self.part_map},'joints':state,'reactions':reactions,'motor_efforts':motor_efforts,'contacts':contacts,'broken_joints':sorted(self.broken)}

    def close(self):
        if pb.isConnected(self.client): pb.disconnect(self.client)
        self.tmp.cleanup()
    def __enter__(self): return self
    def __exit__(self,*args): self.close()

def validate_simulation_options(duration=3,fps=30,dt=1/240,chain_links_per_body=1):
    if not all(math.isfinite(v) for v in (duration,fps,dt)) or duration<=0 or duration>3600 or fps<=0 or fps>240 or dt<=0 or dt>1/60:
        raise ValueError('Require 0 < duration ≤ 3600 s, 0 < fps ≤ 240 and 0 < dt ≤ 1/60 s')

    if isinstance(chain_links_per_body,bool) or not isinstance(chain_links_per_body,int) or not 1<=chain_links_per_body<=1000:
        raise ValueError('chain_links_per_body must be an integer from 1 to 1000')


def simplify_chains(assembly,links_per_body):
    """Freeze runs of passive chain joints at the authored pose, without editing it.

    Attachments and anchors stay on individual links. Explicit motors, drives,
    break thresholds and non-chain joints remain physical joints.
    """
    result=copy.copy(assembly); result.doc=copy.deepcopy(assembly.doc)
    result.joints=copy.deepcopy(assembly.joints)
    candidates=[j for j in result.joints if j.get('metadata',{}).get('chain_link')
                and joint_kind(j)=='spherical'
                and all(assembly.parts[j[e]['part']].kind=='chain' for e in ('a','b'))]
    ids={j['id'] for j in candidates}
    protected={a['part'] for a in assembly.anchors}
    for j in assembly.joints:
        if j['id'] not in ids:
            protected.update(j[e]['part'] for e in ('a','b'))
    driven={d[k] for d in assembly.doc.get('drives',[]) for k in ('driver','follower')}
    degree={}
    for j in candidates:
        for e in ('a','b'):
            pid=j[e]['part'];degree[pid]=degree.get(pid,0)+1
    protected.update(pid for pid,n in degree.items() if n>2)
    groups=UnionFind(assembly.parts); sizes={pid:1 for pid in assembly.parts}
    frozen=[]
    for j in candidates:
        a,b=j['a']['part'],j['b']['part']
        if {a,b}&protected or j.get('motor') or j.get('break_force_n') or j.get('break_torque_nm') or j['id'] in driven: continue
        ga,gb=groups.find(a),groups.find(b)
        if ga==gb or sizes[ga]+sizes[gb]>links_per_body: continue
        size=sizes[ga]+sizes[gb];groups.union(a,b);sizes[groups.find(a)]=size
        j['type']='fixed';j.pop('limits',None);frozen.append(j['id'])
    return result,frozen


def simulate(assembly,duration=3,fps=30,dt=1/240,*,chain_links_per_body=1,progress=None,cancelled=None):
    """Record dynamics; progress receives dictionaries, cancelled is a predicate.

    Use console_progress for flushed Python stdout. Cancellation raises
    SimulationCancelled and releases the Bullet client and temporary files.
    """
    validate_simulation_options(duration,fps,dt,chain_links_per_body)
    reporter=Progress(progress,cancelled)
    reporter.update('preparing','Preparing simulation',force=True)
    doc=copy.deepcopy(assembly.doc); initial=doc.pop('state',{}).get('joints',{})
    frozen=[]
    neutral=Assembly.from_doc(doc,assembly.base,assembly.library)
    if chain_links_per_body>1:
        simplified,frozen=simplify_chains(neutral,chain_links_per_body)
        # Bake only the frozen coordinates into the compound geometry. The
        # remaining joints retain their original axes, limits and motor targets.
        neutral.apply_coordinates({jid:values for jid,values in initial.items() if jid in frozen})
        neutral=simplified
    with World(neutral,dt,progress=reporter) as world:
        world.set_coordinates(initial)
        def snapshot():
            reporter.check()
            frame=world.snapshot()
            for jid in frozen: frame['joints'][jid]=copy.deepcopy(initial.get(jid,{'rotation_deg':[0,0,0]}))
            return frame
        reporter.update('recording','Recording initial pose',force=True)
        frames=[snapshot()]; next_sample=1/fps
        maximum_speed=0.; warnings=[]
        steps=math.ceil(duration/dt); started=time.monotonic()
        reporter.update('simulating',f'Integrating physics: {len(world.joint_map)} joints, {world.solver_iterations} solver iterations',
                        force=True,simulated_s=0,duration_s=duration,percent=0,completed_steps=0,total_steps=steps)
        for step in range(steps):
            reporter.check()
            world.step()
            if world.elapsed+1e-9>=next_sample or step==steps-1:
                frame=snapshot()
                if not all(np.isfinite(p['position_mm']).all() for p in frame['parts'].values()):
                    raise RuntimeError('Physics solver diverged; reduce timestep and inspect initial intersections')
                frames.append(frame); next_sample+=1/fps
            elapsed=time.monotonic()-started
            reporter.update('simulating','Integrating physics',simulated_s=min(world.elapsed,duration),duration_s=duration,
                            percent=100*(step+1)/steps,completed_steps=step+1,total_steps=steps,
                            eta_s=elapsed*(steps-step-1)/(step+1))
        reporter.update('finalizing','Finalizing recording',force=True)
        speeds=[np.linalg.norm(world.part_velocity(pid)[0]) for pid in world.part_map]
        spins=[np.linalg.norm(world.part_velocity(pid)[1]) for pid in world.part_map]
        maximum_speed=max(speeds,default=0)
        for pid,p in neutral.parts.items():
            if p.kind=='human': continue
            original=p.matrix[:3,2]; final=world.part_matrix(pid)[:3,2]
            angle=math.degrees(math.acos(float(np.clip(original@final,-1,1))))
            if angle>20: warnings.append({'type':'tipped_or_rotated','part':pid,'rotation_deg':angle})
        result={'engine':'PyBullet articulated rigid bodies','input_sha256':assembly.input_hash,'duration_s':world.elapsed,'dt_s':dt,'solver_iterations':world.solver_iterations,'substeps':world.substeps,'fps':fps,'settled':bool(maximum_speed<.02 and max(spins,default=0)<.05),'final_max_speed_m_s':maximum_speed,'final_max_angular_speed_rad_s':max(spins,default=0),'frames':frames,'events':world.events+warnings,'chain_simplification':{'links_per_body':chain_links_per_body,'frozen_joints':frozen},'limitations':['Rigid materials; deformation is computed separately by frame FEA','Spherical joints use bounded XYZ rotational coordinates; Euler singularities and transient solver limit errors are possible','Topology changes preserve current poses and velocities; angular axes rebase at the break pose','Unknown strength data is not assigned a fracture threshold']}
        if frozen: result['limitations'].append('Chain simplification freezes internal joints at the starting pose: bending, contact attribution and joint reactions are approximate; frozen joints have no reaction measurement')
        reporter.update('complete','Simulation complete',force=True,simulated_s=duration,duration_s=duration,percent=100)
        return result

def rigid_load_trials(assembly,part,directions,maximum_force_n,steps=12,dwell_s=.5):
    """Independent increasing load cases detect tipping, translation and joint failure."""
    if not 0<dwell_s<=30: raise DocumentError('Rigid trial dwell must be between 0 and 30 seconds')
    trials=[]
    for direction in directions:
        previous=0.; first=None
        for magnitude in np.linspace(maximum_force_n/steps,maximum_force_n,steps):
            doc=copy.deepcopy(assembly.doc)
            target=assembly.parts[part]
            doc.setdefault('loads',[]).append({'part':part,'force_n':(np.array(direction)*magnitude).tolist(),'point_mm':[0,0,target.length/2 if target.kind=='member' else 0]})
            recording=simulate(Assembly.from_doc(doc,assembly.base),dwell_s,fps=min(30,1/dwell_s*5))
            final=recording['frames'][-1]
            movement=[{'part':pid,'translation_mm':float(np.linalg.norm(np.array(final['parts'][pid]['position_mm'])-p.matrix[:3,3]))} for pid,p in assembly.parts.items()]
            movement=[m for m in movement if m['translation_mm']>50]
            events=recording['events']
            if movement or events:
                first={'force_n':float(magnitude),'previous_no_event_force_n':float(previous),'events':events,'movement_over_50_mm':movement,'final_frame':final}; break
            previous=float(magnitude)
        trials.append({'direction':np.array(direction).tolist(),'first_event':first,'last_tested_force_n':float(magnitude)})
    return {'dwell_s':dwell_s,'trials':trials,'limits':{'translation_mm':50,'rotation_deg':20},'limitations':['Each magnitude starts from the authored pose and runs for the stated dwell; absence of an event is not a long-term stability proof','Intended moving subassemblies can trigger displacement events','Explicit joint break thresholds are tested; unknown material fracture remains unmodelled']}
