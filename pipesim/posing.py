"""Project editor transform targets onto an articulated mechanism's joint motion.

Forward kinematics preserves every tree connection exactly. Loop closures and
additional world anchors are solved together, then independently checked before
any document is returned. Targets are soft: a handle stops at its travel limit.
"""
import copy

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .document import Assembly, DocumentError, joint_kind
from .editing import expand_objects, snapshot_design
from .math3d import pose_of, transform
from .snapping import _movement_coordinates, move_document


def _editable(assembly, affected):
    """Bake only manipulated reusable objects, as part of the eventual undo edit."""
    doc=copy.deepcopy(assembly.doc)
    if doc.get('state',{}).get('joints'):
        animation=doc.get('animation');coordinates=doc['state']['joints']
        doc=snapshot_design(assembly,{'time_s':0,'parts':{p:pose_of(v.matrix) for p,v in assembly.parts.items()},
                                     'joints':coordinates})
        if animation:
            for track in animation.get('tracks',[]):
                shift=coordinates.get(track['joint'],{}).get(track['coordinate'],0)
                for key in track['keyframes']: key['value']-=shift
            doc['animation']=animation
        return Assembly.from_doc(doc,assembly.base,assembly.library)
    instances=[o for o in doc.get('objects',[]) if any(p.startswith(o['id']+'/') for p in affected)]
    if not instances: return assembly
    editable=assembly
    for instance in instances:
        editable=Assembly.from_doc(expand_objects(editable,instance['id']),assembly.base,assembly.library)
    return editable


class Mechanism:
    def __init__(self, assembly, selected):
        self.assembly=assembly;self.groups=assembly.editor_groups()
        self.group_of={p:i for i,g in enumerate(self.groups) for p in g};self.selected=self.group_of[selected]
        edges={i:[] for i in range(len(self.groups))}
        for j in assembly.joints:
            a,b=(self.group_of[j[e]['part']] for e in ('a','b'))
            if a!=b: edges[a].append((b,j));edges[b].append((a,j))
        component={self.selected};queue=[self.selected]
        for g in queue:
            for other,_ in edges[g]:
                if other not in component: component.add(other);queue.append(other)
        self.component=component;self.parts={p for g in component for p in self.groups[g]}
        anchored={self.group_of[a['part']] for a in assembly.anchors}&component
        pelvis=[g for g in component if any(assembly.parts[p].kind=='human' and p.split('/')[-1]=='pelvis' for p in self.groups[g])]
        self.root=min(anchored) if anchored else pelvis[0] if pelvis else max(component,key=lambda g:(len(self.groups[g]),sum(assembly.parts[p].length or 0 for p in self.groups[g]),-g))
        if not anchored and not pelvis and assembly.parts[selected].kind=='connector':
            hosts={self.group_of[j['b']['part']] for j in assembly.joints if j['type']=='socket' and not j.get('locked',False)
                   and self.group_of[j['a']['part']]==self.selected and self.group_of[j['b']['part']]!=self.selected}
            if hosts: self.root=max(hosts,key=lambda g:(len(self.groups[g]),sum(assembly.parts[p].length or 0 for p in self.groups[g]),-g))
        self.anchored=anchored
        self.tree=[];seen={self.root};queue=[self.root];used=set()
        for g in queue:
            for other,j in edges[g]:
                if other in seen: continue
                seen.add(other);queue.append(other);used.add(j['id']);self.tree.append((g,other,j,self.group_of[j['a']['part']]==g))
        self.joints=[j for j in assembly.joints if self.group_of[j['a']['part']] in component and self.group_of[j['a']['part']]!=self.group_of[j['b']['part']]]
        self.closures=[j for j in self.joints if j['id'] not in used]
        self.lower=[];self.upper=[];self.variables={};self.frames={}
        for j in self.joints:
            pa,_,axis=assembly.joint_frames(j);kind=joint_kind(j);a=assembly.parts[j['a']['part']]
            basis=a.matrix[:3,:3]@Rotation.from_euler('xyz',j['a'].get('frame',{}).get('rotation_deg',[0,0,0]),degrees=True).as_matrix()
            self.frames[j['id']]=(pa,axis,basis)
            entries=[]
            def variable(name,lo,hi,scale,axis_index=None):
                if lo>hi+1e-8: raise DocumentError(j['id']+': no remaining joint travel')
                if hi-lo<1e-10: entries.append((name,None,scale,axis_index,lo));return
                entries.append((name,len(self.lower),scale,axis_index,0));self.lower.append(lo/scale);self.upper.append(hi/scale)
            if kind in ('prismatic','cylindrical'):
                lo,hi=j.get('limits',{}).get('slide_mm',[-np.inf,np.inf])
                if j['type']=='socket':
                    tube=assembly.parts[j['b']['part']];port=a.ports[j['a']['port']]
                    if port.get('through'):
                        mouth,_=a.frame(j['a']);along=tube.matrix[:3,2];station=(mouth-tube.matrix[:3,3])@along+tube.length/2
                        sign=float(axis@along);half=port['engagement_mm']/2
                        ends=sorted([(station-half)/sign,(station-tube.length+half)/sign])
                    else:
                        depth=j.get('insertion_mm',0);ends=[depth-port['engagement_mm'],depth-port.get('min_engagement_mm',0)]
                    lo=max(lo,ends[0]);hi=min(hi,ends[1])
                variable('slide_mm',lo,hi,100)
            if kind in ('revolute','cylindrical'):
                lo,hi=j.get('limits',{}).get('angle_deg',[-179.99,179.99]);variable('angle_deg',max(lo,-179.99),min(hi,179.99),180/np.pi)
            if kind=='spherical':
                for i,(lo,hi) in enumerate(j.get('limits',{}).get('rotation_deg',[[-179,179],[-89.9,89.9],[-179,179]])):
                    variable('rotation_deg',lo,hi,180/np.pi,i)
            self.variables[j['id']]=entries
        self.lower=np.array(self.lower);self.upper=np.array(self.upper)
        parents={child:(parent,j) for parent,child,j,_ in self.tree}
        relevant={j['id'] for j in self.closures}
        ends={self.selected}|anchored|{self.group_of[j[e]['part']] for j in self.closures for e in ('a','b')}
        for g in ends:
            while g in parents:
                g,j=parents[g];relevant.add(j['id'])
        self.active=np.array([index for jid,entries in self.variables.items() if jid in relevant for _,index,_,_,_ in entries if index is not None],dtype=int)
        # Pose a limb from its anatomical base before recruiting the torso and
        # the opposite limbs. This also keeps interactive previews inexpensive.
        self.local_active=self.active
        if assembly.parts[selected].kind=='human':
            path=set();g=self.selected
            while g in parents:
                g,j=parents[g];path.add(j['id'])
                if any(p.split('/')[-1] in ('pelvis','thorax') for p in self.groups[g]):
                    self.local_active=np.array([index for jid,entries in self.variables.items() if jid in path for _,index,_,_,_ in entries if index is not None],dtype=int)
                    break

    def delta(self,j,q):
        pa,axis,basis=self.frames[j['id']];slide=angle=0.;angles=np.zeros(3)
        for name,index,scale,i,fixed in self.variables[j['id']]:
            value=q[index]*scale if index is not None else fixed
            if name=='slide_mm': slide=value
            elif name=='angle_deg': angle=value
            else: angles[i]=value
        r=basis@Rotation.from_euler('XYZ',angles,degrees=True).as_matrix()@basis.T if joint_kind(j)=='spherical' else Rotation.from_rotvec(axis*np.deg2rad(angle)).as_matrix()
        delta=np.eye(4);delta[:3,:3]=r;delta[:3,3]=pa-r@pa+axis*slide
        return delta

    def forward(self,q,root=None):
        deltas={self.root:np.eye(4) if root is None else root}
        for parent,child,j,forward in self.tree:
            delta=self.delta(j,q)
            if not forward:
                inverse=np.eye(4);inverse[:3,:3]=delta[:3,:3].T;inverse[:3,3]=-delta[:3,:3].T@delta[:3,3];delta=inverse
            deltas[child]=deltas[parent]@delta
        return deltas


def commit_transform(assembly, poses):
    """Revalidate and save a solved pose once, when a drag is accepted."""
    if not poses: return copy.deepcopy(assembly.doc)
    editable=_editable(assembly,set(poses))
    document=move_document(editable,poses)
    from .grouping import restore_objects
    return restore_objects(assembly.doc,document,assembly.base,assembly.library)


def drop_to_floor(assembly, selected, object_id=None, *, prepared=None):
    """Place the selected movement group on Z=0, without partially committing a blocked move."""
    from .geometry import lowest_z
    from .grouping import move_object
    if selected not in assembly.parts: raise DocumentError('Select an existing part to drop to the floor')
    if object_id is not None:
        instance=next((o for o in assembly.doc.get('objects',[]) if o['id']==object_id),None)
        if instance is None or not selected.startswith(object_id+'/'): raise DocumentError('Select a part of the grouped object to drop')
        members={p for p in assembly.parts if p.startswith(object_id+'/')}
        target=pose_of(transform(instance.get('pose')))
    else:
        mechanism=prepared.mechanism(selected) if prepared else Mechanism(assembly,selected)
        # Moving the unanchored root carries the whole mechanism, including any
        # hanging branches. Its lowest part must stop on the floor too.
        members=mechanism.parts if mechanism.selected==mechanism.root and not mechanism.anchored else set(mechanism.groups[mechanism.selected])
        target=pose_of(assembly.parts[selected].matrix)
    def height(poses):
        return min(lowest_z(assembly.parts[p],transform(poses[p]) if p in poses else None) for p in members)
    original_height=height({})
    result={'poses':{},'moved':[],'limited':False}
    if abs(original_height)<=1e-6:
        return {**result,'document':copy.deepcopy(assembly.doc),'message':'Already resting on Z = 0'}
    target['position_mm'][2]-=original_height
    for _ in range(8):
        result=move_object(assembly,object_id,target,preview=True) if object_id is not None else transform_part(assembly,selected,target,preview=True,prepared=prepared)
        remaining=height(result['poses'])
        if abs(remaining)<=.01: break
        if not result['moved']: break
        # Articulated placement can rotate a limb while moving it. Correct for
        # the resulting change in its lowest point, using the original frame.
        target['position_mm'][2]-=remaining
    else: remaining=height(result['poses'])
    if abs(remaining)>.01:
        anchors=[a['part'] for a in assembly.anchors if a['part'] in members]
        reason=f'{", ".join(anchors)} is fixed to the world.' if anchors else result.get('message','A connection limits this movement.')
        raise DocumentError(f'Cannot reach the floor: {reason} Loosen or detach the limiting connection, or release the world fixing, then retry.')
    document=move_object(assembly,object_id,target)['document'] if object_id is not None else commit_transform(assembly,result['poses'])
    return {**result,'document':document,'floor_gap_mm':remaining,'message':f'{"Lifted" if original_height<0 else "Dropped"} to the floor (Z = 0)'}


def transform_part(assembly, selected, target, mode='translate', seed=None, *, preview=False, prepared=None):
    if selected not in assembly.parts: raise DocumentError('Select an existing part to move')
    if mode not in ('translate','rotate'): raise DocumentError('Transform mode must be translate or rotate')
    instance=next((o for o in assembly.doc.get('objects',[]) if o['template']=='chain' and o.get('layout_mode')=='posable' and selected.startswith(o['id']+'/')),None)
    if instance:
        from .chain import pose_chain
        result=pose_chain(assembly,instance,selected,transform(target),mode,preview=preview)
        if result is not None: return result
    desired=transform(target);mechanism=prepared.mechanism(selected) if prepared else Mechanism(assembly,selected)
    original=assembly.parts[selected].matrix
    if mechanism.selected==mechanism.root:
        delta=np.eye(4) if mechanism.anchored else desired@np.linalg.inv(original)
        q=np.zeros(len(mechanism.lower));deltas=mechanism.forward(q,delta)
    elif not len(mechanism.active):
        q=np.zeros(len(mechanism.lower));deltas=mechanism.forward(q)
    else:
        q0=np.zeros(len(mechanism.lower))
        if isinstance(seed,list) and len(seed)==len(q0) and np.isfinite(seed).all(): q0=np.array(seed)
        q0=np.clip(q0,mechanism.lower+1e-8,mechanism.upper-1e-8)
        def difference(a,b):
            return np.r_[a[:3,3]-b[:3,3],100*Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).as_rotvec()]
        def residual(q):
            deltas=mechanism.forward(q);actual=deltas[mechanism.selected]@original
            position=(actual[:3,3]-desired[:3,3])*(.001 if mode=='rotate' else 1)
            angular=Rotation.from_matrix(actual[:3,:3]@desired[:3,:3].T).as_rotvec()*(1000 if mode=='rotate' else 2)
            errors=[position,angular,.01*q]
            for j in mechanism.closures:
                a,b=(mechanism.group_of[j[e]['part']] for e in ('a','b'))
                errors.append(1000*difference(deltas[a]@mechanism.delta(j,q),deltas[b]))
            for g in mechanism.anchored-{mechanism.root}: errors.append(1000*difference(deltas[g],np.eye(4)))
            return np.concatenate(errors)
        active=mechanism.local_active
        def full(values):
            q=np.zeros(len(mechanism.lower));q[active]=values;return q
        solved=least_squares(lambda values:residual(full(values)),q0[active],bounds=(mechanism.lower[active],mechanism.upper[active]),
                             max_nfev=65,ftol=None,xtol=1e-8,gtol=1e-7)
        q=full(solved.x);deltas=mechanism.forward(q)
    posed=copy.copy(assembly);posed.parts={p:copy.copy(v) for p,v in assembly.parts.items()}
    for p in mechanism.parts: posed.parts[p].matrix=deltas[mechanism.group_of[p]]@assembly.parts[p].matrix
    # A difficult or impossible loop must leave the authored assembly intact.
    # The user gets a stopped handle, not a partially disconnected mechanism.
    blocked=None
    try: _movement_coordinates(assembly,posed)
    except DocumentError as exc:
        blocked=str(exc);posed=assembly;q=np.zeros(len(mechanism.lower))
    poses={p:pose_of(v.matrix) for p,v in posed.parts.items() if not np.allclose(v.matrix,assembly.parts[p].matrix,atol=1e-7,rtol=0)}
    actual=posed.parts[selected].matrix
    position_error=float(np.linalg.norm(actual[:3,3]-desired[:3,3]))
    angle_error=float(np.rad2deg(Rotation.from_matrix(actual[:3,:3]@desired[:3,:3].T).magnitude()))
    limited=position_error>.5 if mode=='translate' else angle_error>.1
    result={'poses':poses,'moved':list(poses),'seed':q.tolist(),'limited':limited,
            'position_error_mm':position_error,'angle_error_deg':angle_error,
            'message':blocked or ('Movement stopped at the available joint travel' if limited else 'Connected joints follow the movement')}
    if not preview: result['document']=commit_transform(assembly,poses)
    return result
