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
        from .human_symmetry import CENTRAL_JOINTS, owner
        self.symmetry_owner=owner(assembly,selected)
        self.assembly=assembly;self.groups=assembly.editor_groups()
        self.group_of={p:i for i,g in enumerate(self.groups) for p in g};self.selected=self.group_of[selected]
        edges={i:[] for i in range(len(self.groups))}
        for j in assembly.joints:
            a,b=(self.group_of[j[e]['part']] for e in ('a','b'))
            if a!=b: edges[a].append((b,j));edges[b].append((a,j))
        component={self.selected};queue=[self.selected];distance={self.selected:0}
        for g in queue:
            for other,_ in edges[g]:
                if other not in component:
                    component.add(other);queue.append(other);distance[other]=distance[g]+1
        self.component=component;self.parts={p for g in component for p in self.groups[g]}
        anchored={self.group_of[a['part']] for a in assembly.anchors}&component
        pelvis=[g for g in component if any(assembly.parts[p].kind=='human' and p.split('/')[-1]=='pelvis' for p in self.groups[g])]
        # A compact chain is one large editor group, but its links are a
        # dangling load rather than a structural base. Root an unanchored
        # mechanism at its non-chain host so moving that host carries the
        # attached chain instead of merely swinging the host around it.
        chain_parts={p for instance in assembly.doc.get('objects',[]) if instance['template']=='chain'
                     for p in assembly.parts if p.startswith(instance['id']+'/')}
        hosts=[g for g in component if any(p not in chain_parts for p in self.groups[g])]
        candidates=hosts or component
        score=lambda g:(len(self.groups[g]),sum(assembly.parts[p].length or 0 for p in self.groups[g]),-g)
        structural=[g for g in candidates if any(assembly.parts[p].kind in ('member','connector') and p not in chain_parts for p in self.groups[g])]
        # Start at the nearest fixed group. A distant anchor must not recruit
        # every joint between it and a locally sliding selected part.
        self.root=min(anchored,key=lambda g:(distance[g],g)) if anchored else max(structural,key=score) if structural else pelvis[0] if pelvis else max(candidates,key=score)
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
                    if (self.symmetry_owner and j['id'].startswith(self.symmetry_owner['id']+'/')
                            and j['id'].split('/')[-1] in CENTRAL_JOINTS
                            and i != 0):
                        lo=hi=0
                    variable('rotation_deg',lo,hi,180/np.pi,i)
            self.variables[j['id']]=entries
        self.lower=np.array(self.lower);self.upper=np.array(self.upper)
        parents={child:(parent,j) for parent,child,j,_ in self.tree}
        def path_to_root(group):
            path=set()
            while group in parents:
                group,j=parents[group];path.add(j['id'])
            return path
        relevant=path_to_root(self.selected)
        closure_paths=[(j,path_to_root(self.group_of[j['a']['part']]) ^
                           path_to_root(self.group_of[j['b']['part']])) for j in self.closures]
        anchor_paths=[(g,path_to_root(g)) for g in anchored-{self.root}]
        # A stationary off-path branch remains valid with all its coordinates
        # at zero. Pull in a loop or another anchor only when the moving path
        # crosses it, then repeat because that path may meet another loop.
        while True:
            expanded=set(relevant)
            for joint,path in closure_paths:
                if path & relevant: expanded.update(path);expanded.add(joint['id'])
            for _,path in anchor_paths:
                if path & relevant: expanded.update(path)
            if expanded==relevant: break
            relevant=expanded
        self.active_closures=[j for j,path in closure_paths if path & relevant]
        self.active_anchors={g for g,path in anchor_paths if path & relevant}
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
            if all((q[index] if index is not None else fixed)==0
                   for _,index,_,_,fixed in self.variables[j['id']]):
                # Unchanged branches inherit the parent's rigid transform.
                # Building hundreds of identity joint rotations in every fit
                # evaluation makes a local edit scale with its hanging payload.
                deltas[child]=deltas[parent]
                continue
            delta=self.delta(j,q)
            if not forward:
                inverse=np.eye(4);inverse[:3,:3]=delta[:3,:3].T;inverse[:3,3]=-delta[:3,:3].T@delta[:3,3];delta=inverse
            deltas[child]=deltas[parent]@delta
        return deltas


def commit_transform(assembly, poses):
    """Revalidate and save a solved pose once, when a drag is accepted."""
    if not poses: return copy.deepcopy(assembly.doc)
    from .snapping import rigid_draft_follow, _rigid_object_transforms

    poses,_=rigid_draft_follow(assembly,poses)
    try: _rigid_object_transforms(assembly,poses)
    except DocumentError: editable=_editable(assembly,set(poses))
    else: editable=assembly if not assembly.doc.get('state',{}).get('joints') else _editable(assembly,set(poses))
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

    # Floor placement is a layout operation. An IK reference body is not a
    # world fixing: any free attachment component has an exact vertical move,
    # even when the selected body cannot slide vertically relative to its root.
    # Include complete reusable objects so attached payloads keep their pose.
    graph={p:set() for p in assembly.parts}
    for joint in assembly.joints:
        a,b=joint['a']['part'],joint['b']['part']
        graph[a].add(b);graph[b].add(a)
    for obj in assembly.doc.get('objects',[]):
        parts=[p for p in assembly.parts if p.startswith(obj['id']+'/')]
        for p in parts[1:]:
            graph[parts[0]].add(p);graph[p].add(parts[0])
    component={selected};queue=[selected]
    for p in queue:
        for other in graph[p]-component:
            component.add(other);queue.append(other)
    anchors=[a['part'] for a in assembly.anchors if a['part'] in component]
    if not anchors:
        from .snapping import rigid_draft_follow
        # Discover free draft attachments with a unit translation, using the
        # same following rules as the eventual commit. Include their support
        # geometry before calculating the final translation.
        probe={p:pose_of(assembly.parts[p].matrix) for p in component}
        for pose in probe.values(): pose['position_mm'][2]+=1
        probe,followed_runs=rigid_draft_follow(assembly,probe)
        heights=[lowest_z(assembly.parts[p]) for p in probe]
        if followed_runs:
            from .document import Part
            from .drafting import preview
            for run in preview(assembly):
                if run['id'] in followed_runs:
                    part=Part(run['id'],{},run,transform(run['pose']),assembly.library.paths.get(run['catalog'],assembly.base))
                    heights.append(lowest_z(part))
        original_height=min(heights)
        if abs(original_height)<=1e-6:
            return {'poses':{},'moved':[],'limited':False,'floor_gap_mm':original_height,
                    'document':copy.deepcopy(assembly.doc),'message':'Already resting on Z = 0'}
        poses={p:pose_of(assembly.parts[p].matrix) for p in probe}
        for pose in poses.values(): pose['position_mm'][2]-=original_height
        document=commit_transform(assembly,poses)
        return {'poses':poses,'moved':list(poses),'limited':False,'document':document,'floor_gap_mm':0.,
                'message':f'{"Lifted" if original_height<0 else "Dropped"} connected assembly to the floor (Z = 0); shape preserved'}

    if object_id is not None:
        members={p for p in assembly.parts if p.startswith(object_id+'/')}
        target=pose_of(transform(instance.get('pose')))
    else:
        mechanism=prepared.mechanism(selected) if prepared else Mechanism(assembly,selected)
        members=set(mechanism.groups[mechanism.selected])
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
        fixing=f'{", ".join(anchors)} {"is" if len(anchors)==1 else "are"} fixed to the world'
        side='above' if remaining>0 else 'below'
        raise DocumentError(f'Cannot reach the floor: {fixing}. The available joint motion leaves '
                            f'{selected}\'s moving body {abs(remaining):.2f} mm {side} Z = 0. '
                            'Release that world fixing to place the connected assembly as a whole.')
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
    from .human_symmetry import mirror_limb_poses, owner, project_pelvis_target, validate_human_symmetry
    symmetry_owner=owner(assembly,selected)
    desired=transform(target);mechanism=prepared.mechanism(selected) if prepared else Mechanism(assembly,selected)
    original=assembly.parts[selected].matrix
    if symmetry_owner and selected==symmetry_owner['id']+'/pelvis':
        desired=transform(project_pelvis_target(target,original))
    slide_guided=slide_stopped=False
    if mechanism.selected==mechanism.root:
        delta=np.eye(4) if mechanism.anchored else desired@np.linalg.inv(original)
        q=np.zeros(len(mechanism.lower));deltas=mechanism.forward(q,delta)
    elif not len(mechanism.active):
        q=np.zeros(len(mechanism.lower));deltas=mechanism.forward(q)
    else:
        q0=np.zeros(len(mechanism.lower))
        if isinstance(seed,list) and len(seed)==len(q0) and np.isfinite(seed).all(): q0=np.array(seed)
        q0=np.clip(q0,mechanism.lower+1e-8,mechanism.upper-1e-8)
        direct=next(((parent,j,forward) for parent,child,j,forward in mechanism.tree
                     if child==mechanism.selected and parent==mechanism.root),None)
        slide=next(((index,scale) for name,index,scale,_,_ in mechanism.variables[direct[1]['id']]
                    if name=='slide_mm' and index is not None),None) if direct else None
        if (mode=='translate' and direct and slide and not mechanism.active_closures and not mechanism.active_anchors
                and joint_kind(direct[1]) in ('prismatic','cylindrical')
                and np.allclose(desired[:3,:3],original[:3,:3],atol=1e-5,rtol=0)):
            # A leaf sliding on one guide has an exact one-dimensional answer.
            # Do not fit unrelated hanging chains or free limbs to this target.
            axis=mechanism.frames[direct[1]['id']][1]*(1 if direct[2] else -1)
            index,scale=slide
            q=np.zeros(len(mechanism.lower))
            requested=float((desired[:3,3]-original[:3,3])@axis)/scale
            q[index]=np.clip(requested,mechanism.lower[index],mechanism.upper[index])
            slide_guided=True;slide_stopped=abs(requested-q[index])>1e-7
            deltas=mechanism.forward(q)
        else:
            def difference(a,b):
                return np.r_[a[:3,3]-b[:3,3],100*Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).as_rotvec()]
            def residual(q):
                deltas=mechanism.forward(q);actual=deltas[mechanism.selected]@original
                position=(actual[:3,3]-desired[:3,3])*(.001 if mode=='rotate' else 1)
                angular=Rotation.from_matrix(actual[:3,:3]@desired[:3,:3].T).as_rotvec()*(1000 if mode=='rotate' else 2)
                errors=[position,angular,.01*q]
                for j in mechanism.active_closures:
                    a,b=(mechanism.group_of[j[e]['part']] for e in ('a','b'))
                    errors.append(1000*difference(deltas[a]@mechanism.delta(j,q),deltas[b]))
                for g in mechanism.active_anchors: errors.append(1000*difference(deltas[g],np.eye(4)))
                return np.concatenate(errors)
            active=mechanism.local_active
            def full(values):
                q=np.zeros(len(mechanism.lower));q[active]=values;return q
            solved=least_squares(lambda values:residual(full(values)),q0[active],bounds=(mechanism.lower[active],mechanism.upper[active]),
                                 max_nfev=65,ftol=None,xtol=1e-8,gtol=1e-7)
            q=full(solved.x);deltas=mechanism.forward(q)
    posed=copy.copy(assembly);posed.parts={p:copy.copy(v) for p,v in assembly.parts.items()}
    identity=np.eye(4)
    moved_groups={g for g in mechanism.component if not np.array_equal(deltas[g],identity)}
    for p in mechanism.parts:
        if mechanism.group_of[p] in moved_groups:
            posed.parts[p].matrix=deltas[mechanism.group_of[p]]@assembly.parts[p].matrix
    if symmetry_owner:
        mirror_limb_poses(posed,symmetry_owner,selected)
    # A difficult or impossible loop must leave the authored assembly intact.
    # The user gets a stopped handle, not a partially disconnected mechanism.
    blocked=None
    try:
        if symmetry_owner: validate_human_symmetry(posed)
        _movement_coordinates(assembly,posed)
    except DocumentError as exc:
        blocked=str(exc);posed=assembly;q=np.zeros(len(mechanism.lower))
    candidates=posed.parts if symmetry_owner else (p for p in mechanism.parts if mechanism.group_of[p] in moved_groups)
    poses={p:pose_of(posed.parts[p].matrix) for p in candidates
           if not np.allclose(posed.parts[p].matrix,assembly.parts[p].matrix,atol=1e-7,rtol=0)}
    if poses and assembly.doc.get('draft_subassemblies'):
        from .snapping import apply_rigid_draft_follow, check_rigid_draft_follow, rigid_draft_follow

        poses,run_transforms=rigid_draft_follow(assembly,poses)
        if run_transforms:
            for pid,pose in poses.items(): posed.parts[pid].matrix=transform(pose)
            candidate=copy.deepcopy(assembly.doc)
            apply_rigid_draft_follow(candidate,run_transforms)
            try: check_rigid_draft_follow(assembly,posed,candidate,run_transforms)
            except DocumentError as exc:
                blocked=str(exc);posed=assembly;poses={};q=np.zeros(len(mechanism.lower))
    actual=posed.parts[selected].matrix
    position_error=float(np.linalg.norm(actual[:3,3]-desired[:3,3]))
    angle_error=float(np.rad2deg(Rotation.from_matrix(actual[:3,:3]@desired[:3,:3].T).magnitude()))
    limited=position_error>.5 if mode=='translate' else angle_error>.1
    message=blocked or ('Movement stopped at the available joint travel' if slide_stopped
                        else 'Movement follows the available slide axis' if limited and slide_guided
                        else 'Movement stopped at the available joint travel' if limited
                        else 'Connected joints follow the movement')
    result={'poses':poses,'moved':list(poses),'seed':q.tolist(),'limited':limited,
            'position_error_mm':position_error,'angle_error_deg':angle_error,
            'message':message}
    if not preview: result['document']=commit_transform(assembly,poses)
    return result
