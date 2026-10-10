"""Preview pipe/socket alignment without breaking existing assembly constraints."""
import copy
import itertools

import numpy as np
from scipy.spatial.transform import Rotation

from .connections import socket_attachment
from .document import Assembly, DocumentError, joint_kind
from .editing import snapshot_design
from .math3d import align_axis, pose_of, transform


def _rigid_object_transforms(assembly, poses):
    """Represent whole-object layout edits by parent poses, without rebaking joints."""
    direct={p['id'] for p in assembly.doc['parts']}
    internal=set(poses)-direct
    transforms={}
    for instance in assembly.doc.get('objects',[]):
        members={pid for pid in assembly.parts if pid.startswith(instance['id']+'/')}
        if not members&internal: continue
        if not members<=poses.keys():
            raise DocumentError('Expand the reusable object before moving its internal parts')
        first=next(iter(members))
        delta=transform(poses[first])@np.linalg.inv(assembly.parts[first].matrix)
        if any(not np.allclose(transform(poses[pid]),delta@assembly.parts[pid].matrix,atol=2e-5,rtol=0) for pid in members):
            raise DocumentError('Expand the reusable object before moving its internal parts')
        transforms[instance['id']]=delta
        internal-=members
    if internal: raise DocumentError('Select existing parts to move')
    return transforms


def _pose_changes(assembly, poses):
    if assembly.doc.get('state',{}).get('joints'):
        raise DocumentError('Capture the motion frame before moving parts in this pose')
    _rigid_object_transforms(assembly,poses)
    posed=copy.copy(assembly)
    posed.parts={pid:copy.copy(part) for pid,part in assembly.parts.items()}
    for pid,pose in poses.items(): posed.parts[pid].matrix=transform(pose)
    return posed


def _movement_coordinates(before, after):
    """Measure allowed relative motion; anchors and rigid relations stay fixed.

    This checks a requested rigid movement, rather than solving general inverse
    kinematics. Hinges, sliders, ball joints and loose sockets retain their DOFs.
    """
    for anchor in before.anchors:
        pid=anchor['part']
        if not np.allclose(before.parts[pid].matrix,after.parts[pid].matrix,atol=1e-5,rtol=0):
            raise DocumentError(f'{pid} is fixed to the world')
    coordinates={};socket_contacts=[]
    for joint in before.joints:
        aid,bid=joint['a']['part'],joint['b']['part']
        a0,b0=before.parts[aid],before.parts[bid]; a1,b1=after.parts[aid],after.parts[bid]
        rel0=np.linalg.inv(a0.matrix)@b0.matrix; rel1=np.linalg.inv(a1.matrix)@b1.matrix
        if np.allclose(rel0,rel1,atol=1e-5,rtol=0): continue
        kind=joint_kind(joint); name=joint['id']
        if kind=='fixed': raise DocumentError(f'Move the whole rigid body to preserve {name}')
        pa0,pb0,_=before.joint_frames(joint); pa1,pb1,_=after.joint_frames(joint)
        e0=a0.matrix[:3,:3].T@(pb0-pa0); e1=a1.matrix[:3,:3].T@(pb1-pa1)
        offset=e1-e0; axis=a0.local_frame(joint['a'])[1]
        change=rel1[:3,:3]@rel0[:3,:3].T
        vector=Rotation.from_matrix(change).as_rotvec()
        slide=float(offset@axis); angle=float(np.rad2deg(vector@axis))
        linear_error=np.linalg.norm(offset-axis*slide) if kind in ('prismatic','cylindrical') else np.linalg.norm(offset)
        angular_error=0 if kind in ('spherical','distance') else np.linalg.norm(vector-axis*np.deg2rad(angle)) if kind in ('revolute','cylindrical') else np.linalg.norm(vector)
        if kind=='distance': linear_error=abs(np.linalg.norm(e1)-np.linalg.norm(e0))
        linear_limit=.05
        if joint['type']=='socket' and kind in ('prismatic','cylindrical') and 'fit_tolerance_mm' in joint:
            # A socket accepted with a small assembly allowance keeps that
            # allowance when loosened and rotated. Check its current gap, so
            # repeated edits cannot accumulate clearance beyond the limit.
            linear_error=max(0.,float(np.linalg.norm(e1-axis*(e1@axis)))-joint['fit_tolerance_mm'])
            linear_limit=1e-7
        if linear_error>linear_limit or angular_error>1e-4:
            raise DocumentError(f'This movement exceeds the freedom of {name} ({kind})')
        values={}
        if kind in ('prismatic','cylindrical'): values['slide_mm']=slide
        if kind in ('revolute','cylindrical'): values['angle_deg']=angle
        if kind=='spherical':
            basis=Rotation.from_euler('xyz',joint['a'].get('frame',{}).get('rotation_deg',[0,0,0]),degrees=True).as_matrix()
            values['rotation_deg']=Rotation.from_matrix(basis.T@change@basis).as_euler('XYZ',degrees=True).tolist()
        for coordinate,limits in joint.get('limits',{}).items():
            value=values.get(coordinate,0)
            if coordinate=='rotation_deg': outside=any(v<lo-1e-5 or v>hi+1e-5 for v,(lo,hi) in zip(value,limits))
            else: outside=value<limits[0]-1e-5 or value>limits[1]+1e-5
            if outside: raise DocumentError(f'{name} would exceed its {coordinate} limits')
        coordinates[name]=values
        if joint.get('metadata',{}).get('flexibility')=='full_socket_span' or joint.get('metadata',{}).get('dislocated'):
            socket_contacts.append((aid,bid,name))
    if socket_contacts:
        # Unrestricted sockets retain geometry stops. Ignore only the overlap
        # already present at the joint caps in the reference mannequin.
        from .validation import CollisionWorld
        members={pid for a,b,_ in socket_contacts for pid in (a,b)}
        def subset(assembly):
            result=copy.copy(assembly);result.parts={pid:assembly.parts[pid] for pid in members}
            return result
        with CollisionWorld(subset(before)) as old, CollisionWorld(subset(after)) as new:
            for a,b,name in socket_contacts:
                allowance=max([0., *[-c[8]*1000 for c in old.contacts(a,b)]])
                depth=max([0., *[-c[8]*1000 for c in new.contacts(a,b)]])
                if depth>allowance+1.:
                    raise DocumentError(f'{name}: socket movement would intersect its parent body')
    return coordinates


def rigid_draft_follow(assembly, poses):
    """Carry a free draft attachment graph through one rigid exact-body move.

    Draft runs are not Assembly parts, so the exact joint mechanism cannot see
    them. A connector with a dangling chain and a draft through-run is one
    movable structure only when every already-moving exact joint component has
    the same rigid transform. Articulated edits and anchored graphs retain their
    existing, more constrained behavior.
    """
    from .drafting import runs

    result=dict(poses)
    draft_runs=runs(assembly.doc)
    if not result or not draft_runs: return result,{}
    mirror_modes={run['id']: [plane.get('run_modes',{}).get(run['id'],'free')
                              for plane in group.get('mirrors',[])]
                  for group in assembly.doc.get('draft_subassemblies',[])
                  for run in group['runs']}
    exact={pid:set() for pid in assembly.parts}
    graph={pid:set() for pid in assembly.parts}
    for run in draft_runs: graph[run['id']]=set()
    for joint in assembly.joints:
        a,b=joint['a']['part'],joint['b']['part']
        exact[a].add(b);exact[b].add(a)
        graph[a].add(b);graph[b].add(a)
    for run in draft_runs:
        for attachment in run.get('attachments',[]):
            connector=attachment['connector']
            graph[run['id']].add(connector);graph[connector].add(run['id'])

    anchored={anchor['part'] for anchor in assembly.anchors}
    visited=set();transforms={}
    for seed in poses:
        if seed in visited: continue
        component={seed};queue=[seed]
        for node in queue:
            for other in graph[node]-component:
                component.add(other);queue.append(other)
        visited.update(component)
        contained_runs={run['id'] for run in draft_runs if run['id'] in component}
        if not contained_runs or component&anchored: continue
        # A constrained run may change its working span or insertion depth
        # when a fitting moves. Let the mirror fitter handle that motion;
        # translating the whole run would move its midpoint off the plane.
        if any(mode!='free' for run_id in contained_runs
               for mode in mirror_modes.get(run_id,())):
            continue
        moved=[pid for pid in component if pid in result and pid in assembly.parts]
        if not moved: continue
        delta=transform(result[moved[0]])@np.linalg.inv(assembly.parts[moved[0]].matrix)
        if any(not np.allclose(transform(result[pid])@np.linalg.inv(assembly.parts[pid].matrix),
                               delta,atol=1e-5,rtol=0) for pid in moved[1:]):
            continue
        # A partially moved exact mechanism is an articulation, not a rigid
        # structure. The draft graph must not turn such an edit into a drag of
        # all its other limbs or links.
        exact_seen=set();articulated=False
        for pid in moved:
            if pid in exact_seen: continue
            exact_component={pid};exact_queue=[pid]
            for node in exact_queue:
                for other in exact[node]-exact_component:
                    exact_component.add(other);exact_queue.append(other)
            exact_seen.update(exact_component)
            if any(part not in result for part in exact_component):
                articulated=True;break
        if articulated: continue
        for pid in component & assembly.parts.keys():
            if pid not in result:
                result[pid]=pose_of(delta@assembly.parts[pid].matrix)
        transforms.update({run_id:delta for run_id in contained_runs})
    return result,transforms


def apply_rigid_draft_follow(doc, transforms):
    """Write the same world transform to both endpoints of followed runs."""
    if not transforms: return
    from .drafting import runs

    for run in runs(doc):
        delta=transforms.get(run['id'])
        if delta is None: continue
        for endpoint in ('start_mm','end_mm'):
            run[endpoint]=(delta[:3,:3]@np.asarray(run[endpoint],dtype=float)+delta[:3,3]).tolist()


def check_rigid_draft_follow(before, posed, document, transforms):
    """Reject a new fit or mirror conflict after a rigid draft graph move."""
    if not transforms: return
    from .drafting import preview

    def by_key(assembly):
        return {(conflict.get('run'),conflict.get('code'),conflict.get('connector'),
                 conflict.get('port'),conflict.get('plane')):conflict
                for item in preview(assembly) for conflict in item['conflicts']}

    original=by_key(before)
    candidate=copy.copy(posed);candidate.doc=document
    for key,conflict in by_key(candidate).items():
        old=original.get(key)
        if old is None or any(float(conflict.get(field,0))>float(old.get(field,0))+1e-6
                              for field in ('residual_mm','residual_deg')):
            raise DocumentError(conflict['message'])


def move_document(assembly, poses):
    poses,run_transforms=rigid_draft_follow(assembly,poses)
    posed=_pose_changes(assembly,poses)
    coordinates=_movement_coordinates(assembly,posed)
    doc=copy.deepcopy(assembly.doc)
    for spec in doc['parts']:
        if spec['id'] in poses: spec['pose']=pose_of(posed.parts[spec['id']].matrix)
    object_transforms=_rigid_object_transforms(assembly,poses)
    for instance in doc.get('objects',[]):
        if instance['id'] in object_transforms:
            instance['pose']=pose_of(object_transforms[instance['id']]@transform(instance.get('pose')))
    if doc.get('draft_subassemblies'):
        from .symmetry import fit_moved_centered_runs
        apply_rigid_draft_follow(doc,run_transforms)
        moved_with_runs={attachment['connector'] for run in doc['draft_subassemblies']
                         for item in run['runs'] if item['id'] in run_transforms
                         for attachment in item.get('attachments',[])}
        fit_moved_centered_runs(posed, doc, set(poses)-moved_with_runs)
        check_rigid_draft_follow(assembly,posed,doc,run_transforms)
    if coordinates:
        # Rebase the authored zero and remaining joint travel, including drives,
        # without expanding unrelated reusable objects in the user's document.
        rebased=snapshot_design(assembly,{'time_s':0,'parts':{pid:pose_of(p.matrix) for pid,p in posed.parts.items()},'joints':coordinates})
        by_id={j['id']:j for j in rebased['joints']}
        doc['joints']=[by_id[j['id']] for j in doc.get('joints',[])]
        if 'drives' in doc: doc['drives']=rebased['drives']
        for track in doc.get('animation',{}).get('tracks',[]):
            shift=coordinates.get(track['joint'],{}).get(track['coordinate'],0)
            for key in track['keyframes']: key['value']-=shift
    doc.pop('results',None); doc.pop('build_plan',None)
    return doc


def connection_collisions(before, after, relevant, new_pair, *, cancelled=None):
    """Return blocking contacts, using the same mounting allowances as validation."""
    from .validation import CollisionWorld, _socket_interface_contact
    from .geometry import collision_bounds
    from .connection_jobs import check_cancelled
    check_cancelled(cancelled)
    pairs={}
    for joint in after.joints:
        pairs.setdefault(frozenset((joint['a']['part'],joint['b']['part'])),[]).append(joint)
    # Internal contacts in a rigidly transported assembly cannot change.
    # Broad-phase the remaining pairs before constructing any Bullet bodies;
    # remote bones and chain links need no collision world for this operation.
    delta_groups=[];motion={};same_geometry={}
    for pid,part in after.parts.items():
        check_cancelled(cancelled)
        delta=part.matrix@np.linalg.inv(before.parts[pid].matrix)
        group=next((i for i,m in enumerate(delta_groups) if np.allclose(delta,m,atol=1e-5,rtol=0)),None)
        if group is None: group=len(delta_groups);delta_groups.append(delta)
        motion[pid]=group;same_geometry[pid]=part.shapes==before.parts[pid].shapes
    mesh_cache={};boxes={};candidates=[]
    for a,b in itertools.combinations(after.parts,2):
        check_cancelled(cancelled)
        if not relevant.intersection((a,b)): continue
        if {a,b}!=new_pair and motion[a]==motion[b] and same_geometry[a] and same_geometry[b]: continue
        for pid in (a,b):
            if pid not in boxes: boxes[pid]=collision_bounds(after.parts[pid],mesh_cache)
        if np.any(boxes[a][1]<boxes[b][0]-.5) or np.any(boxes[b][1]<boxes[a][0]-.5): continue
        candidates.append((a,b))
    if not candidates: return []
    subset=copy.copy(after);members={pid for pair in candidates for pid in pair}
    subset.parts={pid:part for pid,part in after.parts.items() if pid in members}
    collisions=[]
    with CollisionWorld(subset) as world:
        for a,b in candidates:
            check_cancelled(cancelled)
            pair=pairs.get(frozenset((a,b)),[])
            if pair and after.parts[a].kind==after.parts[b].kind=='human': continue
            contacts=[c for c in world.contacts(a,b) if c[8]*1000 < -1.]
            contacts=[c for c in contacts if not any(j['type']=='socket' and _socket_interface_contact(after,j,c) for j in pair)]
            if not contacts: continue
            if any(j['type']!='socket' and all(np.linalg.norm(np.array(c[5])*1000-after.joint_frames(j)[0])<12 for c in contacts) for j in pair): continue
            collisions.append((a,b,min(contacts,key=lambda c:c[8])))
    return collisions


def connection_options(assembly, member, connector, port, end='start', insertion_mm=None,
                       at_mm=None, locked=True, poses=None, replace_joint=None, force=False, tolerance_mm=2., force_options=None,
                       *, first_valid=False, cancelled=None):
    """Return independently checked alternatives; making a preview never edits input."""
    original=assembly
    from .connection_jobs import check_cancelled
    check_cancelled(cancelled)
    from .fit_adjustments import settings,adjust_connection
    controls=settings(tolerance_mm,force_options)
    tolerance_mm=controls['tolerance_mm']
    from .posing import _editable
    assembly=_editable(assembly,{member,connector,*list(poses or {})})
    doc=copy.deepcopy(assembly.doc)
    if replace_joint:
        old=next((j for j in doc.get('joints',[]) if j['id']==replace_joint),None)
        if not old or old.get('type')!='socket' or old['a']['part']!=connector or old['b']['part']!=member:
            raise DocumentError('Select an existing socket connection to edit')
        doc['joints']=[j for j in doc['joints'] if j['id']!=replace_joint]
        assembly=Assembly.from_doc(doc,assembly.base,assembly.library)
    endpoint,insertion=socket_attachment(assembly,member,connector,port,end,insertion_mm,at_mm)
    groups=assembly.editor_groups()
    components=assembly.connected_groups()
    member_component=next(g for g in components if member in g)
    connector_component=next(g for g in components if connector in g)
    separate=connector not in member_component
    anchored={a['part'] for a in assembly.anchors}
    poses=dict(poses or {})
    if separate:
        for selected,component in ((member,member_component),(connector,connector_component)):
            moved=[pid for pid in component if pid in poses]
            if not moved or anchored.intersection(component): continue
            reference=selected if selected in poses else moved[0]
            delta=transform(poses[reference])@np.linalg.inv(assembly.parts[reference].matrix)
            # Browser pose serialization rounds positions and Euler angles.
            # Compare position and orientation separately: micrometre rounding
            # must not turn a rigid drag into an attempted chain articulation.
            def follows(pid):
                actual=transform(poses[pid]);expected=delta@assembly.parts[pid].matrix
                return (np.allclose(actual[:3,3],expected[:3,3],atol=.002,rtol=0) and
                        np.allclose(actual[:3,:3],expected[:3,:3],atol=2e-6,rtol=0))
            if all(follows(pid) for pid in moved):
                poses.update({pid:pose_of(delta@assembly.parts[pid].matrix) for pid in component})
    placed=_pose_changes(assembly,poses)
    socket=assembly.parts[connector].ports[port]
    joint={'id':replace_joint or f'{connector}-{port}-{member}','type':'socket',
           'a':{'part':connector,'port':port},'b':endpoint,'insertion_mm':insertion,'locked':bool(locked),'fit_tolerance_mm':tolerance_mm}
    if replace_joint:
        # Preserve hardware notes and limits when editing an existing attachment.
        joint={**copy.deepcopy(old),**joint}
    else:
        existing={j['id'] for j in assembly.joints}; root=joint['id']; n=2
        while joint['id'] in existing: joint['id']=f'{root}-{n}'; n+=1
    options=[]
    from .validation import validate
    issue_key=lambda i:(i['code'],tuple(sorted(i['parts'])),i.get('joint'))
    baseline_errors={issue_key(i) for i in validate(assembly,collisions=False,geometry=False,support_checks=False)['issues'] if i['severity']=='error'}
    from .sliding import sliding_context,slide_connection
    sliding=sliding_context(assembly,connector,member)
    choices=[('connector',connector),('member',member)]
    if poses and member in poses and connector not in poses: choices.reverse()
    choices+=([('both',None)] if sliding else [])
    choices.append(('fit',None))
    for side,pid in choices:
        check_cancelled(cancelled)
        if side=='fit' and any(o['available'] and (not force or not o.get('requires_preview')) for o in options): continue
        if side=='fit' and force: options=[]
        group=next((g for g in groups if pid in g),[])
        if separate and pid is not None:
            component=member_component if pid==member else connector_component
            if not anchored.intersection(component): group=component
        label='Solve hinges and slides together' if side=='fit' else 'Slide connected bodies together' if side=='both' else ('Connector' if side=='connector' else 'Pipe')+f' and connected body ({len(group)} parts)'
        option={'move':side,'label':label,'available':False}
        try:
            fitted_joint=joint
            adjusted_document=None
            if side=='fit':
                from .fitting import fit_connection
                if force and (controls['unlock_connectors'] or controls['resize_members']):
                    adjusted=adjust_connection(assembly,joint,placed,controls,cancelled=cancelled)
                    final_poses,fitted_joint,motions,context,angle=adjusted['fit']
                    adjusted_document=adjusted['document']
                    option.update(adjustments=adjusted['adjustments'])
                else:
                    final_poses,fitted_joint,motions,context,angle=fit_connection(assembly,joint,placed,force,tolerance_mm=tolerance_mm,cancelled=cancelled)
                rotation=Rotation.from_rotvec([np.deg2rad(angle),0,0]).as_matrix()
                description=f'{len(motions)} joints adjusted continuously; rotation snap increments do not limit the fit.'
                if fitted_joint!=joint:
                    value=fitted_joint['b']['at_mm'] if socket.get('through') else fitted_joint['insertion_mm']
                    description+=f' Fitted socket {"centre from pipe start" if socket.get("through") else "insertion"}: {value:.2f} mm.'
                from .validation import SOCKET_AXIS_MIN_DOT
                # Small alignment corrections on loose socket collars are an
                # ordinary connection. Structural hinge poses and Force changes
                # still get a preview before committing.
                moved_joints={m['joint'] for m in motions}
                local_alignment=(not force and angle<=np.rad2deg(np.arccos(SOCKET_AXIS_MIN_DOT)) and
                    all(j['type']=='socket' and not j.get('locked') for j in assembly.joints if j['id'] in moved_joints))
                option.update(motions=motions,context_parts=context,requires_preview=not local_alignment,local_alignment=local_alignment,description=description,
                              label=f'Solve hinges and slides together ({len(final_poses)} parts)',joint=fitted_joint)
            elif side=='both':
                final_poses,slides,context=slide_connection(assembly,joint,placed,sliding);rotation=np.eye(3)
                option.update(slides=slides,context_parts=context,requires_preview=True,
                              label=f'Slide connected bodies together ({len(final_poses)} parts)',
                              description=f'{len(slides)} sliding joints adjusted along their existing axes.')
            else:
                fitted=copy.copy(placed)
                fitted.parts={key:copy.copy(part) for key,part in placed.parts.items()}
                tube=fitted.parts[member]; fitting=fitted.parts[connector]
                mouth,axis=fitting.frame(joint['a']); point,tube_axis=tube.frame(endpoint)
                if socket.get('through') and axis@tube_axis<0: tube_axis=-tube_axis
                rotation=align_axis(axis,tube_axis) if side=='connector' else align_axis(tube_axis,axis)
                delta=np.eye(4); delta[:3,:3]=rotation
                delta[:3,3]=(point+tube_axis*insertion-rotation@mouth) if side=='connector' else (mouth-axis*insertion-rotation@point)
                if member in group and connector in group:
                    if np.linalg.norm(mouth-axis*insertion-point)>tolerance_mm+1e-7 or np.linalg.norm(rotation-np.eye(3))>1e-4:
                        raise DocumentError('These parts already belong to one rigid body; their attachment frames must already coincide')
                    delta=np.eye(4)
                for moved in group: fitted.parts[moved].matrix=delta@fitted.parts[moved].matrix
                final_poses={i:pose_of(p.matrix) for i,p in fitted.parts.items() if not np.allclose(p.matrix,assembly.parts[i].matrix,atol=1e-7,rtol=0)}
            if adjusted_document is not None:
                result=adjusted_document
            else:
                try: _rigid_object_transforms(assembly,final_poses)
                except DocumentError: editable=_editable(assembly,set(final_poses))
                else: editable=assembly
                result=move_document(editable,final_poses)
            result.setdefault('joints',[]).append(fitted_joint)
            final=Assembly.from_doc(result,assembly.base,assembly.library)
            pa,pb,_=final.joint_frames(fitted_joint);gap=float(np.linalg.norm(pb-pa))
            option['gap_mm']=gap;option['tolerance_mm']=tolerance_mm
            if gap>.03:
                if not option.get('local_alignment'): option['requires_preview']=True
                option['description']=option.get('description','')+f' Accepted connection gap: {gap:.2f} mm (allowance {tolerance_mm:g} mm).'
            # Exact attachment and engagement checks precede contact tests.
            report=validate(final,collisions=False,geometry=adjusted_document is not None,support_checks=False)
            relevant=set(final_poses)|{member,connector}
            errors=[i for i in report['issues'] if i['severity']=='error' and issue_key(i) not in baseline_errors and (not i['parts'] or relevant.intersection(i['parts']))]
            if errors: raise DocumentError(errors[0]['message'])
            collisions=connection_collisions(assembly,final,relevant,{member,connector},cancelled=cancelled)
            if collisions:
                left,right,_=collisions[0]
                raise DocumentError(f'{left} would intersect {right}')
            from .grouping import restore_objects
            result=restore_objects(original.doc,result,assembly.base,assembly.library)
            if option.get('adjustments',{}).get('resized'):
                option['preview_parts']=[p for p in final.scene()['parts'] if p['id'] in {r['part'] for r in option['adjustments']['resized']}]
            option.update(available=True,document=result,poses=final_poses,moved=list(final_poses),
                          rotation_deg=float(np.rad2deg(Rotation.from_matrix(rotation).magnitude())))
        except DocumentError as exc: option['reason']=str(exc)
        options.append(option)
        # A clear drop intent can commit immediately. Review requests still
        # enumerate both sides and articulated alternatives as before.
        if first_valid and option['available'] and not option.get('requires_preview'): break
    check_cancelled(cancelled)
    recommended=next((o['move'] for o in options if o['available']),None)
    return {'options':options,'recommended':recommended,'joint':joint}
