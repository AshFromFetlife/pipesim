"""Preview pipe/socket alignment without breaking existing assembly constraints."""
import copy
import itertools

import numpy as np
from scipy.spatial.transform import Rotation

from .connections import socket_attachment
from .document import Assembly, DocumentError, joint_kind
from .editing import snapshot_design
from .math3d import align_axis, pose_of, transform


def _pose_changes(assembly, poses):
    if assembly.doc.get('state',{}).get('joints'):
        raise DocumentError('Capture the motion frame before moving parts in this pose')
    editable={p['id'] for p in assembly.doc['parts']}
    if set(poses)-editable: raise DocumentError('Expand the reusable object before moving its internal parts')
    posed=copy.deepcopy(assembly)
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
    coordinates={}
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
    return coordinates


def move_document(assembly, poses):
    posed=_pose_changes(assembly,poses)
    coordinates=_movement_coordinates(assembly,posed)
    doc=copy.deepcopy(assembly.doc)
    for spec in doc['parts']:
        if spec['id'] in poses: spec['pose']=pose_of(posed.parts[spec['id']].matrix)
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


def connection_collisions(before, after, relevant, new_pair):
    """Return blocking contacts, using the same mounting allowances as validation."""
    from .validation import CollisionWorld
    pairs={}
    for joint in after.joints:
        pairs.setdefault(frozenset((joint['a']['part'],joint['b']['part'])),[]).append(joint)
    collisions=[]
    with CollisionWorld(after) as world:
        for a,b in itertools.combinations(after.parts,2):
            if not relevant.intersection((a,b)): continue
            pair=pairs.get(frozenset((a,b)),[])
            if pair and after.parts[a].kind==after.parts[b].kind=='human': continue
            if {a,b}!=new_pair:
                old=np.linalg.inv(before.parts[a].matrix)@before.parts[b].matrix
                new=np.linalg.inv(after.parts[a].matrix)@after.parts[b].matrix
                if np.allclose(old,new,atol=1e-5,rtol=0) and all(before.parts[p].shapes==after.parts[p].shapes for p in (a,b)): continue
            contacts=[c for c in world.contacts(a,b) if c[8]*1000 < -1.]
            if not contacts: continue
            if any(j['type']!='socket' and all(np.linalg.norm(np.array(c[5])*1000-after.joint_frames(j)[0])<12 for c in contacts) for j in pair): continue
            collisions.append((a,b,min(contacts,key=lambda c:c[8])))
    return collisions


def connection_options(assembly, member, connector, port, end='start', insertion_mm=None,
                       at_mm=None, locked=True, poses=None, replace_joint=None, force=False, tolerance_mm=2., force_options=None):
    """Return independently checked alternatives; making a preview never edits input."""
    original=assembly
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
    placed=_pose_changes(assembly,poses or {})
    groups=assembly.editor_groups()
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
    baseline_errors={issue_key(i) for i in validate(assembly,collisions=False)['issues'] if i['severity']=='error'}
    from .sliding import sliding_context,slide_connection
    sliding=sliding_context(assembly,connector,member)
    choices=[] if force else [('connector',connector),('member',member)]+([('both',None)] if sliding else [])
    choices.append(('fit',None))
    for side,pid in choices:
        if side=='fit' and not force and any(o['available'] for o in options): continue
        group=next((g for g in groups if pid in g),[])
        label='Solve hinges and slides together' if side=='fit' else 'Slide connected bodies together' if side=='both' else ('Connector' if side=='connector' else 'Pipe')+f' and connected body ({len(group)} parts)'
        option={'move':side,'label':label,'available':False}
        try:
            fitted_joint=joint
            adjusted_document=None
            if side=='fit':
                from .fitting import fit_connection
                if force and (controls['unlock_connectors'] or controls['resize_members']):
                    adjusted=adjust_connection(assembly,joint,placed,controls)
                    final_poses,fitted_joint,motions,context,angle=adjusted['fit']
                    adjusted_document=adjusted['document']
                    option.update(adjustments=adjusted['adjustments'])
                else:
                    final_poses,fitted_joint,motions,context,angle=fit_connection(assembly,joint,placed,force,tolerance_mm=tolerance_mm)
                rotation=Rotation.from_rotvec([np.deg2rad(angle),0,0]).as_matrix()
                description=f'{len(motions)} joints adjusted continuously; rotation snap increments do not limit the fit.'
                if fitted_joint!=joint:
                    value=fitted_joint['b']['at_mm'] if socket.get('through') else fitted_joint['insertion_mm']
                    description+=f' Fitted socket {"centre from pipe start" if socket.get("through") else "insertion"}: {value:.2f} mm.'
                option.update(motions=motions,context_parts=context,requires_preview=True,description=description,
                              label=f'Solve hinges and slides together ({len(final_poses)} parts)',joint=fitted_joint)
            elif side=='both':
                final_poses,slides,context=slide_connection(assembly,joint,placed,sliding);rotation=np.eye(3)
                option.update(slides=slides,context_parts=context,requires_preview=True,
                              label=f'Slide connected bodies together ({len(final_poses)} parts)',
                              description=f'{len(slides)} sliding joints adjusted along their existing axes.')
            else:
                fitted=copy.deepcopy(placed)
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
            result=adjusted_document if adjusted_document is not None else move_document(assembly,final_poses)
            result.setdefault('joints',[]).append(fitted_joint)
            final=Assembly.from_doc(result,assembly.base,assembly.library)
            pa,pb,_=final.joint_frames(fitted_joint);gap=float(np.linalg.norm(pb-pa))
            option['gap_mm']=gap;option['tolerance_mm']=tolerance_mm
            if gap>.03:
                option['requires_preview']=True
                option['description']=option.get('description','')+f' Accepted connection gap: {gap:.2f} mm (allowance {tolerance_mm:g} mm).'
            # Exact attachment and engagement checks precede contact tests.
            report=validate(final,collisions=False)
            relevant=set(final_poses)|{member,connector}
            errors=[i for i in report['issues'] if i['severity']=='error' and issue_key(i) not in baseline_errors and (not i['parts'] or relevant.intersection(i['parts']))]
            if errors: raise DocumentError(errors[0]['message'])
            collisions=connection_collisions(assembly,final,relevant,{member,connector})
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
    recommended=next((o['move'] for o in options if o['available']),None)
    return {'options':options,'recommended':recommended,'joint':joint}
