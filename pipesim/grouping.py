"""Editable articulated objects: grouping changes ownership, never joint rigidity."""
import copy
from functools import lru_cache

import numpy as np

from .document import Assembly, DocumentError
from .math3d import pose_of, transform


@lru_cache(maxsize=1)
def _anatomy():
    from .human import humanoid
    return humanoid(hold_pose=True)


def regroup_candidates(assembly):
    direct={p['id'] for p in assembly.doc['parts']}
    occupied={o['id'] for o in assembly.doc.get('objects',[])}
    candidates={}
    for record in assembly.doc.get('expanded_objects',[]):
        instance=record['instance'];oid=instance['id']
        members=sorted(p for p in direct if p.startswith(oid+'/'))
        if members and oid not in occupied:
            candidates[oid]={'id':oid,'template':instance['template'],'parts':members,'recovered':False}
    # Older saves did not record expansion provenance. Only infer a human from
    # its complete anatomical namespace, not arbitrary neighbouring rigid bodies.
    names={p['id'] for p in _anatomy()['parts']}
    for pid in sorted(direct):
        if not pid.endswith('/pelvis'): continue
        oid=pid[:-len('/pelvis')]
        if oid in candidates or oid in occupied: continue
        if all(oid+'/'+name in direct and assembly.parts[oid+'/'+name].kind=='human' for name in names):
            candidates[oid]={'id':oid,'template':'human','parts':sorted(p for p in direct if p.startswith(oid+'/')),'recovered':True}
    return list(candidates.values())


def _human_parameters(assembly, members, parameters):
    from .human import POSTURE_GROUPS
    result=copy.deepcopy(parameters)
    bodies={pid.rsplit('/',1)[-1]:assembly.parts[pid] for pid in members}
    if 'stature_mm' not in result:
        estimates=[]
        for name,kind,key,ratio in [('head','capsule','radius_mm',.045),('left_shin','capsule','radius_mm',.027),('pelvis','box','size_mm',.11)]:
            for shape in bodies.get(name,()).shapes if name in bodies else []:
                if shape['type']==kind and key in shape:
                    value=shape[key][-1] if key=='size_mm' else shape[key]
                    estimates.append(value/ratio)
        result['stature_mm']=float(np.clip(np.median(estimates) if estimates else 1750,500,2500))
    result['mass_kg']=sum(p.mass for p in bodies.values())
    anatomical={j['id']:j for j in _anatomy()['joints']}
    joints=[j for j in assembly.joints if j['a']['part'] in members and j['b']['part'] in members and j['id'].rsplit('/',1)[-1] in anatomical]
    held={j['id'].rsplit('/',1)[-1] for j in joints if 'motor' in j}
    result['hold_pose']=held==set(anatomical)
    result.pop('hold_joints',None)
    if held and not result['hold_pose']:
        result['hold_joints']=next(([name] for name,group in POSTURE_GROUPS.items() if set(group)==held),sorted(held))
    if 'strength_scale' not in result:
        scales=[j['motor']['max_torque_nm']/anatomical[j['id'].rsplit('/',1)[-1]]['motor']['max_torque_nm']*75/result['mass_kg']
                for j in joints if j.get('motor',{}).get('max_torque_nm') and result['mass_kg']]
        result['strength_scale']=float(np.clip(np.median(scales) if scales else 1,.001,10))
    if 'joint_damping_nms_rad' not in result:
        result['joint_damping_nms_rad']=float(np.median([j.get('damping',.08) for j in joints])) if joints else .08
    grip=bodies.get('left_hand',bodies.get('right_hand'))
    if grip and 'diameter_mm' in grip.ports.get('grip',{}): result['grip_diameter_mm']=grip.ports['grip']['diameter_mm']
    return result


def regroup_object(assembly, object_id, pose=None):
    candidate=next((c for c in regroup_candidates(assembly) if c['id']==object_id),None)
    if candidate is None: raise DocumentError('Select an expanded object, or a complete expanded human, to regroup')
    doc=copy.deepcopy(assembly.doc);reference=copy.deepcopy(doc);reference.pop('state',None)
    neutral=Assembly.from_doc(reference,assembly.base,assembly.library)
    members=set(candidate['parts']);prefix=object_id+'/'
    record=next((r for r in doc.get('expanded_objects',[]) if r['instance']['id']==object_id),None)
    instance=copy.deepcopy(record['instance']) if record else {'id':object_id,'template':candidate['template']}
    parent=transform(instance.get('pose'))
    if record and record['reference_part'] in members:
        parent=neutral.parts[record['reference_part']].matrix@np.linalg.inv(transform(record['reference_pose']))@parent
    elif not record:
        # Recover a useful floor-level origin for legacy humans without altering
        # the actual parts, including any manually edited segment dimensions.
        pelvis=neutral.parts[object_id+'/pelvis'].matrix
        parent=pelvis@np.linalg.inv(transform(_anatomy()['parts'][0]['pose']))
    if pose is not None: parent=transform(pose)
    inverse=np.linalg.inv(parent);parts=[]
    for spec in doc['parts']:
        if spec['id'] not in members: continue
        part=copy.deepcopy(spec);part['id']=part['id'][len(prefix):]
        part['pose']=pose_of(inverse@neutral.parts[spec['id']].matrix);parts.append(part)
    internal=[];external=[]
    for joint in doc.get('joints',[]):
        if joint['id'].startswith(prefix) and all(joint[e]['part'] in members for e in ('a','b')):
            joint=copy.deepcopy(joint);joint['id']=joint['id'][len(prefix):]
            for end in ('a','b'): joint[end]['part']=joint[end]['part'][len(prefix):]
            internal.append(joint)
        else: external.append(joint)
    parameters=instance.get('parameters',{})
    if instance['template']=='human': parameters=_human_parameters(neutral,members,parameters)
    if instance['template']=='chain':
        # Portable bundles rename catalogue entries. Keep the resolved link
        # definition available to the chain generator after regrouping.
        catalog=parts[0].get('catalog')
        if catalog and (catalog.startswith('bundle.') or parameters.get('link_catalog','generic.chain-link') not in neutral.library.parts):
            parameters={**parameters,'link_catalog':catalog}
    instance.update(pose=pose_of(parent),parameters=parameters,
                    components={'parts':parts,'joints':internal,'parameter_reference':copy.deepcopy(parameters)})
    doc['parts']=[p for p in doc['parts'] if p['id'] not in members];doc['joints']=external
    doc.setdefault('objects',[]).insert(min(record.get('index',len(doc['objects'])) if record else len(doc['objects']),len(doc['objects'])),instance)
    # Keep world anchors, external joints and all references at their original
    # full IDs. This includes saved joint state, tracks, drives and fit tests.
    doc['expanded_objects']=[r for r in doc.get('expanded_objects',[]) if r['instance']['id']!=object_id]
    if not doc['expanded_objects']: doc.pop('expanded_objects')
    doc.pop('results',None);doc.pop('build_plan',None)
    return doc


def _scale_shape(shape, scale):
    for key in ('size_mm','position_mm'):
        if key in shape: shape[key]=[v*scale for v in shape[key]]
    for key in ('length_mm','radius_mm','diameter_mm','wall_mm'):
        if key in shape: shape[key]*=scale
    if shape['type']=='mesh': shape['scale']=shape.get('scale',1)*scale
    for child in shape.get('collision_geometry',[]): _scale_shape(child,scale)


def object_components(instance,library=None):
    """Resolve parameter deltas around the saved, edited component definitions."""
    components=copy.deepcopy(instance['components'])
    reference=components.get('parameter_reference',instance.get('parameters',{}))
    parameters=instance.get('parameters',{})
    if parameters==reference: return components
    if instance['template']=='chain':
        from .chain import generate
        return generate(parameters,library,components)
    if instance['template']!='human':
        raise DocumentError('Edit the saved components of this object to change its dimensions')
    from .human import humanoid
    changed={k for k in set(parameters)|set(reference) if parameters.get(k)!=reference.get(k)}
    supported={'mass_kg','stature_mm','strength_scale','hold_pose','hold_joints','joint_damping_nms_rad','grip_diameter_mm'}
    if changed-supported: raise DocumentError('This human has an edited pose. Pose its limbs or expand its parts to edit '+', '.join(sorted(changed-supported)))
    generated=humanoid(**parameters)  # Validate human controls and get defaults for newly held joints.
    size=parameters.get('stature_mm',1750)/reference.get('stature_mm',1750)
    mass=parameters.get('mass_kg',75)/reference.get('mass_kg',75)
    effort=parameters.get('strength_scale',1)/reference.get('strength_scale',1)*mass
    for part in components['parts']:
        if part.get('catalog') and library is not None and changed&{'stature_mm','mass_kg','grip_diameter_mm'}:
            from .document import substitute
            definition=library.parts.get(part['catalog'],{})
            definition=substitute(definition,{**definition.get('parameters',{}),**part.get('parameters',{})})
            # Bundles and imported components may keep dimensions and mass in
            # a catalogue definition. Override those values locally for scaling.
            overlay=part.setdefault('body',{})
            for key in ('geometry','ports','mass_kg','center_of_mass_mm','inertia_kg_m2'):
                if key in definition and key not in overlay: overlay[key]=copy.deepcopy(definition[key])
        if size!=1:
            position=part.setdefault('pose',{}).get('position_mm',[0,0,0]);part['pose']['position_mm']=[v*size for v in position]
            for shape in part.get('body',{}).get('geometry',[]): _scale_shape(shape,size)
            for port in part.get('body',{}).get('ports',{}).values():
                if 'position_mm' in port: port['position_mm']=[v*size for v in port['position_mm']]
        for properties in (part,part.get('body',{})):
            if 'mass_kg' in properties: properties['mass_kg']*=mass
            if 'center_of_mass_mm' in properties: properties['center_of_mass_mm']=[v*size for v in properties['center_of_mass_mm']]
            if 'inertia_kg_m2' in properties: properties['inertia_kg_m2']=[v*mass*size**2 for v in properties['inertia_kg_m2']]
    defaults={j['id']:j for j in generated['joints']}
    for joint in components.get('joints',[]):
        if size!=1:
            for end in ('a','b'):
                frame=joint[end].get('frame',{})
                if 'position_mm' in frame: frame['position_mm']=[v*size for v in frame['position_mm']]
            for key in ('slide_mm',):
                if key in joint.get('limits',{}): joint['limits'][key]=[v*size for v in joint['limits'][key]]
        if joint['id'] not in defaults: continue
        if 'joint_damping_nms_rad' in changed: joint['damping']=parameters.get('joint_damping_nms_rad',.08)
        if changed&{'hold_pose','hold_joints'}:
            if 'motor' not in defaults[joint['id']]: joint.pop('motor',None)
            elif 'motor' not in joint:
                joint['motor']=copy.deepcopy(defaults[joint['id']]['motor']);continue
        if 'max_torque_nm' in joint.get('motor',{}): joint['motor']['max_torque_nm']*=effort
    if 'grip_diameter_mm' in changed:
        hands={p['id']:p for p in generated['parts'] if p['id'].endswith('_hand')}
        for part in components['parts']:
            if part['id'] in hands:
                part['body']['geometry']=hands[part['id']]['body']['geometry']
                part['body'].setdefault('ports',{})['grip']=hands[part['id']]['body']['ports']['grip']
    return components


def restore_objects(original, document, base, library, poses=None):
    """Limb editing may bake parts, but does not discard the user's grouping."""
    present={o['id'] for o in document.get('objects',[])}
    for instance in original.get('objects',[]):
        if instance['id'] not in present:
            assembly=Assembly.from_doc(document,base,library)
            document=regroup_object(assembly,instance['id'],(poses or {}).get(instance['id']))
    return document


def move_object(assembly, object_id, target, *, preview=False):
    from .posing import _editable
    from .snapping import move_document
    instance=next((o for o in assembly.doc.get('objects',[]) if o['id']==object_id),None)
    if instance is None: raise DocumentError('Select a grouped object to move as a whole')
    delta=transform(target)@np.linalg.inv(transform(instance.get('pose')))
    members={pid for pid in assembly.parts if pid.startswith(object_id+'/')}
    poses={pid:pose_of(delta@assembly.parts[pid].matrix) for pid in members
           if not np.allclose(delta@assembly.parts[pid].matrix,assembly.parts[pid].matrix,atol=1e-7,rtol=0)}
    if preview:
        from .snapping import _movement_coordinates
        candidate=copy.copy(assembly);candidate.parts={p:copy.copy(v) for p,v in assembly.parts.items()}
        for pid in poses: candidate.parts[pid].matrix=delta@assembly.parts[pid].matrix
        try: _movement_coordinates(assembly,candidate)
        except DocumentError as exc:
            raise DocumentError(f'{exc}. Detach or loosen the attachment in Connections to structure before moving the whole object.') from exc
        return {'poses':poses,'moved':list(poses),'limited':False,'message':'Whole object moved; limb pose preserved'}
    document=copy.deepcopy(assembly.doc)
    if poses:
        try:
            if instance['template']=='chain' and not assembly.doc.get('state',{}).get('joints'):
                # A whole-chain move only changes its parent pose, keeping long
                # unposed chains compact and avoiding expansion/rebasing work.
                from .snapping import _movement_coordinates
                candidate=copy.copy(assembly);candidate.parts={p:copy.copy(v) for p,v in assembly.parts.items()}
                for pid in poses: candidate.parts[pid].matrix=delta@assembly.parts[pid].matrix
                _movement_coordinates(assembly,candidate)
                next(o for o in document['objects'] if o['id']==object_id)['pose']=copy.deepcopy(target)
                document.pop('results',None);document.pop('build_plan',None)
                return {'document':document,'poses':poses,'moved':list(poses),'limited':False,'message':'Whole chain moved; shape preserved'}
            editable=_editable(assembly,members)
            document=move_document(editable,poses)
        except DocumentError as exc:
            raise DocumentError(f'{exc}. Detach or loosen the attachment in Connections to structure before moving the whole object.') from exc
        document=restore_objects(assembly.doc,document,assembly.base,assembly.library,{object_id:target})
    return {'document':document,'poses':poses,'moved':list(poses),'limited':False,'message':'Whole object moved; limb pose preserved'}


def update_object_parameters(assembly, object_id, parameters):
    from .snapping import _movement_coordinates
    doc=copy.deepcopy(assembly.doc)
    instance=next((o for o in doc.get('objects',[]) if o['id']==object_id),None)
    if instance is None: raise DocumentError('Select an object to edit')
    if instance['template']=='chain':
        from .chain import resize
        doc=resize(assembly,object_id,parameters)
    else: instance['parameters']=copy.deepcopy(parameters)
    after=Assembly.from_doc(doc,assembly.base,assembly.library)
    before=copy.copy(assembly)
    # Internal dimensions may change deliberately; external constraints still
    # have to retain their permitted motion and world anchors remain fixed.
    inside=lambda pid:pid.startswith(object_id+'/')
    before.joints=[j for j in assembly.joints if inside(j['a']['part'])!=inside(j['b']['part'])]
    try: _movement_coordinates(before,after)
    except DocumentError as exc: raise DocumentError(f'{exc}. Detach the affected attachment before changing body dimensions or the initial pose.') from exc
    doc.pop('results',None);doc.pop('build_plan',None)
    return doc


def set_object_layout(assembly, object_id, mode):
    if mode not in ('rigid','posable'): raise DocumentError('Choose whole-chain movement or link posing')
    doc=copy.deepcopy(assembly.doc)
    instance=next((o for o in doc.get('objects',[]) if o['id']==object_id),None)
    if instance is None or instance['template']!='chain': raise DocumentError('Select a grouped chain')
    instance['layout_mode']=mode
    return doc


def detach_attachment(assembly, object_id, joint_id):
    """Remember an external attachment so reconnecting does not require JSON."""
    joint=next((j for j in assembly.doc.get('joints',[]) if j['id']==joint_id),None)
    inside=lambda pid:pid.startswith(object_id+'/')
    if joint is None or inside(joint['a']['part'])==inside(joint['b']['part']):
        raise DocumentError('Choose a connection between this object and the structure')
    doc=copy.deepcopy(assembly.doc)
    saved=doc.setdefault('metadata',{}).setdefault('detached_attachments',[])
    saved[:]=[r for r in saved if r['joint']['id']!=joint_id]
    saved.append({'object':object_id,'joint':copy.deepcopy(joint)})
    doc['joints']=[j for j in doc.get('joints',[]) if j['id']!=joint_id]
    doc.get('state',{}).get('joints',{}).pop(joint_id,None)
    if 'animation' in doc: doc['animation']['tracks']=[t for t in doc['animation'].get('tracks',[]) if t['joint']!=joint_id]
    if 'drives' in doc: doc['drives']=[d for d in doc['drives'] if joint_id not in (d['driver'],d['follower'])]
    doc.pop('results',None);doc.pop('build_plan',None)
    return doc


def release_object_anchor(assembly, object_id, part_id):
    from .editing import expand_objects
    if not part_id.startswith(object_id+'/') or not any(a['part']==part_id for a in assembly.anchors):
        raise DocumentError('Select a world anchor on this object')
    doc=expand_objects(assembly,object_id)
    doc['anchors']=[a for a in doc.get('anchors',[]) if a['part']!=part_id]
    return restore_objects(assembly.doc,doc,assembly.base,assembly.library)


def attach_part(assembly, object_id, part_id=None, target=None, kind='revolute', reconnect=None, part_port=None):
    """Reach an attachment point using the skeleton, then attach in that pose."""
    from .posing import transform_part
    original=assembly
    instance=next((o for o in assembly.doc.get('objects',[]) if o['id']==object_id),None)
    if instance is None: raise DocumentError('Select a grouped object to attach')
    chain=instance['template']=='chain'
    saved=next((r for r in assembly.doc.get('metadata',{}).get('detached_attachments',[])
                if r.get('object')==object_id and r['joint']['id']==reconnect),None) if reconnect else None
    if reconnect and not saved: raise DocumentError('This saved attachment no longer exists')
    if saved:
        joint=copy.deepcopy(saved['joint']);side='a' if joint['a']['part'].startswith(object_id+'/') else 'b'
        source=joint[side];target=joint['b' if side=='a' else 'a'];part_id=source['part'];kind=joint['type']
    else:
        if part_id not in assembly.parts: raise DocumentError('Select the body part to attach')
        part=assembly.parts[part_id]
        if chain:
            source={'part':part_id,'port':part_port or ('b' if part_id==object_id+'/link-1' else 'a')}
            part.local_frame(source)
        else: source={'part':part_id,**({'port':'grip'} if 'grip' in part.ports else {'frame':{'position_mm':[0,0,0]}})}
    if not part_id.startswith(object_id+'/') or target is None or target['part'] not in assembly.parts or target['part'].startswith(object_id+'/'):
        raise DocumentError('Choose a body part and a separate part of the structure')
    if kind not in ('fixed','revolute','spherical'): raise DocumentError('Choose fixed, revolute or spherical for this attachment')
    if chain and (any(a['part'].startswith(object_id+'/') for a in assembly.anchors) or
                  any(j['a']['part'].startswith(object_id+'/')!=j['b']['part'].startswith(object_id+'/') for j in assembly.joints)):
        # Attaching the other end is an explicit request to shape the chain.
        doc=copy.deepcopy(assembly.doc);next(o for o in doc['objects'] if o['id']==object_id)['layout_mode']='posable'
        assembly=Assembly.from_doc(doc,assembly.base,assembly.library)
        instance=next(o for o in doc['objects'] if o['id']==object_id)
    point,axis=assembly.parts[target['part']].frame(target)
    for _ in range(4):
        part=assembly.parts[part_id];current,_=part.frame(source);error=float(np.linalg.norm(current-point))
        if error<.05: break
        desired=part.matrix.copy();desired[:3,3]+=point-current
        moved=None
        if chain and instance.get('layout_mode')=='posable':
            from .chain import pose_chain
            local,_=part.local_frame(source)
            port=next((name for name in ('a','b') if np.allclose(part.local_frame({'port':name})[0],local)),None)
            if port: moved=pose_chain(assembly,instance,part_id,desired,endpoint=port)
        elif chain:
            parent=transform(instance.get('pose'));parent[:3,3]+=point-current
            moved=move_object(assembly,object_id,pose_of(parent))
        if moved is None: moved=transform_part(assembly,part_id,pose_of(desired))
        if not moved['moved']: break
        assembly=Assembly.from_doc(moved['document'],assembly.base,assembly.library)
    part=assembly.parts[part_id];current,_=part.frame(source);error=float(np.linalg.norm(current-point))
    if error>.5:
        action='Move the chain closer, increase its length or pose its links' if chain else 'Move the person closer or pose the torso and limb'
        raise DocumentError(f'The attachment is still {error:.1f} mm out of reach. {action}, then preview again.')
    # Keep the chosen point on the actual target. A hinge axis is expressed in
    # each body's local frame and references the newly posed joint zero.
    local,_=part.local_frame(source)
    source={'part':part_id,'frame':{'position_mm':local.tolist(),'axis':(part.matrix[:3,:3].T@axis).tolist()}}
    if saved:
        joint[side]=source
    else:
        jid=object_id+'-'+part_id.rsplit('/',1)[-1]+'-attachment';base=jid;n=2;existing={j['id'] for j in assembly.joints}
        while jid in existing: jid=base+'-'+str(n);n+=1
        joint={'id':jid,'type':kind,'a':copy.deepcopy(target),'b':source,
               'metadata':{'hardware':'Assumed chain end attachment; choose a hook or shackle and its limits' if chain else 'Assumed body attachment; choose grip strength and limits for the intended task'}}
    doc=copy.deepcopy(assembly.doc)
    if any(j['id']==joint['id'] for j in assembly.joints): raise DocumentError('This attachment is already connected')
    doc.setdefault('joints',[]).append(joint)
    if saved: doc['metadata']['detached_attachments']=[r for r in doc['metadata']['detached_attachments'] if r['joint']['id']!=reconnect]
    final=Assembly.from_doc(doc,assembly.base,assembly.library)
    pa,pb,_=final.joint_frames(joint)
    if np.linalg.norm(pa-pb)>.5: raise DocumentError('Attachment frames do not meet')
    poses={pid:pose_of(p.matrix) for pid,p in final.parts.items() if not np.allclose(p.matrix,original.parts[pid].matrix,atol=1e-7,rtol=0)}
    doc.pop('results',None);doc.pop('build_plan',None)
    return {'document':doc,'poses':poses,'moved':list(poses),'joint':joint,
            'message':'Attachment preview ready; the remaining skeleton and structure connections are preserved'}
