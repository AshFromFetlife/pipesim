"""Independent copies of parts, immediate neighbours and reusable subassemblies."""
import copy
import math
import re

import numpy as np
from scipy.spatial.transform import Rotation

from .document import Assembly, DocumentError
from .editing import reference_path
from .geometry import bounds
from .math3d import pose_of, transform


def duplicate_members(assembly, selected, scope='part'):
    if selected not in assembly.parts:
        raise DocumentError('Select a part to duplicate')
    if scope == 'part':
        return {selected}
    if scope == 'touching':
        # Connections include loose sockets and articulated joints. Only one hop;
        # incidental mesh overlaps do not imply an attachment.
        members = {selected}
        for joint in assembly.joints:
            ends = {joint['a']['part'], joint['b']['part']}
            if selected in ends:
                members.update(ends)
        return members
    if scope == 'subassembly':
        direct = {p['id'] for p in assembly.doc['parts']}
        owner = next((o for o in assembly.doc.get('objects', [])
                      if selected not in direct and selected.startswith(o['id']+'/')), None)
        if owner:
            # An explicitly grouped object is a reusable articulated unit. Its
            # grips and other attachments to the surrounding structure stay out.
            return {pid for pid in assembly.parts if pid not in direct and pid.startswith(owner['id']+'/')}
        return set(next(group for group in assembly.rigid_groups() if selected in group))
    raise DocumentError('Choose part, touching or subassembly for the duplicate scope')


def _reference_parts(assembly, members, joints):
    """Retain the saved pose when external joints/anchors are left behind.

    Start at the resolved pose and undo only the copied coordinates, in reverse
    order. Reapplying those coordinates to the independent copy then produces
    the same posture, even when an original world anchor reversed which side of
    a joint moved. Joint limits, motors and animation tracks keep their reference.
    """
    parts = {pid: copy.deepcopy(p) for pid, p in assembly.parts.items() if pid in members}
    reference = Assembly({}, assembly.library, parts, joints, assembly.base, [])
    joint_ids = {j['id'] for j in joints}
    for jid, values in reversed(list(assembly.doc.get('state', {}).get('joints', {}).items())):
        if jid not in joint_ids:
            continue
        inverse = {key: -value for key, value in values.items() if key != 'rotation_deg'}
        if 'rotation_deg' in values:
            inverse['rotation_deg'] = Rotation.from_euler('XYZ', values['rotation_deg'], degrees=True).inv().as_euler('XYZ', degrees=True).tolist()
        reference.apply_coordinates({jid: inverse})
    return parts


def _spec(part, matrix, base):
    spec = copy.deepcopy(part.spec)
    if spec.get('catalog'):
        base = part.base
    # Avoid changing human-entered angles when they already describe the pose.
    if not np.allclose(transform(spec.get('pose')), matrix, rtol=0, atol=1e-8):
        spec['pose'] = pose_of(matrix)
    for shape in spec.get('body', {}).get('geometry', []):
        if shape['type'] == 'mesh':
            shape['file'] = reference_path(part.base/shape['file'], base)
    return spec


def _translate(pose, offset):
    pose = copy.deepcopy(pose or {})
    position = pose.setdefault('position_mm', [0, 0, 0])
    position[0] += offset
    return pose


def duplicate(assembly, selected, scope='part', count=1, grid_mm=1):
    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 100:
        raise DocumentError('Number of copies must be a whole number from 1 to 100')
    if isinstance(grid_mm, bool) or not isinstance(grid_mm, (int, float)) or not math.isfinite(grid_mm) or grid_mm <= 0:
        raise DocumentError('Duplicate spacing grid must be a positive number')
    members = duplicate_members(assembly, selected, scope)
    joints = [copy.deepcopy(j) for j in assembly.joints if {j['a']['part'], j['b']['part']} <= members]
    reference = _reference_parts(assembly, members, joints)
    boxes = np.array([bounds(assembly.parts[pid]) for pid in sorted(members)])
    spacing = math.ceil(max(100, float(boxes[:, 1, 0].max()-boxes[:, 0, 0].min())+50)/grid_mm)*grid_mm
    doc = copy.deepcopy(assembly.doc)
    direct = {p['id'] for p in doc['parts']}
    objects = {}
    for instance in doc.get('objects', []):
        owned = {pid for pid in assembly.parts if pid not in direct and pid.startswith(instance['id']+'/')}
        if owned and owned <= members:
            objects[instance['id']] = (instance, owned)
    used = set(assembly.parts) | {j['id'] for j in assembly.joints}
    used.update(o['id'] for o in doc.get('objects', []))
    used.update(r['instance']['id'] for r in doc.get('expanded_objects', []))
    used.update(d['id'] for d in doc.get('drives', []))

    def new_id(original):
        # A single copied limb must not fall back under its original person's
        # namespace in object selection, regrouping or subsequent posing.
        stem = re.sub(r'-copy(?:-\d+)?$', '', original.replace('/', '-'))+'-copy'
        candidate, index = stem, 1
        while any(value == candidate or value.startswith(candidate+'/') for value in used):
            index += 1
            candidate = f'{stem}-{index}'
        used.add(candidate)
        return candidate

    top_joints = {j['id'] for j in doc.get('joints', [])}
    copies = []
    for index in range(count):
        offset = spacing*(index+1)
        object_ids = {oid: new_id(oid) for oid in objects}
        part_ids, joint_ids, embedded_joints = {}, {}, set()
        for oid, (_, owned) in objects.items():
            part_ids.update({pid: object_ids[oid]+pid[len(oid):] for pid in owned})
            for joint in joints:
                if joint['id'] not in top_joints and {joint['a']['part'], joint['b']['part']} <= owned:
                    joint_ids[joint['id']] = object_ids[oid]+joint['id'][len(oid):]
                    embedded_joints.add(joint['id'])
        for pid in reference:
            if pid not in part_ids:
                part_ids[pid] = new_id(pid)
        for joint in joints:
            if joint['id'] not in joint_ids:
                joint_ids[joint['id']] = new_id(joint['id'])
        used.update(part_ids.values())
        used.update(joint_ids.values())

        for oid, (instance, owned) in objects.items():
            result = copy.deepcopy(instance)
            result['id'] = object_ids[oid]
            first = next(pid for pid in reference if pid in owned)
            delta = reference[first].matrix @ np.linalg.inv(transform(reference[first].spec.get('pose')))
            rigid = all(np.allclose(reference[pid].matrix, delta @ transform(reference[pid].spec.get('pose')), atol=1e-7, rtol=0) for pid in owned)
            generated_anchors = instance.get('components', assembly.library.objects.get(instance['template'], {})).get('anchors', [])
            if rigid and not generated_anchors:
                # Keep parameterised templates (and already edited components)
                # when all their parts share a single change of reference frame.
                parent = instance.get('pose', {})
                if not np.allclose(delta, np.eye(4), atol=1e-8, rtol=0):
                    parent = pose_of(delta @ transform(parent))
                result['pose'] = _translate(parent, offset)
            else:
                parent = transform(instance.get('pose'))
                components = {'parts': [], 'joints': [], 'parameter_reference': copy.deepcopy(instance.get('parameters', {}))}
                for pid, part in reference.items():
                    if pid in owned:
                        spec = _spec(part, np.linalg.inv(parent) @ part.matrix, assembly.base)
                        spec['id'] = pid[len(oid)+1:]
                        components['parts'].append(spec)
                for joint in joints:
                    if joint['id'] in embedded_joints and {joint['a']['part'], joint['b']['part']} <= owned:
                        item = copy.deepcopy(joint)
                        item['id'] = item['id'][len(oid)+1:]
                        for end in ('a', 'b'):
                            item[end]['part'] = item[end]['part'][len(oid)+1:]
                        components['joints'].append(item)
                result['components'] = components
                result['pose'] = _translate(instance.get('pose'), offset)
            doc.setdefault('objects', []).append(result)

        owned_parts = set().union(*(owned for _, owned in objects.values()))
        for pid, part in reference.items():
            if pid not in owned_parts:
                spec = _spec(part, part.matrix, part.base if part.spec.get('catalog') else assembly.base)
                spec['id'] = part_ids[pid]
                spec['pose'] = _translate(spec.get('pose'), offset)
                doc['parts'].append(spec)
        for joint in joints:
            if joint['id'] not in embedded_joints:
                item = copy.deepcopy(joint)
                item['id'] = joint_ids[joint['id']]
                for end in ('a', 'b'):
                    item[end]['part'] = part_ids[item[end]['part']]
                doc.setdefault('joints', []).append(item)
        for jid, values in assembly.doc.get('state', {}).get('joints', {}).items():
            if jid in joint_ids:
                doc['state']['joints'][joint_ids[jid]] = copy.deepcopy(values)
        for track in assembly.doc.get('animation', {}).get('tracks', []):
            if track['joint'] in joint_ids:
                doc['animation']['tracks'].append({**copy.deepcopy(track), 'joint': joint_ids[track['joint']]})
        for drive in assembly.doc.get('drives', []):
            if drive['driver'] in joint_ids and drive['follower'] in joint_ids:
                item = {**copy.deepcopy(drive), 'id': new_id(drive['id']), 'driver': joint_ids[drive['driver']], 'follower': joint_ids[drive['follower']]}
                if 'route_mm' in item:
                    item['route_mm'] = [[x+offset, y, z] for x, y, z in item['route_mm']]
                doc['drives'].append(item)
        labels=assembly.doc.get('metadata',{}).get('part_labels',{})
        for old,new in part_ids.items():
            if old in labels: doc.setdefault('metadata',{}).setdefault('part_labels',{})[new]=labels[old]
        for record in assembly.doc.get('metadata',{}).get('body_labels',[]):
            if set(record['parts'])<=part_ids.keys():
                doc.setdefault('metadata',{}).setdefault('body_labels',[]).append({
                    'parts':sorted(part_ids[p] for p in record['parts']),'label':record['label']})
        copies.append({'parts': part_ids, 'joints': joint_ids, 'objects': object_ids, 'offset_mm': [offset, 0, 0]})
    doc.pop('results', None)
    doc.pop('build_plan', None)
    return {'document': doc, 'copies': copies, 'selected': copies[-1]['parts'][selected], 'parts_per_copy': len(members)}
