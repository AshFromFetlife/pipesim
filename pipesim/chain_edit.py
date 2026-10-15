"""Topology edits for compact flexible lines.

Keep the authored object compact while preserving world poses and references to
its individual links.  A cut creates two editable lines; a splice reverses it.
"""
import copy
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .chain import MAX_LINKS, _fit_span, _joint, dimensions, generate
from .document import DocumentError
from .math3d import align_axis, point, pose_of, transform


def _instance(assembly, object_id):
    instance = next((o for o in assembly.doc.get('objects', [])
                     if o['id'] == object_id and o['template'] == 'chain'), None)
    if instance is None:
        raise DocumentError('Select a flexible line')
    return instance


def _baked(assembly, object_id):
    """Commit a displayed animation pose before changing its joint topology."""
    if not assembly.doc.get('state', {}).get('joints'):
        return assembly
    from .document import Assembly
    from .grouping import restore_objects
    from .posing import _editable
    editable = _editable(assembly, {object_id+'/link-1'})
    document = restore_objects(assembly.doc, editable.doc, assembly.base, assembly.library)
    return Assembly.from_doc(document, assembly.base, assembly.library)


def _components(assembly, instance):
    info = dimensions(instance.get('parameters', {}), assembly.library)
    result = copy.deepcopy(instance.get('components') or
                           generate(instance.get('parameters', {}), assembly.library))
    inverse = np.linalg.inv(transform(instance.get('pose')))
    for spec in result['parts']:
        spec['pose'] = pose_of(inverse @ assembly.parts[instance['id']+'/'+spec['id']].matrix)
    result['parameter_reference'] = copy.deepcopy(instance.get('parameters', {}))
    return result, info


def _replace_references(document, part_ids, joint_ids=None):
    """Map all user references, including saved motion, to renamed links."""
    joint_ids = joint_ids or {}
    def endpoint(value):
        if value.get('part') in part_ids:
            value['part'] = part_ids[value['part']]
    for joint in document.get('joints', []):
        endpoint(joint['a']); endpoint(joint['b'])
    for anchor in document.get('anchors', []):
        endpoint(anchor)
    for load in document.get('loads', []):
        endpoint(load)
    for record in document.get('metadata', {}).get('detached_attachments', []):
        endpoint(record['joint']['a']); endpoint(record['joint']['b'])
    for key in ('state',):
        if key in document and 'joints' in document[key]:
            document[key]['joints'] = {joint_ids.get(j, j): value
                                       for j, value in document[key]['joints'].items()
                                       if joint_ids.get(j, j) is not None}
    for track in document.get('animation', {}).get('tracks', []):
        track['joint'] = joint_ids.get(track['joint'], track['joint'])
    if 'animation' in document:
        document['animation']['tracks'] = [t for t in document['animation'].get('tracks', [])
                                           if t['joint'] is not None]
    for drive in document.get('drives', []):
        for key in ('driver', 'follower'):
            drive[key] = joint_ids.get(drive[key], drive[key])
    if 'drives' in document:
        document['drives'] = [d for d in document['drives']
                              if d['driver'] is not None and d['follower'] is not None]


def _finish(document):
    document.pop('results', None)
    document.pop('build_plan', None)
    return document


def _unique_id(document, base):
    used = {o['id'] for o in document.get('objects', [])}
    if base not in used:
        return base
    number = 2
    while f'{base}-{number}' in used:
        number += 1
    return f'{base}-{number}'


def _set_line(instance, parts, info, parameters=None):
    params = copy.deepcopy(parameters or instance.get('parameters', {}))
    params['length_mm'] = len(parts)*info['pitch_mm']
    instance['parameters'] = params
    instance['components'] = {
        'parts': parts,
        'joints': [_joint(i, info['break_force_n']) for i in range(1, len(parts))],
        'parameter_reference': copy.deepcopy(params),
    }
    instance.pop('pose_preset', None)


def cut(assembly, object_id, after_link):
    """Cut after a selected link, retaining both editable pieces and joints."""
    assembly = _baked(assembly, object_id)
    instance = _instance(assembly, object_id)
    components, info = _components(assembly, instance)
    count = info['count']
    if not isinstance(after_link, int) or isinstance(after_link, bool) or not 1 <= after_link < count:
        raise DocumentError('Choose a link before the end of the line')
    document = copy.deepcopy(assembly.doc)
    original = next(o for o in document['objects'] if o['id'] == object_id)
    tail_id = _unique_id(document, object_id+'-cut')
    tail = copy.deepcopy(original)
    tail['id'] = tail_id
    left = copy.deepcopy(components['parts'][:after_link])
    right = copy.deepcopy(components['parts'][after_link:])
    for number, spec in enumerate(right, 1):
        spec['id'] = f'link-{number}'
    _set_line(original, left, info)
    _set_line(tail, right, info)
    position = document['objects'].index(original)
    document['objects'].insert(position+1, tail)
    parts = {f'{object_id}/link-{i}': f'{tail_id}/link-{i-after_link}'
             for i in range(after_link+1, count+1)}
    joints = {f'{object_id}/join-{i}':
              (None if i == after_link else
               f'{tail_id}/join-{i-after_link}' if i > after_link else f'{object_id}/join-{i}')
              for i in range(1, count)}
    _replace_references(document, parts, joints)
    return {'document': _finish(document), 'tail': tail_id,
            'message': f'Cut after link {after_link}; both pieces remain editable.'}


def _line_nodes(parts, info):
    return np.asarray([point(transform(parts[0].get('pose')), info['b'])] +
                      [point(transform(p.get('pose')), info['a']) for p in parts])


def _link_roll(info):
    if info['profile'] != 'chain':
        return np.eye(3)
    return Rotation.from_rotvec((info['a']-info['b'])/info['pitch_mm']*math.pi/2).as_matrix()


def _flip_matrix(info):
    """Local transform that exchanges the two end ports of a link."""
    rotation = align_axis(info['a']-info['b'], info['b']-info['a'])
    flip = np.eye(4)
    flip[:3, :3] = rotation
    flip[:3, 3] = info['a']-rotation@info['b']
    return flip


def _reverse(parts, info):
    flip = _flip_matrix(info)
    result = list(reversed(parts))
    for part in result:
        part['pose'] = pose_of(transform(part.get('pose'))@flip)
    return result


def _reverse_endpoint_frames(document, object_id, info, assembly):
    """Keep external port and custom-frame locations fixed in world space."""
    inverse = np.linalg.inv(_flip_matrix(info))
    prefix = object_id+'/link-'
    def endpoint(ref):
        if not ref.get('part', '').startswith(prefix):
            return
        local, axis = assembly.parts[ref['part']].local_frame(ref)
        for key in ('port', 'frame', 'at_mm', 'end'):
            ref.pop(key, None)
        ref['frame'] = {'position_mm': point(inverse, local).tolist(),
                        'axis': (inverse[:3, :3]@axis).tolist()}
    for joint in document.get('joints', []):
        endpoint(joint['a']); endpoint(joint['b'])
    for record in document.get('metadata', {}).get('detached_attachments', []):
        endpoint(record['joint']['a']); endpoint(record['joint']['b'])
    for load in document.get('loads', []):
        if load.get('part', '').startswith(prefix) and 'point_mm' in load:
            load['point_mm'] = point(inverse, load['point_mm']).tolist()


def splice(assembly, first_id, second_id, first_end='end', second_end='start'):
    """Join any chosen pair of ends on compatible compact lines."""
    assembly = _baked(assembly, first_id)
    if first_end not in ('start', 'end') or second_end not in ('start', 'end'):
        raise DocumentError('Choose the two line ends to join')
    if first_id == second_id:
        raise DocumentError('Choose two different flexible lines')
    first = _instance(assembly, first_id)
    second = _instance(assembly, second_id)
    left, info = _components(assembly, first)
    right, other = _components(assembly, second)
    if info['link_catalog'] != other['link_catalog']:
        raise DocumentError('Choose lines with the same material and section for this splice')
    first_count, second_count = len(left['parts']), len(right['parts'])
    reverse_first = first_end == 'start'
    reverse_second = second_end == 'end'
    if reverse_first:
        left['parts'] = _reverse(left['parts'], info)
    if reverse_second:
        right['parts'] = _reverse(right['parts'], info)
    local_second = np.linalg.inv(transform(first.get('pose'))) @ transform(second.get('pose'))
    for part in right['parts']:
        part['pose'] = pose_of(local_second @ transform(part.get('pose')))
    start = point(transform(left['parts'][-1]['pose']), info['a'])
    end = point(transform(right['parts'][0]['pose']), info['b'])
    gap = np.linalg.norm(end-start)
    # One rigid segment cannot cover a gap shorter than its pitch. Two or
    # more segments can bend around that gap while both old ends stay fixed.
    bridge_count = 0 if gap < .05 else max(2, math.ceil(gap/info['pitch_mm']-1e-9))
    if first_count+second_count+bridge_count > MAX_LINKS:
        raise DocumentError('The joined line would exceed the segment limit')
    bridge = []
    if bridge_count:
        seed = np.linspace(start, end, bridge_count+1)
        nodes = _fit_span(seed, np.full(bridge_count, info['pitch_mm']), start, end)
        previous = transform(left['parts'][-1]['pose'])
        roll = _link_roll(info)
        for index in range(bridge_count):
            matrix = np.eye(4)
            basis = previous[:3, :3]@roll
            matrix[:3, :3] = align_axis(basis@(info['a']-info['b']),
                                        nodes[index+1]-nodes[index])@basis
            matrix[:3, 3] = nodes[index]-matrix[:3, :3]@info['b']
            bridge.append({'id': '', 'catalog': info['link_catalog'], 'pose': pose_of(matrix)})
            previous = matrix
    merged = left['parts']+bridge+right['parts']
    for number, part in enumerate(merged, 1):
        part['id'] = f'link-{number}'
    document = copy.deepcopy(assembly.doc)
    if reverse_first:
        _reverse_endpoint_frames(document, first_id, info, assembly)
    if reverse_second:
        _reverse_endpoint_frames(document, second_id, info, assembly)
    owner = next(o for o in document['objects'] if o['id'] == first_id)
    document['objects'] = [o for o in document['objects'] if o['id'] != second_id]
    _set_line(owner, merged, info)
    parts = {f'{first_id}/link-{i}': f'{first_id}/link-{first_count+1-i}'
             for i in range(1, first_count+1) if reverse_first}
    parts.update({f'{second_id}/link-{i}':
                  f'{first_id}/link-{first_count+bridge_count+(second_count+1-i if reverse_second else i)}'
                  for i in range(1, second_count+1)})
    joints = {f'{first_id}/join-{i}': f'{first_id}/join-{first_count-i}'
              for i in range(1, first_count) if reverse_first}
    joints.update({f'{second_id}/join-{i}':
                   f'{first_id}/join-{first_count+bridge_count+(second_count-i if reverse_second else i)}'
                   for i in range(1, second_count)})
    _replace_references(document, parts, joints)
    return {'document': _finish(document), 'object': first_id,
            'message': f'Joined the lines with {bridge_count} bridging segment(s).'}


def adjust_span(assembly, object_id, link, change):
    """Insert or remove one segment beside a link while attached links stay put."""
    assembly = _baked(assembly, object_id)
    instance = _instance(assembly, object_id)
    components, info = _components(assembly, instance)
    count = info['count']
    if change not in (-1, 1) or not isinstance(link, int) or isinstance(link, bool) or not 1 <= link <= count:
        raise DocumentError('Select a segment to adjust')
    if count+change < 1 or count+change > MAX_LINKS:
        return {'document': copy.deepcopy(assembly.doc), 'changed': False,
                'message': 'This line is at its segment limit.'}
    prefix = object_id+'/link-'
    pinned_links = {int(anchor['part'][len(prefix):]) for anchor in assembly.anchors
                    if anchor['part'].startswith(prefix) and anchor['part'][len(prefix):].isdigit()}
    for joint in assembly.doc.get('joints', []):
        for side in ('a', 'b'):
            pid = joint[side]['part']
            if pid.startswith(prefix) and pid[len(prefix):].isdigit():
                pinned_links.add(int(pid[len(prefix):]))
    if change < 0 and link in pinned_links:
        return {'document': copy.deepcopy(assembly.doc), 'changed': False,
                'message': 'Select a free segment between the connections to tighten this span.'}
    old_parts = components['parts']
    # The selected link is removed, or a new link is inserted immediately after it.
    old_indices = [i for i in range(1, count+1) if change > 0 or i != link]
    if change > 0:
        old_indices.insert(link, None)
    parts = [copy.deepcopy(old_parts[i-1] if i is not None else old_parts[link-1])
             for i in old_indices]
    mapping = {old: new for new, old in enumerate(old_indices, 1) if old is not None}
    for number, spec in enumerate(parts, 1):
        spec['id'] = f'link-{number}'
    old_nodes = _line_nodes(old_parts, info)
    pitch = info['pitch_mm']
    new_count = len(parts)
    pins = {}
    for old in pinned_links:
        new = mapping[old]
        pins[new-1] = old_nodes[old-1].copy()
        pins[new] = old_nodes[old].copy()
    # A free line keeps its beginning at the authored position. A line with
    # supports has only those actual attachments as fixed boundaries.
    if not pins:
        pins[0] = old_nodes[0].copy()
    ordered = sorted(pins)
    for left, right in zip(ordered, ordered[1:]):
        gap = np.linalg.norm(pins[right]-pins[left])
        if (gap > (right-left)*pitch+1e-6 or
                (right-left == 1 and abs(gap-pitch) > 1e-6)):
            return {'document': copy.deepcopy(assembly.doc), 'changed': False,
                    'message': 'This span is already taut at its current connections.'}
    # Resample the existing centreline as a pose seed, then distribute the
    # added or removed pitch only within the attachment-bounded span.
    old_stations = np.arange(count+1, dtype=float)
    new_stations = np.linspace(0, count, new_count+1)
    nodes = np.column_stack([np.interp(new_stations, old_stations, old_nodes[:,axis])
                             for axis in range(3)])
    for left, right in zip(ordered, ordered[1:]):
        if right == left:
            continue
        nodes[left:right+1] = _fit_span(nodes[left:right+1],
                                       np.full(right-left, pitch), pins[left], pins[right])
    first, last = ordered[0], ordered[-1]
    nodes[first] = pins[first]
    nodes[last] = pins[last]
    old_vectors = np.diff(old_nodes, axis=0)
    for i in range(first-1, -1, -1):
        direction = old_vectors[min(i, count-1)]
        nodes[i] = nodes[i+1]-pitch*direction/max(np.linalg.norm(direction), 1e-9)
    for i in range(last, new_count):
        direction = old_vectors[min(i, count-1)]
        nodes[i+1] = nodes[i]+pitch*direction/max(np.linalg.norm(direction), 1e-9)
    for number, spec in enumerate(parts, 1):
        source = transform(old_parts[(old_indices[number-1] or link)-1]['pose'])
        if old_indices[number-1] is None:
            source[:3, :3] = source[:3, :3]@_link_roll(info)
        old_axis = source[:3, :3] @ (info['a']-info['b'])
        matrix = np.eye(4)
        matrix[:3, :3] = align_axis(old_axis, nodes[number]-nodes[number-1]) @ source[:3, :3]
        matrix[:3, 3] = nodes[number-1]-matrix[:3, :3]@info['b']
        spec['pose'] = pose_of(matrix)
    document = copy.deepcopy(assembly.doc)
    owner = next(o for o in document['objects'] if o['id'] == object_id)
    _set_line(owner, parts, info)
    part_ids = {f'{object_id}/link-{old}': f'{object_id}/link-{new}'
                for old, new in mapping.items() if old != new}
    joint_ids = {f'{object_id}/join-{i}':
                 (f'{object_id}/join-{i+change}' if i >= link else f'{object_id}/join-{i}')
                 for i in range(1, count)}
    if change < 0:
        joint_ids[f'{object_id}/join-{link}'] = None
    _replace_references(document, part_ids, joint_ids)
    return {'document': _finish(document), 'changed': True,
            'message': ('Loosened' if change > 0 else 'Tightened')+' this span by one segment.'}


def resize_free_end(assembly, object_id, length_mm, side):
    """Resize the chosen free tail, leaving every retained link in place.

    Return None when the chosen terminal is attached; the constrained resize
    solver can then handle that case.  A free tail must never be resized from
    the opposite end merely because the link IDs grow in that direction.
    """
    assembly = _baked(assembly, object_id)
    instance = _instance(assembly, object_id)
    components, info = _components(assembly, instance)
    parameters = {**instance.get('parameters', {}), 'length_mm': float(length_mm)}
    requested = dimensions(parameters, assembly.library)
    if requested['link_catalog'] != info['link_catalog']:
        return None
    count, target = info['count'], requested['count']
    prefix = object_id+'/link-'
    occupied = {int(ref['part'][len(prefix):])
                for joint in assembly.doc.get('joints', []) for ref in (joint['a'], joint['b'])
                if ref['part'].startswith(prefix) and ref['part'][len(prefix):].isdigit()}
    occupied.update(int(anchor['part'][len(prefix):]) for anchor in assembly.anchors
                    if anchor['part'].startswith(prefix) and anchor['part'][len(prefix):].isdigit())
    if (side == 'start' and 1 in occupied) or (side == 'end' and count in occupied):
        return None
    removed = count-target
    if removed > 0 and ((side == 'start' and any(i <= removed for i in occupied)) or
                        (side == 'end' and any(i > target for i in occupied))):
        return {'document': copy.deepcopy(assembly.doc), 'changed': False,
                'message': 'The free tail already ends at its nearest connection.'}
    old_parts = components['parts']
    if side == 'start':
        retained = old_parts[max(0, removed):]
    else:
        retained = old_parts[:min(target, count)]
    extra = max(0, target-count)
    fresh = []
    if extra:
        previous = transform(old_parts[0 if side == 'start' else -1]['pose'])
        roll = _link_roll(info)
        for _ in range(extra):
            matrix = np.eye(4)
            matrix[:3, :3] = previous[:3, :3]@(roll.T if side == 'start' else roll)
            meeting = point(previous, info['b' if side == 'start' else 'a'])
            matrix[:3, 3] = meeting-matrix[:3, :3]@info['a' if side == 'start' else 'b']
            fresh.append({'id': '', 'catalog': info['link_catalog'], 'pose': pose_of(matrix)})
            previous = matrix
        if side == 'start':
            fresh.reverse()
    parts = fresh+copy.deepcopy(retained) if side == 'start' else copy.deepcopy(retained)+fresh
    for number, part in enumerate(parts, 1):
        part['id'] = f'link-{number}'
    document = copy.deepcopy(assembly.doc)
    owner = next(o for o in document['objects'] if o['id'] == object_id)
    _set_line(owner, parts, info, parameters)
    old_to_new = ({i: i+extra-max(0, removed) for i in range(max(0, removed)+1, count+1)}
                  if side == 'start' else {i:i for i in range(1, min(count, target)+1)})
    part_ids = {f'{object_id}/link-{old}':f'{object_id}/link-{new}'
                for old,new in old_to_new.items() if old != new}
    joint_ids = {f'{object_id}/join-{i}':
                 (f'{object_id}/join-{i+extra-max(0, removed)}'
                  if side == 'start' and i > max(0, removed) else
                  f'{object_id}/join-{i}' if side == 'end' and i < target else None)
                 for i in range(1, count)}
    _replace_references(document, part_ids, joint_ids)
    return {'document': _finish(document), 'changed': True}
