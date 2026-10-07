"""Draft mirror planes and conversion of their previews into real parts."""

import copy
import hashlib
import math
import re
from pathlib import Path

import numpy as np

from .document import Assembly, DocumentError
from .geometry import mesh_for_part
from .math3d import pose_of, transform


AXES = {'x': 0, 'y': 1, 'z': 2}
LOCAL_REFLECTION = np.diag([-1., 1., 1., 1.])


def reflected_point(point, axis, offset):
    result = list(point)
    index = AXES[axis]
    result[index] = 2*offset-result[index]
    return result


def reflection_matrix(axis, offset):
    matrix = np.eye(4)
    index = AXES[axis]
    matrix[index, index] = -1
    matrix[index, 3] = 2*offset
    return matrix


def mirror_run_conflict(layout, plane, mode):
    """Report a geometric mirror residual without rejecting an editable draft."""
    if mode == 'free':
        return None
    axis = AXES[plane['axis']]
    offset = plane['offset_mm']
    start, end = layout['start'], layout['end']
    if mode == 'centered':
        midpoint_error = abs((start[axis]+end[axis])/2-offset)
        direction_error = abs(abs(end[axis]-start[axis])-np.linalg.norm(end-start))
        residual = max(midpoint_error,direction_error)
        message = (f'Pipe midpoint is {midpoint_error:.2f} mm from the '
                   f'{plane["axis"].upper()} = {offset:g} mm mirror plane' if midpoint_error >= direction_error else
                   f'Pipe direction differs from perpendicular by {direction_error:.2f} mm')
    else:
        residual = max(abs(start[axis]-offset), abs(end[axis]-offset))
        message = (f'Pipe centreline is {residual:.2f} mm from the '
                   f'{plane["axis"].upper()} = {offset:g} mm mirror plane')
    if residual <= .05:
        return None
    return {'code': 'MIRROR_ALIGNMENT', 'residual_mm': round(float(residual), 3),
            'message': message}


def validate_mirrors(assembly, *, geometry=True):
    """Keep plane identities and run-on-plane constraints unambiguous."""
    from .drafting import _layout

    for group in assembly.doc.get('draft_subassemblies', []):
        runs = {run['id']: run for run in group['runs']}
        planes = group.get('mirrors', [])
        if len({plane['id'] for plane in planes}) != len(planes):
            raise DocumentError(f"{group['id']}: duplicate mirror plane id")
        if len({plane['axis'] for plane in planes}) != len(planes):
            raise DocumentError(f"{group['id']}: use at most one mirror plane per axis")
        for plane in planes:
            offset = plane['offset_mm']
            if isinstance(offset, bool) or not isinstance(offset, (int, float)) or not math.isfinite(offset):
                raise DocumentError('Mirror offset must be finite')
            for run_id, mode in plane.get('run_modes', {}).items():
                if run_id not in runs:
                    raise DocumentError(f'{run_id}: mirror constraint refers to an unknown draft run')
                if mode == 'free':
                    continue
                if not geometry:
                    continue
                layout = _layout(assembly, runs[run_id])
                conflict = mirror_run_conflict(layout,plane,mode)
                if conflict:
                    instruction = ('centre the perpendicular pipe on the mirror plane' if mode == 'centered'
                                   else 'place the pipe centreline in the mirror plane')
                    raise DocumentError(f'{run_id}: {instruction}')


def fit_moved_centered_runs(assembly, doc, moved_ids):
    """Let an unlocked, one-ended draft run grow around its mirror plane.

    A connector move changes the effective start/end in ``_layout``. The raw
    draft span is only a working length, so preserve the mirror constraint by
    deriving that length from the moved socket before validating the document.
    """
    from .drafting import _socket

    for group in doc.get('draft_subassemblies', []):
        for run in group['runs']:
            if run.get('locked_length_mm') is not None:
                continue
            ends = [a for a in run.get('attachments', []) if a.get('end') in ('start', 'end')]
            moved = [a for a in ends if a['connector'] in moved_ids]
            if len(ends) == 2 and len(moved) == 1:
                fit_new_centered_end(assembly, group, run, moved[0])
                continue
            if len(ends) != 1 or not moved:
                continue
            attachment = ends[0]
            _, socket, mouth, direction = _socket(assembly, attachment)
            depth = attachment.get('insertion_mm', min(30., socket['engagement_mm']*.8))
            point = mouth-direction*depth
            for plane in group.get('mirrors', []):
                if plane.get('run_modes', {}).get(run['id']) != 'centered':
                    continue
                axis = AXES[plane['axis']]
                # The socket, rather than the saved preview endpoints, defines
                # the effective span. A copied run can have the opposite raw
                # direction from its attached start or end socket.
                component = direction[axis]
                if abs(component) <= 1e-6:
                    continue
                length = 2*(plane['offset_mm']-point[axis])/component
                if length <= 1e-6 or length*(1-abs(component)) > .05:
                    continue
                other = point+direction*length
                start, end = ((point, other) if attachment['end'] == 'start'
                              else (other, point))
                run['start_mm'] = start.tolist()
                run['end_mm'] = end.tolist()


def fit_new_centered_end(assembly, group, run, attachment):
    """Use the new socket's insertion slack to close a centered two-end run."""
    from .drafting import _socket

    ends = {a['end']: a for a in run.get('attachments', []) if a.get('end') in ('start', 'end')}
    if len(ends) != 2 or attachment.get('end') not in ends:
        return
    points = {}
    for name, item in ends.items():
        _, socket, mouth, direction = _socket(assembly, item)
        depth = item.get('insertion_mm', min(30., socket['engagement_mm']*.8))
        points[name] = mouth-direction*depth
    for plane in group.get('mirrors', []):
        if plane.get('run_modes', {}).get(run['id']) != 'centered':
            continue
        axis = AXES[plane['axis']]
        _, socket, _, direction = _socket(assembly, attachment)
        component = direction[axis]
        if abs(component) < .99999:
            continue
        midpoint = (points['start'][axis]+points['end'][axis])/2
        current = attachment.get('insertion_mm', min(30., socket['engagement_mm']*.8))
        depth = current+2*(midpoint-plane['offset_mm'])/component
        if socket.get('min_engagement_mm', 0) <= depth <= socket['engagement_mm']:
            attachment['insertion_mm'] = float(depth)
            points[attachment['end']] = points[attachment['end']] - direction*(depth-current)
            run['start_mm'] = points['start'].tolist()
            run['end_mm'] = points['end'].tolist()


def _unique_id(stem, used):
    candidate = stem
    count = 2
    while candidate in used:
        candidate = f'{stem}-{count}'
        count += 1
    used.add(candidate)
    return candidate


def _mirror_mesh(part, base):
    mesh = mesh_for_part(part)
    mesh.apply_transform(LOCAL_REFLECTION)
    data = mesh.export(file_type='stl')
    filename = f'mirror-{hashlib.sha256(data).hexdigest()[:20]}.stl'
    relative = Path('assets')/'mirrored'/filename
    path = (base/relative).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        path.write_bytes(data)
    return relative.as_posix()


def _mirrored_body(part, base):
    definition = part.definition
    body = {key: copy.deepcopy(definition[key]) for key in
            ('kind', 'material', 'color', 'friction', 'restitution', 'section') if key in definition}
    body['mass_kg'] = part.mass
    body['geometry'] = [{'type': 'mesh', 'file': _mirror_mesh(part, base)}]
    ports = copy.deepcopy(definition.get('ports', {}))
    for port in ports.values():
        if 'position_mm' in port:
            port['position_mm'][0] *= -1
        if 'axis' in port:
            port['axis'][0] *= -1
        if 'rotation_deg' in port:
            matrix = LOCAL_REFLECTION @ transform({'rotation_deg': port['rotation_deg']}) @ LOCAL_REFLECTION
            port['rotation_deg'] = pose_of(matrix)['rotation_deg']
    body['ports'] = ports
    return body


def _mirror_catalog(part, library):
    """Use an explicitly declared counterpart only when its socket frames agree."""
    candidate = part.definition.get('mirror_catalog')
    if isinstance(candidate, dict):
        candidate = candidate.get('x')
    if not isinstance(candidate, str) or candidate not in library.parts:
        return None
    original = part.definition.get('ports', {})
    opposite = library.parts[candidate].get('ports', {})
    if original.keys() != opposite.keys():
        return None
    for name, port in original.items():
        mirrored = opposite[name]
        for field, default in (('position_mm', [0, 0, 0]), ('axis', [0, 0, 1])):
            vector = np.array(port.get(field, default), float)
            vector[0] *= -1
            if not np.allclose(vector, mirrored.get(field, default), atol=1e-6):
                return None
    return candidate


def _source_parts(assembly, group):
    direct = {part['id'] for part in assembly.doc.get('parts', [])}
    selected = set(group.get('mirror_parts', []))
    selected.update(a['connector'] for run in group['runs'] for a in run.get('attachments', []))
    selected &= direct
    changed = True
    while changed:
        changed = False
        for joint in assembly.joints:
            if joint['type'] == 'fixed' or joint.get('locked'):
                a, b = joint['a']['part'], joint['b']['part']
                if a in selected and b in direct and b not in selected:
                    selected.add(b); changed = True
                if b in selected and a in direct and a not in selected:
                    selected.add(a); changed = True
    return selected


def _reflected_socket_on_plane(part, port_name, axis, offset):
    """Find the other real socket of a fitting shared by two mirror copies.

    A fitting whose origin lies on a plane can be shared by both sides only if
    it actually has a distinct socket at the reflection of the used socket.
    Reusing the same bore for two different pipes is physically impossible.
    """
    source = part.ports[port_name]
    mouth, direction = part.frame({'port': port_name})
    reflected_mouth = np.asarray(reflected_point(mouth, axis, offset))
    reflected_direction = direction.copy()
    reflected_direction[AXES[axis]] *= -1
    matches = []
    for name, socket in part.ports.items():
        if name == port_name or socket.get('type') != source.get('type'):
            continue
        if (bool(socket.get('through')) != bool(source.get('through')) or
                socket.get('profile', 'round') != source.get('profile', 'round')):
            continue
        if any(abs(float(socket.get(field, 0))-float(source.get(field, 0))) > .05
               for field in ('diameter_mm', 'engagement_mm', 'min_engagement_mm')):
            continue
        if name in source.get('excludes', ()) or port_name in socket.get('excludes', ()):
            continue
        candidate_mouth, candidate_direction = part.frame({'port': name})
        if (np.linalg.norm(candidate_mouth-reflected_mouth) <= .1 and
                np.linalg.norm(candidate_direction-reflected_direction) <= 1e-5):
            matches.append(name)
    if len(matches) > 1:
        raise DocumentError(f'{part.id}/{port_name}: multiple sockets match its mirror reflection')
    return matches[0] if matches else None


def materialize_mirror(assembly, group_id, plane_id):
    """Bake one draft plane. The returned document has no generated/virtual parts."""
    from .drafting import _layout

    source_group = next((group for group in assembly.doc.get('draft_subassemblies', []) if group['id'] == group_id), None)
    if source_group is None:
        raise DocumentError('Choose an existing draft subassembly')
    plane = next((item for item in source_group.get('mirrors', []) if item['id'] == plane_id), None)
    if plane is None:
        raise DocumentError('Choose an active mirror plane')
    for run in source_group['runs']:
        conflict = mirror_run_conflict(_layout(assembly, run), plane,
            plane.get('run_modes', {}).get(run['id'], 'free'))
        if conflict:
            raise DocumentError(f"{run['id']}: {conflict['message']}")
    axis, offset = plane['axis'], plane['offset_mm']
    mirror = reflection_matrix(axis, offset)
    doc = copy.deepcopy(assembly.doc)
    target = next(group for group in doc['draft_subassemblies'] if group['id'] == group_id)
    used = set(assembly.parts) | {run['id'] for group in doc['draft_subassemblies'] for run in group['runs']}
    used.update(joint['id'] for joint in doc.get('joints', []))
    suffix = re.sub(r'[^A-Za-z0-9_-]', '-', plane_id)
    part_map = {}
    scene_scope = plane.get('scope') == 'scene'
    source_parts = set(assembly.parts) if scene_scope else _source_parts(assembly, source_group)
    # A reference person is one articulated object. Its counterpart to a limb
    # is the other limb of that same person, including when a mirrored fitting
    # has an explicit attachment to a hand or another body part.
    human_parts = {pid for pid, part in assembly.parts.items() if part.kind == 'human'}
    source_parts -= human_parts
    for instance in assembly.doc.get('objects', []):
        symmetry = instance.get('symmetry')
        if instance.get('template') != 'human' or not symmetry or symmetry['axis'] != axis or abs(symmetry['offset_mm'] - offset) > .05:
            continue
        prefix = instance['id'] + '/'
        for pid in human_parts:
            if not pid.startswith(prefix):
                continue
            name = pid[len(prefix):]
            other = ('right_' + name[5:] if name.startswith('left_') else
                     'left_' + name[6:] if name.startswith('right_') else name)
            counterpart = prefix + other
            if counterpart in assembly.parts:
                part_map[pid] = counterpart
    for part_id in sorted(source_parts):
        part = assembly.parts[part_id]
        if abs(part.matrix[AXES[axis], 3]-offset) <= .05:
            part_map[part_id] = part_id
            continue
        mirrored_id = _unique_id(f'{part_id}-mirror-{suffix}', used)
        pose = pose_of(mirror @ part.matrix @ LOCAL_REFLECTION)
        counterpart = _mirror_catalog(part, assembly.library)
        mirrored = {'id': mirrored_id, 'pose': pose, 'label': f'Mirror of {part_id}'}
        if counterpart:
            mirrored['catalog'] = counterpart
            mirrored['parameters'] = copy.deepcopy(part.spec.get('parameters', {}))
        else:
            mirrored['body'] = _mirrored_body(part, assembly.base)
        doc.setdefault('parts', []).append(mirrored)
        part_map[part_id] = mirrored_id
    source_joints = list(assembly.doc.get('joints', []))
    explicit_joint_ids = {joint['id'] for joint in source_joints}
    # Compact chain and library objects generate their internal joints only
    # while Assembly expands them. Their mirrored links become direct parts,
    # so those generated joints must be baked alongside the explicit ones.
    # Humans use articulated left/right counterparts within one object and
    # must not get duplicate internal joints here.
    source_joints.extend(joint for joint in assembly.joints
                         if joint['id'] not in explicit_joint_ids and
                         joint['a']['part'] not in human_parts and
                         joint['b']['part'] not in human_parts)
    for joint in source_joints:
        a, b = joint['a']['part'], joint['b']['part']
        if a not in part_map or b not in part_map or part_map[a] == a and part_map[b] == b:
            continue
        reflected = copy.deepcopy(joint)
        reflected['id'] = _unique_id(f"{joint['id']}-mirror-{suffix}", used)
        for end in ('a', 'b'):
            ref = reflected[end]
            if part_map[ref['part']] != ref['part']:
                ref['part'] = part_map[ref['part']]
                frame = ref.get('frame')
                if frame:
                    for key in ('position_mm', 'axis'):
                        if key in frame: frame[key][0] *= -1
                    if 'rotation_deg' in frame:
                        frame['rotation_deg'] = pose_of(LOCAL_REFLECTION @
                            transform({'rotation_deg': frame['rotation_deg']}) @
                            LOCAL_REFLECTION)['rotation_deg']
            elif 'port' in ref and ref['part'] in assembly.parts:
                other = _reflected_socket_on_plane(assembly.parts[ref['part']], ref['port'], axis, offset)
                if other is None:
                    raise DocumentError(f"{ref['part']}/{ref['port']}: on-plane fitting has no distinct reflected socket")
                ref['port'] = other
        doc.setdefault('joints', []).append(reflected)
    for anchor in assembly.doc.get('anchors', []):
        if anchor['part'] in part_map and part_map[anchor['part']] != anchor['part']:
            mirrored = copy.deepcopy(anchor); mirrored['part'] = part_map[anchor['part']]
            doc.setdefault('anchors', []).append(mirrored)
    copies_by_group = {group['id']: [] for group in doc['draft_subassemblies']}
    source_runs = ((group, run) for group in assembly.doc['draft_subassemblies'] for run in group['runs']) if scene_scope else ((source_group, run) for run in source_group['runs'])
    for owner, run in source_runs:
        owner_target = next(group for group in doc['draft_subassemblies'] if group['id'] == owner['id'])
        layout = _layout(assembly, run)
        start, end = layout['start'], layout['end']
        mirrored_start = np.array(reflected_point(start, axis, offset))
        mirrored_end = np.array(reflected_point(end, axis, offset))
        same = np.linalg.norm(mirrored_start-start) < .05 and np.linalg.norm(mirrored_end-end) < .05
        reversed_ = np.linalg.norm(mirrored_start-end) < .05 and np.linalg.norm(mirrored_end-start) < .05
        attachments = copy.deepcopy(run.get('attachments', []))
        for attachment in attachments:
            original = attachment['connector']
            attachment['connector'] = part_map.get(original, original)
            if not (same or reversed_) and attachment['connector'] == original:
                part = assembly.parts.get(original)
                other = _reflected_socket_on_plane(part, attachment['port'], axis, offset) if part else None
                if other is None:
                    raise DocumentError(f"{run['id']}: {original}/{attachment['port']} sits on the mirror plane without a distinct reflected socket")
                attachment['port'] = other
            if reversed_ and 'end' in attachment:
                attachment['end'] = 'end' if attachment['end'] == 'start' else 'start'
        if same or reversed_:
            existing = next(item for item in owner_target['runs'] if item['id'] == run['id'])
            for attachment in attachments:
                if attachment in existing.get('attachments', []):
                    continue
                if attachment.get('end') and any(a.get('end') == attachment['end'] for a in existing.get('attachments', [])):
                    raise DocumentError(f"{run['id']}: mirrored connector would occupy an already connected pipe end")
                existing.setdefault('attachments', []).append(attachment)
            continue
        clone = copy.deepcopy(run)
        clone['id'] = _unique_id(f"{run['id']}-mirror-{suffix}", used)
        clone['start_mm'] = reflected_point(run['start_mm'], axis, offset)
        clone['end_mm'] = reflected_point(run['end_mm'], axis, offset)
        clone['attachments'] = attachments
        for other in owner_target.get('mirrors', []):
            if other['id'] != plane_id and run['id'] in other.get('run_modes', {}):
                other['run_modes'][clone['id']] = other['run_modes'][run['id']]
        copies_by_group[owner['id']].append(clone)
    for owner_target in doc['draft_subassemblies']:
        owner_target['runs'].extend(copies_by_group[owner_target['id']])
    target['mirrors'] = [item for item in target.get('mirrors', []) if item['id'] != plane_id]
    if target['mirrors']:
        selected = _source_parts(assembly, source_group) if scene_scope else source_parts
        selected -= human_parts
        target['mirror_parts'] = sorted(selected | {part_map[old] for old in selected if part_map.get(old, old) != old})
    else:
        target.pop('mirrors', None); target.pop('mirror_parts', None)
    doc.pop('results', None); doc.pop('build_plan', None)
    Assembly.from_doc(doc, assembly.base, assembly.library,
                      validate_mirror_geometry=False)
    return doc


def materialize_all(assembly, group_ids=None):
    doc = assembly.doc
    for group in list(doc.get('draft_subassemblies', [])):
        if group_ids is not None and group['id'] not in group_ids:
            continue
        for plane in list(group.get('mirrors', [])):
            doc = materialize_mirror(Assembly.from_doc(doc, assembly.base, assembly.library,
                validate_mirror_geometry=False), group['id'], plane['id'])
    return doc
