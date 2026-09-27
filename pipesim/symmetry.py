"""Draft mirror planes and conversion of their previews into real parts."""

import copy
import hashlib
import math
import re
from pathlib import Path

import numpy as np

from .document import Assembly, DocumentError
from .geometry import mesh_for_part
from .math3d import pose_of


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


def validate_mirrors(assembly):
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
                layout = _layout(assembly, runs[run_id])
                start, end = layout['start'], layout['end']
                axis = AXES[plane['axis']]
                if mode == 'centered':
                    length = np.linalg.norm(end-start)
                    if abs((start[axis]+end[axis])/2-offset) > .05 or abs(abs(end[axis]-start[axis])-length) > .05:
                        raise DocumentError(f'{run_id}: centre the perpendicular pipe on the mirror plane')
                elif abs(start[axis]-offset) > .05 or abs(end[axis]-offset) > .05:
                    raise DocumentError(f'{run_id}: place the pipe centreline in the mirror plane')


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
                sign = 1 if run['end_mm'][axis] >= run['start_mm'][axis] else -1
                # A tiny fitting rotation is acceptable if the resulting pipe
                # still meets the mirror's perpendicularity tolerance.
                expected = sign if attachment['end'] == 'start' else -sign
                component = direction[axis]
                if component*expected <= 0:
                    continue
                length = 2*(plane['offset_mm']-point[axis])/component
                if length <= 1e-6 or length*(1-abs(component)) > .05:
                    continue
                center = point.copy()
                center[axis] = plane['offset_mm']
                start = center.copy(); end = center.copy()
                start[axis] -= sign*length/2
                end[axis] += sign*length/2
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
            matrix = np.eye(4)
            from .math3d import transform
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


def materialize_mirror(assembly, group_id, plane_id):
    """Bake one draft plane. The returned document has no generated/virtual parts."""
    from .drafting import _layout

    source_group = next((group for group in assembly.doc.get('draft_subassemblies', []) if group['id'] == group_id), None)
    if source_group is None:
        raise DocumentError('Choose an existing draft subassembly')
    plane = next((item for item in source_group.get('mirrors', []) if item['id'] == plane_id), None)
    if plane is None:
        raise DocumentError('Choose an active mirror plane')
    axis, offset = plane['axis'], plane['offset_mm']
    mirror = reflection_matrix(axis, offset)
    doc = copy.deepcopy(assembly.doc)
    target = next(group for group in doc['draft_subassemblies'] if group['id'] == group_id)
    used = set(assembly.parts) | {run['id'] for group in doc['draft_subassemblies'] for run in group['runs']}
    used.update(joint['id'] for joint in doc.get('joints', []))
    suffix = re.sub(r'[^A-Za-z0-9_-]', '-', plane_id)
    part_map = {}
    source_parts = _source_parts(assembly, source_group)
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
        doc.setdefault('joints', []).append(reflected)
    for anchor in assembly.doc.get('anchors', []):
        if anchor['part'] in part_map and part_map[anchor['part']] != anchor['part']:
            mirrored = copy.deepcopy(anchor); mirrored['part'] = part_map[anchor['part']]
            doc.setdefault('anchors', []).append(mirrored)
    copies = []
    for run in source_group['runs']:
        layout = _layout(assembly, run)
        start, end = layout['start'], layout['end']
        mirrored_start = np.array(reflected_point(start, axis, offset))
        mirrored_end = np.array(reflected_point(end, axis, offset))
        same = np.linalg.norm(mirrored_start-start) < .05 and np.linalg.norm(mirrored_end-end) < .05
        reversed_ = np.linalg.norm(mirrored_start-end) < .05 and np.linalg.norm(mirrored_end-start) < .05
        attachments = copy.deepcopy(run.get('attachments', []))
        for attachment in attachments:
            attachment['connector'] = part_map.get(attachment['connector'], attachment['connector'])
            if reversed_ and 'end' in attachment:
                attachment['end'] = 'end' if attachment['end'] == 'start' else 'start'
        if same or reversed_:
            existing = next(item for item in target['runs'] if item['id'] == run['id'])
            for attachment in attachments:
                if attachment in existing.get('attachments', []):
                    continue
                if attachment.get('end') and any(a.get('end') == attachment['end'] for a in existing.get('attachments', [])):
                    raise DocumentError(f"{run['id']}: mirrored connector would occupy an already connected pipe end")
                existing.setdefault('attachments', []).append(attachment)
            continue
        for attachment in run.get('attachments', []):
            if part_map.get(attachment['connector']) == attachment['connector']:
                raise DocumentError(f"{run['id']}: {attachment['connector']} sits on the mirror plane; use one centered pipe through the plane or move the connector off it")
        clone = copy.deepcopy(run)
        clone['id'] = _unique_id(f"{run['id']}-mirror-{suffix}", used)
        clone['start_mm'] = reflected_point(run['start_mm'], axis, offset)
        clone['end_mm'] = reflected_point(run['end_mm'], axis, offset)
        clone['attachments'] = attachments
        for other in target.get('mirrors', []):
            if other['id'] != plane_id and run['id'] in other.get('run_modes', {}):
                other['run_modes'][clone['id']] = other['run_modes'][run['id']]
        copies.append(clone)
    target['runs'].extend(copies)
    target['mirrors'] = [item for item in target.get('mirrors', []) if item['id'] != plane_id]
    if target['mirrors']:
        target['mirror_parts'] = sorted(source_parts | {new for old, new in part_map.items() if new != old})
    else:
        target.pop('mirrors', None); target.pop('mirror_parts', None)
    doc.pop('results', None); doc.pop('build_plan', None)
    Assembly.from_doc(doc, assembly.base, assembly.library)
    return doc


def materialize_all(assembly, group_ids=None):
    doc = assembly.doc
    for group in list(doc.get('draft_subassemblies', [])):
        if group_ids is not None and group['id'] not in group_ids:
            continue
        for plane in list(group.get('mirrors', [])):
            doc = materialize_mirror(Assembly.from_doc(doc, assembly.base, assembly.library), group['id'], plane['id'])
    return doc
