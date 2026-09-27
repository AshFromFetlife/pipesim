"""A reference human constrained to a vertical line in a draft mirror plane."""
from __future__ import annotations

import copy

import numpy as np

from .document import Assembly, DocumentError
from .math3d import pose_of, transform


CENTRAL_JOINTS = {'lumbar_flex', 'thoracic_flex', 'neck_base', 'neck_head'}
CENTRAL_PARTS = {'pelvis', 'lumbar', 'thorax', 'neck', 'head'}
LOCAL_REFLECTION = np.diag([-1., 1., 1., 1.])


def owner(assembly, part_id):
    return next((instance for instance in assembly.doc.get('objects', [])
                 if instance.get('template') == 'human' and instance.get('symmetry')
                 and part_id.startswith(instance['id'] + '/')), None)


def reflection(symmetry):
    matrix = np.eye(4)
    axis = {'x': 0, 'y': 1}[symmetry['axis']]
    matrix[axis, axis] = -1
    matrix[axis, 3] = 2 * symmetry['offset_mm']
    return matrix


def project_object_pose(pose, symmetry):
    result = copy.deepcopy(pose)
    position = list(result.get('position_mm', [0, 0, 0]))
    normal = {'x': 0, 'y': 1}[symmetry['axis']]
    tangent = 1 - normal
    position[normal] = symmetry['offset_mm']
    position[tangent] = symmetry['line_offset_mm']
    result['position_mm'] = position
    result['rotation_deg'] = list(symmetry['rotation_deg'])
    return result


def project_pelvis_target(target, original):
    """The pelvis can slide along the line, without rotating or moving sideways."""
    result = copy.deepcopy(target)
    position = list(result.get('position_mm', [0, 0, 0]))
    position[:2] = original[:2, 3].tolist()
    result['position_mm'] = position
    result['rotation_deg'] = pose_of(original)['rotation_deg']
    return result


def mirror_limb_poses(posed, instance, selected):
    """Reflect the manipulated limb into its anatomical partner."""
    prefix = instance['id'] + '/'
    side = selected[len(prefix):].split('_', 1)[0]
    source_side = side if side in ('left', 'right') else 'left'
    other_side = 'right' if source_side == 'left' else 'left'
    world = reflection(instance['symmetry'])
    for name in ('clavicle', 'upper_arm', 'forearm', 'hand', 'thigh', 'shin', 'foot'):
        source, target = prefix + source_side + '_' + name, prefix + other_side + '_' + name
        if source in posed.parts and target in posed.parts:
            posed.parts[target].matrix = world @ posed.parts[source].matrix @ LOCAL_REFLECTION


def validate_human_symmetry(assembly):
    for instance in assembly.doc.get('objects', []):
        symmetry = instance.get('symmetry')
        if not symmetry:
            continue
        if instance.get('template') != 'human':
            raise DocumentError('Mirror-line symmetry is only available for a reference human')
        if symmetry.get('axis') not in ('x', 'y'):
            raise DocumentError('A human needs a vertical X or Y mirror plane')
        if not all(isinstance(symmetry.get(key), (float, int)) and np.isfinite(symmetry[key])
                   for key in ('offset_mm', 'line_offset_mm')):
            raise DocumentError('The human mirror line needs finite coordinates')
        if not isinstance(symmetry.get('rotation_deg'), list) or len(symmetry['rotation_deg']) != 3:
            raise DocumentError('The human mirror line needs a fixed facing direction')
        expected = project_object_pose(instance.get('pose', {}), symmetry)
        if not np.allclose(transform(expected), transform(instance.get('pose')), atol=1e-5, rtol=0):
            raise DocumentError(f"{instance['id']}: keep the whole person on the mirror line")
        prefix = instance['id'] + '/'
        world = reflection(symmetry)
        for name in CENTRAL_PARTS:
            part = assembly.parts.get(prefix + name)
            if part is not None and not np.allclose(part.matrix, world @ part.matrix @ LOCAL_REFLECTION,
                                                    atol=1e-3, rtol=0):
                raise DocumentError(f"{instance['id']}: {name} must flex within the mirror plane, without twisting")
        for name in ('clavicle', 'upper_arm', 'forearm', 'hand', 'thigh', 'shin', 'foot'):
            left, right = assembly.parts.get(prefix + 'left_' + name), assembly.parts.get(prefix + 'right_' + name)
            if left is not None and right is not None and not np.allclose(
                    right.matrix, world @ left.matrix @ LOCAL_REFLECTION, atol=1e-3, rtol=0):
                raise DocumentError(f"{instance['id']}: left and right {name.replace('_', ' ')} poses must match across the mirror")


def set_human_symmetry(assembly, object_id, group_id=None, plane_id=None):
    instance = next((item for item in assembly.doc.get('objects', []) if item['id'] == object_id), None)
    if instance is None or instance.get('template') != 'human':
        raise DocumentError('Select a reference human')
    if group_id is None and plane_id is None:
        document = copy.deepcopy(assembly.doc)
        next(item for item in document['objects'] if item['id'] == object_id).pop('symmetry', None)
        document.pop('results', None); document.pop('build_plan', None)
        return document
    group = next((item for item in assembly.doc.get('draft_subassemblies', []) if item['id'] == group_id), None)
    plane = next((item for item in group.get('mirrors', []) if item['id'] == plane_id), None) if group else None
    if plane is None:
        raise DocumentError('Choose an existing draft mirror plane')
    axis = plane['axis']
    if axis not in ('x', 'y'):
        raise DocumentError('A human needs a vertical X or Y mirror plane')
    position = instance.get('pose', {}).get('position_mm', [0, 0, 0])
    yaw = instance.get('pose', {}).get('rotation_deg', [0, 0, 0])[2]
    options = (0, 180) if axis == 'x' else (90, -90)
    facing = min(options, key=lambda angle: abs((yaw-angle+180) % 360-180))
    symmetry = {'axis': axis, 'offset_mm': plane['offset_mm'],
                'line_offset_mm': position[1 if axis == 'x' else 0],
                'rotation_deg': [0, 0, facing]}
    target = project_object_pose(instance.get('pose', {}), symmetry)
    from .grouping import move_object
    document = move_object(assembly, object_id, target)['document']
    next(item for item in document['objects'] if item['id'] == object_id)['symmetry'] = symmetry
    try:
        Assembly.from_doc(document, assembly.base, assembly.library)
    except DocumentError as exc:
        raise DocumentError(f'{object_id}: the current posture is not symmetric. Match both sides and straighten torso and neck before fixing the mirror line. {exc}') from exc
    return document
