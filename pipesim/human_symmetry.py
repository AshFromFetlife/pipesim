"""A reference human constrained to a draft mirror plane with paired limb poses."""
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


def rotation_preserves_mirror(pose, symmetry):
    """A whole body stays symmetric when its left-right axis is the plane normal."""
    normal = {'x': 0, 'y': 1}[symmetry['axis']]
    left_right = transform(pose)[:3, 0]
    return abs(abs(left_right[normal])-1) < 1e-5


def project_object_pose(pose, symmetry, reference_pose=None):
    result = copy.deepcopy(pose)
    position = list(result.get('position_mm', [0, 0, 0]))
    normal = {'x': 0, 'y': 1}[symmetry['axis']]
    position[normal] = symmetry['offset_mm']
    result['position_mm'] = position
    result.setdefault('rotation_deg', list(symmetry['rotation_deg']))
    if not rotation_preserves_mirror(result, symmetry):
        fallback = (reference_pose if reference_pose and 'rotation_deg' in reference_pose
                    and rotation_preserves_mirror(reference_pose, symmetry) else symmetry)
        result['rotation_deg'] = list(fallback['rotation_deg'])
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
            raise DocumentError(f"{instance['id']}: keep the whole person in the mirror plane")
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


def _match_limb_pose(assembly, instance, symmetry):
    """Adopt the more deliberately posed side of each limb pair."""
    from .human import humanoid
    from .posing import commit_transform

    reference = humanoid(**instance.get('parameters', {}))
    neutral = {part['id']: transform(part['pose']) for part in reference['parts']}
    parent = transform(instance.get('pose'))
    prefix = instance['id'] + '/'
    world = reflection(symmetry)
    poses = {}
    for names in (('clavicle', 'upper_arm', 'forearm', 'hand'),
                  ('thigh', 'shin', 'foot')):
        def activity(side):
            return sum(np.linalg.norm(assembly.parts[prefix + side + '_' + name].matrix[:3, 3]
                                      - (parent @ neutral[side + '_' + name])[:3, 3])
                       for name in names)
        source = 'left' if activity('left') > activity('right') else 'right'
        target = 'right' if source == 'left' else 'left'
        for name in names:
            part_id = prefix + target + '_' + name
            mirrored = world @ assembly.parts[prefix + source + '_' + name].matrix @ LOCAL_REFLECTION
            if not np.allclose(mirrored, assembly.parts[part_id].matrix, atol=1e-5, rtol=0):
                poses[part_id] = pose_of(mirrored)
    return commit_transform(assembly, poses) if poses else copy.deepcopy(assembly.doc)


def _project_spinal_joints(assembly, instance):
    """Remove sideways spinal motion while retaining flex and joint centres."""
    from .posing import commit_transform

    prefix = instance['id'] + '/'
    original = {pid:part.matrix for pid, part in assembly.parts.items() if pid.startswith(prefix)}
    projected = {prefix + 'pelvis': original[prefix + 'pelvis']}
    children = {}
    for joint in assembly.joints:
        if joint['id'].startswith(prefix) and joint['a']['part'] in original and joint['b']['part'] in original:
            children.setdefault(joint['a']['part'], []).append(joint)
    queue = [prefix + 'pelvis']
    for parent_id in queue:
        parent = projected[parent_id]
        for joint in children.get(parent_id, []):
            child_id = joint['b']['part']
            relative = original[parent_id][:3, :3].T @ original[child_id][:3, :3]
            if joint['id'][len(prefix):] in CENTRAL_JOINTS:
                angle = np.arctan2(relative[2, 1], relative[1, 1])
                c, s = np.cos(angle), np.sin(angle)
                relative = np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
            child = np.eye(4)
            child[:3, :3] = parent[:3, :3] @ relative
            a = np.array(joint['a'].get('frame', {}).get('position_mm', [0, 0, 0]))
            b = np.array(joint['b'].get('frame', {}).get('position_mm', [0, 0, 0]))
            child[:3, 3] = parent[:3, :3] @ a + parent[:3, 3] - child[:3, :3] @ b
            projected[child_id] = child
            queue.append(child_id)
    poses = {pid: pose_of(matrix) for pid, matrix in projected.items()
             if not np.allclose(matrix, original[pid], atol=1e-5, rtol=0)}
    return commit_transform(assembly, poses) if poses else copy.deepcopy(assembly.doc)


def set_human_symmetry(assembly, object_id, group_id=None, plane_id=None, *, adopt_pose=False):
    instance = next((item for item in assembly.doc.get('objects', []) if item['id'] == object_id), None)
    if instance is None or instance.get('template') != 'human':
        raise DocumentError('Select a reference human')
    if group_id is None and plane_id is None:
        document = copy.deepcopy(assembly.doc)
        freed = next(item for item in document['objects'] if item['id'] == object_id)
        freed.pop('symmetry', None)
        freed['mirror_pose_free'] = True
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
                'rotation_deg': list(instance.get('pose', {}).get('rotation_deg', [0, 0, 0]))}
    if not rotation_preserves_mirror(instance.get('pose', {}), symmetry):
        symmetry['rotation_deg'] = [0, 0, facing]
    target = project_object_pose(instance.get('pose', {}), symmetry)
    from .grouping import move_object
    document = move_object(assembly, object_id, target)['document']
    if adopt_pose:
        moved = Assembly.from_doc(document, assembly.base, assembly.library)
        document = _project_spinal_joints(moved, next(item for item in document['objects'] if item['id'] == object_id))
        moved = Assembly.from_doc(document, assembly.base, assembly.library)
        document = _match_limb_pose(moved, next(item for item in document['objects'] if item['id'] == object_id), symmetry)
    constrained = next(item for item in document['objects'] if item['id'] == object_id)
    constrained.pop('mirror_pose_free', None)
    constrained['symmetry'] = symmetry
    try:
        Assembly.from_doc(document, assembly.base, assembly.library)
    except DocumentError as exc:
        raise DocumentError(f'{object_id}: the current posture is not symmetric. Match both sides and straighten torso and neck before fixing the mirror line. {exc}') from exc
    return document


def sync_scene_humans(assembly, *, errors=None):
    """A scene mirror poses a person through its joints, never a second body."""
    planes = [(group['id'], plane['id'])
              for group in assembly.doc.get('draft_subassemblies', [])
              for plane in group.get('mirrors', [])
              if plane.get('scope') == 'scene' and plane['axis'] in ('x', 'y')]
    if not planes:
        return assembly.doc
    document = assembly.doc
    for instance in list(document.get('objects', [])):
        if instance['template'] != 'human' or instance.get('symmetry') or instance.get('mirror_pose_free'):
            continue
        try:
            current = Assembly.from_doc(document, assembly.base, assembly.library)
            document = set_human_symmetry(current, instance['id'], *planes[0], adopt_pose=True)
        except DocumentError as exc:
            message = f"{instance['id']}: could not apply the scene mirror to the joints. {exc}"
            if errors is None:
                raise DocumentError(message) from exc
            errors.append(message)
    return document
