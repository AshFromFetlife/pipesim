"""Place a wheel and fit its axle to a part without editing joint JSON."""

import copy
import math

import numpy as np

from .document import Assembly, DocumentError
from .math3d import align_axis, pose_of, transform


def _wheel_catalog(assembly, catalog):
    definition = assembly.library.parts.get(catalog)
    if not definition or definition.get('kind') != 'wheel' or 'axle' not in definition.get('ports', {}):
        raise DocumentError('Choose a wheel with an axle port')
    return definition


def _mount(assembly, wheel_id, target, replace_joint=None):
    if wheel_id not in assembly.parts or assembly.parts[wheel_id].kind != 'wheel':
        raise DocumentError('Choose an existing wheel')
    if not isinstance(target, dict) or target.get('part') not in assembly.parts:
        raise DocumentError('Choose an existing part for the wheel axle')
    if target['part'] == wheel_id:
        raise DocumentError('A wheel cannot mount to itself')
    if ('port' in target) == ('frame' in target):
        raise DocumentError('Choose a mounting port or a point on the part')
    old = next((joint for joint in assembly.joints if joint['id'] == replace_joint), None) if replace_joint else None
    if replace_joint and (old is None or old['type'] != 'revolute' or
            not any(old[end].get('part') == wheel_id and old[end].get('port') == 'axle'
                    for end in ('a', 'b'))):
        raise DocumentError('Choose this wheel’s axle joint to edit')
    if any(wheel_id in (joint['a']['part'], joint['b']['part']) and joint['id'] != replace_joint
           for joint in assembly.joints):
        raise DocumentError('Detach the wheel before moving its axle to another part')
    if 'port' in target:
        port = assembly.parts[target['part']].ports.get(target['port'])
        if not port or port.get('type') == 'socket':
            raise DocumentError('Choose a non-socket mounting port')
        if any(joint['id'] != replace_joint and joint[end].get('part') == target['part'] and
               joint[end].get('port') == target['port'] for joint in assembly.joints for end in ('a', 'b')):
            raise DocumentError('The mounting port is already connected')
    target_point, target_axis = assembly.parts[target['part']].frame(target)
    wheel = assembly.parts[wheel_id]
    axle = wheel.ports.get('axle')
    if not axle:
        raise DocumentError('This wheel has no axle port')
    _, axle_axis = wheel.frame({'part': wheel_id, 'port': 'axle'})
    rotation = align_axis(axle_axis, target_axis) @ wheel.matrix[:3, :3]
    local_point = np.asarray(axle.get('position_mm', [0, 0, 0]), dtype=float)
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = target_point - rotation @ local_point
    doc = copy.deepcopy(assembly.doc)
    spec = next((part for part in doc['parts'] if part['id'] == wheel_id), None)
    if spec is None:
        raise DocumentError('Expand the reusable wheel before mounting it')
    spec['pose'] = pose_of(matrix)
    existing = {joint['id'] for joint in assembly.joints}
    number = 1
    while f'{wheel_id}-axle-{number}' in existing:
        number += 1
    joint = {'id': replace_joint or f'{wheel_id}-axle-{number}', 'type': 'revolute',
             'a': copy.deepcopy(target), 'b': {'part': wheel_id, 'port': 'axle'},
             'damping': 0.005,
             'metadata': {'hardware': 'Specify a compatible axle, bearings and retention hardware'}}
    if old:
        index = next((index for index, item in enumerate(doc.get('joints', []))
                      if item['id'] == replace_joint), None)
        if index is None:
            raise DocumentError('Expand the reusable wheel before editing its axle')
        joint = {**joint, **copy.deepcopy(doc['joints'][index]),
                 'type': 'revolute', 'a': copy.deepcopy(target),
                 'b': {'part': wheel_id, 'port': 'axle'}}
        doc['joints'][index] = joint
    else:
        doc.setdefault('joints', []).append(joint)
    doc.pop('results', None)
    doc.pop('build_plan', None)
    final = Assembly.from_doc(doc, assembly.base, assembly.library)
    a, b, _ = final.joint_frames(joint)
    if np.linalg.norm(a-b) > 1e-5:
        raise DocumentError('Wheel axle did not reach the mounting point')
    return {'document': doc, 'wheel': wheel_id, 'joint': joint}


def mount_wheel(assembly, wheel_id, target, replace_joint=None):
    """Fit an axle or edit its existing mount in one operation."""
    return _mount(assembly, wheel_id, target, replace_joint)


def add_wheel(assembly, catalog='generic.wheel', parameters=None, target=None, pose=None):
    """Add a configured wheel, optionally mounted to an existing part."""
    definition = _wheel_catalog(assembly, catalog)
    params = {**definition.get('parameters', {}), **(parameters or {})}
    for name in ('diameter_mm', 'width_mm', 'mass_kg'):
        value = params.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise DocumentError(f'Wheel {name} must be positive and finite')
    existing = set(assembly.parts)
    number = 1
    while f'wheel-{number}' in existing:
        number += 1
    wheel_id = f'wheel-{number}'
    if pose is None:
        pose = {'position_mm': [0, -600, params['diameter_mm']/2]}
    matrix = transform(pose)
    doc = copy.deepcopy(assembly.doc)
    doc.setdefault('parts', []).append({'id': wheel_id, 'catalog': catalog,
                                        'parameters': params, 'pose': pose_of(matrix)})
    mirrored = [group for group in doc.get('draft_subassemblies', []) if group.get('mirrors')]
    if len(mirrored) == 1 and any(plane.get('scope') != 'scene' for plane in mirrored[0]['mirrors']):
        mirrored[0].setdefault('mirror_parts', []).append(wheel_id)
    doc.pop('results', None)
    doc.pop('build_plan', None)
    added = Assembly.from_doc(doc, assembly.base, assembly.library)
    if target is None:
        return {'document': doc, 'wheel': wheel_id, 'joint': None}
    return _mount(added, wheel_id, target)
