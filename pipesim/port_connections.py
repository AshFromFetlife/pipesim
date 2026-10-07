"""Fit two named attachment ports and join their connected rigid bodies."""

import copy

import numpy as np

from .document import Assembly, DocumentError
from .math3d import align_axis, pose_of
from .snapping import move_document


def connect_ports(assembly, a, b, kind='revolute', move='auto', poses=None,
                  preview=False):
    """Move one rigid body until two ports meet, then author their joint.

    The port locations, rather than rounded bolt-hole mesh coordinates, define
    the exact axis. This also permits a sensible capture from a rough drag.
    """
    if kind not in ('fixed', 'revolute', 'spherical'):
        raise DocumentError('Choose a fixed, hinge or ball joint')
    if move not in ('auto', 'a', 'b'):
        raise DocumentError('Choose which connected body to move')
    if not isinstance(a, dict) or not isinstance(b, dict):
        raise DocumentError('Choose two attachment ports')
    endpoints = (a, b)
    for endpoint in endpoints:
        pid, port = endpoint.get('part'), endpoint.get('port')
        if pid not in assembly.parts or port not in assembly.parts[pid].ports:
            raise DocumentError('Choose existing parts and named attachment ports')
        definition = assembly.parts[pid].ports[port]
        if definition.get('type') == 'socket':
            raise DocumentError('Use Connect to a socket for a pipe socket')
        if any(j[end].get('part') == pid and j[end].get('port') == port
               for j in assembly.joints for end in ('a', 'b')):
            raise DocumentError(f'{pid} / {port} is already connected')
    port_a = assembly.parts[a['part']].ports[a['port']]
    port_b = assembly.parts[b['part']].ports[b['port']]
    hinge = {port_a['type'], port_b['type']} == {'eye', 'clevis'}
    if 'eye' in (port_a['type'], port_b['type']) or 'clevis' in (port_a['type'], port_b['type']):
        if not hinge or port_a.get('assembly') != 'bolt' or port_b.get('assembly') != 'bolt':
            raise DocumentError('A hinge needs one bolt eye and one matching clevis')
    if hinge and abs(port_a.get('diameter_mm', 0)-port_b.get('diameter_mm', 0)) > 1:
        raise DocumentError('The hinge bolt-hole diameters do not match')
    if a['part'] == b['part']:
        raise DocumentError('Choose ports on two different parts')
    groups = assembly.rigid_groups()
    group_for = {pid: set(group) for group in groups for pid in group}
    if group_for[a['part']] == group_for[b['part']]:
        raise DocumentError('These ports are already on the same rigid body')
    if poses:
        assembly = Assembly.from_doc(move_document(assembly, poses),
                                     assembly.base, assembly.library)
    anchors = {anchor['part'] for anchor in assembly.anchors}
    candidates = [key for key in ('a', 'b')
                  if not group_for[(a if key == 'a' else b)['part']] & anchors]
    if move == 'auto':
        if not candidates:
            move = None
        else:
            move = min(candidates, key=lambda key:
                       len(group_for[(a if key == 'a' else b)['part']]))
    elif move not in candidates:
        raise DocumentError('That connected body is fixed to the world')
    before = assembly
    gap_before = float(np.linalg.norm(assembly.parts[a['part']].frame(a)[0]
                                      - assembly.parts[b['part']].frame(b)[0]))
    moved_poses = {}
    if move:
        moving = a if move == 'a' else b
        stationary = b if move == 'a' else a
        source, source_axis = assembly.parts[moving['part']].frame(moving)
        target, target_axis = assembly.parts[stationary['part']].frame(stationary)
        if source_axis @ target_axis < 0:
            target_axis = -target_axis
        rotation = align_axis(source_axis, target_axis)
        delta = np.eye(4)
        delta[:3, :3] = rotation
        delta[:3, 3] = target - rotation @ source
        moved_poses = {pid: pose_of(delta @ assembly.parts[pid].matrix)
                       for pid in group_for[moving['part']]}
        assembly = Assembly.from_doc(move_document(assembly, moved_poses),
                                     assembly.base, assembly.library)
    pa, axis_a = assembly.parts[a['part']].frame(a)
    pb, axis_b = assembly.parts[b['part']].frame(b)
    if np.linalg.norm(pa-pb) > .05 or abs(abs(axis_a @ axis_b)-1) > 1e-5:
        raise DocumentError('The bolt holes could not be aligned without changing existing connections')
    doc = copy.deepcopy(assembly.doc)
    existing = {joint['id'] for joint in assembly.joints}
    number = 1
    while f'joint-{number}' in existing:
        number += 1
    joint = {'id': f'joint-{number}', 'type': kind,
             'a': copy.deepcopy(a), 'b': copy.deepcopy(b),
             'metadata': {'hardware': 'Bolt through matched hinge eye and clevis'
                          if hinge else 'Specify the fastener and its installation method'}}
    doc.setdefault('joints', []).append(joint)
    doc.pop('results', None)
    doc.pop('build_plan', None)
    final = Assembly.from_doc(doc, assembly.base, assembly.library)
    ja, jb, _ = final.joint_frames(joint)
    if np.linalg.norm(ja-jb) > .05:
        raise DocumentError('Attachment frames do not meet')
    changed = {pid: pose_of(final.parts[pid].matrix)
               for pid in final.parts
               if not np.allclose(final.parts[pid].matrix,
                                  before.parts[pid].matrix, atol=1e-7, rtol=0)}
    return {'document': doc, 'joint': joint, 'poses': changed,
            'moved': list(changed), 'move': move, 'gap_mm': gap_before,
            'message': f'Joined {a["part"]} / {a["port"]} to {b["part"]} / {b["port"]}'}
