"""Finalize short draft through fits with a spherical chain and scene mirror.

The identifiers and TC161C/TC173MC port dimensions come from the user's ab2
design. The exact later scene was unsaved, so the near-end runs below are a
small reproducible version of the reported attachment topology.
"""
import copy
import os
import random
import secrets

import numpy as np
import pytest

from pipesim.document import Assembly
from pipesim.drafting import _layout, finalize, preview
from pipesim.grouping import attach_part
from pipesim.math3d import axis_frame, pose_of
from pipesim.posing import transform_part
from pipesim.symmetry import materialize_all
from pipesim.validation import validate


def scene(factory, blank, *, start_bound=True, end_bound=True, chain=True,
          mirror=True, locked=False):
    doc = copy.deepcopy(blank)
    doc['parts'] = [
        {'id': 'tc161c-1', 'catalog': 'tubeclamp.TC161C',
         'pose': {'position_mm': [230, 200, 70]}},
        {'id': 'tc173mc-1', 'catalog': 'tubeclamp.TC173MC',
         'pose': {'position_mm': [230, -200, 30]}},
        {'id': 'start-fitting', 'pose': {'position_mm': [230, 200, 0]},
         'body': {'kind': 'connector', 'mass_kg': .2,
                  'geometry': [{'type': 'sphere', 'radius_mm': 10}],
                  'ports': {'socket': {'type': 'socket', 'profile': 'round',
                                       'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                                       'axis': [0, 0, 1], 'engagement_mm': 50,
                                       'min_engagement_mm': 15}}}},
        {'id': 'end-fitting', 'pose': {'position_mm': [230, -200, 100]},
         'body': {'kind': 'connector', 'mass_kg': .2,
                  'geometry': [{'type': 'sphere', 'radius_mm': 10}],
                  'ports': {'socket': {'type': 'socket', 'profile': 'round',
                                       'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                                       'axis': [0, 0, -1], 'engagement_mm': 50,
                                       'min_engagement_mm': 15}}}},
    ]
    runs = []
    if start_bound:
        runs.append({'id': 'tube-c-2', 'catalog': 'tubeclamp.tube-C',
                     'start_mm': [230, 200, -20], 'end_mm': [230, 200, 80],
                     'attachments': [
                         {'connector': 'start-fitting', 'port': 'socket',
                          'end': 'start', 'insertion_mm': 20},
                         {'connector': 'tc161c-1', 'port': 'through'}],
                     **({'locked_length_mm': 100} if locked else {})})
    if end_bound:
        runs.append({'id': 'tube-c-3', 'catalog': 'tubeclamp.tube-C',
                     'start_mm': [230, -200, 20], 'end_mm': [230, -200, 120],
                     'attachments': [
                         {'connector': 'end-fitting', 'port': 'socket',
                          'end': 'end', 'insertion_mm': 20},
                         {'connector': 'tc173mc-1', 'port': 'through'}],
                     **({'locked_length_mm': 100} if locked else {})})
    group = {'id': 'draft-1', 'runs': runs}
    if mirror:
        group['mirrors'] = [{'id': 'mirror-1', 'axis': 'x', 'offset_mm': 0,
                             'scope': 'scene',
                             'run_modes': {run['id']: 'free' for run in runs}}]
    doc['draft_subassemblies'] = [group]
    if locked:
        fixed = ([] if not start_bound else ['start-fitting', 'tc161c-1']) + \
                ([] if not end_bound else ['end-fitting', 'tc173mc-1'])
        doc['anchors'] = [{'part': part, 'surface': 'fixture'} for part in fixed]
    if chain:
        doc['objects'] = [{'id': 'chain-1', 'template': 'chain',
                           'parameters': {'length_mm': 240,
                                          'link_catalog': 'generic.chain-link'},
                           'pose': {'position_mm': [280, -200, 270]}}]
        before = factory(doc)
        doc = attach_part(before, 'chain-1', 'chain-1/link-1',
                          {'part': 'tc173mc-1', 'port': 'hinge'}, 'spherical')['document']
    return factory(doc)


def run_layout(assembly, run_id):
    run = next(run for group in assembly.doc['draft_subassemblies']
               for run in group['runs'] if run['id'] == run_id)
    return _layout(assembly, run)


@pytest.mark.parametrize('side,run_id', [('start', 'tube-c-2'), ('end', 'tube-c-3')])
def test_unlocked_end_and_through_run_extends_at_free_end(factory, blank, side, run_id):
    before = scene(factory, blank, start_bound=side == 'start', end_bound=side == 'end',
                   chain=False, mirror=False)
    conflict = run_layout(before, run_id)['conflicts']
    assert any(item['code'] == 'THROUGH_FIT' and item['engagement_short_mm'] > 12
               for item in conflict)
    result = finalize(before, check_collisions=False)
    assert result['status'] == 'finalized', result
    after = factory(result['document'])
    assert after.parts[run_id].length > 112.5
    report = validate(after, collisions=False)
    assert report['valid'], report['issues']


@pytest.mark.parametrize('side', ('start', 'end'))
def test_locked_end_and_through_run_keeps_declared_length(factory, blank, side):
    before = scene(factory, blank, start_bound=side == 'start', end_bound=side == 'end',
                   chain=False, mirror=False, locked=True)
    result = finalize(before, check_collisions=False)
    assert result['status'] == 'conflict', result
    assert any(item.get('code') == 'THROUGH_FIT' for item in result['conflicts'])
    assert before.doc['draft_subassemblies'][0]['runs'][0]['locked_length_mm'] == 100


def test_two_anchored_end_sockets_do_not_move_to_hide_through_shortfall(factory, blank):
    before = scene(factory, blank, start_bound=True, end_bound=False,
                   chain=False, mirror=False)
    doc = copy.deepcopy(before.doc)
    second = next(part for part in doc['parts'] if part['id'] == 'end-fitting')
    second['pose']['position_mm'] = [230, 200, 60]
    run = doc['draft_subassemblies'][0]['runs'][0]
    run['attachments'].append({'connector': 'end-fitting', 'port': 'socket',
                               'end': 'end', 'insertion_mm': 20})
    doc['anchors'] = [{'part': part, 'surface': 'fixture'}
                      for part in ('start-fitting', 'end-fitting', 'tc161c-1')]
    constrained = factory(doc)
    assert any(c['code'] == 'THROUGH_FIT' and c['engagement_short_mm'] > 12
               for c in run_layout(constrained, 'tube-c-2')['conflicts'])
    result = finalize(constrained, check_collisions=False)
    assert result['status'] == 'conflict', result
    assert any(c.get('code') == 'THROUGH_FIT' for c in result['conflicts'])
    assert constrained.doc == doc


@pytest.mark.parametrize('boundary', ('start', 'end'))
def test_locked_through_socket_accepts_exact_half_engagement_after_pose_roundtrip(factory, blank, boundary):
    seed = int(os.environ.get('PIPESIM_THROUGH_BOUNDARY_SEED') or secrets.randbits(64))
    rng = random.Random(seed)
    cases = int(os.environ.get('PIPESIM_THROUGH_BOUNDARY_CASES', '8'))
    for case in range(cases):
        angles = [rng.uniform(-175, 175) for _ in range(3)]
        position = [rng.uniform(-1500, 1500) for _ in range(3)]
        doc = copy.deepcopy(blank)
        doc['parts'] = [{'id': 'through-fitting',
                         'pose': {'position_mm': position, 'rotation_deg': angles},
                         'body': {'kind': 'connector', 'mass_kg': .2,
                                  'geometry': [{'type': 'sphere', 'radius_mm': 8}],
                                  'ports': {'bore': {'type': 'socket', 'profile': 'round',
                                                     'diameter_mm': 42.4,
                                                     'position_mm': [0, 0, 0], 'axis': [0, 0, 1],
                                                     'through': True, 'engagement_mm': 34,
                                                     'min_engagement_mm': 15}}}}]
        doc['anchors'] = [{'part': 'through-fitting', 'surface': 'fixture'}]
        fitting = factory(doc).parts['through-fitting']
        mouth, axis = fitting.frame({'port': 'bore'})
        if boundary == 'start':
            start, end = mouth - axis * 17, mouth + axis * 83
        else:
            start, end = mouth - axis * 83, mouth + axis * 17
        doc['draft_subassemblies'] = [{'id': 'frame', 'runs': [
            {'id': 'boundary-run', 'catalog': 'tubeclamp.tube-C',
             'start_mm': start.tolist(), 'end_mm': end.tolist(),
             'locked_length_mm': 100,
             'attachments': [{'connector': 'through-fitting', 'port': 'bore'}]}]}]
        try:
            before = factory(doc)
            assert not run_layout(before, 'boundary-run')['conflicts']
            result = finalize(before, check_collisions=False)
            assert result['status'] == 'finalized', result
            after = factory(result['document'])
            report = validate(after, collisions=False)
            assert report['valid'], report['issues']
            assert after.parts['boundary-run'].length == pytest.approx(100, abs=1e-6)
        except Exception as error:
            pytest.fail(f'exact through boundary seed={seed} case={case} side={boundary} '
                        f'angles={angles} position={position}: {error}')


def test_exact_finished_through_joint_accepts_pose_roundtrip_below_micron(factory, blank):
    # The authored station is exactly the 17 mm half-engagement. Serializing
    # the pipe orientation places its physical station only 0.000000213 mm
    # short; this must not become a false THROUGH_ENGAGEMENT error.
    start = np.array([60.89074444245489, -98.86827152956357, 106.60197338913599])
    direction = np.array([0.08182748988309033, -0.8384103818015084, 0.53886203576322])
    length = 287.1061844358846
    mouth = start + direction * 17
    matrix = np.eye(4)
    matrix[:3, :3] = axis_frame(direction)
    matrix[:3, 3] = start + direction * length / 2
    doc = copy.deepcopy(blank)
    doc['parts'] = [
        {'id': 'through-fitting', 'pose': {'position_mm': mouth.tolist()},
         'body': {'kind': 'connector', 'mass_kg': .2,
                  'geometry': [{'type': 'sphere', 'radius_mm': 8}],
                  'ports': {'bore': {'type': 'socket', 'profile': 'round',
                                     'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                                     'axis': direction.tolist(), 'through': True,
                                     'engagement_mm': 34, 'min_engagement_mm': 15}}}},
        {'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
         'parameters': {'length_mm': round(length, 6)}, 'pose': pose_of(matrix)},
    ]
    doc['joints'] = [{'id': 'through-fit', 'type': 'socket', 'locked': True,
                      'a': {'part': 'through-fitting', 'port': 'bore'},
                      'b': {'part': 'pipe', 'at_mm': 17.0}, 'insertion_mm': 0}]
    assembly = factory(doc)
    pipe = assembly.parts['pipe']
    station = float((mouth - pipe.matrix[:3, 3]) @ pipe.matrix[:3, 2] + pipe.length / 2)
    assert station == pytest.approx(16.999999787344294, abs=1e-6)
    report = validate(assembly, collisions=False)
    assert report['valid'], report['issues']


def test_scene_mirror_chain_and_both_near_end_through_runs_finalize(factory, blank):
    before = scene(factory, blank)
    assert len(preview(before)) == 2
    for run_id in ('tube-c-2', 'tube-c-3'):
        assert any(c['code'] == 'THROUGH_FIT' and c['engagement_short_mm'] > 12
                   for c in run_layout(before, run_id)['conflicts'])
    joint = next(j for j in before.joints if j['id'] == 'chain-1-link-1-attachment')
    first, second, _ = before.joint_frames(joint)
    assert np.linalg.norm(first - second) < .05
    result = finalize(before, check_collisions=False)
    assert result['status'] == 'finalized', result
    after = factory(result['document'])
    report = validate(after, collisions=False)
    assert report['valid'], report['issues']
    assert np.linalg.norm(after.joint_frames(joint)[0] - after.joint_frames(joint)[1]) < .05


def test_stale_connector_pose_reproduces_three_mm_chain_gap(factory, blank):
    before = scene(factory, blank)
    stale = copy.deepcopy(before.doc)
    spec = next(part for part in stale['parts'] if part['id'] == 'tc173mc-1')
    spec['pose']['position_mm'][2] += 3.03
    broken = factory(stale)
    joint = next(j for j in broken.joints if j['id'] == 'chain-1-link-1-attachment')
    first, second, _ = broken.joint_frames(joint)
    assert np.linalg.norm(first - second) == pytest.approx(3.03, abs=.01)
    assert any(c['code'] == 'THROUGH_FIT' for c in run_layout(broken, 'tube-c-3')['conflicts'])


def clean_chain_scene(factory, blank):
    source = scene(factory, blank)
    doc = copy.deepcopy(source.doc)
    for run in doc['draft_subassemblies'][0]['runs']:
        if run['id'] == 'tube-c-2':
            run['end_mm'][2] = 100
        else:
            run['start_mm'][2] = 0
    result = factory(doc)
    assert all(not item['conflicts'] for item in preview(result))
    return result


def add_payload(factory, assembly):
    doc = copy.deepcopy(assembly.doc)
    point, _ = assembly.parts['chain-1/link-12'].frame({'port': 'a'})
    doc['parts'].append({'id': 'payload', 'pose': {'position_mm': point.tolist()},
                         'body': {'kind': 'rigid', 'mass_kg': 2,
                                  'geometry': [{'type': 'box', 'size_mm': [20, 20, 20]}]}})
    doc['joints'].append({'id': 'payload-hook', 'type': 'fixed',
                          'a': {'part': 'chain-1/link-12', 'port': 'a'},
                          'b': {'part': 'payload', 'frame': {'position_mm': [0, 0, 0]}}})
    return factory(doc)


def move_host_only(assembly, host_ids):
    doc = copy.deepcopy(assembly.doc)
    for part in doc['parts']:
        if part['id'] in host_ids:
            part['pose']['position_mm'][2] += 3.03
    return Assembly.from_doc(doc, assembly.base, assembly.library)


def test_finalize_closes_legacy_chain_gap_in_original_and_mirror_with_payload(factory, blank, tmp_path):
    before = add_payload(factory, clean_chain_scene(factory, blank))
    source = Assembly.from_doc(before.doc, tmp_path, before.library)
    mirrored = Assembly.from_doc(materialize_all(source), tmp_path, before.library)
    attachments = [joint for joint in mirrored.joints
                   if joint['id'].startswith('chain-1-link-1-attachment')]
    assert len(attachments) == 2  # Original and materialized scene reflection.
    hosts = {joint['a']['part'] for joint in attachments}
    assert len(hosts) == 2
    broken = move_host_only(mirrored, hosts)
    for joint in attachments:
        first, second, _ = broken.joint_frames(joint)
        assert np.linalg.norm(first - second) == pytest.approx(3.03, abs=.01)
    load_ids = [part for part in broken.parts if part.startswith('payload')]
    assert len(load_ids) == 2

    result = finalize(broken, check_collisions=False)
    assert result['status'] == 'finalized', result
    after = Assembly.from_doc(result['document'], broken.base, broken.library)
    for joint in attachments:
        first, second, _ = after.joint_frames(joint)
        assert np.linalg.norm(first - second) < .05
        link = joint['b']['part']
        host = joint['a']['part']
        assert np.allclose(after.parts[link].matrix[:3, 3] - broken.parts[link].matrix[:3, 3],
                           [0, 0, 3.03], atol=.05)
        assert np.allclose(after.parts[host].matrix, broken.parts[host].matrix, atol=.05)
    for load in load_ids:
        assert np.allclose(after.parts[load].matrix[:3, 3] - broken.parts[load].matrix[:3, 3],
                           [0, 0, 3.03], atol=.05)
    report = validate(after, collisions=False)
    assert report['valid'], report['issues']


@pytest.mark.parametrize('boundary', ('anchored', 'cycle'))
def test_finalize_does_not_hide_legacy_chain_gap_across_hard_boundary(factory, blank, boundary):
    before = clean_chain_scene(factory, blank)
    doc = copy.deepcopy(before.doc)
    if boundary == 'anchored':
        doc['anchors'].append({'part': 'chain-1/link-12', 'surface': 'fixture'})
    else:
        tail, _ = before.parts['chain-1/link-12'].frame({'port': 'a'})
        host = before.parts['tc173mc-1']
        local = host.matrix[:3, :3].T @ (tail - host.matrix[:3, 3])
        doc['joints'].append({'id': 'loop-back', 'type': 'fixed',
                              'a': {'part': 'tc173mc-1',
                                    'frame': {'position_mm': local.tolist()}},
                              'b': {'part': 'chain-1/link-12', 'port': 'a'}})
    bounded = factory(doc)
    broken = move_host_only(bounded, {'tc173mc-1'})
    result = finalize(broken, check_collisions=False)
    assert result['status'] == 'conflict', result
    assert any('chain-1' in str(item) or 'loop-back' in str(item)
               for item in result['conflicts'])


def test_connector_first_move_preserves_chain_and_draft_before_finalization(factory, blank):
    before = scene(factory, blank)
    target = before.parts['tc173mc-1'].matrix.copy()
    target[:3, 3] += [0, 0, 3.03]
    moved = transform_part(before, 'tc173mc-1', pose_of(target))
    after = factory(moved['document'])
    assert np.allclose(after.parts['tc173mc-1'].matrix[:3, 3], target[:3, 3], atol=.5)
    joint = next(j for j in after.joints if j['id'] == 'chain-1-link-1-attachment')
    assert np.linalg.norm(after.joint_frames(joint)[0] - after.joint_frames(joint)[1]) < .05
    result = finalize(after, check_collisions=False)
    assert result['status'] == 'finalized', result
    finished = factory(result['document'])
    report = validate(finished, collisions=False)
    assert report['valid'], report['issues']
