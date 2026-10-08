"""Free-end resizing must preserve connected mirrors and capture reached bores."""
import copy
import os
import random
import secrets
import subprocess
import sys
import time
from pathlib import Path

from fuzz_runtime import batch_size, budget

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.document import Assembly, read
from pipesim.drafting import _layout, finalize, preview, runs
from pipesim.math3d import pose_of, transform
from pipesim.resize_drag import resize_drag
from pipesim.validation import validate


FIXTURE = Path(__file__).parent / 'fixtures' / 'resize-connected-lattice.pipe.yaml'


def effective(assembly, identifier):
    run = next(item for item in runs(assembly.doc) if item['id'] == identifier)
    layout = _layout(assembly, run)
    return layout['start'], layout['end']


def exact_valid(assembly):
    assert all(not item['conflicts'] for item in preview(assembly))
    result = finalize(assembly, check_collisions=False)
    assert result['status'] == 'finalized', result
    finished = Assembly.from_doc(result['document'], assembly.base, assembly.library)
    report = validate(finished, collisions=False)
    assert report['valid'], report['issues']


@pytest.mark.parametrize('target,side,shrink', [
    ('tube-c-1-copy-11', 'end', 20),
    ('tube-c-1-copy-16', 'end', 30),
    ('tube-c-2', 'start', 30),
    ('tube-c-2', 'end', 30),
])
def test_free_end_shrink_is_local_in_real_scene_mirror_graph(library, target, side, shrink):
    original = read(FIXTURE)
    before = Assembly.from_doc(original, FIXTURE.parent, library)
    first, last = effective(before, target)
    old_length = float(np.linalg.norm(last - first))
    requested = old_length - shrink
    direction = (last - first) / old_length

    # Construct a known-valid target independently of the resize tool.
    target_doc = copy.deepcopy(before.doc)
    run = next(item for item in runs(target_doc) if item['id'] == target)
    if side == 'start':
        run['start_mm'] = (last - direction * requested).tolist()
    else:
        raw_start = np.asarray(run['start_mm'])
        run['end_mm'] = (raw_start + direction * requested).tolist()
    target_assembly = Assembly.from_doc(target_doc, before.base, before.library)
    expected_first, expected_last = effective(target_assembly, target)
    assert np.isclose(np.linalg.norm(expected_last - expected_first), requested, atol=.01)
    exact_valid(target_assembly)

    result = resize_drag(before, target, requested, side, auto_connect=False)
    assert result['status'] == 'resized', result
    after = Assembly.from_doc(result['document'], before.base, before.library)
    actual_first, actual_last = effective(after, target)
    assert np.isclose(np.linalg.norm(actual_last - actual_first), requested, atol=.01)
    assert np.allclose(actual_last if side == 'start' else actual_first,
                       last if side == 'start' else first, atol=.01)
    assert after.doc['parts'] == before.doc['parts']
    assert after.doc['joints'] == before.doc['joints']
    assert after.doc['draft_subassemblies'][0]['mirrors'] == \
        before.doc['draft_subassemblies'][0]['mirrors']
    other_before = {item['id']: item for item in runs(before.doc) if item['id'] != target}
    other_after = {item['id']: item for item in runs(after.doc) if item['id'] != target}
    assert other_after == other_before
    assert before.doc == original
    exact_valid(after)


def capture_scene(blank, side, socket_type):
    doc = copy.deepcopy(blank)
    def fitting(identifier, x, *, through, axis):
        port = {'type': 'socket', 'profile': 'round', 'diameter_mm': 42.4,
                'position_mm': [0, 0, 0], 'axis': axis,
                'engagement_mm': 40, 'min_engagement_mm': 15}
        if through:
            port['through'] = True
        return {'id': identifier, 'pose': {'position_mm': [x, 0, 0]},
                'body': {'kind': 'connector', 'mass_kg': .2,
                         'geometry': [{'type': 'sphere', 'radius_mm': 12}],
                         'ports': {'bore': port}}}

    doc['parts'] = [fitting('existing', 500, through=True, axis=[1, 0, 0])]
    if socket_type == 'through':
        location = -100 if side == 'start' else 1100
        doc['parts'].append(fitting('reached', location, through=True, axis=[1, 0, 0]))
    else:
        location = -180 if side == 'start' else 1180
        axis = [1, 0, 0] if side == 'start' else [-1, 0, 0]
        doc['parts'].append(fitting('reached', location, through=False, axis=axis))
    doc['draft_subassemblies'] = [{'id': 'frame', 'runs': [
        {'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
         'start_mm': [0, 0, 0], 'end_mm': [1000, 0, 0],
         'attachments': [{'connector': 'existing', 'port': 'bore'}]}],
        'mirrors': [{'id': 'mirror-1', 'axis': 'x', 'offset_mm': 0,
                     'scope': 'scene', 'run_modes': {'pipe': 'free'}}]}]
    return doc


@pytest.mark.parametrize('side', ('start', 'end'))
@pytest.mark.parametrize('socket_type', ('through', 'end'))
def test_extension_captures_reached_socket_and_finalizes(factory, blank, side, socket_type):
    doc = capture_scene(blank, side, socket_type)
    before = factory(doc)
    assert all(not item['conflicts'] for item in preview(before))
    run = before.doc['draft_subassemblies'][0]['runs'][0]
    assert len(run['attachments']) == 1
    result = resize_drag(before, 'pipe', 1200, side, capture_mm=40, auto_connect=True)
    assert result['status'] == 'resized', result
    after = factory(result['document'])
    updated = after.doc['draft_subassemblies'][0]['runs'][0]
    assert len(updated['attachments']) == 2, result
    reached = next(item for item in updated['attachments'] if item['connector'] == 'reached')
    assert reached['port'] == 'bore'
    if socket_type == 'through':
        assert 'end' not in reached
    else:
        assert reached['end'] == side
        assert 15 <= reached['insertion_mm'] <= 40
    assert after.doc['draft_subassemblies'][0]['mirrors'] == \
        before.doc['draft_subassemblies'][0]['mirrors']
    exact_valid(after)


@pytest.mark.parametrize('side', ('start', 'end'))
def test_partial_through_capture_stays_orange_until_later_extension(factory, blank, side):
    doc = capture_scene(blank, side, 'through')
    reached = next(part for part in doc['parts'] if part['id'] == 'reached')
    x = -35 if side == 'start' else 1035
    reached['pose']['position_mm'] = [x, 0, 0]
    reached['body']['ports']['cross'] = {
        'type': 'socket', 'profile': 'round', 'diameter_mm': 42.4,
        'position_mm': [0, 0, 0], 'axis': [0, 1, 0],
        'through': True, 'engagement_mm': 40, 'min_engagement_mm': 15}
    group = doc['draft_subassemblies'][0]
    group['runs'].append({'id': 'cross-pipe', 'catalog': 'tubeclamp.tube-C',
                          'start_mm': [x, -150, 0], 'end_mm': [x, 150, 0],
                          'attachments': [{'connector': 'reached', 'port': 'cross'}]})
    group['mirrors'][0]['run_modes']['cross-pipe'] = 'free'
    before = factory(doc)
    assert all(not item['conflicts'] for item in preview(before))

    partial = resize_drag(before, 'pipe', 1045, side, capture_mm=40, auto_connect=True)
    first = factory(partial['document'])
    main = next(run for run in runs(first.doc) if run['id'] == 'pipe')
    cross = next(run for run in runs(first.doc) if run['id'] == 'cross-pipe')
    assert {'connector': 'reached', 'port': 'bore'} in main['attachments']
    assert cross == next(run for run in runs(before.doc) if run['id'] == 'cross-pipe')
    shown = {item['id']: item for item in preview(first)}
    assert any(c['code'] == 'THROUGH_FIT' and c['engagement_short_mm'] == pytest.approx(10, abs=.05)
               for c in shown['pipe']['conflicts'])
    assert not shown['cross-pipe']['conflicts']

    completed = resize_drag(first, 'pipe', 1070, side, capture_mm=40, auto_connect=True)
    after = factory(completed['document'])
    main = next(run for run in runs(after.doc) if run['id'] == 'pipe')
    assert len(main['attachments']) == 2
    assert all(not item['conflicts'] for item in preview(after))
    exact_valid(after)


def _generated_capture_scene(blank, seed, index):
    """Make a solvable connected graph with one or two active scene mirrors."""
    rng = random.Random(seed)
    side = ('start', 'end')[index % 2]
    socket_type = ('through', 'end')[(index // 2) % 2]
    mirror_count = 1 + (index // 4) % 2
    doc = capture_scene(blank, side, socket_type)
    reached = next(part for part in doc['parts'] if part['id'] == 'reached')
    reached_x = reached['pose']['position_mm'][0]
    reached['body']['ports']['cross'] = {
        'type': 'socket', 'profile': 'round', 'diameter_mm': 42.4,
        'position_mm': [0, 0, 0], 'axis': [0, 1, 0],
        'through': True, 'engagement_mm': 40, 'min_engagement_mm': 15}
    group = doc['draft_subassemblies'][0]
    group['runs'].append({'id': 'cross-pipe', 'catalog': 'tubeclamp.tube-C',
                          'start_mm': [reached_x, -150, 0],
                          'end_mm': [reached_x, 150, 0],
                          'attachments': [{'connector': 'reached', 'port': 'cross'}]})
    group['mirrors'][0]['run_modes']['cross-pipe'] = 'free'
    for branch_index in range(index % 3):
        identifier = 'existing' if branch_index == 0 else 'middle'
        if identifier == 'middle':
            middle = copy.deepcopy(doc['parts'][0])
            middle['id'] = 'middle'
            middle['pose']['position_mm'] = [rng.uniform(680, 780), 0, 0]
            doc['parts'].append(middle)
            group['runs'][0]['attachments'].append({'connector': 'middle',
                                                    'port': 'bore'})
        host = next(part for part in doc['parts'] if part['id'] == identifier)
        host_x = host['pose']['position_mm'][0]
        host['body']['ports']['cross'] = copy.deepcopy(
            reached['body']['ports']['cross'])
        width = rng.uniform(220, 360)
        branch_id = f'cross-{identifier}'
        group['runs'].append({'id': branch_id, 'catalog': 'tubeclamp.tube-C',
                              'start_mm': [host_x, -width/2, 0],
                              'end_mm': [host_x, width/2, 0],
                              'attachments': [{'connector': identifier,
                                               'port': 'cross'}]})
        group['mirrors'][0]['run_modes'][branch_id] = 'free'

    translation = np.array([rng.uniform(-800, 800), rng.uniform(-800, 800),
                            rng.uniform(200, 900)])
    if mirror_count == 1:
        rotation = Rotation.from_euler('xyz', [rng.uniform(-145, 145)
                                               for _ in range(3)], degrees=True)
    else:
        # A Y rotation preserves the second mirror's in-plane main run and
        # the perpendicular cross run, even after a global translation.
        rotation = Rotation.from_euler('y', rng.uniform(-145, 145), degrees=True)
        group['mirrors'].append({'id': 'mirror-2', 'axis': 'y',
                                 'offset_mm': float(translation[1]),
                                 'scope': 'scene',
                                 'run_modes': {'pipe': 'in_plane', **{
                                     run['id']: 'centered' for run in group['runs']
                                     if run['id'] != 'pipe'}}})
    def world(point):
        return (translation + rotation.apply(np.asarray(point, dtype=float))).tolist()
    for part in doc['parts']:
        matrix = transform(part['pose'])
        matrix[:3, :3] = rotation.as_matrix() @ matrix[:3, :3]
        matrix[:3, 3] = world(matrix[:3, 3])
        part['pose'] = pose_of(matrix)
    for run in group['runs']:
        run['start_mm'] = world(run['start_mm'])
        run['end_mm'] = world(run['end_mm'])
    return doc, side, socket_type, mirror_count, rng.uniform(15, 75)


def _check_generated_capture_shrink(factory, blank, case_seed, index):
    doc, side, socket_type, mirror_count, shrink = _generated_capture_scene(
        blank, case_seed, index)
    context = (f'case_seed={case_seed} index={index} side={side} '
               f'socket={socket_type} mirrors={mirror_count}')
    before = factory(doc)
    assert all(not item['conflicts'] for item in preview(before)), context
    captured = resize_drag(before, 'pipe', 1200, side, capture_mm=40,
                           auto_connect=True)
    assert captured['status'] == 'resized', f'{context}: {captured}'
    joined = factory(captured['document'])
    main = next(run for run in runs(joined.doc) if run['id'] == 'pipe')
    assert any(a['connector'] == 'reached' and a['port'] == 'bore'
               for a in main['attachments']), context
    cross_before = next(run for run in runs(before.doc) if run['id'] == 'cross-pipe')
    assert next(run for run in runs(joined.doc)
                if run['id'] == 'cross-pipe') == cross_before, context
    other_attachments = {run['id']: run['attachments']
                         for run in runs(before.doc) if run['id'] != 'pipe'}
    assert {run['id']: run['attachments'] for run in runs(joined.doc)
            if run['id'] != 'pipe'} == other_attachments, context
    exact_valid(joined)

    # The captured handle is fixed by its end socket if present. Editing the
    # opposite handle is always free, and the independent target must validate.
    free_side = 'end' if side == 'start' else 'start'
    first, last = effective(joined, 'pipe')
    old_length = float(np.linalg.norm(last - first))
    requested = old_length - shrink
    assert requested > 1000, context
    axis = (last - first) / old_length
    oracle_doc = copy.deepcopy(joined.doc)
    oracle_run = next(run for run in runs(oracle_doc) if run['id'] == 'pipe')
    if free_side == 'start':
        oracle_run['start_mm'] = (last - axis * requested).tolist()
    else:
        oracle_run['end_mm'] = (first + axis * requested).tolist()
    oracle = factory(oracle_doc)
    oracle_first, oracle_last = effective(oracle, 'pipe')
    assert np.isclose(np.linalg.norm(oracle_last-oracle_first), requested,
                      atol=.05), context
    exact_valid(oracle)

    shrunk = resize_drag(joined, 'pipe', requested, free_side,
                         auto_connect=False)
    assert shrunk['status'] == 'resized', f'{context}: {shrunk}'
    after = factory(shrunk['document'])
    actual_first, actual_last = effective(after, 'pipe')
    assert np.isclose(np.linalg.norm(actual_last - actual_first), requested,
                      atol=.05), context
    assert np.allclose(actual_first if side == 'start' else actual_last,
                       first if side == 'start' else last, atol=.05), context
    cross_after = next(run for run in runs(after.doc) if run['id'] == 'cross-pipe')
    assert cross_after['attachments'] == cross_before['attachments'], context
    assert {run['id']: run['attachments'] for run in runs(after.doc)
            if run['id'] != 'pipe'} == other_attachments, context
    assert after.doc['draft_subassemblies'][0]['mirrors'] == \
        joined.doc['draft_subassemblies'][0]['mirrors'], context
    assert any(a['connector'] == 'reached' for a in next(
        run for run in runs(after.doc) if run['id'] == 'pipe')['attachments']), context
    exact_valid(after)


def test_generated_mirror_capture_and_shrink_sequences(factory, blank):
    """Fresh regular fuzz covers both socket kinds, handles, and mirror counts."""
    root_seed = int(os.environ.get('PIPESIM_RESIZE_CAPTURE_SEED', secrets.randbits(64)))
    replay = os.environ.get('PIPESIM_RESIZE_CAPTURE_CASE_SEED')
    listed = tuple(int(value) for value in os.environ.get('PIPESIM_RESIZE_CAPTURE_CASE_SEEDS', '').split(',') if value)
    count = len(listed) if listed else 1 if replay else int(os.environ.get('PIPESIM_RESIZE_CAPTURE_CASES', 8))
    rng = random.Random(root_seed)
    print(f'resize capture root_seed={root_seed}', flush=True)
    replay_index = int(os.environ.get('PIPESIM_RESIZE_CAPTURE_CASE_INDEX', 0))
    index_base = int(os.environ.get('PIPESIM_RESIZE_CAPTURE_CASE_INDEX_BASE', 0))
    for index in range(count):
        case_seed = listed[index] if listed else int(replay) if replay else rng.getrandbits(64)
        _check_generated_capture_shrink(factory, blank, case_seed,
                                        replay_index if replay else index_base+index)


def test_long_generated_mirror_capture_and_shrink_sequences(factory, blank, tmp_path):
    root_seed = int(os.environ.get('PIPESIM_RESIZE_CAPTURE_SEED', secrets.randbits(64)))
    rng = random.Random(root_seed)
    minutes, started, deadline = budget()
    cases = int(os.environ.get('PIPESIM_RESIZE_CAPTURE_CASES', 1000000))
    print(f'long resize capture root_seed={root_seed}', flush=True)
    completed = 0
    try:
        while completed < cases and time.monotonic() < deadline:
            batch = [rng.getrandbits(64) for _ in range(batch_size(
                10, cases-completed, completed, started, deadline, minutes))]
            batch_dir = tmp_path / f'batch-{completed:06d}'
            child_env = os.environ.copy()
            child_env['PIPESIM_RESIZE_CAPTURE_CASE_SEEDS'] = ','.join(map(str, batch))
            child_env['PIPESIM_RESIZE_CAPTURE_CASE_INDEX_BASE'] = str(completed)
            child = subprocess.run(
                [sys.executable, '-m', 'pytest', '-q', '-s', f'--basetemp={batch_dir}',
                 'tests/test_resize_capture_mirror.py', '-k', 'test_generated_mirror_capture_and_shrink_sequences'],
                env=child_env, capture_output=True, text=True, check=False)
            if child.returncode:
                pytest.fail(f'Resize capture root_seed={root_seed}, completed={completed}, '
                            f'batch_seeds={batch}, repro_dir={batch_dir}\n'
                            f'{child.stdout}\n{child.stderr}', pytrace=False)
            completed += len(batch)
            print(f'long resize capture completed_models={completed} '
                  f'elapsed_s={time.monotonic()-started:.0f} last_case_seed={batch[-1]}', flush=True)
    finally:
        print(f'long resize capture completed_models={completed}', flush=True)
