"""Endpoint resize regressions from a connected, scene-mirrored draft design.

The fixture retains the 23-part connected component of a saved design where
three visually free endpoint drags failed on unrelated through and mirror
constraints. It is self-contained and has no initially conflicting drafts.
"""

import copy
import json
import os
from pathlib import Path
import random
import secrets
import subprocess
import sys
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.document import Assembly, DocumentError, read
from pipesim.drafting import _layout, finalize, preview, runs
from pipesim.resize_drag import resize_drag
from pipesim.validation import validate


FIXTURE = Path(__file__).parent / 'fixtures' / 'resize-connected-lattice.pipe.yaml'
TARGETS = ('tube-c-1-copy-11', 'tube-c-2', 'tube-c-1-copy-16')
FREE_END_CASES = (('tube-c-1-copy-11', 'end'),
                  ('tube-c-2', 'start'), ('tube-c-2', 'end'),
                  ('tube-c-1-copy-16', 'start'), ('tube-c-1-copy-16', 'end'))
RANDOM_SEEDS = tuple(int(value) for value in os.environ.get(
    'PIPESIM_RESIZE_REALWORLD_SEEDS',
    ','.join(map(str, range(12))) + ',' + ','.join(str(secrets.randbits(64)) for _ in range(4))
).split(',') if value)
GRAPH_SEEDS = tuple(int(value) for value in os.environ.get(
    'PIPESIM_RESIZE_GRAPH_SEEDS',
    ','.join(map(str, range(16))) + ',' + ','.join(str(secrets.randbits(64)) for _ in range(4))
).split(',') if value)


@pytest.fixture(scope='module')
def connected_design(library):
    return Assembly.from_doc(read(FIXTURE), FIXTURE.parent, library)


def _exactly_valid(assembly):
    result = finalize(assembly, check_collisions=False)
    assert result['status'] == 'finalized', result
    exact = Assembly.from_doc(result['document'], assembly.base, assembly.library)
    report = validate(exact, collisions=False)
    assert report['valid'], report['issues']


def _effective_frame(assembly, identifier):
    run = next(run for run in runs(assembly.doc) if run['id'] == identifier)
    layout = _layout(assembly, run)
    return layout['start'], layout['end']


def _known_free_space_target(before, target, new_length, side):
    """Construct the valid local edit before asking the resize tool to find it."""
    document = copy.deepcopy(before.doc)
    run = next(run for run in runs(document) if run['id'] == target)
    first, last = _effective_frame(before, target)
    direction = (last-first)/np.linalg.norm(last-first)
    if side == 'start':
        run['start_mm'] = (last-direction*new_length).tolist()
    else:
        # An end socket supplies the effective start, but the saved preview
        # span still supplies its working length.
        raw_start = np.asarray(run['start_mm'])
        run['end_mm'] = (raw_start+direction*new_length).tolist()
    target_assembly = Assembly.from_doc(document, before.base, before.library)
    expected_first, expected_last = _effective_frame(target_assembly, target)
    assert np.isclose(np.linalg.norm(expected_last-expected_first), new_length, atol=.01)
    assert all(not item['conflicts'] for item in preview(target_assembly))
    _exactly_valid(target_assembly)
    return target_assembly


def _assert_graph_intact(before, after):
    assert {part['id'] for part in after.doc['parts']} == {part['id'] for part in before.doc['parts']}
    assert {run['id'] for run in runs(after.doc)} == {run['id'] for run in runs(before.doc)}
    before_runs = {run['id']: run for run in runs(before.doc)}
    after_runs = {run['id']: run for run in runs(after.doc)}
    for identifier, run in before_runs.items():
        assert after_runs[identifier]['attachments'] == run['attachments']
        assert after_runs[identifier]['catalog'] == run['catalog']
    assert after.doc['joints'] == before.doc['joints']
    assert after.doc['draft_subassemblies'][0].get('mirrors', []) == \
        before.doc['draft_subassemblies'][0].get('mirrors', [])
    assert all(not item['conflicts'] for item in preview(after))


def test_connected_lattice_fixture_is_clean_and_exactly_valid(connected_design):
    assert len(connected_design.doc['parts']) == 23
    assert len(runs(connected_design.doc)) == 15
    assert all(not item['conflicts'] for item in preview(connected_design))
    _exactly_valid(connected_design)


@pytest.mark.parametrize('target,side', FREE_END_CASES)
def test_free_endpoint_resize_preserves_connected_graph_and_mirror(connected_design, target, side):
    before = connected_design
    original = copy.deepcopy(before.doc)
    first, last = _effective_frame(before, target)
    old_length = float(np.linalg.norm(last-first))
    _known_free_space_target(before, target, old_length+100, side)

    result = resize_drag(before, target, old_length+100, side, auto_connect=False)
    assert result['status'] == 'resized', result
    after = Assembly.from_doc(result['document'], before.base, before.library)
    changed_first, changed_last = _effective_frame(after, target)
    assert np.isclose(np.linalg.norm(changed_last-changed_first), old_length+100, atol=.01)
    fixed = last if side == 'start' else first
    changed_fixed = changed_last if side == 'start' else changed_first
    assert np.allclose(changed_fixed, fixed, atol=.01)
    _assert_graph_intact(before, after)
    assert before.doc == original
    _exactly_valid(after)


@pytest.mark.parametrize('case_seed', RANDOM_SEEDS)
def test_random_free_endpoint_resize_keeps_exact_design_valid(connected_design, case_seed):
    rng = random.Random(case_seed)
    target, side = rng.choice(FREE_END_CASES)
    delta = rng.uniform(20, 160)
    first, last = _effective_frame(connected_design, target)
    old_length = float(np.linalg.norm(last-first))
    try:
        _known_free_space_target(connected_design, target, old_length+delta, side)
        result = resize_drag(connected_design, target, old_length+delta, side,
                             auto_connect=False)
        after = Assembly.from_doc(result['document'], connected_design.base,
                                  connected_design.library)
        changed_first, changed_last = _effective_frame(after, target)
        assert np.isclose(np.linalg.norm(changed_last-changed_first), old_length+delta, atol=.01)
        _assert_graph_intact(connected_design, after)
        _exactly_valid(after)
    except Exception as error:
        pytest.fail(f'real-world resize case_seed={case_seed} target={target} '
                    f'side={side} delta_mm={delta:.6f}: {error}')


LATTICE_SEQUENCE_SEEDS = tuple(range(6)) + tuple(secrets.randbits(64) for _ in range(2))


@pytest.mark.parametrize('case_seed', LATTICE_SEQUENCE_SEEDS)
def test_connected_lattice_mixed_end_edits_stay_solvable(connected_design, case_seed, tmp_path):
    """Repeated small, independently valid drags on the real graph must work."""
    rng = random.Random(case_seed)
    current = connected_design
    operations = []
    for step in range(3):
        choices = list(FREE_END_CASES)
        rng.shuffle(choices)
        found = None
        for target, side in choices:
            first, last = _effective_frame(current, target)
            old_length = float(np.linalg.norm(last-first))
            for change in (rng.uniform(20, 70), -rng.uniform(10, 35)):
                requested = old_length+change
                if requested <= 50:
                    continue
                try:
                    _known_free_space_target(current, target, requested, side)
                except (AssertionError, DocumentError, ValueError):
                    continue
                found = (target, side, requested)
                break
            if found:
                break
        assert found, f'No independently valid edit found at step {step} for case_seed={case_seed}'
        target, side, requested = found
        operations.append({'target': target, 'side': side, 'length_mm': requested})
        try:
            result = resize_drag(current, target, requested, side, auto_connect=False)
            assert result['status'] == 'resized', result
            after = Assembly.from_doc(result['document'], current.base, current.library)
            _assert_graph_intact(current, after)
            first, last = _effective_frame(after, target)
            assert np.isclose(np.linalg.norm(last-first), requested, atol=.01)
            _exactly_valid(after)
            current = after
        except Exception as error:
            path = tmp_path / f'lattice-sequence-{case_seed}.json'
            path.write_text(json.dumps({'case_seed': case_seed,
                                        'before': current.doc, 'operations': operations},
                                       indent=2), encoding='utf-8')
            pytest.fail(f'connected lattice case_seed={case_seed}, step={step}, '
                        f'repro={path}: {error}')


def _random_connected_graph(seed):
    """Build a valid TC161 graph with variable stations and branch depth."""
    rng = random.Random(seed)
    mirrored = rng.randrange(2) == 0
    if mirrored:
        global_rotation = (Rotation.from_euler('x', rng.uniform(-180, 180), degrees=True) *
                           Rotation.from_euler('y', rng.choice((0, 180)), degrees=True))
    else:
        global_rotation = Rotation.from_euler('xyz',
            [rng.uniform(-175, 175) for _ in range(3)], degrees=True)
    translation = np.array([rng.uniform(-1200, 1200), rng.uniform(-1200, 1200),
                            rng.uniform(250, 1400)])
    def world(point):
        return (translation+global_rotation.apply(point)).tolist()
    def connector(identifier, point, orientation=None):
        local_rotation = orientation or Rotation.identity()
        return {'id': identifier, 'catalog': 'tubeclamp.TC161C', 'pose': {
            'position_mm': world(point),
            'rotation_deg': (global_rotation*local_rotation).as_euler('xyz', degrees=True).tolist()}}
    length = rng.uniform(650, 1300)
    count = rng.randint(1, 6)
    stations = np.linspace(.18, .82, count) if count > 1 else [rng.uniform(.3, .7)]
    parts = []
    main_attachments = []
    branch_runs = []
    modes = {'main': 'in_plane'}
    second_level = False
    for index, fraction in enumerate(stations):
        station = float(length*fraction)
        fitting = f'tee-{index}'
        parts.append(connector(fitting, [0, 0, station]))
        main_attachments.append({'connector': fitting, 'port': 'through'})
        if rng.randrange(3) == 0:
            continue
        width = rng.uniform(320, 600)
        identifier = f'branch-{index}'
        attachments = [{'connector': fitting, 'port': 'cross'}]
        if not second_level and rng.randrange(3) == 0:
            substation = rng.uniform(70, min(150, width/2-40))
            child = f'branch-tee-{index}'
            parts.append(connector(child, [substation, 42.4, station],
                                   Rotation.from_euler('y', 90, degrees=True)))
            attachments.append({'connector': child, 'port': 'through'})
            branch_runs.append({'id': f'child-{index}', 'catalog': 'tubeclamp.tube-C',
                'start_mm': world([substation, 84.8, station-150]),
                'end_mm': world([substation, 84.8, station+150]),
                'attachments': [{'connector': child, 'port': 'cross'}]})
            modes[f'child-{index}'] = 'free'
            second_level = True
        branch_runs.append({'id': identifier, 'catalog': 'tubeclamp.tube-C',
            'start_mm': world([-width/2, 42.4, station]),
            'end_mm': world([width/2, 42.4, station]),
            'attachments': attachments})
        modes[identifier] = 'centered'
    main = {'id': 'main', 'catalog': 'tubeclamp.tube-C',
            'start_mm': world([0, 0, 0]), 'end_mm': world([0, 0, length]),
            'attachments': main_attachments}
    group = {'id': 'graph', 'runs': [main, *branch_runs]}
    if mirrored:
        group['mirrors'] = [{'id': 'midplane', 'scope': 'scene', 'axis': 'x',
                             'offset_mm': float(translation[0]), 'run_modes': modes}]
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': f'Connected resize graph {seed}', 'parts': parts, 'joints': [],
           'anchors': ([{'part': 'tee-0', 'surface': 'fixture'}] if rng.randrange(5) == 0 else []),
           'draft_subassemblies': [group]}
    return doc


def _check_connected_graph_sequence(seed, library, repro_dir=None):
    initial = _random_connected_graph(seed)
    current = None
    operations = []
    try:
        current = Assembly.from_doc(initial, FIXTURE.parent, library)
        assert all(not item['conflicts'] for item in preview(current))
        _exactly_valid(current)
        rng = random.Random(seed ^ 0xA529A529)
        for _ in range(rng.randint(2, 3)):
            side = rng.choice(('start', 'end'))
            delta = rng.uniform(20, 90)
            first, last = _effective_frame(current, 'main')
            requested = float(np.linalg.norm(last-first)+delta)
            operations.append({'side': side, 'requested_length_mm': requested})
            _known_free_space_target(current, 'main', requested, side)
            result = resize_drag(current, 'main', requested, side, auto_connect=False)
            assert result['status'] == 'resized', result
            after = Assembly.from_doc(result['document'], current.base, current.library)
            changed_first, changed_last = _effective_frame(after, 'main')
            assert np.isclose(np.linalg.norm(changed_last-changed_first), requested, atol=.01)
            fixed = last if side == 'start' else first
            assert np.allclose(changed_last if side == 'start' else changed_first, fixed, atol=.01)
            _assert_graph_intact(current, after)
            _exactly_valid(after)
            current = after
    except Exception as error:
        path = None
        if repro_dir is not None:
            path = repro_dir / f'resize-graph-{seed}.json'
            path.write_text(json.dumps({'case_seed': seed, 'initial_document': initial,
                'document_before_failure': current.doc if current else None,
                'operations': operations}, indent=2), encoding='utf-8')
        pytest.fail(f'connected resize graph case_seed={seed}, operations={operations}, '
                    f'repro={path or "set PIPESIM_RESIZE_GRAPH_SEEDS to this seed"}: {error}')


@pytest.mark.parametrize('case_seed', GRAPH_SEEDS)
def test_randomized_connected_graph_resize_sequence(case_seed, library, tmp_path):
    _check_connected_graph_sequence(case_seed, library, tmp_path)


@pytest.mark.skipif(os.environ.get('PIPESIM_RESIZE_REALWORLD_LONG') != '1',
                    reason='opt in with PIPESIM_RESIZE_REALWORLD_LONG=1')
def test_resize_realworld_long_randomized(library, tmp_path):
    root_seed = int(os.environ.get('PIPESIM_RESIZE_REALWORLD_ROOT_SEED', secrets.randbits(64)))
    case_limit = int(os.environ.get('PIPESIM_RESIZE_REALWORLD_CASES', '1000000'))
    deadline = time.monotonic() + float(os.environ.get('PIPESIM_RESIZE_REALWORLD_HOURS', '2'))*3600
    started = time.monotonic()
    rng = random.Random(root_seed)
    print(f'Resize fuzz root_seed={root_seed}, case_limit={case_limit}', flush=True)
    completed = 0
    while completed < case_limit and time.monotonic() < deadline:
        batch = [rng.getrandbits(64) for _ in range(min(25, case_limit-completed))]
        batch_dir = tmp_path / f'batch-{completed//25:06d}'
        child_env = os.environ.copy()
        child_env['PIPESIM_RESIZE_GRAPH_SEEDS'] = ','.join(map(str, batch))
        child_env.pop('PIPESIM_RESIZE_REALWORLD_LONG', None)
        child = subprocess.run(
            [sys.executable, '-m', 'pytest', '-q', '-s', f'--basetemp={batch_dir}',
             'tests/test_resize_realworld.py', '-k', 'randomized_connected_graph_resize_sequence'],
            env=child_env, capture_output=True, text=True, check=False)
        if child.returncode:
            pytest.fail(f'Resize fuzz root_seed={root_seed}, completed={completed}, '
                        f'batch_seeds={batch}, repro_dir={batch_dir}\n'
                        f'{child.stdout}\n{child.stderr}', pytrace=False)
        completed += len(batch)
        print(f'Resize fuzz completed={completed}, elapsed_s={time.monotonic()-started:.0f}, '
              f'last_case_seed={batch[-1]}', flush=True)
    assert completed > 0, 'Long fuzz budget did not permit a single case'
