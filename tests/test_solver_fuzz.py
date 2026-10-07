"""Geometry fuzzing for draft repair and exact socket search.

The bounded suite combines known seeds with new random cases on every run. Failures
include a case seed; replay one with ``PIPESIM_FUZZ_CASE_SEEDS=<seed> pytest
tests/test_solver_fuzz.py -k draft_repair_varied``. The optional long run is
documented in docs/development.md.
"""

import copy
import json
import math
import os
import random
import secrets
import subprocess
import sys
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.drafting import finalize, preview, repair, runs
from pipesim.grouping import attach_part
from pipesim.snapping import connection_options
from pipesim.snapping import move_document
from pipesim.validation import validate


SEEDS = tuple(int(s) for s in os.environ.get(
    'PIPESIM_FUZZ_SEEDS',
    ','.join(map(str, range(16, 32))) + ',' + ','.join(str(secrets.randbits(64)) for _ in range(8))
).split(',') if s)
CASE_SEEDS = tuple(int(s) for s in os.environ.get(
    'PIPESIM_FUZZ_CASE_SEEDS',
    ','.join(map(str, range(96))) + ',' + ','.join(str(secrets.randbits(64)) for _ in range(16))
).split(',') if s)


def _pose(point, angles):
    return {'position_mm': np.asarray(point, dtype=float).tolist(),
            'rotation_deg': np.asarray(angles, dtype=float).tolist()}


def _varied_draft_case(seed):
    """Construct a known-solvable graph, then disturb movable socket frames.

    Cases include long through runs, a shared perpendicular tee, a closed grid
    of four tees and four pipes, and two-ended runs. Global orientation, anchor
    choice, pipe direction, connector count, and perturbation direction vary
    independently from the case seed.
    """
    rng = random.Random(seed)
    # Include substantial but still solvable user drags, not only nearly
    # aligned fittings. The untouched baseline is the constructive oracle.
    wide_edit = seed % 8 == 0
    family = seed % 4
    include_scene_mirror = rng.randrange(2) == 0
    constrained_mirror = include_scene_mirror and family != 2 and rng.randrange(2) == 0
    # A signed axis permutation makes centered/in-plane mirror modes exact.
    # Arbitrary rotations remain covered by unmirrored and free-mode cases.
    angles = ([0., 0., float(rng.choice((0, 90, 180, 270)))] if constrained_mirror else
              [rng.uniform(-80, 80), rng.uniform(-80, 80), rng.uniform(-180, 180)])
    rotation = Rotation.from_euler('xyz', angles, degrees=True)
    origin = np.array([rng.uniform(-1200, 1200), rng.uniform(-1200, 1200),
                       rng.uniform(250, 1300)])
    def world(local):
        return origin + rotation.apply(local)
    def run(identifier, axis, length, attachments):
        direction = np.zeros(3); direction[axis] = 1
        return {'id': identifier, 'catalog': 'tubeclamp.tube-C',
                'start_mm': world(-direction*length/2).tolist(),
                'end_mm': world(direction*length/2).tolist(),
                'attachments': attachments}
    parts = []
    run_defs = []
    anchors = []
    disturbed_ids = []
    mounted_on = {}
    joints = []
    if family == 0:
        # Several independent through fittings on one continuous cut length.
        length = rng.uniform(500, 1300)
        count = rng.randint(1, 6)
        positions = np.linspace(-.34, .34, count) if count > 1 else [0.]
        attachments = []
        for index, fraction in enumerate(positions):
            pid = f'through-{index}'
            parts.append(_fitting(pid, world([0, 0, length*fraction]),
                                  {'bore': _socket([0, 0, 1], through=True)}))
            parts[-1]['pose']['rotation_deg'] = angles.copy()
            attachments.append({'connector': pid, 'port': 'bore'})
            if index == 0 and count > 1 and rng.randrange(2):
                anchors.append({'part': pid, 'surface': 'fixture'})
            else:
                disturbed_ids.append(pid)
        run_defs.append(run('rail', 2, length, attachments))
        if rng.randrange(3) == 0:
            host = rng.choice(disturbed_ids)
            host_part = next(part for part in parts if part['id'] == host)
            offset = np.array([rng.uniform(100, 250), rng.uniform(-150, 150),
                               rng.uniform(-100, 100)])
            kind = rng.choice(('panel', 'wheel'))
            mounted = {'id': 'mounted',
                       'catalog': 'generic.panel' if kind == 'panel' else 'generic.wheel',
                       'pose': _pose(np.asarray(host_part['pose']['position_mm']) +
                                     rotation.apply(offset), angles)}
            if kind == 'panel':
                mounted['parameters'] = {'width_mm': 80, 'depth_mm': 80,
                                         'thickness_mm': 15}
            parts.append(mounted)
            joint_frame = {'position_mm': offset.tolist()}
            if kind == 'wheel':
                joint_frame['axis'] = [0, 1, 0]
            joints.append({'id': 'mount',
                           'type': 'fixed' if kind == 'panel' else 'revolute',
                           'a': {'part': host, 'frame': joint_frame},
                           'b': {'part': 'mounted',
                                 **({'frame': {'axis': [0, 1, 0]}} if kind == 'wheel' else {})}})
            mounted_on[host] = 'mounted'
    elif family == 1:
        # A tee's two independent sockets must rotate together to fit both runs.
        lengths = (rng.uniform(500, 1000), rng.uniform(500, 1000))
        parts.append(_fitting('tee', origin, {
            'through': _socket([0, 0, 1], through=True),
            'cross': _socket([1, 0, 0], through=True)}))
        parts[-1]['pose']['rotation_deg'] = angles.copy()
        disturbed_ids.append('tee')
        for axis, port, name, length in ((2, 'through', 'upright', lengths[0]),
                                         (0, 'cross', 'crossbar', lengths[1])):
            attachments = [{'connector': 'tee', 'port': port}]
            if rng.randrange(2):
                direction = np.zeros(3); direction[axis] = 1
                pid = f'anchor-{name}'
                parts.append(_fitting(pid, world(direction*length*.28),
                                      {'bore': _socket(direction, through=True)}))
                parts[-1]['pose']['rotation_deg'] = angles.copy()
                anchors.append({'part': pid, 'surface': 'fixture'})
                attachments.append({'connector': pid, 'port': 'bore'})
            run_defs.append(run(name, axis, length, attachments))
    elif family == 2:
        # Two end sockets make a pipe line; a middle through fit adds a station.
        length = rng.uniform(500, 1000)
        direction = np.array([0., 0., 1.])
        for name, sign in (('start', -1), ('end', 1)):
            point = world(direction*sign*(length/2+20))
            parts.append(_fitting(name, point,
                                  {'bore': _socket(direction*(-sign))}))
            parts[-1]['pose']['rotation_deg'] = angles.copy()
        anchors.append({'part': 'start', 'surface': 'fixture'})
        disturbed_ids.append('end')
        attachments = [{'connector': 'start', 'port': 'bore', 'end': 'start',
                        'insertion_mm': 20},
                       {'connector': 'end', 'port': 'bore', 'end': 'end',
                        'insertion_mm': 20}]
        if rng.randrange(2):
            parts.append(_fitting('middle', world([0, 0, length*rng.uniform(-.25, .25)]),
                                  {'bore': _socket([0, 0, 1], through=True)}))
            parts[-1]['pose']['rotation_deg'] = angles.copy()
            attachments.append({'connector': 'middle', 'port': 'bore'})
            disturbed_ids.append('middle')
        run_defs.append(run('rail', 2, length, attachments))
    else:
        # A small lattice makes each tee constrain two separate continuous
        # pipes. Moving one fitting must close both lines without moving the
        # anchor or merging those two pipes into one connection.
        spacing = rng.uniform(280, 520)
        length = spacing + rng.uniform(250, 500)
        coordinates = (-spacing/2, spacing/2)
        for ix, x in enumerate(coordinates):
            for iy, y in enumerate(coordinates):
                pid = f'tee-{ix}-{iy}'
                parts.append(_fitting(pid, world([x, y, 0]), {
                    'horizontal': _socket([1, 0, 0], through=True),
                    'vertical': _socket([0, 1, 0], through=True)}))
                parts[-1]['pose']['rotation_deg'] = angles.copy()
                if ix == 0 and iy == 0:
                    anchors.append({'part': pid, 'surface': 'fixture'})
                else:
                    disturbed_ids.append(pid)
        for iy, y in enumerate(coordinates):
            run_defs.append({'id': f'horizontal-{iy}',
                'catalog': 'tubeclamp.tube-C',
                'start_mm': world([-length/2, y, 0]).tolist(),
                'end_mm': world([length/2, y, 0]).tolist(),
                'attachments': [{'connector': f'tee-{ix}-{iy}',
                                 'port': 'horizontal'} for ix in range(2)]})
        for ix, x in enumerate(coordinates):
            run_defs.append({'id': f'vertical-{ix}',
                'catalog': 'tubeclamp.tube-C',
                'start_mm': world([x, -length/2, 0]).tolist(),
                'end_mm': world([x, length/2, 0]).tolist(),
                'attachments': [{'connector': f'tee-{ix}-{iy}',
                                 'port': 'vertical'} for iy in range(2)]})
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': f'Varied draft fuzz {seed}', 'parts': parts, 'joints': joints,
           'anchors': anchors,
           'draft_subassemblies': [{'id': 'frame', 'runs': run_defs}]}
    if include_scene_mirror:
        mirror_axis = rng.randrange(3)
        offset = float(origin[mirror_axis])
        modes = {}
        for definition in run_defs:
            start = np.asarray(definition['start_mm'])
            end = np.asarray(definition['end_mm'])
            length = np.linalg.norm(end-start)
            if (constrained_mirror and
                    abs(abs(end[mirror_axis]-start[mirror_axis])-length) < 1e-6 and
                    abs((start[mirror_axis]+end[mirror_axis])/2-offset) < 1e-6):
                mode = 'centered'
            elif (constrained_mirror and abs(start[mirror_axis]-offset) < 1e-6 and
                  abs(end[mirror_axis]-offset) < 1e-6):
                mode = 'in_plane'
            else:
                mode = 'free'
            modes[definition['id']] = mode
        doc['draft_subassemblies'][0]['mirrors'] = [{
            'id': 'scene-plane', 'scope': 'scene', 'axis': 'xyz'[mirror_axis],
            'offset_mm': offset, 'run_modes': modes}]
    baseline = copy.deepcopy(doc)
    # A through-only run is a free pose in draft. Move its entire preview span
    # in some cases: this is the repair path used after dragging a pipe onto
    # sockets that were already occupied by other graph constraints.
    run_modes = (doc['draft_subassemblies'][0].get('mirrors') or [{}])[0].get('run_modes', {})
    free_runs = [definition for definition in run_defs
                 if all('end' not in attachment for attachment in definition['attachments'])
                 and run_modes.get(definition['id'], 'free') == 'free']
    move_run = bool(free_runs) and rng.randrange(2) == 0
    # Disturb one or several movable fittings as well; run-only cases are
    # deliberately included to isolate the free-run pose solve.
    noisy = rng.sample(disturbed_ids,
                       k=rng.randint(0 if move_run else 1, len(disturbed_ids)))
    for pid in noisy:
        part = next(item for item in doc['parts'] if item['id'] == pid)
        before_position = np.asarray(part['pose']['position_mm'])
        before_rotation = Rotation.from_euler('xyz', part['pose']['rotation_deg'], degrees=True)
        lateral = rotation.apply([rng.choice((-1, 1))*rng.uniform(5 if wide_edit else 2,
                                                                     40 if wide_edit else 12),
                                  rng.choice((-1, 1))*rng.uniform(5 if wide_edit else 2,
                                                                     40 if wide_edit else 12),
                                  rng.uniform(-15 if wide_edit else -5,
                                              15 if wide_edit else 5)])
        part['pose']['position_mm'] = (np.asarray(part['pose']['position_mm']) + lateral).tolist()
        turn = Rotation.from_rotvec(np.asarray([rng.uniform(-1, 1) for _ in range(3)]) *
                                    math.radians(rng.uniform(5 if wide_edit else 2,
                                                             20 if wide_edit else 8)))
        after_rotation = turn*before_rotation
        part['pose']['rotation_deg'] = after_rotation.as_euler('xyz', degrees=True).tolist()
        if pid in mounted_on:
            mounted = next(item for item in doc['parts'] if item['id'] == mounted_on[pid])
            relative = before_rotation.inv().apply(
                np.asarray(mounted['pose']['position_mm'])-before_position)
            mounted['pose'] = _pose(np.asarray(part['pose']['position_mm']) +
                                    after_rotation.apply(relative),
                                    part['pose']['rotation_deg'])
    if move_run:
        definition = rng.choice(free_runs)
        start = np.asarray(definition['start_mm'])
        end = np.asarray(definition['end_mm'])
        midpoint = (start+end)/2
        direction = (end-start)/np.linalg.norm(end-start)
        trial = np.array([rng.uniform(-1, 1) for _ in range(3)])
        lateral = trial-direction*(trial@direction)
        if np.linalg.norm(lateral) < 1e-9:
            fallback = np.eye(3)[int(np.argmin(abs(direction)))]
            lateral = fallback-direction*(fallback@direction)
        lateral /= np.linalg.norm(lateral)
        offset = lateral*rng.uniform(5 if wide_edit else 2, 40 if wide_edit else 12)
        turn = Rotation.from_rotvec(lateral*math.radians(
            rng.uniform(5 if wide_edit else 2, 20 if wide_edit else 8)))
        definition['start_mm'] = (midpoint+offset+turn.apply(start-midpoint)).tolist()
        definition['end_mm'] = (midpoint+offset+turn.apply(end-midpoint)).tolist()
    return baseline, doc


def _check_varied_draft_case(seed, factory, *, repro_dir=None, check_collisions=None):
    # Straight runs and end sockets have no deliberate pipe crossings. Keep
    # physical collision checks on for these families in every long batch.
    if check_collisions is None:
        check_collisions = seed % 4 in (0, 2)
    baseline, doc = _varied_draft_case(seed)
    try:
        original = copy.deepcopy(doc)
        sound = factory(baseline)
        assert all(not item['conflicts'] for item in preview(sound))
        assert finalize(sound, check_collisions=check_collisions)['status'] == 'finalized'
        disturbed = factory(doc)
        assert any(item['conflicts'] for item in preview(disturbed))
        result = repair(disturbed)
        assert result['status'] in ('repaired', 'aligned'), result
        restored = factory(result.get('document', doc))
        assert restored.doc['draft_subassemblies'][0].get('mirrors', []) == \
            doc['draft_subassemblies'][0].get('mirrors', [])
        assert all(not item['conflicts'] for item in preview(restored))
        exact = finalize(restored, check_collisions=check_collisions)
        assert exact['status'] == 'finalized', exact
        report = validate(factory(exact['document']), collisions=check_collisions)
        assert report['valid'], report['issues']
        for anchor in sound.anchors:
            pid = anchor['part']
            assert np.allclose(restored.parts[pid].matrix, sound.parts[pid].matrix, atol=1e-6)
        assert doc == original  # Repair and finalization are pure until accepted.
    except Exception as exc:
        path = None
        if repro_dir is not None:
            path = repro_dir / f'draft-fuzz-{seed}.json'
            path.write_text(json.dumps({'case_seed': seed, 'baseline': baseline,
                                        'disturbed': doc}, indent=2), encoding='utf-8')
        raise AssertionError(f'Draft fuzz case_seed={seed} family={seed % 4} '
                             f'repro={path or "set PIPESIM_FUZZ_CASE_SEEDS to this seed"}') from exc


@pytest.mark.parametrize('case_seed', CASE_SEEDS)
def test_draft_repair_varied_geometry(case_seed, factory, tmp_path):
    _check_varied_draft_case(case_seed, factory, repro_dir=tmp_path)


COLLISION_CASE_SEEDS = tuple(range(0, 40, 2)) + tuple(
    secrets.randbits(63) * 2 for _ in range(4))


@pytest.mark.parametrize('case_seed', COLLISION_CASE_SEEDS)
def test_draft_repair_collision_valid_geometry(case_seed, factory, tmp_path):
    """A physically valid starting model must still finalize after repair."""
    _check_varied_draft_case(case_seed, factory, repro_dir=tmp_path,
                             check_collisions=True)


@pytest.mark.skipif(os.environ.get('PIPESIM_FUZZ_LONG') != '1',
                    reason='opt in with PIPESIM_FUZZ_LONG=1')
def test_draft_repair_long_randomized(factory, tmp_path):
    """Explore in fresh processes until the wall-time or case budget expires.

    Geometry libraries may retain native allocations across models. A bounded
    child process prevents a long campaign from exhausting memory and keeps
    failing case models in the pytest temporary directory for replay.
    """
    root_seed = int(os.environ.get('PIPESIM_FUZZ_ROOT_SEED', secrets.randbits(64)))
    case_limit = int(os.environ.get('PIPESIM_FUZZ_CASES', '1000000'))
    deadline = time.monotonic() + float(os.environ.get('PIPESIM_FUZZ_HOURS', '2'))*3600
    rng = random.Random(root_seed)
    print(f'Draft fuzz root_seed={root_seed}, case_limit={case_limit}', flush=True)
    completed = 0
    started = time.monotonic()
    while completed < case_limit and time.monotonic() < deadline:
        batch = [rng.getrandbits(64) for _ in range(min(25, case_limit-completed))]
        batch_dir = tmp_path / f'batch-{completed//25:06d}'
        child_env = os.environ.copy()
        child_env['PIPESIM_FUZZ_CASE_SEEDS'] = ','.join(map(str, batch))
        child_env.pop('PIPESIM_FUZZ_LONG', None)
        child = subprocess.run(
            [sys.executable, '-m', 'pytest', '-q', '-s', f'--basetemp={batch_dir}',
             'tests/test_solver_fuzz.py', '-k', 'draft_repair_varied_geometry'],
            env=child_env, capture_output=True, text=True, check=False)
        if child.returncode:
            pytest.fail(f'Draft fuzz root_seed={root_seed}, completed={completed}, '
                        f'batch_seeds={batch}, repro_dir={batch_dir}\n'
                        f'{child.stdout}\n{child.stderr}', pytrace=False)
        completed += len(batch)
        print(f'Draft fuzz completed={completed}, elapsed_s={time.monotonic()-started:.0f}, '
              f'last_case_seed={batch[-1]}', flush=True)
    assert completed > 0, 'Long fuzz budget did not permit a single case'


def _socket(axis, *, through=False):
    return {'type': 'socket', 'profile': 'round', 'diameter_mm': 42.4,
            'position_mm': [0, 0, 0], 'axis': list(axis), 'through': through,
            'engagement_mm': 40, 'min_engagement_mm': 15}


def _fitting(identifier, point, ports):
    return {'id': identifier, 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 5, 'position_mm': [0, -80, 0]}],
        'ports': ports}, 'pose': {'position_mm': list(point)}}


def _draft_case(seed, *, mirrors, companion, anchored):
    rng = random.Random(seed)
    angle = rng.uniform(.25, 1.05)
    direction = np.array([0., math.cos(angle), math.sin(angle)])
    half = rng.uniform(240, 370)
    span = rng.uniform(350, 650)
    origin = np.array([half-20, rng.uniform(-200, 200), rng.uniform(250, 500)])
    far = origin+span*direction
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': f'Draft fuzz {seed}',
           'parts': [_fitting('hub', origin, {'rail': _socket(direction),
                                           'cross': _socket([-1, 0, 0])}),
                     _fitting('far', far, {'rail': _socket(-direction)})],
           'joints': [], 'anchors': ([{'part': 'hub', 'surface': 'fixture'}] if anchored else []),
           'draft_subassemblies': [{'id': 'frame', 'runs': [
               {'id': 'rail', 'catalog': 'tubeclamp.tube-C',
                'start_mm': (origin-20*direction).tolist(),
                'end_mm': (far+20*direction).tolist(), 'attachments': [
                    {'connector': 'hub', 'port': 'rail', 'end': 'start', 'insertion_mm': 20},
                    {'connector': 'far', 'port': 'rail', 'end': 'end', 'insertion_mm': 20}]},
               {'id': 'cross', 'catalog': 'tubeclamp.tube-C',
                'start_mm': [-half, origin[1], origin[2]],
                'end_mm': [half, origin[1], origin[2]], 'attachments': [
                    {'connector': 'hub', 'port': 'cross', 'end': 'end', 'insertion_mm': 20}]}],
               'mirrors': [{'id': 'left-right', 'axis': 'x', 'offset_mm': 0,
                            'run_modes': {'cross': 'centered'}}]}]}
    if mirrors == 2:
        doc['draft_subassemblies'][0]['mirrors'].append(
            {'id': 'fore-aft', 'axis': 'y', 'offset_mm': float(origin[1]),
             'run_modes': {'cross': 'in_plane'}})
    if companion == 'board':
        offset = np.array([0, 400., 0])
        doc['parts'].append({'id': 'board', 'catalog': 'generic.panel',
            'parameters': {'width_mm': 80, 'depth_mm': 80, 'thickness_mm': 15},
            'pose': {'position_mm': (origin+offset).tolist()}})
        doc['joints'].append({'id': 'board-mount', 'type': 'fixed',
            'a': {'part': 'hub', 'frame': {'position_mm': offset.tolist()}},
            'b': {'part': 'board'}})
    elif companion == 'wheel':
        offset = np.array([0, 0, 250.])
        doc['parts'].append({'id': 'wheel', 'catalog': 'generic.wheel',
            'pose': {'position_mm': (origin+offset).tolist()}})
        doc['joints'].append({'id': 'wheel-axle', 'type': 'revolute',
            'a': {'part': 'hub', 'frame': {'position_mm': offset.tolist(), 'axis': [0, 1, 0]}},
            'b': {'part': 'wheel', 'frame': {'axis': [0, 1, 0]}}})
    elif companion == 'chain':
        doc['objects'] = [{'id': 'chain', 'template': 'chain',
            'parameters': {'length_mm': 80},
            'pose': {'position_mm': [-2500, -2500, 1200]}}]
    return doc


@pytest.mark.parametrize('seed', SEEDS)
def test_draft_repair_seeded_geometry(seed, factory):
    mirrors = 1 + (seed // 4) % 2
    companion = ('none', 'board', 'wheel', 'chain')[seed % 4]
    anchored = bool((seed // 8) % 2)
    doc = _draft_case(seed, mirrors=mirrors, companion=companion, anchored=anchored)
    baseline = factory(doc)
    assert all(not part['conflicts'] for part in preview(baseline)), seed
    baseline_exact = finalize(baseline, check_collisions=False)
    assert baseline_exact['status'] == 'finalized', (seed, baseline_exact)
    assert validate(factory(baseline_exact['document']), collisions=False)['valid'], seed
    original = copy.deepcopy(doc)
    rng = random.Random(seed+1000)
    far = next(part for part in doc['parts'] if part['id'] == 'far')
    far['pose']['position_mm'] = [value+rng.uniform(-3, 3) for value in far['pose']['position_mm']]
    far['pose']['rotation_deg'] = [rng.uniform(-.5, .5) for _ in range(3)]
    disturbed = factory(doc)
    result = repair(disturbed, run_id='rail')
    assert result['status'] in ('repaired', 'aligned'), (seed, result)
    restored = factory(result.get('document', doc))
    assert all(not part['conflicts'] for part in preview(restored)), seed
    exact = finalize(restored, check_collisions=False)
    assert exact['status'] == 'finalized', (seed, exact)
    assert validate(factory(exact['document']), collisions=False)['valid'], seed
    assert doc != original and baseline.doc == original
    for anchor in baseline.anchors:
        part = anchor['part']
        assert np.allclose(restored.parts[part].matrix, baseline.parts[part].matrix, atol=1e-6), seed


@pytest.mark.parametrize('seed', SEEDS[:8])
def test_repair_discovers_multiple_unrecorded_through_fits(seed, factory):
    rng = random.Random(seed+3000)
    angle = rng.uniform(-.8, .8)
    direction = np.array([math.cos(angle), math.sin(angle), rng.uniform(-.4, .4)])
    direction /= np.linalg.norm(direction)
    origin = np.array([rng.uniform(-300, 300), rng.uniform(-300, 300), 600.])
    length = rng.uniform(700, 1100)
    parts = []
    for index, fraction in enumerate((.2, .4, .6, .8)):
        point = origin+direction*length*fraction
        if index:
            point += np.array([rng.uniform(-2, 2) for _ in range(3)])
        fitting = _fitting(f'fit-{index}', point, {'bore': _socket(direction, through=True)})
        if index:
            fitting['pose']['rotation_deg'] = [rng.uniform(-1.5, 1.5) for _ in range(3)]
        parts.append(fitting)
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': f'Through fuzz {seed}',
           'parts': parts, 'joints': [], 'anchors': [{'part': 'fit-0', 'surface': 'fixture'}],
           'draft_subassemblies': [{'id': 'frame', 'runs': [{
               'id': 'rail', 'catalog': 'tubeclamp.tube-C', 'start_mm': origin.tolist(),
               'end_mm': (origin+direction*length).tolist(),
               'attachments': [{'connector': 'fit-0', 'port': 'bore'}]}]}]}
    assert len(runs(doc)[0]['attachments']) == 1
    result = repair(factory(doc), run_id='rail')
    assert result['status'] == 'repaired', (seed, result)
    assert result['inferred_through_connections'] == 3, (seed, result)
    restored = factory(result['document'])
    assert len(runs(restored.doc)[0]['attachments']) == 4, seed
    assert preview(restored)[0]['conflicts'] == [], seed
    exact = finalize(restored, check_collisions=False)
    assert exact['status'] == 'finalized', (seed, exact)
    assert len(exact['document']['joints']) == 4, seed
    assert validate(factory(exact['document']), collisions=False)['valid'], seed


def _exact_case(seed, *, companion, anchored):
    rng = random.Random(seed)
    angles = [rng.uniform(-35, 35), rng.uniform(-35, 35), rng.uniform(-180, 180)]
    rotation = Rotation.from_euler('xyz', angles, degrees=True)
    center = np.array([rng.uniform(1000, 2000), rng.uniform(-500, 500), rng.uniform(700, 1500)])
    length = rng.uniform(350, 850)
    fitting_position = center + rotation.apply([0, 0, length/2+20])
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': f'Exact fuzz {seed}',
           'parts': [{'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
                      'parameters': {'length_mm': length},
                      'pose': {'position_mm': center.tolist(), 'rotation_deg': angles}},
                     _fitting('fitting', fitting_position, {'end': _socket([0, 0, -1])})],
           'joints': [], 'anchors': [{'part': anchored, 'surface': 'fixture'}]}
    doc['parts'][1]['pose']['rotation_deg'] = angles.copy()
    if companion in ('board', 'wheel'):
        offset = np.array([0, 400, 0]) if companion == 'board' else np.array([0, 0, 250])
        position = fitting_position + rotation.apply(offset)
        part = {'id': companion, 'catalog': 'generic.panel' if companion == 'board' else 'generic.wheel',
                'pose': {'position_mm': position.tolist(), 'rotation_deg': angles.copy()}}
        if companion == 'board':
            part['parameters'] = {'width_mm': 80, 'depth_mm': 80, 'thickness_mm': 15}
        doc['parts'].append(part)
        frame = {'position_mm': offset.tolist()}
        if companion == 'wheel':
            frame['axis'] = [0, 1, 0]
        doc['joints'].append({'id': 'companion', 'type': 'fixed' if companion == 'board' else 'revolute',
                              'a': {'part': 'fitting', 'frame': frame},
                              'b': {'part': companion, **({'frame': {'axis': [0, 1, 0]}} if companion == 'wheel' else {})}})
    elif companion == 'chain':
        doc['objects'] = [{'id': 'chain', 'template': 'chain',
                           'parameters': {'length_mm': 80},
                           'pose': {'position_mm': [-2500, -2500, 1200]}}]
    return doc


@pytest.mark.parametrize('seed', SEEDS)
def test_exact_connection_search_seeded_pose_noise(seed, factory):
    companion = ('none', 'board', 'wheel', 'chain')[seed % 4]
    anchored = 'fitting' if companion == 'chain' or seed % 2 == 0 else 'pipe'
    doc = _exact_case(seed, companion=companion, anchored=anchored)
    if companion == 'chain':
        doc = attach_part(factory(doc), 'chain', 'chain/link-1',
                          {'part': 'fitting', 'frame': {'position_mm': [0, 400, 0]}},
                          'spherical')['document']
    baseline = factory(doc)
    report = validate(baseline, collisions=False)
    assert report['valid'], (seed, report['issues'])

    def connected(source):
        options = connection_options(factory(source), 'pipe', 'fitting', 'end', end='end')
        chosen = next(option for option in options['options'] if option['move'] ==
                      ('connector' if anchored == 'pipe' else 'member'))
        assert chosen['available'], (seed, chosen.get('reason'))
        finished = factory(chosen['document'])
        report = validate(finished, collisions=False)
        assert report['valid'], (seed, report['issues'])
        assert len(finished.joints) == len(baseline.joints)+1
        return finished

    connected(doc)
    original = copy.deepcopy(doc)
    rng = random.Random(seed+2000)
    # Move a mounted companion with its fitting so its existing joint remains valid.
    fitting = next(part for part in doc['parts'] if part['id'] == 'fitting')
    old_position = np.array(fitting['pose']['position_mm'])
    old_rotation = Rotation.from_euler('xyz', fitting['pose']['rotation_deg'], degrees=True)
    for part in doc['parts'][:2]:
        part['pose']['position_mm'] = (np.array(part['pose']['position_mm'])+
                                       [rng.uniform(-15, 15) for _ in range(3)]).tolist()
        part['pose']['rotation_deg'] = (np.array(part['pose']['rotation_deg'])+
                                       [rng.uniform(-8, 8) for _ in range(3)]).tolist()
    if companion in ('board', 'wheel'):
        new_position = np.array(fitting['pose']['position_mm'])
        new_rotation = Rotation.from_euler('xyz', fitting['pose']['rotation_deg'], degrees=True)
        mounted = doc['parts'][2]
        mounted['pose']['position_mm'] = (new_position + new_rotation.apply(
            old_rotation.inv().apply(np.array(mounted['pose']['position_mm'])-old_position))).tolist()
        mounted['pose']['rotation_deg'] = fitting['pose']['rotation_deg'].copy()
    elif companion == 'chain':
        new_position = np.array(fitting['pose']['position_mm'])
        new_rotation = Rotation.from_euler('xyz', fitting['pose']['rotation_deg'], degrees=True)
        old_object = original['objects'][0]['pose']
        old_object_rotation = Rotation.from_euler('xyz', old_object.get('rotation_deg', [0, 0, 0]), degrees=True)
        doc['objects'][0]['pose']['position_mm'] = (new_position + new_rotation.apply(
            old_rotation.inv().apply(np.array(old_object['position_mm'])-old_position))).tolist()
        doc['objects'][0]['pose']['rotation_deg'] = (
            new_rotation * old_rotation.inv() * old_object_rotation).as_euler('xyz', degrees=True).tolist()
    report = validate(factory(doc), collisions=False)
    assert report['valid'], (seed, report['issues'])
    finished = connected(doc)
    assert doc != original and baseline.doc == original
    assert np.allclose(finished.parts[anchored].matrix, factory(doc).parts[anchored].matrix, atol=1e-6)
    assert len(finished.parts) == len(baseline.parts)


@pytest.mark.parametrize('seed', SEEDS[:4])
def test_draft_repair_seeded_pipe_span_noise(seed, factory):
    rng = random.Random(seed+3000)
    angles = [rng.uniform(-30, 30), rng.uniform(-30, 30), rng.uniform(-180, 180)]
    rotation = Rotation.from_euler('xyz', angles, degrees=True)
    center = np.array([rng.uniform(500, 1500), rng.uniform(-700, 700), rng.uniform(500, 1200)])
    length = rng.uniform(450, 900)
    axis = rotation.apply([0, 0, 1])
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': f'Draft span fuzz {seed}',
           'parts': [_fitting('through', center, {'bore': _socket([0, 0, 1], through=True)})],
           'joints': [], 'draft_subassemblies': [{'id': 'span', 'runs': [
               {'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
                'start_mm': (center-axis*length/2).tolist(),
                'end_mm': (center+axis*length/2).tolist(),
                'attachments': [{'connector': 'through', 'port': 'bore'}]}]}]}
    doc['parts'][0]['pose']['rotation_deg'] = angles
    baseline = factory(doc)
    assert all(not run['conflicts'] for run in preview(baseline))
    assert finalize(baseline, check_collisions=False)['status'] == 'finalized'
    original = copy.deepcopy(doc)
    noisy_rotation = Rotation.from_euler('xyz', [rng.uniform(-.5, .5) for _ in range(3)], degrees=True)
    offset = np.array([rng.uniform(-2, 2) for _ in range(3)])
    span = doc['draft_subassemblies'][0]['runs'][0]
    for key in ('start_mm', 'end_mm'):
        span[key] = (center + offset + noisy_rotation.apply(np.array(span[key])-center)).tolist()
    disturbed = factory(doc)
    result = repair(disturbed)
    assert result['status'] in ('repaired', 'aligned'), (seed, result)
    restored = factory(result.get('document', doc))
    assert all(not run['conflicts'] for run in preview(restored)), seed
    exact = finalize(restored, check_collisions=False)
    assert exact['status'] == 'finalized', (seed, exact)
    assert validate(factory(exact['document']), collisions=False)['valid'], seed
    assert doc != original and baseline.doc == original


@pytest.mark.parametrize('seed', SEEDS)
def test_centered_mirror_follows_moved_socket_with_reversed_preview_span(seed, factory):
    rng = random.Random(seed+5000)
    mirror_axis = seed % 3
    transverse = (mirror_axis+1) % 3
    bound_end = 'start' if seed % 2 else 'end'
    side = 1 if seed % 4 < 2 else -1
    offset = rng.uniform(-250, 250)
    distance = rng.uniform(230, 650)
    direction = np.zeros(3)
    direction[mirror_axis] = -side
    direction[transverse] = rng.uniform(-.006, .006)
    direction /= np.linalg.norm(direction)
    point = np.array([70., -140., 320.])
    point[mirror_axis] = offset+side*distance
    length = 2*(offset-point[mirror_axis])/direction[mirror_axis]
    raw_start = point.copy(); raw_end = point.copy()
    raw_start[mirror_axis] = offset-length/2
    raw_end[mirror_axis] = offset+length/2
    if seed % 3 == 0:
        raw_start, raw_end = raw_end, raw_start
    insertion = 24.
    connector = _fitting('end-fit', point+direction*insertion,
                         {'bore': _socket(direction)})
    through_point = point+direction*min(180, length/3)
    through = _fitting('through-fit', through_point,
                       {'bore': _socket(direction, through=True)})
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': f'Mirror move fuzz {seed}', 'parts': [connector, through],
           'joints': [], 'draft_subassemblies': [{'id': 'frame', 'runs': [{
               'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
               'start_mm': raw_start.tolist(), 'end_mm': raw_end.tolist(),
               'attachments': [{'connector': 'end-fit', 'port': 'bore',
                                'end': bound_end, 'insertion_mm': insertion},
                               {'connector': 'through-fit', 'port': 'bore'}]}],
               'mirrors': [{'id': 'plane', 'axis': 'xyz'[mirror_axis],
                            'offset_mm': offset, 'run_modes': {'pipe': 'centered'}}]}]}
    baseline = factory(doc)
    assert not preview(baseline)[0]['conflicts'], seed
    movement = rng.uniform(-4, 4)
    moved_position = (np.array(connector['pose']['position_mm'])+
                      direction*movement).tolist()
    changed = move_document(baseline, {'end-fit': {'position_mm': moved_position}})
    restored = factory(changed)
    result = preview(restored)[0]
    assert not result['conflicts'], (seed, result['conflicts'])
    assert abs(result['pose']['position_mm'][mirror_axis]-offset) < .05, seed
    assert abs(result['length_mm']-(length-2*movement)) < .05, seed
    assert changed['draft_subassemblies'][0]['mirrors'][0]['run_modes']['pipe'] == 'centered'
