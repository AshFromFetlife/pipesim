"""Seeded geometry fuzzing for draft repair and exact socket search.

Rerun a failure with ``PIPESIM_FUZZ_SEEDS=<seed> pytest tests/test_solver_fuzz.py``.
The generated documents are intentionally small enough to inspect on failure.
"""

import copy
import math
import os
import random

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.drafting import finalize, preview, repair, runs
from pipesim.grouping import attach_part
from pipesim.snapping import connection_options
from pipesim.validation import validate


SEEDS = tuple(int(s) for s in os.environ.get('PIPESIM_FUZZ_SEEDS', ','.join(map(str, range(16, 32)))).split(','))


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
                                       [rng.uniform(-2, 2) for _ in range(3)]).tolist()
        part['pose']['rotation_deg'] = (np.array(part['pose']['rotation_deg'])+
                                       [rng.uniform(-.4, .4) for _ in range(3)]).tolist()
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
