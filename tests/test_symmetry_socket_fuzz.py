"""Fuzz mirror baking around nearly symmetric, on-plane corner sockets."""

import os
import random
import secrets

from pipesim.document import Assembly
from pipesim.drafting import finalize
from pipesim.symmetry import materialize_mirror


def _case(base, seed):
    rng = random.Random(seed)
    offset = rng.uniform(-1000, 1000)
    center = [offset + rng.uniform(-.015, .015), rng.uniform(-500, 500),
              rng.uniform(300, 1500)]
    corner = {'id': 'corner', 'catalog': 'tubeclamp.TC116C', 'pose': {
        'position_mm': center,
        'rotation_deg': [rng.uniform(-.005, .005), rng.uniform(-.005, .005),
                         -135 + rng.uniform(-.015, .015)]}}
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': 'On-plane socket mirror fuzz', 'parts': [corner], 'joints': [],
           'draft_subassemblies': [{'id': 'frame', 'runs': [], 'mirrors': [
               {'id': 'plane', 'axis': 'x', 'offset_mm': offset,
                'scope': 'scene'}]}]}
    source = Assembly.from_doc(doc, base)
    mouth, direction = source.parts['corner'].frame({'port': 'y'})
    start = mouth - direction * 30
    end = start + direction * rng.uniform(400, 1000)
    far_mouth = end - direction * 30
    doc['parts'].append({'id': 'far', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 5,
                      'position_mm': [0, 0, 80]}],
        'ports': {'socket': {'type': 'socket', 'position_mm': [0, 0, 0],
            'axis': (-direction).tolist(), 'diameter_mm': 42.4,
            'profile': 'round', 'engagement_mm': 40,
            'min_engagement_mm': 15}}},
        'pose': {'position_mm': far_mouth.tolist()}})
    doc['draft_subassemblies'][0]['runs'].append({
        'id': 'edge', 'catalog': 'tubeclamp.tube-C',
        'start_mm': start.tolist(), 'end_mm': end.tolist(),
        'attachments': [
            {'connector': 'corner', 'port': 'y', 'end': 'start',
             'insertion_mm': 30},
            {'connector': 'far', 'port': 'socket', 'end': 'end',
             'insertion_mm': 30}]})
    return doc


def _spanning_case(seed):
    rng = random.Random(seed)
    offset = rng.uniform(-1000, 1000)
    height = rng.uniform(300, 1500)
    insertion = rng.uniform(16, 30)
    span = rng.uniform(400, 1000)
    def fitting(identifier, x, direction):
        return {'id': identifier, 'body': {'kind': 'connector', 'mass_kg': .2,
            'geometry': [{'type': 'sphere', 'radius_mm': 5,
                          'position_mm': [0, 0, 80]}],
            'ports': {'end': {'type': 'socket', 'position_mm': [0, 0, 0],
                'axis': direction, 'diameter_mm': 42.4, 'profile': 'round',
                'engagement_mm': 40, 'min_engagement_mm': 15}}},
            'pose': {'position_mm': [x, 0, height]}}
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
        'name': 'Spanning socket mirror fuzz',
        'parts': [fitting('on-plane', offset, [1, 0, 0]),
                  fitting('far', offset+span, [-1, 0, 0])],
        'joints': [], 'draft_subassemblies': [{'id': 'frame', 'runs': [{
            'id': 'edge', 'catalog': 'tubeclamp.tube-C',
            'start_mm': [offset-insertion, 0, height],
            'end_mm': [offset+span+insertion, 0, height],
            'attachments': [
                {'connector': 'on-plane', 'port': 'end', 'end': 'start',
                 'insertion_mm': insertion},
                {'connector': 'far', 'port': 'end', 'end': 'end',
                 'insertion_mm': insertion}]}],
            'mirrors': [{'id': 'plane', 'axis': 'x', 'offset_mm': offset,
                         'scope': 'scene'}]}]}


def test_nearly_symmetric_socket_mirror_fuzz(tmp_path):
    root = int(os.environ.get('PIPESIM_MIRROR_SOCKET_FUZZ_SEED', secrets.randbits(64)))
    count = int(os.environ.get('PIPESIM_MIRROR_SOCKET_FUZZ_CASES', 40))
    rng = random.Random(root)
    seeds = [rng.getrandbits(64) for _ in range(count)]
    print(f'mirror socket fuzz root_seed={root}', flush=True)
    for index, seed in enumerate(seeds):
        doc = _case(tmp_path, seed)
        assembly = Assembly.from_doc(doc, tmp_path)
        warnings = []
        baked = materialize_mirror(assembly, 'frame', 'plane', warnings=warnings)
        assert not warnings, (index, seed, warnings)
        runs = baked['draft_subassemblies'][0]['runs']
        assert len(runs) == 2, (index, seed, runs)
        assert {a['port'] for run in runs for a in run['attachments']
                if a['connector'] == 'corner'} == {'x', 'y'}, (index, seed)
        exact = Assembly.from_doc(baked, tmp_path)
        if index % 10 == 0:
            result = finalize(exact)
            assert result['status'] == 'finalized', (index, seed, result)

    # A pipe extending just past the plane must retain its original span
    # when two different reflected connectors would claim its occupied ends.
    for index, seed in enumerate(seeds[:16]):
        doc = _spanning_case(seed)
        warnings = []
        baked = materialize_mirror(Assembly.from_doc(doc, tmp_path),
                                    'frame', 'plane', warnings=warnings)
        assert len(warnings) == 1 and warnings[0]['code'] == 'MIRROR_UNAVAILABLE', (index, seed, warnings)
        assert baked['draft_subassemblies'][0]['runs'] == doc['draft_subassemblies'][0]['runs'], (index, seed)
        if index % 8 == 0:
            result = finalize(Assembly.from_doc(baked, tmp_path))
            assert result['status'] == 'finalized', (index, seed, result)
