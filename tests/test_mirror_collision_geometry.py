"""Reflection must preserve hollow collision shapes as well as visible geometry."""
import copy
import json
import os
import random
import secrets

import pytest

from pipesim.document import Assembly
from pipesim.drafting import finalize
from pipesim.math3d import transform
from pipesim.symmetry import materialize_all
from pipesim.validation import validate


SEEDS = tuple(int(s) for s in os.environ.get('PIPESIM_REALWORLD_MIRROR_CASE_SEEDS', '').split(',') if s)
SEEDS = SEEDS or (2687427251, 0, 1, secrets.randbits(64), secrets.randbits(64))


def offset_socket_document(seed):
    rng = random.Random(seed)
    pose = {'position_mm': [rng.uniform(-900, 900), rng.uniform(-900, 900), 2500],
            'rotation_deg': [rng.uniform(-180, 180) for _ in range(3)]}
    left, right = rng.uniform(100, 800), rng.uniform(100, 800)
    axis = rng.choice('xyz')
    if seed == 2687427251:
        pose = {'position_mm': [-634.94801, -93.97676, 2549.35307],
                'rotation_deg': [-43.4108, 6.503, 140.60799]}
        left, right, axis = 190, 556.718172, 'z'
    matrix = transform(pose)
    ends = [(matrix @ [x, 42.4, 0, 1])[:3].tolist() for x in (right, -left)]
    if seed % 2 == 0:
        ends.reverse()
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': f'Offset socket {seed}',
            'parts': [{'id': 'cross', 'catalog': 'tubeclamp.TC161C', 'pose': pose}],
            'joints': [], 'draft_subassemblies': [{'id': 'frame', 'runs': [{
                'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
                'start_mm': ends[0], 'end_mm': ends[1],
                'attachments': [{'connector': 'cross', 'port': 'cross'}]}],
                'mirrors': [{'id': 'plane', 'axis': axis, 'offset_mm': -10000}]}]}


@pytest.mark.parametrize('case_seed', SEEDS)
def test_realworld_offset_socket_reflection(case_seed, tmp_path, library):
    document = offset_socket_document(case_seed)
    original = copy.deepcopy(document)
    original['draft_subassemblies'][0].pop('mirrors')
    assert finalize(Assembly.from_doc(original, tmp_path, library))['status'] == 'finalized'
    try:
        baked = materialize_all(Assembly.from_doc(document, tmp_path, library))
        result = finalize(Assembly.from_doc(baked, tmp_path, library))
        assert result['status'] == 'finalized', result
        exact = Assembly.from_doc(result['document'], tmp_path, library)
        assert len(exact.parts) == 4 and len(exact.joints) == 2
        assert validate(exact)['valid']
        # A real obstruction must remain detectable on both reflected sides.
        obstructed = copy.deepcopy(result['document'])
        for name in ('cross', 'cross-mirror-plane'):
            part = exact.parts[name]
            center = (part.matrix @ [0, 42.4, 0, 1])[:3].tolist()
            obstructed['parts'].append({'id': f'obstacle-{name}', 'catalog': 'generic.box',
                'parameters': {'width_mm': 60, 'depth_mm': 60, 'height_mm': 60, 'mass_kg': 1},
                'pose': {'position_mm': center}})
        issues = validate(Assembly.from_doc(obstructed, tmp_path, library))['issues']
        for pipe, obstacle in [('pipe', 'obstacle-cross'), ('pipe-mirror-plane', 'obstacle-cross-mirror-plane')]:
            assert any(i['code'] == 'INTERSECTION' and set(i['parts']) == {pipe, obstacle} for i in issues)
    except Exception as error:
        path = tmp_path / f'offset-socket-{case_seed}.json'
        path.write_text(json.dumps({'case_seed': case_seed, 'document': document}, indent=2))
        pytest.fail(f'case_seed={case_seed}; repro={path}: {error}')
