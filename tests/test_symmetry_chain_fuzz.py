"""Mirrored flexible lines stay grouped, with no copy across their own plane."""

import copy
import json
import random
import secrets
import time
from pathlib import Path

import pytest

from fuzz_runtime import budget
from pipesim.deletion import delete_parts
from pipesim.document import Assembly
from pipesim.symmetry import materialize_mirror


SOURCE = Path(__file__).parent / 'fixtures' / 'chain_semi_circle_attachment.json'


def _case(seed, base, library):
    rng = random.Random(seed)
    mode = seed % 3
    offset = rng.uniform(-40, 40)
    if mode == 1:
        saved = json.loads(SOURCE.read_text())
        source = copy.deepcopy(saved['objects'][0])
        source['pose']['position_mm'][0] += rng.uniform(-20, 20)
        count = 30
    else:
        count = rng.randint(4, 40)
        source = {'id': 'chain', 'template': 'chain',
                  'parameters': {'length_mm': count * 20, 'link_catalog': 'generic.chain-link'},
                  'pose': {'position_mm': [offset if mode == 2 else offset + rng.uniform(80, 400),
                                           rng.uniform(-100, 100), rng.uniform(300, 800)]}}
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': f'Chain mirror fuzz {seed}',
           'parts': [], 'joints': [], 'objects': [source],
           'draft_subassemblies': [{'id': 'scene', 'runs': [],
                                    'mirrors': [{'id': 'plane', 'axis': 'x',
                                                 'offset_mm': offset, 'scope': 'scene'}]}]}
    original = Assembly.from_doc(doc, base, library)
    baked = materialize_mirror(original, 'scene', 'plane')
    exact = Assembly.from_doc(baked, base, library)
    if mode in (1, 2):
        assert len(baked['objects']) == 1, (seed, mode)
        assert len(exact.parts) == count, (seed, mode)
    else:
        assert len(baked['objects']) == 2, seed
        clone = baked['objects'][1]['id']
        assert len(baked['objects'][1]['components']['parts']) == count, seed
        assert len(exact.parts) == 2 * count, seed
        assert len(exact.joints) == 2 * (count - 1), seed
        removed = delete_parts(exact, object_id=clone)
        remaining = Assembly.from_doc(removed['document'], base, library)
        assert len(remaining.parts) == count, seed
        assert len(remaining.doc['objects']) == 1, seed


@pytest.mark.parametrize('seed', range(15))
def test_seeded_chain_mirror_topologies(seed, tmp_path, library):
    _case(seed, tmp_path, library)


def test_long_randomized_chain_mirrors(tmp_path, library):
    root = secrets.randbits(64)
    minutes, _, deadline = budget(default_minutes=5)
    rng = random.Random(root)
    completed = 0
    print(f'chain mirror fuzz root_seed={root} budget_minutes={minutes}', flush=True)
    while time.monotonic() < deadline:
        seed = rng.getrandbits(64)
        try:
            _case(seed, tmp_path, library)
        except Exception as exc:
            raise AssertionError(f'chain mirror fuzz root_seed={root} case_seed={seed} '
                                 f'completed={completed}') from exc
        completed += 1
    print(f'chain mirror fuzz completed={completed}', flush=True)
    assert completed >= 3
