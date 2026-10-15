"""Randomized flexible-line presets across sizes, profiles and fixed links."""

import random
import secrets
import time
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from pipesim.chain_poses import PRESETS, apply_preset
from pipesim.document import Assembly
from pipesim.grouping import attach_part
from fuzz_runtime import budget


PROFILES = ('generic.chain-link', 'generic.rope-nylon-10', 'generic.strap-seatbelt-65')
MIRRORED_ATTACHMENT = Path(__file__).parent / 'fixtures' / 'chain_semi_circle_attachment.json'


def _mirrored_attachment_case(seed, base, library):
    """A mirrored link sharing the chain prefix must never force general IK."""
    rng = random.Random(seed)
    doc = copy.deepcopy(json.loads(MIRRORED_ATTACHMENT.read_text()))
    target = next(part for part in doc['parts'] if part['id'] == 'tc173mc-1-mirror-mirror-1')
    target['pose']['position_mm'] = [value + rng.uniform(-2, 2)
                                      for value in target['pose']['position_mm']]
    mirror = next(part for part in doc['parts'] if part['id'] == 'chain-1/link-1-mirror-mirror-1')
    for index in range(rng.randrange(5)):
        decoy = copy.deepcopy(mirror)
        decoy['id'] = f'chain-1/link-{index + 2}-mirror-fuzz-{seed}'
        doc['parts'].append(decoy)
    assembly = Assembly.from_doc(doc, base, library)

    def slow_path(*args, **kwargs):
        pytest.fail(f'Mirrored chain attachment fell back to general IK: seed={seed}')
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr('pipesim.posing.transform_part', slow_path)
        patch.setattr('pipesim.posing.commit_transform', slow_path)
        result = attach_part(assembly, 'chain-1', 'chain-1/link-1',
                             {'part': target['id'], 'port': 'hinge'},
                             'spherical', part_port='b')
    after = Assembly.from_doc(result['document'], base, library)
    assert len(result['moved']) == 30, seed
    assert len(after.doc['objects'][0]['components']['parts']) == 30, seed
    assert 'chain-1/link-1-mirror-mirror-1' in after.parts, seed
    first, second, _ = after.joint_frames(result['joint'])
    assert np.linalg.norm(first - second) < .5, seed


def _case(seed, base, library):
    rng = random.Random(seed)
    profile = rng.choice(PROFILES)
    definition = library.parts[profile]
    a, b = (np.array(definition['ports'][port]['position_mm']) for port in ('a', 'b'))
    pitch = float(np.linalg.norm(a - b))
    paired_ends = seed % 5 == 0
    count = rng.randint(12 if paired_ends else 3, 90)
    preset = 'u-deep' if paired_ends else rng.choice(tuple(PRESETS))
    anchor = None if paired_ends else rng.choice((None, None, 1, count // 2, count))
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': f'Line preset fuzz {seed}',
           'parts': [], 'joints': [],
           'anchors': ([{'part': f'line/link-{anchor}', 'surface': 'fixture'}] if anchor else []),
           'objects': [{'id': 'line', 'template': 'chain',
                        'parameters': {'length_mm': count * pitch, 'link_catalog': profile},
                        'pose': {'position_mm': [rng.uniform(-500, 500), rng.uniform(-500, 500), 1200],
                                 'rotation_deg': [rng.uniform(-60, 60), rng.uniform(-60, 60),
                                                  rng.uniform(-180, 180)]}}]}
    before = Assembly.from_doc(doc, base, library)
    if paired_ends:
        # A hanging span has slack between its separated endpoints. The U
        # preset must change its free middle without shifting either fixing.
        prepared = apply_preset(before, 'line', 'hanging-arc')
        doc = prepared['document']
        doc['anchors'] = [{'part': 'line/link-1', 'surface': 'fixture'},
                          {'part': f'line/link-{count}', 'surface': 'fixture'}]
        before = Assembly.from_doc(doc, base, library)
    result = apply_preset(before, 'line', preset)
    after = Assembly.from_doc(result['document'], base, library)
    if paired_ends:
        assert result['changed'], (seed, count, 'preset lost the available bend')
    assert len(after.parts) == count, (seed, preset, count)
    for joint in after.joints:
        first, second, _ = after.joint_frames(joint)
        assert np.linalg.norm(first - second) < .05, (seed, preset, joint['id'])
    for number in ((1, count) if paired_ends else (anchor,) if anchor else ()):
        name = f'line/link-{number}'
        assert np.allclose(after.parts[name].matrix, before.parts[name].matrix,
                           atol=1e-5, rtol=0), (seed, preset, name)
    if result['changed'] and not anchor and not result['limited'] and count >= 5:
        start = after.parts['line/link-1'].frame({'port': 'b'})[0]
        end = after.parts[f'line/link-{count}'].frame({'port': 'a'})[0]
        if preset == 'circle-touch':
            assert np.linalg.norm(end - start) < .05, (seed, count)


@pytest.mark.parametrize('seed', range(40))
def test_seeded_line_presets(seed, tmp_path, library):
    _case(seed, tmp_path, library)


def test_long_randomized_line_presets(tmp_path, library):
    root = secrets.randbits(64)
    minutes, started, deadline = budget(default_minutes=5)
    rng = random.Random(root)
    completed = 0
    print(f'line pose fuzz root_seed={root} budget_minutes={minutes}', flush=True)
    while time.monotonic() < deadline:
        seed = rng.getrandbits(64)
        try:
            if completed % 5 == 0:
                _mirrored_attachment_case(seed, tmp_path, library)
            else:
                _case(seed, tmp_path, library)
        except Exception as exc:
            raise AssertionError(f'line pose fuzz root_seed={root} case_seed={seed} '
                                 f'completed={completed}') from exc
        completed += 1
    print(f'line pose fuzz completed={completed}', flush=True)
    assert completed > 0
