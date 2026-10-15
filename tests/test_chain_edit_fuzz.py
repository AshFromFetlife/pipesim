"""Randomized topology edits for chain, rope and webbing lines."""
import copy
import random
import time

import numpy as np

from fuzz_runtime import budget
from pipesim.chain import _fit_span, dimensions, generate
from pipesim.chain_edit import adjust_span, cut, splice
from pipesim.math3d import align_axis, pose_of


PROFILES = ('generic.chain-link', 'generic.rope-jute-6',
            'generic.rope-nylon-10', 'generic.strap-seatbelt-65')


def _case(seed, factory, blank, library):
    rng = random.Random(seed)
    profile = rng.choice(PROFILES)
    count = rng.randint(8, 35)
    pitch = dimensions({'length_mm': 100, 'link_catalog': profile}, library)['pitch_mm']
    parameters = {'length_mm': count*pitch, 'link_catalog': profile}
    components = generate(parameters, library)
    info = dimensions(parameters, library)
    # The two attached ends leave enough slack to remove an interior segment.
    end = np.array([count*pitch*rng.uniform(.25, .55), 0., 0.])
    nodes = _fit_span(np.linspace(np.zeros(3), end, count+1),
                      np.full(count, pitch), np.zeros(3), end)
    for index, part in enumerate(components['parts']):
        matrix = np.eye(4)
        matrix[:3, :3] = align_axis(info['a']-info['b'], nodes[index+1]-nodes[index])
        matrix[:3, 3] = nodes[index]-matrix[:3, :3]@info['b']
        part['pose'] = pose_of(matrix)
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id':'line', 'template':'chain', 'parameters':parameters,
                       'components':components, 'pose':{'position_mm':[0,0,500]}}]
    doc['anchors'] = [{'part':'line/link-1','surface':'fixture'},
                      {'part':f'line/link-{count}','surface':'fixture'}]
    before = factory(doc)
    saved_start = before.parts['line/link-1'].matrix.copy()
    saved_end = before.parts[f'line/link-{count}'].matrix.copy()
    selected = rng.randint(3, count-2)
    change = rng.choice((-1, 1))
    edited = adjust_span(before, 'line', selected, change)
    assert edited['changed'], (seed, profile, count, selected, change, edited['message'])
    after = factory(edited['document'])
    assert np.allclose(after.parts['line/link-1'].matrix, saved_start, atol=1e-4), seed
    assert np.allclose(after.parts[f'line/link-{count+change}'].matrix, saved_end, atol=1e-4), seed
    for joint in after.joints:
        a, b, _ = after.joint_frames(joint)
        assert np.linalg.norm(a-b) < 1e-4, (seed, joint['id'])
    split_at = rng.randint(2, count+change-2)
    split = cut(after, 'line', split_at)
    separated = factory(split['document'])
    first_end = rng.choice(('start', 'end'))
    second_end = rng.choice(('start', 'end'))
    rejoined = factory(splice(separated, 'line', split['tail'],
                              first_end, second_end)['document'])
    assert len(rejoined.parts) >= count+change, seed
    assert len(rejoined.anchors) == 2, seed
    assert all(anchor['part'].startswith('line/link-') for anchor in rejoined.anchors), seed
    for joint in rejoined.joints:
        a, b, _ = rejoined.joint_frames(joint)
        assert np.linalg.norm(a-b) < 1e-4, (seed, joint['id'])


def test_chain_edit_fuzz(factory, blank, library):
    _, _, deadline = budget(default_minutes=15)
    seed = 0
    while seed < 24 or time.monotonic() < deadline:
        _case(seed, factory, blank, library)
        seed += 1
