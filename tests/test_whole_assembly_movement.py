"""Whole-object layout is a rigid placement, including connected flexible objects."""
import copy

import numpy as np
import pytest

from pipesim.document import DocumentError
from pipesim.grouping import move_object, set_object_layout
from pipesim.math3d import pose_of, transform
from pipesim.posing import transform_part


def crossed_chains(factory, blank, kind='spherical'):
    doc = copy.deepcopy(blank)
    doc['objects'] = [
        {'id': name, 'template': 'chain', 'parameters': {'length_mm': 1000},
         'layout_mode': 'posable',
         'pose': {'position_mm': [0, 0, 1400], 'rotation_deg': [0, angle, 0]}}
        for name, angle in [('left', -35), ('right', 35), ('unconnected', 0)]]
    assembly = factory(doc)
    a, b = {'part': 'left/link-25', 'port': 'a'}, {'part': 'right/link-25', 'port': 'a'}
    shift = assembly.parts[a['part']].frame(a)[0] - assembly.parts[b['part']].frame(b)[0]
    doc['objects'][1]['pose']['position_mm'] = (np.array([0, 0, 1400]) + shift).tolist()
    doc['joints'] = [{'id': 'midpoint', 'type': kind, 'a': a, 'b': b}]
    return factory(doc)


@pytest.mark.parametrize('kind', ['spherical', 'revolute', 'fixed'])
@pytest.mark.parametrize('shape', ['straight', 'posed', 'saved-state'])
def test_whole_chain_carries_free_midpoint_attachment(factory, blank, kind, shape, monkeypatch):
    before = crossed_chains(factory, blank, kind)
    if shape == 'posed':
        target = pose_of(before.parts['left/link-50'].matrix)
        target['position_mm'][0] += 30
        target['position_mm'][2] += 40
        before = factory(transform_part(before, 'left/link-50', target)['document'])
    elif shape == 'saved-state':
        doc = copy.deepcopy(before.doc)
        doc['state'] = {'joints': {'left/join-40': {'rotation_deg': [10, 0, 0]}}}
        before = factory(doc)
    before = factory(set_object_layout(before, 'left', 'rigid'))
    def unexpected(*args, **kwargs):
        raise AssertionError('Whole-object layout must not solve internal joints or rebake object shapes')
    monkeypatch.setattr('pipesim.posing.Mechanism.__init__', unexpected)
    if shape != 'saved-state':
        monkeypatch.setattr('pipesim.posing._editable', unexpected)
    original = copy.deepcopy(before.doc)
    target = {'position_mm': [-4000, 7000, 2200], 'rotation_deg': [24, -18, 120]}
    delta = transform(target) @ np.linalg.inv(transform(before.doc['objects'][0]['pose']))
    expected = {pid for pid in before.parts if not pid.startswith('unconnected/')}
    preview = move_object(before, 'left', target, preview=True)
    result = move_object(before, 'left', target)
    after = factory(result['document'])
    assert set(preview['moved']) == set(result['moved']) == expected
    assert not result['limited'] and not preview['limited']
    for pid, part in before.parts.items():
        desired = delta @ part.matrix if pid in expected else part.matrix
        np.testing.assert_allclose(after.parts[pid].matrix, desired, atol=1e-6, rtol=0)
        if pid in expected:
            np.testing.assert_allclose(transform(preview['poses'][pid]), desired, atol=1e-6, rtol=0)
    assert before.doc == original
    assert {o['id'] for o in after.doc['objects']} == {'left', 'right', 'unconnected'}
    assert not after.doc['parts']
    assert len(after.joints) == len(before.joints)
    for joint in after.joints:
        a, b, _ = after.joint_frames(joint)
        assert np.linalg.norm(a-b) < .05, joint['id']
    if shape != 'saved-state':
        assert after.doc['joints'] == before.doc['joints']
        for old, new in zip(before.doc['objects'], after.doc['objects']):
            assert {k: v for k, v in old.items() if k != 'pose'} == {k: v for k, v in new.items() if k != 'pose'}


@pytest.mark.parametrize('preview', [False, True])
def test_connected_world_anchor_still_prevents_whole_assembly_translation(factory, blank, preview):
    before = crossed_chains(factory, blank)
    doc = copy.deepcopy(before.doc)
    doc['anchors'] = [{'part': 'right/link-1', 'surface': 'fixture'}]
    before = factory(doc)
    with pytest.raises(DocumentError):
        move_object(before, 'left', {'position_mm': [4000, 0, 0]}, preview=preview)
    assert before.doc == doc
