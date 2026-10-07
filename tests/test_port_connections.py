import copy

import numpy as np
import pytest

from pipesim.document import DocumentError
from pipesim.port_connections import connect_ports
from pipesim.validation import validate


def hinge_pair(blank):
    doc = copy.deepcopy(blank)
    doc['parts'] = [
        {'id': 'male', 'catalog': 'tubeclamp.TC173MC',
         'pose': {'position_mm': [0, 0, 1000]}},
        {'id': 'female', 'catalog': 'tubeclamp.TC173FC',
         'pose': {'position_mm': [80, 0, 900], 'rotation_deg': [0, 0, 90]}},
        {'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
         'parameters': {'length_mm': 400},
         'pose': {'position_mm': [80, 0, 500]}}]
    doc['joints'] = [{'id': 'female-pipe', 'type': 'fixed',
                      'a': {'part': 'female', 'frame': {'position_mm': [0, 0, 0]}},
                      'b': {'part': 'pipe', 'frame': {'position_mm': [0, 0, 400]}}}]
    doc['anchors'] = [{'part': 'male', 'surface': 'fixture'}]
    return doc


def test_dragged_hinge_ports_snap_exactly_and_carry_attached_pipe(factory, blank):
    before = factory(hinge_pair(blank))
    a = {'part': 'male', 'port': 'hinge'}
    b = {'part': 'female', 'port': 'hinge'}
    result = connect_ports(before, a, b)
    after = factory(result['document'])
    assert result['move'] == 'b'
    assert result['gap_mm'] > 30
    assert result['joint']['type'] == 'revolute'
    pa, pb, axis = after.joint_frames(result['joint'])
    assert np.linalg.norm(pa-pb) < 1e-6
    assert abs(axis @ after.parts['female'].frame(b)[1]) > 1-1e-6
    assert np.allclose(after.parts['male'].matrix, before.parts['male'].matrix)
    assert not np.allclose(after.parts['pipe'].matrix, before.parts['pipe'].matrix)
    assert np.allclose(np.linalg.inv(after.parts['female'].matrix)
                       @ after.parts['pipe'].matrix,
                       np.linalg.inv(before.parts['female'].matrix)
                       @ before.parts['pipe'].matrix)


def test_hinge_port_cannot_be_claimed_twice(factory, blank):
    before = factory(hinge_pair(blank))
    a, b = ({'part': pid, 'port': 'hinge'} for pid in ('male', 'female'))
    after = factory(connect_ports(before, a, b)['document'])
    with pytest.raises(DocumentError, match='already connected'):
        connect_ports(after, a, b)


def test_male_half_moves_when_female_rigid_body_is_anchored(factory, blank):
    doc = hinge_pair(blank)
    doc['anchors'] = [{'part': 'female', 'surface': 'fixture'}]
    before = factory(doc)
    a, b = ({'part': pid, 'port': 'hinge'} for pid in ('male', 'female'))
    result = connect_ports(before, a, b)
    after = factory(result['document'])
    assert result['move'] == 'a'
    assert np.allclose(after.parts['female'].matrix, before.parts['female'].matrix)
    assert np.allclose(after.parts['pipe'].matrix, before.parts['pipe'].matrix)
    assert not np.allclose(after.parts['male'].matrix, before.parts['male'].matrix)
    pa, pb, _ = after.joint_frames(result['joint'])
    assert np.linalg.norm(pa-pb) < 1e-6


def test_hinge_bolt_holes_must_have_compatible_diameters(factory, blank):
    doc = hinge_pair(blank)
    doc['parts'][0]['catalog'] = 'tubeclamp.TC173MA'
    before = factory(doc)
    a, b = ({'part': pid, 'port': 'hinge'} for pid in ('male', 'female'))
    with pytest.raises(DocumentError, match='diameters do not match'):
        connect_ports(before, a, b)


def test_two_hinge_eyes_do_not_form_a_hinge(factory, blank):
    doc = hinge_pair(blank)
    doc['parts'][1]['catalog'] = 'tubeclamp.TC173MC'
    before = factory(doc)
    a, b = ({'part': pid, 'port': 'hinge'} for pid in ('male', 'female'))
    with pytest.raises(DocumentError, match='one bolt eye and one matching clevis'):
        connect_ports(before, a, b)


def test_aligned_hinge_pair_validates_as_an_assembly(factory, blank):
    doc = hinge_pair(blank)
    doc['parts'] = doc['parts'][:2]
    doc['joints'] = []
    before = factory(doc)
    a, b = ({'part': pid, 'port': 'hinge'} for pid in ('male', 'female'))
    result = connect_ports(before, a, b)
    report = validate(factory(result['document']))
    assert report['valid'], report['issues']
