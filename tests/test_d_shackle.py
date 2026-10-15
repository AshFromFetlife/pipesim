"""The D-shackle is a pin-and-bow connector, not the old ring proxy."""

import copy

import numpy as np
import pytest

from pipesim.document import Assembly, DocumentError, Library
from pipesim.geometry import mesh_for_part
from pipesim.port_connections import connect_ports, shackle_pin_hole
from pipesim.quick_connections import attach_nearby
from pipesim.validation import validate


def design(*parts):
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
            'name': 'D-shackle', 'parts': list(parts), 'joints': []}


def test_low_poly_shackle_catalog_and_legacy_id():
    library = Library.load()
    current = library.parts['generic.d-shackle']
    legacy = library.parts['generic.d-link']
    assert legacy['geometry'] == current['geometry']
    assert legacy['ports']['eye']['position_mm'] == [0, 0, 0]
    assert legacy['ports']['eye']['axis'] == [0, 1, 0]
    assert current['name'].startswith('D shackle')
    assert {port for port in current['ports']} == {'eye', 'pin_hook', 'pin_bolt'}
    assembly = Assembly.from_doc(design({'id': 'shackle', 'catalog': 'generic.d-shackle'}))
    part = assembly.parts['shackle']
    mesh = mesh_for_part(part)
    assert len(part.shapes) == 15
    assert len(mesh.faces) < 1000
    assert mesh.bounds[0, 0] < -20 and mesh.bounds[1, 0] > 20
    assert mesh.bounds[0, 2] < -18 and mesh.bounds[1, 2] > 25
    assert 0 < part.mass < .5


def test_shackle_bow_attaches_to_chain_eye_as_a_ball_joint():
    before = Assembly.from_doc(design(
        {'id': 'shackle', 'catalog': 'generic.d-shackle',
         'pose': {'position_mm': [20, 0, 20]}},
        {'id': 'link', 'catalog': 'generic.chain-link'}))
    result = connect_ports(before, {'part': 'shackle', 'port': 'eye'},
                           {'part': 'link', 'port': 'a'}, 'spherical', move='a')
    after = Assembly.from_doc(result['document'])
    assert result['joint']['type'] == 'spherical'
    pa, pb, _ = after.joint_frames(result['joint'])
    assert np.linalg.norm(pa - pb) < 1e-6
    assert np.allclose(after.parts['link'].matrix, before.parts['link'].matrix)
    assert validate(after)['issues'] == []


def test_legacy_shackle_eye_keeps_its_saved_frame_without_a_false_collision():
    before = Assembly.from_doc(design(
        {'id': 'shackle', 'catalog': 'generic.d-link',
         'pose': {'position_mm': [20, 0, 20]}},
        {'id': 'link', 'catalog': 'generic.chain-link'}))
    result = connect_ports(before, {'part': 'shackle', 'port': 'eye'},
                           {'part': 'link', 'port': 'a'}, 'spherical', move='a')
    after = Assembly.from_doc(result['document'])
    assert not any(issue['code'] == 'INTERSECTION' for issue in validate(after)['issues'])


def test_shackle_pin_attaches_to_bolt_eye_and_excludes_another_pin_connection():
    before = Assembly.from_doc(design(
        {'id': 'shackle', 'catalog': 'generic.d-shackle',
         'pose': {'position_mm': [40, 0, 20]}},
        {'id': 'eye', 'catalog': 'tubeclamp.TC173MC'}))
    result = connect_ports(before, {'part': 'shackle', 'port': 'pin_bolt'},
                           {'part': 'eye', 'port': 'hinge'}, 'revolute', move='a')
    after = Assembly.from_doc(result['document'])
    assert result['joint']['type'] == 'revolute'
    pa, pb, _ = after.joint_frames(result['joint'])
    assert np.linalg.norm(pa - pb) < 1e-6
    assert np.allclose(after.parts['eye'].matrix, before.parts['eye'].matrix)
    another = copy.deepcopy(result['document'])
    pin, _ = after.parts['shackle'].frame({'port': 'pin_hook'})
    another['parts'].append({'id': 'link', 'catalog': 'generic.chain-link',
                             'pose': {'position_mm': (pin + [0, 0, 10]).tolist()}})
    with pytest.raises(DocumentError, match='already connected'):
        connect_ports(Assembly.from_doc(another),
                      {'part': 'shackle', 'port': 'pin_hook'},
                      {'part': 'link', 'port': 'a'}, 'spherical', move='a')
    nearby = attach_nearby(Assembly.from_doc(another), 'shackle')
    assert nearby['count'] == 0


def test_shackle_pin_fits_catalogue_bolt_hole_but_not_undersized_hole():
    before = Assembly.from_doc(design(
        {'id': 'shackle', 'catalog': 'generic.d-shackle',
         'pose': {'position_mm': [40, 0, 20]}},
        {'id': 'flange', 'catalog': 'tubeclamp.TC131T'}))
    pin = before.parts['shackle'].ports['pin_bolt']
    hole = before.parts['flange'].ports['bolt1']
    assert pin['diameter_mm'] == hole['diameter_mm'] == 8
    result = connect_ports(before, {'part': 'shackle', 'port': 'pin_bolt'},
                           {'part': 'flange', 'port': 'bolt1'}, 'revolute', move='a')
    after = Assembly.from_doc(result['document'])
    pa, pb, _ = after.joint_frames(result['joint'])
    assert np.linalg.norm(pa - pb) < 1e-6
    assert not shackle_pin_hole(pin, {**hole, 'diameter_mm': 7})
    assert not shackle_pin_hole(pin, {**hole, 'diameter_mm': 12})
