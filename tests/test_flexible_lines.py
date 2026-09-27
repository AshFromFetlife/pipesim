"""Catalog-driven flexible lines share the chain pose and physics mechanisms."""
import copy

import numpy as np
import pytest

from pipesim.chain import summary
from pipesim.geometry import collision_primitives, mesh_for_part
from pipesim.grouping import attach_part
from pipesim.physics import World, simulate
from pipesim.posing import transform_part
from pipesim.validation import validate


@pytest.mark.parametrize('catalog,profile,shape,width,mass,strength', [
    ('generic.chain-heavy-100', 'chain', 'tube', 100, 1.2, 60000),
    ('generic.rope-jute-6', 'rope', 'cylinder', 6, .0006, 350),
    ('generic.rope-nylon-10', 'rope', 'cylinder', 10, .0018, 2000),
    ('generic.rope-polypropylene-12', 'rope', 'cylinder', 12, .0025, 1200),
    ('generic.strap-ratchet-25', 'strap', 'box', 25, .001, 5000),
    ('generic.strap-seatbelt-65', 'strap', 'box', 65, .0035, 8000),
])
def test_profiles_drive_geometry_mass_and_strength(factory, blank, catalog, profile, shape, width, mass, strength):
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id': 'line', 'template': 'chain',
                       'parameters': {'length_mm': 80, 'link_catalog': catalog},
                       'pose': {'position_mm': [0, 0, 500]}}]
    assembly = factory(doc)
    info = summary(doc['objects'][0], assembly.library)
    assert info['profile'] == profile and info['break_force_n'] == strength
    assert info['break_strain'] == pytest.approx(strength / info['axial_rigidity_n'])
    assert len(assembly.parts) == info['count']
    assert len(assembly.joints) == info['count']-1
    assert all(part.mass == pytest.approx(mass) for part in assembly.parts.values())
    assert all(part.shapes[0]['type'] == shape for part in assembly.parts.values())
    for part in assembly.parts.values():
        collision = collision_primitives(part)[0][0]
        assert collision['type'] == ('box' if shape == 'tube' else shape)
        cross_section = part.shapes[0]
        assert (cross_section['size_mm'][0] if shape == 'box' else cross_section['diameter_mm']) == width
        assert mesh_for_part(part).extents[0] == pytest.approx(width, abs=.1)
    assert all(joint['break_force_n'] == strength for joint in assembly.joints)
    if profile == 'strap':
        first, second = list(assembly.parts.values())[:2]
        assert first.matrix[:3, 0] @ second.matrix[:3, 0] > .999
    report = validate(assembly, collisions=False)
    assert report['valid'], report['issues']


@pytest.mark.parametrize('catalog', ['generic.rope-jute-6', 'generic.strap-seatbelt-65'])
def test_rope_and_webbing_can_be_posed_with_chain_solver(factory, blank, catalog):
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id': 'line', 'template': 'chain', 'layout_mode': 'posable',
                       'parameters': {'length_mm': 200, 'link_catalog': catalog},
                       'pose': {'position_mm': [0, 0, 600]}}]
    assembly = factory(doc)
    last = f"line/link-{len(assembly.parts)}"
    target = {'position_mm': (assembly.parts[last].matrix[:3, 3]+[40, 0, 20]).tolist()}
    result = transform_part(assembly, last, target)
    assert result['position_error_mm'] < .1
    posed = factory(result['document'])
    assert validate(posed, collisions=False)['valid']


def test_rated_flexible_joint_breaks_under_load(factory, blank):
    doc = copy.deepcopy(blank)
    profile = copy.deepcopy(factory(blank).library.parts['generic.rope-jute-6'])
    profile['break_force_n'] = .01
    doc['definitions'] = {'test.fine-rope': profile}
    doc['environment'] = {'ground': False}
    doc['objects'] = [{'id': 'line', 'template': 'chain',
                       'parameters': {'length_mm': 40, 'link_catalog': 'test.fine-rope'},
                       'pose': {'position_mm': [0, 0, 500]}}]
    doc['anchors'] = [{'part': 'line/link-1', 'surface': 'fixture'}]
    doc['loads'] = [{'part': 'line/link-2', 'force_n': [0, 0, -20]}]
    result = simulate(factory(doc), duration=.04, fps=25)
    assert any(event['type'] == 'joint_break' and event['joint'] == 'line/join-1'
               for event in result['events'])
    assert 'line/join-1' in result['frames'][-1]['broken_joints']


def test_light_rope_hangs_without_diverging(factory, blank):
    doc = copy.deepcopy(blank)
    doc['environment'] = {'ground': False}
    doc['objects'] = [{'id': 'line', 'template': 'chain',
                       'parameters': {'length_mm': 400, 'link_catalog': 'generic.rope-jute-6'},
                       'pose': {'position_mm': [0, 0, 700], 'rotation_deg': [0, 35, 0]}}]
    doc['anchors'] = [{'part': 'line/link-1', 'surface': 'fixture'}]
    result = simulate(factory(doc), duration=.15, fps=20)
    start = np.array(result['frames'][0]['parts']['line/link-20']['position_mm'])
    end = np.array(result['frames'][-1]['parts']['line/link-20']['position_mm'])
    assert np.isfinite(end).all() and np.linalg.norm(end-start) < 1000
    assert not any(event['type'] == 'joint_break' for event in result['events'])


@pytest.mark.parametrize('catalog,length,rigidity', [
    ('generic.rope-jute-6', 160, 30000),
    ('generic.strap-ratchet-25', 200, 150000),
])
def test_two_ended_line_transfers_profile_tension_and_breaks(factory, blank, catalog, length, rigidity):
    doc = copy.deepcopy(blank)
    profile = copy.deepcopy(factory(blank).library.parts[catalog])
    profile['break_force_n'] = 2
    doc['definitions'] = {'test.webbing': profile}
    doc['environment'] = {'ground': False}
    doc['objects'] = [{'id': 'line', 'template': 'chain',
                       'parameters': {'length_mm': length, 'link_catalog': 'test.webbing'}}]
    doc['parts'] = [{'id': 'top', 'catalog': 'generic.d-link',
                     'pose': {'position_mm': [0, 0, 600]}},
                    {'id': 'bottom', 'catalog': 'generic.d-link',
                     'pose': {'position_mm': [0, 0, 600-length]}}]
    doc['anchors'] = [{'part': 'top', 'surface': 'fixture'}]
    first = factory(attach_part(factory(doc), 'line', 'line/link-1',
                                {'part': 'top', 'port': 'eye'}, 'spherical')['document'])
    second = factory(attach_part(first, 'line', 'line/link-8',
                                 {'part': 'bottom', 'port': 'eye'}, 'spherical')['document'])
    assert validate(second, collisions=False)['valid']
    with World(second) as world:
        assert len(world._chain_spans) == 1
        assert world._chain_spans[0][-1] == pytest.approx(rigidity / (length/1000))
    second.doc['loads'] = [{'part': 'bottom', 'force_n': [0, 0, -300]}]
    result = simulate(factory(second.doc), duration=.12, fps=20)
    assert any(event['type'] == 'joint_break' and event['joint'].startswith('line/join-')
               for event in result['events'])
