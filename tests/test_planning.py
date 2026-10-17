"""Build sequences for closed frames and coordinated sliding subassemblies."""
import copy
import time

import numpy as np
import pytest

from pipesim import planning
from pipesim.document import Assembly
from pipesim.planning import plan_build
from pipesim.validation import validate
from planning_cases import (FIXTURES, assert_complete_plan, assert_gravity_support,
                            assert_sweep_samples,
                            anchored_cantilever, capture_frame, cube, fixture, ladder,
                            remote_bore_obstruction, rotated_split_clamp)


@pytest.mark.parametrize('seed', [0, 1, 2, 417])
def test_closed_cube_uses_coordinated_translation(seed, library):
    doc = cube(seed)
    original = copy.deepcopy(doc)
    assembly = Assembly.from_doc(doc, FIXTURES, library)
    assert validate(assembly, collisions=False)['valid']
    started = time.monotonic()
    result = plan_build(assembly)
    assert_complete_plan(assembly, result, grouped=True)
    assert_gravity_support(assembly, result)
    assert_sweep_samples(assembly, result)
    assert time.monotonic() - started < 30
    assert not result['certificate'].get('fallback_used', False)
    assert all(step.get('group_mass_kg', 0) <= 15 for step in result['steps'])
    assert doc == original


@pytest.mark.parametrize('levels,seed', [(1, 0), (4, 817), (7, 1911)])
def test_closed_short_rungs_slide_on_common_parallel_rails(levels, seed, library):
    assembly = Assembly.from_doc(ladder(seed, levels), FIXTURES, library)
    assert validate(assembly, collisions=False)['valid']
    result = plan_build(assembly)
    assert_complete_plan(assembly, result)
    assert_gravity_support(assembly, result)
    assert_sweep_samples(assembly, result)
    assert not result['certificate'].get('fallback_used', False)
    assert all(step.get('group_mass_kg', 0) <= 15 for step in result['steps'])


def test_reported_complicated_metal_frame_finishes_with_subassemblies(library):
    doc = fixture('complicated-metal')
    original = copy.deepcopy(doc)
    assembly = Assembly.from_doc(doc, FIXTURES, library)
    assert len(assembly.parts) == 218 and len(assembly.joints) == 284
    started = time.monotonic()
    result = plan_build(assembly)
    elapsed = time.monotonic() - started
    assert_complete_plan(assembly, result, grouped=True)
    assert_gravity_support(assembly, result)
    assert_sweep_samples(assembly, result)
    assert elapsed < 301
    assert result['certificate']['stability_checked']
    assert any(step.get('grounded_preparation') for step in result['steps'])
    assert any(step.get('kind') == 'lift' for step in result['steps'])
    assert all('poses' in step for step in result['steps'])
    assert doc == original


def test_rotated_real_frame_handles_line_contacts_in_preparation(library):
    # This exact fresh case found a 2-D support vector being passed to the 3-D
    # axis normalizer while choosing a floor orientation for a round member.
    from test_planning_fuzz import case_document
    doc = case_document(221317699909015487)
    original = copy.deepcopy(doc)
    assembly = Assembly.from_doc(doc, FIXTURES, library)
    result = plan_build(assembly)
    assert_complete_plan(assembly, result, grouped=True)
    assert_gravity_support(assembly, result)
    assert doc == original


def capped_socket(*, obstructed=False):
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
        'name': 'One-way capped socket', 'environment': {'ground': False},
        'build': {'require_stability': False,
                  'sequence': ['tube', 'fitting']},
        'parts': [
            {'id': 'tube', 'catalog': 'tubeclamp.tube-C',
             'parameters': {'length_mm': 200},
             'pose': {'position_mm': [0, 0, 70]}},
            {'id': 'fitting', 'body': {'kind': 'connector', 'mass_kg': 1,
             'geometry': [
                 {'type': 'tube', 'diameter_mm': 60, 'wall_mm': 8,
                  'length_mm': 60, 'position_mm': [0, 0, -30]},
                 {'type': 'cylinder', 'diameter_mm': 60, 'length_mm': 10,
                  'position_mm': [0, 0, -65]}],
             'ports': {'end': {'type': 'socket', 'profile': 'round',
                 'diameter_mm': 42.4, 'engagement_mm': 60,
                 'min_engagement_mm': 25, 'assembly': 'slide',
                 'through': False, 'position_mm': [0, 0, 0],
                 'axis': [0, 0, 1]}}}}],
        'joints': [{'id': 'screw', 'type': 'socket',
                    'a': {'part': 'fitting', 'port': 'end'},
                    'b': {'part': 'tube', 'end': 'start'},
                    'insertion_mm': 30, 'locked': True}]}
    if obstructed:
        doc['parts'].append({'id': 'obstacle', 'catalog': 'generic.box',
            'parameters': {'width_mm': 200, 'depth_mm': 200,
                           'height_mm': 1, 'mass_kg': 1},
            'pose': {'position_mm': [0, 0, -160.123]}})
        doc['build']['sequence'].insert(0, 'obstacle')
    return doc


def test_closed_socket_withdraws_only_through_its_mouth(library):
    assembly = Assembly.from_doc(capped_socket(), FIXTURES, library)
    result = plan_build(assembly)
    assert_complete_plan(assembly, result)
    fitting = next(step for step in result['steps'] if step['part'] == 'fitting')
    assert np.allclose(fitting['approach_direction'], [0, 0, 1])


def test_thin_unrelated_obstacle_blocks_capped_socket_sweep(library):
    assembly = Assembly.from_doc(capped_socket(obstructed=True), FIXTURES, library)
    result = plan_build(assembly)
    assert result['status'] == 'blocked', result
    assert 'fitting' in result['reason']
    assert not result['steps']


def test_planner_cancellation_does_not_publish_partial_instructions(library):
    assembly = Assembly.from_doc(fixture('complicated-metal'), FIXTURES, library)
    started = time.monotonic()
    result = plan_build(assembly, cancelled=lambda: True)
    assert result['status'] == 'cancelled'
    assert not result['steps']
    assert time.monotonic() - started < 5


@pytest.mark.parametrize('seed', [0, 817, 14, 19, 29])
def test_socket_bore_does_not_erase_remote_obstacles_on_same_fitting(seed, library):
    assembly = Assembly.from_doc(remote_bore_obstruction(seed), FIXTURES, library)
    # A filled convex proxy is intentional at the declared bore; validate the
    # complete physical hollow-bore counterpart, and socket geometry for both.
    assert validate(assembly, collisions=not bool(seed & 8))['valid']
    result = plan_build(assembly)
    assert result['status'] == 'blocked', result
    assert not result['steps']


@pytest.mark.parametrize('seed', [417, 14])
def test_unobstructed_declared_bore_can_slide(seed, library):
    assembly = Assembly.from_doc(remote_bore_obstruction(seed, obstructed=False),
                                 FIXTURES, library)
    assert_complete_plan(assembly, plan_build(assembly))


def test_cancellation_during_collision_sweeps(library, monkeypatch):
    assembly = Assembly.from_doc(cube(9), FIXTURES, library)
    calls = []
    original = planning.pb.getClosestPoints
    def closest(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)
    monkeypatch.setattr(planning.pb, 'getClosestPoints', closest)
    result = plan_build(assembly, cancelled=lambda: bool(calls))
    assert result['status'] == 'cancelled', result
    assert not result['steps']
    assert len(calls) == result['collision_queries'] == 1


def test_collision_budget_is_enforced_inside_sweep(library):
    doc = capped_socket(obstructed=True)
    doc['build']['collision_query_limit'] = 1
    assembly = Assembly.from_doc(doc, FIXTURES, library)
    result = plan_build(assembly)
    assert result['status'] == 'indeterminate', result
    assert result['collision_queries'] == 1
    assert not result['steps']
    assert 'budget' in result['reason'].lower()


@pytest.mark.parametrize('seed', [417, 216, 436])
def test_split_clamp_between_stops_is_rotation_invariant(seed, library):
    assembly = Assembly.from_doc(rotated_split_clamp(seed), FIXTURES, library)
    result = plan_build(assembly)
    assert_complete_plan(assembly, result)
    assert_sweep_samples(assembly, result)
    step = result['steps'][-1]
    assert step['part'] == 'middle'
    axis = assembly.parts['middle'].frame({'port': 'through'})[1]
    assert abs(np.asarray(step['approach_direction']) @ axis) < .01


def test_disconnected_parts_are_staged_with_separate_support(library):
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': 'Separately positioned parts', 'joints': [],
           'parts': [{'id': f'box-{index}', 'catalog': 'generic.box',
                      'parameters': {'width_mm': 20, 'depth_mm': 20,
                                     'height_mm': 20, 'mass_kg': 1},
                      'pose': {'position_mm': [index*200, 0, 10]}}
                     for index in range(2)]}
    assembly = Assembly.from_doc(doc, FIXTURES, library)
    result = plan_build(assembly)
    assert_complete_plan(assembly, result)
    assert_gravity_support(assembly, result)


def test_floor_anchor_cannot_stabilize_an_overhanging_heavy_load(library):
    assembly = Assembly.from_doc(anchored_cantilever(44), FIXTURES, library)
    assert validate(assembly, collisions=False)['valid']
    result = plan_build(assembly)
    assert result['status'] == 'blocked', result
    assert not result['steps']


def test_light_frame_uses_bounded_connected_assisted_handoffs(library):
    doc = capture_frame(0)
    original = copy.deepcopy(doc)
    assembly = Assembly.from_doc(doc, FIXTURES, library)
    assert validate(assembly, collisions=False)['valid']
    result = plan_build(assembly)
    assert_complete_plan(assembly, result)
    assert_gravity_support(assembly, result)
    assert_sweep_samples(assembly, result)
    assert any(step.get('needs_grounding') or step.get('capture_pending')
               for step in result['steps']), result
    assert doc == original


def test_heavy_purchased_worktop_uses_assistance_on_stable_legs(load):
    assembly = load('workbench')
    original = copy.deepcopy(assembly.doc)
    assert assembly.parts['worktop'].mass > 15
    result = plan_build(assembly)
    assert_complete_plan(assembly, result)
    assert_gravity_support(assembly, result)
    assert_sweep_samples(assembly, result)
    worktop = next(step for step in result['steps']
                   if 'worktop' in step.get('introduced_parts', ()))
    assert worktop['assistance_required']
    assert worktop['group_mass_kg'] == pytest.approx(assembly.parts['worktop'].mass)
    assert result['certificate']['max_lift_mass_kg'] == pytest.approx(
        assembly.parts['worktop'].mass)
    assert assembly.doc == original


def test_explicit_handling_limit_is_not_raised_for_heavy_worktop(load):
    assembly = load('workbench')
    assembly.doc['build']['max_lift_mass_kg'] = 15
    result = plan_build(assembly)
    assert result['status'] == 'indeterminate', result
    assert not result['steps']
