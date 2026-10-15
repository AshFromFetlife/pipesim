"""Wide webbing favours facewise bends while retaining ball-joint freedom."""

import copy
import math
import random
import secrets
import time

import numpy as np
import pybullet as pb
import pytest
from scipy.spatial.transform import Rotation

from pipesim.chain import generate
from pipesim.chain_poses import apply_preset
from pipesim.document import Assembly
from pipesim.math3d import point, pose_of, transform
from pipesim.physics import World, simulate
from fuzz_runtime import budget
from test_swing_dynamics import assert_recorded_line_continuity


def _two_link_bend(factory, blank, library, axis, *, bend_deg=20,
                   line_rotation_deg=(0, 0, 0)):
    parameters = {'length_mm': 50, 'link_catalog': 'generic.strap-seatbelt-65'}
    components = generate(parameters, library)
    first, second = components['parts']
    a, b = (np.array(library.parts[parameters['link_catalog']]['ports'][port]['position_mm'])
            for port in ('a', 'b'))
    first_matrix = transform(first['pose'])
    second_matrix = transform(second['pose'])
    pivot = point(first_matrix, a)
    direction = first_matrix[:3, :3] @ np.eye(3)[:, axis]
    rotation = Rotation.from_rotvec(direction * math.radians(bend_deg)).as_matrix()
    changed = second_matrix.copy()
    changed[:3, :3] = rotation @ second_matrix[:3, :3]
    changed[:3, 3] = pivot + rotation @ (second_matrix[:3, 3] - pivot)
    second['pose'] = pose_of(changed)
    doc = copy.deepcopy(blank)
    doc['environment'] = {'ground': False, 'gravity_m_s2': [0, 0, 0]}
    doc['objects'] = [{'id': 'belt', 'template': 'chain', 'parameters': parameters,
                       'pose': {'position_mm': [0, 0, 1000],
                                'rotation_deg': list(line_rotation_deg)},
                       'components': components}]
    doc['anchors'] = [{'part': 'belt/link-1', 'surface': 'fixture'}]
    return factory(doc), Rotation.from_euler('xyz', line_rotation_deg, degrees=True).apply(direction)


def test_wide_webbing_prefers_facewise_bending_without_a_hinge(factory, blank, library):
    responses = []
    for axis in (0, 1, 2):
        assembly, direction = _two_link_bend(factory, blank, library, axis)
        assert next(j for j in assembly.joints if j['id'] == 'belt/join-1')['type'] == 'spherical'
        with World(assembly, 1 / 1000) as world:
            world.step()
            angular = np.array(world.part_velocity('belt/link-2')[1])
            responses.append(abs(angular @ direction))
    assert responses[0] > 0
    assert responses[1] > 2 * responses[0]
    assert 0 < responses[2] < responses[1]


def _random_bend_case(seed, factory, blank, library):
    rng = random.Random(seed)
    rotation = [rng.uniform(-85, 85), rng.uniform(-85, 85), rng.uniform(-180, 180)]
    angle = rng.uniform(5, 45)
    responses = []
    for axis in (0, 1, 2):
        assembly, direction = _two_link_bend(factory, blank, library, axis,
                                             bend_deg=angle, line_rotation_deg=rotation)
        with World(assembly, 1 / 1000) as world:
            world.step()
            responses.append(abs(np.array(world.part_velocity('belt/link-2')[1]) @ direction))
            assert np.isfinite(responses[-1]), seed
    assert responses[1] > 1.5 * responses[0], (seed, angle, rotation, responses)
    assert 0 < responses[2] < responses[1], (seed, angle, rotation, responses)


def _random_slack_case(seed, factory, blank):
    rng = random.Random(seed)
    rotation = [0, rng.uniform(-20, 20), rng.uniform(-80, 80)]
    doc = copy.deepcopy(blank)
    doc['environment'] = {'ground': False}
    doc['objects'] = [{'id': 'belt', 'template': 'chain',
                       'parameters': {'length_mm': 400, 'link_catalog': 'generic.strap-seatbelt-65'},
                       'pose': {'position_mm': [0, 0, 1700], 'rotation_deg': rotation}}]
    posed = apply_preset(factory(doc), 'belt', 's-curve')['document']
    posed['anchors'] = [{'part': 'belt/link-1', 'surface': 'fixture'},
                         {'part': 'belt/link-16', 'surface': 'fixture'}]
    assembly = factory(posed)
    recording = simulate(assembly, .6, 10, mode='full')
    centers = np.array([recording['frames'][-1]['parts'][f'belt/link-{i}']['position_mm']
                        for i in range(1, 17)])
    directions = np.diff(centers, axis=0)
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    turns = np.degrees(np.arccos(np.clip(np.sum(directions[:-1] * directions[1:], axis=1), -1, 1)))
    chord_z = np.interp(8, [0, 15], [centers[0, 2], centers[-1, 2]])
    assert centers[8, 2] < chord_z - 15, (seed, rotation)
    # The first two folds can meet a fixed attachment at its prescribed angle.
    assert turns[2:-2].max() < 40, (seed, rotation, turns.max())
    assert_recorded_line_continuity(assembly, recording)


@pytest.mark.parametrize('seed', range(20))
def test_seeded_webbing_bend_axes(seed, factory, blank, library):
    _random_bend_case(seed, factory, blank, library)


def test_webbing_fuzz_oracle_rejects_isotropic_free_ball_joints(factory, blank, library):
    neutral_library = copy.deepcopy(library)
    for key in ('bend_stiffness_nm_rad', 'bend_damping_nm_s_rad'):
        neutral_library.parts['generic.strap-seatbelt-65'].pop(key, None)
    base = factory(blank).base
    neutral_factory = lambda doc: Assembly.from_doc(doc, base, neutral_library)
    with pytest.raises(AssertionError):
        _random_bend_case(0, neutral_factory, blank, neutral_library)


@pytest.mark.parametrize('seed', range(12))
def test_seeded_webbing_slack(seed, factory, blank):
    _random_slack_case(seed, factory, blank)


def test_randomized_webbing_bend_axes(factory, blank, library):
    root = secrets.randbits(64)
    minutes, _, deadline = budget(default_minutes=5)
    rng = random.Random(root)
    completed = 0
    print(f'webbing bend fuzz root_seed={root} budget_minutes={minutes}', flush=True)
    while time.monotonic() < deadline:
        seed = rng.getrandbits(64)
        try:
            _random_bend_case(seed, factory, blank, library)
            if completed % 128 == 0:
                _random_slack_case(seed, factory, blank)
        except Exception as exc:
            raise AssertionError(f'webbing bend fuzz root_seed={root} case_seed={seed} '
                                 f'completed={completed}') from exc
        completed += 1
    print(f'webbing bend fuzz completed={completed}', flush=True)
    assert completed > 0


@pytest.mark.parametrize('rotation_deg', [(0, 0, 0), (0, 20, 35)])
def test_wide_webbing_slack_sags_as_a_smooth_u(factory, blank, rotation_deg):
    doc = copy.deepcopy(blank)
    doc['environment'] = {'ground': False}
    doc['objects'] = [{'id': 'belt', 'template': 'chain',
                       'parameters': {'length_mm': 600, 'link_catalog': 'generic.strap-seatbelt-65'},
                       'pose': {'position_mm': [0, 0, 1700],
                                'rotation_deg': list(rotation_deg)}}]
    authored = factory(doc)
    posed = apply_preset(authored, 'belt', 's-curve')['document']
    posed['anchors'] = [{'part': 'belt/link-1', 'surface': 'fixture'},
                         {'part': 'belt/link-24', 'surface': 'fixture'}]
    assembly = factory(posed)
    result = simulate(assembly, .4, 10, mode='full')
    assert any('local-axis bending' in note for note in result['limitations'])
    frame = result['frames'][-1]
    centers = np.array([frame['parts'][f'belt/link-{i}']['position_mm']
                        for i in range(1, 25)])
    vectors = np.diff(centers, axis=0)
    unit = vectors / np.linalg.norm(vectors, axis=1)[:, None]
    turns = np.degrees(np.arccos(np.clip(np.sum(unit[:-1] * unit[1:], axis=1), -1, 1)))
    assert np.isfinite(centers).all()
    chord_z = np.interp(12, [0, 23], [centers[0, 2], centers[-1, 2]])
    assert centers[12, 2] < chord_z - 30
    assert max(np.linalg.norm(np.diff(centers, axis=0), axis=1)) < 60
    assert turns.max() < 30


def test_diagonal_webbing_can_turn_and_fall_under_gravity(factory, blank):
    doc = copy.deepcopy(blank)
    doc['environment'] = {'ground': False}
    doc['objects'] = [{'id': 'belt', 'template': 'chain',
                       'parameters': {'length_mm': 300, 'link_catalog': 'generic.strap-seatbelt-65'},
                       'pose': {'position_mm': [0, 0, 1800], 'rotation_deg': [0, 55, 35]}}]
    doc['anchors'] = [{'part': 'belt/link-1', 'surface': 'fixture'}]
    assembly = factory(doc)
    initial = assembly.parts['belt/link-12'].matrix[:3, 3]
    frame = simulate(assembly, .65, 10, mode='full')['frames'][-1]
    final = np.array(frame['parts']['belt/link-12']['position_mm'])
    assert final[2] < initial[2] - 40
    assert np.linalg.norm(final[:2]) < .6 * np.linalg.norm(initial[:2])


def test_diagonal_webbing_preview_keeps_slack_and_joint_closure(factory, blank):
    doc = copy.deepcopy(blank)
    doc['environment'] = {'ground': False}
    doc['objects'] = [{'id': 'belt', 'template': 'chain',
                       'parameters': {'length_mm': 300, 'link_catalog': 'generic.strap-seatbelt-65'},
                       'pose': {'position_mm': [0, 0, 1800], 'rotation_deg': [0, 55, 35]}}]
    doc['anchors'] = [{'part': 'belt/link-1', 'surface': 'fixture'}]
    assembly = factory(doc)
    initial = assembly.parts['belt/link-12'].matrix[:3, 3]
    recording = simulate(assembly, .65, 10, mode='preview', chain_links_per_body=4)
    final = np.array(recording['frames'][-1]['parts']['belt/link-12']['position_mm'])
    assert final[2] < initial[2] - 40
    assert np.linalg.norm(final[:2]) < .85 * np.linalg.norm(initial[:2])
    assert_recorded_line_continuity(assembly, recording)
