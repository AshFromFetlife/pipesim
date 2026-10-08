"""A short jute line carrying a modest weight is a physics regression."""

import math
import copy

import numpy as np
import pytest
from scipy.optimize import brentq

from pipesim.chain import generate
from pipesim.math3d import align_axis, pose_of, transform, point

from pipesim.physics import simulate


def hanging_weight(blank, library, *, mass_kg=10, rope_mm=400, slack_mm=0,
                   swing_deg=0, catalog='generic.rope-jute-6'):
    angle = math.radians(swing_deg)
    endpoint = [-rope_mm*math.sin(angle), 0,
                700-rope_mm*math.cos(angle)]
    doc = {**blank, 'environment': {'ground': False}}
    doc['parts'] = [
        {'id': 'support', 'body': {'kind': 'rigid', 'mass_kg': 1,
         'geometry': [{'type': 'sphere', 'radius_mm': 10}],
         'ports': {'eye': {'type': 'eye', 'position_mm': [0, 0, 0]}}},
         'pose': {'position_mm': [0, 0, 700]}},
        {'id': 'weight', 'body': {'kind': 'rigid', 'mass_kg': mass_kg,
         'geometry': [{'type': 'box', 'size_mm': [100, 100, 100]}],
         'ports': {'eye': {'type': 'eye', 'position_mm': [0, 0, 50]}}},
         'pose': {'position_mm': [endpoint[0], endpoint[1], endpoint[2]-50]}},
    ]
    parameters = {'length_mm': rope_mm+slack_mm,
                  'link_catalog': catalog}
    line = {'id': 'line', 'template': 'chain', 'parameters': parameters,
            'pose': {'position_mm': [0, 0, 700],
                     'rotation_deg': [0, swing_deg, 0]}}
    if slack_mm:
        assert rope_mm == 400 and slack_mm == 100 and not swing_deg
        components = generate(parameters, library)
        side = 12
        down = (rope_mm-20)/(side*2)
        across = math.sqrt(20**2-down**2)
        point = np.zeros(3)
        for index, part in enumerate(components['parts']):
            dx = across if index < side else 0 if index == side else -across
            dz = down if index != side else 20
            next_point = point+np.array([dx, 0, -dz])
            matrix = np.eye(4)
            matrix[:3, :3] = align_axis([0, 0, 1], point-next_point)
            matrix[:3, 3] = (point+next_point)/2
            part['pose'] = pose_of(matrix)
            point = next_point
        assert np.allclose(point, [0, 0, -rope_mm])
        line['components'] = components
    doc['objects'] = [line]
    last = int((rope_mm+slack_mm)/20)
    doc['joints'] = [
        {'id': 'top', 'type': 'spherical',
         'a': {'part': 'support', 'port': 'eye'},
         'b': {'part': 'line/link-1', 'port': 'b'}},
        {'id': 'bottom', 'type': 'spherical',
         'a': {'part': 'line/link-' + str(last), 'port': 'a'},
         'b': {'part': 'weight', 'port': 'eye'}},
    ]
    doc['anchors'] = [{'part': 'support', 'surface': 'fixture'}]
    return doc


def four_rope_swing(blank):
    """Four rated lines, each carrying about 15 kg, seated inside their mounts."""
    doc = copy.deepcopy(blank)
    doc['environment'] = {'ground': False}
    corners = [(x, y) for x in (-150, 150) for y in (-150, 150)]
    doc['parts'] = [{
        'id': 'payload',
        'body': {'kind': 'rigid', 'mass_kg': 60,
                 'geometry': [{'type': 'box', 'size_mm': [420, 420, 200]}],
                 'ports': {f'eye-{i}': {'type': 'eye', 'position_mm': [x, y, 0]}
                           for i, (x, y) in enumerate(corners)}},
        'pose': {'position_mm': [0, 0, 500]},
    }]
    doc['objects'] = []
    doc['joints'] = []
    doc['anchors'] = []
    for i, (x, y) in enumerate(corners):
        doc['parts'].append({
            'id': f'support-{i}',
            'body': {'kind': 'rigid', 'mass_kg': 1,
                     'geometry': [{'type': 'sphere', 'radius_mm': 55}],
                     'ports': {'eye': {'type': 'eye', 'position_mm': [0, 0, 0]}}},
            'pose': {'position_mm': [x, y, 1500]},
        })
        doc['objects'].append({
            'id': f'rope-{i}', 'template': 'chain',
            'parameters': {'length_mm': 1000, 'link_catalog': 'generic.rope-nylon-10'},
            'pose': {'position_mm': [x, y, 1500]},
        })
        doc['joints'].extend([
            {'id': f'top-{i}', 'type': 'spherical',
             'a': {'part': f'support-{i}', 'port': 'eye'},
             'b': {'part': f'rope-{i}/link-1', 'port': 'b'}},
            {'id': f'bottom-{i}', 'type': 'spherical',
             'a': {'part': f'rope-{i}/link-40', 'port': 'a'},
             'b': {'part': 'payload', 'port': f'eye-{i}'}},
        ])
        doc['anchors'].append({'part': f'support-{i}', 'surface': 'fixture'})
    return doc


def max_joint_gap_mm(assembly, frames, *, excluded=()):
    # A chain can visually explode without reporting a fracture: Bullet's
    # point constraints remain present while their two pivots drift apart.
    largest = 0.
    for frame in frames:
        for joint in assembly.joints:
            if joint['id'] in excluded:
                continue
            a, b = joint['a'], joint['b']
            pa = point(transform(frame['parts'][a['part']]),
                       assembly.parts[a['part']].local_frame(a)[0])
            pb_ = point(transform(frame['parts'][b['part']]),
                        assembly.parts[b['part']].local_frame(b)[0])
            largest = max(largest, float(np.linalg.norm(pa-pb_)))
    return largest


@pytest.mark.parametrize('mass_kg', [10, 24])
def test_hanging_weight_at_rest_does_not_break_jute(factory, blank, library, mass_kg):
    result = simulate(factory(hanging_weight(blank, library, mass_kg=mass_kg)),
                      duration=.2, fps=20)
    assert not any(event['type'] == 'joint_break' for event in result['events']), result['events']


def test_ten_kg_drop_into_jute_slack_does_not_shatter(factory, blank, library):
    doc = hanging_weight(blank, library, slack_mm=100)
    # A posed line saved before the catalog correction still contains the
    # former generated joint threshold. Its selected catalog now owns it.
    for joint in doc['objects'][0]['components']['joints']:
        joint['break_force_n'] = 350
    assembly = factory(doc)
    assert all(joint['break_force_n'] == 1800 for joint in assembly.joints
               if joint.get('metadata', {}).get('chain_link'))
    result = simulate(assembly, duration=.35, fps=30)
    heights = [frame['parts']['weight']['position_mm'][2] for frame in result['frames']]
    assert min(heights) < heights[0]-80
    assert not any(event['type'] == 'joint_break' for event in result['events']), result['events']


def test_four_ropes_seated_in_mounts_carry_sixty_kg_without_exploding(factory, blank):
    assembly = factory(four_rope_swing(blank))
    result = simulate(assembly, duration=.25, fps=20)
    assert not any(event['type'] == 'joint_break' for event in result['events']), result['events']
    assert result['final_max_speed_m_s'] < 5
    assert max_joint_gap_mm(assembly, result['frames']) < 8


def test_24_kg_jute_pendulum_swing_does_not_break(factory, blank, library):
    result = simulate(factory(hanging_weight(blank, library, mass_kg=24,
                                             swing_deg=30)), duration=.7, fps=20)
    assert not any(event['type'] == 'joint_break' for event in result['events']), result['events']


def test_overloaded_rope_first_fails_at_one_link(factory, blank, library):
    doc = hanging_weight(blank, library, slack_mm=100)
    weak_jute = copy.deepcopy(library.parts['generic.rope-jute-6'])
    weak_jute['break_force_n'] = 350
    doc['definitions'] = {'generic.rope-jute-6': weak_jute}
    assembly = factory(doc)
    result = simulate(assembly, duration=.18, fps=30)
    breaks = [event for event in result['events'] if event['type'] == 'joint_break']
    assert breaks
    first_time = breaks[0]['time_s']
    assert sum(abs(event['time_s']-first_time) < 1e-9 for event in breaks) == 1, breaks
    assert len(breaks) == 1, breaks
    assert max_joint_gap_mm(assembly, result['frames'][-1:],
                            excluded={breaks[0]['joint']}) < 8


@pytest.mark.parametrize('slack_mm,swing_deg', [(0, 30), (0, 90), (100, 0)])
def test_unbreakable_chain_carries_ten_kg_pendulum(factory, blank, library,
                                                  slack_mm, swing_deg):
    doc = hanging_weight(blank, library, catalog='generic.chain-link',
                         slack_mm=slack_mm, swing_deg=swing_deg)
    assembly = factory(doc)
    result = simulate(assembly, duration=1, fps=30)
    assert not [event for event in result['events'] if event['type'] == 'joint_break']
    weight_positions = [frame['parts']['weight']['position_mm']
                        for frame in result['frames']]
    assert max(abs(position[0]) for position in weight_positions) < 500
    assert max(position[2] for position in weight_positions) <= weight_positions[0][2]+10
    assert result['final_max_speed_m_s'] < 6
    assert max_joint_gap_mm(assembly, result['frames']) < 8


@pytest.mark.parametrize('side_mm,drop_mm', [(200, 200), (100, 100),
                                           (300, 100), (100, 300)])
def test_unbreakable_chain_curved_with_sideways_weight(factory, blank, library,
                                                      side_mm, drop_mm):
    doc = hanging_weight(blank, library, catalog='generic.chain-link',
                         rope_mm=500)
    doc['parts'][0]['body']['geometry'] = [
        {'type': 'cylinder', 'diameter_mm': 42, 'length_mm': 1000,
         'axis': [1, 0, 0]}]
    doc['parts'][0]['body']['ports']['eye']['position_mm'] = [0, 0, -30]
    doc['parts'][0]['pose']['position_mm'] = [0, 0, 730]
    line = doc['objects'][0]
    components = generate(line['parameters'], library)
    count = len(components['parts'])
    segment_mm = 20
    displacement = np.array([-float(side_mm), 0., -float(drop_mm)])
    chord = np.linalg.norm(displacement)
    theta = brentq(lambda angle: 2 * segment_mm *
                   math.sin(angle/2) / (2*math.sin(angle/(2*count))) - chord,
                   .01, 2*math.pi-.01)
    radius = segment_mm/(2*math.sin(theta/(2*count)))
    along = displacement/chord
    down = np.array([-along[2], 0, along[0]])
    nodes = [radius*(math.sin((index/count-.5)*theta)+math.sin(theta/2))*along
             + radius*(math.cos((index/count-.5)*theta)-math.cos(theta/2))*down
             for index in range(count+1)]
    assert np.allclose(nodes[-1], displacement)
    for part, first, last in zip(components['parts'], nodes, nodes[1:]):
        matrix = np.eye(4)
        matrix[:3, :3] = align_axis([0, 0, 1], first-last)
        matrix[:3, 3] = (first+last)/2
        part['pose'] = pose_of(matrix)
    for index, part in enumerate(components['parts'][1:-1], 1):
        part['pose']['position_mm'][1] += math.sin(index*1.7)*2
    line['components'] = components
    doc['parts'][1]['body']['ports']['eye']['position_mm'] = [50, 0, 0]
    doc['parts'][1]['pose']['position_mm'] = [-side_mm-50, 0, 700-drop_mm]
    assembly = factory(doc)
    result = simulate(assembly, duration=5, fps=30)
    assert not [event for event in result['events'] if event['type'] == 'joint_break']
    weight_positions = [frame['parts']['weight']['position_mm']
                        for frame in result['frames']]
    assert max_joint_gap_mm(assembly, result['frames']) < 8
    assert max(position[2] for position in weight_positions) <= 710-drop_mm
    assert result['final_max_speed_m_s'] < 6
