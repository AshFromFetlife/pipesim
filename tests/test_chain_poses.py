"""Flexible-line presets keep exact link lengths and existing connections."""

import copy

import numpy as np
import pytest

from pipesim.chain_poses import PRESETS, apply_preset
from pipesim.grouping import attach_part, move_object, set_object_layout
from pipesim.math3d import pose_of, transform
from pipesim.posing import transform_part


def line(factory, blank, profile, count=32):
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id': 'line', 'template': 'chain',
                       'parameters': {'length_mm': count * (20 if profile == 'generic.chain-link' else 25),
                                      'link_catalog': profile},
                       'pose': {'position_mm': [100, 200, 1000]}}]
    return factory(doc)


def nodes(assembly):
    count = sum(name.startswith('line/link-') for name in assembly.parts)
    return np.array([assembly.parts['line/link-1'].frame({'port': 'b'})[0]] +
                    [assembly.parts[f'line/link-{i}'].frame({'port': 'a'})[0]
                     for i in range(1, count + 1)])


@pytest.mark.parametrize('profile', ['generic.chain-link', 'generic.rope-nylon-10',
                                     'generic.strap-seatbelt-65'])
@pytest.mark.parametrize('preset', PRESETS)
def test_shape_presets_keep_a_compact_line_and_exact_joints(factory, blank, profile, preset):
    before = line(factory, blank, profile)
    result = apply_preset(before, 'line', preset)
    after = factory(result['document'])
    assert result['changed'] or preset == 'straight-down'
    assert len(after.doc['objects']) == 1 and not after.doc['parts']
    if result['changed']:
        assert after.doc['objects'][0]['pose_preset'] == preset
    assert len(after.editor_groups()) == 1
    measured = nodes(after)
    pitch = np.linalg.norm(measured[1] - measured[0])
    assert np.allclose(np.linalg.norm(np.diff(measured, axis=0), axis=1), pitch, atol=.001)
    for joint in after.joints:
        a, b, _ = after.joint_frames(joint)
        assert np.linalg.norm(a - b) < .01
    if preset == 'circle-touch':
        assert np.linalg.norm(measured[-1] - measured[0]) < .05
    if preset == 'circle-overlap':
        assert np.linalg.norm(measured[-2] - measured[0]) < .05
        assert np.linalg.norm(measured[-1] - measured[1]) < .05


def test_whole_line_move_preserves_a_selected_shape(factory, blank):
    before = line(factory, blank, 'generic.rope-nylon-10')
    posed = factory(apply_preset(before, 'line', 'spiral-two')['document'])
    moved = factory(move_object(posed, 'line', {'position_mm': [700, -300, 1200],
                                               'rotation_deg': [20, 15, 45]})['document'])
    before_points, after_points = nodes(posed), nodes(moved)
    assert np.allclose(np.linalg.norm(np.diff(before_points, axis=0), axis=1),
                       np.linalg.norm(np.diff(after_points, axis=0), axis=1), atol=1e-5)
    first = posed.parts['line/link-1'].matrix
    delta = moved.parts['line/link-1'].matrix @ np.linalg.inv(first)
    for part in posed.parts:
        assert np.allclose(moved.parts[part].matrix, delta @ posed.parts[part].matrix,
                           atol=1e-5)


def test_free_hanging_rope_reaches_fixed_shackle_without_bending(factory, blank):
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id': 'line', 'template': 'chain',
                       'parameters': {'length_mm': 1260, 'link_catalog': 'generic.rope-jute-6'},
                       'pose': {'position_mm': [0, 0, 500]}}]
    straight = factory(doc)
    shaped = factory(apply_preset(straight, 'line', 'hanging-arc')['document'])
    editable = factory(set_object_layout(shaped, 'line', 'posable'))
    start = editable.parts['line/link-1'].frame({'port': 'b'})[0]
    doc = copy.deepcopy(editable.doc)
    doc['parts'] = [{'id': 'shackle', 'body': {'kind': 'connector', 'mass_kg': .1,
        'geometry': [{'type': 'sphere', 'radius_mm': 8}],
        'ports': {'eye': {'type': 'eye', 'assembly': 'hook',
                          'position_mm': [0, 0, 0], 'axis': [1, 0, 0]}}},
        'pose': {'position_mm': (start + [234.6, 0, 0]).tolist()}}]
    doc['anchors'] = [{'part': 'shackle', 'surface': 'fixture'}]
    before = factory(doc)
    attached = attach_part(before, 'line', 'line/link-1',
                           {'part': 'shackle', 'port': 'eye'}, 'spherical', part_port='b')
    after = factory(attached['document'])
    assert len([part for part in after.parts if part.startswith('line/link-')]) == 63
    assert np.allclose(after.parts['shackle'].matrix, before.parts['shackle'].matrix)
    assert np.allclose(nodes(after) - nodes(before), [234.6, 0, 0], atol=1e-5)
    first, second, _ = after.joint_frames(attached['joint'])
    assert np.linalg.norm(first - second) < .5


@pytest.mark.parametrize('mirror', [False, True])
def test_free_rope_first_segment_can_be_posed_with_scene_mirror(factory, blank, mirror):
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id': 'line', 'template': 'chain',
                       'parameters': {'length_mm': 1260, 'link_catalog': 'generic.rope-jute-6'},
                       'pose': {'position_mm': [100, 0, 500]}}]
    if mirror:
        doc['draft_subassemblies'] = [{'id': 'scene', 'runs': [],
                                       'mirrors': [{'id': 'plane', 'axis': 'x',
                                                    'offset_mm': 0, 'scope': 'scene'}]}]
    shaped = factory(apply_preset(factory(doc), 'line', 'hanging-arc')['document'])
    before = factory(set_object_layout(shaped, 'line', 'posable'))
    target = pose_of(before.parts['line/link-1'].matrix)
    target['position_mm'][1] += 50

    preview = transform_part(before, 'line/link-1', target, preview=True)
    assert not preview['limited']
    assert 'line/link-1' in preview['moved']
    after = factory(transform_part(before, 'line/link-1', target)['document'])
    assert np.allclose(after.parts['line/link-1'].matrix[:3, 3], target['position_mm'], atol=.01)
    assert np.allclose(nodes(after)-nodes(before), [0, 50, 0], atol=.01)
    if mirror:
        assert after.doc['draft_subassemblies'][0]['mirrors'] == doc['draft_subassemblies'][0]['mirrors']


@pytest.mark.parametrize('mirror', [False, True])
def test_rope_interior_eye_attaches_with_three_mm_reach_allowance(factory, blank, mirror):
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id': 'line', 'template': 'chain',
                       'parameters': {'length_mm': 1260, 'link_catalog': 'generic.rope-jute-6'},
                       'pose': {'position_mm': [0, 0, 500]}}]
    if mirror:
        doc['draft_subassemblies'] = [{'id': 'scene', 'runs': [],
                                       'mirrors': [{'id': 'plane', 'axis': 'x',
                                                    'offset_mm': 0, 'scope': 'scene'}]}]
    shaped = apply_preset(factory(doc), 'line', 'hanging-arc')['document']
    shaped['objects'][0]['layout_mode'] = 'posable'
    original = factory(shaped)
    first = original.parts['line/link-1'].frame({'port': 'b'})[0]
    fifteenth = original.parts['line/link-15'].frame({'port': 'a'})[0]
    doc = copy.deepcopy(shaped)
    doc['parts'] = [
        {'id': 'first-eye', 'body': {'kind': 'connector', 'mass_kg': .1,
         'geometry': [{'type': 'sphere', 'radius_mm': 8}],
         'ports': {'eye': {'type': 'eye', 'position_mm': [0, 0, 0], 'axis': [1, 0, 0]}}},
         'pose': {'position_mm': first.tolist()}},
        {'id': 'target-eye', 'catalog': 'generic.d-shackle',
         'pose': {'position_mm': (fifteenth + [3, 0, -10]).tolist()}},
    ]
    doc['joints'] = [{'id': 'first-attachment', 'type': 'spherical',
                      'a': {'part': 'first-eye', 'port': 'eye'},
                      'b': {'part': 'line/link-1', 'port': 'b'}}]
    doc['anchors'] = [{'part': name, 'surface': 'fixture'} for name in ('first-eye', 'target-eye')]
    before = factory(doc)
    attached = attach_part(before, 'line', 'line/link-15',
                           {'part': 'target-eye', 'port': 'eye'}, 'spherical', part_port='a')
    after = factory(attached['document'])
    assert np.allclose(after.parts['first-eye'].matrix, before.parts['first-eye'].matrix)
    assert np.allclose(after.parts['target-eye'].matrix, before.parts['target-eye'].matrix)
    for joint in after.joints:
        a, b, _ = after.joint_frames(joint)
        assert np.linalg.norm(a - b) < .5
    source_local = after.parts['line/link-15'].local_frame(attached['joint']['b'])[0]
    nominal_local = after.parts['line/link-15'].local_frame({'port': 'a'})[0]
    assert .5 < np.linalg.norm(source_local - nominal_local) <= 3.5
    target_pose = pose_of(after.parts['line/link-25'].matrix)
    target_pose['position_mm'][1] += 10
    reshaped = factory(transform_part(after, 'line/link-25', target_pose)['document'])
    for joint in reshaped.joints:
        a, b, _ = reshaped.joint_frames(joint)
        assert np.linalg.norm(a - b) < .5


def test_anchor_and_two_end_connections_remain_placed(factory, blank):
    before = line(factory, blank, 'generic.chain-link')
    closed = factory(apply_preset(before, 'line', 'teardrop')['document'])
    first, last = nodes(closed)[[0, -1]]
    doc = copy.deepcopy(closed.doc)
    doc['parts'] = [{'id': 'hub', 'body': {'kind': 'connector', 'mass_kg': .1,
                        'geometry': [{'type': 'sphere', 'radius_mm': 5}],
                        'ports': {'start': {'type': 'eye', 'position_mm': first.tolist(),
                                             'axis': [0, 1, 0], 'diameter_mm': 8},
                                  'end': {'type': 'eye', 'position_mm': last.tolist(),
                                           'axis': [0, 1, 0], 'diameter_mm': 8}}}}]
    doc['joints'] = [{'id': 'start', 'type': 'spherical',
                      'a': {'part': 'hub', 'port': 'start'},
                      'b': {'part': 'line/link-1', 'port': 'b'}},
                     {'id': 'end', 'type': 'spherical',
                      'a': {'part': 'hub', 'port': 'end'},
                      'b': {'part': 'line/link-32', 'port': 'a'}}]
    doc['anchors'] = [{'part': 'hub', 'surface': 'fixture'}]
    attached = factory(doc)
    result = apply_preset(attached, 'line', 'hanging-arc')
    assert result['changed']
    after = factory(result['document'])
    assert np.allclose(nodes(after)[[0, -1]], nodes(attached)[[0, -1]], atol=.05)
    assert np.allclose(transform(after.doc['parts'][0].get('pose')),
                       transform(attached.doc['parts'][0].get('pose')))
    for joint in after.joints:
        a, b, _ = after.joint_frames(joint)
        assert np.linalg.norm(a - b) < .05


def test_manual_segment_pose_clears_the_saved_preset_name(factory, blank):
    original = line(factory, blank, 'generic.chain-link')
    posed = factory(apply_preset(original, 'line', 'u-deep')['document'])
    editable = factory(set_object_layout(posed, 'line', 'posable'))
    target = pose_of(editable.parts['line/link-16'].matrix)
    target['position_mm'][1] += 8
    edited = factory(transform_part(editable, 'line/link-16', target)['document'])
    assert 'pose_preset' not in edited.doc['objects'][0]


def test_preset_fits_between_two_fixed_separated_ends(factory, blank):
    original = line(factory, blank, 'generic.rope-nylon-10')
    doc = copy.deepcopy(original.doc)
    doc['anchors'] = [{'part': 'line/link-1', 'surface': 'fixture'}]
    anchored = factory(doc)
    editable = factory(set_object_layout(anchored, 'line', 'posable'))
    target = pose_of(editable.parts['line/link-32'].matrix)
    target['position_mm'][0] += 100
    target['position_mm'][2] += 150
    bent = factory(transform_part(editable, 'line/link-32', target)['document'])
    doc = copy.deepcopy(bent.doc)
    doc['anchors'].append({'part': 'line/link-32', 'surface': 'fixture'})
    fixed = factory(doc)
    result = apply_preset(fixed, 'line', 'u-deep')
    after = factory(result['document'])
    assert result['changed']
    for name in ('line/link-1', 'line/link-32'):
        assert np.allclose(after.parts[name].matrix, fixed.parts[name].matrix, atol=1e-5)
    assert not np.allclose(nodes(after)[8:24], nodes(fixed)[8:24], atol=.1)
    for joint in after.joints:
        a, b, _ = after.joint_frames(joint)
        assert np.linalg.norm(a - b) < .001
