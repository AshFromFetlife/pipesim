import copy

import numpy as np
import pytest

from pipesim.document import Assembly, DocumentError
from pipesim.duplication import duplicate
from pipesim.grouping import move_object
from pipesim.human_symmetry import reflection, set_human_symmetry, sync_scene_humans
from pipesim.math3d import pose_of, transform
from pipesim.posing import transform_part
from pipesim.symmetry import materialize_mirror


def design(axis='x'):
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': 'Mirror-line human',
            'parts': [], 'joints': [], 'objects': [{'id': 'person', 'template': 'human',
                'parameters': {'pose': 'standing'}, 'pose': {'position_mm': [300, 120, 0]}}],
            'draft_subassemblies': [{'id': 'frame', 'runs': [{
                'id': 'reference-pipe', 'catalog': 'tubeclamp.tube-C',
                'start_mm': [0, 0, 100], 'end_mm': [0, 0, 1100]}],
                'mirrors': [{'id': 'center', 'axis': axis, 'offset_mm': 0}]}]}


@pytest.mark.parametrize('axis', ['x', 'y'])
def test_human_fixes_to_a_vertical_mirror_line_and_restricts_sideways_movement(axis):
    source = Assembly.from_doc(design(axis))
    document = set_human_symmetry(source, 'person', 'frame', 'center')
    assembly = Assembly.from_doc(document)
    symmetry = document['objects'][0]['symmetry']
    assert symmetry['axis'] == axis
    assert symmetry['line_offset_mm'] == (120 if axis == 'x' else 300)
    assert document['objects'][0]['pose']['position_mm'][:2] == ([0, 120] if axis == 'x' else [300, 0])
    assert np.allclose(assembly.parts['person/right_hand'].matrix,
                       reflection(symmetry) @ assembly.parts['person/left_hand'].matrix @ np.diag([-1, 1, 1, 1]))
    moved = move_object(assembly, 'person', {'position_mm': [100, 200, 50], 'rotation_deg': [0, 0, 45]})
    assert moved['limited']
    new_pose = next(item['pose'] for item in moved['document']['objects'] if item['id'] == 'person')
    assert new_pose['position_mm'] == ([0, 120, 50] if axis == 'x' else [300, 0, 50])
    assert new_pose['rotation_deg'] == symmetry['rotation_deg']
    Assembly.from_doc(moved['document'])


@pytest.mark.parametrize(('axis', 'angles'), [
    ('x', [32, 0, 0]), ('x', [-25, 0, 180]),
    ('y', [28, 0, 90]), ('y', [-20, 0, -90]),
])
def test_whole_person_can_rotate_while_remaining_symmetric(axis, angles):
    document = set_human_symmetry(Assembly.from_doc(design(axis)), 'person', 'frame', 'center')
    assembly = Assembly.from_doc(document)
    before = assembly.doc['objects'][0]['pose']
    target = {'position_mm': before['position_mm'], 'rotation_deg': angles}
    preview = move_object(assembly, 'person', target, preview=True)
    assert not preview['limited'] and preview['poses']
    moved = move_object(assembly, 'person', target)
    assert not moved['limited']
    assert moved['document']['objects'][0]['pose']['rotation_deg'] == pytest.approx(angles)
    assert moved['document']['objects'][0]['symmetry']['rotation_deg'] == pytest.approx(angles)
    Assembly.from_doc(moved['document'])


@pytest.mark.parametrize(('axis', 'angles'), [
    ('x', [30, 0, 0]), ('x', [25, 0, 180]),
    ('y', [25, 0, 90]), ('y', [-30, 0, -90]),
])
def test_enabling_mirror_keeps_an_existing_symmetric_whole_person_rotation(axis, angles):
    doc = design(axis)
    doc['objects'][0]['pose'] = {'position_mm': [0, 120, 0] if axis == 'x' else [300, 0, 0],
                                'rotation_deg': angles}
    before = Assembly.from_doc(doc)
    result = set_human_symmetry(before, 'person', 'frame', 'center')
    assert result['objects'][0]['pose']['rotation_deg'] == pytest.approx(angles)
    after = Assembly.from_doc(result)
    for part_id, part in before.parts.items():
        assert np.allclose(part.matrix, after.parts[part_id].matrix, atol=1e-5)


def test_reenabling_mirror_after_free_rotation_keeps_the_rotated_pose():
    aligned = set_human_symmetry(Assembly.from_doc(design()), 'person', 'frame', 'center')
    free = set_human_symmetry(Assembly.from_doc(aligned), 'person')
    rotated = move_object(Assembly.from_doc(free), 'person',
                          {'position_mm': [0, 120, 0], 'rotation_deg': [37, 0, 180]})['document']
    restored = set_human_symmetry(Assembly.from_doc(rotated), 'person', 'frame', 'center')
    assert restored['objects'][0]['pose']['rotation_deg'] == pytest.approx([37, 0, 180])
    Assembly.from_doc(restored)


def test_invalid_rotation_stops_at_current_symmetric_pose_even_with_old_reference_angle():
    doc = set_human_symmetry(Assembly.from_doc(design()), 'person', 'frame', 'center')
    doc['objects'][0]['pose']['rotation_deg'] = [30, 0, 0]
    assembly = Assembly.from_doc(doc)
    target = {'position_mm': [0, 120, 0], 'rotation_deg': [30, 0, 45]}
    result = move_object(assembly, 'person', target)
    assert result['limited']
    assert result['document']['objects'][0]['pose']['rotation_deg'] == pytest.approx([30, 0, 0])


@pytest.mark.parametrize('axis', ['x', 'y'])
def test_symmetric_limb_edit_moves_the_matching_limb_and_keeps_grouping(axis):
    assembly = Assembly.from_doc(set_human_symmetry(Assembly.from_doc(design(axis)), 'person', 'frame', 'center'))
    target = pose_of(assembly.parts['person/left_hand'].matrix)
    tangent = 1 if axis == 'x' else 0
    normal = 1-tangent
    target['position_mm'][tangent] += 80
    result = transform_part(assembly, 'person/left_hand', target)
    assert result['position_error_mm'] < .1
    assert 'person/right_hand' in result['poses']
    after = Assembly.from_doc(result['document'])
    assert after.doc['objects'][0]['symmetry'] == assembly.doc['objects'][0]['symmetry']
    assert after.doc['objects'][0].get('components')
    assert abs(after.parts['person/left_hand'].matrix[tangent, 3]-after.parts['person/right_hand'].matrix[tangent, 3]) < 1e-4
    assert abs(after.parts['person/left_hand'].matrix[normal, 3]+after.parts['person/right_hand'].matrix[normal, 3]) < 1e-4


@pytest.mark.parametrize('part', ['head', 'thorax'])
def test_symmetric_human_can_nod_and_arch_but_cannot_turn_or_twist(part):
    assembly = Assembly.from_doc(set_human_symmetry(Assembly.from_doc(design()), 'person', 'frame', 'center'))
    target = pose_of(assembly.parts['person/'+part].matrix)
    target['rotation_deg'][0] += 15
    flex = transform_part(assembly, 'person/'+part, target, mode='rotate')
    assert flex['angle_error_deg'] < .1
    Assembly.from_doc(flex['document'])
    target = pose_of(assembly.parts['person/'+part].matrix)
    target['rotation_deg'][2] += 25
    twist = transform_part(assembly, 'person/'+part, target, mode='rotate')
    assert twist['limited']
    assert twist['angle_error_deg'] > 20
    assert 'person/'+part not in twist['poses']


def test_asymmetric_saved_pose_is_rejected_until_mirror_line_is_removed():
    doc = design()
    doc['objects'][0]['parameters']['joint_angles_deg'] = {'neck_head': [0, 0, 15]}
    original = copy.deepcopy(doc)
    with pytest.raises(DocumentError, match='current posture is not symmetric'):
        set_human_symmetry(Assembly.from_doc(doc), 'person', 'frame', 'center')
    assert doc == original

    aligned = set_human_symmetry(Assembly.from_doc(design()), 'person', 'frame', 'center')
    released = set_human_symmetry(Assembly.from_doc(aligned), 'person')
    assert 'symmetry' not in released['objects'][0]
    Assembly.from_doc(released)


def test_horizontal_mirror_cannot_fix_an_upright_human():
    with pytest.raises(DocumentError, match='vertical X or Y'):
        set_human_symmetry(Assembly.from_doc(design('z')), 'person', 'frame', 'center')


def test_duplicate_symmetric_human_moves_its_reference_line_with_the_copy():
    source = Assembly.from_doc(set_human_symmetry(Assembly.from_doc(design()), 'person', 'frame', 'center'))
    result = duplicate(source, 'person/left_hand', 'subassembly', offset_mm=[400, 250, 0])
    copied = next(obj for obj in result['document']['objects'] if obj['id'] == 'person-copy')
    assert copied['symmetry']['offset_mm'] == 400
    assert copied['symmetry']['line_offset_mm'] == 370
    Assembly.from_doc(result['document'])


def test_scene_mirror_poses_one_human_with_real_paired_joints():
    doc = design()
    doc['draft_subassemblies'][0]['mirrors'][0]['scope'] = 'scene'
    aligned = sync_scene_humans(Assembly.from_doc(doc))
    assert aligned['objects'][0]['symmetry']['axis'] == 'x'
    assembly = Assembly.from_doc(aligned)
    right = pose_of(assembly.parts['person/right_hand'].matrix)
    right['position_mm'][1] += 80
    result = transform_part(assembly, 'person/right_hand', right)
    assert not result['limited']
    saved = result['document']
    assert len(saved['objects']) == 1
    assert not any(part['id'].startswith('person/') for part in saved['parts'])
    reopened = Assembly.from_doc(copy.deepcopy(saved))
    assert np.allclose(reopened.parts['person/left_hand'].matrix,
                       reflection(aligned['objects'][0]['symmetry']) @
                       reopened.parts['person/right_hand'].matrix @ np.diag([-1, 1, 1, 1]), atol=1e-3)
    assert all(abs(reopened.parts['person/' + name].matrix[0, 3]) < 1e-4
               for name in ('pelvis', 'lumbar', 'thorax', 'neck', 'head'))
    baked = materialize_mirror(reopened, 'frame', 'center')
    assert len(baked['objects']) == 1
    assert not any(part['id'].startswith('person/') for part in baked['parts'])
    Assembly.from_doc(baked)


def test_scene_mirror_adopts_the_already_raised_arm_and_can_be_explicitly_freed():
    doc = design()
    doc['draft_subassemblies'][0]['mirrors'][0]['scope'] = 'scene'
    before = Assembly.from_doc(doc)
    target = pose_of(before.parts['person/right_hand'].matrix)
    target['position_mm'][1] += 80
    posed = Assembly.from_doc(transform_part(before, 'person/right_hand', target)['document'])
    right_before = posed.parts['person/right_hand'].matrix.copy()
    right_before[0, 3] -= 300  # The whole person's centre moves from X=300 to X=0.
    aligned = sync_scene_humans(posed)
    after = Assembly.from_doc(aligned)
    assert np.allclose(after.parts['person/right_hand'].matrix, right_before, atol=1e-3)
    assert np.allclose(after.parts['person/left_hand'].matrix,
                       reflection(aligned['objects'][0]['symmetry']) @ right_before @ np.diag([-1, 1, 1, 1]), atol=1e-3)
    free = set_human_symmetry(after, 'person')
    assert free['objects'][0]['mirror_pose_free'] is True
    assert not sync_scene_humans(Assembly.from_doc(free))['objects'][0].get('symmetry')


def test_scene_mirror_projects_an_existing_torso_twist_onto_spinal_flex():
    doc = design()
    doc['draft_subassemblies'][0]['mirrors'][0]['scope'] = 'scene'
    before = Assembly.from_doc(doc)
    target = pose_of(before.parts['person/thorax'].matrix)
    target['rotation_deg'][2] += 10
    twisted = Assembly.from_doc(transform_part(before, 'person/thorax', target, mode='rotate')['document'])
    aligned = Assembly.from_doc(sync_scene_humans(twisted))
    assert all(abs(aligned.parts['person/' + name].matrix[0, 3]) < 1e-3
               for name in ('pelvis', 'lumbar', 'thorax', 'neck', 'head'))
    # Bending along the line remains available after the twist is removed.
    flex = pose_of(aligned.parts['person/thorax'].matrix)
    flex['rotation_deg'][0] += 15
    bent = transform_part(aligned, 'person/thorax', flex, mode='rotate')
    assert not bent['limited']
    Assembly.from_doc(bent['document'])


def test_baking_a_scene_mirror_attaches_the_copied_fitting_to_the_other_real_hand(tmp_path):
    doc = design()
    doc['draft_subassemblies'][0]['mirrors'][0]['scope'] = 'scene'
    aligned = sync_scene_humans(Assembly.from_doc(doc, tmp_path))
    hand = Assembly.from_doc(aligned, tmp_path).parts['person/right_hand'].matrix[:3, 3].tolist()
    aligned['parts'].append({'id': 'grip', 'body': {'kind': 'connector', 'mass_kg': .1,
        'geometry': [{'type': 'sphere', 'radius_mm': 5}]}, 'pose': {'position_mm': hand}})
    aligned['joints'].append({'id': 'gripped', 'type': 'spherical',
        'a': {'part': 'person/right_hand', 'frame': {'rotation_deg': [12, 20, 30]}},
        'b': {'part': 'grip'}})
    baked = materialize_mirror(Assembly.from_doc(aligned, tmp_path), 'frame', 'center')
    assert len(baked['objects']) == 1
    assert not any(part['id'].startswith('person/') for part in baked['parts'])
    mirrored = next(part['id'] for part in baked['parts'] if part['id'].startswith('grip-mirror-'))
    counterpart = next(joint for joint in baked['joints'] if
                       {joint['a']['part'], joint['b']['part']} == {'person/left_hand', mirrored})
    original_rotation = transform({'rotation_deg': [12, 20, 30]})
    expected_rotation = np.diag([-1, 1, 1, 1]) @ original_rotation @ np.diag([-1, 1, 1, 1])
    assert np.allclose(transform({'rotation_deg': counterpart['a']['frame']['rotation_deg']}),
                       expected_rotation, atol=1e-6)
    Assembly.from_doc(baked, tmp_path)
