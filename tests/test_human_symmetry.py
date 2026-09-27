import copy

import numpy as np
import pytest

from pipesim.document import Assembly, DocumentError
from pipesim.duplication import duplicate
from pipesim.grouping import move_object
from pipesim.human_symmetry import reflection, set_human_symmetry
from pipesim.math3d import pose_of
from pipesim.posing import transform_part


def design(axis='x'):
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': 'Mirror-line human',
            'parts': [], 'joints': [], 'objects': [{'id': 'person', 'template': 'human',
                'parameters': {'pose': 'standing'}, 'pose': {'position_mm': [300, 120, 0]}}],
            'draft_subassemblies': [{'id': 'frame', 'runs': [{
                'id': 'reference-pipe', 'catalog': 'tubeclamp.tube-C',
                'start_mm': [0, 0, 100], 'end_mm': [0, 0, 1100]}],
                'mirrors': [{'id': 'center', 'axis': axis, 'offset_mm': 0}]}]}


@pytest.mark.parametrize('axis', ['x', 'y'])
def test_human_fixes_to_a_vertical_mirror_line_and_only_slides_vertically(axis):
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
