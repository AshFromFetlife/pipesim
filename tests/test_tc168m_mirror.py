"""The 90-degree corner male has two distinct, mirrorable hinge eyes."""

import copy

import numpy as np
import pytest

from pipesim.document import Assembly
from pipesim.math3d import pose_of
from pipesim.symmetry import materialize_mirror
from pipesim.validation import validate


@pytest.mark.parametrize('size,diameter,mass,dimensions', [
    ('A', 26.9, .216, (73.3, 7.3, 32.5, 32.5, 8.0)),
    ('B', 33.7, .318, (85.9, 8.7, 38.4, 38.4, 10.7)),
    ('C', 42.4, .292, (91.0, 8.4, 32.8, 32.8, 10.4)),
    ('D', 48.3, .438, (102.6, 9.2, 37.4, 45.7, 10.5)),
])
def test_tc168m_catalogue_matches_supplier_drawing(library, size, diameter, mass, dimensions):
    part = library.parts[f'tubeclamp.TC168M{size}']
    assert part['mass_kg'] == mass
    assert tuple(part['drawing_dimensions_mm'][key] for key in 'fghij') == dimensions
    assert part['ports']['through']['diameter_mm'] == diameter
    assert part['ports']['through']['through'] is True
    first, second = part['ports']['hinge_x'], part['ports']['hinge_y']
    assert first['type'] == second['type'] == 'eye'
    assert first['diameter_mm'] == second['diameter_mm'] == dimensions[-1]
    assert np.dot(first['axis'], second['axis']) == 0
    assert np.linalg.norm(first['position_mm']) == pytest.approx(np.linalg.norm(second['position_mm']))


def _forty_five_degree_corner(library, size):
    male = library.parts[f'tubeclamp.TC168M{size}']
    offset = male['ports']['hinge_x']['position_mm'][0]
    diagonal = np.sqrt(.5)
    radial = np.array([diagonal, diagonal, 0.])
    tangent = np.array([diagonal, -diagonal, 0.])
    hinge = radial * offset + [0, 0, 500]
    female = library.parts[f'tubeclamp.TC173F{size}']
    female_eye = np.array(female['ports']['hinge']['position_mm'], float)
    # The clevis points radially away from the male eye. Its bolt axis matches
    # hinge_x, and reflection must then land it on hinge_y.
    basis = np.column_stack((np.array([0., 0., -1.]), -tangent, radial))
    assert np.linalg.det(basis) == pytest.approx(1)
    matrix = np.eye(4)
    matrix[:3, :3] = basis
    matrix[:3, 3] = hinge - basis @ female_eye
    return {
        'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
        'name': 'TC168M corner swivel 45 degree mirror',
        'parts': [
            {'id': 'corner', 'catalog': f'tubeclamp.TC168M{size}',
             'pose': {'position_mm': [0, 0, 500], 'rotation_deg': [0, 0, 45]}},
            {'id': 'right-eye', 'catalog': f'tubeclamp.TC173F{size}',
             'pose': pose_of(matrix)},
        ],
        'joints': [{'id': 'right-hinge', 'type': 'revolute',
                    'a': {'part': 'corner', 'port': 'hinge_x'},
                    'b': {'part': 'right-eye', 'port': 'hinge'}}],
        'draft_subassemblies': [{
            'id': 'corner-draft', 'runs': [],
            'mirror_parts': ['corner', 'right-eye'],
            'mirrors': [{'id': 'bisector', 'axis': 'y', 'offset_mm': 0}],
        }],
    }


@pytest.mark.parametrize('size', ['A', 'B', 'C', 'D'])
def test_mirror_bisects_tc168m_between_eyes_and_preserves_symmetric_hinges(tmp_path, library, size):
    doc = _forty_five_degree_corner(library, size)
    before = copy.deepcopy(doc)
    source = Assembly.from_doc(doc, tmp_path, library)
    corner = source.parts['corner']
    first, first_axis = corner.frame({'port': 'hinge_x'})
    second, second_axis = corner.frame({'port': 'hinge_y'})
    assert np.allclose(first, second * [1, -1, 1])
    assert np.allclose(first_axis, second_axis * [1, -1, 1])
    assert np.allclose(first[:2], [first[0], first[0]])  # 45 degrees
    a, b, _ = source.joint_frames(source.joints[0])
    assert np.linalg.norm(a-b) < .01

    baked = materialize_mirror(source, 'corner-draft', 'bisector')
    assert doc == before
    assert [part['id'] for part in baked['parts']] == ['corner', 'right-eye', 'right-eye-mirror-bisector']
    assert len(baked['joints']) == 2
    assert {(j['a']['port'], j['b']['part']) for j in baked['joints']} == {
        ('hinge_x', 'right-eye'), ('hinge_y', 'right-eye-mirror-bisector')}

    result = Assembly.from_doc(baked, tmp_path, library)
    right, left = result.parts['right-eye'], result.parts['right-eye-mirror-bisector']
    assert np.allclose(left.matrix[:3, 3], right.matrix[:3, 3] * [1, -1, 1])
    for joint in result.joints:
        a, b, axis = result.joint_frames(joint)
        assert np.linalg.norm(a-b) < .01
        assert abs(np.dot(axis, result.parts[joint['b']['part']].frame(joint['b'])[1])) > .999
    finished = copy.deepcopy(baked)
    finished.pop('draft_subassemblies')
    assert validate(Assembly.from_doc(finished, tmp_path, library))['valid']
