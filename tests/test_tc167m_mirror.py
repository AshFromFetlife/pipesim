"""The inline double swivel male exposes two opposite, mirrorable eye ports."""

import copy

import numpy as np
import pytest

from pipesim.document import Assembly
from pipesim.math3d import pose_of
from pipesim.symmetry import materialize_mirror
from pipesim.validation import validate


@pytest.mark.parametrize('size,diameter,mass,dimensions', [
    ('A', 26.9, .226, (108.6, 7.4, 75.9, 32.6, 32.9, 8.4)),
    ('B', 33.7, .334, (124.3, 8.9, 86.2, 38.5, 41.1, 10.5)),
    ('C', 42.4, .412, (134.0, 10.1, 95.5, 38.5, 45.9, 10.5)),
    ('D', 48.3, .458, (144.8, 9.6, 105.8, 38.9, 48.4, 10.5)),
    ('E', 60.3, .500, (161.0, 9.0, 120.0, 38.0, 53.0, 10.5)),
])
def test_tc167m_catalogue_matches_supplier_drawing(library, size, diameter, mass, dimensions):
    part = library.parts[f'tubeclamp.TC167M{size}']
    assert part['mass_kg'] == mass
    assert tuple(part['drawing_dimensions_mm'][key] for key in 'fghijk') == dimensions
    assert part['ports']['through']['diameter_mm'] == diameter
    assert part['ports']['through']['through'] is True
    right, left = part['ports']['hinge_right'], part['ports']['hinge_left']
    assert right['type'] == left['type'] == 'eye'
    assert right['diameter_mm'] == left['diameter_mm'] == dimensions[-1]
    assert np.allclose(right['position_mm'], -np.array(left['position_mm']))
    assert np.linalg.norm(right['position_mm']) == pytest.approx(dimensions[2]/2)
    assert np.allclose(right['axis'], left['axis'])
    assert abs(np.dot(right['axis'], right['position_mm'])) < 1e-10


def _inline_swivel_with_one_hinge(library, size):
    male = library.parts[f'tubeclamp.TC167M{size}']
    female = library.parts[f'tubeclamp.TC173F{size}']
    eye_x = male['ports']['hinge_right']['position_mm'][0]
    angle = np.deg2rad(45)
    bolt_axis = np.array([0., np.cos(angle), np.sin(angle)])
    radial = np.array([1., 0., 0.])
    basis = np.column_stack((np.cross(bolt_axis, radial), bolt_axis, radial))
    assert np.linalg.det(basis) == pytest.approx(1)
    matrix = np.eye(4)
    matrix[:3, :3] = basis
    matrix[:3, 3] = np.array([eye_x, 0., 500.]) - basis @ np.array(female['ports']['hinge']['position_mm'])
    return {
        'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
        'name': 'TC167M inline swivel mirrored hinges',
        'parts': [
            {'id': 'inline', 'catalog': f'tubeclamp.TC167M{size}',
             'pose': {'position_mm': [0, 0, 500], 'rotation_deg': [45, 0, 0]}},
            {'id': 'right-eye', 'catalog': f'tubeclamp.TC173F{size}',
             'pose': pose_of(matrix)},
        ],
        'joints': [{'id': 'right-hinge', 'type': 'revolute',
                    'a': {'part': 'inline', 'port': 'hinge_right'},
                    'b': {'part': 'right-eye', 'port': 'hinge'}}],
        'draft_subassemblies': [{
            'id': 'inline-draft', 'runs': [],
            'mirror_parts': ['inline', 'right-eye'],
            'mirrors': [{'id': 'bisector', 'axis': 'x', 'offset_mm': 0}],
        }],
    }


@pytest.mark.parametrize('size', ['A', 'B', 'C', 'D', 'E'])
def test_mirror_bisects_tc167m_between_opposing_eyes_and_preserves_hinges(tmp_path, library, size):
    doc = _inline_swivel_with_one_hinge(library, size)
    before = copy.deepcopy(doc)
    source = Assembly.from_doc(doc, tmp_path, library)
    inline = source.parts['inline']
    right, right_axis = inline.frame({'port': 'hinge_right'})
    left, left_axis = inline.frame({'port': 'hinge_left'})
    assert np.allclose(left, right * [-1, 1, 1])
    assert np.allclose(left_axis, right_axis * [-1, 1, 1])
    assert right[0] == pytest.approx(inline.definition['drawing_dimensions_mm']['h']/2)
    a, b, _ = source.joint_frames(source.joints[0])
    assert np.linalg.norm(a-b) < .01

    baked = materialize_mirror(source, 'inline-draft', 'bisector')
    assert doc == before
    assert [part['id'] for part in baked['parts']] == ['inline', 'right-eye', 'right-eye-mirror-bisector']
    assert len(baked['joints']) == 2
    assert {(j['a']['port'], j['b']['part']) for j in baked['joints']} == {
        ('hinge_right', 'right-eye'), ('hinge_left', 'right-eye-mirror-bisector')}

    result = Assembly.from_doc(baked, tmp_path, library)
    assert np.allclose(result.parts['right-eye-mirror-bisector'].matrix[:3, 3],
                       result.parts['right-eye'].matrix[:3, 3] * [-1, 1, 1])
    for joint in result.joints:
        a, b, axis = result.joint_frames(joint)
        assert np.linalg.norm(a-b) < .01
        assert abs(np.dot(axis, result.parts[joint['b']['part']].frame(joint['b'])[1])) > .999
    finished = copy.deepcopy(baked)
    finished.pop('draft_subassemblies')
    assert validate(Assembly.from_doc(finished, tmp_path, library))['valid']
