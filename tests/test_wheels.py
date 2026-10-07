import numpy as np
import pytest

from pipesim.document import DocumentError
from pipesim.validation import validate
from pipesim.wheels import add_wheel, mount_wheel


def pipe_design(blank):
    doc = dict(blank)
    doc['parts'] = [{'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
                     'parameters': {'length_mm': 1000},
                     'pose': {'position_mm': [0, 0, 500]}}]
    doc['anchors'] = [{'part': 'pipe', 'surface': 'fixture'}]
    return doc


def test_add_wheel_to_pipe_creates_free_spinning_aligned_axle(factory, blank):
    assembly = factory(pipe_design(blank))
    target = {'part': 'pipe', 'frame': {'position_mm': [0, 50, 0], 'axis': [0, 1, 0]}}
    result = add_wheel(assembly, parameters={'diameter_mm': 120, 'width_mm': 40, 'mass_kg': 2}, target=target)
    after = factory(result['document'])
    assert result['wheel'] == 'wheel-1'
    assert result['joint']['type'] == 'revolute'
    assert result['joint']['a'] == target
    assert result['joint']['b'] == {'part': 'wheel-1', 'port': 'axle'}
    assert not result['joint'].get('locked')
    a, b, axis = after.joint_frames(result['joint'])
    assert np.allclose(a, b, atol=1e-6)
    assert np.allclose(axis, [0, 1, 0])
    assert after.parts['wheel-1'].definition['geometry'][0]['diameter_mm'] == 120
    assert after.parts['wheel-1'].mass == 2
    assert len(after.rigid_groups()) == 2
    report = validate(after)
    assert report['valid'], report['issues']


def test_free_wheel_can_be_mounted_later(factory, blank):
    assembly = factory(pipe_design(blank))
    free = add_wheel(assembly)
    assert free['joint'] is None
    target = {'part': 'pipe', 'frame': {'position_mm': [0, 60, 200], 'axis': [1, 0, 0]}}
    result = mount_wheel(factory(free['document']), free['wheel'], target)
    after = factory(result['document'])
    a, b, _ = after.joint_frames(result['joint'])
    assert np.allclose(a, b, atol=1e-6)
    with pytest.raises(DocumentError, match='Detach the wheel'):
        mount_wheel(after, free['wheel'], target)


@pytest.mark.parametrize('parameter,value', [('diameter_mm', 0), ('width_mm', -1), ('mass_kg', float('nan'))])
def test_wheel_requires_physical_dimensions(factory, blank, parameter, value):
    with pytest.raises(DocumentError, match='must be positive and finite'):
        add_wheel(factory(blank), parameters={parameter: value})
