"""TC148 is one stepped casting; paired tees swivel on a common pipe."""
import copy

import numpy as np
import pytest

from pipesim.editing import connect_member
from pipesim.geometry import shape_mesh
from pipesim.math3d import pose_of
from pipesim.posing import transform_part
from pipesim.validation import validate
from scipy.spatial.transform import Rotation


def tee_design(blank,size='C',paired=False):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'tee','catalog':f'tubeclamp.TC148{size}','pose':{'position_mm':[0,0,500]}},
                  {'id':'upright','catalog':f'tubeclamp.tube-{size}','parameters':{'length_mm':1000}},
                  {'id':'rail','catalog':f'tubeclamp.tube-{size}','parameters':{'length_mm':500}}]
    doc=connect_member(doc,'.','upright','tee','through',at_mm=500,locked=False)
    doc=connect_member(doc,'.','rail','tee','branch')
    doc['anchors']=[{'part':'upright','surface':'fixture'}]
    return doc


@pytest.mark.parametrize('size,width,reach,depth,mass',[
    ('A',20.3,54.8,24.7,.184),('B',25.2,57.8,24.3,.288),('C',28.5,76.9,27,.438),
    ('D',33.7,101,34.3,.636),('E',36,145,40,.95)])
def test_swivel_tee_geometry_socket_frames_and_published_dimensions(factory,blank,size,width,reach,depth,mass):
    assembly=factory(tee_design(blank,size));tee=assembly.parts['tee']
    assert tee.mass==mass
    assert tee.ports['through']['engagement_mm']==width
    assert tee.ports['branch']['position_mm']==[reach,0,width/2]
    assert tee.ports['branch']['engagement_mm']==depth
    assert not tee.ports['branch']['through']
    assert shape_mesh(tee.shapes[0],tee.base).is_volume
    report=validate(assembly);assert report['valid'],report['issues']
    assert len(assembly.rigid_groups())==2


def test_swivel_tee_turns_with_its_branch_around_the_main_pipe(factory,blank):
    before=factory(tee_design(blank));target=before.parts['tee'].matrix.copy()
    target[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()@target[:3,:3]
    result=transform_part(before,'tee',pose_of(target),'rotate');after=factory(result['document'])
    assert set(result['moved'])=={'tee','rail'}
    assert result['angle_error_deg']<.001
    assert np.allclose(before.parts['upright'].matrix,after.parts['upright'].matrix,atol=1e-6,rtol=0)
    assert validate(after)['valid']


@pytest.mark.parametrize('angle',[-30,30,90])
def test_inverted_pair_has_level_branch_axes_and_adjusts_to_supplier_range(load,factory,angle):
    before=load('swivel-short-tee')
    axes=[before.parts[p].frame({'part':p,'port':'branch'})[0] for p in ('lower-tee','upper-tee')]
    assert axes[0][2]==pytest.approx(axes[1][2])
    target=before.parts['upper-tee'].matrix.copy();target[:3,:3]=Rotation.from_euler('z',angle,degrees=True).as_matrix()@target[:3,:3]
    result=transform_part(before,'upper-tee',pose_of(target),'rotate');after=factory(result['document'])
    assert result['angle_error_deg']<.001
    assert validate(after)['valid']
