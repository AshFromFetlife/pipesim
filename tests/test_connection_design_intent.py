import numpy as np
import pytest

from connection_intent_cases import connect, repeat_connections, square_case


@pytest.mark.parametrize('force',[False,True])
def test_reconnecting_valid_square_geometry_does_not_accumulate_drift(factory,force):
    source=square_case(travel=0)
    before=factory(source)
    option,after=repeat_connections(source,factory,force=force)
    assert option['move']=='keep' and not option['moved']
    assert not option.get('requires_preview')
    for pid in before.parts: assert np.array_equal(before.parts[pid].matrix,after.parts[pid].matrix)


def test_translation_keeps_round_positions_and_plumb_angles(factory):
    option,after=repeat_connections(square_case(),factory)
    assert option['move']=='connector' and option['rotation_deg']==0
    assert np.array_equal(after.parts['fitting'].matrix[:3,3],[0,0,980])


@pytest.mark.parametrize('force',[False,True])
def test_articulated_fit_slides_without_tipping_a_square_upright(factory,force):
    option,after=repeat_connections(square_case(articulated=True,tilt=0),factory,force=force)
    assert np.array_equal(after.parts['pipe'].matrix[:3,3],[50,0,500])
    assert option['rotation_deg']==0


def test_strict_tolerance_can_still_require_a_nonround_position_and_angle(factory):
    source=square_case(tilt=6)
    option,after=connect(source,factory,tolerance_mm=.01)
    assert option['gap_mm']<.01
    assert option['rotation_deg']==pytest.approx(6)
    assert not np.allclose(after.parts['fitting'].matrix[:3,3],np.round(after.parts['fitting'].matrix[:3,3]))


def test_translation_falls_back_to_fractional_positions_when_tolerance_requires_it(factory):
    option,_=connect(square_case(),factory,tolerance_mm=.01)
    assert option['gap_mm']<.01 and option['rotation_deg']==0


def test_continuous_solver_prefers_slide_to_a_small_upright_rotation(factory):
    from pipesim.fitting import fit_connection
    from pipesim.math3d import transform
    before=factory(square_case(articulated=True,tilt=0))
    joint={'id':'join','type':'socket','a':{'part':'fitting','port':'seat'},
           'b':{'part':'pipe','end':'end'},'insertion_mm':20,'locked':True,'fit_tolerance_mm':2}
    poses,_,_,_,angle=fit_connection(before,joint,before,True)
    assert angle==0
    assert np.array_equal(transform(poses['pipe'])[:3,3],[50,0,500])


def test_force_edit_allowances_do_not_recut_an_already_valid_frame(factory):
    from pathlib import Path
    from pipesim.document import read
    from pipesim.snapping import connection_options
    source=read(Path(__file__).parent/'fixtures/assembly-adjustment.pipe.yaml')
    before=factory(source)
    result=connection_options(before,'bottom-arm','elbow','x',end='end',insertion_mm=30,force=True,
        force_options={'unlock_connectors':2,'resize_members':2})
    option=next(o for o in result['options'] if o['move']==result['recommended'])
    assert option['move']=='keep' and not option['moved']
    assert option['document']['parts']==source['parts']
