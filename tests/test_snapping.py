import copy

import numpy as np
import pytest

from pipesim.document import Assembly,DocumentError
from pipesim.editing import connect_member
from pipesim.snapping import connection_options,move_document
from pipesim.validation import validate


def two_parts(blank, connector='TC128C'):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},'pose':{'position_mm':[0,0,500]}},
                  {'id':'fitting','catalog':'tubeclamp.'+connector,'pose':{'position_mm':[300,200,950]}}]
    return doc


def chosen(result,side='connector'):
    option=next(o for o in result['options'] if o['move']==side)
    assert option['available'],option.get('reason')
    return option['document']


def test_misaligned_connector_rotates_onto_anchored_pipe_without_changing_length(blank,factory):
    doc=two_parts(blank); doc['anchors']=[{'part':'pipe','surface':'floor'}]
    original=copy.deepcopy(doc)
    result=connection_options(factory(doc),'pipe','fitting','x',end='end')
    final=factory(chosen(result))
    assert result['recommended']=='connector'
    assert not result['options'][1]['available']
    assert np.allclose(final.parts['pipe'].matrix,factory(doc).parts['pipe'].matrix)
    assert final.parts['pipe'].length==1000
    assert validate(final)['valid']
    assert doc==original  # Preview has no side effects.


def test_anchored_connector_offers_pipe_movement(blank,factory):
    doc=two_parts(blank); doc['anchors']=[{'part':'fitting','surface':'ceiling'}]
    result=connection_options(factory(doc),'pipe','fitting','z',end='start')
    assert result['recommended']=='member'
    assert not result['options'][0]['available']
    assert validate(factory(chosen(result,'member')))['valid']


@pytest.mark.parametrize('size',list('ABCDE'))
def test_long_tee_continuous_run_at_half_length_is_centred(blank,factory,size):
    doc=two_parts(blank,'TC104'+size)
    doc['parts'][0]['catalog']='tubeclamp.tube-'+size
    final=factory(chosen(connection_options(factory(doc),'pipe','fitting','through',at_mm=500)))
    assert final.joints[0]['b']=={'part':'pipe','at_mm':500.0}
    assert final.joints[0]['insertion_mm']==0
    assert np.allclose(final.parts['pipe'].matrix[:3,3],final.parts['fitting'].matrix[:3,3])
    assert validate(final)['valid']


def test_tee_modes_cannot_share_one_bore(blank,factory):
    doc=chosen(connection_options(factory(two_parts(blank,'TC104C')),'pipe','fitting','through',at_mm=500))
    doc['parts'].append({'id':'other','catalog':'tubeclamp.tube-C'})
    with pytest.raises(DocumentError,match='occupied'):
        connection_options(factory(doc),'other','fitting','run_end')
    invalid=copy.deepcopy(doc)
    invalid['joints'].append({'id':'conflict','type':'socket','a':{'part':'fitting','port':'run_end'},'b':{'part':'other','end':'start'},'insertion_mm':30,'locked':True})
    assert any(i['code']=='SHARED_BORE_OCCUPIED' for i in validate(factory(invalid),collisions=False)['issues'])


def test_end_insertion_cannot_be_used_as_middle_station(blank):
    with pytest.raises(DocumentError,match='through socket'):
        connect_member(two_parts(blank,'TC104C'),'.','pipe','fitting','run_end',insertion_mm=500)


def test_connection_can_be_changed_from_pipe_end_to_continuous_run(blank,factory):
    doc=two_parts(blank,'TC104C')
    doc=connect_member(doc,'.','pipe','fitting','run_end')
    old=doc['joints'][0]['id']
    result=connection_options(factory(doc),'pipe','fitting','through',at_mm=500,replace_joint=old)
    final=factory(chosen(result))
    assert len(final.joints)==1 and final.joints[0]['id']==old
    assert np.allclose(final.parts['pipe'].matrix[:3,3],final.parts['fitting'].matrix[:3,3])


def test_connected_rigid_parts_move_together(blank,factory):
    doc=two_parts(blank,'TC101C')
    doc['parts'].append({'id':'rail','catalog':'tubeclamp.tube-C','parameters':{'length_mm':500}})
    doc=connect_member(doc,'.','rail','fitting','branch')
    before=factory(doc)
    final=factory(chosen(connection_options(before,'pipe','fitting','through',at_mm=500)))
    assert np.allclose(np.linalg.inv(before.parts['fitting'].matrix)@before.parts['rail'].matrix,
                       np.linalg.inv(final.parts['fitting'].matrix)@final.parts['rail'].matrix)
    assert validate(final)['valid']


def test_loose_socket_allows_axial_move_and_rebases_remaining_travel(blank,factory):
    doc=chosen(connection_options(factory(two_parts(blank,'TC101C')),'pipe','fitting','through',at_mm=500,locked=False))
    doc['anchors']=[{'part':'pipe','surface':'fixture'}]
    doc['joints'][0]['limits']={'slide_mm':[-100,100],'angle_deg':[-90,90]}
    result=move_document(factory(doc),{'fitting':{'position_mm':[0,0,550]}})
    assert result['joints'][0]['limits']['slide_mm']==pytest.approx([-50,150])  # Pipe relative to fitting.
    assert result['joints'][0]['b']['at_mm']==pytest.approx(550)
    with pytest.raises(DocumentError,match='freedom'):
        move_document(factory(result),{'fitting':{'position_mm':[40,0,550]}})
    with pytest.raises(DocumentError,match='limits'):
        move_document(factory(result),{'fitting':{'position_mm':[0,0,620]}})


def test_anchor_and_rigid_constraints_reject_free_drag(blank,factory):
    doc=chosen(connection_options(factory(two_parts(blank,'TC101C')),'pipe','fitting','through',at_mm=500))
    with pytest.raises(DocumentError,match='whole rigid body'):
        move_document(factory(doc),{'fitting':{'position_mm':[0,0,550]}})
    doc['anchors']=[{'part':'pipe','surface':'floor'}]
    with pytest.raises(DocumentError,match='fixed to the world'):
        move_document(factory(doc),{'pipe':{'position_mm':[0,0,550]},'fitting':{'position_mm':[0,0,550]}})


def test_wrong_size_and_occupied_pipe_end_are_not_snap_targets(blank,factory):
    doc=two_parts(blank,'TC128B')
    with pytest.raises(DocumentError,match='size or profile'):
        connection_options(factory(doc),'pipe','fitting','z')
    doc=chosen(connection_options(factory(two_parts(blank)),'pipe','fitting','z'))
    doc['parts'].append({'id':'other','catalog':'tubeclamp.TC128C'})
    with pytest.raises(DocumentError,match='pipe end'):
        connection_options(factory(doc),'pipe','other','z')


def test_three_one_metre_pipes_make_an_i_with_500mm_stations(load):
    assembly=load('tee-midpoint')
    report=validate(assembly)
    assert report['valid'],report['issues']
    assert len(assembly.rigid_groups())==1
    assert all(p.length==1000 for p in assembly.parts.values() if p.kind=='member')
    for side in ('lower','upper'):
        tee=assembly.parts[side+'-tee']; bar=assembly.parts[side+'-crossbar']
        assert np.allclose(tee.matrix[:3,3],bar.matrix[:3,3])
        assert abs(bar.matrix[:3,2]@assembly.parts['spine'].matrix[:3,2])<1e-7


@pytest.mark.parametrize('travel,preferred',[(100,'connector'),(30,'member')])
def test_hinge_freedom_determines_which_side_can_align(blank,factory,travel,preferred):
    doc=two_parts(blank)
    fitting=factory(doc).parts['fitting']; socket=fitting.ports['x']
    insertion=min(30,socket['engagement_mm']*.8)
    start=np.array([300,200,950])+np.array([0,0,socket['position_mm'][0]-insertion])
    doc['parts'][0]['pose']={'position_mm':(start+[0,0,500]).tolist()}
    doc['parts'].append({'id':'base','body':{'kind':'rigid','mass_kg':1,'geometry':[{'type':'box','size_mm':[80,80,80]}]},'pose':{'position_mm':[300,200,850]}})
    doc['anchors']=[{'part':'base','surface':'fixture'}]
    doc['joints']=[{'id':'hinge','type':'revolute','a':{'part':'base','frame':{'position_mm':[0,0,100],'axis':[0,1,0]}},
                   'b':{'part':'fitting','frame':{'axis':[0,1,0]}},'limits':{'angle_deg':[-travel,travel]}}]
    result=connection_options(factory(doc),'pipe','fitting','x')
    assert result['recommended']==preferred,result
    assert validate(factory(chosen(result,preferred)))['valid']
