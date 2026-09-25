import copy
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.document import Assembly, DocumentError, read
from pipesim.math3d import pose_of
from pipesim.resizing import resize_member
from pipesim.snapping import connection_options
from pipesim.validation import validate


@pytest.fixture
def square(factory):
    return factory(read(Path(__file__).parent/'fixtures/resize-square.pipe.yaml'))


def accepted(assembly, member, length, **kwargs):
    result=resize_member(assembly, member, length, **kwargs)
    assert result['status']=='resized',result
    final=Assembly.from_doc(result['document'],assembly.base,assembly.library)
    errors=[i for i in validate(final,collisions=False)['issues'] if i['severity']=='error']
    assert not errors,errors
    return final,result


def tee_at(blank,factory,station=500,locked=True):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},'pose':{'position_mm':[0,0,600]}},
                  {'id':'tee','catalog':'tubeclamp.TC101C','pose':{'position_mm':[300,0,600]}}]
    preview=connection_options(factory(doc),'pipe','tee','through',at_mm=station,locked=locked)
    return factory(next(o['document'] for o in preview['options'] if o['move']=='connector'))


def test_resize_moves_end_fittings_and_complete_branches(load):
    before=load('tee-midpoint');original=copy.deepcopy(before.doc)
    after,result=accepted(before,'spine',1200)
    assert set(result['moved'])=={'lower-tee','lower-crossbar','upper-tee','upper-crossbar'}
    assert after.parts['spine'].length==1200
    assert np.allclose(after.parts['spine'].matrix,before.parts['spine'].matrix)
    for prefix,distance in [('lower',-100),('upper',100)]:
        for suffix in ('tee','crossbar'):
            pid=prefix+'-'+suffix
            assert np.allclose(after.parts[pid].matrix[:3,3]-before.parts[pid].matrix[:3,3],[0,0,distance])
            assert np.allclose(after.parts[pid].matrix[:3,:3],before.parts[pid].matrix[:3,:3])
        assert after.parts[prefix+'-crossbar'].length==1000
    assert after.doc['joints']==before.doc['joints']
    assert before.doc==original


@pytest.mark.parametrize('anchor,delta',[('lower-tee',100),('upper-tee',-100)])
def test_resize_keeps_floor_or_ceiling_fitting_fixed(load,factory,anchor,delta):
    doc=load('tee-midpoint').doc;doc['anchors']=[{'part':anchor,'surface':'ceiling' if delta<0 else 'floor'}]
    before=factory(doc);after,_=accepted(before,'spine',1200)
    assert np.allclose(after.parts[anchor].matrix,before.parts[anchor].matrix)
    assert np.allclose(after.parts['spine'].matrix[:3,3]-before.parts['spine'].matrix[:3,3],[0,0,delta])


@pytest.mark.parametrize('flip',[False,True])
def test_locked_through_station_is_from_start_in_any_orientation(blank,factory,flip):
    before=tee_at(blank,factory,400)
    if flip:
        before.doc['parts'][1]['pose']['rotation_deg']=[180,0,0]
        before=factory(before.doc)
    rotation=np.eye(4);rotation[:3,:3]=Rotation.from_euler('xyz',[31,47,63],degrees=True).as_matrix()
    for p in before.doc['parts']: p['pose']=pose_of(rotation@before.parts[p['id']].matrix)
    before=factory(before.doc)
    after,_=accepted(before,'pipe',1200)
    assert after.doc['joints'][0]['b']['at_mm']==400
    assert np.allclose(after.parts['tee'].matrix[:3,3]-before.parts['tee'].matrix[:3,3],-100*before.parts['pipe'].matrix[:3,2])


def test_locked_square_blocks_resize_without_mutation_and_identifies_socket(square):
    assert validate(square)['valid']
    before=copy.deepcopy(square.doc)
    result=resize_member(square,'bottom',900)
    assert result['status']=='blocked' and 'closed path' in result['reason']
    assert any(b['joint']=='bottom-end' and b['connector']=='corner-1' and b['port']=='x' for b in result['blockers'])
    assert result['suggestions'] and all(s['action']=='detach' for s in result['suggestions'])
    for suggestion in result['suggestions']:
        after,_=accepted(square,'bottom',900,releases=suggestion['releases'])
        assert len(after.joints)==7 and all(j['locked'] for j in after.joints)
        assert all(p.length==1000 for pid,p in after.parts.items() if p.kind=='member' and pid!='bottom')
    assert square.doc==before


@pytest.mark.parametrize('length,count',[(995,1),(990,2)])
def test_square_suggests_only_screws_that_provide_enough_engagement(square,length,count):
    result=resize_member(square,'bottom',length)
    assert result['status']=='blocked'
    suggestion=result['suggestions'][0]
    assert suggestion['action']=='loosen' and len(suggestion['joints'])==count
    after,_=accepted(square,'bottom',length,releases=suggestion['releases'])
    assert len(after.joints)==8
    assert {j['id'] for j in after.joints if not j['locked']}==set(suggestion['joints'])
    for joint in after.joints:
        if not joint['locked']:
            assert 24.115-1e-5<=joint['insertion_mm']<30
    assert validate(after)['valid']


def test_two_world_anchors_explain_why_the_end_cannot_move(load,factory):
    doc=load('tee-midpoint').doc;doc['anchors']=[{'part':'lower-tee','surface':'floor'},{'part':'upper-tee','surface':'ceiling'}]
    before=factory(doc);result=resize_member(before,'spine',1200,suggest=False)
    assert result['status']=='blocked' and 'World anchors' in result['reason']
    assert {b['anchor'] for b in result['blockers'] if 'anchor' in b}=={'lower-tee','upper-tee'}


def test_loose_anchored_collar_rebases_station_travel_motor_and_animation(blank,factory):
    before=tee_at(blank,factory,500,locked=False);doc=before.doc
    doc['anchors']=[{'part':'pipe','surface':'fixture'},{'part':'tee','surface':'fixture'}]
    joint=doc['joints'][0];joint['limits']={'slide_mm':[-120,120]}
    joint['motor']={'mode':'position','target':50,'max_force_n':100,'schedule':[{'time_s':0,'target':50}]}
    doc['animation']={'tracks':[{'joint':joint['id'],'coordinate':'slide_mm','keyframes':[{'time_s':0,'value':0},{'time_s':1,'value':50}]}]}
    before=factory(doc);after,result=accepted(before,'pipe',1200)
    assert result['moved']==[]
    new=after.doc['joints'][0]
    assert new['b']['at_mm']==pytest.approx(600)
    assert new['limits']['slide_mm']==pytest.approx([-20,220])
    assert new['motor']['target']==pytest.approx(150)
    assert new['motor']['schedule'][0]['target']==pytest.approx(150)
    assert [k['value'] for k in after.doc['animation']['tracks'][0]['keyframes']]==pytest.approx([100,150])
    blocked=resize_member(before,'pipe',1300,suggest=False)
    assert blocked['status']=='blocked' and 'slide travel' in blocked['reason']


def test_socket_cannot_slide_beyond_the_new_pipe_end(blank,factory):
    before=tee_at(blank,factory,900)
    result=resize_member(before,'pipe',800,suggest=False)
    assert result['status']=='blocked' and 'engagement' in result['reason']
    assert result['blockers'][0]['joint']==before.joints[0]['id']
    before.doc['joints'][0]['locked']=False
    after,_=accepted(factory(before.doc),'pipe',800)
    assert after.doc['joints'][0]['b']['at_mm']<800


def test_loose_collar_chooses_a_feasible_pose_at_its_travel_limit(blank,factory):
    before=tee_at(blank,factory,500,locked=False)
    before.doc['joints'][0]['limits']={'slide_mm':[-50,50]}
    before=factory(before.doc);after,_=accepted(before,'pipe',1200)
    # An unrestrained collar stays near its old world location, stopping at its
    # allowed 50 mm travel rather than moving off-axis or outside the bore.
    assert after.parts['tee'].matrix[2,3]==pytest.approx(before.parts['tee'].matrix[2,3]-50)
    assert after.doc['joints'][0]['b']['at_mm']==pytest.approx(550)
    assert after.doc['joints'][0]['limits']['slide_mm']==pytest.approx([0,100])


def test_resize_refuses_a_new_obstruction(blank,factory):
    before=tee_at(blank,factory)
    before.doc['parts'].append({'id':'obstacle','body':{'kind':'rigid','mass_kg':1,'geometry':[{'type':'box','size_mm':[100,100,50]}]},'pose':{'position_mm':[0,0,1190]}})
    before=factory(before.doc);result=resize_member(before,'pipe',1400)
    assert result['status']=='blocked' and 'obstacle' in result['reason']
    assert not result['suggestions']
    assert before.parts['pipe'].length==1000


def test_attached_reusable_human_moves_without_changing_pose_or_expanding(load,factory):
    before=load('tee-midpoint');doc=before.doc
    doc['objects']=[{'id':'person','template':'human','parameters':{'pose':'seated'},'pose':{'position_mm':[1500,0,1800]}}]
    initial=factory(doc);tee=initial.parts['upper-tee'];hand=initial.parts['person/right_hand']
    pivot=hand.matrix[:3,3]
    doc['joints'].append({'id':'hand-support','type':'spherical','a':{'part':tee.id,'frame':{'position_mm':(tee.matrix[:3,:3].T@(pivot-tee.matrix[:3,3])).tolist()}},'b':{'part':hand.id}})
    before=factory(doc);after,_=accepted(before,'spine',1200,collisions=False)
    assert len(after.doc['parts'])==len(before.doc['parts'])
    assert after.doc['objects'][0]['parameters']==before.doc['objects'][0]['parameters']
    for pid in before.parts:
        if pid.startswith('person/'):
            assert np.allclose(after.parts[pid].matrix[:3,3]-before.parts[pid].matrix[:3,3],[0,0,100])


@pytest.mark.parametrize('length',[0,-1,float('nan'),float('inf'),'1000',True])
def test_invalid_lengths_never_change_the_document(blank,factory,length):
    before=tee_at(blank,factory);original=copy.deepcopy(before.doc)
    with pytest.raises(DocumentError,match='finite positive'): resize_member(before,'pipe',length)
    assert before.doc==original


@pytest.mark.parametrize('catalog',['tubeclamp.tube-C','porta.DOW-19','minitec.20.1006'])
def test_free_members_resize_the_geometry_and_mass(blank,factory,catalog):
    from pipesim.geometry import bounds
    blank['parts']=[{'id':'member','catalog':catalog,'parameters':{'length_mm':1000},'pose':{'position_mm':[0,0,500]}}]
    before=factory(blank);after,_=accepted(before,'member',800)
    assert after.parts['member'].mass==pytest.approx(before.parts['member'].mass*.8)
    box=bounds(after.parts['member']);assert box[1,2]-box[0,2]==pytest.approx(800)
    assert np.allclose(after.parts['member'].matrix,before.parts['member'].matrix)


def test_fixed_custom_geometry_cannot_change_only_its_declared_length(blank,factory):
    blank['parts']=[{'id':'custom','parameters':{'length_mm':1000},'body':{'kind':'member','mass_kg':1,'geometry':[{'type':'cylinder','diameter_mm':20,'length_mm':1000}]}}]
    with pytest.raises(DocumentError,match='geometry use'): resize_member(factory(blank),'custom',800)


def test_a_saved_joint_pose_requires_capturing_the_frame_first(blank,factory):
    before=tee_at(blank,factory,locked=False)
    before.doc['state']={'joints':{before.joints[0]['id']:{'slide_mm':10}}}
    with pytest.raises(DocumentError,match='Capture the motion frame'): resize_member(factory(before.doc),'pipe',1200)


def test_releases_cannot_change_an_unrelated_structure(blank,factory):
    before=tee_at(blank,factory)
    before.doc['parts'].append({'id':'other','catalog':'tubeclamp.tube-C','pose':{'position_mm':[500,500,500]}})
    with pytest.raises(DocumentError,match='connected structure'):
        resize_member(factory(before.doc),'other',1200,releases=[{'joint':before.joints[0]['id'],'action':'detach'}])
