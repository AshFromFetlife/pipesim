import copy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.document import Assembly
from pipesim.math3d import pose_of
from pipesim.posing import transform_part
from pipesim.snapping import _movement_coordinates, connection_options
from pipesim.validation import validate


def human(factory,blank,pose='seated'):
    doc=copy.deepcopy(blank)
    doc['objects']=[{'id':'person','template':'human','parameters':{'pose':pose}},
                    {'id':'bystander','template':'human','pose':{'position_mm':[2000,0,0]}}]
    return factory(doc)


@pytest.mark.parametrize('pose',['standing','seated','pull-up'])
def test_shin_translation_bends_hip_and_knee_and_brings_ankle_and_foot(factory,blank,pose):
    before=human(factory,blank,pose);original=copy.deepcopy(before.doc)
    target=pose_of(before.parts['person/left_shin'].matrix);target['position_mm'][1]+=30
    result=transform_part(before,'person/left_shin',target);after=factory(result['document'])
    assert result['position_error_mm']<1
    assert set(result['moved'])=={'person/left_thigh','person/left_shin','person/left_foot'}
    _movement_coordinates(before,after)
    for j in after.joints:
        pa,pb,_=after.joint_frames(j)
        assert np.linalg.norm(pa-pb)<1e-5,j['id']
    assert [o['id'] for o in after.doc['objects']]==['person','bystander']
    assert 'components' in after.doc['objects'][0]
    assert after.doc['objects'][1]==original['objects'][1]
    assert before.doc==original
    for pid in set(before.parts)-set(result['moved']):
        assert np.allclose(before.parts[pid].matrix,after.parts[pid].matrix,atol=1e-7,rtol=0)


def test_posing_a_hanging_leg_keeps_both_grips_and_the_frame_fixed(load,factory):
    before=load('human-pull-up');target=pose_of(before.parts['person/left_shin'].matrix);target['position_mm'][1]+=30
    result=transform_part(before,'person/left_shin',target);after=factory(result['document'])
    assert result['position_error_mm']<.1
    assert set(result['moved'])=={'person/left_thigh','person/left_shin','person/left_foot'}
    _movement_coordinates(before,after)
    for pid in set(before.parts)-set(result['moved']):
        assert np.allclose(before.parts[pid].matrix,after.parts[pid].matrix,atol=1e-7,rtol=0)
    assert validate(after,collisions=False)['valid']


def cross(factory,blank):
    doc=copy.deepcopy(blank);doc['parts']=[
        {'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},'pose':{'position_mm':[0,0,500]}},
        {'id':'cross','catalog':'tubeclamp.TC161C'}]
    doc['anchors']=[{'part':'pipe','surface':'ceiling'}]
    options=connection_options(factory(doc),'pipe','cross','cross',at_mm=500,locked=False)
    return factory(next(o['document'] for o in options['options'] if o['available']))


@pytest.mark.parametrize('angle',[30,90,-90])
def test_offset_cross_rotates_about_the_pipe_instead_of_its_mesh_origin(factory,blank,angle):
    before=cross(factory,blank);target=before.parts['cross'].matrix.copy()
    target[:3,:3]=Rotation.from_euler('z',angle,degrees=True).as_matrix()@target[:3,:3]
    result=transform_part(before,'cross',pose_of(target),'rotate');after=factory(result['document'])
    assert result['angle_error_deg']<.001
    assert np.linalg.norm(after.parts['cross'].matrix[:3,3]-before.parts['cross'].matrix[:3,3])>10
    assert np.allclose(after.parts['pipe'].matrix,before.parts['pipe'].matrix,atol=1e-8,rtol=0)
    assert validate(after)['valid']
    _movement_coordinates(before,after)


def test_a_loose_socket_projects_parallax_onto_the_rail_and_stops_before_leaving_it(factory,blank):
    before=cross(factory,blank);target=pose_of(before.parts['cross'].matrix)
    target['position_mm']=(np.array(target['position_mm'])+[80,50,1000]).tolist()
    result=transform_part(before,'cross',target);after=factory(result['document'])
    assert result['limited'] and result['moved']==['cross']
    # A cylindrical socket may also turn to approach the target, while its bore
    # stays on the pipe and the offset keeps its fixed radius.
    assert np.linalg.norm(after.parts['cross'].matrix[:2,3])==pytest.approx(42.4,abs=1e-6)
    assert after.joints[0]['b']['at_mm']==pytest.approx(1000-45.1/2,abs=1e-5)
    assert validate(after)['valid']


def test_locked_and_anchored_connector_stops_without_breaking_any_connection(factory,blank):
    before=cross(factory,blank);before.doc['joints'][0]['locked']=True;before=factory(before.doc)
    target=pose_of(before.parts['cross'].matrix);target['position_mm'][0]+=30
    result=transform_part(before,'cross',target)
    assert result['limited'] and not result['moved']
    assert result['document']==before.doc


def test_hinge_limit_clamps_rotation_and_rebases_remaining_travel(factory,blank):
    before=cross(factory,blank);before.doc['joints'][0]['limits']={'angle_deg':[-20,20],'slide_mm':[-40,40]};before=factory(before.doc)
    target=before.parts['cross'].matrix.copy();target[:3,:3]=Rotation.from_euler('z',80,degrees=True).as_matrix()@target[:3,:3]
    result=transform_part(before,'cross',pose_of(target),'rotate');after=factory(result['document'])
    assert result['limited'] and result['angle_error_deg']==pytest.approx(60,abs=.001)
    assert sorted(after.joints[0]['limits']['angle_deg'])==pytest.approx([0,40],abs=.001)
    _movement_coordinates(before,after)
    assert validate(after)['valid']


def test_selected_rigid_body_brings_all_locked_parts(factory,blank):
    before=cross(factory,blank);before.doc['anchors']=[];before.doc['joints'][0]['locked']=True;before=factory(before.doc)
    target=pose_of(before.parts['cross'].matrix);target['position_mm'][0]+=30
    result=transform_part(before,'cross',target);after=factory(result['document'])
    assert set(result['moved'])=={'cross','pipe'}
    _movement_coordinates(before,after)
    assert result['position_error_mm']<1e-5


def test_pose_can_be_edited_after_a_saved_joint_state(factory,blank):
    before=human(factory,blank);before.doc['state']={'joints':{'person/left_knee':{'angle_deg':10}}}
    before.doc['animation']={'tracks':[{'joint':'person/left_knee','coordinate':'angle_deg','keyframes':[{'time_s':0,'value':10},{'time_s':1,'value':20}]}]}
    before=factory(before.doc)
    target=pose_of(before.parts['person/left_shin'].matrix);target['position_mm'][1]+=20
    result=transform_part(before,'person/left_shin',target);after=factory(result['document'])
    assert result['position_error_mm']<.1
    coordinates=_movement_coordinates(before,after)
    shift=coordinates['person/left_knee']['angle_deg']
    assert [k['value'] for k in after.doc['animation']['tracks'][0]['keyframes']]==pytest.approx([-shift,10-shift])
    assert 'state' not in after.doc


def test_offset_cross_carries_a_long_rigid_branch_around_an_unanchored_pipe(factory,blank):
    before=cross(factory,blank);doc=copy.deepcopy(before.doc);doc['anchors']=[]
    doc['parts'].append({'id':'arm','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1500}})
    options=connection_options(factory(doc),'arm','cross','through',at_mm=750)
    before=factory(next(o['document'] for o in options['options'] if o['move']=='member' and o['available']))
    target=before.parts['cross'].matrix.copy();target[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()@target[:3,:3]
    result=transform_part(before,'cross',pose_of(target),'rotate');after=factory(result['document'])
    assert result['angle_error_deg']<.001 and set(result['moved'])=={'cross','arm'}
    assert np.allclose(before.parts['pipe'].matrix,after.parts['pipe'].matrix,atol=1e-7,rtol=0)
    _movement_coordinates(before,after)
    assert validate(after)['valid']
