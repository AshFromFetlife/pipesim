import copy
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.document import Assembly,read
from pipesim.math3d import pose_of
from pipesim.snapping import connection_options
from pipesim.validation import validate


@pytest.fixture
def corner(factory):
    return factory(read(Path(__file__).parent/'fixtures/sliding-corner.pipe.yaml'))


def connect(assembly,**kwargs):
    return connection_options(assembly,'bottom-arm','elbow','x',end='end',insertion_mm=30,**kwargs)


def coordinated(result):
    option=next(o for o in result['options'] if o['move']=='both')
    assert option['available'],option.get('reason')
    assert option['requires_preview']
    return option


def assert_frame_preserved(before,after):
    for pid in ['bottom','left','top','right',*[f'corner-{i}' for i in range(4)]]:
        assert np.allclose(after.parts[pid].matrix,before.parts[pid].matrix,rtol=0,atol=1e-7)
    assert before.doc.get('anchors',[])==after.doc.get('anchors',[])
    for pid in before.parts:
        assert before.parts[pid].length==after.parts[pid].length
        assert np.allclose(before.parts[pid].matrix[:3,:3],after.parts[pid].matrix[:3,:3],rtol=0,atol=1e-7)


@pytest.mark.parametrize('anchor',[None,'corner-0','corner-2'])
def test_perpendicular_sliding_tees_close_the_corner_without_moving_its_frame(corner,factory,anchor):
    if anchor: corner.doc['anchors']=[{'part':anchor,'surface':'ceiling'}];corner=factory(corner.doc)
    original=copy.deepcopy(corner.doc);result=connect(corner)
    assert result['recommended']=='both'
    assert not result['options'][0]['available'] and not result['options'][1]['available']
    option=coordinated(result);after=factory(option['document'])
    assert validate(after)['valid']
    assert_frame_preserved(corner,after)
    assert set(option['moved'])=={'left-tee','bottom-tee','left-arm','bottom-arm','elbow'}
    assert np.allclose(after.parts['left-tee'].matrix[:3,3],[0,466.7,500],atol=1e-7)
    assert np.allclose(after.parts['bottom-tee'].matrix[:3,3],[466.7,0,500],atol=1e-7)
    assert {s['joint']:s['slide_mm'] for s in option['slides']}==pytest.approx({'left-slide':16.7,'bottom-slide':133.3})
    assert {'left','bottom'}<=set(option['context_parts'])
    assert len(after.joints)==len(corner.joints)+1
    for j in corner.joints:
        assert next(after_j for after_j in after.joints if after_j['id']==j['id'])['locked']==j['locked']
    assert corner.doc==original


@pytest.mark.parametrize('lock',['left-slide','bottom-slide'])
def test_a_locked_tee_prevents_using_its_sliding_freedom(corner,factory,lock):
    next(j for j in corner.doc['joints'] if j['id']==lock)['locked']=True
    corner=factory(corner.doc);result=connect(corner)
    assert result['recommended'] is None
    assert all(not o['available'] for o in result['options'])


def test_sliding_tees_and_loose_branch_sockets_are_solved_as_one_mechanism(corner,factory):
    for j in corner.doc['joints']:
        if j['id'] in ['left-arm-seat','bottom-arm-seat']: j['locked']=False
    corner=factory(corner.doc);option=coordinated(connect(corner));after=factory(option['document'])
    assert validate(after)['valid']
    assert_frame_preserved(corner,after)
    assert len(option['slides'])==4
    assert len(after.rigid_groups())>1


def test_joint_stations_limits_motors_tracks_and_drives_rebase_together(corner,factory):
    doc=corner.doc
    left=next(j for j in doc['joints'] if j['id']=='left-slide');left['limits']={'slide_mm':[-100,100]}
    left['motor']={'target':50,'mode':'position','max_force_n':100,'schedule':[{'time_s':0,'target':80}]}
    bottom=next(j for j in doc['joints'] if j['id']=='bottom-slide');bottom['limits']={'slide_mm':[-200,200]}
    doc['animation']={'tracks':[{'joint':'left-slide','coordinate':'slide_mm','keyframes':[{'time_s':0,'value':0},{'time_s':1,'value':50}]}]}
    doc['parts'].append({'id':'driver','body':{'kind':'rigid','mass_kg':1,'geometry':[{'type':'box','size_mm':[10,10,10]}]},'pose':{'position_mm':[0,0,650]}})
    doc['joints'].append({'id':'driver-axis','type':'revolute','a':{'part':'corner-0','frame':{'position_mm':[0,0,150]}},'b':{'part':'driver'}})
    doc['drives']=[{'id':'belt','type':'gt2','driver':'driver-axis','follower':'left-slide','offset_mm':40}]
    corner=factory(doc);after=factory(coordinated(connect(corner))['document'])
    j=next(j for j in after.joints if j['id']=='left-slide')
    assert j['b']['at_mm']==pytest.approx(566.1)
    assert j['limits']['slide_mm']==pytest.approx([-116.7,83.3])
    assert j['motor']['target']==pytest.approx(33.3)
    assert j['motor']['schedule'][0]['target']==pytest.approx(63.3)
    assert [k['value'] for k in after.doc['animation']['tracks'][0]['keyframes']]==pytest.approx([-16.7,33.3])
    assert after.doc['drives'][0]['offset_mm']==pytest.approx(23.3)
    assert validate(after)['valid']


def test_insufficient_slide_travel_still_blocks_and_names_the_joint(corner,factory):
    next(j for j in corner.doc['joints'] if j['id']=='bottom-slide')['limits']={'slide_mm':[-50,50]}
    result=connect(factory(corner.doc));option=next(o for o in result['options'] if o['move']=='both')
    assert not option['available'] and 'bottom-slide' in option['reason'] and 'travel' in option['reason']


def test_a_socket_cannot_leave_its_rail_to_make_the_connection(corner,factory):
    # Lengthen the two independent inner arms as complete seated branches. The
    # requested meeting point is now beyond the outer rails' usable length.
    from pipesim.resizing import resize_member
    first=resize_member(corner,'left-arm',1500,collisions=False,suggest=False)
    assert first['status']=='resized'
    second=resize_member(factory(first['document']),'bottom-arm',1500,collisions=False,suggest=False)
    assert second['status']=='resized'
    result=connect(factory(second['document']));option=next(o for o in result['options'] if o['move']=='both')
    assert not option['available'] and 'engagement' in option['reason']


def test_collision_check_still_rejects_the_coordinated_pose(corner,factory):
    corner.doc['parts'].append({'id':'obstruction','body':{'kind':'rigid','mass_kg':1,'geometry':[{'type':'box','size_mm':[20,20,20]}]},'pose':{'position_mm':[466.7,200,500]}})
    result=connect(factory(corner.doc));option=next(o for o in result['options'] if o['move']=='both')
    assert not option['available'] and 'obstruction' in option['reason'] and 'intersect' in option['reason']


def test_rotated_frames_and_opposite_through_axes_keep_the_same_solution(corner,factory):
    transform=np.eye(4);transform[:3,:3]=Rotation.from_euler('xyz',[17,-31,53],degrees=True).as_matrix();transform[:3,3]=[300,-800,900]
    for p in corner.doc['parts']: p['pose']=pose_of(transform@corner.parts[p['id']].matrix)
    corner=factory(corner.doc);after=factory(coordinated(connect(corner))['document'])
    assert_frame_preserved(corner,after)
    assert validate(after)['valid']
    assert np.allclose(after.parts['elbow'].matrix[:3,3],transform[:3,:3]@[466.7,466.7,500]+transform[:3,3],atol=1e-6)


def test_camera_plane_drag_is_projected_onto_real_slide_axes(corner,factory):
    poses={}
    for pid in ['left-tee','left-arm','elbow']:
        pose=pose_of(corner.parts[pid].matrix);pose['position_mm']=(np.array(pose['position_mm'])+[100,20,70]).tolist();poses[pid]=pose
    option=coordinated(connect(corner,poses=poses));after=factory(option['document'])
    assert_frame_preserved(corner,after)
    assert np.allclose(after.parts['elbow'].matrix[:3,3],[466.7,466.7,500],atol=1e-6)
    assert validate(after)['valid']


def test_free_elbow_avoids_the_other_arm_then_closes_both_sliding_branches(factory):
    before=factory(read(Path(__file__).parent/'fixtures/sliding-free-elbow.pipe.yaml'))
    original=copy.deepcopy(before.doc);frame=max(before.rigid_groups(),key=len)
    first=coordinated(connection_options(before,'tube-c-7','tc125c-1','x',end='end',insertion_mm=30))
    middle=factory(first['document'])
    assert validate(middle)['valid']
    second=coordinated(connection_options(middle,'tube-c-8','tc125c-1','z',end='start',insertion_mm=30))
    after=factory(second['document'])
    assert validate(after)['valid']
    for pid in frame:
        assert np.allclose(before.parts[pid].matrix,after.parts[pid].matrix,atol=1e-7,rtol=0)
    assert len(after.joints)==len(before.joints)+2
    for j in before.joints:
        assert next(k for k in after.joints if k['id']==j['id'])['locked']==j['locked']
    assert before.doc==original
