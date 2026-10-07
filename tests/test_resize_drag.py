"""End handles must have the same geometry and connector semantics."""
import copy
import os
import random
import secrets
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.document import Assembly, read
from pipesim.document import DocumentError
from pipesim.drafting import preview
from pipesim.math3d import align_axis, pose_of
from pipesim.resize_drag import _frame, resize_drag
from pipesim.validation import validate


def sample(blank, *, attached=True, extra=False):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'fitting','pose':{'position_mm':[700,0,0]},'body':{
        'kind':'connector','mass_kg':.2,'geometry':[{'type':'sphere','radius_mm':12}],
        'ports':{'bore':{'type':'socket','profile':'round','diameter_mm':42.4,
                         'position_mm':[0,0,0],'axis':[1,0,0],'through':True,
                         'engagement_mm':40,'min_engagement_mm':15}}}}]
    if extra:
        second=copy.deepcopy(doc['parts'][0]);second['id']='new-fitting'
        second['pose']['position_mm']=[1100,0,0];doc['parts'].append(second)
    doc['draft_subassemblies']=[{'id':'frame','runs':[{'id':'pipe','catalog':'tubeclamp.tube-C',
        'start_mm':[0,0,0],'end_mm':[1000,0,0],
        'attachments':[{'connector':'fitting','port':'bore'}] if attached else []}]}]
    return doc


@pytest.mark.parametrize('side',['start','end'])
def test_draft_drag_keeps_opposite_end_and_moves_through_connector(factory,blank,side):
    before=factory(sample(blank));first,last,_=_frame(before,'pipe')
    result=resize_drag(before,'pipe',800,side)
    after=factory(result['document']);a,b,_=_frame(after,'pipe')
    assert np.allclose(b if side=='start' else a,last if side=='start' else first)
    assert np.isclose(np.linalg.norm(b-a),800)
    expected=700+200*.3 if side=='start' else 700-200*.7
    assert np.isclose(after.parts['fitting'].matrix[0,3],expected)
    assert before.doc==sample(blank)


@pytest.mark.parametrize('side,position',[('start',300),('end',700)])
def test_shift_shrink_disconnects_only_excluded_socket(factory,blank,side,position):
    doc=sample(blank);doc['parts'][0]['pose']['position_mm']=[position,0,0]
    before=factory(doc)
    result=resize_drag(before,'pipe',500,side,behavior='detach')
    run=result['document']['draft_subassemblies'][0]['runs'][0]
    assert run['attachments']==[]
    assert result['document']['parts'][0]['pose']==doc['parts'][0]['pose']


@pytest.mark.parametrize('side',['start','end'])
def test_shift_shrink_preserves_a_socket_still_on_the_pipe(factory,blank,side):
    doc=sample(blank);doc['parts'][0]['pose']['position_mm']=[500,0,0]
    result=resize_drag(factory(doc),'pipe',800,side,behavior='detach')
    run=result['document']['draft_subassemblies'][0]['runs'][0]
    assert run['attachments']==[{'connector':'fitting','port':'bore'}]
    assert result['document']['parts'][0]['pose']==doc['parts'][0]['pose']


@pytest.mark.parametrize('side',['start','end'])
def test_closed_frame_stretches_parallel_member_and_preserves_fixed_end(factory,side):
    before=factory(read(Path(__file__).parent/'fixtures/resize-square.pipe.yaml'))
    first,last,_=_frame(before,'bottom')
    result=resize_drag(before,'bottom',900,side)
    after=factory(result['document']);a,b,_=_frame(after,'bottom')
    assert np.allclose(b if side=='start' else a,last if side=='start' else first)
    assert np.isclose(after.parts['bottom'].length,900)
    assert np.isclose(after.parts['top'].length,900)
    assert validate(after,collisions=False)['valid']


def test_randomly_oriented_closed_frames_resize_from_both_ends(factory):
    seed=int(os.environ.get('PIPESIM_RESIZE_SEED') or secrets.randbits(64));rng=random.Random(seed)
    baseline=read(Path(__file__).parent/'fixtures/resize-square.pipe.yaml')
    for case in range(int(os.environ.get('PIPESIM_RESIZE_FRAME_CASES','16'))):
        doc=copy.deepcopy(baseline)
        rotation=Rotation.from_euler('xyz',[rng.uniform(-70,70) for _ in range(3)],degrees=True)
        translation=np.array([rng.uniform(-1000,1000) for _ in range(3)])
        for spec in doc['parts']:
            pose=spec['pose']
            matrix=np.eye(4)
            matrix[:3,:3]=rotation.as_matrix()@Rotation.from_euler('xyz',pose.get('rotation_deg',[0,0,0]),degrees=True).as_matrix()
            matrix[:3,3]=rotation.apply(pose['position_mm'])+translation
            spec['pose']=pose_of(matrix)
        try:
            before=factory(doc);first,last,_=_frame(before,'bottom')
            requested=rng.uniform(800,1200)
            for side in ('start','end'):
                result=resize_drag(before,'bottom',requested,side)
                after=factory(result['document']);a,b,_=_frame(after,'bottom')
                assert np.allclose(b if side=='start' else a,last if side=='start' else first,atol=1e-4)
                assert np.isclose(after.parts['top'].length,requested)
                assert validate(after,collisions=False)['valid']
        except Exception as error:
            pytest.fail(f'closed-frame resize seed={seed} case={case}: {error}')


@pytest.mark.parametrize('side',['start','end'])
def test_single_finished_member_resizes_from_either_end(factory,blank,side):
    doc=copy.deepcopy(blank);doc['parts']=[{'id':'pipe','catalog':'tubeclamp.tube-C',
        'parameters':{'length_mm':1000},'pose':{'position_mm':[0,0,500]}}]
    before=factory(doc);first,last,_=_frame(before,'pipe')
    result=resize_drag(before,'pipe',1100,side)
    after=factory(result['document']);a,b,_=_frame(after,'pipe')
    assert np.allclose(b if side=='start' else a,last if side=='start' else first)
    assert np.isclose(np.linalg.norm(b-a),1100)


@pytest.mark.parametrize('side,expected',[('start',760),('end',560)])
def test_finished_through_fitting_follows_either_handle(factory,blank,side,expected):
    doc=sample(blank)['parts']
    model=copy.deepcopy(blank)
    model['parts']=doc+[{'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},
        'pose':{'position_mm':[500,0,0],'rotation_deg':[0,90,0]}}]
    model['joints']=[{'id':'fit','type':'socket','a':{'part':'fitting','port':'bore'},
        'b':{'part':'pipe','at_mm':700},'locked':True,'insertion_mm':0}]
    result=resize_drag(factory(model),'pipe',800,side)
    after=factory(result['document'])
    assert np.isclose(after.parts['fitting'].matrix[0,3],expected)
    assert validate(after,collisions=False)['valid']


def test_shift_shrink_finished_pipe_detaches_fitting_left_outside(factory,blank):
    model=copy.deepcopy(blank)
    model['parts']=sample(blank)['parts']+[{'id':'pipe','catalog':'tubeclamp.tube-C',
        'parameters':{'length_mm':1000},'pose':{'position_mm':[500,0,0],'rotation_deg':[0,90,0]}}]
    model['joints']=[{'id':'fit','type':'socket','a':{'part':'fitting','port':'bore'},
        'b':{'part':'pipe','at_mm':700},'locked':True,'insertion_mm':0}]
    result=resize_drag(factory(model),'pipe',500,'end',behavior='detach')
    assert result['document']['joints']==[]
    assert result['document']['parts'][0]['pose']==model['parts'][0]['pose']


@pytest.mark.parametrize('side,station',[('start',300),('end',500)])
def test_shift_shrink_finished_pipe_keeps_socket_still_on_it(factory,blank,side,station):
    model=copy.deepcopy(blank)
    model['parts']=sample(blank)['parts']+[{'id':'pipe','catalog':'tubeclamp.tube-C',
        'parameters':{'length_mm':1000},'pose':{'position_mm':[500,0,0],'rotation_deg':[0,90,0]}}]
    model['parts'][0]['pose']['position_mm']=[500,0,0]
    model['joints']=[{'id':'fit','type':'socket','a':{'part':'fitting','port':'bore'},
        'b':{'part':'pipe','at_mm':500},'locked':True,'insertion_mm':0}]
    result=resize_drag(factory(model),'pipe',800,side,behavior='detach')
    assert len(result['document']['joints'])==1
    assert np.isclose(result['document']['joints'][0]['b']['at_mm'],station)
    assert result['document']['parts'][0]['pose']==model['parts'][0]['pose']


def test_extension_never_attaches_one_fitting_to_two_sockets(factory,blank):
    doc=sample(blank,attached=False,extra=True)
    duplicate=copy.deepcopy(doc['parts'][1]['body']['ports']['bore'])
    doc['parts'][1]['body']['ports']['other-bore']=duplicate
    result=resize_drag(factory(doc),'pipe',1200,'end')
    assert len(result['document']['draft_subassemblies'][0]['runs'][0]['attachments'])==1


@pytest.mark.parametrize('side,position',[('start',-100),('end',1100)])
def test_finished_pipe_extension_captures_free_through_socket(factory,blank,side,position):
    model=copy.deepcopy(blank)
    model['parts']=sample(blank)['parts']+[{'id':'pipe','catalog':'tubeclamp.tube-C',
        'parameters':{'length_mm':1000},'pose':{'position_mm':[500,0,0],'rotation_deg':[0,90,0]}}]
    model['parts'][0]['pose']['position_mm']=[position,30,0]
    result=resize_drag(factory(model),'pipe',1200,side)
    assert len(result['document']['joints'])==1
    assert result['document']['joints'][0]['a']['part']=='fitting'
    assert np.isclose(result['document']['parts'][0]['pose']['position_mm'][1],0)


def test_extension_captures_open_port_on_connector_already_serving_another_pipe(factory,blank):
    doc=sample(blank,attached=False)
    doc['parts'][0]['pose']['position_mm']=[1100,0,0]
    doc['parts'][0]['body']['ports']['cross']={
        'type':'socket','profile':'round','diameter_mm':42.4,'axis':[0,1,0],
        'through':True,'engagement_mm':40,'min_engagement_mm':15}
    doc['draft_subassemblies'][0]['runs'].append({'id':'cross','catalog':'tubeclamp.tube-C',
        'start_mm':[1100,-250,0],'end_mm':[1100,250,0],
        'attachments':[{'connector':'fitting','port':'cross'}]})
    result=resize_drag(factory(doc),'pipe',1200,'end')
    main=result['document']['draft_subassemblies'][0]['runs'][0]
    assert main['attachments']==[{'connector':'fitting','port':'bore'}]
    assert result['document']['parts'][0]['pose']==doc['parts'][0]['pose']


def test_angled_rigid_branch_uses_translation_when_stretch_would_break_joint(factory,blank):
    direction=np.array([2**-.5,0,2**-.5]);matrix=np.eye(4)
    matrix[:3,:3]=align_axis([0,0,1],direction)
    matrix[:3,3]=np.array([0,0,700])+direction*130
    model=copy.deepcopy(blank)
    model['parts']=[{'id':'spine','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},
        'pose':{'position_mm':[0,0,500]}},
        {'id':'branch','catalog':'tubeclamp.tube-C','parameters':{'length_mm':300},'pose':pose_of(matrix)},
        {'id':'tee','body':{'kind':'connector','mass_kg':.2,'geometry':[{'type':'sphere','radius_mm':12}],
            'ports':{'through':{'type':'socket','profile':'round','diameter_mm':42.4,
                                 'axis':[0,0,1],'through':True,'engagement_mm':40},
                     'branch':{'type':'socket','profile':'round','diameter_mm':42.4,
                               'axis':direction.tolist(),'engagement_mm':40,'min_engagement_mm':15}}},
         'pose':{'position_mm':[0,0,700]}}]
    model['joints']=[{'id':'main','type':'socket','a':{'part':'tee','port':'through'},
                      'b':{'part':'spine','at_mm':700},'locked':True,'insertion_mm':0},
                     {'id':'side','type':'socket','a':{'part':'tee','port':'branch'},
                      'b':{'part':'branch','end':'start'},'locked':True,'insertion_mm':20}]
    before=factory(model)
    result=resize_drag(before,'spine',800,'end')
    after=factory(result['document'])
    assert np.isclose(after.parts['spine'].length,800)
    assert np.isclose(after.parts['branch'].length,300)
    assert np.isclose(after.parts['tee'].matrix[2,3]-after.parts['branch'].matrix[2,3],
                      before.parts['tee'].matrix[2,3]-before.parts['branch'].matrix[2,3])
    assert validate(after,collisions=False)['valid']


def test_extension_attaches_newly_spanned_free_through_socket(factory,blank):
    before=factory(sample(blank,attached=False,extra=True))
    result=resize_drag(before,'pipe',1200,'end',capture_mm=40)
    run=result['document']['draft_subassemblies'][0]['runs'][0]
    assert run['attachments']==[{'connector':'new-fitting','port':'bore'}]


@pytest.mark.parametrize('side',['start','end'])
def test_following_through_fittings_bunch_without_overlapping(factory,blank,side):
    doc=sample(blank)
    second=copy.deepcopy(doc['parts'][0]);second['id']='second';second['pose']['position_mm']=[800,0,0]
    doc['parts'].append(second)
    doc['draft_subassemblies'][0]['runs'][0]['attachments'].append({'connector':'second','port':'bore'})
    result=resize_drag(factory(doc),'pipe',100,side)
    assert np.isclose(result['length_mm'],260)
    after=factory(result['document'])
    assert np.isclose(abs(after.parts['second'].matrix[0,3]-after.parts['fitting'].matrix[0,3]),26)


@pytest.mark.parametrize('side',['start','end'])
def test_shrink_keeps_through_engagement_at_both_cut_ends(factory,blank,side):
    """Reproduce randomized case seed 2601291923157989291, case 19."""
    doc=sample(blank)
    stations=[102.011965,272.681866]
    if side=='end':stations=[1000-position for position in stations]
    doc['parts'][0]['pose']['position_mm']=[stations[0],0,0]
    second=copy.deepcopy(doc['parts'][0]);second['id']='second'
    second['pose']['position_mm']=[stations[1],0,0]
    doc['parts'].append(second)
    doc['draft_subassemblies'][0]['runs'][0]['attachments'].append(
        {'connector':'second','port':'bore'})

    result=resize_drag(factory(doc),'pipe',165.328999,side)
    after=factory(result['document'])
    run=after.doc['draft_subassemblies'][0]['runs'][0]
    assert result['length_mm']>=1000*20/102.011965-1e-5
    assert {attachment['connector'] for attachment in run['attachments']}=={'fitting','second'}
    assert all(not item['conflicts'] for item in preview(after))


@pytest.mark.parametrize('side,position',[('start',10),('end',90)])
def test_already_orange_draft_can_shrink_either_free_end(factory,blank,side,position):
    doc=sample(blank)
    doc['parts'][0]['pose']['position_mm']=[position,0,0]
    doc['draft_subassemblies'][0]['runs'][0]['end_mm']=[100,0,0]
    before=factory(doc)
    assert preview(before)[0]['conflicts'][0]['engagement_short_mm']==10

    result=resize_drag(before,'pipe',90,side,auto_connect=False)
    after=factory(result['document'])
    start,end,_=_frame(after,'pipe')
    assert result['status']=='resized'
    assert result['length_mm']==90
    assert np.linalg.norm(end-start)==pytest.approx(90)
    assert preview(after)[0]['conflicts'][0]['engagement_short_mm']==20
    assert after.doc['parts']==doc['parts']
    assert after.doc['draft_subassemblies'][0]['runs'][0]['attachments']==[
        {'connector':'fitting','port':'bore'}]


@pytest.mark.parametrize('side',['start','end'])
def test_draft_resize_preserves_a_rigid_brace_between_through_fittings(factory,blank,side):
    doc=sample(blank)
    doc['parts'][0]['pose']['position_mm']=[700,0,500]
    second=copy.deepcopy(doc['parts'][0])
    second['id']='second'
    second['pose']['position_mm']=[300,0,500]
    doc['parts'].append(second)
    doc['parts'].append({'id':'brace','pose':{'position_mm':[500,0,500]},
                         'body':{'kind':'rigid','mass_kg':1,
                                 'geometry':[{'type':'box','size_mm':[400,10,10]}]}})
    doc['joints']=[
        {'id':'brace-left','type':'fixed',
         'a':{'part':'second','frame':{'position_mm':[0,0,0]}},
         'b':{'part':'brace','frame':{'position_mm':[-200,0,0]}}},
        {'id':'brace-right','type':'fixed',
         'a':{'part':'fitting','frame':{'position_mm':[0,0,0]}},
         'b':{'part':'brace','frame':{'position_mm':[200,0,0]}}},
    ]
    run=doc['draft_subassemblies'][0]['runs'][0]
    run['start_mm']=[0,0,500]
    run['end_mm']=[1000,0,500]
    run['attachments'].append({'connector':'second','port':'bore'})
    before=factory(doc)
    assert all(np.linalg.norm(before.joint_frames(j)[0]-before.joint_frames(j)[1])<1e-6
               for j in before.joints)

    result=resize_drag(before,'pipe',800,side,auto_connect=False)
    after=factory(result['document'])
    assert result['length_mm']==800
    assert [after.parts[pid].matrix[0,3] for pid in ('second','brace','fitting')]==[
        300,500,700]
    assert all(np.linalg.norm(after.joint_frames(j)[0]-after.joint_frames(j)[1])<1e-6
               for j in after.joints)
    exact=copy.deepcopy(result['document'])
    exact.pop('draft_subassemblies')
    assert validate(factory(exact),collisions=False)['valid']


@pytest.mark.parametrize('side,position',[('start',-100),('end',1100)])
def test_extension_captures_a_nearby_through_socket_from_either_end(factory,blank,side,position):
    doc=sample(blank,attached=False,extra=True)
    doc['parts'][1]['pose']['position_mm']=[position,30,0]
    result=resize_drag(factory(doc),'pipe',1200,side,capture_mm=40)
    run=result['document']['draft_subassemblies'][0]['runs'][0]
    assert run['attachments']==[{'connector':'new-fitting','port':'bore'}]
    assert np.isclose(next(p for p in result['document']['parts'] if p['id']=='new-fitting')['pose']['position_mm'][1],0)


@pytest.mark.parametrize('side',['start','end'])
@pytest.mark.parametrize('catalog',['generic.chain-link','generic.rope-jute-6','generic.strap-seatbelt-65'])
def test_flexible_line_resizes_from_either_end(factory,blank,side,catalog):
    doc=copy.deepcopy(blank);doc['objects']=[{'id':'line','template':'chain',
        'parameters':{'length_mm':200,'link_catalog':catalog},
        'pose':{'position_mm':[0,0,300]}}]
    before=factory(doc);info=before.doc['objects'][0]
    from pipesim.chain import summary
    ends=summary(info,before.library)
    fixed_id=ends['end_part'] if side=='start' else ends['start_part']
    fixed_port=ends['end_port'] if side=='start' else ends['start_port']
    fixed,_=before.parts[fixed_id].frame({'port':fixed_port})
    result=resize_drag(before,'line',300,side)
    after=factory(result['document']);ends=summary(after.doc['objects'][0],after.library)
    fixed_id=ends['end_part'] if side=='start' else ends['start_part']
    fixed_port=ends['end_port'] if side=='start' else ends['start_port']
    current,_=after.parts[fixed_id].frame({'port':fixed_port})
    assert np.allclose(current,fixed)


@pytest.mark.parametrize('side',['start','end'])
@pytest.mark.parametrize('catalog',['generic.chain-link','generic.rope-jute-6','generic.strap-seatbelt-65'])
def test_flexible_line_resizes_while_other_pipes_are_draft(factory,blank,side,catalog):
    doc=sample(blank)
    doc['objects']=[{'id':'line','template':'chain',
        'parameters':{'length_mm':160,'link_catalog':catalog},
        'pose':{'position_mm':[2000,0,500]}}]
    before=factory(doc)
    result=resize_drag(before,'line',320,side)
    after=factory(result['document'])
    assert result['status']=='resized'
    assert after.doc['draft_subassemblies']==before.doc['draft_subassemblies']
    assert all(not item['conflicts'] for item in preview(after))
    assert before.doc==doc


@pytest.mark.parametrize('side',['start','end'])
def test_finished_pipe_resizes_while_other_pipes_are_draft(factory,blank,side):
    doc=sample(blank)
    doc['parts'].append({'id':'finished','catalog':'tubeclamp.tube-C',
        'parameters':{'length_mm':500},'pose':{'position_mm':[2000,0,500]}})
    before=factory(doc)
    result=resize_drag(before,'finished',600,side)
    after=factory(result['document'])
    assert np.isclose(after.parts['finished'].length,600)
    assert after.doc['draft_subassemblies']==before.doc['draft_subassemblies']
    assert all(not item['conflicts'] for item in preview(after))


def test_anchored_flexible_line_keeps_the_fixed_end_and_rejects_moving_it(factory,blank):
    doc=copy.deepcopy(blank);doc['objects']=[{'id':'line','template':'chain',
        'parameters':{'length_mm':200,'link_catalog':'generic.chain-link'},
        'pose':{'position_mm':[0,0,300]}}]
    doc['anchors']=[{'part':'line/link-1','surface':'fixture'}]
    before=factory(doc)
    result=resize_drag(before,'line',300,'end')
    assert np.allclose(factory(result['document']).parts['line/link-1'].matrix,before.parts['line/link-1'].matrix)
    with pytest.raises(DocumentError,match='anchored'):
        resize_drag(before,'line',300,'start')


def test_random_reflected_endpoint_edits_have_the_same_geometry(factory,blank):
    seed=int(os.environ.get('PIPESIM_RESIZE_SEED') or secrets.randbits(64));rng=random.Random(seed)
    for case in range(int(os.environ.get('PIPESIM_RESIZE_CASES','40'))):
        first_position=rng.uniform(100,500)
        second_position=first_position+rng.uniform(50,300)
        requested=rng.uniform(150,1350)
        angle=rng.uniform(-180,180)
        radians=np.deg2rad(angle);axis=np.array([np.cos(radians),np.sin(radians),0.])
        origin=np.array([rng.uniform(-800,800),rng.uniform(-800,800),rng.uniform(0,800)])
        world=lambda station:(origin+axis*station).tolist()
        left=sample(blank)
        left['parts'][0]['pose']['position_mm']=[first_position,0,0]
        second=copy.deepcopy(left['parts'][0]);second['id']='second'
        second['pose']['position_mm']=[second_position,0,0]
        left['parts'].append(second)
        left['draft_subassemblies'][0]['runs'][0]['attachments'].append({'connector':'second','port':'bore'})
        right=copy.deepcopy(left)
        for spec in right['parts']:
            spec['pose']['position_mm'][0]=1000-spec['pose']['position_mm'][0]
        for doc in (left,right):
            for spec in doc['parts']:
                spec['pose']['position_mm']=world(spec['pose']['position_mm'][0])
                spec['pose']['rotation_deg']=[0,0,angle]
            run=doc['draft_subassemblies'][0]['runs'][0]
            run['start_mm']=world(0);run['end_mm']=world(1000)
        try:
            a=resize_drag(factory(left),'pipe',requested,'start')
            b=resize_drag(factory(right),'pipe',requested,'end')
            assert np.isclose(a['length_mm'],b['length_mm'])
            for pid in ('fitting','second'):
                x=np.dot(np.array(next(p for p in a['document']['parts'] if p['id']==pid)['pose']['position_mm'])-origin,axis)
                reflected=np.dot(np.array(next(p for p in b['document']['parts'] if p['id']==pid)['pose']['position_mm'])-origin,axis)
                assert np.isclose(x+reflected,1000)
            assert len(a['document']['draft_subassemblies'][0]['runs'][0]['attachments'])==2
            assert len(b['document']['draft_subassemblies'][0]['runs'][0]['attachments'])==2
        except Exception as error:
            pytest.fail(f'resize reflection seed={seed} case={case}: {error}')


@pytest.mark.parametrize('physical_end',[0,1000])
def test_reversing_saved_pipe_direction_does_not_change_handle_behavior(factory,blank,physical_end):
    forward=sample(blank)
    reversed_doc=copy.deepcopy(forward)
    run=reversed_doc['draft_subassemblies'][0]['runs'][0]
    run['start_mm'],run['end_mm']=run['end_mm'],run['start_mm']
    side='start' if physical_end==0 else 'end'
    opposite='end' if side=='start' else 'start'
    a=resize_drag(factory(forward),'pipe',800,side)
    b=resize_drag(factory(reversed_doc),'pipe',800,opposite)
    left=a['document']['draft_subassemblies'][0]['runs'][0]
    right=b['document']['draft_subassemblies'][0]['runs'][0]
    assert np.allclose(left['start_mm'],right['end_mm'])
    assert np.allclose(left['end_mm'],right['start_mm'])
    assert np.isclose(a['document']['parts'][0]['pose']['position_mm'][0],
                      b['document']['parts'][0]['pose']['position_mm'][0])


@pytest.mark.parametrize('side',['start','end'])
@pytest.mark.parametrize('attached',[False,True])
def test_centered_mirror_stays_centered_when_either_handle_moves(factory,blank,side,attached):
    doc=sample(blank,attached=attached)
    if not attached:doc['parts']=[]
    doc['draft_subassemblies'][0]['mirrors']=[{'id':'midplane','axis':'x','offset_mm':500,
                                               'run_modes':{'pipe':'centered'}}]
    result=resize_drag(factory(doc),'pipe',1200,side)
    run=result['document']['draft_subassemblies'][0]['runs'][0]
    assert np.allclose(run['start_mm'],[-100,0,0])
    assert np.allclose(run['end_mm'],[1100,0,0])
    if attached:
        # An extension into free space does not disturb an already seated socket.
        assert result['document']['parts'][0]['pose']==doc['parts'][0]['pose']
