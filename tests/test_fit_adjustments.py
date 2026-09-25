import copy
from pathlib import Path

import numpy as np
import pytest

from pipesim.document import Assembly,DocumentError,read,write
from pipesim.fit_adjustments import settings
from pipesim.math3d import pose_of,point,transform
from pipesim.resizing import resize_member
from pipesim.snapping import connection_options,connection_collisions,move_document
from pipesim.validation import validate


@pytest.fixture
def frame(factory):
    return factory(read(Path(__file__).parent/'fixtures/assembly-adjustment.pipe.yaml'))


def connect(frame,**options):
    return connection_options(frame,'bottom-arm','elbow','x',end='end',insertion_mm=30,**options)


def chosen(result):
    assert result['recommended'],[o.get('reason') for o in result['options']]
    return next(o for o in result['options'] if o['move']==result['recommended'])


def verify(before,option,factory,unlock=0,resize=0,maximum=20):
    after=factory(option['document']);assert validate(after)['valid']
    assert len(after.joints)==len(before.joints)+1
    for anchor in before.anchors:
        assert np.allclose(before.parts[anchor['part']].matrix,after.parts[anchor['part']].matrix,atol=1e-7,rtol=0)
    for original in before.joints:
        j=next(j for j in after.joints if j['id']==original['id'])
        assert j.get('locked')==original.get('locked')
        pa,pb,_=after.joint_frames(j);assert np.linalg.norm(pa-pb)<.03,j['id']
    edits=option.get('adjustments',{'unlocked':[],'resized':[]})
    assert len({a['connector'] for a in edits['unlocked']})<=unlock
    changed=[pid for pid in before.parts if abs(after.parts[pid].length-before.parts[pid].length)>1e-6]
    assert len(changed)<=resize and set(changed)=={r['part'] for r in edits['resized']}
    for pid in changed: assert abs(after.parts[pid].length-before.parts[pid].length)<=maximum+1e-5
    assert not any(pid.startswith('__fit_length') for pid in after.parts)
    assert not any(j['id'].startswith('__fit_length') for j in after.joints)
    return after


def test_one_mm_rejection_can_use_a_small_explicit_assembly_allowance(frame,factory,tmp_path):
    original=copy.deepcopy(frame.doc)
    strict=connect(frame,force=True,tolerance_mm=.03)
    assert strict['recommended'] is None and '1.00 mm' in strict['options'][0]['reason']
    option=chosen(connect(frame));after=verify(frame,option,factory)
    assert option['gap_mm']==pytest.approx(1) and option['requires_preview']
    assert after.joints[-1]['fit_tolerance_mm']==2
    assert 'Accepted connection gap: 1.00 mm' in option['description']
    for pid in frame.parts: assert np.allclose(frame.parts[pid].matrix,after.parts[pid].matrix,atol=1e-6,rtol=0)
    path=tmp_path/'fit.pipe.yaml';write(path,after.doc)
    assert Assembly.load(path).joints[-1]['fit_tolerance_mm']==2
    assert frame.doc==original


def test_force_can_loosen_and_retighten_one_connector_without_changing_lengths(frame,factory):
    original=copy.deepcopy(frame.doc)
    option=chosen(connect(frame,force=True,tolerance_mm=.03,force_options={'unlock_connectors':1}))
    verify(frame,option,factory,unlock=1)
    assert option['gap_mm']<.03 and option['adjustments']['unlocked']
    assert not option['adjustments']['resized']
    assert option['joint']['metadata']['assembly_adjustments']==option['adjustments']
    assert frame.doc==original


@pytest.mark.parametrize('rotated',[False,True])
def test_force_solves_cut_length_with_the_connected_poses_and_previews_real_geometry(frame,factory,rotated):
    if rotated:
        delta=transform({'position_mm':[400,100,2200],'rotation_deg':[125,-23,47]})
        for p in frame.doc['parts']: p['pose']=pose_of(delta@frame.parts[p['id']].matrix)
        frame.doc['anchors'][0]['surface']='ceiling';frame=factory(frame.doc)
    original=copy.deepcopy(frame.doc)
    option=chosen(connect(frame,force=True,tolerance_mm=.03,force_options={'resize_members':1,'max_length_change_mm':2}))
    after=verify(frame,option,factory,resize=1,maximum=2)
    assert after.parts['left-arm'].length==pytest.approx(400,abs=.03)
    assert option['gap_mm']<.03 and not option['adjustments']['unlocked']
    assert len(option['preview_parts'])==1 and option['preview_parts'][0]['length_mm']==pytest.approx(400,abs=.03)
    assert after.parts['left-arm'].shapes!=frame.parts['left-arm'].shapes
    assert frame.doc==original


def test_resize_bound_and_zero_permissions_do_not_silently_modify_a_frame(frame):
    for options in ({},{'resize_members':0,'max_length_change_mm':20},{'resize_members':1,'max_length_change_mm':.5}):
        original=copy.deepcopy(frame.doc)
        result=connect(frame,force=True,tolerance_mm=.03,force_options=options)
        assert result['recommended'] is None
        assert frame.doc==original


def test_force_options_are_not_used_without_pressing_force(frame):
    result=connect(frame,tolerance_mm=.03,force_options={'unlock_connectors':2,'resize_members':2})
    assert result['recommended'] is None


def test_parallel_rigid_paths_can_require_two_simultaneous_length_changes(frame,factory):
    doc=copy.deepcopy(frame.doc)
    brace=copy.deepcopy(next(p for p in doc['parts'] if p['id']=='left-arm'));brace['id']='brace'
    matrix=frame.parts['left-arm'].matrix.copy();matrix[:3,3]+=[0,0,200];brace['pose']=pose_of(matrix)
    doc['parts'].append(brace);temp=factory(doc)
    for side,host in [('start','left-tee'),('end','elbow')]:
        world,_=temp.parts['brace'].frame({'end':side})
        local=point(np.linalg.inv(temp.parts[host].matrix),world).tolist()
        doc['joints'].append({'id':'brace-'+side,'type':'fixed','a':{'part':host,'frame':{'position_mm':local}},'b':{'part':'brace','end':side}})
    before=factory(doc)
    assert connect(before,force=True,tolerance_mm=.03,force_options={'resize_members':1,'max_length_change_mm':2})['recommended'] is None
    option=chosen(connect(before,force=True,tolerance_mm=.03,force_options={'resize_members':2,'max_length_change_mm':2}))
    after=verify(before,option,factory,resize=2,maximum=2)
    assert {r['part'] for r in option['adjustments']['resized']}=={'left-arm','brace'}
    assert after.parts['brace'].length==pytest.approx(400,abs=.03)


def test_locked_travel_limits_remain_limits_while_screws_are_released(frame,factory):
    for j in frame.doc['joints']: j['limits']={'slide_mm':[-.001,.001],'angle_deg':[-.001,.001]}
    before=factory(frame.doc)
    result=connect(before,force=True,tolerance_mm=.03,force_options={'unlock_connectors':1})
    assert result['recommended'] is None


def test_unrelated_grouped_objects_and_animation_survive_adjustment(frame,factory):
    frame.doc['objects']=[{'id':'person','template':'human','pose':{'position_mm':[4000,0,0]}}]
    frame.doc['animation']={'tracks':[{'joint':'person/left_knee','coordinate':'angle_deg','keyframes':[{'time_s':0,'value':0},{'time_s':1,'value':10}]}]}
    before=factory(frame.doc)
    option=chosen(connect(before,force=True,tolerance_mm=.03,force_options={'resize_members':1,'max_length_change_mm':2}))
    after=verify(before,option,factory,resize=1,maximum=2)
    assert after.doc['animation']==before.doc['animation']
    assert len(after.doc['objects'])==1 and after.doc['objects'][0]['id']=='person'
    assert not any(pid.startswith('person/') for pid in option['context_parts'])
    for pid in before.parts:
        if pid.startswith('person/'): assert np.allclose(before.parts[pid].matrix,after.parts[pid].matrix,atol=1e-6,rtol=0)


@pytest.mark.parametrize('tolerance,options',[(0,None),(float('nan'),None),(21,None),(True,None),
    (2,{'unlock_connectors':-1}),(2,{'unlock_connectors':1.5}),(2,{'resize_members':True}),
    (2,{'resize_members':5}),(2,{'max_length_change_mm':float('inf')}),(2,{'max_length_change_mm':0}),(2,{'unknown':1})])
def test_invalid_adjustment_bounds_are_rejected(tolerance,options):
    with pytest.raises(DocumentError): settings(tolerance,options)


def test_allowance_does_not_hide_overlarge_gaps_or_an_unengaged_pipe_end(frame,factory):
    result=resize_member(frame,'left-arm',frame.parts['left-arm'].length+4,suggest=False,collisions=False)
    before=factory(result['document'])
    assert connect(before,force=True,tolerance_mm=2)['recommended'] is None
    # Declared insertion alone is not enough when a permitted gap leaves the
    # real pipe tip short of the screw's required engagement.
    allowed=chosen(connect(frame));doc=allowed['document'];j=doc['joints'][-1]
    j['insertion_mm']=factory(doc).parts['elbow'].ports['x']['min_engagement_mm']
    member=next(p for p in doc['parts'] if p['id']=='bottom-arm');member['pose']['position_mm'][1]-=20
    assert any(i['code']=='ENGAGEMENT_SHORT' for i in validate(factory(doc),collisions=False)['issues'])


def test_loose_offset_socket_keeps_its_allowance_during_rotation_without_accumulating_gap(blank,factory):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},'pose':{'position_mm':[0,0,500]}},
                  {'id':'collar','catalog':'tubeclamp.TC161C'}]
    doc=chosen(connection_options(factory(doc),'pipe','collar','through',at_mm=500,locked=False))['document']
    doc['anchors']=[{'part':'pipe','surface':'ceiling'}]
    before=factory(doc);pose=pose_of(before.parts['collar'].matrix)
    pose['position_mm'][0]=1
    doc=move_document(before,{'collar':pose})
    pose['rotation_deg']=[0,0,90]
    doc=move_document(factory(doc),{'collar':pose})
    assert validate(factory(doc))['valid']
    pose['position_mm'][0]=1.5
    doc=move_document(factory(doc),{'collar':pose})
    assert any(i['code']=='ASSEMBLY_FIT_ALLOWANCE' for i in validate(factory(doc),collisions=False)['issues'])
    # Several small drags still cannot exceed the same absolute allowance.
    pose['position_mm'][0]=2.01
    with pytest.raises(DocumentError,match='freedom'):
        move_document(factory(doc),{'collar':pose})


def test_length_changes_check_collisions_even_with_unchanged_relative_poses(blank,factory):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},'pose':{'position_mm':[0,0,500]}},
                  {'id':'obstacle','body':{'kind':'rigid','mass_kg':1,'geometry':[{'type':'box','size_mm':[60,60,30]}]},
                   'pose':{'position_mm':[0,0,1080]}}]
    before=factory(doc)
    assert not connection_collisions(before,before,{'pipe'},set())
    doc['parts'][0]['parameters']['length_mm']=1200
    collisions=connection_collisions(before,factory(doc),{'pipe'},set())
    assert [(a,b) for a,b,_ in collisions]==[('pipe','obstacle')]
