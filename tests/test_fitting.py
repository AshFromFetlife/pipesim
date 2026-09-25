import copy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pathlib import Path

from pipesim.document import Assembly, DocumentError, read
from pipesim.math3d import pose_of, transform
from pipesim.posing import transform_part
from pipesim.snapping import connection_options, _movement_coordinates
from pipesim.validation import validate


def fit(assembly, connector='upper-back', station=800, **kwargs):
    return connection_options(assembly, 'top-bar', connector, 'through', at_mm=station, locked=False, **kwargs)


def chosen(result):
    option=next((o for o in result['options'] if o['move']==result['recommended']),None)
    assert option is not None,[(o['move'],o.get('reason')) for o in result['options']]
    assert option['available']
    return option


def verify(before, option, factory):
    after=factory(option['document'])
    report=validate(after)
    assert report['valid'],report['issues']
    assert not [i for i in report['issues'] if i['code']=='INTERSECTION'],report['issues']
    for pid, part in before.parts.items():
        assert after.parts[pid].length==part.length
    for pid in ('bottom','right','top','left',*[f'corner-{i}' for i in range(4)]):
        assert np.allclose(after.parts[pid].matrix,before.parts[pid].matrix,atol=1e-7,rtol=0),pid
    for original in before.joints:
        copied=next(j for j in after.joints if j['id']==original['id'])
        assert copied.get('locked')==original.get('locked'),original['id']
    for j in after.joints:
        a,b,axis=after.joint_frames(j)
        delta=b-a
        if j['type']=='socket' and not j.get('locked'): delta-=axis*(delta@axis)
        assert np.linalg.norm(delta)<.03,j['id']
    _movement_coordinates(before,after)
    return after


@pytest.mark.parametrize('anchored',[False,True])
def test_three_tee_ended_pipes_close_a_triangle_around_the_last_pipe(load,factory,anchored):
    before=load('hinged-triangle')
    if not anchored:
        before.doc['anchors']=[];before=factory(before.doc)
    original=copy.deepcopy(before.doc)
    assert len(before.parts)==18
    assert len([p for p in before.parts.values() if p.spec.get('catalog') in ('tubeclamp.TC101C','tubeclamp.TC148C')])==6
    first=chosen(fit(before));middle=verify(before,first,factory)
    assert first['move']=='fit' and first['requires_preview']
    second=chosen(fit(middle,'upper-left',500));after=verify(middle,second,factory)
    assert second['move']=='fit'
    motions={m['joint']:m for m in second['motions']}
    assert {'hinge-front','hinge-back','hinge-left','thread-front','upper-back-through-top-bar'}<=set(motions)
    assert abs(motions['hinge-left']['angle_deg']%15)>1
    assert abs(motions['hinge-front']['slide_mm'])>1
    assert len(after.joints)==len(before.joints)+2
    assert before.doc==original


def test_rotated_ceiling_frame_and_opposite_socket_axis_have_the_same_fit(load,factory):
    before=load('hinged-triangle');delta=transform({'position_mm':[200,-500,3000],'rotation_deg':[125,-23,47]})
    for p in before.doc['parts']: p['pose']=pose_of(delta@before.parts[p['id']].matrix)
    before.doc['anchors'][0]['surface']='ceiling';before=factory(before.doc)
    middle=verify(before,chosen(fit(before)),factory)
    verify(middle,chosen(fit(middle,'upper-left',500)),factory)


def test_force_adjusts_the_requested_station_when_slide_limits_require_a_different_fit(load,factory):
    before=load('hinged-triangle');middle=factory(chosen(fit(before))['document'])
    for j in middle.doc['joints']:
        if j['type']=='socket' and not j.get('locked'):
            j['limits']={'slide_mm':[-.00001,.00001]}
    middle=factory(middle.doc);original=copy.deepcopy(middle.doc)
    ordinary=fit(middle,'upper-left',50)
    assert ordinary['recommended'] is None
    forced=chosen(fit(middle,'upper-left',50,force=True))
    after=verify(middle,forced,factory)
    assert forced['joint']['b']['at_mm']>400
    assert 'Fitted socket centre' in forced['description']
    assert len(after.joints)==len(middle.joints)+1
    assert middle.doc==original


def test_force_keeps_locked_connections_and_reports_search_failure_without_changing_document(load,factory):
    before=load('hinged-triangle')
    for j in before.doc['joints']: j['locked']=True
    before=factory(before.doc);original=copy.deepcopy(before.doc)
    result=fit(before,force=True)
    assert result['recommended'] is None
    assert all(not o['available'] for o in result['options'])
    assert before.doc==original


def test_force_preserves_limits_and_does_not_pull_a_tee_off_its_pipe(load,factory):
    before=load('hinged-triangle')
    for j in before.doc['joints']:
        if j['id'].startswith('hinge-'): j['limits']={'angle_deg':[-.1,.1],'slide_mm':[-20,20]}
    before=factory(before.doc);original=copy.deepcopy(before.doc)
    result=fit(before,force=True)
    assert result['recommended'] is None
    assert 'Travel limits' in result['options'][0]['reason']
    assert before.doc==original


def test_fitting_rebases_motors_limits_tracks_and_drive_offsets(load,factory):
    before=load('hinged-triangle')
    j=next(j for j in before.doc['joints'] if j['id']=='hinge-back')
    j['limits']={'angle_deg':[-50,50],'slide_mm':[-200,200]}
    j['motor']={'mode':'position','target':10,'max_force_n':100,'schedule':[{'time_s':1,'target':20}]}
    before.doc['animation']={'tracks':[{'joint':'hinge-back','coordinate':'angle_deg','keyframes':[{'time_s':0,'value':0},{'time_s':1,'value':10}]}]}
    before.doc['drives']=[{'id':'coupling','type':'gt2','driver':'hinge-back','follower':'hinge-front'}]
    before=factory(before.doc);option=chosen(fit(before,station=500));after=verify(before,option,factory)
    angles={m['joint']:m.get('angle_deg',0) for m in option['motions']};shift=angles['hinge-back']
    slides={m['joint']:m.get('slide_mm',0) for m in option['motions']}
    j=next(j for j in after.joints if j['id']=='hinge-back')
    assert j['limits']['angle_deg']==pytest.approx([-50-shift,50-shift])
    assert abs(slides['hinge-back'])>1
    assert j['motor']['target']==pytest.approx(10-slides['hinge-back'])
    assert j['motor']['schedule'][0]['target']==pytest.approx(20-slides['hinge-back'])
    assert [k['value'] for k in after.doc['animation']['tracks'][0]['keyframes']]==pytest.approx([-shift,10-shift])
    assert after.doc['drives'][0]['offset_mm']==pytest.approx(40/360*angles['hinge-back']-slides['hinge-front'])


def test_drag_intent_with_depth_and_15_degree_errors_is_projected_onto_the_mechanism(load,factory):
    before=load('hinged-triangle');poses={}
    ids=next(g for g in before.rigid_groups() if 'upper-back' in g)
    intent=transform({'position_mm':[0,25,0],'rotation_deg':[0,15,0]})
    pivot=before.parts['upper-back'].matrix[:3,3]
    intent[:3,3]+=pivot-intent[:3,:3]@pivot
    for pid in ids: poses[pid]=pose_of(intent@before.parts[pid].matrix)
    option=chosen(fit(before,poses=poses));verify(before,option,factory)


def test_threading_an_already_posed_frame_keeps_unrelated_grouped_humans(load,factory):
    before=load('hinged-triangle')
    before.doc['objects']=[{'id':'person','template':'human','pose':{'position_mm':[4000,0,0]}}]
    before.doc['state']={'joints':{'hinge-back':{'angle_deg':5}}}
    before=factory(before.doc);original=copy.deepcopy(before.doc)
    option=chosen(fit(before));after=factory(option['document'])
    assert len(after.doc['objects'])==1 and after.doc['objects'][0]['id']=='person'
    for pid in before.parts:
        if pid.startswith('person/'):
            assert np.allclose(before.parts[pid].matrix,after.parts[pid].matrix,atol=1e-6,rtol=0)
    assert validate(after,collisions=False)['valid']
    assert before.doc==original


def test_force_can_fit_a_terminal_socket_without_changing_cut_lengths(factory):
    before=factory(read(Path(__file__).parent/'fixtures/sliding-corner.pipe.yaml'))
    result=connection_options(before,'bottom-arm','elbow','x',end='end',insertion_mm=30,force=True)
    option=chosen(result);after=verify(before,option,factory)
    port=after.parts['elbow'].ports['x'];joint=after.joints[-1]
    assert port['min_engagement_mm']<=joint['insertion_mm']<=port['engagement_mm']
    assert joint['locked']


@pytest.mark.parametrize('failure',['occupied','diameter'])
def test_force_still_checks_socket_occupancy_and_pipe_size(load,factory,failure):
    before=load('hinged-triangle')
    if failure=='diameter':
        next(p for p in before.doc['parts'] if p['id']=='top-bar')['catalog']='tubeclamp.tube-D'
        before=factory(before.doc)
    with pytest.raises(DocumentError,match='occupied' if failure=='occupied' else 'size or profile'):
        fit(before,'upper-front' if failure=='occupied' else 'upper-back',force=True)
