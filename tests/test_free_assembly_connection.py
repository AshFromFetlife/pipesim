"""A rigid layout operation must not search the attached human/chain joints."""
import copy
import json
from pathlib import Path
import time

import numpy as np
import pytest

from pipesim.document import Assembly
from pipesim.math3d import pose_of, transform
from pipesim.posing import transform_part
from pipesim.snapping import connection_options

FIXTURE=Path(__file__).parent/'fixtures'/'four-way-swing'


def swing(library):
    return Assembly.from_doc(json.loads((FIXTURE/'design.json').read_text()),FIXTURE,library)


def component(assembly, selected):
    # Independent graph traversal; includes every connection, regardless of its DOFs.
    members={selected}
    while True:
        before=set(members)
        for joint in assembly.joints:
            edge={joint['a']['part'],joint['b']['part']}
            if members&edge: members|=edge
        if before==members:return members


def no_search(*args,**kwargs):
    pytest.fail('A free rigid assembly must not enter inverse kinematics or Force search')


@pytest.mark.parametrize('force',[False,True])
@pytest.mark.parametrize('tilted',[False,True])
def test_swing_connection_carries_every_attachment_without_solving_joints(library,monkeypatch,force,tilted):
    before=swing(library)
    if tilted:
        doc=copy.deepcopy(before.doc);members=component(before,'tube-c-4')
        delta=transform({'position_mm':[120,-80,150], 'rotation_deg':[14,-9,0]})
        for part in doc['parts']:
            if part['id'] in members:part['pose']=pose_of(delta@before.parts[part['id']].matrix)
        before=Assembly.from_doc(doc,FIXTURE,library)
    original=copy.deepcopy(before.doc)
    monkeypatch.setattr('pipesim.fitting.fit_connection',no_search)
    monkeypatch.setattr('pipesim.posing.least_squares',no_search)
    from pipesim.validation import CollisionWorld
    inspected=[]
    def local_world(assembly):
        inspected.append(set(assembly.parts))
        assert len(assembly.parts)<len(before.parts)//2, 'remote anatomy/chain links entered narrow-phase collision checking'
        return CollisionWorld(assembly)
    monkeypatch.setattr('pipesim.validation.CollisionWorld',local_world)
    result=connection_options(before,'tube-c-4','tc104c-1','branch',end='end',force=force)
    assert inspected
    assert result['recommended'] and len(result['options'])==2
    for option in result['options']:
        assert option['available'],option
        assert not option.get('requires_preview')
        selected='tube-c-4' if option['move']=='member' else 'tc104c-1'
        moved=component(before,selected)
        after=Assembly.from_doc(option['document'],FIXTURE,library)
        delta=after.parts[selected].matrix@np.linalg.inv(before.parts[selected].matrix)
        assert set(option['moved'])==moved
        for pid,part in before.parts.items():
            expected=delta@part.matrix if pid in moved else part.matrix
            np.testing.assert_allclose(after.parts[pid].matrix,expected,atol=.002,rtol=0,err_msg=pid)
        assert after.doc['joints'][:-1]==original['joints']
        for old,new in zip(original['objects'],after.doc['objects']):
            assert {k:v for k,v in old.items() if k!='pose'}=={k:v for k,v in new.items() if k!='pose'}
        a,b,_=after.joint_frames(after.joints[-1]);assert np.linalg.norm(a-b)<.001
    assert before.doc==original


@pytest.mark.parametrize('selected',['tc104c-1','tube-c-4'])
def test_swing_drag_carries_the_whole_component_and_commits_compact_objects(library,monkeypatch,selected):
    before=swing(library)
    monkeypatch.setattr('pipesim.posing.least_squares',no_search)
    target=pose_of(before.parts[selected].matrix);target['position_mm'][2]+=100
    started=time.monotonic();result=transform_part(before,selected,target,preview=True)
    assert time.monotonic()-started<1.5, 'rigid preview must remain interactive'
    assert result['position_error_mm']<.001
    assert set(result['moved'])==component(before,selected)
    result=transform_part(before,selected,target)
    after=Assembly.from_doc(result['document'],FIXTURE,library)
    assert after.doc['joints']==before.doc['joints']
    for pid in before.parts:
        expected=before.parts[pid].matrix.copy()
        if pid in result['moved']:expected[2,3]+=100
        np.testing.assert_allclose(after.parts[pid].matrix,expected,atol=.002,rtol=0,err_msg=pid)


def test_partial_drag_preview_is_carried_through_the_final_connection(library,monkeypatch):
    before=swing(library)
    monkeypatch.setattr('pipesim.fitting.fit_connection',no_search)
    group=next(g for g in before.editor_groups() if 'tube-c-4' in g)
    poses={pid:pose_of(before.parts[pid].matrix) for pid in group}
    for pose in poses.values():pose['position_mm'][2]-=75
    result=connection_options(before,'tube-c-4','tc104c-1','branch',end='end',poses=poses)
    assert result['recommended']=='member'
    assert all(o['available'] for o in result['options']),result


def test_rounded_browser_poses_still_identify_one_rigid_component(library,monkeypatch):
    before=swing(library);doc=copy.deepcopy(before.doc)
    world=transform({'position_mm':[327.3,-219.7,5000], 'rotation_deg':[7.891,-8.123,123.456]})
    for part in [*doc['parts'],*doc['objects']]:
        part['pose']=pose_of(world@transform(part.get('pose')))
    before=Assembly.from_doc(doc,FIXTURE,library)
    group=next(g for g in before.editor_groups() if 'tc104c-1' in g)
    poses={pid:pose_of(before.parts[pid].matrix) for pid in group}
    for pose in poses.values():
        pose['position_mm'][2]+=75
        for key,values in pose.items():pose[key]=[round(value,5) for value in values]
    monkeypatch.setattr('pipesim.fitting.fit_connection',no_search)
    result=connection_options(before,'tube-c-4','tc104c-1','branch',end='end',poses=poses)
    assert all(o['available'] for o in result['options']),result


def test_clear_drop_checks_only_the_requested_rigid_alternative(library,monkeypatch):
    import pipesim.snapping as snapping
    before=swing(library);checked=[];check=snapping.connection_collisions
    def inspect(*args,**kwargs):
        checked.append(args)
        return check(*args,**kwargs)
    monkeypatch.setattr(snapping,'connection_collisions',inspect)
    result=connection_options(before,'tube-c-4','tc104c-1','branch',end='end',
                              poses={'tube-c-4':pose_of(before.parts['tube-c-4'].matrix)},first_valid=True)
    assert result['recommended']=='member'
    assert len(result['options'])==len(checked)==1
    assert result['options'][0]['available']


def test_world_anchor_is_a_real_boundary_for_rigid_connection(library):
    before=swing(library);doc=copy.deepcopy(before.doc)
    doc['anchors']=[{'part':'tc104c-1','surface':'fixture'}]
    before=Assembly.from_doc(doc,FIXTURE,library)
    result=connection_options(before,'tube-c-4','tc104c-1','branch',end='end')
    assert result['recommended']=='member'
    assert not next(o for o in result['options'] if o['move']=='connector')['available']


def test_collision_broad_phase_keeps_a_real_new_obstruction(factory,blank):
    from pipesim.snapping import connection_collisions
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':name,'catalog':'generic.box','parameters':{
        'width_mm':100,'depth_mm':100,'height_mm':100,'mass_kg':1},
        'pose':{'position_mm':[x,0,1000]}} for name,x in [('moving',300),('fixed',0)]]
    before=factory(doc);doc['parts'][0]['pose']['position_mm'][0]=20
    collisions=connection_collisions(before,factory(doc),{'moving'},set())
    assert len(collisions)==1 and set(collisions[0][:2])=={'moving','fixed'}
