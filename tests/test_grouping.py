import copy

import numpy as np
import pytest

from pipesim.document import Assembly, DocumentError, parse, write, read
from pipesim.editing import expand_objects, relocate_design
from pipesim.grouping import regroup_candidates, regroup_object, move_object, update_object_parameters, detach_attachment, attach_part
from pipesim.math3d import pose_of, transform
from pipesim.posing import transform_part
from pipesim.snapping import _movement_coordinates


def human(factory,blank):
    doc=copy.deepcopy(blank)
    doc['objects']=[{'id':'person','template':'human','parameters':{'pose':'seated','hold_joints':['arms'],'stature_mm':1800},
                     'pose':{'position_mm':[40,120,60],'rotation_deg':[10,5,35]}},
                    {'id':'other','template':'human','pose':{'position_mm':[3000,0,0]}}]
    return factory(doc)


def equivalent(before,after):
    assert set(before.parts)==set(after.parts)
    for pid,p in before.parts.items():
        assert np.allclose(p.matrix,after.parts[pid].matrix,atol=1e-6,rtol=0),pid
        assert p.definition==after.parts[pid].definition,pid
    assert sorted(before.joints,key=lambda j:j['id'])==sorted(after.joints,key=lambda j:j['id'])
    assert before.anchors==after.anchors


def test_selected_expansion_and_regroup_preserve_edited_physics_and_other_objects(factory,blank):
    before=human(factory,blank);doc=expand_objects(before,'person')
    assert [o['id'] for o in doc['objects']]==['other']
    assert [r['instance']['id'] for r in doc['expanded_objects']]==['person']
    doc['parts'][0]['body']['geometry'][0]['size_mm'][0]+=12
    doc['parts'][0]['mass_kg']=17
    doc['joints'][0]['limits']['rotation_deg'][0]=[-3,11]
    doc['joints'][5]['motor']={'mode':'position','target':2,'max_torque_nm':12,'schedule':[{'time_s':2,'target':4}]}
    doc['anchors']=[{'part':'person/right_foot','surface':'fixture'}]
    edited=factory(doc);regrouped=regroup_object(edited,'person');after=factory(regrouped)
    equivalent(edited,after)
    assert [o['id'] for o in regrouped['objects']]==['person','other']
    assert not regrouped['parts'] and 'expanded_objects' not in regrouped
    assert regrouped['objects'][0]['components']['parts'][0]['id']=='pelvis'
    equivalent(after,factory(expand_objects(after,'person')))


def test_older_expanded_human_is_recovered_without_regenerating_its_pose(load,factory):
    doc=expand_objects(load('human-pull-up'));doc.pop('expanded_objects')
    before=factory(doc);target=pose_of(before.parts['person/left_shin'].matrix);target['position_mm'][1]+=30
    posed=factory(transform_part(before,'person/left_shin',target)['document'])
    assert regroup_candidates(posed)[0]['recovered']
    after=factory(regroup_object(posed,'person'))
    equivalent(posed,after)
    params=after.doc['objects'][0]['parameters']
    assert params['hold_joints']==['upper_body']
    assert params['grip_diameter_mm']==42.4
    assert params['strength_scale']==pytest.approx(6)
    assert after.doc['objects'][0]['template']=='human'


def test_recovery_does_not_claim_an_incomplete_or_unrelated_human(factory,blank):
    doc=expand_objects(human(factory,blank),'person');doc.pop('expanded_objects')
    doc['parts']=[p for p in doc['parts'] if p['id']!='person/head']
    doc['joints']=[j for j in doc['joints'] if j['b']['part']!='person/head']
    assembly=factory(doc)
    assert regroup_candidates(assembly)==[]
    with pytest.raises(DocumentError,match='complete expanded human'): regroup_object(assembly,'person')


def test_regroup_preserves_saved_coordinates_animation_loads_and_grips(load,factory):
    doc=load('human-pull-up').doc
    # Closed grips cannot be represented by joint coordinates, so use a free
    # human here and test the external joints separately in the legacy test.
    doc['joints']=[j for j in doc['joints'] if 'grip' not in j['id']]
    doc['state']={'joints':{'person/left_knee':{'angle_deg':-5}}}
    doc['animation']={'tracks':[{'joint':'person/left_knee','coordinate':'angle_deg','keyframes':[{'time_s':0,'value':-5},{'time_s':1,'value':-15}]}]}
    doc['loads']=[{'part':'person/left_hand','force_n':[1,2,3]}]
    original=factory(doc);expanded=factory(expand_objects(original));after=factory(regroup_object(expanded,'person'))
    equivalent(original,after)
    for key in ('state','animation','loads'): assert after.doc[key]==original.doc[key]


def test_whole_person_translation_and_flip_then_limb_pose_keep_one_object(factory,blank):
    before=human(factory,blank)
    target={'position_mm':[4040,-3120,1000],'rotation_deg':[180,0,-145]}
    result=move_object(before,'person',target);after=factory(result['document'])
    delta=transform(target)@np.linalg.inv(transform(before.doc['objects'][0]['pose']))
    assert set(result['moved'])=={p for p in before.parts if p.startswith('person/')}
    for pid in result['moved']: assert np.allclose(after.parts[pid].matrix,delta@before.parts[pid].matrix,atol=1e-6,rtol=0)
    assert len(after.doc['objects'])==2 and not after.doc['parts']
    _movement_coordinates(before,after)
    target=pose_of(after.parts['person/left_hand'].matrix);target['position_mm'][0]+=20
    posed=factory(transform_part(after,'person/left_hand',target)['document'])
    assert len(posed.doc['objects'])==2 and not posed.doc['parts']
    assert np.linalg.norm(posed.parts['person/left_hand'].matrix[:3,3]-after.parts['person/left_hand'].matrix[:3,3])>5
    _movement_coordinates(after,posed)
    for pid in ('person/pelvis','person/right_hand','person/left_foot'):
        assert np.allclose(posed.parts[pid].matrix,after.parts[pid].matrix,atol=1e-6,rtol=0)


def test_whole_person_with_saved_state_moves_posture_and_animation_together(factory,blank):
    before=human(factory,blank);doc=before.doc
    doc['state']={'joints':{'person/left_knee':{'angle_deg':10}}}
    before=factory(doc);target={'position_mm':[2000,1000,500],'rotation_deg':[0,0,180]}
    result=move_object(before,'person',target);after=factory(result['document'])
    delta=transform(target)@np.linalg.inv(transform(before.doc['objects'][0]['pose']))
    for pid in result['moved']: assert np.allclose(after.parts[pid].matrix,delta@before.parts[pid].matrix,atol=1e-6,rtol=0)
    assert len(after.doc['objects'])==2


def test_grips_block_whole_translation_with_actionable_named_attachment(load):
    before=load('human-pull-up');original=copy.deepcopy(before.doc)
    with pytest.raises(DocumentError,match=r'grip.*Detach.*Connections to structure'):
        move_object(before,'person',{'position_mm':[5000,0,0]})
    assert before.doc==original


def test_component_muscle_controls_keep_pose_and_custom_motor_settings(factory,blank):
    before=human(factory,blank);before=factory(regroup_object(factory(expand_objects(before,'person')),'person'))
    obj=before.doc['objects'][0];custom=next(j for j in obj['components']['joints'] if j['id']=='left_elbow')
    custom['motor'].update(kp=19,target=7);custom['limits']['angle_deg']=[-7,22]
    before=factory(before.doc);params=copy.deepcopy(obj['parameters']);params['hold_joints']=['upper_body'];params['strength_scale']=2
    after=factory(update_object_parameters(before,'person',params))
    assert sum('motor' in j for j in after.joints if j['id'].startswith('person/'))==12
    elbow=next(j for j in after.joints if j['id']=='person/left_elbow')
    assert elbow['motor']['kp']==19 and elbow['motor']['target']==7
    assert elbow['motor']['max_torque_nm']==pytest.approx(custom['motor']['max_torque_nm']*2)
    assert elbow['limits']['angle_deg']==[-7,22]
    for pid in before.parts: assert np.allclose(before.parts[pid].matrix,after.parts[pid].matrix,atol=1e-6,rtol=0)
    params['hold_pose']=False;params['hold_joints']=[]
    relaxed=factory(update_object_parameters(after,'person',params))
    assert not any('motor' in j for j in relaxed.joints if j['id'].startswith('person/'))


def test_component_size_and_mass_controls_scale_saved_geometry_and_joint_frames(factory,blank):
    original=human(factory,blank);before=factory(regroup_object(factory(expand_objects(original,'person')),'person'))
    params=copy.deepcopy(before.doc['objects'][0]['parameters']);params.update(stature_mm=1980,mass_kg=90)
    after=factory(update_object_parameters(before,'person',params))
    for pid in before.parts:
        if pid.startswith('person/'):
            assert after.parts[pid].mass==pytest.approx(before.parts[pid].mass*1.2)
    for joint in after.joints:
        pa,pb,_=after.joint_frames(joint);assert np.linalg.norm(pa-pb)<1e-6
    params['pose']='standing'
    with pytest.raises(DocumentError,match='edited pose'): update_object_parameters(after,'person',params)


def test_grouped_design_yaml_save_reload_preserves_physics(factory,blank,tmp_path):
    before=human(factory,blank);doc=regroup_object(factory(expand_objects(before,'person')),'person')
    path=tmp_path/'person.pipe.yaml';write(path,doc)
    equivalent(before,Assembly.load(path))
    assert read(path)['objects'][0]['components']['parts'][0]['id']=='pelvis'


def test_save_as_relocates_component_meshes(blank,tmp_path):
    old=tmp_path/'original';new=tmp_path/'saved'
    doc=copy.deepcopy(blank);doc['objects']=[{'id':'mesh','template':'custom','components':{'parts':[
        {'id':'piece','body':{'geometry':[{'type':'mesh','file':'assets/piece.stl'}],'mass_kg':1}}]}}]
    moved=relocate_design(doc,old,new)
    assert moved['objects'][0]['components']['parts'][0]['body']['geometry'][0]['file']=='../original/assets/piece.stl'


def test_detach_move_reattach_keeps_skeleton_and_checks_reach(load,factory):
    before=load('human-pull-up')
    detached=factory(detach_attachment(before,'person','left-hand-grip'))
    assert 'left-hand-grip' not in {j['id'] for j in detached.joints}
    assert 'right-hand-grip' in {j['id'] for j in detached.joints}
    restored=factory(attach_part(detached,'person',reconnect='left-hand-grip')['document'])
    for joint in restored.joints:
        pa,pb,_=restored.joint_frames(joint)
        assert np.linalg.norm(pa-pb)<1e-6,joint['id']
    assert {j['id'] for j in restored.joints}=={j['id'] for j in before.joints}
    free=factory(detach_attachment(detached,'person','right-hand-grip'))
    moved=factory(move_object(free,'person',{'position_mm':[5000,0,0]})['document'])
    with pytest.raises(DocumentError,match='out of reach'): attach_part(moved,'person',reconnect='left-hand-grip')
    assert len(moved.doc['objects'])==1


def test_new_grip_uses_limb_ik_and_preserves_the_other_hand(factory,blank):
    before=human(factory,blank);point=before.parts['person/left_hand'].matrix[:3,3]+[15,0,0]
    doc=before.doc;doc['parts']=[{'id':'bar','catalog':'tubeclamp.tube-C','parameters':{'length_mm':800},'pose':{'position_mm':point.tolist(),'rotation_deg':[0,90,0]}}]
    before=factory(doc);after=factory(attach_part(before,'person','person/left_hand',{'part':'bar','at_mm':400})['document'])
    grip=next(j for j in after.joints if j['id']=='person-left_hand-attachment')
    pa,pb,_=after.joint_frames(grip);assert np.linalg.norm(pa-pb)<.5
    assert np.allclose(before.parts['person/right_hand'].matrix,after.parts['person/right_hand'].matrix,atol=1e-6,rtol=0)
    assert len(after.doc['objects'])==2


def test_grouping_preserves_short_simulation_of_free_legs_and_hand_grips(load,factory):
    from pipesim.physics import simulate
    before=load('human-pull-up');after=factory(regroup_object(factory(expand_objects(before,'person')),'person'))
    results=[simulate(assembly,.15,20) for assembly in (before,after)]
    for pid in before.parts:
        a,b=[transform(r['frames'][-1]['parts'][pid]) for r in results]
        assert np.allclose(a,b,atol=1e-4,rtol=0),pid


def test_regrouped_portable_bundle_keeps_working_mass_controls(factory,blank,tmp_path):
    from pipesim.packaging import bundle
    path=bundle(human(factory,blank),tmp_path/'portable')
    expanded=Assembly.load(path);regrouped=Assembly.from_doc(regroup_object(expanded,'person'),path.parent)
    params=copy.deepcopy(regrouped.doc['objects'][0]['parameters']);params['mass_kg']=90
    after=Assembly.from_doc(update_object_parameters(regrouped,'person',params),path.parent)
    for pid in regrouped.parts:
        if pid.startswith('person/'):
            assert after.parts[pid].mass==pytest.approx(regrouped.parts[pid].mass*1.2)
            assert np.allclose(regrouped.parts[pid].matrix,after.parts[pid].matrix,atol=1e-6,rtol=0)


def test_generic_articulated_object_and_global_joint_ids_roundtrip(factory,blank):
    doc=copy.deepcopy(blank);body={'geometry':[{'type':'box','size_mm':[20,20,50]}],'mass_kg':1}
    doc['objects']=[{'id':'tool','template':'saved-custom-tool','pose':{'position_mm':[100,200,500]},'components':{
        'parts':[{'id':'base','body':copy.deepcopy(body)},{'id':'arm','body':copy.deepcopy(body),'pose':{'position_mm':[0,0,50]}}],
        'joints':[]}}]
    doc['joints']=[{'id':'global-hinge','type':'revolute','a':{'part':'tool/base','frame':{'position_mm':[0,0,25],'axis':[1,0,0]}},
                   'b':{'part':'tool/arm','frame':{'position_mm':[0,0,-25],'axis':[1,0,0]}},'limits':{'angle_deg':[-90,90]}}]
    before=factory(doc);expanded=factory(expand_objects(before));after=factory(regroup_object(expanded,'tool'))
    equivalent(before,after)
    assert after.doc['joints'][0]['id']=='global-hinge'
    assert len(after.rigid_groups())==2


def test_component_world_anchor_can_be_released_without_losing_the_object(factory,blank):
    from pipesim.grouping import release_object_anchor
    before=human(factory,blank);doc=regroup_object(factory(expand_objects(before,'person')),'person')
    doc['objects'][0]['components']['anchors']=[{'part':'pelvis','surface':'ceiling'}]
    before=factory(doc)
    with pytest.raises(DocumentError,match='fixed to the world'): move_object(before,'person',{'position_mm':[4000,0,0]})
    after=factory(release_object_anchor(before,'person','person/pelvis'))
    assert not after.anchors and len(after.doc['objects'])==2
    assert len(move_object(after,'person',{'position_mm':[4000,0,0]})['moved'])==19
