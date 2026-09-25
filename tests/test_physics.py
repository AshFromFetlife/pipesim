import copy
import math
import numpy as np
import pytest
import pybullet as pb
from pipesim.document import DocumentError
from pipesim.physics import World,simulate
from pipesim.editing import snapshot_design,expand_objects

def test_free_fall_matches_gravity_before_impact(blank,factory):
    blank['parts']=[{'id':'load','catalog':'generic.box','pose':{'position_mm':[0,0,2000]}}]
    r=simulate(factory(blank),.2,20)
    z=r['frames'][-1]['parts']['load']['position_mm'][2]
    assert z==pytest.approx(2000-.5*9810*.2**2,abs=5)
    assert not r['settled']

def test_loose_collar_falls_to_real_stop_and_locked_collar_stays(load,factory):
    a=load('sliding-collar'); r=simulate(a,1.2,10)
    z=r['frames'][-1]['parts']['slider']['position_mm'][2]
    assert 60<z<75
    assert r['settled']
    assert r['frames'][-1]['contacts']
    doc=copy.deepcopy(a.doc); next(j for j in doc['joints'] if j['id']=='loose-screw')['locked']=True
    locked=simulate(factory(doc),.25,10)
    assert locked['frames'][-1]['parts']['slider']['position_mm'][2]==pytest.approx(1300,abs=.01)

def test_collar_disengages_and_continues_falling(load,factory):
    doc=copy.deepcopy(load('sliding-collar').doc)
    doc['parts']=[p for p in doc['parts'] if p['id']!='stop']
    doc['joints']=[j for j in doc['joints'] if j['id']=='loose-screw']
    doc['joints'][0]['limits']['slide_mm']=[-3000,3000]
    doc['environment']={'ground':False}
    r=simulate(factory(doc),1.1,20)
    assert any(e['type']=='socket_disengaged' for e in r['events'])
    assert r['frames'][-1]['parts']['slider']['position_mm'][2]<-2000
    assert 'loose-screw' in r['frames'][-1]['broken_joints']

def test_fixed_bond_break_releases_child_under_gravity(blank,factory):
    blank['parts']=[{'id':'a','catalog':'generic.box','pose':{'position_mm':[0,0,2000]}},
                    {'id':'b','catalog':'generic.box','pose':{'position_mm':[300,0,2000]}}]
    blank['anchors']=[{'part':'a','surface':'ceiling'}]
    blank['joints']=[{'id':'bond','type':'fixed','a':{'part':'a'},'b':{'part':'b'},'break_force_n':50}]
    r=simulate(factory(blank),.3,10)
    assert r['events'][0]['type']=='joint_break'
    assert r['events'][0]['force_n']==pytest.approx(20*9.81,rel=.01)
    assert r['frames'][-1]['parts']['b']['position_mm'][2]<1700
    assert r['frames'][-1]['parts']['a']['position_mm'][2]==pytest.approx(2000)

def test_gt2_converts_five_turns_to_200_mm(load,factory):
    a=load('gt2-stage'); doc=copy.deepcopy(a.doc)
    j=next(j for j in doc['joints'] if j.get('motor'))
    j['motor']['schedule']=[{'time_s':0,'target':0},{'time_s':1,'target':1800},{'time_s':1.5,'target':1800}]
    r=simulate(factory(doc),1.5,10)
    joints=r['frames'][-1]['joints']
    driver=doc['drives'][0]['driver']; follower=doc['drives'][0]['follower']
    assert joints[driver]['angle_deg']==pytest.approx(1800,abs=1)
    # 5 turns × 20 teeth × 2 mm pitch = 200 mm.
    assert joints[follower]['slide_mm']==pytest.approx(200,abs=.5)

def test_human_fall_does_not_generate_explosive_energy(load):
    a=load('human')
    r=simulate(a,2.5,20)
    positions=np.array([[p['position_mm'] for p in f['parts'].values()] for f in r['frames']])/1000
    speeds=np.linalg.norm(np.diff(positions,axis=0)*20,axis=2)
    assert speeds.max()<10
    assert r['final_max_speed_m_s']<.5
    for jid,values in r['frames'][-1]['joints'].items():
        j=next(j for j in a.joints if j['id']==jid)
        for key,value in values.items():
            pairs=zip(value,j['limits'][key]) if isinstance(value,list) else [(value,j['limits'][key])]
            for v,(lo,hi) in pairs: assert lo-2<v<hi+2

def test_initial_joint_coordinate_agrees_with_renderer(load,factory):
    doc=copy.deepcopy(load('sliding-collar').doc); doc['state']={'joints':{'loose-screw':{'slide_mm':350,'angle_deg':20}}}
    a=factory(doc); r=simulate(a,.01,20)
    assert r['frames'][0]['parts']['slider']['position_mm']==pytest.approx(a.parts['slider'].matrix[:3,3],abs=.01)

def test_snapshot_is_editable_and_preserves_remaining_slide_limits(load,factory):
    a=load('sliding-collar'); r=simulate(a,.25,20); frame=r['frames'][-1]
    saved=factory(snapshot_design(a,frame))
    assert saved.parts['slider'].matrix[:3,3]==pytest.approx(frame['parts']['slider']['position_mm'],abs=.01)
    q=frame['joints']['loose-screw']['slide_mm']
    j=next(j for j in saved.joints if j['id']=='loose-screw')
    assert j['limits']['slide_mm'][1]==pytest.approx(1250-q)
    assert simulate(saved,.1,10)['frames'][-1]['parts']['slider']['position_mm'][2]<frame['parts']['slider']['position_mm'][2]

def test_partial_anchor_is_explicitly_rejected_by_physics(load,factory):
    doc=copy.deepcopy(load('cantilever').doc); doc['anchors'][0]['dofs']=['x','y','z']
    with pytest.raises(DocumentError,match='partial DOFs'): simulate(factory(doc),.1)

def test_dynamic_load_trial_records_overturning(blank,factory):
    from pipesim.physics import rigid_load_trials
    blank['parts']=[{'id':'tower','catalog':'generic.box','parameters':{'width_mm':100,'depth_mm':100,'height_mm':1000,'mass_kg':10},'pose':{'position_mm':[0,0,500]}}]
    report=rigid_load_trials(factory(blank),'tower',[[1,0,0]],200,steps=2,dwell_s=.5)
    failure=report['trials'][0]['first_event']
    assert failure is not None
    assert failure['movement_over_50_mm'] or any(e['type']=='tipped_or_rotated' for e in failure['events'])

def test_shape_offset_sets_physical_com_and_preserves_authoring_origin(blank,factory):
    blank['parts']=[{'id':'asymmetric','pose':{'position_mm':[50,60,700]},'body':{'kind':'rigid','mass_kg':5,'geometry':[{'type':'box','size_mm':[100,100,100],'position_mm':[100,0,0]}]}}]
    a=factory(blank)
    assert a.parts['asymmetric'].center_of_mass==pytest.approx([100,0,0])
    with World(a) as world:
        assert world.part_matrix('asymmetric')[:3,3]==pytest.approx([50,60,700],abs=.01)
        body=world.part_map['asymmetric'][0]
        assert pb.getBasePositionAndOrientation(body,physicsClientId=world.client)[0]==pytest.approx([.15,.06,.7])

def test_expanding_a_posed_object_does_not_double_apply_state(load,factory):
    doc=copy.deepcopy(load('human').doc); doc['state']={'joints':{'person/right_elbow':{'angle_deg':40}}}
    a=factory(doc); b=factory(expand_objects(a))
    for pid in a.parts: assert np.allclose(a.parts[pid].matrix,b.parts[pid].matrix,atol=1e-5)
