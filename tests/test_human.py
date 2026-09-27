import copy
import json
from pathlib import Path
import numpy as np
import pytest
from pipesim.human import humanoid,reach,seat_fit,clearance,run_fit_tests
from pipesim.human_poses import HUMAN_POSES
from pipesim.physics import simulate
from pipesim.geometry import bounds
from pipesim.document import DocumentError
from pipesim.math3d import transform

def test_nineteen_segments_total_mass_and_anatomical_pivots(load):
    a=load('human')
    assert len(a.parts)==19 and len(a.joints)==18
    assert sum(p.mass for p in a.parts.values())==pytest.approx(75)
    box=np.array([bounds(p) for p in a.parts.values()])
    assert box[:,0,2].min()==pytest.approx(0,abs=.01)
    assert box[:,1,2].max()==pytest.approx(1750,abs=1)
    for j in a.joints:
        left,right,_=a.joint_frames(j)
        assert np.linalg.norm(left-right)<.001


def test_mass_changes_body_envelope_without_changing_skeleton(blank,factory):
    bodies={}
    for mass in (50,100):
        doc=copy.deepcopy(blank)
        doc['objects']=[{'id':'person','template':'human','parameters':{'stature_mm':1750,'mass_kg':mass}}]
        bodies[mass]=factory(doc)
    lean,heavy=bodies[50],bodies[100]
    assert len(lean.parts)==len(heavy.parts)==19
    for name in ('pelvis','lumbar','thorax'):
        small=lean.parts['person/'+name].shapes[0]['size_mm']
        large=heavy.parts['person/'+name].shapes[0]['size_mm']
        assert large[0]>small[0] and large[1]>small[1]
        assert large[2]==pytest.approx(small[2])
    assert heavy.parts['person/lumbar'].shapes[0]['size_mm'][1]>lean.parts['person/lumbar'].shapes[0]['size_mm'][1]*1.4
    for side in ('left','right'):
        assert heavy.parts[f'person/{side}_thigh'].shapes[0]['radius_mm']>lean.parts[f'person/{side}_thigh'].shapes[0]['radius_mm']*1.25
    for name in lean.parts:
        assert np.allclose(lean.parts[name].matrix,heavy.parts[name].matrix,atol=1e-8)
    for name in ('head','neck','left_hand','right_hand'):
        assert lean.parts['person/'+name].shapes==heavy.parts['person/'+name].shapes
    assert sum(part.mass for part in lean.parts.values())==pytest.approx(50)
    assert sum(part.mass for part in heavy.parts.values())==pytest.approx(100)


def test_seat_width_responds_to_body_mass(load,factory):
    baseline=load('seated-human')
    widths=[]
    for mass in (50,100):
        doc=copy.deepcopy(baseline.doc)
        doc['objects'][0]['parameters']['mass_kg']=mass
        widths.append(seat_fit(factory(doc),'person','seat')['required_width_mm'])
    assert widths[1]>widths[0]+60

@pytest.mark.parametrize('pose',['standing','seated','crouching'])
def test_poses_preserve_joint_pivots_and_lengths(blank,factory,pose):
    blank['objects']=[{'id':'p','template':'human','parameters':{'pose':pose,'stature_mm':1800,'mass_kg':90}}]
    a=factory(blank)
    for j in a.joints:
        l,r,_=a.joint_frames(j)
        assert np.linalg.norm(l-r)<.001
    assert sum(p.mass for p in a.parts.values())==pytest.approx(90)


def test_all_initial_poses_are_distinct_connected_and_within_limits(blank,factory):
    assert 30<=len(HUMAN_POSES)<=45
    signatures=set()
    for name,preset in HUMAN_POSES.items():
        model=humanoid(pose=name)
        assert len(model['parts'])==19 and len(model['joints'])==18, name
        poses={p['id']:p['pose'] for p in model['parts']}
        signature=tuple(round(value,3) for p in model['parts'] for value in
                        (*p['pose']['position_mm'],*p['pose']['rotation_deg']))
        assert signature not in signatures, name
        signatures.add(signature)
        for joint in model['joints']:
            a,b=joint['a'],joint['b']
            left=transform(poses[a['part']])@np.r_[a['frame']['position_mm'],1]
            right=transform(poses[b['part']])@np.r_[b['frame']['position_mm'],1]
            assert np.linalg.norm(left-right)<.001, (name,joint['id'])
        assert preset['label'] and preset['description'] and preset['group']
        if preset['ground']:
            doc=copy.deepcopy(blank)
            doc['objects']=[{'id':'person','template':'human','parameters':{'pose':name}}]
            assembly=factory(doc)
            assert min(bounds(p)[0,2] for p in assembly.parts.values())==pytest.approx(0,abs=1), name


def test_editor_pose_catalog_matches_generator():
    source=(Path(__file__).parents[1]/'pipesim/web/human-poses.js').read_text(encoding='utf-8')
    options=json.loads(source.split('export const HUMAN_POSES=',1)[1].rsplit(';',1)[0])
    assert options==[{'id':name, **{key:preset[key] for key in ('label','group','description')}}
                     for name,preset in HUMAN_POSES.items()]

def test_custom_limb_measurement_changes_actual_chain_length(blank,factory):
    blank['objects']=[{'id':'p','template':'human','parameters':{'measurements':{'upper_arm_length_mm':400}}}]
    a=factory(blank)
    joints={j['id']:j for j in a.joints}
    shoulder=a.joint_frames(joints['p/right_shoulder'])[0]
    elbow=a.joint_frames(joints['p/right_elbow'])[0]
    assert np.linalg.norm(shoulder-elbow)==pytest.approx(400)

def test_reachable_target_and_impossible_target(load):
    a=load('human')
    near=reach(a,'person',[231,350,1100])
    assert near['reachable'] and near['error_mm']<5
    far=reach(a,'person',[231,3000,1100])
    assert not far['reachable'] and far['error_mm']>1500

def test_reach_cannot_pass_a_hand_through_a_box(load,factory):
    doc=copy.deepcopy(load('human').doc)
    doc['parts'].append({'id':'obstacle','catalog':'generic.box','pose':{'position_mm':[231,350,1100]}})
    result=reach(factory(doc),'person',[231,350,1100])
    assert result['position_reachable'] and not result['reachable']
    assert any('obstacle' in pair for pair in result['collisions'])

def test_seat_dimensions_and_small_seat_failure(load,factory):
    a=load('seated-human'); result=seat_fit(a,'person','seat')
    assert result['fits']
    assert result['current_pose']['thigh_nearly_horizontal']
    assert result['required_depth_mm']==pytest.approx(.235*1750*.65)
    doc=copy.deepcopy(a.doc); doc['parts'][0]['parameters']['width_mm']=200
    assert not seat_fit(factory(doc),'person','seat')['fits']

def test_clearance_detects_structure_through_torso(load,factory):
    a=load('human'); assert clearance(a,'person',10)['clear']
    doc=copy.deepcopy(a.doc)
    doc['parts'].append({'id':'obstacle','catalog':'generic.box','pose':{'position_mm':[0,0,1300]}})
    assert not clearance(factory(doc),'person',10)['clear']

def test_saved_expected_negative_fit_is_a_passing_design_test(load):
    assert run_fit_tests(load('seated-human'))['passed']

def test_bounded_posture_servos_keep_a_seated_manikin_on_the_seat(load,factory):
    doc=copy.deepcopy(load('seated-human').doc)
    doc['objects'][0]['parameters']['hold_pose']=True
    r=simulate(factory(doc),1.5,10)
    pelvis=r['frames'][-1]['parts']['person/pelvis']
    assert abs(pelvis['position_mm'][1])<100
    assert 500<pelvis['position_mm'][2]<650
    assert abs(pelvis['rotation_deg'][0])<15
    assert r['frames'][-1]['contacts']

@pytest.mark.parametrize('params',[{'stature_mm':10},{'mass_kg':-1},{'measurements':{'thigh_length_mm':-1}},{'measurements':{'arm_lenght_mm':100}},{'joint_angles_deg':{'right_knee':90}}])
def test_invalid_anatomy_is_rejected(params):
    with pytest.raises(DocumentError): humanoid(**params)
