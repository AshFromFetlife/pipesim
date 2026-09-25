"""Two-handed support, selective muscle control and passive human motion."""
import copy
from pathlib import Path
import numpy as np
import pybullet as pb
import pytest
from scipy.spatial.transform import Rotation
from pipesim.document import Assembly,DocumentError,read,write
from pipesim.editing import expand_objects
from pipesim.human import humanoid,ARM_JOINTS,TORSO_JOINTS,LEG_JOINTS
from pipesim.math3d import point,transform
from pipesim.physics import World,simulate
from pipesim.validation import validate

ROOT=Path(__file__).resolve().parents[1]


def test_upper_body_control_leaves_all_six_leg_joints_passive():
    human=humanoid(pose='pull-up',hold_joints=['upper_body'],grip_diameter_mm=42.4)
    held={j['id'] for j in human['joints'] if j.get('motor')}
    assert held==set(ARM_JOINTS+TORSO_JOINTS)
    assert not held.intersection(LEG_JOINTS)
    assert len(human['parts'])==19
    assert sum(p['body']['mass_kg'] for p in human['parts'])==pytest.approx(75)
    for j in human['joints']:
        byid={p['id']:p for p in human['parts']}
        pivots=[point(transform(byid[j[end]['part']]['pose']),j[end]['frame']['position_mm']) for end in ('a','b')]
        assert np.linalg.norm(pivots[0]-pivots[1])<1e-6
    for side in ('left','right'):
        hand=next(p for p in human['parts'] if p['id']==side+'_hand')['body']
        assert hand['ports']['grip']['position_mm']==[0,0,0]
        rings=[s for s in hand['geometry'] if s['type']=='tube']
        assert len(rings)==4
        assert all(s['diameter_mm']-2*s['wall_mm']==pytest.approx(44.4) for s in rings)


def test_individual_muscles_and_legacy_whole_body_hold():
    selected=humanoid(hold_joints=['left_elbow','right_elbow','torso','left_elbow'])
    assert {j['id'] for j in selected['joints'] if j.get('motor')}==set(TORSO_JOINTS+('left_elbow','right_elbow'))
    assert all(j.get('motor') for j in humanoid(hold_pose=True)['joints'])
    assert all(not j.get('motor') for j in humanoid()['joints'])
    assert all(j['damping']==1 for j in humanoid(joint_damping_nms_rad=1)['joints'])


@pytest.mark.parametrize('params',[
    {'hold_joints':'arms'}, {'hold_joints':[123]}, {'hold_joints':['left_elbw']},
    {'hold_pose':'arms'}, {'grip_diameter_mm':0}, {'grip_diameter_mm':float('nan')},
    {'grip_diameter_mm':100}, {'joint_angles_deg':{'left_elbw':40}},
    {'joint_damping_nms_rad':-1}, {'joint_damping_nms_rad':float('inf')},
])
def test_invalid_grip_and_posture_parameters_are_rejected(params):
    with pytest.raises(DocumentError): humanoid(**params)


@pytest.mark.parametrize('reverse',[False,True])
def test_two_hand_loop_keeps_every_anatomical_limit_and_motor(load,reverse):
    assembly=load('human-pull-up')
    if reverse: assembly.joints.reverse()
    anatomical={j['id'] for j in assembly.joints if j.get('metadata',{}).get('anatomical')}
    with World(assembly) as world:
        assert anatomical<=world.joint_map.keys()
        assert len(anatomical)==18
        assert len({'left-hand-grip','right-hand-grip'}&world.joint_map.keys())==1
        assert len(world.constraints)==2  # The other revolute grip closes the loop.
        for j in assembly.joints:
            if j['id'] not in anatomical: continue
            for coordinate,(body,index) in world.joint_map[j['id']].items():
                limits=j['limits']['angle_deg'] if coordinate=='angle' else j['limits']['rotation_deg'][('rx','ry','rz').index(coordinate)]
                actual=pb.getJointInfo(body,index,physicsClientId=world.client)[8:10]
                assert np.rad2deg(actual)==pytest.approx(limits)


def test_a_cycle_with_no_passive_closure_is_explicitly_rejected(load,factory):
    doc=copy.deepcopy(load('human-pull-up').doc)
    for j in doc['joints']:
        if j.get('metadata',{}).get('human_grip'): j['limits']={'angle_deg':[-45,45]}
    with pytest.raises(DocumentError,match='passive hinge or ball joint without travel limits'):
        with World(factory(doc)): pass


@pytest.mark.parametrize('anchored',['a','b'])
def test_spherical_coordinates_match_when_the_parent_or_child_is_anchored(blank,factory,anchored):
    blank['parts']=[{'id':pid,'catalog':'generic.box','pose':{'position_mm':[0,0,z]}} for pid,z in [('a',1000),('b',1400)]]
    limits=[[-35,50],[-40,70],[-60,80]]
    blank['joints']=[{'id':'joint','type':'spherical','a':{'part':'a','frame':{'position_mm':[0,0,200]}},
                     'b':{'part':'b','frame':{'position_mm':[0,0,-200]}},'limits':{'rotation_deg':limits}}]
    blank['anchors']=[{'part':anchored,'surface':'fixture'}]
    angles=[12,-17,22]
    with World(factory(blank)) as world:
        world.set_coordinates({'joint':{'rotation_deg':angles}})
        a,b=(world.part_matrix(pid) for pid in ('a','b'))
        assert a[:3,:3].T@b[:3,:3]==pytest.approx(Rotation.from_euler('XYZ',angles,degrees=True).as_matrix(),abs=1e-6)
        assert point(a,[0,0,200])==pytest.approx(point(b,[0,0,-200]),abs=.001)
        assert world.snapshot()['joints']['joint']['rotation_deg']==pytest.approx(angles)


@pytest.fixture(scope='module')
def held_interaction(library):
    assembly=Assembly.from_doc(read(ROOT/'examples/human-pull-up.pipe.yaml'),ROOT/'examples',library)
    return assembly,simulate(assembly,4,20)


def grip_errors(assembly,recording):
    errors=[]
    for j in assembly.joints:
        if not j.get('metadata',{}).get('human_grip'): continue
        local=[point(np.linalg.inv(assembly.parts[j[end]['part']].matrix),assembly.parts[j[end]['part']].frame(j[end])[0]) for end in ('a','b')]
        for frame in recording['frames']:
            pivots=[point(transform(frame['parts'][j[end]['part']]),p) for end,p in zip(('a','b'),local)]
            errors.append(np.linalg.norm(pivots[0]-pivots[1]))
    return np.array(errors)


def test_pull_up_holds_arms_and_both_grips_while_legs_swing(held_interaction):
    assembly,r=held_interaction
    assert len(assembly.parts)==59
    assert validate(assembly)['valid']
    assert not r['events'] and not assembly.doc.get('animation')
    assert not any(a['part'].startswith('person/') for a in assembly.anchors)
    assert grip_errors(assembly,r).max()<2
    for frame in r['frames']:
        assert frame['parts']['top-bar-front']['position_mm']==pytest.approx(assembly.parts['top-bar-front'].matrix[:3,3],abs=.001)
        assert not frame['broken_joints']
    for side in ('left','right'):
        elbow=np.array([f['joints']['person/'+side+'_elbow']['angle_deg'] for f in r['frames']])
        shoulder=np.array([f['joints']['person/'+side+'_shoulder']['rotation_deg'] for f in r['frames']])
        assert abs(elbow).max()<2
        assert abs(shoulder).max()<3
        # Motion relative to the pelvis proves the legs are articulated, rather
        # than just carried around by a rigid whole-body pendulum.
        relative=[]
        for f in r['frames']:
            relative.append(point(np.linalg.inv(transform(f['parts']['person/pelvis'])),f['parts']['person/'+side+'_foot']['position_mm']))
            assert f['parts']['person/'+side+'_foot']['position_mm'][2]>150
        swing=np.array(relative)[:,1]
        assert np.ptp(swing)>75
        velocity=np.diff(swing)
        assert velocity.min()<-1 and velocity.max()>1
        for joint in ('hip','knee','ankle'):
            assert all('person/'+side+'_'+joint not in f['motor_efforts'] for f in r['frames'])
    efforts=[]
    for j in assembly.joints:
        if not j.get('motor'): continue
        for f in r['frames']:
            values=[v['torque_nm'] for v in f['motor_efforts'][j['id']].values()]
            assert np.abs(values).max()<=j['motor']['max_torque_nm']+1e-5
            efforts.extend(values)
    assert np.abs(efforts).max()>10


def test_relaxed_comparison_unfolds_arms_but_keeps_hands_attached(held_interaction,library):
    _,held=held_interaction
    assembly=Assembly.from_doc(read(ROOT/'examples/human-pull-up-relaxed.pipe.yaml'),ROOT/'examples',library)
    relaxed=simulate(assembly,3,20)
    assert grip_errors(assembly,relaxed).max()<2
    assert all(not f['motor_efforts'] for f in relaxed['frames'])
    pelvis=np.array([f['parts']['person/pelvis']['position_mm'] for f in relaxed['frames']])
    assert pelvis[:,2].max()<pelvis[0,2]+30  # No launch when elbows hit their stops.
    assert np.linalg.norm(np.diff(pelvis,axis=0)*20/1000,axis=1).max()<6
    for side in ('left','right'):
        assert relaxed['frames'][-1]['joints']['person/'+side+'_elbow']['angle_deg']<-90
    assert held['frames'][-1]['parts']['person/pelvis']['position_mm'][2]-relaxed['frames'][-1]['parts']['person/pelvis']['position_mm'][2]>250


def test_hand_attachments_and_posture_survive_saved_and_expanded_designs(load,library,tmp_path):
    assembly=load('human-pull-up')
    path=tmp_path/'interaction.pipe.yaml';write(path,assembly.doc)
    reopened=Assembly.from_doc(read(path),tmp_path,library)
    expanded=Assembly.from_doc(expand_objects(assembly),assembly.base,library)
    for other in (reopened,expanded):
        assert len(other.parts)==59
        assert sum(bool(j.get('motor')) for j in other.joints)==12
        assert {j['id'] for j in other.joints if j.get('metadata',{}).get('human_grip')}=={'left-hand-grip','right-hand-grip'}
        for pid in assembly.parts:
            assert other.parts[pid].matrix==pytest.approx(assembly.parts[pid].matrix,abs=1e-6)
