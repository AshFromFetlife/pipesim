import copy

import numpy as np
import pybullet as pb
import pytest

from pipesim.document import DocumentError
from pipesim.editing import expand_objects, snapshot_design
from pipesim.grouping import regroup_object, update_object_parameters
from pipesim.human import humanoid, reach
from pipesim.human_motion import ACTIVITIES, FLEXIBILITIES, activity_torque
from pipesim.human_poses import HUMAN_POSES
from pipesim.physics import World
from pipesim.validation import CollisionWorld


def person(blank, factory, **parameters):
    doc=copy.deepcopy(blank)
    doc['objects']=[{'id':'person','template':'human','parameters':parameters}]
    doc['environment']={'ground':False,'gravity_m_s2':[0,0,0]}
    doc['anchors']=[{'part':'person/pelvis'}]
    return factory(doc)


@pytest.mark.parametrize('flexibility', FLEXIBILITIES)
def test_flexibility_preserves_connected_skeleton_and_accepts_catalog_poses(flexibility):
    from pipesim.math3d import transform, point
    for pose in HUMAN_POSES:
        model=humanoid(flexibility=flexibility,pose=pose)
        parts={p['id']:transform(p['pose']) for p in model['parts']}
        assert len(parts)==19 and len(model['joints'])==18
        for j in model['joints']:
            assert np.linalg.norm(point(parts[j['a']['part']],j['a']['frame']['position_mm'])-
                                  point(parts[j['b']['part']],j['b']['frame']['position_mm']))<1e-6
            limits=j['limits'].get('rotation_deg',[j['limits'].get('angle_deg')])
            assert all(lo<=0<=hi for lo,hi in limits)


def test_envelopes_expand_and_high_flexibility_has_hyperextension():
    joints={mode:{j['id']:j for j in humanoid(flexibility=mode)['joints']} for mode in FLEXIBILITIES}
    for mode,following in zip(('minimum','athletic','gymnast'),('athletic','gymnast','contortionist')):
        a=joints[mode]['right_hip']['limits']['rotation_deg']
        b=joints[following]['right_hip']['limits']['rotation_deg']
        assert all(c<=lo and hi<=d for (lo,hi),(c,d) in zip(a,b))
    assert joints['gymnast']['right_knee']['limits']['rotation_deg'][0][1]>0
    assert joints['gymnast']['right_hip']['limits']['rotation_deg'][1][1]>=90
    assert all(j['type']=='spherical' for j in joints['full_socket_span'].values())
    assert all(j['type']=='spherical' for j in joints['full_360'].values())


@pytest.mark.parametrize('parameters', [
    {'flexibility':'unlimited'}, {'posture_control':'panic'}, {'movement_seed':True},
    {'movement_seed':1.5}, {'movement_seed':-1}, {'movement_seed':2147483648},
    {'flexibility':'minimum','joint_angles_deg':{'right_elbow':130}},
    {'joint_angles_deg':{'right_shoulder':[1,2]}},
    {'joint_angles_deg':{'right_elbow':[20,10,0]}},
])
def test_invalid_new_controls_and_explicit_angles_are_rejected(parameters):
    with pytest.raises(DocumentError): humanoid(**parameters)


@pytest.mark.parametrize('mode', ACTIVITIES)
def test_activity_is_seeded_bounded_and_changes_direction(mode):
    first=np.array([activity_torque(mode,17,'person/right_hip',t,120) for t in np.arange(0,5,.025)])
    repeat=np.array([activity_torque(mode,17,'person/right_hip',t,120) for t in np.arange(0,5,.025)])
    other=np.array([activity_torque(mode,18,'person/right_hip',t,120) for t in np.arange(0,5,.025)])
    assert np.array_equal(first,repeat)
    assert not np.array_equal(first,other)
    assert np.linalg.norm(first,axis=1).max()<=120+1e-9
    assert np.any(first[:,0]>0) and np.any(first[:,0]<0)
    if mode in ('random_spasms','destructive'):
        assert np.any(np.linalg.norm(first,axis=1)==0)
        assert np.linalg.norm(first,axis=1).max()>60
    if mode=='fidget':
        assert np.array_equal(activity_torque(mode,1,'hip',.1,120),activity_torque(mode,1,'hip',.99,120))
        assert not np.array_equal(activity_torque(mode,1,'hip',.99,120),activity_torque(mode,1,'hip',1.01,120))
    if mode=='struggle':
        assert np.linalg.norm(first,axis=1).max()<=120*.18+1e-9
        assert np.linalg.norm(activity_torque(mode,1,'hip',.7-1e-7,120)-activity_torque(mode,1,'hip',.7+1e-7,120))<.001


@pytest.mark.parametrize('mode', ACTIVITIES)
def test_activity_runs_as_physical_torque_without_moving_authored_pose(blank,factory,mode):
    assembly=person(blank,factory,posture_control=mode,movement_seed=9)
    original={pid:p.matrix.copy() for pid,p in assembly.parts.items()}
    with World(assembly) as world:
        for _ in range(25): world.step()
        frame=world.snapshot()
        assert len(frame['activity_efforts'])==18
        assert any(np.linalg.norm(world.part_matrix(pid)-matrix)>1e-5 for pid,matrix in original.items())
        assert all(np.isfinite(world.part_matrix(pid)).all() for pid in original)
        if mode=='fidget': assert all(j.get('motor') for j in assembly.joints)
        else: assert all(not j.get('motor') for j in assembly.joints)
    assert all(np.array_equal(assembly.parts[pid].matrix,matrix) for pid,matrix in original.items())


def test_full_360_suppresses_only_same_person_contact(blank,factory):
    assembly=person(blank,factory,flexibility='full_360')
    assembly.parts['person/right_thigh'].matrix=assembly.parts['person/left_thigh'].matrix.copy()
    other=copy.deepcopy(assembly.parts['person/left_thigh']);other.id='other/left_thigh'
    assembly.parts[other.id]=other
    fixture=copy.deepcopy(other);fixture.id='fixture';fixture.definition['kind']='rigid'
    assembly.parts[fixture.id]=fixture
    with CollisionWorld(assembly) as world:
        assert not world.intersect('person/left_thigh','person/right_thigh')
        assert world.intersect('person/left_thigh','other/left_thigh')
        assert world.intersect('person/left_thigh','fixture')
        # Geometric overlap exists; the mannequin's self contact policy suppresses it.
        a,b=(world.world.part_map['person/'+side+'_thigh'][0] for side in ('left','right'))
        assert pb.getClosestPoints(a,b,0,physicsClientId=world.world.client)
    for part in assembly.parts.values(): part.definition['source']['flexibility']='full_socket_span'
    with CollisionWorld(assembly) as world:
        assert world.intersect('person/left_thigh','person/right_thigh')


def test_fragile_releases_under_stop_load_and_snapshot_preserves_the_injury(blank,factory):
    assembly=person(blank,factory,flexibility='fragile')
    assembly.doc['loads']=[{'part':'person/left_shin','moment_nm':[300,0,0]}]
    original=copy.deepcopy(assembly.joints)
    with World(assembly) as world:
        world.step()
        assert 'person/left_knee' in world.dislocated
        assert not world.broken
        assert any(e['type']=='human_joint_dislocation' for e in world.events)
        for _ in range(5): world.step()
        captured=snapshot_design(assembly,world.snapshot())
        assert all(np.isfinite(world.part_matrix(pid)).all() for pid in assembly.parts)
    restored=factory(captured)
    knee=next(j for j in restored.joints if j['id']=='person/left_knee')
    assert knee['metadata']['dislocated']
    assert knee['limits']['rotation_deg'][0][1]>1e6
    assert assembly.joints==original


def test_fragile_does_not_release_without_a_strong_load(blank,factory):
    assembly=person(blank,factory,flexibility='fragile')
    with World(assembly) as world:
        for _ in range(8): world.step()
        assert not world.dislocated


def test_edited_human_updates_flexibility_activity_strength_and_keeps_pose(blank,factory):
    assembly=person(blank,factory,flexibility='minimum')
    assembly=factory(regroup_object(factory(expand_objects(assembly)),'person'))
    parameters={**assembly.doc['objects'][0]['parameters'],'flexibility':'gymnast',
                'posture_control':'fidget','movement_seed':41,'strength_scale':2}
    changed=factory(update_object_parameters(assembly,'person',parameters))
    assert all(np.allclose(changed.parts[p].matrix,assembly.parts[p].matrix) for p in assembly.parts)
    assert all(j['metadata']['human_activity']['seed']==41 for j in changed.joints)
    assert all(j['metadata']['flexibility']=='gymnast' for j in changed.joints)
    assert all(j.get('motor') for j in changed.joints)
    assert next(j for j in changed.joints if j['id']=='person/left_knee')['type']=='spherical'
    parameters['posture_control']='struggle'
    changed=factory(update_object_parameters(changed,'person',parameters))
    assert all(not j.get('motor') for j in changed.joints)
    assert all(j['metadata']['human_activity']['mode']=='struggle' for j in changed.joints)


def test_unrestricted_elbow_reach_handles_three_axes(blank,factory):
    assembly=person(blank,factory,flexibility='full_360')
    result=reach(assembly,'person',[231,350,1100],check_collision=False)
    assert result['position_reachable']
    assert len(result['angles_deg']['person/right_elbow'])==3


def test_full_socket_parent_contact_blocks_a_fold_through_the_upper_arm(blank,factory):
    from pipesim.snapping import _movement_coordinates
    before=person(blank,factory,flexibility='full_socket_span')
    after=person(blank,factory,flexibility='full_socket_span',joint_angles_deg={'right_elbow':180})
    with pytest.raises(DocumentError,match='intersect its parent'):
        _movement_coordinates(before,after)
    before=person(blank,factory,flexibility='full_360')
    after=person(blank,factory,flexibility='full_360',joint_angles_deg={'right_elbow':180})
    assert 'person/right_elbow' in _movement_coordinates(before,after)


def test_struggle_stays_at_human_movement_speeds_during_sustained_exertion(blank,factory):
    assembly=person(blank,factory,posture_control='struggle')
    with World(assembly) as world:
        for _ in range(120): world.step()
        assert max(np.linalg.norm(world.part_velocity(pid)[0]) for pid in world.part_map)<5
        assert max(np.linalg.norm(world.part_velocity(pid)[1]) for pid in world.part_map)<20


def test_fidget_holds_horizontal_arms_against_gravity_with_small_motion(blank,factory):
    displacements={}
    for mode in ('hold','fidget','passive'):
        assembly=person(blank,factory,pose='arms-forward',posture_control=mode,movement_seed=19)
        assembly.doc['environment']['gravity_m_s2']=[0,0,-9.81]
        with World(assembly) as world:
            for _ in range(240): world.step()
            displacements[mode]=max(np.linalg.norm(world.part_matrix('person/'+side+'_hand')[:3,3]-
                                                  assembly.parts['person/'+side+'_hand'].matrix[:3,3])
                                    for side in ('left','right'))
    # The baseline holding servo has a little gravitational sag. Fidget adds
    # small motion around that posture rather than letting the arms collapse.
    assert displacements['hold']<100
    assert displacements['fidget']<100
    assert displacements['fidget']>displacements['hold']
    assert displacements['fidget']-displacements['hold']<20
    assert displacements['passive']>500
