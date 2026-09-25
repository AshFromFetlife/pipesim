import copy
import threading
import time

import pybullet as pb
import pytest

from pipesim.physics import World, simulate, simplify_chains
from pipesim.simulation_control import SimulationCancelled
from pipesim.simulation_jobs import SimulationJobs
from pipesim.math3d import transform


def chain(factory,blank,count=12):
    doc=copy.deepcopy(blank)
    doc['environment']={'ground':False}
    doc['objects']=[{'id':'chain','template':'chain','parameters':{'length_mm':20*count},
                     'pose':{'position_mm':[0,0,1000],'rotation_deg':[0,25,0]}}]
    doc['anchors']=[{'part':'chain/link-1','surface':'ceiling'}]
    return factory(doc)


def test_simplification_conserves_mass_pose_and_individual_part_ids(factory,blank):
    assembly=chain(factory,blank)
    original=copy.deepcopy(assembly.doc)
    simplified,frozen=simplify_chains(assembly,4)
    assert len(frozen)==8
    assert assembly.doc==original
    assert all(j['type']=='spherical' for j in assembly.joints)
    with World(simplified) as world:
        assert len(world.joint_map)==3
        assert world.part_map['chain/link-1'][:2]!=world.part_map['chain/link-2'][:2]
        groups={}
        for pid,(body,link,_) in world.part_map.items(): groups.setdefault((body,link),[]).append(pid)
        assert max(map(len,groups.values()))==4
        mass=sum(pb.getDynamicsInfo(body,i,physicsClientId=world.client)[0]
                 for body in world.body_ids for i in range(-1,pb.getNumJoints(body,physicsClientId=world.client)))
        assert mass==pytest.approx(sum(p.mass for p in assembly.parts.values())-assembly.parts['chain/link-1'].mass+6e-6)
        for pid,pose in world.snapshot()['parts'].items():
            assert transform(pose)==pytest.approx(assembly.parts[pid].matrix,abs=.001)


@pytest.mark.parametrize('anchor',['chain/link-1','chain/link-12'])
def test_grouped_simulation_keeps_saved_pose_and_reports_approximation(factory,blank,anchor):
    assembly=chain(factory,blank)
    doc=copy.deepcopy(assembly.doc)
    doc['anchors'][0]['part']=anchor
    doc['state']={'joints':{'chain/join-2':{'rotation_deg':[12,3,0]},
                           'chain/join-4':{'rotation_deg':[3,6,2]},
                           'chain/join-5':{'rotation_deg':[0,8,0]}}}
    assembly=factory(doc)
    result=simulate(assembly,.02,chain_links_per_body=4)
    for pid,pose in result['frames'][0]['parts'].items():
        assert transform(pose)==pytest.approx(assembly.parts[pid].matrix,abs=.002)
    frozen=result['chain_simplification']['frozen_joints']
    for jid in frozen:
        assert all(frame['joints'][jid]==result['frames'][0]['joints'][jid] for frame in result['frames'])
        assert all(jid not in frame['reactions'] for frame in result['frames'])
    assert any('Chain simplification' in text for text in result['limitations'])
    assert result['input_sha256']==assembly.input_hash
    assert assembly.doc==doc


def test_motors_breakable_joints_and_external_attachments_remain_flexible(factory,blank):
    assembly=chain(factory,blank)
    assembly.joints[2]['motor']={'rotation_deg':[0,0,0]}
    assembly.joints[4]['break_force_n']=100
    assembly.joints.append({'id':'attachment','type':'distance','a':{'part':'chain/link-8'},'b':{'part':'chain/link-1'}})
    simplified,frozen=simplify_chains(assembly,100)
    assert not {'chain/join-3','chain/join-5','chain/join-7','chain/join-8','attachment'}&set(frozen)
    assert simplified.joints[-1]==assembly.joints[-1]


def test_progress_reports_setup_steps_and_completion(factory,blank):
    updates=[]
    result=simulate(chain(factory,blank,3),.02,progress=updates.append)
    assert {'preparing','building','loading','recording','simulating','finalizing','complete'}<={u['phase'] for u in updates}
    assert updates[-1]['percent']==100
    assert all(a['elapsed_s']<=b['elapsed_s'] for a,b in zip(updates,updates[1:]))
    assert result['chain_simplification']=={'links_per_body':1,'frozen_joints':[]}


@pytest.mark.parametrize('phase',['building','simulating'])
def test_cancellation_releases_world_and_does_not_edit_design(factory,blank,phase,monkeypatch):
    assembly=chain(factory,blank,3);original=copy.deepcopy(assembly.doc)
    cancel=threading.Event();clients=[]
    connect=pb.connect
    def track(*args,**kwargs):
        client=connect(*args,**kwargs);clients.append(client);return client
    monkeypatch.setattr(pb,'connect',track)
    def progress(value):
        if value['phase']==phase: cancel.set()
    with pytest.raises(SimulationCancelled):
        simulate(assembly,1,progress=progress,cancelled=cancel.is_set)
    assert clients and all(not pb.isConnected(client) for client in clients)
    assert assembly.doc==original


@pytest.mark.parametrize('options',[{'duration':float('nan')},{'fps':float('inf')},
    {'dt':float('nan')},{'chain_links_per_body':0},{'chain_links_per_body':True},{'chain_links_per_body':2.5}])
def test_bad_simulation_options_fail_before_building(factory,blank,options):
    with pytest.raises(ValueError): simulate(factory(blank),**options)


def test_worker_progress_result_cancellation_and_restart(factory,blank):
    jobs=SimulationJobs()
    try:
        assembly=chain(factory,blank,3)
        job=jobs.start(assembly,duration=.02,chain_links_per_body=2)
        deadline=time.monotonic()+30
        while job['status']=='running':
            assert time.monotonic()<deadline
            time.sleep(.05);job=jobs.get(job['id'])
        assert job['status']=='completed',job
        assert 'result' not in job
        result=jobs.get(job['id'],include_result=True)['result']
        assert result['frames'] and result['chain_simplification']['links_per_body']==2
        running=jobs.start(assembly,duration=30)
        directory=jobs.jobs[running['id']]['directory'].name
        with pytest.raises(ValueError,match='already running'): jobs.start(assembly,duration=30)
        stopped=jobs.get(running['id'],cancel=True)
        assert stopped['status']=='cancelled'
        from pathlib import Path
        assert not Path(directory).exists()
        assert jobs.get(running['id'],cancel=True)['status']=='cancelled'
        next_job=jobs.start(assembly,duration=30)
        assert jobs.get(next_job['id'],cancel=True)['status']=='cancelled'
    finally: jobs.close()


def test_worker_failure_is_reported_and_a_new_job_can_start(factory,blank):
    jobs=SimulationJobs(log_progress=False)
    try:
        assembly=chain(factory,blank,3)
        assembly.doc['anchors'][0]['dofs']=['z']
        job=jobs.start(assembly,duration=.02)
        deadline=time.monotonic()+30
        while job['status']=='running':
            assert time.monotonic()<deadline
            time.sleep(.05);job=jobs.get(job['id'])
        assert job['status']=='failed'
        assert 'fixed world anchors' in job['error']
        job=jobs.start(chain(factory,blank,3),duration=30)
        process=jobs.jobs[job['id']]['process']
        process.terminate();process.join(timeout=2)
        crashed=jobs.get(job['id'])
        assert crashed['status']=='failed'
        assert 'exited unexpectedly' in crashed['error']
    finally: jobs.close()
