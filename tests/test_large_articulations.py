import copy

import numpy as np
import pybullet as pb
import pytest

import pipesim.physics as physics
from pipesim.math3d import transform


def hanging_chain(factory,blank,count=250):
    doc=copy.deepcopy(blank)
    doc['environment']={'ground':False}
    doc['objects']=[{'id':'chain','template':'chain','parameters':{'length_mm':count*20},
                     'pose':{'position_mm':[300,-200,6500],'rotation_deg':[0,35,20]}}]
    doc['anchors']=[{'part':'chain/link-1','surface':'ceiling'}]
    chain=factory(doc)
    end=f'chain/link-{count}'
    pivot=chain.parts[end].frame({'port':'a'})[0]
    doc['parts']=[{'id':'load','catalog':'generic.box',
                   'parameters':{'width_mm':200,'depth_mm':200,'height_mm':200,'mass_kg':115},
                   'pose':{'position_mm':(pivot-[0,0,100]).tolist()}}]
    doc['joints']=[{'id':'sling','type':'spherical','a':{'part':end,'port':'a'},
                    'b':{'part':'load','frame':{'position_mm':[0,0,100]}}}]
    return factory(doc)


@pytest.mark.parametrize('count',[40,41,250])
def test_long_chain_keeps_every_joint_readable_and_conserves_mass(factory,blank,count):
    assembly=hanging_chain(factory,blank,count)
    original=copy.deepcopy(assembly.doc)
    with physics.World(assembly) as world:
        assert all(pb.getNumJoints(b,physicsClientId=world.client)<=120 for b in world.body_ids)
        frame=world.snapshot()
        assert set(frame['parts'])==set(assembly.parts)
        assert set(frame['joints'])=={j['id'] for j in assembly.joints}
        for pid,part in assembly.parts.items():
            assert transform(frame['parts'][pid])==pytest.approx(part.matrix,abs=.001)
        for mapping in world.joint_map.values():
            assert set(mapping)=={'rx','ry','rz'}
            for body,index in mapping.values():
                assert np.isfinite(pb.getJointState(body,index,physicsClientId=world.client)[0])
        mass=sum(pb.getDynamicsInfo(b,i,physicsClientId=world.client)[0]
                 for b in world.body_ids for i in range(-1,pb.getNumJoints(b,physicsClientId=world.client)))
        expected=sum(p.mass for p in assembly.parts.values())-assembly.parts['chain/link-1'].mass
        assert mass==pytest.approx(expected+2e-6*len(assembly.joints),abs=1e-8)
    assert assembly.doc==original


def test_partitioned_chain_initial_coordinates_match_the_editor(factory,blank,monkeypatch):
    monkeypatch.setattr(physics,'MAX_ARTICULATION_JOINTS',9)
    assembly=hanging_chain(factory,blank,10)
    doc=copy.deepcopy(assembly.doc)
    doc['state']={'joints':{'chain/join-1':{'rotation_deg':[4,5,0]},
                           'chain/join-5':{'rotation_deg':[0,-3,4]}}}
    assembly=factory(doc)
    result=physics.simulate(assembly,.01,30)
    initial=result['frames'][0]
    for pid,part in assembly.parts.items():
        assert transform(initial['parts'][pid])==pytest.approx(part.matrix,abs=.002)
    for jid,values in doc['state']['joints'].items():
        assert initial['joints'][jid]['rotation_deg']==pytest.approx(values['rotation_deg'])


def test_branched_articulation_shares_a_root_without_duplicating_its_mass(factory,blank,monkeypatch):
    monkeypatch.setattr(physics,'MAX_ARTICULATION_JOINTS',6)
    blank['environment']={'ground':False}
    blank['parts']=[{'id':f'part-{i}','body':{'kind':'rigid','mass_kg':2,
        'geometry':[{'type':'sphere','radius_mm':10}]},'pose':{'position_mm':[i*100,0,1000]}}
        for i in range(9)]
    blank['joints']=[{'id':f'branch-{i}','type':'spherical','a':{'part':'part-0','frame':{'position_mm':[i*100,0,0]}},
        'b':{'part':f'part-{i}'},'limits':{'rotation_deg':[[-30,30]]*3}} for i in range(1,9)]
    assembly=factory(blank)
    with physics.World(assembly) as world:
        assert len(world.body_ids)==4
        initial=world.snapshot()
        for _ in range(12): world.step()
        frame=world.snapshot()
        for pid in assembly.parts:
            displacement=np.array(frame['parts'][pid]['position_mm'])-initial['parts'][pid]['position_mm']
            assert displacement==pytest.approx([0,0,-.5*9810*.05**2],abs=1.5)
        assert set(frame['joints'])=={j['id'] for j in assembly.joints}
