import copy

import numpy as np
import pybullet as pb
import pytest

from pipesim.physics import World, simulate


def carriage(blank,kind='cylindrical',swing=False):
    doc=copy.deepcopy(blank)
    doc['environment']={'ground':False}
    doc['parts']=[{'id':'frame','catalog':'generic.box','pose':{'position_mm':[0,0,2000]}},
                  {'id':'carriage','catalog':'generic.box','pose':{'position_mm':[400,0,2000] if swing else [0,0,3000]}}]
    doc['anchors']=[{'part':'frame','surface':'fixture'}]
    doc['joints']=[]
    for i,station in enumerate((-300,300)):
        doc['joints'].append({'id':f'bearing-{i}','type':kind,'damping':0,
            'a':{'part':'frame','frame':{'position_mm':[0,station,0] if swing else [station,0,1000],'axis':[0,1,0] if swing else [0,0,1]}},
            'b':{'part':'carriage','frame':{'position_mm':[-400,station,0] if swing else [station,0,0]}}})
    return doc


@pytest.mark.parametrize('kind',['cylindrical','prismatic'])
def test_two_rail_carriage_slides_under_gravity_without_twisting(factory,blank,kind):
    doc=carriage(blank,kind);before=copy.deepcopy(doc)
    result=simulate(factory(doc),.2,20)
    final=result['frames'][-1]
    assert final['parts']['carriage']['position_mm']==pytest.approx([0,0,3000-.5*9810*.2**2],abs=5)
    assert final['parts']['carriage']['rotation_deg']==pytest.approx([0,0,0],abs=.05)
    assert final['parts']['frame']['position_mm']==pytest.approx([0,0,2000],abs=.001)
    assert final['joints']['bearing-0']['slide_mm']==pytest.approx(final['joints']['bearing-1']['slide_mm'],abs=.1)
    assert doc==before


def test_two_coaxial_bearings_allow_gravity_swing(factory,blank):
    result=simulate(factory(carriage(blank,swing=True)),.3,30)
    final=result['frames'][-1]
    pos=final['parts']['carriage']['position_mm']
    assert pos[2]<1800 and abs(pos[1])<.1
    assert np.linalg.norm(np.array(pos)-[0,0,2000])==pytest.approx(400,abs=.2)
    assert abs(final['joints']['bearing-0']['angle_deg'])>20
    assert result['final_max_speed_m_s']<3


def test_closure_retains_slide_limits_and_total_mass(factory,blank):
    doc=carriage(blank)
    doc['joints'][1]['limits']={'slide_mm':[-100,100]}
    assembly=factory(doc)
    with World(assembly) as world:
        mass=sum(pb.getDynamicsInfo(body,link,physicsClientId=world.client)[0]
                 for body in world.body_ids for link in range(-1,pb.getNumJoints(body,physicsClientId=world.client)))
        assert mass==pytest.approx(assembly.parts['carriage'].mass,abs=1e-5)
        for _ in range(240): world.step()
        assert world.part_matrix('carriage')[2,3]==pytest.approx(2900,abs=.5)
        assert world.joint_state('bearing-1','slide')[0]==pytest.approx(-.1,abs=.0005)


def test_loop_motor_drives_both_rails(factory,blank):
    doc=carriage(blank,'prismatic');doc['environment']['gravity_m_s2']=[0,0,0]
    doc['joints'][1]['motor']={'mode':'position','target':100,'max_force_n':1000}
    result=simulate(factory(doc),1,10)
    final=result['frames'][-1]
    assert final['parts']['carriage']['position_mm'][2]==pytest.approx(3100,abs=.5)
    for jid in ('bearing-0','bearing-1'):
        assert final['joints'][jid]['slide_mm']==pytest.approx(100,abs=.5)


@pytest.mark.parametrize('reverse_order',[False,True])
def test_sliding_closure_keeps_a_limited_hinge_in_the_tree(factory,blank,reverse_order):
    doc=carriage(blank,swing=True)
    doc['joints'][1]['type']='revolute'
    doc['joints'][1]['limits']={'angle_deg':[-30,30]}
    if reverse_order: doc['joints'].reverse()
    result=simulate(factory(doc),.8,20)
    assert result['frames'][-1]['joints']['bearing-1']['angle_deg']==pytest.approx(30,abs=.2)


def test_loop_remains_connected_across_articulation_partitions(factory,blank,monkeypatch):
    monkeypatch.setattr('pipesim.physics.MAX_ARTICULATION_JOINTS',2)
    result=simulate(factory(carriage(blank)),.2,20)
    assert result['frames'][-1]['parts']['carriage']['position_mm']==pytest.approx([0,0,2803.8],abs=5)


@pytest.mark.parametrize('reverse_order',[False,True])
def test_real_two_socket_carriage_and_disengagement(load,factory,reverse_order):
    doc=copy.deepcopy(load('sliding-collar').doc)
    doc.pop('animation',None);doc['environment']['ground']=False
    doc['parts']=[p for p in doc['parts'] if p['id']!='stop']
    doc['joints']=[j for j in doc['joints'] if j['id']=='loose-screw']
    doc['joints'][0].pop('limits')
    for part in copy.deepcopy(doc['parts']):
        part['id']+='-2';part['pose']['position_mm'][0]+=800;doc['parts'].append(part)
    second=copy.deepcopy(doc['joints'][0]);second['id']='second-bearing'
    for end in ('a','b'): second[end]['part']+='-2'
    doc['joints'].extend([second,{'id':'frame','type':'fixed','a':{'part':'guide'},'b':{'part':'guide-2'}},
                         {'id':'carriage','type':'fixed','a':{'part':'slider'},'b':{'part':'slider-2'}}])
    if reverse_order: doc['joints'].reverse()
    result=simulate(factory(doc),1,20)
    at_point_two=result['frames'][4]['parts']
    for pid in ('slider','slider-2'):
        assert at_point_two[pid]['position_mm'][2]==pytest.approx(1103.8,abs=5)
        assert result['frames'][-1]['parts'][pid]['position_mm'][2]<-3000
    assert {e['joint'] for e in result['events'] if e['type']=='socket_disengaged'}=={'loose-screw','second-bearing'}
