import copy

import numpy as np
import pytest

from pipesim.document import DocumentError
from pipesim.port_connections import connect_ports
from pipesim.validation import validate


def hinge_pair(blank):
    doc = copy.deepcopy(blank)
    doc['parts'] = [
        {'id': 'male', 'catalog': 'tubeclamp.TC173MC',
         'pose': {'position_mm': [0, 0, 1000]}},
        {'id': 'female', 'catalog': 'tubeclamp.TC173FC',
         'pose': {'position_mm': [80, 0, 900], 'rotation_deg': [0, 0, 90]}},
        {'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
         'parameters': {'length_mm': 400},
         'pose': {'position_mm': [80, 0, 500]}}]
    doc['joints'] = [{'id': 'female-pipe', 'type': 'fixed',
                      'a': {'part': 'female', 'frame': {'position_mm': [0, 0, 0]}},
                      'b': {'part': 'pipe', 'frame': {'position_mm': [0, 0, 400]}}}]
    doc['anchors'] = [{'part': 'male', 'surface': 'fixture'}]
    return doc


def test_dragged_hinge_ports_snap_exactly_and_carry_attached_pipe(factory, blank):
    before = factory(hinge_pair(blank))
    a = {'part': 'male', 'port': 'hinge'}
    b = {'part': 'female', 'port': 'hinge'}
    result = connect_ports(before, a, b)
    after = factory(result['document'])
    assert result['move'] == 'b'
    assert result['gap_mm'] > 30
    assert result['joint']['type'] == 'revolute'
    pa, pb, axis = after.joint_frames(result['joint'])
    assert np.linalg.norm(pa-pb) < 1e-6
    assert abs(axis @ after.parts['female'].frame(b)[1]) > 1-1e-6
    assert np.allclose(after.parts['male'].matrix, before.parts['male'].matrix)
    assert not np.allclose(after.parts['pipe'].matrix, before.parts['pipe'].matrix)
    assert np.allclose(np.linalg.inv(after.parts['female'].matrix)
                       @ after.parts['pipe'].matrix,
                       np.linalg.inv(before.parts['female'].matrix)
                       @ before.parts['pipe'].matrix)


def test_hinge_port_cannot_be_claimed_twice(factory, blank):
    before = factory(hinge_pair(blank))
    a, b = ({'part': pid, 'port': 'hinge'} for pid in ('male', 'female'))
    after = factory(connect_ports(before, a, b)['document'])
    with pytest.raises(DocumentError, match='already connected'):
        connect_ports(after, a, b)


def test_male_half_moves_when_female_rigid_body_is_anchored(factory, blank):
    doc = hinge_pair(blank)
    doc['anchors'] = [{'part': 'female', 'surface': 'fixture'}]
    before = factory(doc)
    a, b = ({'part': pid, 'port': 'hinge'} for pid in ('male', 'female'))
    result = connect_ports(before, a, b)
    after = factory(result['document'])
    assert result['move'] == 'a'
    assert np.allclose(after.parts['female'].matrix, before.parts['female'].matrix)
    assert np.allclose(after.parts['pipe'].matrix, before.parts['pipe'].matrix)
    assert not np.allclose(after.parts['male'].matrix, before.parts['male'].matrix)
    pa, pb, _ = after.joint_frames(result['joint'])
    assert np.linalg.norm(pa-pb) < 1e-6


def test_hinge_bolt_holes_must_have_compatible_diameters(factory, blank):
    doc = hinge_pair(blank)
    doc['parts'][0]['catalog'] = 'tubeclamp.TC173MA'
    before = factory(doc)
    a, b = ({'part': pid, 'port': 'hinge'} for pid in ('male', 'female'))
    with pytest.raises(DocumentError, match='diameters do not match'):
        connect_ports(before, a, b)


def test_two_hinge_eyes_do_not_form_a_hinge(factory, blank):
    doc = hinge_pair(blank)
    doc['parts'][1]['catalog'] = 'tubeclamp.TC173MC'
    before = factory(doc)
    a, b = ({'part': pid, 'port': 'hinge'} for pid in ('male', 'female'))
    with pytest.raises(DocumentError, match='one bolt eye and one matching clevis'):
        connect_ports(before, a, b)


def test_aligned_hinge_pair_validates_as_an_assembly(factory, blank):
    doc = hinge_pair(blank)
    doc['parts'] = doc['parts'][:2]
    doc['joints'] = []
    before = factory(doc)
    a, b = ({'part': pid, 'port': 'hinge'} for pid in ('male', 'female'))
    result = connect_ports(before, a, b)
    report = validate(factory(result['document']))
    assert report['valid'], report['issues']


@pytest.mark.parametrize('kind',['fixed','revolute','spherical'])
@pytest.mark.parametrize('reverse',[False,True])
@pytest.mark.parametrize('port',[None,'bolt1'])
def test_abstract_load_attaches_without_port_and_preserves_both_poses(factory,blank,kind,reverse,port):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'host','catalog':'tubeclamp.TC132C',
                   'pose':{'position_mm':[300,-200,1000],'rotation_deg':[20,35,-15]}},
                  {'id':'load','catalog':'generic.box','parameters':{'mass_kg':37},
                   'pose':{'position_mm':[450,-90,1300],'rotation_deg':[-35,15,75]}}]
    doc['anchors']=[{'part':'host','surface':'fixture'}]
    doc['results']={'simulate':{'frames':[]}}
    before=factory(doc)
    host={'part':'host',**({'port':port} if port else {})};load={'part':'load'}
    a,b=(load,host) if reverse else (host,load)
    preview=connect_ports(before,a,b,kind,preview=True)
    result=connect_ports(before,a,b,kind)
    assert preview==result
    assert result['abstract_load'] and result['move'] is None and result['moved']==[]
    assert result['joint']['type']==kind
    assert 'port' not in result['joint']['a'] and 'port' not in result['joint']['b']
    after=factory(result['document'])
    pa,pb,axis=after.joint_frames(result['joint'])
    assert np.allclose(pa,pb) and np.allclose(pa,before.parts['host'].frame(host)[0])
    assert all(np.array_equal(after.parts[pid].matrix,before.parts[pid].matrix) for pid in before.parts)
    assert after.parts['load'].mass==37
    assert 'results' not in result['document'] and 'results' in before.doc
    assert validate(after)['valid']


def test_abstract_load_does_not_occupy_a_physical_connector_port(factory,blank):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'host','catalog':'tubeclamp.TC132C'},
                  {'id':'load','catalog':'generic.box','pose':{'position_mm':[0,0,250]}}]
    first=connect_ports(factory(doc),{'part':'host','port':'bolt1'},{'part':'load'},'fixed')
    doc=first['document']
    doc['parts'].append({'id':'other-load','catalog':'generic.box','pose':{'position_mm':[0,300,250]}})
    second=connect_ports(factory(doc),{'part':'host','port':'bolt1'},{'part':'other-load'},'fixed')
    assert len(second['document']['joints'])==2 and second['moved']==[]


def test_fixed_abstract_load_is_supported_in_physics_instead_of_falling(factory,blank):
    from pipesim.physics import World
    doc=copy.deepcopy(blank)
    doc['environment']={'ground':False}
    doc['parts']=[{'id':'host','catalog':'tubeclamp.TC132C','pose':{'position_mm':[0,0,1000]}},
                  {'id':'load','catalog':'generic.box','parameters':{'mass_kg':37},
                   'pose':{'position_mm':[120,80,1300]}}]
    doc['anchors']=[{'part':'host','surface':'fixture'}]
    free=factory(doc)
    attached=factory(connect_ports(free,{'part':'host'},{'part':'load'},'fixed')['document'])
    with World(attached) as world:
        for _ in range(30): world.step()
        assert np.allclose(world.part_matrix('load'),free.parts['load'].matrix,atol=1e-5)
    with World(free) as world:
        for _ in range(30): world.step()
        assert world.part_matrix('load')[2,3]<free.parts['load'].matrix[2,3]-10


def test_abstract_load_display_can_overlap_its_host_without_hiding_other_collisions(factory,blank):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'host','catalog':'tubeclamp.TC132C'},
                  {'id':'load','catalog':'generic.box'}]
    doc['anchors']=[{'part':'host','surface':'fixture'}]
    attached=connect_ports(factory(doc),{'part':'host'},{'part':'load'},'fixed')['document']
    assert validate(factory(attached))['valid']
    attached['parts'].append({'id':'obstacle','body':{'kind':'rigid','mass_kg':1,
        'geometry':[{'type':'box','size_mm':[200,200,200]}]}})
    report=validate(factory(attached))
    assert any(issue['code']=='INTERSECTION' and 'obstacle' in issue['parts'] for issue in report['issues'])
