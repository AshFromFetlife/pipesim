import copy
import pytest
from pipesim.validation import validate,support
from pipesim.planning import plan_build
from pipesim.document import DocumentError

def codes(assembly,collisions=False): return {i['code'] for i in validate(assembly,collisions)['issues'] if i['severity']=='error'}
@pytest.mark.parametrize('name',['workbench','cantilever','sliding-collar','gt2-stage','human','seated-human'])
def test_positive_example_geometry(load,name): assert not codes(load(name))
def test_workbench_collision_free(load): assert validate(load('workbench'))['valid']
def test_two_pipes_in_one_socket(load,factory):
    doc=load('workbench').doc; part=copy.deepcopy(doc['parts'][1]); part['id']='duplicate-leg'; doc['parts'].append(part)
    joint=copy.deepcopy(doc['joints'][0]); joint['id']='duplicate-joint'; joint['b']['part']=part['id']; doc['joints'].append(joint)
    assert 'SOCKET_OCCUPIED' in codes(factory(doc))
def test_wrong_diameter(load,factory):
    doc=load('workbench').doc; doc['parts'][1]['catalog']='tubeclamp.tube-D'; assert 'PROFILE_MISMATCH' in codes(factory(doc))
def test_member_cannot_reach(load,factory):
    doc=load('workbench').doc; doc['parts'][1]['parameters']['length_mm']=300; assert 'UNREACHABLE_JOINT' in codes(factory(doc))
def test_axis_misalignment(load,factory):
    doc=load('workbench').doc; doc['parts'][1]['pose']['rotation_deg']=[20,0,0]; assert 'AXIS_MISMATCH' in codes(factory(doc))
@pytest.mark.parametrize('insertion,code',[(1,'ENGAGEMENT_SHORT'),(100,'SOCKET_BOTTOMED')])
def test_insertion_depth(load,factory,insertion,code):
    doc=load('workbench').doc; doc['joints'][0]['insertion_mm']=insertion; assert code in codes(factory(doc))
def test_through_socket_needs_full_engagement(load,factory):
    doc=load('sliding-collar').doc; doc['joints'][1]['b']['at_mm']=1; assert 'THROUGH_ENGAGEMENT' in codes(factory(doc))
def test_station_outside_member(load,factory):
    doc=load('sliding-collar').doc; doc['joints'][1]['b']['at_mm']=3000; assert 'STATION_OUTSIDE_MEMBER' in codes(factory(doc))
def test_excessive_wall(load,factory):
    doc=load('cantilever').doc; doc['parts'][0]['parameters']['wall_mm']=30; assert 'WALL' in codes(factory(doc))
def test_self_intersection_same_body(blank,factory):
    blank['parts']=[{'id':id,'catalog':'generic.box','parameters':{'width_mm':100,'depth_mm':100,'height_mm':100,'mass_kg':5},'pose':{'position_mm':[x,0,50]}} for id,x in [('a',0),('b',40)]]
    blank['joints']=[{'id':'fixed','type':'fixed','a':{'part':'a','frame':{'position_mm':[20,0,0]}},'b':{'part':'b','frame':{'position_mm':[-20,0,0]}}}]
    assert 'INTERSECTION' in codes(factory(blank),True)
def test_tipped_load_outside_support(blank,factory):
    blank['parts']=[{'id':'post','catalog':'generic.box','parameters':{'width_mm':20,'depth_mm':20,'height_mm':100,'mass_kg':1},'pose':{'position_mm':[0,0,50]}},{'id':'load','catalog':'generic.box','parameters':{'width_mm':20,'depth_mm':20,'height_mm':20,'mass_kg':100},'pose':{'position_mm':[150,0,100]}}]
    blank['joints']=[{'id':'weld','type':'fixed','a':{'part':'post','frame':{'position_mm':[150,0,50]}},'b':{'part':'load'}}]
    assert not support(factory(blank))['stable']
def test_ceiling_anchor_is_support(blank,factory):
    blank['parts']=[{'id':'box','catalog':'generic.box','pose':{'position_mm':[0,0,2000]}}];blank['anchors']=[{'part':'box','surface':'ceiling'}]
    assert support(factory(blank))['stable']
def test_lock_changes_inferred_rigid_bodies(load,factory):
    a=load('sliding-collar');assert len(a.rigid_groups())==2
    a.doc['joints'][1]['locked']=True;assert len(factory(a.doc).rigid_groups())==1
def test_state_moves_unanchored_side(load,factory):
    doc=load('sliding-collar').doc;doc['state']={'joints':{'loose-screw':{'slide_mm':200}}}
    a=factory(doc);assert a.parts['slider'].matrix[2,3]==pytest.approx(1100);assert a.parts['guide'].matrix[2,3]==1000;assert not codes(a)
def test_state_outside_limit(load,factory):
    doc=load('sliding-collar').doc;doc['state']={'joints':{'loose-screw':{'slide_mm':2000}}};assert 'STATE_OUTSIDE_LIMIT' in codes(factory(doc))
def test_locked_joint_cannot_move(load,factory):
    doc=load('sliding-collar').doc;doc['state']={'joints':{'stop-screw':{'slide_mm':20}}}
    with pytest.raises(DocumentError,match='locked'):factory(doc)
def collar_trap(blank):
    blank['parts']=[{'id':'rail','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},'pose':{'position_mm':[0,0,500]}}]+[{'id':id,'catalog':'tubeclamp.TC179C','pose':{'position_mm':[0,0,z]}} for id,z in [('bottom',40),('top',950),('middle',500)]]
    blank['anchors']=[{'part':'rail','surface':'fixture'}]
    blank['joints']=[{'id':id+'-screw','type':'socket','a':{'part':id,'port':'through'},'b':{'part':'rail','at_mm':z},'locked':True} for id,z in [('bottom',40),('top',950),('middle',500)]]
    return blank
def test_closed_collar_cannot_be_added_between_stops(blank,factory):
    doc=collar_trap(blank);doc['build']={'sequence':['rail','bottom','top','middle']}
    result=plan_build(factory(doc));assert result['status']=='blocked';assert 'middle' in result['reason']
def test_planner_reorders_collar_before_stop(blank,factory):
    result=plan_build(factory(collar_trap(blank)));assert result['status']=='buildable'
    sequence=[s['part'] for s in result['steps']];assert sequence.index('middle')<sequence.index('top')
def test_split_clamp_can_be_added_radially(blank,factory):
    doc=collar_trap(blank);doc['parts'][-1]['catalog']='tubeclamp.TC136C';doc['build']={'sequence':['rail','bottom','top','middle']}
    assert plan_build(factory(doc))['status']=='buildable'
def test_false_radial_override_rejected(blank,factory):
    doc=collar_trap(blank);doc['joints'][-1]['assembly']='radial';assert 'CLOSED_SOCKET_RADIAL' in codes(factory(doc))
def test_workbench_buildable(load):
    result=plan_build(load('workbench'));assert result['status']=='buildable';assert len(result['steps'])==13
    assert result['steps'][-1]['part']=='worktop';assert result['certificate']['stability_checked']
def test_search_exhaustion_is_not_impossibility(load):
    result=plan_build(load('workbench'),search_limit=1);assert result['status']=='indeterminate'
def test_unsupported_final_is_not_buildable(blank,factory):
    blank['parts']=[{'id':'box','catalog':'generic.box','pose':{'position_mm':[0,0,1000]}}];assert plan_build(factory(blank))['status']=='blocked'
