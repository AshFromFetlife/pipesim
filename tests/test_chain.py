import copy

import numpy as np
import pytest

from pipesim.chain import dimensions
from pipesim.document import Assembly, DocumentError, write
from pipesim.editing import expand_objects
from pipesim.grouping import (attach_part, detach_attachment, move_object, regroup_object,
                              set_object_layout, update_object_parameters)
from pipesim.math3d import pose_of, transform
from pipesim.posing import Mechanism, transform_part
from pipesim.snapping import _movement_coordinates
from pipesim.validation import validate


def chain(factory, blank, count=25, mode='rigid'):
    doc=copy.deepcopy(blank)
    doc['objects']=[{'id':'chain','template':'chain','parameters':{'length_mm':count*20},
                     'layout_mode':mode,'pose':{'position_mm':[0,0,count*20+200]}}]
    return factory(doc)


def aligned(assembly):
    for j in assembly.joints:
        a,b,_=assembly.joint_frames(j)
        assert np.linalg.norm(a-b)<.05,j['id']


def test_chain_length_rounds_to_whole_library_links_and_stays_compact(factory,blank,library,tmp_path):
    before=chain(factory,blank)
    doc=update_object_parameters(before,'chain',{'length_mm':103})
    after=factory(doc);info=dimensions(doc['objects'][0]['parameters'],library)
    assert info['count']==6 and info['length_mm']==120
    assert len(after.parts)==6 and len(after.joints)==5
    assert not doc['parts'] and not doc['joints'] and 'components' not in doc['objects'][0]
    aligned(after)
    assert abs(after.parts['chain/link-1'].matrix[:3,1]@after.parts['chain/link-2'].matrix[:3,1])<1e-7
    path=tmp_path/'chain.pipe.yaml';write(path,doc);loaded=Assembly.load(path)
    assert loaded.doc==doc and len(loaded.parts)==6
    assert validate(after)['valid']
    assert not [i for i in validate(after)['issues'] if i['code']=='INTERSECTION']


@pytest.mark.parametrize('length',[0,-10,True,'500',float('inf'),20001])
def test_invalid_chain_lengths_do_not_edit_the_design(factory,blank,length):
    before=chain(factory,blank);original=copy.deepcopy(before.doc)
    with pytest.raises(DocumentError,match='length'):
        update_object_parameters(before,'chain',{'length_mm':length})
    assert before.doc==original


def test_long_chain_is_one_layout_body_and_has_no_internal_solver_variables(factory,blank):
    before=chain(factory,blank,200)
    assert len(before.editor_groups())==1 and len(before.rigid_groups())==200
    assert len(Mechanism(before,'chain/link-200').lower)==0
    moved=move_object(before,'chain',{'position_mm':[3000,-1000,4500],'rotation_deg':[0,180,90]})
    after=factory(moved['document']);aligned(after)
    assert 'components' not in after.doc['objects'][0]
    _movement_coordinates(before,after)
    delta=after.parts['chain/link-1'].matrix@np.linalg.inv(before.parts['chain/link-1'].matrix)
    for pid in before.parts:
        assert np.allclose(after.parts[pid].matrix,delta@before.parts[pid].matrix,atol=1e-7,rtol=0)


@pytest.mark.parametrize('count',[25,200])
def test_long_chain_poses_without_dense_general_ik_and_preserves_joint_frames(factory,blank,count,monkeypatch):
    before=chain(factory,blank,count,'posable');target=pose_of(before.parts[f'chain/link-{count}'].matrix)
    target['position_mm'][0]+=30;target['position_mm'][2]+=30
    def unexpected(*args,**kwargs): raise AssertionError('Chain posing constructed the dense mechanism solver')
    monkeypatch.setattr(Mechanism,'__init__',unexpected)
    result=transform_part(before,f'chain/link-{count}',target)
    assert result['position_error_mm']<.05
    after=factory(result['document']);aligned(after);_movement_coordinates(before,after)
    assert len(after.doc['objects'])==1 and not after.doc['parts']
    frozen=factory(set_object_layout(after,'chain','rigid'))
    assert len(frozen.editor_groups())==1 and len(frozen.rigid_groups())==count
    for pid in before.parts:
        assert np.allclose(frozen.parts[pid].matrix,after.parts[pid].matrix,atol=1e-7,rtol=0)


def test_posed_chain_grows_and_shrinks_at_its_end_without_losing_the_retained_shape(factory,blank):
    before=chain(factory,blank,12,'posable');target=pose_of(before.parts['chain/link-12'].matrix)
    target['position_mm'][0]+=20;target['position_mm'][2]+=30
    posed=factory(transform_part(before,'chain/link-12',target)['document'])
    grown=factory(update_object_parameters(posed,'chain',{'length_mm':340}))
    assert len(grown.parts)==17;aligned(grown)
    for pid in posed.parts:
        assert np.allclose(posed.parts[pid].matrix,grown.parts[pid].matrix,atol=1e-7,rtol=0)
    trimmed=factory(update_object_parameters(grown,'chain',{'length_mm':160}))
    assert len(trimmed.parts)==8;aligned(trimmed)
    for pid in trimmed.parts:
        assert np.allclose(grown.parts[pid].matrix,trimmed.parts[pid].matrix,atol=1e-7,rtol=0)


def test_an_already_bent_long_chain_follows_further_drag_targets(factory,blank):
    before=chain(factory,blank,200,'posable')
    for offset in ([30,0,30],[5,5,5],[-10,5,10]):
        target=pose_of(before.parts['chain/link-200'].matrix)
        target['position_mm']=(np.array(target['position_mm'])+offset).tolist()
        result=transform_part(before,'chain/link-200',target)
        assert result['position_error_mm']<.1
        after=factory(result['document']);aligned(after);_movement_coordinates(before,after)
        before=after


def test_rotating_a_chain_link_keeps_its_pivot_and_brings_the_free_tail(factory,blank):
    from scipy.spatial.transform import Rotation
    before=chain(factory,blank,25,'posable');target=before.parts['chain/link-12'].matrix.copy()
    target[:3,:3]=Rotation.from_euler('y',20,degrees=True).as_matrix()@target[:3,:3]
    result=transform_part(before,'chain/link-12',pose_of(target),'rotate');after=factory(result['document'])
    assert result['angle_error_deg']<.01
    aligned(after);_movement_coordinates(before,after)
    assert np.allclose(before.parts['chain/link-12'].frame({'port':'b'})[0],after.parts['chain/link-12'].frame({'port':'b'})[0],atol=1e-6,rtol=0)
    assert 'chain/link-25' in result['moved']


def test_duplicate_subassembly_preserves_chain_length_controls(factory,blank):
    from pipesim.duplication import duplicate
    before=chain(factory,blank,8)
    copied=factory(duplicate(before,'chain/link-8','subassembly')['document'])
    assert len(copied.doc['objects'])==2 and not copied.doc['parts']
    after=factory(update_object_parameters(copied,'chain-copy',{'length_mm':200}))
    assert len(after.parts)==18;aligned(after)
    for pid in before.parts:
        assert np.allclose(before.parts[pid].matrix,after.parts[pid].matrix,atol=1e-7,rtol=0)


def test_length_edit_bakes_saved_state_and_removes_only_trimmed_internal_references(factory,blank):
    before=chain(factory,blank,12);doc=before.doc
    doc['state']={'joints':{'chain/join-3':{'rotation_deg':[15,0,0]},'chain/join-10':{'rotation_deg':[10,0,0]}}}
    doc['animation']={'tracks':[{'joint':j,'coordinate':'angle_deg','keyframes':[{'time_s':0,'value':0}]} for j in ('chain/join-3','chain/join-10')]}
    before=factory(doc);after=factory(update_object_parameters(before,'chain',{'length_mm':160}))
    assert len(after.parts)==8 and 'state' not in after.doc
    assert [t['joint'] for t in after.doc['animation']['tracks']]==['chain/join-3']
    for pid in after.parts:
        assert np.allclose(after.parts[pid].matrix,before.parts[pid].matrix,atol=1e-7,rtol=0)
    aligned(after)


def test_shortening_identifies_the_attached_link_and_never_drops_an_anchor(factory,blank):
    before=chain(factory,blank,12);before.doc['anchors']=[{'part':'chain/link-12','surface':'ceiling'}]
    before=factory(before.doc);original=copy.deepcopy(before.doc)
    with pytest.raises(DocumentError,match='world anchor on chain/link-12'):
        update_object_parameters(before,'chain',{'length_mm':160})
    assert before.doc==original


def test_expansion_and_regrouping_preserve_length_controls_and_saved_link_edits(factory,blank):
    before=chain(factory,blank,8);expanded=expand_objects(before,'chain')
    expanded['parts'][2]['color']='#ff8800'
    grouped=factory(regroup_object(factory(expanded),'chain'))
    after=factory(update_object_parameters(grouped,'chain',{'length_mm':220}))
    assert after.parts['chain/link-3'].spec['color']=='#ff8800'
    assert len(after.parts)==11;aligned(after)


def test_chain_uses_custom_library_pitch(factory,blank):
    doc=copy.deepcopy(blank);doc['definitions']={'custom.link':{'kind':'chain','mass_kg':.02,
        'geometry':[{'type':'tube','diameter_mm':40,'length_mm':5,'wall_mm':5,'axis':[0,1,0]}],
        'ports':{'a':{'position_mm':[0,0,-15]},'b':{'position_mm':[0,0,15]}}}}
    doc['objects']=[{'id':'chain','template':'chain','parameters':{'link_catalog':'custom.link','length_mm':121}}]
    result=factory(doc);assert len(result.parts)==5;aligned(result)


def test_regrouped_portable_chain_uses_its_bundled_link_definition(factory,blank,tmp_path):
    from pipesim.packaging import bundle
    before=chain(factory,blank,8)
    before.doc['definitions']={'custom.link':copy.deepcopy(before.library.parts['generic.chain-link'])}
    before.doc['definitions']['custom.link']['ports']['a']['position_mm'][2]=-15
    before.doc['definitions']['custom.link']['ports']['b']['position_mm'][2]=15
    before.doc['objects'][0]['parameters']={'link_catalog':'custom.link','length_mm':240}
    path=bundle(factory(before.doc),tmp_path/'portable')
    expanded=Assembly.load(path)
    grouped=Assembly.from_doc(regroup_object(expanded,'chain'),path.parent)
    params=copy.deepcopy(grouped.doc['objects'][0]['parameters']);params['length_mm']=301
    after=Assembly.from_doc(update_object_parameters(grouped,'chain',params),path.parent)
    assert len(after.parts)==11;aligned(after)
    assert dimensions(params,after.library)['length_mm']==330


def test_chain_physics_flexes_even_when_its_editor_layout_is_rigid(factory,blank):
    from pipesim.physics import simulate
    before=chain(factory,blank,8);before.doc['objects'][0]['pose']['rotation_deg']=[0,40,0]
    before.doc['anchors']=[{'part':'chain/link-1','surface':'ceiling'}];before=factory(before.doc)
    result=simulate(before,.3,20)
    start,end=result['frames'][0]['parts'],result['frames'][-1]['parts']
    assert np.allclose(transform(start['chain/link-1']),transform(end['chain/link-1']),atol=1e-6,rtol=0)
    assert np.linalg.norm(np.array(start['chain/link-8']['position_mm'])-end['chain/link-8']['position_mm'])>10
    assert len(before.editor_groups())==1 and len(before.joints)==7


def test_chain_end_attachments_pose_the_chain_and_detach_reconnect(factory,blank):
    before=chain(factory,blank,25)
    before.doc['parts']=[{'id':'hook-start','catalog':'generic.d-link','pose':{'position_mm':[0,0,800]}},
                         {'id':'hook-end','catalog':'generic.d-link','pose':{'position_mm':[250,0,500]}}]
    before.doc['anchors']=[{'part':p,'surface':'fixture'} for p in ('hook-start','hook-end')]
    before=factory(before.doc)
    first=factory(attach_part(before,'chain','chain/link-1',{'part':'hook-start','port':'eye'},'spherical')['document'])
    assert np.linalg.norm(first.parts['chain/link-1'].frame({'port':'b'})[0]-[0,0,800])<.05
    second=attach_part(first,'chain','chain/link-25',{'part':'hook-end','port':'eye'},'spherical')
    after=factory(second['document']);aligned(after)
    assert after.doc['objects'][0]['layout_mode']=='posable'
    assert np.linalg.norm(after.parts['chain/link-25'].frame({'port':'a'})[0]-[250,0,500])<.05
    with pytest.raises(DocumentError,match='attachment'):
        move_object(after,'chain',{'position_mm':[3000,0,800]})
    with pytest.raises(DocumentError,match=second['joint']['id']):
        update_object_parameters(after,'chain',{'length_mm':100})
    detached=factory(detach_attachment(after,'chain',second['joint']['id']))
    restored=factory(attach_part(detached,'chain',reconnect=second['joint']['id'])['document'])
    aligned(restored);assert len(restored.joints)==len(after.joints)
