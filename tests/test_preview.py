"""Preparation must save work without weakening joint checks or caching edits."""
import copy
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path

import numpy as np
import pytest

from pipesim.document import Assembly, DocumentError, Library, write
from pipesim.math3d import pose_of, transform
from pipesim.posing import commit_transform, transform_part
from pipesim.preview import PreviewCache
from pipesim.server import Handler
from pipesim.snapping import _movement_coordinates
from test_server import editor, request


def body_doc(blank):
    return {**blank, 'objects':[{'id':'person','template':'human','parameters':{'pose':'seated'}}]}


def post(server, route, doc, **extra):
    return json.loads(request(server, '/api/'+route, {'document':doc, **extra})[1])


def target_for(assembly, selected='person/left_shin', distance=30):
    pose=pose_of(assembly.parts[selected].matrix); pose['position_mm'][1]+=distance
    return pose


def assert_poses(assembly, result):
    for pid, pose in result['poses'].items():
        assert np.allclose(assembly.parts[pid].matrix, transform(pose), atol=1e-6, rtol=0), pid


@pytest.mark.parametrize('name,selected', [('human-pull-up','person/left_shin'), ('hinged-triangle','upper-back'), ('chain','chain/link-50')])
def test_lightweight_preview_matches_commit_without_expansion_or_rebasing(load,factory,monkeypatch,name,selected):
    before=load(name)
    if name=='chain':
        before.doc['objects'][0]['layout_mode']='posable'; before=factory(before.doc)
    original=copy.deepcopy(before.doc); target=target_for(before,selected)
    prepared=PreviewCache().remember(before)
    expected=transform_part(before,selected,target)
    with monkeypatch.context() as patch:
        patch.setattr('pipesim.posing.commit_transform',lambda *args:pytest.fail('Preview rebuilt the document'))
        result=transform_part(before,selected,target,preview=True,prepared=prepared)
    assert 'document' not in result and 'scene' not in result
    assert result=={k:v for k,v in expected.items() if k!='document'}
    after=factory(commit_transform(before,result['poses']))
    assert_poses(after,result); _movement_coordinates(before,after)
    assert before.doc==original
    assert [o['id'] for o in after.doc.get('objects',[])]==[o['id'] for o in original.get('objects',[])]


def test_open_scene_prepares_first_drag_and_release_uses_exact_stored_solution(editor,blank,factory,monkeypatch):
    doc=body_doc(blank); before=factory(doc)
    post(editor,'resolve',doc)
    payload={'selected':'person/left_shin','target':target_for(before),'mode':'translate'}
    with monkeypatch.context() as patch:
        patch.setattr(Handler,'assembly',lambda *args:pytest.fail('Re-resolved the document during preview'))
        result=post(editor,'move',doc,**payload,preview=True,pose_only=True)
        second=post(editor,'move',doc,**{**payload,'target':target_for(before,distance=35)},seed=result['seed'],preview=True,pose_only=True)
    assert result['preview_id']!=second['preview_id']
    assert len(json.dumps(result))<2000 and 'document' not in result
    with monkeypatch.context() as patch:
        patch.setattr('pipesim.posing.transform_part',lambda *args,**kwargs:pytest.fail('Solved again instead of accepting the preview'))
        accepted=post(editor,'move',doc,**payload,preview_id=result['preview_id'])
    assert accepted['poses']==result['poses']
    after=factory(accepted['document']);assert_poses(after,result);_movement_coordinates(before,after)
    assert len(accepted['scene']['parts'])==19 and len(after.doc['objects'])==1
    assert 'components' not in doc['objects'][0]


@pytest.mark.parametrize('change',['lock','anchor','limit','length','definition','state','object_parameters','layout'])
def test_cached_movement_invalidates_on_design_edits(factory,blank,change):
    doc=body_doc(blank);doc['parts']=[{'id':'tube','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000}}]
    before=factory(doc);cache=PreviewCache();prepared=cache.remember(before)
    mechanism=prepared.mechanism('person/left_shin')
    assert prepared.mechanism('person/left_shin') is mechanism
    assert cache.prepared(copy.deepcopy(doc),before.base,lambda:pytest.fail('Cache miss')) is prepared
    doc=copy.deepcopy(doc)
    if change=='lock':
        doc['objects'][0]['parameters']['hold_pose']=True
        # An explicit joint lock changes the actual rigid graph.
        doc['parts'] += [{'id':'other','catalog':'tubeclamp.TC101C'}]
        doc['joints']=[{'id':'locked','type':'fixed','a':{'part':'tube'},'b':{'part':'other'}}]
    elif change=='anchor': doc['anchors']=[{'part':'person/left_shin','surface':'fixture'}]
    elif change=='limit':
        from pipesim.editing import expand_objects
        doc=expand_objects(before)
        next(j for j in doc['joints'] if j['id']=='person/left_knee')['limits']={'angle_deg':[-1,1]}
    elif change=='length': doc['parts'][0]['parameters']['length_mm']=500
    elif change=='definition': doc['definitions']={'tubeclamp.tube-C':copy.deepcopy(before.library.parts['tubeclamp.tube-C'])};doc['definitions']['tubeclamp.tube-C']['mass_kg']=99
    elif change=='state': doc['state']={'joints':{'person/left_knee':{'angle_deg':10}}}
    elif change=='object_parameters': doc['objects'][0]['parameters']['stature_mm']=1500
    else: doc['objects'][0]['layout_mode']='rigid'
    changed=cache.prepared(doc,before.base,lambda:factory(doc))
    assert changed is not prepared and changed.assembly.doc==doc
    new=transform_part(changed.assembly,'person/left_shin',target_for(before),preview=True,prepared=changed)
    direct=transform_part(factory(doc),'person/left_shin',target_for(before),preview=True)
    assert new==direct
    if change=='anchor': assert not new['moved'] and new['limited']


def test_library_and_mesh_file_changes_invalidate_prepared_snapshots(editor,blank,tmp_path,monkeypatch):
    definition={'kind':'rigid','geometry':[{'type':'box','size_mm':[10,10,10]}],'mass_kg':1}
    path=tmp_path/'parts.yaml'
    write(path,{'format':'pipesim-library/1','name':'Test','parts':{'custom.part':definition}})
    doc={**blank,'libraries':['parts.yaml'],'parts':[{'id':'piece','catalog':'custom.part'}]}
    handler=object.__new__(Handler);handler.server=editor
    build=lambda:handler.assembly(doc,tmp_path)
    first=editor.previews.prepared(doc,tmp_path,build)
    with monkeypatch.context() as patch:
        patch.setattr(Library,'load',lambda *args:pytest.fail('Unchanged library reparsed'))
        assert handler.assembly(doc,tmp_path).parts['piece'].mass==1
    definition['mass_kg']=2
    write(path,{'format':'pipesim-library/1','name':'Test','parts':{'custom.part':definition}})
    second=editor.previews.prepared(doc,tmp_path,build)
    assert second is not first and second.assembly.parts['piece'].mass==2
    path.unlink()
    with pytest.raises(FileNotFoundError): editor.previews.prepared(doc,tmp_path,build)
    mesh=tmp_path/'shape.stl';mesh.write_text('first')
    doc={**blank,'parts':[{'id':'piece','body':{'mass_kg':1,'geometry':[{'type':'mesh','file':'shape.stl'}]}}]}
    first=editor.previews.prepared(doc,tmp_path,build)
    mesh.write_text('changed mesh contents')
    second=editor.previews.prepared(doc,tmp_path,build)
    assert second is not first
    mesh.unlink()
    with pytest.raises(FileNotFoundError): editor.previews.prepared(doc,tmp_path,build)


def test_cache_keeps_tabs_directories_and_library_overrides_independent(editor,blank,tmp_path):
    doc={**blank,'parts':[{'id':'piece','catalog':'tubeclamp.TC101C'}]}
    post(editor,'resolve',doc)
    override=copy.deepcopy(doc)
    original=editor.previews.library(doc,tmp_path).parts['tubeclamp.TC101C']
    override['definitions']={'tubeclamp.TC101C':{**original,'mass_kg':123}}
    with ThreadPoolExecutor(max_workers=4) as pool:
        scenes=list(pool.map(lambda item:post(editor,'resolve',item),[doc,override,doc,override]))
    assert [s['parts'][0]['mass_kg'] for s in scenes]==[original['mass_kg'],123,original['mass_kg'],123]
    cache=editor.previews
    a=cache.prepared(doc,tmp_path,lambda:Assembly.from_doc(doc,tmp_path,cache.library(doc,tmp_path)))
    b=cache.prepared(doc,tmp_path/'elsewhere',lambda:Assembly.from_doc(doc,tmp_path/'elsewhere',cache.library(doc,tmp_path)))
    assert a is not b and a.assembly.base!=b.assembly.base


def test_preview_token_is_bound_to_inputs_and_eviction_falls_back_safely(editor,blank,factory):
    doc=body_doc(blank);before=factory(doc);payload={'selected':'person/left_shin','target':target_for(before),'mode':'translate'}
    result=post(editor,'move',doc,**payload,preview=True,pose_only=True)
    moved=post(editor,'move',doc,**{**payload,'target':target_for(before,distance=40)},preview_id=result['preview_id'])
    assert moved['position_error_mm']<.1 and moved['poses']!=result['poses']
    edited=copy.deepcopy(doc);edited['anchors']=[{'part':'person/left_shin','surface':'fixture'}]
    locked=post(editor,'move',edited,**payload,preview_id=result['preview_id'])
    assert not locked['moved'] and locked['document']==edited
    for i in range(12): post(editor,'resolve',{**blank,'name':f'Tab {i}'})
    assert len(editor.previews.assemblies)<=8
    accepted=post(editor,'move',doc,**payload,preview_id=result['preview_id'],seed=result['seed'])
    assert accepted['position_error_mm']<.1
    _movement_coordinates(before,factory(accepted['document']))


def test_concurrent_drag_targets_do_not_change_shared_joint_setup(editor,blank,factory):
    doc=body_doc(blank);before=factory(doc);post(editor,'resolve',doc)
    payloads=[{'selected':'person/left_shin','target':target_for(before,distance=distance),'mode':'translate'} for distance in (10,20,30,40)]
    with ThreadPoolExecutor(max_workers=4) as pool:
        results=list(pool.map(lambda payload:post(editor,'move',doc,**payload,preview=True,pose_only=True),payloads))
    assert len({r['preview_id'] for r in results})==4
    for payload,result in zip(payloads,results):
        assert result['position_error_mm']<.1
        accepted=post(editor,'move',doc,**payload,preview_id=result['preview_id'])
        assert accepted['poses']==result['poses']
        _movement_coordinates(before,factory(accepted['document']))
    assert doc==before.doc


def test_saved_joint_state_preview_rebases_animation_only_when_accepted(editor,blank,factory):
    doc=body_doc(blank);doc['state']={'joints':{'person/left_knee':{'angle_deg':10}}}
    doc['animation']={'tracks':[{'joint':'person/left_knee','coordinate':'angle_deg','keyframes':[{'time_s':0,'value':10},{'time_s':1,'value':20}]}]}
    before=factory(doc);payload={'selected':'person/left_shin','target':target_for(before),'mode':'translate'}
    result=post(editor,'move',doc,**payload,preview=True,pose_only=True)
    accepted=post(editor,'move',doc,**payload,preview_id=result['preview_id'])
    after=factory(accepted['document']);assert_poses(after,result)
    shift=_movement_coordinates(before,after)['person/left_knee']['angle_deg']
    assert [k['value'] for k in after.doc['animation']['tracks'][0]['keyframes']]==pytest.approx([-shift,10-shift])
    assert 'state' not in after.doc and 'state' in doc


def test_tightening_a_previewed_socket_or_reducing_its_travel_never_reuses_the_loose_pose(editor,load,factory):
    before=load('sliding-collar');doc=before.doc
    target=pose_of(before.parts['slider'].matrix);target['position_mm'][2]+=100
    payload={'selected':'slider','target':target,'mode':'translate'}
    loose=post(editor,'move',doc,**payload,preview=True,pose_only=True)
    assert loose['moved']==['slider'] and not loose['limited']
    locked=copy.deepcopy(doc);locked['joints'][1]['locked']=True
    stopped=post(editor,'move',locked,**payload,preview_id=loose['preview_id'])
    assert not stopped['moved'] and stopped['document']==locked
    limited=copy.deepcopy(doc);limited['joints'][1]['limits']['slide_mm']=[-10,10]
    stopped=post(editor,'move',limited,**payload,preview_id=loose['preview_id'])
    assert stopped['limited']
    assert abs(stopped['poses']['slider']['position_mm'][2]-1300)==pytest.approx(10,abs=1e-5)
    _movement_coordinates(factory(limited),factory(stopped['document']))


@pytest.mark.parametrize('saved',[False,True])
def test_whole_object_previews_and_legacy_tabs_keep_their_contract(editor,blank,factory,saved):
    doc=body_doc(blank)
    if saved: doc['state']={'joints':{'person/left_knee':{'angle_deg':10}}}
    target={'position_mm':[3000,100,500],'rotation_deg':[0,0,180]}
    result=post(editor,'move-object',doc,object='person',target=target,preview=True,pose_only=True)
    assert 'document' not in result and len(result['poses'])==19
    accepted=post(editor,'move-object',doc,object='person',target=target)
    assert_poses(factory(accepted['document']),result)
    legacy=post(editor,'move',doc,selected='person/left_shin',target=target_for(factory(doc)),preview=True)
    assert 'document' in legacy and 'scene' not in legacy
    anchored={**doc,'anchors':[{'part':'person/pelvis','surface':'fixture'}]}
    import urllib.error
    with pytest.raises(urllib.error.HTTPError): post(editor,'move-object',anchored,object='person',target=target,preview=True,pose_only=True)
