import copy
import math
import time

import numpy as np
import pytest

from pipesim.bom_editing import analyse,normalize,BomPreviews
from pipesim.document import Assembly,DocumentError
from pipesim.connection_jobs import ConnectionCancelled
from pipesim.validation import validate
from pipesim.dimensions import rounded_dimension
from bom_cases import square,rail_stations,check_batch,naive_rounding
from planning_cases import FIXTURES,fixture


def test_bom_groups_exact_lengths_by_profile_with_bounded_spread(factory,monkeypatch):
    def no_plan(*a,**k):raise AssertionError('BOM must not find assembly order')
    monkeypatch.setattr('pipesim.planning.plan_build',no_plan)
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[
        {'id':str(i),'catalog':'tubeclamp.tube-C','parameters':{'length_mm':length},'pose':{'position_mm':[i*100,0,100]}}
        for i,length in enumerate([174.885476,174.886027,174.887322,175.8,176.6])],'joints':[]}
    doc['parts'].append({'id':'other-profile','catalog':'tubeclamp.tube-C','parameters':{'length_mm':175,'wall_mm':2}})
    assembly=factory(doc);original=copy.deepcopy(assembly.doc)
    result=analyse(assembly,1)
    assert assembly.doc==original
    assert all(g['maximum_mm']-g['minimum_mm']<=1 for g in result['length_groups'])
    assert len(result['length_groups'][0]['parts'])==4
    assert len(result['length_groups'])==3
    assert sum(i['quantity'] for i in result['bom']['items'])==6


@pytest.mark.parametrize('seed',range(6))
@pytest.mark.parametrize('anchored',[False,True])
def test_closed_frame_lengths_and_angles_fit_in_one_batch(factory,seed,anchored):
    doc,targets=square(seed,anchored)
    result=check_batch(factory,doc,targets)
    again=normalize(factory(result['document']),targets)
    assert again['status']=='unchanged'
    assert again['document']==result['document']


@pytest.mark.parametrize('seed',range(6))
def test_through_stations_move_connected_tuples_together(factory,seed):
    check_batch(factory,*rail_stations(seed))


def test_preview_cache_requires_current_source_and_copies_cached_result(factory):
    doc,targets=square();source=factory(doc);result=normalize(source,targets)
    cache=BomPreviews();cache.remember('checked',result)
    result['document']['parts'].clear()
    applied=cache.apply(source,'checked');assert applied['status']=='applied' and applied['document']['parts']
    doc['name']='Edited after preview'
    assert cache.apply(factory(doc),'checked')['status']=='stale'
    assert cache.apply(source,'expired')['status']=='stale'


def test_no_targets_preserves_caches_and_exact_document(factory):
    doc,_=square();doc['build_plan']={'cached':True}
    assert normalize(factory(doc),{})['document']==doc


def test_retained_anchor_goal_does_not_invalidate_caches_or_offer_an_empty_apply(factory):
    doc,_=square(0,True);doc['build_plan']={'cached':True}
    result=normalize(factory(doc),{'angles':{'corner-0':[0,0,15]}})
    assert result['status']=='unchanged' and result['document']==doc
    assert result['retained_targets']==['corner-0']


def test_cancelled_batch_never_commits(factory):
    doc,targets=square();assembly=factory(doc);original=copy.deepcopy(assembly.doc)
    calls=0
    def cancelled():
        nonlocal calls
        calls+=1;return calls>25
    with pytest.raises(ConnectionCancelled):normalize(assembly,targets,cancelled)
    assert assembly.doc==original


def test_draft_bom_is_available_without_finalization(factory):
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[],'draft_subassemblies':[
        {'id':'draft','runs':[{'id':'pipe','catalog':'tubeclamp.tube-C','start_mm':[0,0,0],'end_mm':[0,0,174.9]}]}]}
    result=analyse(factory(doc))
    assert result['estimated'] and result['bom']['items'][0]['length_mm']==175
    assert result['length_groups'][0]['parts'][0]['editable'] is False


@pytest.mark.parametrize('targets',[{'lengths':{'bottom':0}},{'lengths':{'bottom':float('nan')}},
    {'lengths':None},{'angles':{'bottom':[0,1]}},{'stations':{'missing':20}}])
def test_invalid_numeric_inputs_leave_document_unchanged(factory,targets):
    doc,_=square();assembly=factory(doc)
    with pytest.raises(DocumentError):normalize(assembly,targets)
    assert assembly.doc==doc


def test_regression_and_fuzz_oracle_both_reject_naive_rounding(factory):
    with pytest.raises(AssertionError):check_batch(factory,*square(2),operation=naive_rounding)
    for seed in range(4):
        with pytest.raises(AssertionError):check_batch(factory,*rail_stations(seed),operation=naive_rounding)


@pytest.mark.parametrize('seed',[None,2978354784897548468])
def test_real_metal_frame_rounds_all_cuts_and_supported_stations(library,seed):
    from test_bom_fuzz import case
    document=fixture('complicated-metal') if seed is None else case(seed)[0]
    assembly=Assembly.from_doc(document,FIXTURES,library)
    before=copy.deepcopy(assembly.doc);report=analyse(assembly)
    lengths={p['id']:rounded_dimension(g['suggested_mm']) for g in report['length_groups'] for p in g['parts'] if p['editable']}
    stations={p['joint']:max(math.ceil(p['minimum_mm']),min(math.floor(lengths[p['part']]-p['minimum_mm']),round(p['station_mm'])))
              for p in report['stations'] if p['editable']}
    angles={p['part']:[round(v/15)*15 for v in p['rotation_deg']] for p in report['angles'] if p['editable']}
    started=time.monotonic();result=normalize(assembly,{'lengths':lengths,'stations':stations,'angles':angles})
    assert time.monotonic()-started<120
    assert result['status']=='preview',result
    after=Assembly.from_doc(result['document'],FIXTURES,library)
    assert validate(after)['valid'] and assembly.doc==before
    assert all(c['matched'] for c in result['changes'] if c['kind'] in ('length','station'))
    assert sum(c['kind']=='angle' and not c['matched'] for c in result['changes'])==6
    near=[p for p in after.parts.values() if 174<assembly.parts[p.id].length<176]
    assert len(near)==6 and all(p.length==175 for p in near)
    assert len(after.parts)==218 and len(after.joints)==284 and after.anchors==assembly.anchors


def test_authored_joint_pose_is_not_applied_twice_when_editing_other_parts(load,factory):
    doc=copy.deepcopy(load('hinged-triangle').doc)
    doc['state']={'joints':{'hinge-back':{'angle_deg':5}}}
    doc['parts'].append({'id':'spare','catalog':'tubeclamp.tube-C','parameters':{'length_mm':174.9},
                         'pose':{'position_mm':[10000,0,200]}})
    before=factory(doc);result=normalize(before,{'lengths':{'spare':175}})
    assert result['status']=='preview',result['message']
    after=factory(result['document'])
    assert after.doc['state']==doc['state']
    for pid in before.parts:
        if pid!='spare':assert np.allclose(before.parts[pid].matrix,after.parts[pid].matrix,atol=1e-8)


@pytest.mark.parametrize('length,station',[(75.84,37.92),(175,160)])
def test_station_at_bore_boundary_retains_engagement_and_other_edits(factory,length,station):
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[
        {'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':174.9},'pose':{'position_mm':[0,0,300]}},
        {'id':'tee','catalog':'tubeclamp.TC101C','pose':{'position_mm':[0,0,300]}},
        {'id':'spare','catalog':'tubeclamp.tube-C','parameters':{'length_mm':174.9},'pose':{'position_mm':[1000,0,300]}}
    ],'joints':[{'id':'through','type':'socket','a':{'part':'tee','port':'through'},
       'b':{'part':'pipe','at_mm':87.45},'locked':False,'metadata':{'hardware':'Keep grub screw'},
       'limits':{'slide_mm':[-20,20]}}]}
    before=factory(doc);half=before.parts['tee'].ports['through']['engagement_mm']/2
    if length<100:length=2*half;station=half
    result=normalize(before,{'lengths':{'pipe':length,'spare':175},'stations':{'through':station}})
    after=factory(result['document']);assert validate(after)['valid']
    assert after.parts['pipe'].length==length and after.parts['spare'].length==175
    assert after.joints[0]['b']['at_mm']==min(station,length-half)
    assert after.joints[0]['metadata']==doc['joints'][0]['metadata']
    assert after.joints[0]['limits']==doc['joints'][0]['limits'] and not after.joints[0]['locked']
    assert before.doc==doc


def test_custom_member_geometry_is_visible_but_not_resized(factory):
    doc,_=square();spec=next(p for p in doc['parts'] if p['id']=='bottom')
    spec['body']={'geometry':[{'type':'cylinder','radius_mm':21.2,'length_mm':1000}]}
    report=analyse(factory(doc))
    part=next(p for g in report['length_groups'] for p in g['parts'] if p['id']=='bottom')
    assert not part['editable']
    assert any('bottom' in row['parts'] for row in report['bom']['items'])


def test_authored_sliding_socket_stays_at_its_current_state(factory):
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[
        {'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':500},'pose':{'position_mm':[0,0,300]}},
        {'id':'tee','catalog':'tubeclamp.TC101C','pose':{'position_mm':[0,0,300]}},
        {'id':'spare','catalog':'tubeclamp.tube-C','parameters':{'length_mm':174.9},'pose':{'position_mm':[1000,0,300]}}
    ],'joints':[{'id':'through','type':'socket','a':{'part':'tee','port':'through'},'b':{'part':'pipe','at_mm':250},'locked':False}],
       'state':{'joints':{'through':{'slide_mm':31,'twist_deg':19}}}}
    before=factory(doc);result=normalize(before,{'lengths':{'spare':175}})
    assert result['status']=='preview',result['message']
    after=factory(result['document']);assert after.parts['spare'].length==175
    assert after.joints==before.joints and after.doc['state']==doc['state']
    for pid in ('tee','pipe'):assert np.allclose(after.parts[pid].matrix,before.parts[pid].matrix,atol=1e-9)


def test_regression_and_fuzz_detect_reapplication_of_authored_state(factory,monkeypatch,tmp_path):
    from test_bom_fuzz import check
    monkeypatch.setattr('pipesim.bom_editing._state_parts',lambda assembly:set())
    with pytest.raises(AssertionError):test_authored_sliding_socket_stays_at_its_current_state(factory)
    with pytest.raises(AssertionError):check(4,tmp_path)
