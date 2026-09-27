import copy

import numpy as np
import pytest

from pipesim.deletion import delete_parts
from pipesim.document import DocumentError
from pipesim.drafting import preview


def scene(factory,blank):
    doc=copy.deepcopy(blank)
    doc['objects']=[{'id':'chain','template':'chain','parameters':{'length_mm':100}}]
    doc['parts']=[{'id':pid,'catalog':'generic.box'} for pid in ('support','other')]
    doc['joints']=[{'id':'attachment','type':'revolute','a':{'part':'chain/link-5'},'b':{'part':'support'}}]
    doc['anchors']=[{'part':'chain/link-1','surface':'ceiling'}]
    doc['loads']=[{'part':'chain/link-3','force_n':[0,0,-10]},{'part':'other','force_n':[0,0,-2]}]
    return factory(doc)


def test_deleting_object_removes_links_references_and_preserves_external_parts(factory,blank):
    assembly=scene(factory,blank);original=copy.deepcopy(assembly.doc)
    assembly.doc['animation']={'duration_s':1,'tracks':[{'joint':'attachment','coordinate':'angle_deg',
        'keyframes':[{'time_s':0,'value':0},{'time_s':1,'value':10}]}]}
    assembly.doc['drives']=[{'id':'drive','type':'gear','driver':'attachment','follower':'chain/join-1'}]
    assembly.doc['metadata']={'detached_attachments':[{'joint':copy.deepcopy(assembly.joints[0])}]}
    assembly.doc['tests']=[{'id':'seat','type':'seat','human':'someone','seat':'chain/link-4'}]
    assembly.doc['build']={'sequence':['chain/link-1','support'],'fixtures':[{'part':'chain/link-2','description':'Hold link'}]}
    result=delete_parts(assembly,object_id='chain');after=factory(result['document'])
    assert set(after.parts)=={'support','other'}
    assert not after.joints and not after.anchors
    assert after.doc['loads']==original['loads'][1:]
    assert after.doc['animation']['tracks']==[] and after.doc['drives']==[] and after.doc['tests']==[]
    assert after.doc['metadata']['detached_attachments']==[]
    assert after.doc['build']=={'sequence':['support'],'fixtures':[]}
    for pid in after.parts: assert np.allclose(after.parts[pid].matrix,assembly.parts[pid].matrix)
    assert len(result['deleted_parts'])==5 and len(result['deleted_joints'])==5
    assert 'chain/link-1' in assembly.parts


def test_delete_body_only_removes_explicit_members_even_with_external_fixed_joint(factory,blank):
    assembly=scene(factory,blank)
    assembly.doc['joints'][0]['type']='fixed';assembly=factory(assembly.doc)
    after=factory(delete_parts(assembly,['support'])['document'])
    assert 'support' not in after.parts and 'chain/link-5' in after.parts
    assert after.doc['objects']==assembly.doc['objects']
    assert len(after.joints)==4


def test_deleting_posed_attachment_keeps_survivors_in_place(factory,blank):
    assembly=scene(factory,blank)
    assembly.doc['state']={'joints':{'attachment':{'angle_deg':35},'chain/join-2':{'rotation_deg':[0,20,0]}}}
    assembly=factory(assembly.doc)
    after=factory(delete_parts(assembly,object_id='chain')['document'])
    for pid in after.parts: assert np.allclose(after.parts[pid].matrix,assembly.parts[pid].matrix,atol=1e-5)
    after=factory(delete_parts(assembly,['support'])['document'])
    assert len(after.doc['objects'])==1
    for pid in after.parts: assert np.allclose(after.parts[pid].matrix,assembly.parts[pid].matrix,atol=1e-5)


@pytest.mark.parametrize('args',[{'members':['missing']},{'members':['chain/link-2']},{'object_id':'missing'},{'members':[]}])
def test_invalid_or_partial_object_deletion_is_atomic(factory,blank,args):
    assembly=scene(factory,blank);before=copy.deepcopy(assembly.doc)
    with pytest.raises(DocumentError): delete_parts(assembly,**args)
    assert assembly.doc==before


def test_delete_last_object_leaves_valid_empty_design(factory,blank):
    doc=copy.deepcopy(blank);doc['objects']=[{'id':'chain','template':'chain','parameters':{'length_mm':100}}]
    assert not factory(delete_parts(factory(doc),object_id='chain')['document']).parts


def test_deleting_a_mirrored_fitting_removes_its_draft_membership(factory,blank):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'tc101c-1','catalog':'tubeclamp.TC101C'}]
    doc['draft_subassemblies']=[{'id':'frame','runs':[{'id':'run','catalog':'tubeclamp.tube-C',
        'start_mm':[0,200,100],'end_mm':[1000,200,100]}],
        'mirrors':[{'id':'side','axis':'y','offset_mm':0}], 'mirror_parts':['tc101c-1']}]
    result=delete_parts(factory(doc),['tc101c-1'])
    assert result['document']['draft_subassemblies'][0]['mirror_parts']==[]
    factory(result['document'])


def connected_mirrored_draft(blank):
    doc=copy.deepcopy(blank)
    def fitting(identifier,x,axis):
        return {'id':identifier,'body':{'kind':'connector','mass_kg':.2,
            'geometry':[{'type':'sphere','radius_mm':5}],
            'ports':{'socket':{'type':'socket','profile':'round','diameter_mm':42.4,
                'position_mm':[0,0,0],'axis':axis,'engagement_mm':40,'min_engagement_mm':15}}},
            'pose':{'position_mm':[x,0,100]}}
    doc['parts']=[fitting('left',0,[1,0,0]),fitting('right',1000,[-1,0,0])]
    doc['draft_subassemblies']=[{'id':'frame','runs':[
        {'id':'tube','catalog':'tubeclamp.tube-C','start_mm':[0,0,100],
         'end_mm':[1000,0,100],'attachments':[
             {'connector':'left','port':'socket','end':'start','insertion_mm':20},
             {'connector':'right','port':'socket','end':'end','insertion_mm':20}]},
        {'id':'spare','catalog':'tubeclamp.tube-C','start_mm':[0,0,300],
         'end_mm':[600,0,300],'attachments':[]}],
        'mirrors':[{'id':'midline','axis':'y','offset_mm':0,
                    'run_modes':{'tube':'in_plane','spare':'in_plane'}}],
        'mirror_parts':['left','right']}]
    return doc


def test_delete_connected_draft_fitting_detaches_and_preserves_span(factory,blank):
    assembly=factory(connected_mirrored_draft(blank))
    before=next(run for run in preview(assembly) if run['id']=='tube')
    original=copy.deepcopy(assembly.doc)
    result=delete_parts(assembly,['left'])
    after=factory(result['document'])
    run=next(run for run in preview(after) if run['id']=='tube')
    assert {a['connector'] for a in run['attachments']}=={'right'}
    assert np.allclose(run['pose']['position_mm'],before['pose']['position_mm'])
    assert run['length_mm']==pytest.approx(before['length_mm'])
    assert after.doc['draft_subassemblies'][0]['mirror_parts']==['right']
    assert assembly.doc==original


@pytest.mark.parametrize('connector',['left','right'])
def test_delete_end_fitting_keeps_centered_mirror_constraint(factory,blank,connector):
    doc=connected_mirrored_draft(blank)
    doc['draft_subassemblies'][0]['mirrors']=[{'id':'centre','axis':'x','offset_mm':500,
                                                'run_modes':{'tube':'centered'}}]
    assembly=factory(doc)
    after=factory(delete_parts(assembly,[connector])['document'])
    run=next(run for run in preview(after) if run['id']=='tube')
    assert run['length_mm']==pytest.approx(1040)
    assert run['pose']['position_mm'][0]==pytest.approx(500)
    assert len(run['attachments'])==1


def test_delete_connected_draft_run_cleans_mirror_constraints(factory,blank):
    assembly=factory(connected_mirrored_draft(blank))
    original=copy.deepcopy(assembly.doc)
    result=delete_parts(assembly,['tube'])
    after=factory(result['document'])
    group=after.doc['draft_subassemblies'][0]
    assert [run['id'] for run in group['runs']]==['spare']
    assert group['mirrors'][0]['run_modes']=={'spare':'in_plane'}
    assert {part['id'] for part in after.doc['parts']}=={'left','right'}
    assert result['deleted_parts']==['tube']
    assert 'draft_subassemblies' not in delete_parts(after,['spare'])['document']
    assert assembly.doc==original
