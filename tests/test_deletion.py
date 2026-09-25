import copy

import numpy as np
import pytest

from pipesim.deletion import delete_parts
from pipesim.document import DocumentError


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
