import copy

import numpy as np
import pytest

from pipesim.document import DocumentError, write, Assembly
from pipesim.naming import rename, part_name
from pipesim.deletion import delete_parts
from pipesim.duplication import duplicate
from pipesim.editing import expand_objects
from pipesim.grouping import regroup_object


def chain(factory,blank):
    doc=copy.deepcopy(blank)
    doc['objects']=[{'id':'chain','template':'chain','parameters':{'length_mm':100}}]
    doc['parts']=[{'id':'support','catalog':'generic.box'}]
    doc['joints']=[{'id':'attachment','type':'spherical','a':{'part':'support'},'b':{'part':'chain/link-1'}}]
    return factory(doc)


def test_object_rename_keeps_ids_pose_references_and_survives_save_expand_regroup(factory,blank,tmp_path):
    source=chain(factory,blank);before=copy.deepcopy(source.doc)
    result=rename(source,'  Hanging chain  ',object_id='chain')
    after=factory(result['document'])
    assert after.doc['objects'][0]['label']=='Hanging chain'
    assert after.joints==source.joints and after.parts.keys()==source.parts.keys()
    for pid in after.parts: assert np.array_equal(after.parts[pid].matrix,source.parts[pid].matrix)
    path=tmp_path/'named.yaml';write(path,after.doc)
    assert Assembly.load(path).doc==after.doc
    expanded=factory(expand_objects(after))
    regrouped=regroup_object(expanded,'chain')
    assert regrouped['objects'][0]['label']=='Hanging chain'
    assert source.doc==before
    assert not rename(after,'Hanging chain',object_id='chain')['changed']


def test_part_names_work_inside_objects_and_on_independent_parts(factory,blank):
    a=chain(factory,blank)
    for pid in ('support','chain/link-2'):
        a=factory(rename(a,'Named '+pid,part_id=pid)['document'])
        assert part_name(a,pid)=='Named '+pid
    assert 'components' not in a.doc['objects'][0]
    copied=factory(duplicate(a,'chain/link-1','subassembly')['document'])
    assert part_name(copied,'chain-copy/link-2')=='Named chain/link-2'
    expanded=factory(expand_objects(a))
    expanded=factory(rename(expanded,'Changed after expanding',part_id='chain/link-2')['document'])
    assert part_name(expanded,'chain/link-2')=='Changed after expanding'
    deleted=delete_parts(a,object_id='chain')['document']
    assert not deleted['metadata']['part_labels']


def test_body_names_follow_members_instead_of_tree_order_and_copy_with_body(factory,blank):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':pid,'catalog':'generic.box'} for pid in ('a','b','other')]
    doc['joints']=[{'id':'fixed','type':'fixed','a':{'part':'a'},'b':{'part':'b'}}]
    a=factory(doc)
    a=factory(rename(a,'Base frame',members=['b','a'])['document'])
    assert a.doc['metadata']['body_labels']==[{'parts':['a','b'],'label':'Base frame'}]
    assert not rename(a,'Base frame',members=['a','b'])['changed']
    copied=duplicate(a,'a','subassembly')['document']
    assert copied['metadata']['body_labels'][-1]=={'parts':['a-copy','b-copy'],'label':'Base frame'}
    assert not delete_parts(a,['a','b'])['document']['metadata']['body_labels']
    with pytest.raises(DocumentError,match='changed'): rename(a,'Wrong body',members=['a','other'])


@pytest.mark.parametrize('name',['','  ',None,42,'x'*121,'two\nlines'])
def test_bad_names_fail_without_editing(factory,blank,name):
    a=chain(factory,blank);before=copy.deepcopy(a.doc)
    with pytest.raises(DocumentError): rename(a,name,object_id='chain')
    assert a.doc==before


def test_unknown_or_ambiguous_targets_are_rejected(factory,blank):
    a=chain(factory,blank)
    for target in ({'object_id':'missing'},{'part_id':'missing'},{'members':[]},{'object_id':'chain','part_id':'support'}):
        with pytest.raises(DocumentError): rename(a,'Name',**target)
