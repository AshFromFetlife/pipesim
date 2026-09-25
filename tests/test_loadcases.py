import copy
import pytest
import numpy as np
from pipesim.physics import simulate
from pipesim.loadcases import contact_load_case
from pipesim.fea import analyse
from pipesim.document import DocumentError

def test_settled_box_contact_transfers_weight_to_bench(load,factory):
    doc=copy.deepcopy(load('workbench').doc); doc['loads']=[]
    doc['parts'].append({'id':'test-load','catalog':'generic.box','pose':{'position_mm':[0,0,960]}})
    a=factory(doc); recording=simulate(a,1.0,30)
    case=contact_load_case(a,recording)
    assert 'test-load' not in [p['id'] for p in case['parts']]
    transferred=case['metadata']['contact_load_case']['resultants']
    force=np.sum([v['force_n'] for v in transferred.values()],axis=0)
    assert force[2]==pytest.approx(-196.2,rel=.02)
    assert np.linalg.norm(force[:2])<1
    result=analyse(factory(case),include_self_weight=False)
    assert result['status']=='solved'
    assert sum(v[2] for v in result['anchor_reactions'].values())==pytest.approx(196.2,rel=.02)

def test_stale_contact_data_is_not_reused(load):
    with pytest.raises(DocumentError,match='stale'):
        contact_load_case(load('workbench'),{'input_sha256':'old'})

def test_library_edit_changes_result_fingerprint(load):
    a=load('workbench'); original=a.input_hash
    next(p for p in a.parts.values() if p.kind=='member').definition['material_data']['youngs_modulus_pa']*=.9
    assert a.input_hash!=original
