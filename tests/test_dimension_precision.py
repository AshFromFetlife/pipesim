import copy
import random

import pytest

from pipesim.dimensions import dimension_precision, format_dimension, format_dimension_text
from pipesim.exporting import bom, cutting_plan


def tubes(blank, lengths):
    blank['parts']=[{'id':f'tube-{i}','catalog':'tubeclamp.tube-C',
                     'parameters':{'length_mm':length}} for i,length in enumerate(lengths)]
    return blank


def test_supplier_cuts_group_the_six_near_175mm_lengths_without_changing_geometry(blank,factory):
    lengths=[174.886027,174.885476,174.887322]*2+[1937.2001412798647]
    assembly=factory(tubes(blank,lengths)); before=copy.deepcopy(assembly.doc)
    items=bom(assembly)['items']
    grouped=next(row for row in items if row['length_mm']==175)
    assert grouped['quantity']==6
    assert list(grouped['exact_lengths_mm'].values())==lengths[:6]
    assert next(row for row in items if row['length_mm']==1937)['quantity']==1
    cuts=cutting_plan(assembly)
    assert sorted(c['length_mm'] for b in cuts['bars'] for c in b['cuts'])==[175]*6+[1937]
    assert sorted(c['exact_length_mm'] for b in cuts['bars'] for c in b['cuts'])==sorted(lengths)
    assert assembly.doc==before


def test_rounding_changes_stock_allocation_and_keeps_exact_evidence(blank,factory):
    assembly=factory(tubes(blank,[100.49,100.49]))
    rounded=cutting_plan(assembly,stock_lengths_mm=[200],kerf_mm=0)
    precise=cutting_plan(assembly,stock_lengths_mm=[200],kerf_mm=0,precision_mm=.01)
    assert rounded['stock_count']==1 and rounded['offcut_total_mm']==0
    assert precise['stock_count']==2
    assert len(bom(assembly,precision_mm=.01)['items'])==1


def test_precision_formatting_and_invalid_options():
    assert format_dimension(1937.2001412798647,.1)=='1937.2'
    assert format_dimension(-1.25,.5)=='-1.5'
    assert format_dimension(-.00001)=='0'
    assert format_dimension_text('Position tube, cut to 0.2 mm.')=='Position tube, cut to 1 mm.'
    assert format_dimension_text('Cut to 0.2 mm.')=='Cut to 1 mm.'
    assert format_dimension_text('Insert tube-1937.2 by 22.650722 mm; turn 90.0001°.',.1)=='Insert tube-1937.2 by 22.7 mm; turn 90.0001°.'
    for value in [0,-1,.0001,11,float('nan'),float('inf'),True,'1']:
        with pytest.raises(ValueError):dimension_precision(override=value)


def test_cached_book_uses_selected_precision_in_html_csv_and_instructions(blank,factory,tmp_path,monkeypatch):
    import json
    from pipesim.exporting import build_export
    from pipesim.planning import PLANNING_ALGORITHM_VERSION
    doc=tubes(blank,[1937.2001412798647]);doc['environment']={'ground':False}
    assembly=factory(doc);p=assembly.parts['tube-0']
    pose={'position_mm':[123.4567,0,0],'rotation_deg':[0,0,0]}
    plan={'status':'buildable','input_sha256':assembly.input_hash,
          'certificate':{'algorithm_version':PLANNING_ALGORITHM_VERSION},
          'steps':[{'number':1,'kind':'position','part':p.id,'parts':[p.id],
                    'installed_parts':[p.id],'pose':pose,'poses':{p.id:pose},
                    'instruction':'Position tube-0, cut to 1937.2001412798647 mm.',
                    'fasten':[],'fixture':None}]}
    assembly.doc['build_plan']=plan
    monkeypatch.setattr('pipesim.exporting.plan_build',lambda *_:pytest.fail('Formatting must not replan'))
    build_export(assembly,tmp_path,illustrations=False,precision_mm=.1)
    html=(tmp_path/'instructions.html').read_text(encoding='utf-8')
    assert 'cut to 1937.2 mm' in html and 'X 123.5' in html
    assert '1937.2001412798647' not in html
    assert '1937.2' in (tmp_path/'bill-of-materials.csv').read_text(encoding='utf-8-sig')
    evidence=json.loads((tmp_path/'bill-of-materials.json').read_text())
    assert evidence['items'][0]['exact_lengths_mm'][p.id]==1937.2001412798647
    assert assembly.doc['build_plan']==plan


def test_seeded_precision_cutting_matrix_preserves_stock_and_quantizes_every_cut(blank,factory):
    # Includes the observed supplier-list failure on every seed, plus rounding
    # boundaries, very short parts, fractional kerfs and multiple stock sizes.
    for seed in range(100):
        rng=random.Random(seed); step=rng.choice([.001,.0011,.01,.1,.123456,.333333,.5,1,2,10])
        lengths=[174.886027,174.885476,174.887322]*2
        lengths += [rng.uniform(.01,2000) for _ in range(15)]
        doc=tubes(copy.deepcopy(blank),lengths)
        doc['build']={'dimension_precision_mm':step}
        assembly=factory(doc); before=copy.deepcopy(assembly.doc)
        cuts=cutting_plan(assembly,kerf_mm=rng.choice([0,.3,3.2]),stock_lengths_mm=[2500,6000],end_trim_mm=2.5)
        items=bom(assembly)['items']; cut_by_id={c['part']:c for b in cuts['bars'] for c in b['cuts']}
        assert sum(item['quantity'] for item in items)==len(lengths)
        for item in items:
            for pid in item['parts']:
                cut=cut_by_id[pid]
                assert cut['length_mm']==item['length_mm']>0
                assert cut['length_mm']/step==pytest.approx(round(cut['length_mm']/step),abs=1e-7)
                assert abs(cut['length_mm']-cut['exact_length_mm'])<=step+1e-8
        assert cuts['stock_total_mm']==pytest.approx(sum(cuts[k] for k in ['finished_total_mm','kerf_total_mm','trim_total_mm','offcut_total_mm']))
        assert assembly.doc==before
