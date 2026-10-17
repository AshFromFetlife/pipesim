"""Timed, replayable batch-normalisation campaign, including the reported model."""
from __future__ import annotations
import copy
import json
import math
import os
from pathlib import Path
import random
import secrets
import subprocess
import sys
import time

if __name__=='__main__':sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import pytest
from pipesim.bom_editing import analyse,normalize
from pipesim.document import Assembly
from pipesim.validation import validate
from bom_cases import square,rail_stations,check_batch,naive_rounding
from fuzz_runtime import budget
from planning_cases import FIXTURES,fixture

PREVIEW_TIMEOUT_SECONDS=120
# The oracle independently builds/checks the source and result as well as the
# preview's own validation. Those checks are not time spent in the UI operation.
CASE_TIMEOUT_SECONDS=450


def timed_normalize(assembly,targets):
    started=time.monotonic()
    result=normalize(assembly,targets,lambda:time.monotonic()-started>PREVIEW_TIMEOUT_SECONDS)
    elapsed=time.monotonic()-started
    assert elapsed<PREVIEW_TIMEOUT_SECONDS,('preview deadline exceeded',elapsed)
    return result


def case(seed):
    rng=random.Random(seed);family=seed%6;step=rng.choice([1,2,5])
    if family==0:
        doc,targets=square(seed,anchored=bool(seed&4))
        targets['lengths']={pid:round(value/step)*step for pid,value in targets['lengths'].items()}
    elif family==1:
        doc,targets=rail_stations(seed)
        targets['stations']={pid:round(value/step)*step for pid,value in targets['stations'].items()}
        targets['lengths']={p['id']:round(p['parameters']['length_mm']/step)*step
                           for p in doc['parts'] if 'length_mm' in p.get('parameters',{})}
        for j in doc['joints']:j['locked']=bool(rng.getrandbits(1))
    elif family==2:
        doc=fixture('complicated-metal');report=analyse(Assembly.from_doc(doc,FIXTURES))
        lengths={p['id']:round(g['suggested_mm']) for g in report['length_groups'] for p in g['parts'] if p['editable']}
        stations={p['joint']:max(math.ceil(p['minimum_mm']),min(math.floor(lengths[p['part']]-p['minimum_mm']),round(p['station_mm'])))
                  for p in report['stations'] if p['editable']}
        targets={'lengths':lengths,'stations':stations}
        # Preserve the troublesome almost-Euler-singular mirrored fittings.
        # The six diagonal goals are intentionally not required to be met.
        if seed&4:targets['angles']={p['part']:[round(v/15)*15 for v in p['rotation_deg']] for p in report['angles'] if p['editable']}
    elif family in (3,4):
        length=174+rng.random();station=length-38
        doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[
            {'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':length},'pose':{'position_mm':[0,0,300]}},
            {'id':'tee','catalog':'tubeclamp.TC101C','pose':{'position_mm':[0,0,300+station-length/2]}},
            {'id':'spare','catalog':'tubeclamp.tube-C','parameters':{'length_mm':length},'pose':{'position_mm':[1000,0,300]}}
        ],'joints':[{'id':'through','type':'socket','a':{'part':'tee','port':'through'},'b':{'part':'pipe','at_mm':station},'locked':bool(seed&8)}]}
        half=Assembly.from_doc(doc,FIXTURES).parts['tee'].ports['through']['engagement_mm']/2
        targets={'lengths':{'pipe':2*half if seed&16 else 175,'spare':175},'stations':{'through':180}}
        if family==4:
            doc['parts'][0]['parameters']['length_mm']=500
            doc['parts'][1]['pose']['position_mm']=[0,0,300]
            doc['joints'][0]['b']['at_mm']=250;doc['joints'][0]['locked']=False
            doc['state']={'joints':{'through':{'slide_mm':rng.uniform(-50,50),'twist_deg':rng.uniform(-45,45)}}}
            targets={'lengths':{'spare':175}}
    else:
        doc,_=square(seed,anchored=True);doc['build_plan']={'cached':True}
        targets={'angles':{'corner-0':[0,0,15]}}
    rng.shuffle(doc['parts']);rng.shuffle(doc['joints'])
    return doc,targets


def check(seed,repro):
    doc,targets=case(seed);original=copy.deepcopy(doc);result=None
    try:
        factory=lambda d:Assembly.from_doc(d,FIXTURES)
        if seed%6 in (0,1):result=check_batch(factory,doc,targets,operation=timed_normalize)
        elif seed%6==2:
            before=factory(doc);assert validate(before)['valid']
            result=timed_normalize(before,targets);after=factory(result['document'])
            assert result['status']=='preview' and validate(after)['valid']
            assert all(c['matched'] for c in result['changes'] if c['kind']!='angle'),result['retained_targets']
            assert set(before.parts)==set(after.parts) and before.anchors==after.anchors
            assert len(before.joints)==len(after.joints)==284
            for pid,value in targets['lengths'].items():assert after.parts[pid].length==value
            for j in after.joints:
                a,b,_=after.joint_frames(j)
                assert math.dist(a,b)<=j.get('fit_tolerance_mm',1.1)
        elif seed%6==3:
            before=factory(doc);assert validate(before)['valid']
            result=timed_normalize(before,targets);after=factory(result['document'])
            assert result['status']=='preview' and validate(after)['valid']
            for pid,value in targets['lengths'].items():assert after.parts[pid].length==value
            half=after.parts['tee'].ports['through']['engagement_mm']/2
            assert after.joints[0]['b']['at_mm']==after.parts['pipe'].length-half
            assert result['retained_targets']==['through']
            a,b,_=after.joint_frames(after.joints[0]);assert math.dist(a,b)<.002
        elif seed%6==4:
            import numpy as np
            before=factory(doc);result=timed_normalize(before,targets);after=factory(result['document'])
            assert result['status']=='preview' and after.parts['spare'].length==175
            assert validate(after)['valid'] and before.joints==after.joints and doc['state']==after.doc['state']
            for pid in ('tee','pipe'):assert np.allclose(before.parts[pid].matrix,after.parts[pid].matrix,atol=1e-8)
        else:
            result=timed_normalize(factory(doc),targets)
            assert result['status']=='unchanged' and result['document']==doc
            assert result['retained_targets']==['corner-0']
        assert doc==original
    except Exception:
        path=Path(repro);path.mkdir(parents=True,exist_ok=True)
        (path/f'bom-{seed}.json').write_text(json.dumps({'seed':seed,'document':doc,'targets':targets,'result':result},indent=2),encoding='utf-8')
        raise


def run(seed,repro):
    try:
        result=subprocess.run([sys.executable,'-B',str(Path(__file__).resolve()),str(seed),str(repro)],
            capture_output=True,text=True,timeout=CASE_TIMEOUT_SECONDS)
    except subprocess.TimeoutExpired:
        doc,targets=case(seed);Path(repro).mkdir(parents=True,exist_ok=True)
        (Path(repro)/f'bom-{seed}-timeout.json').write_text(json.dumps({'seed':seed,'document':doc,'targets':targets}),encoding='utf-8')
        raise
    assert result.returncode==0,f'BOM case_seed={seed}\n{result.stdout}\n{result.stderr}'


@pytest.mark.parametrize('seed',[0,1,2,3,4,5,6,7,8,14,19,21,22,2978354784897548468])
def test_retained_bom_batch_matrix(seed,tmp_path):run(seed,tmp_path)


def test_generated_bom_oracle_detects_unfitted_graph(factory):
    for seed in (0,1,6,7):
        with pytest.raises(AssertionError):check_batch(factory,*case(seed),operation=naive_rounding)


def test_fresh_bom_batch_campaign():
    root_seed=int(os.environ.get('PIPESIM_BOM_FUZZ_SEED',str(secrets.randbits(64))))
    rng=random.Random(root_seed);repro=Path(os.environ.get('PIPESIM_BOM_FUZZ_REPRO_DIR','fuzz-runs/bom'))
    replay=os.environ.get('PIPESIM_BOM_FUZZ_CASE_SEED')
    if replay is not None:run(int(replay),repro);return
    minutes,started,deadline=budget(default_minutes=30);count=0
    print(f'BOM root_seed={root_seed} budget_minutes={minutes}',flush=True)
    while time.monotonic()<deadline or count<6:
        seed=rng.randrange(2**60)*6+count%6
        run(seed,repro);count+=1
        print(f'BOM passed case_seed={seed} family={seed%6} count={count}',flush=True)
    print(f'BOM passed {count} fresh cases in {time.monotonic()-started:.1f}s; root_seed={root_seed}',flush=True)


if __name__=='__main__':check(int(sys.argv[1]),sys.argv[2])
