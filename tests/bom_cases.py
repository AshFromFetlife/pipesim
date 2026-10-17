"""Valid manufacturing edits and independent connection checks for BOM batching."""
import copy
import random
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from pipesim.document import read, Assembly
from pipesim.math3d import pose_of, transform
from pipesim.bom_editing import normalize
from pipesim.validation import validate
from planning_cases import ladder

ROOT=Path(__file__).resolve().parents[1]


def square(seed=0, anchored=False):
    rng=random.Random(seed)
    doc=read(ROOT/'tests/fixtures/resize-square.pipe.yaml')
    lengths=[rng.randint(400,1400)+rng.uniform(.15,.45) for _ in range(2)]
    rotation=Rotation.from_euler('z',0 if anchored else rng.choice([-30,0,15,60])+.27,degrees=True).as_matrix()
    for spec in doc['parts']:
        m=transform(spec.get('pose'))
        m[0,3]*=(lengths[0]+65.6)/1065.6;m[1,3]*=(lengths[1]+65.6)/1065.6
        if 'parameters' in spec:spec['parameters']['length_mm']=lengths[0 if spec['id'] in ('bottom','top') else 1]
        m[:3,3]=rotation@m[:3,3]+[0,0,100]
        m[:3,:3]=rotation@m[:3,:3];spec['pose']=pose_of(m)
    if anchored:doc['anchors']=[{'part':'corner-0','surface':'fixture'}]
    targets={'lengths':{p['id']:round(p['parameters']['length_mm']) for p in doc['parts'] if 'parameters' in p},
             'angles':{p['id']:[round(v/15)*15 for v in p['pose']['rotation_deg']] for p in doc['parts'] if not anchored}}
    return doc,targets


def rail_stations(seed=0):
    doc=ladder(seed,levels=3)
    # The source is valid but deliberately has non-round socket stations.
    parts={p['id']:p for p in doc['parts']}
    direction=transform(parts['rail-left']['pose'])[:3,2]
    for spec in doc['parts']:
        if spec['id'].startswith('rail-'):continue
        spec['pose']['position_mm']=(np.array(spec['pose']['position_mm'])+direction*.37).tolist()
    targets={'stations':{}}
    for j in doc['joints']:
        j['metadata']={'hardware':'Retain this original screw specification'}
        if 'at_mm' in j['b']:
            j['b']['at_mm']+=.37;targets['stations'][j['id']]=round(j['b']['at_mm'])
    return doc,targets


def check_batch(factory,doc,targets,operation=normalize):
    before=factory(doc);original=copy.deepcopy(before.doc)
    assert validate(before)['valid']
    result=operation(before,targets)
    assert before.doc==original,'preview mutated source document'
    assert result['status']=='preview',result
    assert all(c['matched'] for c in result['changes']),result['changes']
    after=factory(result['document']);report=validate(after)
    assert report['valid'],report
    assert set(before.parts)==set(after.parts)
    assert before.anchors==after.anchors
    assert [j['id'] for j in before.joints]==[j['id'] for j in after.joints]
    for old,new in zip(before.joints,after.joints):
        for key in set(old)-{'insertion_mm','b'}:assert new[key]==old[key],key
        for key in set(old['b'])-{'at_mm'}:assert new['b'][key]==old['b'][key],key
        a,b,_=after.joint_frames(new)
        assert np.linalg.norm(a-b)<.002,('connection is not closed after rounding',new['id'],a-b)
    for anchor in before.anchors:
        assert np.allclose(before.parts[anchor['part']].matrix,after.parts[anchor['part']].matrix,atol=1e-8)
    for pid,value in targets.get('lengths',{}).items():assert after.parts[pid].length==value
    for jid,value in targets.get('stations',{}).items():
        j=next(j for j in after.joints if j['id']==jid);assert j['b']['at_mm']==value
        pipe=after.parts[j['b']['part']];mouth,_=after.parts[j['a']['part']].frame(j['a'])
        station=(mouth-pipe.matrix[:3,3])@pipe.matrix[:3,2]+pipe.length/2
        assert abs(station-value)<.002
    return result


def naive_rounding(assembly,targets):
    """Negative control: change numbers without fitting the connection graph."""
    doc=copy.deepcopy(assembly.doc)
    for p in doc['parts']:
        if p['id'] in targets.get('lengths',{}):p['parameters']['length_mm']=targets['lengths'][p['id']]
        if p['id'] in targets.get('angles',{}):p['pose']['rotation_deg']=targets['angles'][p['id']]
    for j in doc['joints']:
        if j['id'] in targets.get('stations',{}):j['b']['at_mm']=targets['stations'][j['id']]
    return {'status':'preview','document':doc,'changes':[{'matched':True}]}
