"""Loose sockets that cross the selected Body boundary during draft conversion."""
import copy
import random

import numpy as np

from pipesim.drafting import finalize, reopen, runs
from pipesim.math3d import pose_of, transform
from pipesim.validation import validate


def loose_socket_case(seed=0, *, terminal=False):
    rng=random.Random(seed)
    length=rng.randrange(8,20)*100
    station=length+20 if terminal else rng.uniform(100,length-100)
    port='end' if terminal else 'cross'
    socket={'type':'socket','profile':'round','diameter_mm':42.4,'engagement_mm':40,
            'min_engagement_mm':15,'axis':[0,0,-1 if terminal else 1],
            'position_mm':[0,0,-40 if terminal else 0],'through':not terminal}
    joint={'id':f'tc161c-1-{port}-tube-c-5','type':'socket',
           'a':{'part':'tc161c-1','port':port},
           'b':{'part':'tube-c-5',**({'end':'end'} if terminal else {'at_mm':station})},
           'insertion_mm':20 if terminal else 0,'locked':False,'fit_tolerance_mm':2,
           'limits':{'slide_mm':[-30,70],'angle_deg':[-35,80]},'torque_nm':12,
           'metadata':{'note':'Keep this sliding connection','snapshot_reference_coordinates':{'slide_mm':6.4,'angle_deg':.38}}}
    if seed%3==1: joint.pop('locked')  # Legacy sockets default to loose.
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','name':'Return Body to draft across a loose socket',
         'parts':[{'id':'tube-c-5','catalog':'tubeclamp.tube-C','parameters':{'length_mm':length},
                   'pose':{'position_mm':[0,0,length/2]}},
                  {'id':'tc161c-1','body':{'kind':'connector','mass_kg':1,
                    'geometry':[{'type':'sphere','radius_mm':5,'position_mm':[0,140,0]}],
                    'ports':{port:socket,'other':{**socket,'through':True,'position_mm':[0,80,0],'axis':[1,0,0]}}},
                   'pose':{'position_mm':[0,0,station]}},
                  {'id':'other-pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':600},
                   'pose':{'position_mm':[0,80,station],'rotation_deg':[0,90,0]}}],
         'joints':[joint,{'id':'external-body','type':'socket','a':{'part':'tc161c-1','port':'other'},
                         'b':{'part':'other-pipe','at_mm':300},'insertion_mm':0,'locked':True}],
         'anchors':[{'part':'tc161c-1','surface':'fixture'}],'environment':{'ground':False}}
    delta=transform({'position_mm':[rng.randrange(-10,10)*100 for _ in range(3)],
                     'rotation_deg':[rng.uniform(-180,180) for _ in range(3)]})
    for part in doc['parts']: part['pose']=pose_of(delta@transform(part['pose']))
    return doc


def check_loose_reopen(factory,seed=0,*,terminal=False):
    source=loose_socket_case(seed,terminal=terminal)
    before=factory(source);original=copy.deepcopy(source)
    members=next(group for group in before.editor_groups() if 'tube-c-5' in group)
    assert 'tc161c-1' not in members
    report=validate(before)
    assert report['valid'],report['issues']
    result=reopen(before,members)
    assert result['status']=='reopened' and result['preserved_loose_connections']==1
    assert result['converted_parts']==['tube-c-5']
    draft=result['document'];run=runs(draft)[0]
    assert run['attachments'][0]['joint']==original['joints'][0]
    assert draft['parts']==original['parts'][1:]
    assert draft['joints']==original['joints'][1:]
    assert draft['anchors']==original['anchors']
    # Insertion/station must be derived from the edited geometry, while the
    # original screw state and joint settings survive another round trip.
    finalized=finalize(factory(draft))
    assert finalized['status']=='finalized',finalized
    after=factory(finalized['document'])
    restored=next(j for j in after.joints if j['id']==original['joints'][0]['id'])
    assert restored['locked'] is False
    for key,value in original['joints'][0].items():
        if key!='b': assert restored[key]==value,(key,restored)
    assert after.parts['tube-c-5'].length==before.parts['tube-c-5'].length
    for pid in before.parts:
        assert np.allclose(after.parts[pid].matrix,before.parts[pid].matrix,atol=1e-5,rtol=0),pid
    assert validate(after)['valid']
    assert before.doc==source==original
    return result,finalized
