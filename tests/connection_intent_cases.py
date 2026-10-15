"""Shared square-frame cases for connection regressions and randomized replay."""
import copy

import numpy as np

from pipesim.math3d import pose_of, transform
from pipesim.snapping import connection_options
from pipesim.validation import validate


def square_case(*, offset=.35, tilt=.5, travel=50, articulated=False, orientation=(0,0,0), origin=(0,0,0)):
    axis=transform({'rotation_deg':[tilt,0,0]})[:3,:3]@np.array([0.,0.,-1.])
    socket={'type':'socket','profile':'round','diameter_mm':42.4,'engagement_mm':40,
            'min_engagement_mm':15,'position_mm':[offset,0,0],'axis':axis.tolist()}
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','name':'Preserve square design',
         'parts':[{'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},
                   'pose':{'position_mm':[0,0,500]}},
                  {'id':'fitting','body':{'kind':'connector','mass_kg':1,
                    'geometry':[{'type':'sphere','radius_mm':5,'position_mm':[0,-80,0]}],
                    'ports':{'seat':socket}},'pose':{'position_mm':[travel,0,980]}}],
         'joints':[],'anchors':[{'part':'fitting' if articulated else 'pipe','surface':'fixture'}],
         'environment':{'ground':False}}
    if articulated:
        # The pipe can slide along X or tilt around X. The lateral bore offset
        # is inside tolerance; it must not tip the entire upright to remove it.
        socket['position_mm']=[0,offset,0]
        doc['parts'].append({'id':'guide','body':{'kind':'rigid','mass_kg':1,
             'geometry':[{'type':'sphere','radius_mm':5,'position_mm':[0,-200,0]}]},
             'pose':{'position_mm':[0,0,500]}})
        doc['joints'].append({'id':'slide','type':'cylindrical',
              'a':{'part':'guide','frame':{'axis':[1,0,0]}},
              'b':{'part':'pipe','frame':{'axis':[1,0,0]}},
              'limits':{'slide_mm':[-500,500],'angle_deg':[-30,30]}})
        doc['anchors'].append({'part':'guide','surface':'fixture'})
    delta=transform({'position_mm':list(origin),'rotation_deg':list(orientation)})
    for part in doc['parts']: part['pose']=pose_of(delta@transform(part.get('pose')))
    return doc


def connect(source,factory,**kwargs):
    options=connection_options(factory(source),'pipe','fitting','seat',end='end',insertion_mm=20,**kwargs)
    option=next((o for o in options['options'] if o['move']==options['recommended']),None)
    assert option is not None,[(o['move'],o.get('reason')) for o in options['options']]
    result=factory(option['document'])
    report=validate(result)
    assert report['valid'],report['issues']
    assert not any(i['code']=='INTERSECTION' for i in report['issues']),report['issues']
    return option,result


def repeat_connections(source,factory,cycles=30,**kwargs):
    before=factory(source)
    option,first=connect(source,factory,**kwargs)
    for pid in before.parts:
        assert first.parts[pid].length==before.parts[pid].length
        assert np.allclose(first.parts[pid].matrix[:3,:3],before.parts[pid].matrix[:3,:3],atol=1e-8,rtol=0),pid
    current=first
    for _ in range(cycles):
        detached=copy.deepcopy(current.doc)
        detached['joints']=[j for j in detached['joints'] if j['id']!=first.joints[-1]['id']]
        _,current=connect(detached,factory,**kwargs)
        for pid in before.parts:
            assert np.array_equal(current.parts[pid].matrix,first.parts[pid].matrix),pid
            assert current.parts[pid].length==before.parts[pid].length
    return option,first
