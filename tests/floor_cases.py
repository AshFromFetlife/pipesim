"""Free frames whose loose horizontal sockets cannot supply a vertical drop."""
import copy
import random

import numpy as np

from pipesim.geometry import lowest_z
from pipesim.math3d import pose_of, transform
from pipesim.posing import drop_to_floor
from pipesim.snapping import _movement_coordinates


def free_floor_case(seed=0):
    rng=random.Random(seed)
    length=rng.randrange(12,30)*100
    height=rng.choice((-1,1))*rng.uniform(200,800)
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','name':f'Free floor frame {seed}',
         'parts':[{'id':'rail','catalog':'tubeclamp.tube-C','parameters':{'length_mm':length},
                   'pose':{'rotation_deg':[0,90,0]}}], 'joints':[], 'anchors':[]}
    def box(pid,point,size):
        return {'id':pid,'body':{'kind':'rigid','mass_kg':1,
                 'geometry':[{'type':'box','size_mm':size}]},'pose':{'position_mm':point}}
    # Several sockets on the same pipe form a closed loop through another body.
    # The larger body becomes the IK reference, despite having no world fixing.
    for index,x in enumerate((-length*.3,0,length*.3)):
        pid=f'sleeve-{index}'
        doc['parts'].append({'id':pid,'body':{'kind':'connector','mass_kg':1,
            'geometry':[{'type':'box','size_mm':[60,60,60]}],
            'ports':{'bore':{'type':'socket','profile':'round','diameter_mm':42.4,
                            'through':True,'engagement_mm':60,'axis':[1,0,0],
                            'position_mm':[0,0,0]}}},'pose':{'position_mm':[x,0,0]}})
        doc['parts'].append(box(f'host-{index}',[x,100,0],[50,50,50]))
        doc['joints'].append({'id':f'slide-{index}','type':'socket',
            'a':{'part':pid,'port':'bore'},'b':{'part':'rail','at_mm':x+length/2},
            'insertion_mm':0,'locked':False,
            'limits':{'slide_mm':[-1,1],'angle_deg':[-1,1]}})
        if seed%2: doc['joints'][-1].pop('locked')
        doc['joints'].append({'id':f'host-mount-{index}','type':'fixed',
            'a':{'part':pid,'frame':{'position_mm':[0,100,0]}},'b':{'part':f'host-{index}'}})
        if index:
            doc['joints'].append({'id':f'host-bridge-{index}','type':'fixed',
                'a':{'part':'sleeve-0','frame':{'position_mm':[x+length*.3,0,0]}},
                'b':{'part':pid}})
    # Either the selected body or an attached body supplies the lowest point.
    for index,x in enumerate((-length*.4,length*.4)):
        doc['parts'].append(box(f'foot-{index}',[x,0,-80],[40,40,100]))
        doc['joints'].append({'id':f'foot-mount-{index}','type':'fixed',
            'a':{'part':'rail','frame':{'position_mm':[80,0,x]}},'b':{'part':f'foot-{index}'}})
    if seed%3==0:
        doc['parts'][2]['body']['geometry'][0]['position_mm']=[0,0,-200]
    world=transform({'position_mm':[rng.uniform(-2000,2000),rng.uniform(-2000,2000),height],
                     'rotation_deg':[rng.uniform(-2,2),rng.uniform(-2,2),rng.uniform(-180,180)]})
    for part in doc['parts']: part['pose']=pose_of(world@transform(part.get('pose')))
    if seed%4==0:
        doc['objects']=[{'id':'payload','template':'custom','pose':pose_of(world),'components':{
            'parts':[box('mount',[0,100,0],[20,20,20]),box('weight',[0,100,-350],[50,50,50])],
            'joints':[]}}]
        doc['joints'].append({'id':'payload-mount','type':'spherical',
            'a':{'part':'host-1'},'b':{'part':'payload/mount'}})
    rng.shuffle(doc['parts'])
    # An unrelated world fixing below the floor must neither block nor set height.
    doc['parts'].append(box('unrelated',[5000,0,-5000],[100,100,100]))
    doc['anchors']=[{'part':'unrelated','surface':'fixture'}]
    return doc


def check_free_floor(factory,seed=0,*,selected='rail',prepared=None):
    source=free_floor_case(seed);before=factory(source);original=copy.deepcopy(before.doc)
    members=set(before.parts)-{'unrelated'}
    height=min(lowest_z(before.parts[p]) for p in members)
    result=drop_to_floor(before,selected,prepared=prepared(before) if prepared else None)
    after=factory(result['document'])
    assert set(result['moved'])==members
    assert abs(min(lowest_z(after.parts[p]) for p in members))<1e-7
    assert abs(result['floor_gap_mm'])<1e-7 and not result['limited']
    delta=np.eye(4);delta[2,3]=-height
    for pid in members:
        assert np.allclose(after.parts[pid].matrix,delta@before.parts[pid].matrix,atol=1e-7,rtol=0),pid
        assert after.parts[pid].length==before.parts[pid].length
    assert np.array_equal(before.parts['unrelated'].matrix,after.parts['unrelated'].matrix)
    assert after.doc['joints']==before.doc['joints']
    assert after.doc['anchors']==before.doc['anchors']
    for obj in before.doc.get('objects',[]):
        moved=next(o for o in after.doc['objects'] if o['id']==obj['id'])
        assert {k:v for k,v in moved.items() if k!='pose'}=={k:v for k,v in obj.items() if k!='pose'}
    assert _movement_coordinates(before,after)=={}
    assert before.doc==original
    again=drop_to_floor(after,selected)
    assert not again['moved'] and again['document']==after.doc
    return result
