"""The structural beam result must have a visible, contact-aware counterpart."""
import copy

import numpy as np
import pytest

from pipesim.fea import analyse
from pipesim.physics import simulate, _beam_deflection_mm


def test_sliding_guide_keeps_its_continuous_track(load):
    from pipesim.beam_dynamics import prepare_beams
    assembly=load('sliding-collar')
    dynamic,model,analysis=prepare_beams(assembly)
    assert 'guide' not in model
    assert dynamic.parts['guide'].length==2000
    assert analysis['rigid_sliding_guides']==['guide']


def test_moving_person_does_not_force_a_stiff_frame_into_beam_elements(load):
    from pipesim.beam_dynamics import prepare_beams
    assembly=load('human-pull-up');joints=copy.deepcopy(assembly.joints)
    dynamic,model,analysis=prepare_beams(assembly)
    assert analysis['status']=='mechanism'
    assert analysis['beam_selection_reference']=='temporarily_locked_pose'
    assert not model
    assert dynamic is assembly
    assert assembly.joints==joints


def cantilever(load, factory, *, height=1000, ground=False, platform=False, overload=False):
    doc = copy.deepcopy(load('cantilever').doc)
    doc['parts'][0]['parameters']['length_mm'] = 5000
    doc['parts'][0]['pose']['position_mm'] = [2500, 0, height]
    doc['anchors'][0]['position_mm'] = [0, 0, height]
    doc['loads'] = ([{'part':'beam','at_mm':5000,'force_n':[0,0,-20000]}]
                    if overload else [])
    doc['environment'] = {'ground':ground, 'gravity_m_s2':[0,0,-9.81]}
    if platform:
        doc['parts'].append({'id':'platform','pose':{'position_mm':[4500,0,height-130]},
            'body':{'kind':'rigid','mass_kg':20,
                    'geometry':[{'type':'box','size_mm':[1000,500,100]}]}})
        doc['anchors'].append({'part':'platform','surface':'floor'})
    return factory(doc)


def beam_positions(result, frame=-1):
    ids=result['beam_model']['beam']['segments']
    return np.array([result['frames'][frame]['parts'][pid]['position_mm'] for pid in ids])


def test_deflection_warning_ignores_whole_beam_motion():
    original=[]
    for x in (0,100,200):
        matrix=np.eye(4);matrix[0,3]=x;original.append(matrix)
    moved=np.eye(4);moved[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
    moved[:3,3]=[500,200,-20]
    current=[moved@matrix for matrix in original]
    assert _beam_deflection_mm(original,current)<1e-8
    current[-1]=current[-1].copy();current[-1][0,3]+=5
    assert _beam_deflection_mm(original,current)==pytest.approx(5)


def test_long_cantilever_sags_in_free_space(load, factory):
    assembly=cantilever(load,factory,ground=False)
    predicted=analyse(assembly)['members'][0]['max_displacement_mm']
    result=simulate(assembly,3,5)
    initial=beam_positions(result,0); final=beam_positions(result)
    assert len(final)>=10
    assert initial[-1,2]-final[-1,2]>70
    assert abs(final[-1,1])<5
    assert result['frames'][-1]['beam_status']['beam']['deflection_mm']>70
    assert not result['frames'][-1]['beam_status']['beam']['yielded']
    assert .7*predicted<result['frames'][-1]['beam_status']['beam']['deflection_mm']<1.3*predicted
    assert result['final_max_speed_m_s']<.25


def test_preview_cantilever_preserves_elastic_deflection_with_fewer_elements(load,factory):
    from pipesim.math3d import point,transform
    assembly=cantilever(load,factory,ground=False)
    predicted=analyse(assembly)['members'][0]['max_displacement_mm']
    result=simulate(assembly,3,5,mode='preview')
    assert result['quality'].get('resolved_mode')!='full'
    assert len(result['beam_model']['beam']['segments'])<10
    status=result['frames'][-1]['beam_status']['beam']
    assert not status['yielded']
    def tip(recording,index):
        model=recording['beam_model']['beam']
        return point(transform(recording['frames'][index]['parts'][model['segments'][-1]]),
                     [0,0,model['segment_length_mm']/2])
    # Compare the same physical endpoint. Element-centre displacement samples
    # different locations when Preview changes the mesh resolution.
    preview_tip=tip(result,-1)
    assert .7*predicted<np.linalg.norm(preview_tip-tip(result,0))<1.3*predicted
    full=simulate(assembly,3,5)
    assert np.linalg.norm(preview_tip-tip(full,-1))<.2*np.linalg.norm(tip(full,-1)-tip(full,0))
    assert result['final_max_speed_m_s']<.25


def test_long_cantilever_contacts_ground(load, factory):
    result=simulate(cantilever(load,factory,height=120,ground=True),1,10)
    final=beam_positions(result)
    assert 18<final[-1,2]<70
    assert abs(final[-1,1])<5
    assert any('world' in contact['a']+contact['b'] for contact in result['frames'][-1]['contacts'])


def test_metre_scale_linear_prediction_settles_on_ground_without_false_yield(load,factory):
    assembly=cantilever(load,factory,height=120,ground=True)
    assembly.doc['parts'][0]['parameters']['length_mm']=10000
    assembly.doc['parts'][0]['pose']['position_mm']=[5000,0,120]
    assembly=factory(assembly.doc)
    structural=analyse(assembly)['members'][0]
    assert structural['max_displacement_mm']>2000
    assert structural['yield_utilisation']>1
    result=simulate(assembly,1,10)
    status=result['frames'][-1]['beam_status']['beam']
    assert 18<beam_positions(result)[-1,2]<70
    assert status['deflection_mm']>90
    assert not status['yielded']


def test_long_cantilever_lands_on_other_geometry(load, factory):
    free=simulate(cantilever(load,factory,height=500,ground=False),1,10)
    supported=simulate(cantilever(load,factory,height=500,ground=False,platform=True),1,10)
    assert beam_positions(supported)[-1,2]>beam_positions(free)[-1,2]+20
    assert any('platform' in contact['a']+contact['b']
               for frame in supported['frames'] for contact in frame['contacts'])


def supported_span(load, factory, *, height=1000, platform=False):
    doc=copy.deepcopy(load('cantilever').doc)
    doc['parts'][0]['parameters']['length_mm']=10000
    doc['parts'][0]['pose']['position_mm']=[5000,0,height]
    doc['parts'] += [
        {'id':'left','pose':{'position_mm':[0,0,height]},'body':{'kind':'rigid','mass_kg':20,'geometry':[{'type':'box','size_mm':[80,80,80]}]}},
        {'id':'right','pose':{'position_mm':[10000,0,height]},'body':{'kind':'rigid','mass_kg':20,'geometry':[{'type':'box','size_mm':[80,80,80]}]}}]
    doc['anchors']=[{'part':'left','surface':'wall'},{'part':'right','surface':'wall'}]
    doc['joints']=[
        {'id':'left-joint','type':'fixed','a':{'part':'left'},'b':{'part':'beam','end':'start'}},
        {'id':'right-slide','type':'prismatic','a':{'part':'right','frame':{'axis':[1,0,0]}},
         'b':{'part':'beam','end':'end'},'limits':{'slide_mm':[-1000,1000]}}]
    doc['loads']=[];doc['environment']={'ground':height<200,'gravity_m_s2':[0,0,-9.81]}
    if platform:
        doc['parts'].append({'id':'platform','pose':{'position_mm':[5000,0,height-100]},
            'body':{'kind':'rigid','mass_kg':20,'geometry':[{'type':'box','size_mm':[1200,600,100]}]}})
        doc['anchors'].append({'part':'platform','surface':'floor'})
    return factory(doc)


def test_long_span_has_nonzero_static_midspan_sag(load, factory):
    report=analyse(supported_span(load,factory))
    assert report['status']=='solved'
    assert report['members'][0]['max_displacement_mm']>25


def test_large_axial_extension_is_not_misclassified_as_bending(load,factory):
    from pipesim.beam_dynamics import prepare_beams
    assembly=cantilever(load,factory,ground=False)
    assembly.doc['environment']['gravity_m_s2']=[0,0,0]
    assembly.doc['loads']=[{'part':'beam','at_mm':5000,'force_n':[10000000,0,0]}]
    assembly=factory(assembly.doc)
    assert analyse(assembly)['members'][0]['max_displacement_mm']>1
    _,model,_=prepare_beams(assembly)
    assert model=={}


def test_midspan_fitting_moves_with_the_bent_segment(load,factory):
    assembly=cantilever(load,factory,ground=False)
    assembly.doc['parts'].append({'id':'payload','pose':{'position_mm':[2500,0,1000]},
        'body':{'kind':'rigid','mass_kg':8,'geometry':[{'type':'box','size_mm':[80,80,80]}]}})
    assembly.doc['joints'].append({'id':'payload-mount','type':'fixed',
        'a':{'part':'beam','at_mm':2500},'b':{'part':'payload'}})
    result=simulate(factory(assembly.doc),.5,10)
    frame=result['frames'][-1]
    segment=result['beam_model']['beam']['segments'][len(result['beam_model']['beam']['segments'])//2]
    assert np.linalg.norm(np.array(frame['parts']['payload']['position_mm'])-
                          np.array(frame['parts'][segment]['position_mm']))<1
    assert frame['parts']['payload']['position_mm'][2]<990


@pytest.mark.parametrize('height,platform',[(1000,False),(120,False),(200,True)])
def test_long_span_sags_between_supported_ends(load,factory,height,platform):
    result=simulate(supported_span(load,factory,height=height,platform=platform),1,10)
    positions=beam_positions(result)
    midpoint=positions[len(positions)//2,2]
    assert midpoint<height-10
    assert abs(positions[len(positions)//2,1])<5
    if height==120: assert midpoint>18
    if platform: assert any('platform' in c['a']+c['b'] for c in result['frames'][-1]['contacts'])


def test_overloaded_cantilever_reports_permanent_damage(load,factory):
    result=simulate(cantilever(load,factory,ground=False,overload=True),.15,10)
    assert any(e['type']=='beam_yield' and e['part']=='beam' for e in result['events'])
    assert result['frames'][-1]['beam_status']['beam']['yielded']
    assert result['beam_analysis']['members'][0]['yield_utilisation']>1


def test_recorded_beam_renders_bent_elements_in_export(load,factory,tmp_path):
    from pipesim.rendering import render_video
    assembly=cantilever(load,factory,ground=False)
    result=simulate(assembly,.1,10,deflection_warning_mm=.1)
    destination=tmp_path/'beam.png'
    render_video(assembly,destination,recording=result,width=160,height=120)
    assert (tmp_path/'beam'/'frame-00000.png').is_file()
    assert (tmp_path/'beam'/'frame-00001.png').is_file()
