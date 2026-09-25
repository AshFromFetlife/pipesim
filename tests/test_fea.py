import math
import numpy as np
import pytest
from pipesim.fea import analyse,section_properties,beam_stiffness,destruction_test

def test_cantilever_against_closed_form(load):
    a=load('cantilever');p=a.parts['beam'];s=section_properties(p);E=p.definition['material_data']['youngs_modulus_pa']
    r=analyse(a);assert r['status']=='solved';expected=1000/(3*E*s['iy'])*1000
    assert r['members'][0]['max_displacement_mm']==pytest.approx(expected,rel=1e-9)
    assert r['equilibrium_residual_n_nm']<1e-7
def test_axial_extension_closed_form(load,factory):
    a=load('cantilever');a.doc['loads'][0]['force_n']=[1000,0,0];a=factory(a.doc)
    s=section_properties(a.parts['beam']);expected=1000/(200e9*s['area'])*1000
    assert analyse(a)['members'][0]['max_displacement_mm']==pytest.approx(expected,rel=1e-8)
def test_end_moment_deflection(load,factory):
    a=load('cantilever');a.doc['loads'][0]['force_n']=[0,0,0];a.doc['loads'][0]['moment_nm']=[0,100,0];a=factory(a.doc)
    expected=100/(2*200e9*section_properties(a.parts['beam'])['iy'])*1000
    assert analyse(a)['members'][0]['max_displacement_mm']==pytest.approx(expected,rel=1e-8)
def test_torsion_closed_form(load,factory):
    a=load('cantilever');a.doc['loads'][0]['force_n']=[0,0,0];a.doc['loads'][0]['moment_nm']=[100,0,0];a=factory(a.doc)
    expected=math.degrees(100/((200e9/2.6)*section_properties(a.parts['beam'])['j']))
    r=analyse(a);assert r['nodes'][1]['rotation_deg'][0]==pytest.approx(expected,rel=1e-8)
def test_self_weight_uniform_load_closed_form(load,factory):
    a=load('cantilever');a.doc['loads']=[];a.doc['environment']['gravity_m_s2']=[0,0,-9.81];a=factory(a.doc)
    p=a.parts['beam'];expected=p.mass*9.81/(8*200e9*section_properties(p)['iy'])*1000
    assert analyse(a)['members'][0]['max_displacement_mm']==pytest.approx(expected,rel=1e-8)
def test_material_linear_scaling(load,factory):
    a=load('cantilever');first=analyse(a);a.doc['loads'][0]['force_n']=[0,0,-2000];second=analyse(factory(a.doc))
    assert second['members'][0]['max_displacement_mm']==pytest.approx(first['members'][0]['max_displacement_mm']*2)
def test_free_frame_reports_mechanism(load,factory):
    a=load('cantilever');a.doc['anchors']=[];assert analyse(factory(a.doc))['status']=='mechanism'
def test_overload_detected(load,factory):
    a=load('cantilever');a.doc['loads'][0]['force_n']=[0,0,-20000];r=analyse(factory(a.doc));assert any(f['mode']=='yield' for f in r['failures']);assert r['certified'] is False
def test_compressive_buckling(load,factory):
    a=load('cantilever');a.doc['loads'][0]['force_n']=[-50000,0,0];r=analyse(factory(a.doc));assert any(f['mode']=='euler_buckling' for f in r['failures'])
def test_unknown_connector_strength_stays_unknown(load):
    r=analyse(load('workbench'));assert r['status']=='solved';assert any(c['axial_slip_utilisation'] is None for c in r['connectors']);assert all('casting fracture' in c['unknown_modes'] for c in r['connectors'])
def test_frame_element_has_six_rigid_modes(load):
    s=section_properties(load('cantilever').parts['beam']);k=beam_stiffness(1,200e9,80e9,s)
    assert np.allclose(k,k.T);assert np.linalg.matrix_rank(k,tol=1e-5)==6
def test_destruction_directions_reproducible(load):
    a=load('cantilever');first=destruction_test(a,'beam',10000,8,2,123);second=destruction_test(a,'beam',10000,8,2,123)
    assert first==second;assert len(first['trials'])==8;assert any(t['first_failure'] for t in first['trials'])
