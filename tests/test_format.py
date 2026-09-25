import copy
import json
import numpy as np
import pytest
from pipesim.document import Assembly,DocumentError,parse,write,read,Library
from pipesim.math3d import transform,pose_of,align_axis,segment_distance

def test_duplicate_yaml_rejected():
    with pytest.raises(DocumentError,match='Duplicate'): parse('format: pipesim/1\nformat: pipesim/2')
@pytest.mark.parametrize('value',['.nan','.inf','-.inf'])
def test_nonfinite_rejected(value):
    with pytest.raises(DocumentError,match='NaN'): parse('value: '+value)
def test_recursive_alias_rejected():
    with pytest.raises(DocumentError,match='Recursive'): parse('value: &a [*a]')
def test_unsafe_yaml_tag_rejected():
    with pytest.raises(DocumentError): parse('value: !!python/object/apply:os.system [echo bad]')
@pytest.mark.parametrize('change',[{'format':'pipesim/2'},{'units':'metres'},{'partz':[]}])
def test_version_units_unknown_keys(blank,factory,change):
    blank.update(change)
    with pytest.raises(DocumentError): factory(blank)
def test_roundtrip_yaml_and_json(load,tmp_path):
    a=load('workbench')
    for suffix in ('.yaml','.json'):
        path=tmp_path/('roundtrip'+suffix); write(path,a.doc); assert read(path)==a.doc

def test_simulation_json_keeps_small_coordinates_and_efforts_numeric(tmp_path):
    recording={'frames':[{'parts':{'hand':{'position_mm':[1e-8,-2e-7,2300]}},'motor_efforts':{'wrist':{'rx':{'torque_nm':5e-10}}}}]}
    path=tmp_path/'simulation.json';write(path,recording)
    assert read(path)==recording
    assert isinstance(read(path)['frames'][0]['parts']['hand']['position_mm'][0],float)

def test_duplicate_json_keys_are_rejected_and_flow_yaml_still_loads():
    with pytest.raises(DocumentError,match='Duplicate JSON'): parse('{"motor":{"target":0,"target":90}}')
    assert parse('{name: example, parts: []}')=={'name':'example','parts':[]}
def test_unknown_part(load,factory):
    doc=load('workbench').doc; doc['parts'][0]['catalog']='nonexistent'
    with pytest.raises(DocumentError,match='Unknown catalog'): factory(doc)
def test_duplicate_part(load,factory):
    doc=load('workbench').doc; doc['parts'].append(copy.deepcopy(doc['parts'][0]))
    with pytest.raises(DocumentError,match='Duplicate part'): factory(doc)
def test_unknown_socket(load,factory):
    doc=load('workbench').doc; doc['joints'][0]['a']['port']='nowhere'
    with pytest.raises(DocumentError,match='unknown port'): factory(doc)
def test_duplicate_joint(load,factory):
    doc=load('workbench').doc; doc['joints'].append(copy.deepcopy(doc['joints'][0]))
    with pytest.raises(DocumentError,match='Duplicate joint'): factory(doc)
def test_missing_joint_part(load,factory):
    doc=load('workbench').doc; doc['joints'][0]['b']['part']='missing'
    with pytest.raises(DocumentError,match='unknown part'): factory(doc)
def test_inline_library_override(load,factory):
    doc=load('cantilever').doc; part=copy.deepcopy(load('cantilever').library.parts['tubeclamp.tube-C']); part['parameters']['wall_mm']=4
    doc['definitions']={'tubeclamp.tube-C':part}; assert factory(doc).parts['beam'].section['wall_mm']==4
@pytest.mark.parametrize('angles',[[0,0,0],[30,40,50],[0,90,0],[180,0,0]])
def test_pose_roundtrip(angles):
    m=transform({'position_mm':[20,40,80],'rotation_deg':angles}); assert np.allclose(transform(pose_of(m)),m)
def test_axis_flip(): assert np.allclose(align_axis([0,0,1],[0,0,-1])@[0,0,1],[0,0,-1])
def test_segment_distance_parallel(): assert segment_distance([0,0,0],[1,0,0],[0,1,0],[1,1,0])[0]==pytest.approx(1)
def test_segment_distance_crossing(): assert segment_distance([-1,0,0],[1,0,0],[0,-1,0],[0,1,0])[0]==pytest.approx(0)
def test_library_has_real_drawing_sources(library):
    sourced=[p for k,p in library.parts.items() if k.startswith('tubeclamp.TC')]
    assert len(sourced)>=50
    assert all(p['source']['drawing_url'].startswith('https://cdn.shopify.com') for p in sourced)
    assert library.parts['tubeclamp.TC101C']['drawing_dimensions_mm']=={'f':56,'g':89,'h':59}
    assert library.parts['minitec.20.1006']['section']['ix_mm4']==159340
