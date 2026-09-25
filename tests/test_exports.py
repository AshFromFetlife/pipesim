import copy
import json
from pathlib import Path
import numpy as np
import pytest
import trimesh
from PIL import Image
from pipesim.document import Assembly,DocumentError,read
from pipesim.exporting import bom,cutting_plan,build_export
from pipesim.importing import import_mesh
from pipesim.packaging import bundle
from pipesim.rendering import render_image,animation_frames,render_video
from pipesim.motion import check_motion,longest_valid_clip

def test_cutting_stock_conserves_length_and_keeps_sections_separate(load):
    r=cutting_plan(load('workbench'),kerf_mm=3,stock_lengths_mm=[2400,6000],end_trim_mm=10)
    assert r['stock_total_mm']==pytest.approx(r['finished_total_mm']+r['kerf_total_mm']+r['trim_total_mm']+r['offcut_total_mm'])
    assert len([c for b in r['bars'] for c in b['cuts']])==4
    for b in r['bars']:
        assert b['remaining_mm']>=0
        for c,next_cut in zip(b['cuts'],b['cuts'][1:]): assert c['start_mm']+c['length_mm']+c['kerf_mm']==pytest.approx(next_cut['start_mm'])

def test_exact_stock_length_needs_no_fictitious_end_cut(blank,factory):
    blank['parts']=[{'id':'tube','catalog':'tubeclamp.tube-C','parameters':{'length_mm':6000}}]
    r=cutting_plan(factory(blank))
    assert r['stock_count']==1 and r['offcut_total_mm']==0 and r['kerf_total_mm']==0

def test_oversize_cut_is_rejected(blank,factory):
    blank['parts']=[{'id':'tube','catalog':'tubeclamp.tube-C','parameters':{'length_mm':6001}}]
    with pytest.raises(DocumentError,match='exceeds'): cutting_plan(factory(blank))

def test_bom_includes_panel_dimensions_and_real_supplier_links(load):
    result=bom(load('workbench'))
    panels=[r for r in result['items'] if 'panel_dimensions_mm' in r]
    assert panels[0]['panel_dimensions_mm']==[1400,800,30]
    assert any('tubeclamp.com.au' in r['supplier_url'] for r in result['items'])

def test_build_export_has_every_step_and_reloadable_bundled_design(load,tmp_path):
    a=load('workbench'); report=build_export(a,tmp_path,engineering=True,illustrations=False)
    assert report['steps']==13
    assert len(read(tmp_path/'build-plan.json')['steps'])==13
    b=Assembly.load(tmp_path/'design.pipe.yaml')
    for pid,p in a.parts.items():
        assert b.parts[pid].mass==pytest.approx(p.mass)
        assert b.parts[pid].length==pytest.approx(p.length)
    html=(tmp_path/'instructions.html').read_text(encoding='utf-8')
    assert '1400 × 800 × 30' in html
    assert 'Structural response' in html

def test_imported_mesh_and_custom_material_are_portable(blank,tmp_path):
    source=tmp_path/'original'; source.mkdir(); destination=tmp_path/'packed'
    mesh=trimesh.creation.box([100,200,300]); mesh.export(source/'box.stl')
    import_mesh(source/'box.stl',source/'library.yaml','custom.box',10)
    blank['libraries']=['library.yaml']; blank['parts']=[{'id':'box','catalog':'custom.box'}]
    a=Assembly.from_doc(blank,source); path=bundle(a,destination)
    # Move the supplied source files, proving the bundle no longer resolves them.
    (source/'assets').rename(source/'old-assets')
    b=Assembly.load(path)
    from pipesim.geometry import mesh_for_part
    assert np.allclose(mesh_for_part(b.parts['box']).extents,[100,200,300])
    assert b.parts['box'].mass==10

def test_cli_renderer_runs_without_display_and_has_foreground(load,tmp_path):
    path=tmp_path/'image.png'; render_image(load('workbench'),path,width=320,height=240,background='transparent',lighting='flat')
    pic=Image.open(path); assert pic.size==(320,240)
    assert pic.mode=='RGBA' and np.asarray(pic)[:,:,3].max()==255
    assert np.asarray(pic)[:,:,:3].std()>10

def test_animation_moves_the_named_joint_and_gif_has_multiple_frames(load,tmp_path):
    a=load('sliding-collar'); recording=animation_frames(a,duration=4,fps=1)
    assert recording['frames'][0]['parts']['slider']!=recording['frames'][2]['parts']['slider']
    path=tmp_path/'motion.gif'; render_video(a,path,recording,width=200,height=240,lighting='flat')
    assert Image.open(path).n_frames>=3

def test_motion_range_excludes_collar_intersecting_stop(load):
    a=load('sliding-collar'); recording=animation_frames(a,duration=4,fps=1,joint='loose-screw')
    report=check_motion(a,recording=recording)
    assert not report['valid'] and report['valid_intervals']
    clip=longest_valid_clip(recording,report)
    assert len(clip['frames'])<len(recording['frames'])
    assert any(i['code']=='INTERSECTION' for s in report['samples'] for i in s['issues'])
