"""Prepared groups stay identifiable and movable through the build book."""
from collections import Counter
import copy
import html
import json
import re

import numpy as np
from PIL import Image
import pybullet as pb
import pytest

from pipesim.document import Assembly
from pipesim.exporting import build_export
from pipesim.dimensions import format_dimension_text
from pipesim.planning import PLANNING_ALGORITHM_VERSION, plan_build
from pipesim.math3d import pose_of
from pipesim.rendering import Renderer
from planning_cases import FIXTURES, fixture


@pytest.fixture(scope='module')
def anchored_cube(library):
    doc = fixture('cube')
    # Anchor every corner so at least one prepared cap contains several mounts.
    # Mounting is a final placement operation, never a bench preparation step.
    doc['anchors'] = [{'part': part['id'], 'surface': 'fixture'}
                      for part in doc['parts']
                      if part.get('catalog') != 'tubeclamp.tube-C']
    assembly = Assembly.from_doc(doc, FIXTURES, library)
    plan = plan_build(assembly)
    assert plan['status'] == 'buildable', plan
    assembly.doc['build_plan'] = plan
    return assembly, plan


def test_cube_mounts_are_deferred_until_the_group_reaches_main(anchored_cube):
    assembly, plan = anchored_cube
    mounts = Counter()
    merged_mounts = []
    prepared = {entry['id']: set(entry['parts']) for entry in plan['subassemblies']}
    for step in plan['steps']:
        anchors = step.get('world_anchors', [])
        if step['subassembly'] != 'main':
            assert not anchors and step['world_anchor'] is None, step
            assert set(step['poses'])==set(step['installed_parts'])
            if step['manual_support']:
                assert step['kind']=='lift' and set(step['held_parts'])==set(step['moving_parts'])
            else:
                assert step['grounded_after']
                assert 'floor' in step['instruction'].lower(), step
        for anchor in anchors:
            assert anchor['part'] in step['moving_parts'], step
            assert anchor['part'] in step['installed_parts'], step
            mounts[anchor['part']] += 1
        if step['kind'] == 'merge':
            assert prepared[step['prepared_subassembly']] == set(step['moving_parts'])
            assert step['prepared_subassembly'] in step['instruction']
            assert 'together' in step['instruction']
            if len(anchors) > 1:
                merged_mounts.append(step)
    assert merged_mounts, 'The case must exercise a cap with several deferred mounts'
    assert mounts == Counter({anchor['part']: 1 for anchor in assembly.anchors})


def test_group_renderer_highlights_every_moving_part_and_keeps_single_id_api(factory):
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'parts': [{'id': pid, 'catalog': 'generic.box',
                      'parameters': {'width_mm': 50, 'depth_mm': 50,
                                     'height_mm': 50, 'mass_kg': 1},
                      'pose': {'position_mm': position}}
                     for pid, position in [('a', [-100, 0, 25]),
                                           ('b', [100, 0, 25]),
                                           ('fixed', [0, 120, 25])]],
           'joints': []}
    assembly = factory(doc)
    with Renderer(assembly, width=240, height=180, lighting='flat') as renderer:
        image = renderer.frame(highlight=['a', 'b'], arrow=[1, 0, 0])
        def color(pid):
            body = renderer.world.part_map[pid][0]
            return pb.getVisualShapeData(body, physicsClientId=renderer.world.client)[0][7]
        assert color('a') == pytest.approx(color('b'))
        assert color('a') != pytest.approx(color('fixed'))
        assert color('a')[:3] == pytest.approx(np.array([238, 162, 76])/255)
        assert np.asarray(image)[:, :, :3].std() > 10
        renderer.frame(highlight='a', arrow=[1, 0, 0])
        assert color('a') != pytest.approx(color('b'))
        assert color('b') == pytest.approx(color('fixed'))


@pytest.mark.parametrize('orthographic', [False, True])
def test_lift_detail_focus_preserves_distant_visible_geometry(factory, orthographic):
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'parts': [{'id': pid, 'catalog': 'generic.box',
                      'parameters': {'width_mm': 100, 'depth_mm': 100,
                                     'height_mm': 100, 'mass_kg': 1},
                      'pose': {'position_mm': [x, 0, 50]}}
                     for pid, x in [('moving', 0), ('retained', 10000), ('hidden', -12000)]],
           'joints': []}
    assembly = factory(doc)
    poses = {'moving': {'position_mm': [500, 700, 50]}}
    with Renderer(assembly, width=320, height=240, background='transparent',
                  lighting='flat', orthographic=orthographic) as renderer:
        renderer.frame(poses=poses, visible=['moving', 'retained'], highlight='moving', fit_visible=True)
        full_width = abs(renderer.project([450, 700, 50])[0]-renderer.project([550, 700, 50])[0])
        renderer.frame(poses=poses, visible=['moving', 'retained'], highlight='moving',
                       fit_visible=True, fit_parts=['moving', 'hidden'], floor_z_mm=0)
        detail_width = abs(renderer.project([450, 700, 50])[0]-renderer.project([550, 700, 50])[0])
        assert detail_width > max(50, full_width*20)
        assert renderer._frame_bounds == pytest.approx(np.array([[450, 650, 0], [550, 750, 100]]))
        for pid, position in [('moving', [500, 700, 50]), ('retained', [10000, 0, 50])]:
            body = renderer.world.part_map[pid][0]
            actual, _ = pb.getBasePositionAndOrientation(body, physicsClientId=renderer.world.client)
            assert np.array(actual)*1000 == pytest.approx(position)
            assert pb.getVisualShapeData(body, physicsClientId=renderer.world.client)[0][7][3] == 1


def test_cube_book_renders_whole_groups_and_mounts_each_corner_once(
        anchored_cube, tmp_path, monkeypatch):
    assembly, plan = anchored_cube
    calls = []
    class RecordingRenderer:
        def __init__(self, *args, **kwargs): pass
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def frame(self, **kwargs):
            calls.append(kwargs)
            return Image.new('RGB', (32, 32), 'white')
    monkeypatch.setattr('pipesim.rendering.Renderer', RecordingRenderer)
    monkeypatch.setattr('pipesim.engineering_plot.layout_plot',
                        lambda assembly, path: Image.new('RGB', (32, 32)).save(path))
    report = build_export(assembly, tmp_path, illustrations=True)
    displayed=json.loads((tmp_path/'build-plan.json').read_text(encoding='utf-8'))['instruction_steps']
    assert report['steps'] == len(displayed)
    assert report['physical_steps'] == len(plan['steps'])
    assert calls[0] == {}
    rendered=iter(calls[1:])
    for step in displayed:
        if step['kind']=='lift' and step.get('start_poses'):
            before=next(rendered)
            assert before['poses']=={**step.get('poses',{}),**step['start_poses']}
        call=next(rendered)
        assert set(call['highlight']) == set(step['moving_parts'])
        assert set(call['visible']) == set(step['installed_parts'])
        expected_arrow = step['approach_direction'] if step['kind'] not in ('lift','position') and any(step['start_offset_mm']) else None
        assert call['arrow'] == expected_arrow
        assert call['fit_visible'] is True
        assert call['poses']==step.get('poses')
        assert call['floor_z_mm']==(step.get('floor_z_mm',0) if step.get('poses') else None)
        assert (tmp_path/f"step-{step['image_number']:03}.png").is_file()
    assert next(rendered,None) is None
    content = (tmp_path/'instructions.html').read_text(encoding='utf-8')
    for anchor in assembly.anchors:
        assert content.count('Fix '+html.escape(anchor['part'])+' to the declared') == 1
    for step in displayed:
        assert html.escape(format_dimension_text(step['instruction'])) in content
    combined=[step for step in displayed if len(step['source_steps'])>1]
    assert combined, 'Cube exercises ordinary loose-part placement and turning'
    pages=re.findall(r'<section>(.*?)</section>',content,re.S)
    for step in combined:
        page=next(page for page in pages if f'class="step-number">{step["number"]:02}</div>' in page)
        assert step['kind']=='position'
        assert html.escape(step['display_title']) in page
        assert f'src="step-{step["image_number"]:03}.png"' in page
        assert 'pose-pair' not in page and 'Lifted assembly:' not in page
    assert 'highlighted moving parts' in content


def test_large_prepared_group_has_compact_printed_summary(anchored_cube, tmp_path, library):
    """The user's 81-part side frame must not become a page-long ID list."""
    _, cube_plan = anchored_cube
    assembly = Assembly.from_doc(fixture('complicated-metal'), FIXTURES, library)
    step = copy.deepcopy(next(step for step in cube_plan['steps'] if step['kind'] == 'merge'))
    moving = sorted(assembly.parts)[:81]
    step.update(number=1, part=moving[0], moving_parts=moving,
                installed_parts=sorted(assembly.parts), target_parts=[],
                subassembly='main', prepared_subassembly='side-frame-A',
                instruction='Support side-frame-A and slide all 81 parts together onto main along the shown direction.',
                world_anchors=[], world_anchor=None, fasten=[], fixture=None)
    step['poses']={pid:pose_of(part.matrix) for pid,part in assembly.parts.items()}
    # Isolate the presentation contract from the separately tested search.
    assembly.doc['build_plan'] = {'status': 'buildable', 'input_sha256': assembly.input_hash,
                                  'steps': [step],'certificate':{'algorithm_version':PLANNING_ALGORITHM_VERSION}}
    build_export(assembly, tmp_path, illustrations=False)
    content = (tmp_path/'instructions.html').read_text(encoding='utf-8')
    sections = re.findall(r'<section(?:\s[^>]*)?>(.*?)</section>', content, re.S)
    stage = next(section for section in sections if 'class="step-number"' in section)
    notes = re.findall(r'<p class="note">(.*?)</p>', stage, re.S)
    summary = next(note for note in notes if 'together' in note)
    assert '81' in summary and 'side-frame-A' in stage
    assert len(re.sub('<[^>]*>', '', html.unescape(summary))) < 500, summary
    assert sum(html.escape(pid) in summary for pid in moving) <= 8, summary


def test_prepared_group_positioning_is_not_labelled_as_a_merge(
        anchored_cube, tmp_path, monkeypatch):
    """A prepared group can be staged beside other parts before it connects."""
    assembly, plan = anchored_cube
    step = copy.deepcopy(next(step for step in plan['steps'] if step['kind'] == 'merge'))
    step.update(number=1, kind='position', subassembly='subassembly-31',
                prepared_subassembly='subassembly-32', manual_support=True,
                instruction='Position subassembly-32 at the shown alignment; support it separately until a later step connects it.',
                fasten=[], world_anchors=[], world_anchor=None, fixture=None)
    step['poses']={pid:pose_of(assembly.parts[pid].matrix) for pid in step['installed_parts']}
    monkeypatch.setitem(assembly.doc, 'build_plan', {
        'status': 'buildable', 'input_sha256': assembly.input_hash, 'steps': [step],
        'certificate':{'algorithm_version':PLANNING_ALGORITHM_VERSION}})
    build_export(assembly, tmp_path, illustrations=False)
    content = (tmp_path/'instructions.html').read_text(encoding='utf-8')
    sections = re.findall(r'<section(?:\s[^>]*)?>(.*?)</section>', content, re.S)
    stage = next(section for section in sections if 'class="step-number"' in section)
    assert '<h2>Position Subassembly 32</h2>' in stage
    assert 'Merge Subassembly 32' not in stage
    assert html.escape(step['instruction']) in stage
    assert f'{len(step["moving_parts"])} highlighted parts' in stage
    assert '<li>' not in stage, 'Staging must not invent a fastening or mounting step'


def test_lift_book_shows_actual_before_and_after_poses(anchored_cube,tmp_path,monkeypatch):
    assembly,plan=anchored_cube
    step=copy.deepcopy(next(step for step in plan['steps'] if step['kind']=='merge'))
    poses={pid:copy.deepcopy(assembly.parts[pid].spec.get('pose',{})) for pid in step['installed_parts']}
    for pose in poses.values():
        pose.setdefault('position_mm',[0,0,0]);pose.setdefault('rotation_deg',[0,0,0])
    start=copy.deepcopy(poses)
    for pid in step['moving_parts']:
        start[pid]['position_mm']=[100,200,50]
        start[pid]['rotation_deg']=[90,0,0]
    step.update(number=1,kind='lift',prepared_subassembly='subassembly-32',
                poses=poses,start_poses=start,floor_z_mm=0,
                instruction='Lift and turn the connected group from its resting pose.',
                fasten=[],world_anchors=[],world_anchor=None,fixture=None)
    monkeypatch.setitem(assembly.doc,'build_plan',{
        'status':'buildable','input_sha256':assembly.input_hash,'steps':[step],
        'certificate':{'algorithm_version':PLANNING_ALGORITHM_VERSION}})
    calls=[]
    class RecordingRenderer:
        def __init__(self,*args,**kwargs):pass
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def frame(self,**kwargs):
            calls.append(kwargs)
            return Image.new('RGB',(32,32),'white')
    monkeypatch.setattr('pipesim.rendering.Renderer',RecordingRenderer)
    monkeypatch.setattr('pipesim.engineering_plot.layout_plot',
                        lambda assembly,path:Image.new('RGB',(32,32)).save(path))
    build_export(assembly,tmp_path,illustrations=True)
    assert len(calls)==3
    assert calls[1]['poses']==start and calls[2]['poses']==poses
    assert calls[1]['floor_z_mm']==calls[2]['floor_z_mm']==0
    assert calls[2]['arrow'] is None
    assert (tmp_path/'step-001-before.png').exists()
    assert (tmp_path/'step-001.png').exists()
    content=(tmp_path/'instructions.html').read_text(encoding='utf-8')
    assert '<h2>Lift Subassembly 32</h2>' in content
    assert 'Before lifting' in content and 'Lifted position' in content
    assert 'Lift and turn' in content and 'Lifted assembly:' in content


def test_book_discloses_assisted_fallback_and_required_clear_space(anchored_cube,tmp_path,monkeypatch):
    assembly,original=anchored_cube
    plan=copy.deepcopy(original)
    plan['certificate'].update(fallback_used=True,floor_work_area_mm=[12123,8345],
                               maximum_overhead_height_mm=4321)
    monkeypatch.setitem(assembly.doc,'build_plan',plan)
    build_export(assembly,tmp_path,illustrations=False)
    content=(tmp_path/'instructions.html').read_text(encoding='utf-8')
    assert 'Largest handling operation:' in content
    assert 'lighter alternatives may exist' in content
    assert '12.2 × 8.4 m' in content
    assert 'Clear height above the floor: 4.4 m' in content


@pytest.mark.parametrize('version,has_poses',[(2,True),(PLANNING_ALGORITHM_VERSION,False)])
def test_export_replans_cached_sequences_without_current_gravity_poses(
        anchored_cube,tmp_path,monkeypatch,version,has_poses):
    assembly,original=anchored_cube
    current=copy.deepcopy(original)
    current.setdefault('certificate',{})['algorithm_version']=PLANNING_ALGORITHM_VERSION
    for step in current['steps']:
        step['poses']={pid:pose_of(assembly.parts[pid].matrix) for pid in step['installed_parts']}
    stale=copy.deepcopy(current)
    stale['certificate']['algorithm_version']=version
    if not has_poses:
        for step in stale['steps']:step.pop('poses',None)
    monkeypatch.setitem(assembly.doc,'build_plan',stale)
    calls=[]
    def fresh_plan(value):
        calls.append(value)
        return current
    monkeypatch.setattr('pipesim.exporting.plan_build',fresh_plan)
    build_export(assembly,tmp_path,illustrations=False)
    assert calls==[assembly]
    exported=json.loads((tmp_path/'build-plan.json').read_text(encoding='utf-8'))
    assert exported['certificate']['algorithm_version']==PLANNING_ALGORITHM_VERSION
    assert all(step['poses'] for step in exported['steps'])


def test_preparation_renderer_shows_depth_tested_floor_and_resting_pose(factory):
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','environment':{'ground':False},
         'parts':[{'id':'part','catalog':'generic.box',
                   'parameters':{'width_mm':100,'depth_mm':100,'height_mm':100,'mass_kg':1},
                   'pose':{'position_mm':[0,0,3000]}}],'joints':[]}
    assembly=factory(doc)
    with Renderer(assembly,width=320,height=240,background='transparent',lighting='flat') as renderer:
        # Separate preparation areas can be far outside the design overview.
        posed=renderer.frame(poses={'part':{'position_mm':[150000,-250000,50]}},
                             visible=['part'],highlight=['part'],fit_visible=True,floor_z_mm=0)
        pixels=np.asarray(posed)
        floor_lines=(pixels[:,:,0]<223)&(pixels[:,:,1]>pixels[:,:,0]+8)&(pixels[:,:,2]>=pixels[:,:,1]+3)
        assert np.count_nonzero(floor_lines)>100
        assert np.count_nonzero(np.all(pixels[:,:,:3]==[238,162,76],axis=2))>500
        assert np.count_nonzero(pixels[-10:,:,3])>3000
        assert renderer.world.ground is None and renderer._floor_body is not None
        unposed=renderer.frame()
        assert np.count_nonzero(np.asarray(unposed)[-10:,:,3])==0


@pytest.mark.parametrize('orthographic', [False, True])
def test_visible_camera_fits_small_and_posed_steps_then_restores_overview(factory, orthographic):
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'parts': [{'id': pid, 'catalog': 'generic.box',
                      'parameters': {'width_mm': 100, 'depth_mm': 100,
                                     'height_mm': 100, 'mass_kg': 1},
                      'pose': {'position_mm': [x, 0, 50]}}
                     for pid, x in [('foot', -3000), ('distant', 3000)]],
           'joints': []}
    assembly = factory(doc)
    with Renderer(assembly, width=320, height=240, background='transparent',
                  lighting='flat', orthographic=orthographic) as renderer:
        original_view = np.array(renderer.view)
        original_projection = np.array(renderer.projection)
        small = renderer.frame(visible=['foot'], highlight='foot')
        before_width = abs(renderer.project([-3050, 0, 50])[0]-renderer.project([-2950, 0, 50])[0])
        fitted = renderer.frame(visible=['foot'], highlight=['foot'], fit_visible=True)
        small_pixels = np.count_nonzero(np.asarray(small)[:, :, 3])
        fitted_pixels = np.count_nonzero(np.asarray(fitted)[:, :, 3])
        if not orthographic:
            assert fitted_pixels > max(1000, small_pixels*10)
        after_width = abs(renderer.project([-3050, 0, 50])[0]-renderer.project([-2950, 0, 50])[0])
        assert after_width > max(50, before_width*10)
        initial_rotation = original_view.reshape(4, 4).T[:3, :3]
        assert np.allclose(np.array(renderer.view).reshape(4, 4).T[:3, :3],
                           initial_rotation, atol=1e-5)
        posed = [9000, 12000, 5000]
        renderer.frame(poses={'foot': {'position_mm': posed,
                                      'rotation_deg': [20, 30, 40]}},
                       visible=['foot'], highlight=['foot'], arrow=[1, 0, 0],
                       fit_visible=True)
        for point in (posed, np.array(posed)-[180, 0, 0], np.array(posed)-[40, 0, 0]):
            x, y = renderer.project(point)
            assert 5 < x < renderer.width-5 and 5 < y < renderer.height-5
        renderer.frame(visible=['foot'])
        assert np.array_equal(renderer.view, original_view)
        assert np.array_equal(renderer.projection, original_projection)
