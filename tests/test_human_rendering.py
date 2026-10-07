"""Headless regression: cross-renderer skinning, actual images and animations."""
import copy
import base64
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
from PIL import Image
from scipy.spatial.transform import Rotation
from pipesim.document import Assembly, DocumentError
from pipesim.human_rendering import HumanSkin
from pipesim.math3d import transform
from pipesim.rendering import Renderer, render_image, render_video, find_ffmpeg
from pipesim.human_assets import _read_container

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def rigs(tmp_path_factory):
    directory=tmp_path_factory.mktemp('headless-rigs')
    result=subprocess.run(['node',str(ROOT/'tests/helpers/headless-human-fixtures.mjs'),str(directory)],
                          cwd=ROOT,env={**os.environ,'PYTHON':sys.executable},capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    return directory,json.loads((directory/'reference.json').read_text())


def assembly(directory,case,enabled=True):
    config=copy.deepcopy(case['human']['render_model']); config['enabled']=enabled
    return Assembly.from_doc({'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[],'joints':[],
        'objects':[{'id':'person','template':'human','parameters':{'pose':'hands-up'},'render_model':config}],
        'environment':{'ground':False}},directory)


@pytest.mark.parametrize('variant',['mixamo','blender','rotated','vrm'])
def test_headless_matches_browser_bones_and_every_weighted_vertex(rigs,variant):
    directory,cases=rigs; case=next(c for c in cases if c['variant']==variant)
    skin=HumanSkin(directory/(variant+'.glb'),case['human'])
    # Match the browser constructor's initial standing frame.
    skin.update({pid:transform(pose) for pid,pose in case['frames'][0]['poses'].items()},0)
    for frame in case['frames']:
        skin.update({pid:transform(pose) for pid,pose in frame['poses'].items()},frame['time_s'])
        for segment,matrix in frame['bones'].items():
            np.testing.assert_allclose(skin.world[skin.mapping[segment]],np.array(matrix).reshape(4,4).T,atol=2e-4,err_msg=variant+'/'+frame['name']+'/'+segment)
        for primitive,mesh in zip(skin.primitives,frame['meshes'],strict=True):
            np.testing.assert_allclose(skin.vertices(primitive),mesh['vertices'],atol=3e-4,err_msg=variant+'/'+frame['name'])


def test_headless_mapped_bone_offsets_match_post_retarget_local_rotation(rigs):
    directory,cases=rigs; case=copy.deepcopy(next(c for c in cases if c['variant']=='mixamo'))
    poses={pid:transform(pose) for pid,pose in case['frames'][0]['poses'].items()}
    baseline=HumanSkin(directory/'mixamo.glb',case['human']); baseline.update(poses,0)
    angles=[15,-10,25];case['human']['render_model']['bone_offsets']={'left_clavicle':angles}
    adjusted=HumanSkin(directory/'mixamo.glb',case['human']);adjusted.update(poses,0)
    i=adjusted.mapping['left_clavicle']
    expected=baseline.world[i,:3,:3]@Rotation.from_euler('XYZ',angles,degrees=True).as_matrix()
    np.testing.assert_allclose(adjusted.world[i,:3,:3],expected,atol=2e-6)


@pytest.mark.parametrize('variant',['mixamo','blender'])
def test_render_images_and_toggle_preserve_model_configuration(rigs,tmp_path,variant):
    directory,cases=rigs; case=next(c for c in cases if c['variant']==variant)
    a=assembly(directory,case); before=copy.deepcopy(a.doc)
    options=dict(width=320,height=320,lighting='flat',background='transparent')
    imported=tmp_path/'imported.png'; simple=tmp_path/'simple.png'; restored=tmp_path/'restored.png'
    render_image(a,imported,**options)
    render_image(assembly(directory,case,False),simple,**options)
    render_image(a,restored,**options)
    skin=np.array(Image.open(imported)); mannequin=np.array(Image.open(simple))
    assert skin[:,:,3].min()==0 and skin[:,:,3].max()==255
    assert np.count_nonzero(skin[:,:,3])>500
    assert np.mean(np.abs(skin.astype(float)-mannequin))>2
    np.testing.assert_array_equal(skin,np.array(Image.open(restored)))
    assert a.doc==before


def test_recorded_png_sequence_and_gif_use_skins_and_timestamps(rigs,tmp_path):
    directory,cases=rigs; case=cases[0]; a=assembly(directory,case)
    frames=[{'time_s':f['time_s'],'parts':f['poses']} for f in case['frames'] if f['name'].startswith('simulation')]
    recording={'fps':10,'frames':frames}
    options=dict(width=200,height=240,lighting='flat',background='transparent')
    render_video(a,tmp_path/'frames',recording,**options)
    render_video(a,tmp_path/'motion.gif',recording,**options)
    assert Image.open(tmp_path/'motion.gif').n_frames==len(frames)
    with Renderer(a,**options) as renderer:
        for i,frame in enumerate(frames):
            expected=renderer.frame(frame['parts'],time_s=frame['time_s'])
            np.testing.assert_array_equal(np.array(expected),np.array(Image.open(tmp_path/'frames'/f'frame-{i:05}.png')))
        assert renderer.skins[0].last_time==frames[-1]['time_s']


def test_enabled_missing_model_fails_export_instead_of_silent_mannequin(rigs,tmp_path):
    directory,cases=rigs; case=copy.deepcopy(cases[0]); case['human']['render_model']['file']='missing.glb'
    with pytest.raises(DocumentError,match='Cannot render imported human'):
        render_image(assembly(directory,case),tmp_path/'missing.png',width=100,height=100)
    render_image(assembly(directory,case,False),tmp_path/'disabled.png',width=100,height=100)
    assert (tmp_path/'disabled.png').exists()


def test_headless_preserves_uvs_and_renders_embedded_texture_colors(rigs,tmp_path):
    directory,cases=rigs; case=copy.deepcopy(cases[0])
    document,binary=_read_container((directory/'mixamo.glb').read_bytes(),'.glb')
    document['buffers'][0]['uri']='data:application/octet-stream;base64,'+base64.b64encode(binary).decode()
    pixels=np.zeros((64,64,4),dtype=np.uint8); pixels[:,:,3]=255
    pixels[:32,:32,:3]=[255,0,0]; pixels[:32,32:,:3]=[0,255,0]
    pixels[32:,:32,:3]=[0,0,255]; pixels[32:,32:,:3]=[255,255,0]
    encoded=io.BytesIO(); Image.fromarray(pixels).save(encoded,format='PNG')
    document['images']=[{'uri':'data:image/png;base64,'+base64.b64encode(encoded.getvalue()).decode()}]
    document['textures']=[{'source':0}]
    document['materials'][0]['pbrMetallicRoughness']={'baseColorFactor':[1,1,1,1],'baseColorTexture':{'index':0}}
    path=tmp_path/'textured.gltf'; path.write_text(json.dumps(document))
    case['human']['render_model']['file']=path.name
    skin=HumanSkin(path,case['human']); primitive=skin.primitives[0]
    original=skin.accessor(document['meshes'][0]['primitives'][0]['attributes']['TEXCOORD_0'])
    np.testing.assert_allclose(primitive['uv'][:,0],original[:,0])
    np.testing.assert_allclose(primitive['uv'][:,1],1-original[:,1])
    output=tmp_path/'textured.png'
    render_image(assembly(tmp_path,case),output,width=400,height=400,lighting='flat',background='transparent')
    rgba=np.array(Image.open(output)); colors=rgba[rgba[:,:,3]>0,:3]
    for color in ([255,0,0],[0,255,0],[0,0,255],[255,255,0]):
        assert np.count_nonzero(np.max(np.abs(colors.astype(int)-color),axis=1)<10)>30


def test_meshes_outside_the_default_gltf_scene_are_not_rendered(rigs,tmp_path):
    directory,cases=rigs; case=copy.deepcopy(cases[0])
    document,binary=_read_container((directory/'mixamo.glb').read_bytes(),'.glb')
    document['buffers'][0]['uri']='data:application/octet-stream;base64,'+base64.b64encode(binary).decode()
    count=len(document['nodes'])
    document['nodes'].append({'name':'Unused alternate character','mesh':0,'translation':[10,0,0]})
    document['scenes'].append({'nodes':[count]})
    path=tmp_path/'multiple-scenes.gltf'; path.write_text(json.dumps(document))
    skin=HumanSkin(path,case['human'])
    assert len(skin.primitives)==1
    assert count not in skin.active


def test_partial_build_illustration_hides_uninstalled_character_segments(rigs):
    directory,cases=rigs; a=assembly(directory,cases[0])
    with Renderer(a,width=200,height=240,background='transparent') as renderer:
        whole=np.array(renderer.frame())[:,:,3]
        head=np.array(renderer.frame(visible=['person/head']))[:,:,3]
        empty=np.array(renderer.frame(visible=[]))[:,:,3]
    assert 0<np.count_nonzero(head)<np.count_nonzero(whole)/2
    assert not np.count_nonzero(empty)


def test_cli_image_and_simulation_recording_export_use_imported_skin(rigs,tmp_path):
    import yaml
    directory,cases=rigs; a=assembly(directory,cases[0])
    doc=copy.deepcopy(a.doc)
    doc['objects'][0]['render_model']['file']=str(directory/'mixamo.glb')
    source=tmp_path/'design.pipe.yaml'; source.write_text(yaml.safe_dump(doc))
    destination=tmp_path/'cli.png'
    result=subprocess.run([sys.executable,'-m','pipesim','render',str(source),'-o',str(destination),
                           '--width','160','--height','200','--lighting','flat'],cwd=ROOT,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    expected=tmp_path/'expected.png'; render_image(a,expected,width=160,height=200,lighting='flat')
    np.testing.assert_array_equal(np.array(Image.open(destination)),np.array(Image.open(expected)))
    frames=[{'time_s':f['time_s'],'parts':f['poses']} for f in cases[0]['frames'] if f['name'].startswith('simulation')]
    recording=tmp_path/'recording.json'; recording.write_text(json.dumps({'fps':10,'frames':frames}))
    result=subprocess.run([sys.executable,'-m','pipesim','animate',str(source),'--simulation',str(recording),
                           '-o',str(tmp_path/'cli-frames'),'--width','160','--height','200','--lighting','flat'],
                          cwd=ROOT,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    with Renderer(a,width=160,height=200,lighting='flat') as renderer:
        for i,frame in enumerate(frames):
            np.testing.assert_array_equal(np.array(renderer.frame(frame['parts'],time_s=frame['time_s'])),
                np.array(Image.open(tmp_path/'cli-frames'/f'frame-{i:05}.png')))


def test_real_mp4_encoder_exports_and_decodes_imported_character_frames(rigs,tmp_path):
    encoder=find_ffmpeg()
    if not encoder: pytest.skip('FFmpeg is not available')
    directory,cases=rigs; a=assembly(directory,cases[0])
    frames=[{'time_s':f['time_s'],'parts':f['poses']} for f in cases[0]['frames'] if f['name'].startswith('simulation')]
    path=tmp_path/'character.mp4'
    render_video(a,path,{'fps':10,'frames':frames},width=160,height=200,lighting='flat')
    result=subprocess.run([encoder,'-v','error','-i',str(path),'-f','rawvideo','-pix_fmt','rgb24','-'],capture_output=True)
    assert result.returncode==0,result.stderr.decode(errors='replace')
    decoded=np.frombuffer(result.stdout,dtype=np.uint8).reshape(-1,200,160,3)
    assert len(decoded)==len(frames)
    with Renderer(a,width=160,height=200,lighting='flat') as renderer:
        for image,frame in zip(decoded,frames):
            expected=np.array(renderer.frame(frame['parts'],time_s=frame['time_s']))[:,:,:3]
            # H.264 is lossy, but must preserve the actual skinned frames.
            assert np.mean(np.abs(image.astype(float)-expected))<6
