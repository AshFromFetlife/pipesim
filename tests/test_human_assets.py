import base64
import copy
import json
import struct
from pathlib import Path
import pytest
from pipesim.document import Assembly, DocumentError
from pipesim.human_assets import (MAX_ASSET_BYTES, import_human_model,
                                  prepare_import, validate_stored_asset, _auto_bone_map)


def encoded(value):
    return base64.b64encode(value).decode('ascii')


def skinned_fixture(vrm=False, external=False):
    """Two actual weighted rigs, with UVs, a texture and an extra hair bone."""
    names = ['Hips', 'Spine', 'Chest', 'Head', 'LeftArm', 'LeftForeArm',
             'RightArm', 'RightForeArm', 'LeftUpLeg', 'LeftLeg',
             'RightUpLeg', 'RightLeg', 'Hair']
    parents = [None, 0, 1, 2, 2, 4, 2, 6, 0, 8, 0, 10, 3]
    nodes = [{'name': name, 'translation': [0, .1 if i else .9, 0]}
             for i, name in enumerate(names)]
    for i, parent in enumerate(parents):
        if parent is not None: nodes[parent].setdefault('children', []).append(i)
    nodes.append({'name': 'Body mesh', 'mesh': 0, 'skin': 0})
    positions = struct.pack('<9f', -.1, .9, 0, .1, .9, 0, 0, 1.1, 0)
    weights = struct.pack('<12f', *([1, 0, 0, 0] * 3))
    joints = struct.pack('<12H', *([0, 0, 0, 0] * 3))
    uv = struct.pack('<6f', 0, 0, 1, 0, .5, 1)
    raw = positions + weights + joints + uv
    data = {'asset': {'version': '2.0'}, 'scene': 0, 'scenes': [{'nodes': [0, 13]}],
            'nodes': nodes, 'skins': [{'joints': list(range(13))}],
            'buffers': [{'byteLength': len(raw), 'uri': 'body.bin' if external else 'data:application/octet-stream;base64,' + encoded(raw)}],
            'bufferViews': [{'buffer': 0, 'byteOffset': offset, 'byteLength': size}
                            for offset, size in ((0, 36), (36, 48), (84, 24), (108, 24))],
            'accessors': [{'bufferView': i, 'componentType': 5123 if i == 2 else 5126,
                           'count': 3, 'type': kind}
                          for i, kind in enumerate(('VEC3', 'VEC4', 'VEC4', 'VEC2'))],
            'meshes': [{'primitives': [{'attributes': {'POSITION': 0, 'WEIGHTS_0': 1, 'JOINTS_0': 2, 'TEXCOORD_0': 3}, 'material': 0}]}],
            'images': [{'uri': 'albedo.png' if external else 'data:image/png;base64,' + encoded(PNG)}],
            'textures': [{'source': 0}],
            'materials': [{'pbrMetallicRoughness': {'baseColorTexture': {'index': 0}}}]}
    if vrm:
        data['extensions'] = {'VRMC_vrm': {'humanoid': {'humanBones': {
            'hips': {'node': 0}, 'spine': {'node': 1}, 'chest': {'node': 2}, 'head': {'node': 3}}}}}
    return data, raw


PNG = base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAusB9Wl6cZUAAAAASUVORK5CYII=')


def glb(data, binary=None):
    payload = json.dumps(data).encode()
    payload += b' ' * (-len(payload) % 4)
    chunks = struct.pack('<II', len(payload), 0x4E4F534A) + payload
    if binary is not None:
        binary += b'\0' * (-len(binary) % 4)
        chunks += struct.pack('<II', len(binary), 0x004E4942) + binary
    return struct.pack('<4sII', b'glTF', 2, 12 + len(chunks)) + chunks


@pytest.mark.parametrize('vrm', [False, True])
def test_import_preserves_weights_uv_textures_and_bone_metadata(tmp_path, vrm):
    data, binary = skinned_fixture(vrm=vrm)
    suffix = '.vrm' if vrm else '.glb'
    data['buffers'][0].pop('uri')
    result = import_human_model(tmp_path / 'assets', tmp_path, 'avatar' + suffix, encoded(glb(data, binary)))
    metadata = validate_stored_asset(result['asset'])
    assert metadata['vertex_count'] == 3
    assert metadata['uv_sets'] == ['TEXCOORD_0']
    assert metadata['texture_count'] == 1
    hair = metadata['bones'][-1]
    assert {key: value for key, value in hair.items() if key != 'position'} == {
        'name': 'Hair', 'named': True, 'index': 12, 'parent': 3,
        'parent_name': 'Head', 'parent_bone': 3}
    assert hair['position'] == pytest.approx([0.0, 1.3, 0.0])
    assert result['render_model']['metadata'] == metadata
    assert result['render_model']['bone_map']['pelvis'] == 0
    assert result['render_model']['bone_map']['left_upper_arm'] == 4
    duplicate = import_human_model(tmp_path / 'assets', tmp_path, 'copy' + suffix, encoded(glb(data, binary)))
    assert duplicate['asset'] == result['asset']


def test_gltf_embeds_selected_sidecars_without_changing_skeletal_data(tmp_path):
    data, binary = skinned_fixture(external=True)
    result = import_human_model(tmp_path / 'assets', tmp_path, 'avatar.gltf', encoded(json.dumps(data).encode()),
                                [{'name': 'body.bin', 'content_base64': encoded(binary)},
                                 {'name': 'albedo.png', 'content_base64': encoded(PNG)}])
    saved = json.loads(result['asset'].read_text())
    assert saved['skins'] == data['skins']
    assert saved['meshes'] == data['meshes']
    assert saved['buffers'][0]['uri'] == 'data:application/octet-stream;base64,' + encoded(binary)
    assert saved['images'][0]['uri'] == 'data:image/png;base64,' + encoded(PNG)
    assert validate_stored_asset(result['asset'])['skin_count'] == 1


def test_gltf_companion_picker_matches_unique_basenames_and_rejects_ambiguity():
    data,binary=skinned_fixture(external=True)
    data['images'][0]['uri']='textures/albedo.png'
    files=[{'name':'body.bin','content_base64':encoded(binary)},
           {'name':'albedo.png','content_base64':encoded(PNG)}]
    prepared,_,_=prepare_import('avatar.gltf',encoded(json.dumps(data).encode()),files)
    assert json.loads(prepared)['images'][0]['uri'].startswith('data:image/png;base64,')
    files[1]['name']='a/albedo.png'
    files.append({'name':'b/albedo.png','content_base64':encoded(PNG)})
    with pytest.raises(DocumentError,match='ambiguous companion'):
        prepare_import('avatar.gltf',encoded(json.dumps(data).encode()),files)


def test_unnamed_bones_get_node_labels_and_bind_pose_positions():
    data, _ = skinned_fixture()
    data['nodes'][12].pop('name')
    _, _, metadata = prepare_import('avatar.gltf', encoded(json.dumps(data).encode()))
    hair = metadata['bones'][-1]
    assert hair['name'] == 'Bone 12' and hair['named'] is False
    assert hair['parent_bone'] == 3 and hair['position'] == pytest.approx([0.0, 1.3, 0.0])


def test_name_and_hierarchy_heuristics_match_noisy_shoulders_and_unnamed_torso_chain():
    names = ['Bone 0', 'Bone 1', 'Bone 2', 'Bone 3', 'Character_Head_Control',
             'Rig_L_Shoulder_Joint', 'Bone 6', 'Bone 7', 'Bone 8',
             'RightShoulderControl', 'Bone 10', 'Bone 11', 'Bone 12']
    parents = {1: 0, 2: 1, 3: 2, 4: 3, 5: 2, 6: 5, 7: 6, 8: 7,
               9: 2, 10: 9, 11: 10, 12: 11}
    nodes = [{'name': name if not name.startswith('Bone ') else ''} for name in names]
    result = _auto_bone_map(nodes, list(range(len(nodes))), {}, parents)
    assert [result[key] for key in ('pelvis', 'lumbar', 'thorax', 'neck', 'head')] == [0, 1, 2, 3, 4]
    assert [result[key] for key in ('left_clavicle', 'left_upper_arm', 'left_forearm', 'left_hand')] == [5, 6, 7, 8]
    assert [result[key] for key in ('right_clavicle', 'right_upper_arm', 'right_forearm', 'right_hand')] == [9, 10, 11, 12]


@pytest.mark.parametrize('uri', ['../secret.bin', '/secret.bin', 'https://example.org/x.bin', 'file:///secret.bin', '%2e%2e/secret.bin', 'x\\secret.bin'])
def test_import_rejects_remote_and_traversing_resources(uri):
    data, _ = skinned_fixture()
    data['buffers'][0]['uri'] = uri
    with pytest.raises(DocumentError): prepare_import('avatar.gltf', encoded(json.dumps(data).encode()))


@pytest.mark.parametrize('mutation,match', [
    (lambda d: d['nodes'][0].update(children=[1, 0]), 'tree'),
    (lambda d: d['nodes'][12].update(children=[0]), 'cyclic'),
    (lambda d: d['skins'][0].update(joints=[999]), 'index'),
    (lambda d: d['bufferViews'][0].update(byteLength=99999), 'exceeds'),
    (lambda d: d['buffers'][0].update(byteLength=99999), 'byteLength'),
    (lambda d: d['accessors'][0].update(byteOffset=99999), 'exceeds'),
    (lambda d: d['nodes'][0].update(translation=[0, float('nan'), 0]), 'JSON'),
    (lambda d: d['nodes'][0].update(name={'invalid': 'name'}), 'names must be strings'),
    (lambda d: d['meshes'][0]['primitives'][0]['attributes'].pop('WEIGHTS_0'), 'weights'),
    (lambda d: d.update(extensionsUsed=['KHR_draco_mesh_compression']), 'uncompressed'),
    (lambda d: d.update(extensions={'other': {'uri': 'https://example.org/data'}}), 'extension'),
    (lambda d: d.update(extensions={'VRMC_vrm': {'humanoid': {'humanBones': []}}}), 'bone map'),
])
def test_import_rejects_invalid_and_unsupported_assets(mutation, match):
    data, _ = skinned_fixture()
    mutation(data)
    with pytest.raises(DocumentError, match=match): prepare_import('avatar.gltf', encoded(json.dumps(data).encode()))


def test_missing_sidecar_and_corrupt_or_oversized_upload_fail_before_writing(tmp_path):
    data, _ = skinned_fixture(external=True)
    with pytest.raises(DocumentError, match='companion'): prepare_import('avatar.gltf', encoded(json.dumps(data).encode()))
    for filename, raw in [('x.glb', b'glTF'), ('x.gltf', b'{'), ('x.obj', b'hello')]:
        with pytest.raises(DocumentError): import_human_model(tmp_path / 'assets', tmp_path, filename, encoded(raw))
    with pytest.raises(DocumentError, match='20 MiB'): prepare_import('x.glb', 'A' * (MAX_ASSET_BYTES * 2))
    assert not (tmp_path / 'assets').exists()


def test_render_config_does_not_change_physics_and_follows_save_as_and_expand(blank, tmp_path):
    from pipesim.editing import relocate_design, expand_objects
    data, _ = skinned_fixture()
    config = import_human_model(tmp_path / 'assets', tmp_path, 'avatar.gltf', encoded(json.dumps(data).encode()))['render_model']
    plain = copy.deepcopy(blank)
    plain['objects'] = [{'id': 'person', 'template': 'human', 'pose': {'position_mm': [100, 200, 300]}}]
    styled = copy.deepcopy(plain)
    styled['objects'][0]['render_model'] = config
    styled_assembly = Assembly.from_doc(styled, tmp_path)
    assert styled_assembly.scene()['parts'] == Assembly.from_doc(plain, tmp_path).scene()['parts']
    human = styled_assembly.scene()['humans'][0]
    assert human['id'] == 'person' and len(human['rest_parts']) == 19
    assert human['rest_parts'][0]['pose']['position_mm'][0] == 0
    relocated = relocate_design(styled, tmp_path, tmp_path / 'nested')
    assert relocated['objects'][0]['render_model']['file'].startswith('../assets/')
    expanded = expand_objects(styled_assembly)
    assert Assembly.from_doc(expanded, tmp_path).scene()['humans'][0]['render_model'] == config
    styled['objects'][0]['render_model']['enabled'] = False
    assert Assembly.from_doc(styled, tmp_path).scene()['humans'][0]['render_model']['metadata'] == config['metadata']
    from pipesim.packaging import bundle
    path = bundle(styled_assembly, tmp_path / 'portable')
    bundled = Assembly.load(path)
    bundled_config = bundled.scene()['humans'][0]['render_model']
    assert bundled_config['file'].startswith('assets/human-')
    assert (path.parent / bundled_config['file']).read_bytes() == (tmp_path / config['file']).read_bytes()


@pytest.mark.parametrize('update', [{'enabled': 'yes'}, {'bone_map': {'tail': 'bone'}}, {'bone_map': {'head': -1}},
                                    {'bone_offsets': {'tail': [0, 0, 0]}}, {'bone_offsets': {'head': [0, 1]}},
                                    {'default_extra_mode': 'unknown'}, {'extra_bones': {'Hair': {'damping': -1}}},
                                    {'extra_bones': {'Hair': {'rotation_deg': [0, 1]}}}])
def test_invalid_render_configuration_is_rejected(blank, update):
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id': 'person', 'template': 'human', 'render_model': {'file': 'avatar.glb', **update}}]
    with pytest.raises(DocumentError): Assembly.from_doc(doc)


def test_bundle_keeps_disabled_missing_model_reference_without_loading_it(blank,tmp_path):
    from pipesim.packaging import bundle
    doc=copy.deepcopy(blank)
    config={'file':'missing.glb','enabled':False,'bone_map':{'pelvis':0},'extra_bones':{}}
    doc['objects']=[{'id':'person','template':'human','render_model':config}]
    path=bundle(Assembly.from_doc(doc,tmp_path),tmp_path/'portable')
    copied=Assembly.load(path).scene()['humans'][0]['render_model']
    assert not copied['enabled'] and copied['bone_map']==config['bone_map']
    assert (path.parent/copied['file']).resolve()==tmp_path/'missing.glb'
