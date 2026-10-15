"""Scene mirrors preserve every joint of compact articulated objects."""

import copy
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from pipesim.document import Assembly
from pipesim.deletion import delete_parts
from pipesim.geometry import mesh_for_part
from pipesim.symmetry import materialize_mirror
from pipesim.validation import validate


def mirrored_chain_fixture(base):
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': 'Compact chain mirror',
           'objects': [{'id': 'chain', 'template': 'chain',
                        'parameters': {'length_mm': 80,
                                       'link_catalog': 'generic.chain-link'},
                        'pose': {'position_mm': [200, 0, 500]}}],
           'parts': [], 'joints': [],
           'draft_subassemblies': [{'id': 'scene', 'runs': [],
                                    'mirrors': [{'id': 'x-plane', 'axis': 'x',
                                                 'offset_mm': 0, 'scope': 'scene'}]}]}
    expanded = Assembly.from_doc(doc, base)
    tail, _ = expanded.parts['chain/link-4'].frame({'port': 'a'})
    doc['parts'].append({'id': 'payload',
                         'pose': {'position_mm': tail.tolist()},
                         'body': {'kind': 'rigid', 'mass_kg': 1,
                                  'geometry': [{'type': 'box',
                                                'size_mm': [20, 20, 20]}]}})
    doc['joints'].append({'id': 'payload-hook', 'type': 'fixed',
                          'a': {'part': 'chain/link-4', 'port': 'a'},
                          'b': {'part': 'payload'}})
    return doc


def test_scene_mirror_keeps_chain_grouped_and_payload_connected(tmp_path):
    doc = mirrored_chain_fixture(tmp_path)
    original = copy.deepcopy(doc)
    source = Assembly.from_doc(doc, tmp_path)
    assert len(source.joints) == 4  # Three generated links, one explicit payload.
    baked = materialize_mirror(source, 'scene', 'x-plane')
    assert doc == original
    assert baked['objects'][0] == original['objects'][0]
    assert baked['objects'][1]['id'] == 'chain-mirror-x-plane'
    assert len(baked['objects'][1]['components']['parts']) == 4
    exact = Assembly.from_doc(baked, tmp_path)
    assert len(exact.parts) == 10
    assert len(exact.joints) == 8
    mirrored_links = {f'chain-mirror-x-plane/link-{i}' for i in range(1, 5)}
    assert mirrored_links <= set(exact.parts)
    internal = [joint for joint in exact.joints
                if joint['id'].startswith('chain-mirror-x-plane/join-')]
    assert len(internal) == 3
    assert all({joint['a']['part'], joint['b']['part']} <= mirrored_links
               for joint in internal)
    assert any({joint['a']['part'], joint['b']['part']} ==
               {'chain-mirror-x-plane/link-4', 'payload-mirror-x-plane'}
               for joint in baked['joints'])
    reflected_vertices = mesh_for_part(source.parts['chain/link-1'], world=True).vertices.copy()
    reflected_vertices[:, 0] *= -1
    cloned_vertices = mesh_for_part(exact.parts['chain-mirror-x-plane/link-1'], world=True).vertices
    assert cKDTree(cloned_vertices).query(reflected_vertices)[0].max() < 1e-5
    for joint in exact.joints:
        first, second, _ = exact.joint_frames(joint)
        assert np.linalg.norm(first-second) < .05, joint['id']
    finished = copy.deepcopy(baked)
    finished.pop('draft_subassemblies')
    assert validate(Assembly.from_doc(finished, tmp_path), collisions=False)['valid']
    deleted = delete_parts(exact, object_id='chain-mirror-x-plane')
    remaining = Assembly.from_doc(deleted['document'], tmp_path)
    assert not mirrored_links & remaining.parts.keys()
    assert 'chain/link-1' in remaining.parts


def test_chain_crossing_mirror_plane_remains_one_grouped_line(tmp_path):
    fixture = Path(__file__).parent / 'fixtures' / 'chain_semi_circle_attachment.json'
    saved = json.loads(fixture.read_text())
    source = saved['objects'][0]
    doc = {'format': saved['format'], 'units': saved['units'], 'name': 'Crossing chain',
           'objects': [source], 'parts': [], 'joints': [],
           'draft_subassemblies': [{'id': 'scene', 'runs': [],
                                    'mirrors': [{'id': 'x-plane', 'axis': 'x',
                                                 'offset_mm': 0, 'scope': 'scene'}]}]}
    assembly = Assembly.from_doc(doc, tmp_path)
    x = [part.matrix[0, 3] for part in assembly.parts.values()]
    assert min(x) < 0 < max(x)
    baked = materialize_mirror(assembly, 'scene', 'x-plane')
    assert [obj['id'] for obj in baked['objects']] == [source['id']]
    assert not baked['parts']
    exact = Assembly.from_doc(baked, tmp_path)
    assert len(exact.parts) == 30
    assert len(exact.joints) == 29


def test_chain_in_mirror_plane_is_not_duplicated(tmp_path):
    doc = mirrored_chain_fixture(tmp_path)
    doc['objects'][0]['pose']['position_mm'][0] = 0
    doc['parts'] = []
    doc['joints'] = []
    baked = materialize_mirror(Assembly.from_doc(doc, tmp_path), 'scene', 'x-plane')
    assert [obj['id'] for obj in baked['objects']] == ['chain']
    assert len(Assembly.from_doc(baked, tmp_path).parts) == 4
