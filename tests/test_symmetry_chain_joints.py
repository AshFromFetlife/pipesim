"""Scene mirrors preserve every joint of compact articulated objects."""

import copy

import numpy as np

from pipesim.document import Assembly
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


def test_scene_mirror_bakes_generated_chain_joints_and_payload_connection(tmp_path):
    doc = mirrored_chain_fixture(tmp_path)
    original = copy.deepcopy(doc)
    source = Assembly.from_doc(doc, tmp_path)
    assert len(source.joints) == 4  # Three generated links, one explicit payload.
    baked = materialize_mirror(source, 'scene', 'x-plane')
    assert doc == original
    assert baked['objects'] == original['objects']
    exact = Assembly.from_doc(baked, tmp_path)
    assert len(exact.parts) == 10
    assert len(exact.joints) == 8
    mirrored_links = {f'chain/link-{i}-mirror-x-plane' for i in range(1, 5)}
    assert mirrored_links <= set(exact.parts)
    internal = [joint for joint in baked['joints']
                if joint['id'].startswith('chain/join-')]
    assert len(internal) == 3
    assert all({joint['a']['part'], joint['b']['part']} <= mirrored_links
               for joint in internal)
    assert any({joint['a']['part'], joint['b']['part']} ==
               {'chain/link-4-mirror-x-plane', 'payload-mirror-x-plane'}
               for joint in baked['joints'])
    for joint in exact.joints:
        first, second, _ = exact.joint_frames(joint)
        assert np.linalg.norm(first-second) < .05, joint['id']
    finished = copy.deepcopy(baked)
    finished.pop('draft_subassemblies')
    assert validate(Assembly.from_doc(finished, tmp_path), collisions=False)['valid']
