"""Purchased mirror identity must survive port permutations and repeated bakes."""
import copy
import json
import os
import random
import secrets

import numpy as np
import pytest

from pipesim.document import Assembly
from pipesim.exporting import bom
from pipesim.math3d import align_axis, pose_of
from pipesim.symmetry import (
    _mirrored_body, canonical_catalog, materialize_mirror,
    reflection_matrix,
)


def fixture(catalog='tubeclamp.TC101C', *, axis='x', offset=-1200, pose=None):
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
            'parts': [{'id': 'fitting', 'catalog': catalog,
                       'pose': pose or {'position_mm': [240, 370, 600],
                                       'rotation_deg': [17, -28, 39]}}],
            'joints': [], 'environment': {'ground': False, 'gravity_m_s2': [0, 0, 0]},
            'draft_subassemblies': [{'id': 'frame', 'runs': [],
                'mirror_parts': ['fitting'],
                'mirrors': [{'id': 'plane', 'axis': axis, 'offset_mm': offset}]}]}


def bake(document, tmp_path, library):
    source = Assembly.from_doc(document, tmp_path, library)
    result = materialize_mirror(source, 'frame', 'plane')
    return source, result, Assembly.from_doc(result, tmp_path, library)


def matching_port(source, clone, name, mirror):
    position, axis = source.frame({'port': name})
    expected_position = mirror[:3, :3] @ position + mirror[:3, 3]
    expected_axis = mirror[:3, :3] @ axis
    matches = [other for other, port in clone.ports.items()
               if port.get('type') == source.ports[name].get('type')
               and np.allclose(clone.frame({'port': other})[0], expected_position, atol=1e-5)
               and np.allclose(clone.frame({'port': other})[1], expected_axis, atol=1e-7)]
    assert len(matches) == 1, (source.id, name, matches)
    return matches[0]


def test_repeated_tee_mirror_reuses_catalogue_without_mesh_assets(tmp_path, library):
    document = fixture()
    for turn in range(6):
        _, result, resolved = bake(document, tmp_path, library)
        new = result['parts'][-1]
        assert new['catalog'] == 'tubeclamp.TC101C'
        assert not new.get('body', {}).get('geometry')
        assert canonical_catalog(resolved.parts[new['id']], resolved)[0] == 'tubeclamp.TC101C'
        document = result
        document['draft_subassemblies'][0].update(
            mirror_parts=[new['id']], mirrors=[{'id': 'plane', 'axis': 'xyz'[turn % 3],
                                               'offset_mm': -1200 + turn * 90}])
    assert not list(tmp_path.rglob('*.stl'))
    document.pop('draft_subassemblies')
    items = bom(Assembly.from_doc(document, tmp_path, library))['items']
    assert len(items) == 1 and items[0]['quantity'] == 7
    assert items[0]['catalog'] == 'tubeclamp.TC101C'


def test_catalogue_reuse_preserves_authored_appearance_and_mass_frame(tmp_path, library):
    document = fixture('tubeclamp.TC128C')
    document['parts'][0].update(color='#abc123', friction=.24,
        center_of_mass_mm=[3, 5, 7], inertia_kg_m2=[.03, .04, .05])
    source, _, after = bake(document, tmp_path, library)
    original, clone = source.parts['fitting'], after.parts['fitting-mirror-plane']
    assert clone.spec['catalog'] == 'tubeclamp.TC128C'
    assert clone.definition['color'] == '#abc123'
    assert clone.definition['friction'] == .24
    reflection = reflection_matrix('x', -1200)
    original_center = original.matrix @ np.r_[original.center_of_mass, 1]
    clone_center = clone.matrix @ np.r_[clone.center_of_mass, 1]
    assert np.allclose(clone_center, reflection @ original_center, atol=1e-5)
    original_inertia = original.matrix[:3, :3] @ np.diag(original.definition['inertia_kg_m2']) @ original.matrix[:3, :3].T
    clone_inertia = clone.matrix[:3, :3] @ np.diag(clone.definition['inertia_kg_m2']) @ clone.matrix[:3, :3].T
    assert np.allclose(clone_inertia, reflection[:3, :3] @ original_inertia @ reflection[:3, :3].T)


def test_legacy_member_mirror_keeps_its_own_cut_after_original_is_resized(tmp_path, library):
    document = fixture('tubeclamp.tube-C')
    document['parts'][0]['parameters'] = {'length_mm': 500}
    original = Assembly.from_doc(document, tmp_path, library).parts['fitting']
    document['parts'].append({'id': 'old-cut', 'label': 'Mirror of fitting',
        'body': _mirrored_body(original, tmp_path), 'pose': {'position_mm': [-300, 600, 700]}})
    document['parts'][0]['parameters']['length_mm'] = 1000
    document['draft_subassemblies'][0]['mirror_parts'] = ['old-cut']
    _, _, after = bake(document, tmp_path, library)
    mirrored = after.parts['old-cut-mirror-plane']
    assert mirrored.spec['catalog'] == 'tubeclamp.tube-C'
    assert mirrored.length == 500
    assert mirrored.mass == pytest.approx(original.mass)


def test_catalogue_tube_mirror_preserves_authored_mass(tmp_path, library):
    document = fixture('tubeclamp.tube-C')
    document['parts'][0].update(parameters={'length_mm': 500}, mass_kg=3.25)
    _, _, after = bake(document, tmp_path, library)
    clone = after.parts['fitting-mirror-plane']
    assert clone.spec['catalog'] == 'tubeclamp.tube-C'
    assert clone.mass == 3.25


def test_old_baked_mirror_recovers_catalogue_and_bom_without_changing_design(tmp_path, library):
    document = fixture()
    source = Assembly.from_doc(document, tmp_path, library).parts['fitting']
    body = _mirrored_body(source, tmp_path)
    # This is the exact legacy provenance available in complicatedMetalRaw.json.
    document['parts'].append({'id': 'fitting-mirror-mirror-1', 'label': 'Mirror of fitting',
                              'body': body, 'pose': {'position_mm': [-400, 300, 200]}})
    document['parts'].append({'id': 'fitting-mirror-mirror-1-mirror-mirror-2',
        'label': 'Mirror of fitting-mirror-mirror-1', 'body': copy.deepcopy(body),
        'pose': {'position_mm': [-900, 300, 200]}})
    document.pop('draft_subassemblies')
    original = copy.deepcopy(document)
    assembly = Assembly.from_doc(document, tmp_path, library)
    items = bom(assembly)['items']
    assert len(items) == 1 and items[0]['quantity'] == 3
    assert items[0]['catalog'] == 'tubeclamp.TC101C'
    assert items[0]['name'] == library.parts['tubeclamp.TC101C']['name']
    assert document == original
    # An arbitrary custom body carrying a similar label is not enough evidence.
    custom = copy.deepcopy(document)
    custom['parts'][1]['body']['geometry'] = [{'type': 'box', 'size_mm': [10, 20, 30]}]
    resolved = Assembly.from_doc(custom, tmp_path, library)
    assert canonical_catalog(resolved.parts['fitting-mirror-mirror-1'], resolved)[0] is None


@pytest.mark.parametrize('attachment_kind', ['joint', 'draft'])
def test_corner_port_permutation_remaps_joints_and_draft_attachments(tmp_path, library, attachment_kind):
    document = fixture('tubeclamp.TC128C')
    source = Assembly.from_doc(document, tmp_path, library).parts['fitting']
    for name in ('x', 'y', 'z'):
        mouth, axis = source.frame({'port': name})
        if attachment_kind == 'joint':
            document['parts'].append({'id': 'witness-' + name, 'catalog': 'generic.box',
                'pose': {'position_mm': mouth.tolist()}})
            document['joints'].append({'id': 'joint-' + name, 'type': 'fixed',
                'a': {'part': 'fitting', 'port': name},
                'b': {'part': 'witness-' + name, 'frame': {'position_mm': [0, 0, 0]}}})
        else:
            depth = source.ports[name]['engagement_mm'] * .8
            document['draft_subassemblies'][0]['runs'].append({
                'id': 'run-' + name, 'catalog': 'tubeclamp.tube-C',
                'start_mm': (mouth - axis * depth).tolist(),
                'end_mm': (mouth + axis * 200).tolist(),
                'attachments': [{'connector': 'fitting', 'port': name, 'end': 'start',
                                 'insertion_mm': depth}]})
    original, result, after = bake(document, tmp_path, library)
    clone = after.parts['fitting-mirror-plane']
    assert clone.spec['catalog'] == 'tubeclamp.TC128C'
    reflection = reflection_matrix('x', -1200)
    remapped = {}
    for name in ('x', 'y', 'z'):
        expected = matching_port(original.parts['fitting'], clone, name, reflection)
        remapped[name] = expected
        if attachment_kind == 'joint':
            joint = next(j for j in result['joints'] if j['id'] == 'joint-' + name + '-mirror-plane')
            assert joint['a']['port'] == expected
            first, second, _ = after.joint_frames(joint)
            assert np.allclose(first, second, atol=1e-5)
        else:
            run = next(r for r in result['draft_subassemblies'][0]['runs']
                       if r['id'] == 'run-' + name + '-mirror-plane')
            assert run['attachments'][0]['port'] == expected
    assert any(old != new for old, new in remapped.items())


@pytest.mark.parametrize('axis', list('xyz'))
def test_offset_through_bore_preserves_oriented_slide_state_limits_and_motor(tmp_path, library, monkeypatch, axis):
    document = fixture('tubeclamp.TC161C', axis=axis)
    source = Assembly.from_doc(document, tmp_path, library).parts['fitting']
    mouth, direction = source.frame({'port': 'cross'})
    matrix = np.eye(4)
    matrix[:3, :3] = align_axis([0, 0, 1], direction)
    matrix[:3, 3] = mouth
    document['parts'].append({'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
        'parameters': {'length_mm': 600}, 'pose': pose_of(matrix)})
    document['joints'] = [{'id': 'slide', 'type': 'socket', 'locked': False,
        'a': {'part': 'fitting', 'port': 'cross'}, 'b': {'part': 'pipe', 'at_mm': 300},
        'limits': {'slide_mm': [-80, 125], 'angle_deg': [-32, 57]},
        'motor': {'mode': 'velocity', 'target': 20, 'max_force_n': 12}}]
    document['draft_subassemblies'][0]['mirror_parts'].append('pipe')
    document['state'] = {'joints': {'slide': {'slide_mm': 31, 'twist_deg': 19}}}
    original, result, after = bake(document, tmp_path, library)
    clone = after.parts['fitting-mirror-plane']
    assert clone.spec['catalog'] == 'tubeclamp.TC161C'
    assert 'geometry' not in clone.spec.get('body', {})
    reflection = reflection_matrix(axis, -1200)
    matching_port(original.parts['fitting'], clone, 'cross', reflection)
    joint = next(j for j in result['joints'] if j['id'] == 'slide-mirror-plane')
    assert joint['limits'] == document['joints'][0]['limits']
    assert joint['motor'] == document['joints'][0]['motor']
    # Reusing a catalogue body must retain the same signed coordinate system
    # and current-state snapshot as the previous physically reflected body.
    monkeypatch.setattr('pipesim.symmetry._mirror_catalog', lambda *_: None)
    _, _, legacy = bake(document, tmp_path, library)
    for movement in ({'slide_mm': 13}, {'twist_deg': -11}, {'slide_mm': -27, 'twist_deg': 23}):
        after.apply_coordinates({'slide-mirror-plane': movement})
        legacy.apply_coordinates({'slide-mirror-plane': movement})
        assert np.allclose(after.parts['pipe-mirror-plane'].matrix,
                           legacy.parts['pipe-mirror-plane'].matrix, atol=1e-5)
    assert np.allclose(after.joint_frames(joint)[2], legacy.joint_frames(joint)[2], atol=1e-7)


def test_custom_mirror_of_mirror_restores_original_asset_and_definition(tmp_path, library):
    document = fixture()
    definition = {'kind': 'connector', 'name': 'Custom asymmetric bracket', 'mass_kg': .7,
        'source': {'geometry_status': 'approximate'}, 'center_of_mass_mm': [8, 3, 1],
        'inertia_kg_m2': [.03, .04, .05],
        'geometry': [{'type': 'box', 'size_mm': [11, 23, 37], 'position_mm': [7, 2, 9]},
                     {'type': 'box', 'size_mm': [13, 7, 9], 'position_mm': [-8, 19, 5]}],
        'ports': {}}
    document['parts'][0].pop('catalog')
    document['parts'][0]['body'] = definition
    source, result, after = bake(document, tmp_path, library)
    first = after.parts['fitting-mirror-plane']
    assert first.spec['body']['name'] == definition['name']
    assert first.spec['body']['source'] == definition['source']
    assert first.spec['body']['center_of_mass_mm'] == [-8, 3, 1]
    assert first.spec['body']['inertia_kg_m2'] == definition['inertia_kg_m2']
    mesh_files = set(tmp_path.rglob('*.stl'))
    result['draft_subassemblies'][0].update(mirror_parts=[first.id],
        mirrors=[{'id': 'plane', 'axis': 'x', 'offset_mm': -1200}])
    _, second_doc, second = bake(result, tmp_path, library)
    restored = second.parts[second_doc['parts'][-1]['id']]
    assert restored.spec['body'] == definition
    assert np.allclose(restored.matrix, source.parts['fitting'].matrix, atol=1e-7)
    assert set(tmp_path.rglob('*.stl')) == mesh_files


MIRROR_SEED = int(os.environ.get('PIPESIM_CATALOG_MIRROR_SEED', secrets.randbits(64)))
MIRROR_CASES = int(os.environ.get('PIPESIM_CATALOG_MIRROR_CASES', 60))


def test_catalogue_mirror_fuzz_preserves_identity_and_signed_port_frames(tmp_path, library):
    rng = random.Random(MIRROR_SEED)
    print(f'catalogue mirror fuzz seed={MIRROR_SEED}; cases={MIRROR_CASES}', flush=True)
    catalogs = [name for name in library.parts if name.startswith('tubeclamp.') and name.endswith('C')]
    for index in range(max(MIRROR_CASES, len(catalogs))):
        catalog = catalogs[index % len(catalogs)]
        axis = 'xyz'[(index // len(catalogs) + index) % 3]
        offset = rng.uniform(-4000, -2000)
        document = fixture(catalog, axis=axis, offset=offset, pose={
            'position_mm': [rng.uniform(200, 900) for _ in range(3)],
            'rotation_deg': [rng.uniform(-180, 180) for _ in range(3)]})
        if catalog == 'tubeclamp.tube-C':
            document['parts'][0].update(parameters={'length_mm': rng.uniform(100, 1500)},
                                        mass_kg=rng.uniform(.5, 8))
        try:
            fitting = Assembly.from_doc(document, tmp_path, library).parts['fitting']
            # Include actual endpoint references: merely verifying that some
            # equivalent port exists would miss an omitted reference remap.
            for name in list(fitting.ports)[:3]:
                mouth, direction = fitting.frame({'port': name})
                matrix = np.eye(4)
                matrix[:3, :3] = align_axis([0, 0, 1], direction)
                matrix[:3, 3] = mouth
                document['parts'].append({'id': 'witness-' + name, 'catalog': 'tubeclamp.tube-C',
                    'parameters': {'length_mm': 100}, 'pose': pose_of(matrix)})
                document['joints'].append({'id': 'joint-' + name, 'type': 'fixed',
                    'a': {'part': 'fitting', 'port': name},
                    'b': {'part': 'witness-' + name, 'at_mm': 50}})
            source, result, after = bake(document, tmp_path, library)
            clone = after.parts['fitting-mirror-plane']
            assert clone.spec['catalog'] == catalog
            assert not clone.spec.get('body', {}).get('geometry')
            reflection = reflection_matrix(axis, offset)
            for name in source.parts['fitting'].ports:
                matching_port(source.parts['fitting'], clone, name, reflection)
            for joint in result['joints']:
                if not joint['id'].endswith('-mirror-plane'):
                    continue
                original = next(item for item in document['joints']
                                if item['id'] + '-mirror-plane' == joint['id'])
                assert joint['a']['port'] == matching_port(
                    source.parts['fitting'], clone, original['a']['port'], reflection)
                first, second, _ = after.joint_frames(joint)
                assert np.allclose(first, second, atol=1e-5)
            # Every family also exercises a second reflection and any axis
            # override left by the first. No generated STL should be needed.
            result['draft_subassemblies'][0].update(mirror_parts=[clone.id], mirrors=[
                {'id': 'plane', 'axis': axis, 'offset_mm': offset}])
            _, twice_doc, twice = bake(result, tmp_path, library)
            final = twice.parts['fitting-mirror-plane-mirror-plane']
            assert final.spec['catalog'] == catalog
            for name in clone.ports:
                matching_port(clone, final, name, reflection)
            if source.parts['fitting'].kind == 'member':
                assert clone.length == final.length == source.parts['fitting'].length
                assert clone.mass == final.mass == source.parts['fitting'].mass
        except Exception as error:
            path = tmp_path / f'catalog-mirror-{MIRROR_SEED}-{index}.json'
            path.write_text(json.dumps(document, indent=2))
            pytest.fail(f'seed={MIRROR_SEED}; case={index}; catalog={catalog}; repro={path}: {error}')
    assert not list(tmp_path.rglob('*.stl'))


def test_custom_repeated_mirror_fuzz_retains_provenance_and_stops_mesh_growth(tmp_path, library):
    rng = random.Random(MIRROR_SEED ^ 0x435553544F4D)
    for index in range(18):
        axis = 'xyz'[index % 3]
        offset = rng.uniform(-3000, -1000)
        document = fixture(axis=axis, offset=offset, pose={
            'position_mm': [rng.uniform(100, 900) for _ in range(3)],
            'rotation_deg': [rng.uniform(-180, 180) for _ in range(3)]})
        document['parts'][0].pop('catalog')
        document['parts'][0]['body'] = {'kind': 'rigid', 'mass_kg': rng.uniform(.1, 5),
            'geometry': [{'type': 'box', 'size_mm': [rng.uniform(8, 40) for _ in range(3)],
                          'position_mm': [rng.uniform(-30, 30) for _ in range(3)]}
                         for _ in range(3)]}
        original, once_doc, once = bake(document, tmp_path, library)
        assets = set(tmp_path.rglob('*.stl'))
        once_doc['draft_subassemblies'][0].update(mirror_parts=['fitting-mirror-plane'],
            mirrors=[{'id': 'plane', 'axis': axis, 'offset_mm': offset}])
        _, twice_doc, twice = bake(once_doc, tmp_path, library)
        final = twice.parts['fitting-mirror-plane-mirror-plane']
        assert final.spec['body'] == document['parts'][0]['body'], (MIRROR_SEED, index)
        assert np.allclose(final.matrix, original.parts['fitting'].matrix, atol=1e-5), (MIRROR_SEED, index)
        assert set(tmp_path.rglob('*.stl')) == assets, (MIRROR_SEED, index)


def test_legacy_baked_catalogue_mirror_fuzz_recovers_bom_and_future_copies(tmp_path, library):
    rng = random.Random(MIRROR_SEED ^ 0x4C4547414359)
    for index, catalog in enumerate(['tubeclamp.TC101C', 'tubeclamp.TC128C', 'tubeclamp.TC161C', 'tubeclamp.tube-C'] * 3):
        document = fixture(catalog, axis='xyz'[index % 3], pose={
            'position_mm': [rng.uniform(100, 900) for _ in range(3)],
            'rotation_deg': [rng.uniform(-180, 180) for _ in range(3)]})
        if catalog == 'tubeclamp.tube-C':
            document['parts'][0]['parameters'] = {'length_mm': rng.uniform(100, 1000)}
        source = Assembly.from_doc(document, tmp_path, library).parts['fitting']
        legacy = {'id': 'legacy', 'label': 'Mirror of fitting',
                  'body': _mirrored_body(source, tmp_path),
                  'pose': {'position_mm': [-400, 600, 900],
                           'rotation_deg': [rng.uniform(-180, 180) for _ in range(3)]}}
        document['parts'].append(legacy)
        if source.kind == 'member':
            document['parts'][0]['parameters']['length_mm'] += rng.uniform(100, 1000)
        document['draft_subassemblies'][0]['mirror_parts'] = ['legacy']
        assets = set(tmp_path.rglob('*.stl'))
        before, result, after = bake(document, tmp_path, library)
        clone = after.parts['legacy-mirror-plane']
        assert clone.spec['catalog'] == catalog, (MIRROR_SEED, index)
        if source.kind == 'member':
            assert clone.length == source.length and clone.mass == pytest.approx(source.mass), (MIRROR_SEED, index)
        reflection = reflection_matrix('xyz'[index % 3], -1200)
        for name in before.parts['legacy'].ports:
            matching_port(before.parts['legacy'], clone, name, reflection)
        result.pop('draft_subassemblies')
        items = bom(Assembly.from_doc(result, tmp_path, library))['items']
        assert all(item['catalog'] == catalog for item in items), (MIRROR_SEED, index)
        assert sum(item['quantity'] for item in items) == 3, (MIRROR_SEED, index)
        assert len(items) == (2 if source.kind == 'member' else 1), (MIRROR_SEED, index)
        assert set(tmp_path.rglob('*.stl')) == assets, (MIRROR_SEED, index)
