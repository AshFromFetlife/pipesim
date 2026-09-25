import copy

import numpy as np
import pytest

from pipesim.document import Assembly, DocumentError, write
from pipesim.duplication import duplicate, duplicate_members
from pipesim.editing import expand_objects, relocate_design
from pipesim.grouping import regroup_object
from pipesim.geometry import mesh_for_part
from pipesim.math3d import transform


def graph(blank, factory):
    doc = copy.deepcopy(blank)
    positions = {'a': [0, 0, 10], 'b': [20, 0, 10], 'c': [40, 0, 10],
                 'd': [60, 0, 10], 'e': [80, 0, 10], 'f': [20, 20, 10]}
    doc['parts'] = [{'id': pid, 'body': {'kind': 'rigid', 'mass_kg': 2, 'geometry': [{'type': 'box', 'size_mm': [20, 20, 20]}]},
                     'pose': {'position_mm': pos}} for pid, pos in positions.items()]
    for a, b, kind in [('a', 'b', 'fixed'), ('b', 'c', 'fixed'), ('c', 'd', 'cylindrical'), ('d', 'e', 'fixed'), ('b', 'f', 'fixed')]:
        frame = (np.array(positions[b])-positions[a])/2
        doc['joints'].append({'id': a+'-'+b, 'type': kind,
                              'a': {'part': a, 'frame': {'position_mm': frame.tolist(), 'axis': [1, 0, 0]}},
                              'b': {'part': b, 'frame': {'position_mm': (-frame).tolist(), 'axis': [1, 0, 0]}}})
    doc['anchors'] = [{'part': 'a', 'surface': 'fixture'}]
    return factory(doc)


def assert_copies(source, result, factory):
    after = factory(result['document'])
    for pid, part in source.parts.items():
        assert np.allclose(after.parts[pid].matrix, part.matrix, atol=1e-6, rtol=0), pid
        assert after.parts[pid].definition == part.definition
    for record in result['copies']:
        translation = transform({'position_mm': record['offset_mm']})
        for old, new in record['parts'].items():
            assert np.allclose(after.parts[new].matrix, translation @ source.parts[old].matrix, atol=1e-5, rtol=0), (old, new)
            assert after.parts[new].definition == source.parts[old].definition
        for old, new in record['joints'].items():
            original = copy.deepcopy(next(j for j in source.joints if j['id'] == old))
            original['id'] = new
            for end in ('a', 'b'):
                original[end]['part'] = record['parts'][original[end]['part']]
            assert next(j for j in after.joints if j['id'] == new) == original
    assert after.anchors == source.anchors
    return after


@pytest.mark.parametrize('selected,scope,expected', [
    ('b', 'part', {'b'}),
    ('b', 'touching', {'a', 'b', 'c', 'f'}),
    ('c', 'touching', {'b', 'c', 'd'}),
    ('c', 'subassembly', {'a', 'b', 'c', 'f'}),
    ('d', 'subassembly', {'d', 'e'}),
])
def test_copy_scopes_keep_internal_connections_and_stop_at_their_boundary(blank, factory, selected, scope, expected):
    source = graph(blank, factory)
    before = copy.deepcopy(source.doc)
    result = duplicate(source, selected, scope)
    assert set(result['copies'][0]['parts']) == expected
    assert result['parts_per_copy'] == len(expected)
    assert_copies(source, result, factory)
    assert source.doc == before


def test_multiple_copies_are_independent_unique_and_spaced_on_the_grid(blank, factory):
    source = graph(blank, factory)
    source.doc['results'] = {'old': True}
    source.doc['build_plan'] = {'old': True}
    result = duplicate(source, 'c', 'subassembly', count=4, grid_mm=25)
    after = assert_copies(source, result, factory)
    assert len(after.parts) == 22 and len(after.joints) == 17
    assert [record['parts']['c'] for record in result['copies']] == ['c-copy', 'c-copy-2', 'c-copy-3', 'c-copy-4']
    assert result['selected'] == 'c-copy-4'
    assert all(record['offset_mm'][0] % 25 == 0 for record in result['copies'])
    assert 'results' not in result['document'] and 'build_plan' not in result['document']
    result['document']['parts'][-1]['body']['mass_kg'] = 123
    assert source.parts['f'].mass == 2
    assert result['document']['parts'][9]['body']['mass_kg'] == 2


def test_short_copy_of_a_generated_limb_keeps_saved_pose_and_physics_without_owning_person(blank, factory):
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id': 'person', 'template': 'human', 'parameters': {'hold_joints': ['arms']},
                       'pose': {'position_mm': [900, 300, 0], 'rotation_deg': [10, 5, 35]}}]
    doc['state'] = {'joints': {'person/right_shoulder': {'rotation_deg': [12, -9, 8]}, 'person/right_elbow': {'angle_deg': 30}}}
    source = factory(doc)
    result = duplicate(source, 'person/right_hand')
    after = assert_copies(source, result, factory)
    assert result['selected'] == 'person-right_hand-copy'
    assert len(after.parts) == 20 and len(after.joints) == 18
    assert result['document']['objects'] == doc['objects']
    assert result['document']['state'] == doc['state']
    assert duplicate_members(after, result['selected'], 'subassembly') == {result['selected']}


@pytest.mark.parametrize('anchored', [False, True])
def test_whole_human_preserves_coordinates_tracks_motors_and_drives_when_detached(blank, factory, anchored):
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id': 'person', 'template': 'human', 'parameters': {'hold_joints': ['arms']},
                       'pose': {'position_mm': [900, 300, 0], 'rotation_deg': [10, 5, 35]}}]
    doc['state'] = {'joints': {'person/left_knee': {'angle_deg': -15}, 'person/right_shoulder': {'rotation_deg': [12, -9, 8]}, 'person/right_elbow': {'angle_deg': 30}}}
    doc['animation'] = {'tracks': [{'joint': 'person/right_elbow', 'coordinate': 'angle_deg', 'keyframes': [{'time_s': 0, 'value': 30}, {'time_s': 1, 'value': 60}]}]}
    doc['drives'] = [{'id': 'exercise', 'type': 'gear', 'driver': 'person/right_elbow', 'follower': 'person/left_elbow', 'ratio': 1, 'route_mm': [[900, 300, 500], [1200, 300, 500]]}]
    if anchored:
        doc['anchors'] = [{'part': 'person/left_foot', 'surface': 'fixture'}]
    source = factory(doc)
    result = duplicate(source, 'person/right_hand', 'subassembly', count=2)
    after = assert_copies(source, result, factory)
    assert len(after.parts) == 57 and len(after.joints) == 54
    assert len(result['document']['objects']) == 3 and result['document']['parts'] == []
    for record in result['copies']:
        assert result['document']['state']['joints'][record['joints']['person/right_elbow']] == {'angle_deg': 30}
        track = next(t for t in result['document']['animation']['tracks'] if t['joint'] == record['joints']['person/right_elbow'])
        assert track['keyframes'] == doc['animation']['tracks'][0]['keyframes']
        drive = next(d for d in result['document']['drives'] if d['driver'] == record['joints']['person/right_elbow'])
        assert drive['follower'] == record['joints']['person/left_elbow']
        assert drive['route_mm'][0] == [900+record['offset_mm'][0], 300, 500]


def test_neighbours_of_a_posed_limb_keep_only_internal_articulation(blank, factory):
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id': 'person', 'template': 'human'}]
    doc['state'] = {'joints': {'person/right_shoulder': {'rotation_deg': [20, 10, 15]}, 'person/right_elbow': {'angle_deg': 40}}}
    source = factory(doc)
    result = duplicate(source, 'person/right_forearm', 'touching')
    after = assert_copies(source, result, factory)
    assert set(result['copies'][0]['parts']) == {'person/right_upper_arm', 'person/right_forearm', 'person/right_hand'}
    assert len(after.joints) == 20 and len(after.parts) == 22
    assert set(result['copies'][0]['joints']) == {'person/right_elbow', 'person/right_wrist'}
    assert len(result['document']['objects']) == 1


def test_a_regrouped_person_keeps_component_edits_and_drops_its_grip(load, factory):
    source = load('human-pull-up')
    expanded = expand_objects(source, 'person')
    next(p for p in expanded['parts'] if p['id'] == 'person/right_hand')['mass_kg'] = 4.2
    source = factory(regroup_object(factory(expanded), 'person'))
    result = duplicate(source, 'person/right_hand', 'subassembly')
    after = assert_copies(source, result, factory)
    assert len(result['copies'][0]['parts']) == 19
    assert result['document']['objects'][0]['components'] == result['document']['objects'][1]['components']
    assert after.parts['person-copy/right_hand'].mass == 4.2
    copied = set(result['copies'][0]['parts'].values())
    assert not any(bool(j['a']['part'] in copied) != bool(j['b']['part'] in copied) for j in after.joints)


def test_custom_template_mesh_copy_strips_internal_world_anchors_and_survives_save_as(blank, tmp_path):
    folder = tmp_path/'library'; folder.mkdir()
    import trimesh
    trimesh.creation.box([20, 30, 40]).export(folder/'body.stl')
    template = {'parts': [{'id': 'body', 'body': {'kind': 'rigid', 'mass_kg': 2, 'geometry': [{'type': 'mesh', 'file': 'body.stl'}]}},
                           {'id': 'tip', 'body': {'kind': 'rigid', 'mass_kg': 1, 'geometry': [{'type': 'box', 'size_mm': [20, 30, 40]}]}, 'pose': {'position_mm': [20, 0, 0]}}],
                'joints': [{'id': 'join', 'type': 'fixed', 'a': {'part': 'body'}, 'b': {'part': 'tip'}}],
                'anchors': [{'part': 'body', 'surface': 'fixture'}]}
    write(folder/'parts.yaml', {'format': 'pipesim-library/1', 'name': 'Custom', 'objects': {'custom': template}})
    doc = copy.deepcopy(blank); doc['libraries'] = ['library/parts.yaml']
    doc['objects'] = [{'id': 'machine', 'template': 'custom', 'pose': {'position_mm': [20, 40, 60]}}]
    source = Assembly.from_doc(doc, tmp_path)
    for scope in ('part', 'subassembly'):
        result = duplicate(source, 'machine/body', scope)
        after = Assembly.from_doc(result['document'], tmp_path)
        assert after.anchors == source.anchors
        copied = after.parts[result['selected']]
        assert mesh_for_part(copied).is_volume
        save_base = tmp_path/'designs'
        relocated = relocate_design(result['document'], tmp_path, save_base)
        write(save_base/(scope+'.yaml'), relocated)
        reloaded = Assembly.load(save_base/(scope+'.yaml'))
        assert mesh_for_part(reloaded.parts[result['selected']]).is_volume
        assert len(reloaded.anchors) == 1


@pytest.mark.parametrize('count', [0, -1, 1.5, '3', True, 101, None])
def test_invalid_count_leaves_original_unchanged(blank, factory, count):
    source = graph(blank, factory); before = copy.deepcopy(source.doc)
    with pytest.raises(DocumentError, match='whole number'):
        duplicate(source, 'a', count=count)
    assert source.doc == before


def test_bad_selection_and_scope_are_actionable(blank, factory):
    source = graph(blank, factory)
    with pytest.raises(DocumentError, match='Select a part'):
        duplicate(source, 'missing')
    with pytest.raises(DocumentError, match='scope'):
        duplicate(source, 'a', 'everything')
