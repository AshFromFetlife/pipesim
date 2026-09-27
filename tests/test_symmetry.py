import copy
import json

import numpy as np
import pytest
from test_server import editor

from pipesim.document import Assembly, DocumentError
from pipesim.geometry import mesh_for_part
from pipesim.symmetry import materialize_mirror, materialize_all


def design():
    def connector(identifier, x, y, axis):
        return {'id': identifier, 'body': {'kind': 'connector', 'mass_kg': .2,
            'geometry': [{'type': 'box', 'size_mm': [20, 10, 15], 'position_mm': [15, 4, 0]}],
            'ports': {'socket': {'type': 'socket', 'position_mm': [0, 0, 0], 'axis': axis,
                'diameter_mm': 42.4, 'profile': 'round', 'engagement_mm': 40,
                'min_engagement_mm': 15}}}, 'pose': {'position_mm': [x, y, 100]}}
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': 'Mirror fixture',
        'parts': [connector('left', 0, 200, [1, 0, 0]), connector('right', 1000, 200, [-1, 0, 0])],
        'joints': [], 'draft_subassemblies': [{'id': 'frame', 'runs': [{
            'id': 'tube', 'catalog': 'tubeclamp.tube-C', 'start_mm': [0, 200, 100],
            'end_mm': [1000, 200, 100], 'attachments': [
                {'connector': 'left', 'port': 'socket', 'end': 'start', 'insertion_mm': 20},
                {'connector': 'right', 'port': 'socket', 'end': 'end', 'insertion_mm': 20}]}],
            'mirrors': [{'id': 'y-zero', 'axis': 'y', 'offset_mm': 0}]}]}


def test_bake_creates_reflected_chiral_connectors_and_draft_links(tmp_path):
    doc = design()
    original = copy.deepcopy(doc)
    assembly = Assembly.from_doc(doc, tmp_path)
    result = materialize_mirror(assembly, 'frame', 'y-zero')
    assert doc == original
    assert not result['draft_subassemblies'][0].get('mirrors')
    assert len(result['draft_subassemblies'][0]['runs']) == 2
    mirrored = next(run for run in result['draft_subassemblies'][0]['runs'] if run['id'] != 'tube')
    assert mirrored['start_mm'] == [0, -200, 100]
    assert mirrored['end_mm'] == [1000, -200, 100]
    assert {a['connector'] for a in mirrored['attachments']} == {'left-mirror-y-zero', 'right-mirror-y-zero'}
    after = Assembly.from_doc(result, tmp_path)
    assert len(after.parts) == 4
    for original_id in ('left', 'right'):
        clone = after.parts[original_id+'-mirror-y-zero']
        source = assembly.parts[original_id]
        assert clone.spec.get('catalog') is None
        assert clone.spec['body']['geometry'][0]['file'].startswith('assets/mirrored/')
        original_port, mirrored_port = source.frame({'part': original_id, 'port': 'socket'}), clone.frame({'part': clone.id, 'port': 'socket'})
        assert np.allclose(mirrored_port[0], [original_port[0][0], -original_port[0][1], original_port[0][2]])
        assert np.allclose(mirrored_port[1], [original_port[1][0], -original_port[1][1], original_port[1][2]])
        original_vertices = mesh_for_part(source, world=True).vertices.copy()
        original_vertices[:, 1] *= -1
        reflected_vertices = mesh_for_part(clone, world=True).vertices
        assert np.array_equal(np.unique(original_vertices.round(3), axis=0),
                              np.unique(reflected_vertices.round(3), axis=0))


def test_declared_chiral_counterpart_uses_catalog_when_socket_frames_match(tmp_path):
    doc = design()
    source = doc['parts'][0]
    source.pop('body'); source['catalog'] = 'test.left'
    doc['definitions'] = {
        'test.left': {'kind': 'connector', 'mass_kg': .2, 'mirror_catalog': 'test.right',
            'geometry': [{'type': 'box', 'size_mm': [20, 10, 15], 'position_mm': [15, 4, 0]}],
            'ports': {'socket': {'type': 'socket', 'position_mm': [0, 0, 0], 'axis': [1, 0, 0],
                'diameter_mm': 42.4, 'profile': 'round', 'engagement_mm': 40, 'min_engagement_mm': 15}}},
        'test.right': {'kind': 'connector', 'mass_kg': .2,
            'geometry': [{'type': 'box', 'size_mm': [20, 10, 15], 'position_mm': [-15, 4, 0]}],
            'ports': {'socket': {'type': 'socket', 'position_mm': [0, 0, 0], 'axis': [-1, 0, 0],
                'diameter_mm': 42.4, 'profile': 'round', 'engagement_mm': 40, 'min_engagement_mm': 15}}}}
    baked = materialize_mirror(Assembly.from_doc(doc, tmp_path), 'frame', 'y-zero')
    reflected = next(p for p in baked['parts'] if p['id'] == 'left-mirror-y-zero')
    assert reflected['catalog'] == 'test.right'
    assert 'body' not in reflected


def test_perpendicular_centered_run_is_shared_by_mirrored_fitting(tmp_path):
    doc = design()
    doc['parts'] = [doc['parts'][0]]
    doc['parts'][0]['pose']['position_mm'] = [-480, 0, 100]
    doc['draft_subassemblies'][0]['runs'] = [{
        'id': 'tube', 'catalog': 'tubeclamp.tube-C', 'start_mm': [-500, 0, 100],
        'end_mm': [500, 0, 100], 'attachments': [
            {'connector': 'left', 'port': 'socket', 'end': 'start', 'insertion_mm': 20}]}]
    plane = doc['draft_subassemblies'][0]['mirrors'][0]
    plane.update({'axis': 'x', 'run_modes': {'tube': 'centered'}})
    result = materialize_mirror(Assembly.from_doc(doc, tmp_path), 'frame', 'y-zero')
    runs = result['draft_subassemblies'][0]['runs']
    assert len(runs) == 1
    assert {a['end'] for a in runs[0]['attachments']} == {'start', 'end'}
    assert {a['connector'] for a in runs[0]['attachments']} == {'left', 'left-mirror-y-zero'}
    Assembly.from_doc(result, tmp_path)


def test_moving_attached_fitting_resizes_unlocked_centered_draft(tmp_path):
    from pipesim.snapping import move_document
    from pipesim.drafting import _layout

    doc = design()
    doc['parts'] = [doc['parts'][0]]
    doc['parts'][0]['pose']['position_mm'] = [-480, 0, 100]
    doc['draft_subassemblies'][0]['runs'] = [{
        'id': 'tube', 'catalog': 'tubeclamp.tube-C', 'start_mm': [-500, 0, 100],
        'end_mm': [500, 0, 100], 'attachments': [
            {'connector': 'left', 'port': 'socket', 'end': 'start', 'insertion_mm': 20}]}]
    doc['draft_subassemblies'][0]['mirrors'][0].update(
        {'axis': 'x', 'run_modes': {'tube': 'centered'}})
    assembly = Assembly.from_doc(doc, tmp_path)
    moved = move_document(assembly, {'left': {'position_mm': [-483, 0, 100]}})
    run = moved['draft_subassemblies'][0]['runs'][0]
    assert run['start_mm'] == [-503, 0, 100]
    assert run['end_mm'] == [503, 0, 100]
    resolved = Assembly.from_doc(moved, tmp_path)
    assert _layout(resolved, run)['length'] == pytest.approx(1006)


def test_new_second_end_uses_insertion_slack_to_keep_draft_centered(tmp_path):
    from pipesim.drafting import connect, _layout

    doc = design()
    doc['parts'][0]['pose']['position_mm'] = [0, 0, 100]
    doc['parts'][1]['pose']['position_mm'] = [997, 0, 100]
    run = doc['draft_subassemblies'][0]['runs'][0]
    run['start_mm'] = [-20, 0, 100]
    run['end_mm'] = [1020, 0, 100]
    run['attachments'] = run['attachments'][:1]
    doc['draft_subassemblies'][0]['mirrors'][0].update(
        {'axis': 'x', 'offset_mm': 500, 'run_modes': {'tube': 'centered'}})
    connected = connect(Assembly.from_doc(doc, tmp_path), 'tube', 'right', 'socket', 'end', 20)
    after = connected['draft_subassemblies'][0]['runs'][0]
    assert after['attachments'][-1]['insertion_mm'] == pytest.approx(23)
    assert after['start_mm'] == [-20, 0, 100]
    assert after['end_mm'] == [1020, 0, 100]
    assert _layout(Assembly.from_doc(connected, tmp_path), after)['length'] == pytest.approx(1040)
    from pipesim.snapping import move_document
    moved = move_document(Assembly.from_doc(connected, tmp_path),
                          {'right': {'position_mm': [994, 0, 100]}})
    moved_run = moved['draft_subassemblies'][0]['runs'][0]
    assert moved_run['attachments'][-1]['insertion_mm'] == pytest.approx(26)
    assert _layout(Assembly.from_doc(moved, tmp_path), moved_run)['length'] == pytest.approx(1040)


def test_two_mirror_planes_make_four_quadrants(tmp_path):
    doc = design()
    doc['draft_subassemblies'][0]['mirrors'].append({'id': 'x-minus-500', 'axis': 'x', 'offset_mm': -500})
    baked = materialize_all(Assembly.from_doc(doc, tmp_path))
    assert len(baked['draft_subassemblies'][0]['runs']) == 4
    assert len(baked['parts']) == 8
    assert not baked['draft_subassemblies'][0].get('mirrors')
    Assembly.from_doc(baked, tmp_path)


def test_kept_copy_inherits_constraint_on_the_remaining_plane(tmp_path):
    doc = design()
    doc['parts'] = []
    doc['draft_subassemblies'][0]['runs'][0]['attachments'] = []
    doc['draft_subassemblies'][0]['mirrors'].append({
        'id': 'deck', 'axis': 'z', 'offset_mm': 100, 'run_modes': {'tube': 'in_plane'}})
    baked = materialize_mirror(Assembly.from_doc(doc, tmp_path), 'frame', 'y-zero')
    group = baked['draft_subassemblies'][0]
    assert len(group['runs']) == 2
    assert group['mirrors'][0]['run_modes'] == {'tube': 'in_plane', 'tube-mirror-y-zero': 'in_plane'}


def test_invalid_on_plane_constraint_is_rejected(tmp_path):
    doc = design()
    doc['draft_subassemblies'][0]['mirrors'][0]['run_modes'] = {'tube': 'in_plane'}
    with pytest.raises(DocumentError, match='centreline'):
        Assembly.from_doc(doc, tmp_path)


def test_explicit_free_mode_does_not_constrain_a_pipe(tmp_path):
    doc = design()
    doc['draft_subassemblies'][0]['mirrors'][0]['run_modes'] = {'tube': 'free'}
    Assembly.from_doc(doc, tmp_path)


def test_pipe_centreline_can_lie_at_an_angle_in_a_height_plane(tmp_path):
    doc = design()
    doc['parts'] = []
    doc['draft_subassemblies'][0]['runs'] = [{
        'id': 'tube', 'catalog': 'tubeclamp.tube-C',
        'start_mm': [0, 0, 350], 'end_mm': [700, 300, 350], 'attachments': []}]
    doc['draft_subassemblies'][0]['mirrors'] = [{
        'id': 'deck', 'axis': 'z', 'offset_mm': 350, 'run_modes': {'tube': 'in_plane'}}]
    baked = materialize_mirror(Assembly.from_doc(doc, tmp_path), 'frame', 'deck')
    assert len(baked['draft_subassemblies'][0]['runs']) == 1
    assert not baked['draft_subassemblies'][0].get('mirrors')


def test_selected_finalize_bakes_only_its_mirror_group(editor):
    from test_server import request

    doc = design()
    doc['parts'] = []
    doc['draft_subassemblies'][0]['runs'][0]['attachments'] = []
    doc['draft_subassemblies'][0]['runs'].append({
        'id': 'separate-in-frame', 'catalog': 'tubeclamp.tube-C',
        'start_mm': [0, 500, 100], 'end_mm': [1000, 500, 100]})
    doc['draft_subassemblies'].append({'id': 'other', 'runs': [{
        'id': 'other-tube', 'catalog': 'tubeclamp.tube-C',
        'start_mm': [0, 900, 100], 'end_mm': [1000, 900, 100]}],
        'mirrors': [{'id': 'other-mirror', 'axis': 'y', 'offset_mm': 700}]})
    status, raw = request(editor, '/api/draft-finalize-selected', {'document': doc, 'run': 'tube'})
    result = json.loads(raw)
    assert status == 200 and result['status'] == 'finalized', result
    assert {part['id'] for part in result['document']['parts']} == {'tube', 'tube-mirror-y-zero'}
    assert {group['id'] for group in result['document']['draft_subassemblies']} == {'frame', 'other'}
    frame = next(group for group in result['document']['draft_subassemblies'] if group['id'] == 'frame')
    assert {run['id'] for run in frame['runs']} == {'separate-in-frame', 'separate-in-frame-mirror-y-zero'}
    other = next(group for group in result['document']['draft_subassemblies'] if group['id'] == 'other')
    assert other['mirrors'][0]['id'] == 'other-mirror'
