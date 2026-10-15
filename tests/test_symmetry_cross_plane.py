"""A table edge can join fittings that each lie on a different mirror."""

import copy
import json

import numpy as np
import pytest

from pipesim.document import Assembly
from pipesim.drafting import finalize
from pipesim.symmetry import materialize_all, materialize_mirror
from test_server import editor, request


def square_table_edge(base, *, first_axis='x', x_offset=0, y_offset=0,
                      reach=300, height=1000):
    """One diagonal edge of a square with TC128C fittings on the axes."""
    # The corner's x/y sockets are symmetric about its respective mirror.
    # Each reflected edge must use the *other* bore of the same fitting.
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': 'Four-way mirrored square table edge',
               'parts': [
               {'id': 'on-x', 'catalog': 'tubeclamp.TC128C',
                'pose': {'position_mm': [x_offset, y_offset+reach, height],
                         'rotation_deg': [0, 0, 225]}},
               {'id': 'on-y', 'catalog': 'tubeclamp.TC128C',
                'pose': {'position_mm': [x_offset+reach, y_offset, height],
                         'rotation_deg': [0, 0, 135]}}],
           'joints': [],
           'draft_subassemblies': [{'id': 'table', 'runs': [],
                                    'mirrors': [
                                        {'id': 'x-plane', 'axis': 'x', 'offset_mm': x_offset,
                                         'scope': 'scene'},
                                        {'id': 'y-plane', 'axis': 'y', 'offset_mm': y_offset,
                                         'scope': 'scene'}]}]}
    if first_axis == 'y':
        doc['draft_subassemblies'][0]['mirrors'].reverse()
    assembly = Assembly.from_doc(doc, base)
    start_mouth, start_axis = assembly.parts['on-x'].frame({'port': 'y'})
    end_mouth, end_axis = assembly.parts['on-y'].frame({'port': 'x'})
    start = start_mouth - start_axis * 30
    end = end_mouth - end_axis * 30
    assert np.dot(end-start, start_axis) > 0
    assert np.linalg.norm(np.cross(end-start, start_axis)) < 1e-7
    doc['draft_subassemblies'][0]['runs'] = [{
        'id': 'edge', 'catalog': 'tubeclamp.tube-C',
        'start_mm': start.tolist(), 'end_mm': end.tolist(),
        'attachments': [
            {'connector': 'on-x', 'port': 'y', 'end': 'start', 'insertion_mm': 30},
            {'connector': 'on-y', 'port': 'x', 'end': 'end', 'insertion_mm': 30}]}]
    return doc


@pytest.mark.parametrize('first_axis', ['x', 'y'])
def test_cross_plane_edge_materializes_four_distinct_runs_with_valid_sockets(tmp_path, first_axis):
    doc = square_table_edge(tmp_path, first_axis=first_axis)
    original = copy.deepcopy(doc)
    baked = materialize_all(Assembly.from_doc(doc, tmp_path))
    assert doc == original
    group = baked['draft_subassemblies'][0]
    assert not group.get('mirrors')
    assert len(group['runs']) == 4
    assert len(baked['parts']) == 4
    occupied = [(a['connector'], a['port']) for run in group['runs']
                for a in run['attachments']]
    assert len(occupied) == len(set(occupied)) == 8
    for run in group['runs']:
        assert len({a['connector'] for a in run['attachments']}) == 2
    assert {port for connector, port in occupied if connector == 'on-x'} == {'x', 'y'}
    assert {port for connector, port in occupied if connector == 'on-y'} == {'x', 'y'}
    Assembly.from_doc(baked, tmp_path)


@pytest.mark.parametrize('first_axis', ['x', 'y'])
def test_cross_plane_edge_finalizes_as_exact_four_edge_square(tmp_path, first_axis):
    doc = square_table_edge(tmp_path, first_axis=first_axis)
    baked = materialize_all(Assembly.from_doc(doc, tmp_path))
    result = finalize(Assembly.from_doc(baked, tmp_path))
    assert result['status'] == 'finalized', result
    exact = Assembly.from_doc(result['document'], tmp_path)
    assert not exact.doc.get('draft_subassemblies')
    assert len(exact.joints) == 8
    assert {part_id for part_id, part in exact.parts.items() if part.kind == 'member'} == {
        run['id'] for run in baked['draft_subassemblies'][0]['runs']}


@pytest.mark.parametrize('x_offset,y_offset,reach,height', [
    (-125, 240, 180, 550),
    (750, -125, 500, 1300),
    (-1000, -500, 900, 2200),
])
def test_cross_plane_materialization_respects_offsets_and_table_sizes(
        tmp_path, x_offset, y_offset, reach, height):
    doc = square_table_edge(tmp_path, x_offset=x_offset, y_offset=y_offset,
                            reach=reach, height=height)
    baked = materialize_all(Assembly.from_doc(doc, tmp_path))
    assembly = Assembly.from_doc(baked, tmp_path)
    assert len(baked['draft_subassemblies'][0]['runs']) == 4
    assert len(assembly.parts) == 4
    assert {round(part.matrix[2, 3]) for part in assembly.parts.values()} == {height}
    assert {'on-x', 'on-x-mirror-y-plane'} <= set(assembly.parts)
    assert {'on-y', 'on-y-mirror-x-plane'} <= set(assembly.parts)


def test_on_plane_fitting_without_reflected_socket_warns_and_keeps_original(tmp_path, editor):
    doc = square_table_edge(tmp_path)
    # A fitting on the plane with only one off-plane socket cannot host both
    # reflected edges. The other side needs a real, distinct socket.
    part = doc['parts'][0]
    part.pop('catalog')
    part['body'] = {'kind': 'connector', 'mass_kg': .2,
                    'geometry': [{'type': 'box', 'size_mm': [20, 20, 20]}],
                    'ports': {'y': {'type': 'socket', 'position_mm': [-62.8, 0, 0],
                                    'axis': [-1, 0, 0], 'diameter_mm': 42.4,
                                    'profile': 'round', 'engagement_mm': 37.1,
                                    'min_engagement_mm': 24.115}}}
    # A single-plane mirror exercises the warning. Keep the pipe geometry
    # self-consistent with this deliberately asymmetric part.
    part['pose']['rotation_deg'] = [0, 0, -45]
    doc['draft_subassemblies'][0]['mirrors'] = [
        {'id': 'x-plane', 'axis': 'x', 'offset_mm': 0, 'scope': 'scene'}]
    assembly = Assembly.from_doc(doc, tmp_path)
    warnings = []
    baked = materialize_mirror(assembly, 'table', 'x-plane', warnings=warnings)
    assert baked['draft_subassemblies'][0].get('mirrors') is None
    assert len(baked['draft_subassemblies'][0]['runs']) == 1
    assert warnings == [{'code': 'MIRROR_UNAVAILABLE', 'part': 'on-x',
        'run': 'edge', 'message':
        'edge: on-x/y has no second socket for a reflected pipe. The original pipe was kept.'}]
    status, raw = request(editor, '/api/draft-mirror-bake',
                          {'document': doc, 'group': 'table', 'plane': 'x-plane'})
    result = json.loads(raw)
    assert status == 200
    assert result['warnings'] == warnings


@pytest.mark.parametrize('mouth_offset', [0, 20])
def test_missing_reflected_socket_does_not_block_draft_finalization(editor, mouth_offset):
    def fitting(identifier, x, direction, mouth_offset=0):
        return {'id': identifier, 'body': {'kind': 'connector', 'mass_kg': .2,
            'geometry': [{'type': 'sphere', 'radius_mm': 5,
                          'position_mm': [0, 0, 80]}],
            'ports': {'end': {'type': 'socket', 'position_mm': [mouth_offset, 0, 0],
                'axis': direction, 'diameter_mm': 42.4, 'profile': 'round',
                'engagement_mm': 40, 'min_engagement_mm': 15}}},
            'pose': {'position_mm': [x, 0, 100]}}
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': 'One-sided fitting',
           'parts': [fitting('on-plane', 0, [1, 0, 0], mouth_offset=mouth_offset),
                     fitting('far', 500, [-1, 0, 0])],
           'joints': [], 'draft_subassemblies': [{'id': 'frame', 'runs': [{
               'id': 'edge', 'catalog': 'tubeclamp.tube-C',
               'start_mm': [mouth_offset-20, 0, 100], 'end_mm': [520, 0, 100],
               'attachments': [
                   {'connector': 'on-plane', 'port': 'end', 'end': 'start',
                    'insertion_mm': 20},
                   {'connector': 'far', 'port': 'end', 'end': 'end',
                    'insertion_mm': 20}]}],
               'mirrors': [{'id': 'x-plane', 'axis': 'x', 'offset_mm': 0,
                            'scope': 'scene'}]}]}
    status, raw = request(editor, '/api/draft-finalize', {'document': doc})
    result = json.loads(raw)
    assert status == 200
    assert result['status'] == 'finalized', result
    assert result['warnings'][0]['part'] == 'on-plane'
    assert result['warnings'][0]['code'] == 'MIRROR_UNAVAILABLE'
    assert not result['document'].get('draft_subassemblies')


def nearly_symmetric_corner():
    # Reproduces the user's on-plane corner fitting. Its two usable bores
    # reflect within 0.0032 mm / 0.0026 degrees despite rounded saved poses.
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': 'Nearly symmetric corner', 'parts': [
               {'id': 'tc116c-1', 'catalog': 'tubeclamp.TC116C', 'pose': {
                   'position_mm': [-0.00022435, 0.02075925, 1000.00943213],
                   'rotation_deg': [-0.00081308, 0.0016363, -135.00122496]}},
               {'id': 'tc173fc-1', 'catalog': 'tubeclamp.TC173FC', 'pose': {
                   'position_mm': [459.5535, -459.5535, 1000],
                   'rotation_deg': [45, 90, 0]}}],
           'joints': [], 'draft_subassemblies': [{'id': 'draft-1', 'runs': [{
               'id': 'tube-c-1-copy-2', 'catalog': 'tubeclamp.tube-C',
               'start_mm': [23.475945135393385, -23.475945135393378, 1000],
               'end_mm': [480.7667004623066, -480.7667004623065, 1000],
               'attachments': [
                   {'connector': 'tc116c-1', 'port': 'y', 'end': 'start',
                    'insertion_mm': 30},
                   {'connector': 'tc173fc-1', 'port': 'socket', 'end': 'end',
                    'insertion_mm': 30}]}],
               'mirrors': [{'id': 'mirror-1', 'axis': 'x', 'offset_mm': 0,
                            'scope': 'scene'}]}]}


def test_nearly_symmetric_tc116c_sockets_bake_and_finalize(tmp_path):
    doc = nearly_symmetric_corner()
    original = copy.deepcopy(doc)
    assembly = Assembly.from_doc(doc, tmp_path, validate_mirror_geometry=False)
    corner = assembly.parts['tc116c-1']
    _, used_axis = corner.frame({'port': 'y'})
    _, other_axis = corner.frame({'port': 'x'})
    reflected_axis = used_axis.copy()
    reflected_axis[0] *= -1
    assert 1e-5 < np.linalg.norm(other_axis-reflected_axis) < 1e-3
    warnings = []
    baked = materialize_mirror(assembly, 'draft-1', 'mirror-1', warnings=warnings)
    assert warnings == []
    runs = baked['draft_subassemblies'][0]['runs']
    assert len(runs) == 2
    assert not any(part['id'].startswith('tc116c-1-mirror-') for part in baked['parts'])
    assert {a['port'] for run in runs for a in run['attachments']
            if a['connector'] == 'tc116c-1'} == {'x', 'y'}
    result = finalize(Assembly.from_doc(baked, tmp_path,
                                        validate_mirror_geometry=False))
    assert result['status'] == 'finalized', result
    assert doc == original


def test_nearly_symmetric_corner_finalizes_through_editor_api(editor):
    doc = nearly_symmetric_corner()
    status, raw = request(editor, '/api/draft-finalize', {'document': doc})
    result = json.loads(raw)
    assert status == 200
    assert result['status'] == 'finalized', result
    assert not result.get('warnings')
    assert not result['document'].get('draft_subassemblies')


def test_existing_exact_socket_joint_uses_other_on_plane_bore(tmp_path):
    doc = square_table_edge(tmp_path)
    doc['draft_subassemblies'][0].pop('mirrors')
    source = finalize(Assembly.from_doc(doc, tmp_path), check_collisions=False)
    assert source['status'] == 'finalized', source
    exact = source['document']
    exact['draft_subassemblies'] = [{'id': 'table', 'runs': [], 'mirrors': [
        {'id': 'x-plane', 'axis': 'x', 'offset_mm': 0, 'scope': 'scene'}]}]
    mirrored = materialize_mirror(Assembly.from_doc(exact, tmp_path), 'table', 'x-plane')
    joints = mirrored['joints']
    assert len(joints) == 4
    assert {(j['a']['part'], j['a']['port']) for j in joints
            if j['a']['part'] == 'on-x'} == {('on-x', 'x'), ('on-x', 'y')}
    Assembly.from_doc(mirrored, tmp_path)
