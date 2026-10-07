"""A table edge can join fittings that each lie on a different mirror."""

import copy

import numpy as np
import pytest

from pipesim.document import Assembly, DocumentError
from pipesim.drafting import finalize
from pipesim.symmetry import materialize_all, materialize_mirror


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


def test_on_plane_fitting_without_reflected_socket_stays_rejected(tmp_path):
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
    # A single-plane mirror is enough to exercise the refusal. Keep the pipe
    # geometry self-consistent with this deliberately asymmetric part.
    part['pose']['rotation_deg'] = [0, 0, -45]
    doc['draft_subassemblies'][0]['mirrors'] = [
        {'id': 'x-plane', 'axis': 'x', 'offset_mm': 0, 'scope': 'scene'}]
    assembly = Assembly.from_doc(doc, tmp_path)
    with pytest.raises(DocumentError, match='no distinct reflected socket|mirror plane'):
        materialize_mirror(assembly, 'table', 'x-plane')


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
