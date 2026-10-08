"""Three orthogonal scene mirrors make a physical cube from one corner."""

import copy
import json

import numpy as np
import pytest
from test_server import editor, request

from pipesim.document import Assembly, read, write
from pipesim.drafting import _mirror_reopen_digest, finalize
from pipesim.symmetry import materialize_all
from pipesim.validation import validate


def cube_corner(base, reach=300, origin=(0, 0, 1000)):
    reaches = (reach,)*3 if isinstance(reach, (int, float)) else tuple(reach)
    assert len(reaches) == 3 and all(value > 50 for value in reaches)
    corner = [origin[i]-reaches[i] for i in range(3)]
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': 'Mirrored square table cube',
           'parts': [{'id': 'corner', 'catalog': 'tubeclamp.TC128C',
                      'pose': {'position_mm': corner}}],
           'joints': [],
           'draft_subassemblies': [{'id': 'cube', 'runs': [],
                                    'mirrors': [
                                        {'id': f'{axis}-plane', 'axis': axis,
                                         'offset_mm': origin[i], 'scope': 'scene',
                                         'run_modes': {f'{axis}-edge': 'centered'}}
                                        for i, axis in enumerate('xyz')]}]}
    fitting = Assembly.from_doc({**doc, 'draft_subassemblies': []}, base).parts['corner']
    for i, axis in enumerate('xyz'):
        mouth, direction = fitting.frame({'port': axis})
        start = mouth-direction*30
        end = start.copy()
        end[i] = 2*origin[i]-start[i]
        assert np.linalg.norm(end-start) > 0
        doc['draft_subassemblies'][0]['runs'].append({
            'id': f'{axis}-edge', 'catalog': 'tubeclamp.tube-C',
            'start_mm': start.tolist(), 'end_mm': end.tolist(),
            'attachments': [{'connector': 'corner', 'port': axis,
                             'end': 'start', 'insertion_mm': 30}]})
    return doc


def test_three_mirrors_materialize_one_corner_into_connected_cube(tmp_path):
    doc = cube_corner(tmp_path)
    original = copy.deepcopy(doc)
    baked = materialize_all(Assembly.from_doc(doc, tmp_path))
    assert doc == original
    assert not baked['draft_subassemblies'][0].get('mirrors')
    assert len(baked['parts']) == 8
    runs = baked['draft_subassemblies'][0]['runs']
    assert len(runs) == 12
    assert all(len(run['attachments']) == 2 for run in runs)
    assert len({(a['connector'], a['port']) for run in runs
                for a in run['attachments']}) == 24
    Assembly.from_doc(baked, tmp_path)


def test_cube_finalizes_with_real_collision_checks(tmp_path):
    baked = materialize_all(Assembly.from_doc(cube_corner(tmp_path), tmp_path))
    result = finalize(Assembly.from_doc(baked, tmp_path))
    assert result['status'] == 'finalized', result
    exact = Assembly.from_doc(result['document'], tmp_path)
    assert len(exact.parts) == 20
    assert len(exact.joints) == 24
    assert validate(exact)['valid']


def test_editor_finalize_endpoint_accepts_three_mirror_cube(editor):
    status, raw = request(editor, '/api/draft-finalize',
                          {'document': cube_corner(editor.root)})
    response = json.loads(raw)
    assert status == 200 and response['status'] == 'finalized', response
    assert len(response['document']['parts']) == 20
    assert len(response['document']['joints']) == 24


def test_mirrored_cube_returns_to_live_draft_and_can_be_finalized_again(editor):
    original = cube_corner(editor.root)
    status, raw = request(editor, '/api/draft-finalize', {'document': original})
    exact = json.loads(raw)
    assert status == 200 and exact['status'] == 'finalized', exact
    assert len(exact['document']['parts']) == 20
    assert exact['document']['metadata']['draft_mirror_reopen']['final_digest'] == _mirror_reopen_digest(exact['document'])
    path = editor.root / 'mirrored-roundtrip.pipe.yaml'
    write(path, exact['document'])
    saved = read(path)

    status, raw = request(editor, '/api/draft-reopen', {
        'document': saved,
        'members': [part['id'] for part in saved['parts']]})
    reopened = json.loads(raw)
    assert status == 200 and reopened['status'] == 'reopened', reopened
    draft = reopened['document']
    assert {part['id'] for part in draft['parts']} == {'corner'}
    assert {run['id'] for run in draft['draft_subassemblies'][0]['runs']} == {
        'x-edge', 'y-edge', 'z-edge'}
    assert {plane['axis'] for plane in draft['draft_subassemblies'][0]['mirrors']} == {
        'x', 'y', 'z'}
    assert 'draft_mirror_reopen' not in draft.get('metadata', {})

    # The source run remains editable and its new free-end span is mirrored.
    edge = next(run for run in draft['draft_subassemblies'][0]['runs']
                if run['id'] == 'x-edge')
    edge['end_mm'][0] += 5
    status, raw = request(editor, '/api/draft-finalize', {'document': draft})
    again = json.loads(raw)
    assert status == 200 and again['status'] == 'finalized', again
    assert len(again['document']['parts']) == 20
    assert validate(Assembly.from_doc(again['document'], editor.root))['valid']


def test_one_pipe_reopens_its_linked_mirrored_assembly(editor):
    status, raw = request(editor, '/api/draft-finalize',
                          {'document': cube_corner(editor.root)})
    exact = json.loads(raw)
    assert status == 200 and exact['status'] == 'finalized', exact
    status, raw = request(editor, '/api/draft-reopen', {
        'document': exact['document'], 'members': ['x-edge']})
    reopened = json.loads(raw)
    assert status == 200 and reopened['status'] == 'reopened', reopened
    assert {part['id'] for part in reopened['document']['parts']} == {'corner'}
    assert len(reopened['document']['draft_subassemblies'][0]['mirrors']) == 3


def test_edited_exact_mirror_design_reopens_without_stale_mirror_history(editor):
    status, raw = request(editor, '/api/draft-finalize',
                          {'document': cube_corner(editor.root)})
    exact = json.loads(raw)
    assert status == 200 and exact['status'] == 'finalized', exact
    exact['document']['parts'][0]['pose']['position_mm'][0] += 1
    status, raw = request(editor, '/api/draft-reopen', {
        'document': exact['document'], 'members': ['x-edge']})
    reopened = json.loads(raw)
    assert status == 200 and reopened['status'] == 'reopened', reopened
    assert 'draft_mirror_reopen' not in reopened['document'].get('metadata', {})
    assert not reopened['document']['draft_subassemblies'][0].get('mirrors')


def socket_with_extra_obstacle(extra=False):
    """A valid socket with optional connector geometry hitting the pipe later."""
    shapes = [{'type': 'box', 'size_mm': [55, 55, 30],
               'position_mm': [0, 0, -15]}]
    if extra:
        shapes.append({'type': 'box', 'size_mm': [55, 55, 30],
                       'position_mm': [0, 0, 120]})
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
            'name': 'Socket collision region',
            'parts': [
                {'id': 'fitting', 'body': {'kind': 'connector', 'mass_kg': 1,
                                         'geometry': shapes,
                                         'ports': {'socket': {'type': 'socket',
                                                              'position_mm': [0, 0, 0],
                                                              'axis': [0, 0, 1],
                                                              'diameter_mm': 42.4,
                                                              'profile': 'round',
                                                              'engagement_mm': 40,
                                                              'min_engagement_mm': 20}}},
                 'pose': {'position_mm': [0, 0, 1000]}},
                {'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
                 'parameters': {'length_mm': 500},
                 'pose': {'position_mm': [0, 0, 1220]}}],
            'joints': [{'id': 'seat', 'type': 'socket',
                        'a': {'part': 'fitting', 'port': 'socket'},
                        'b': {'part': 'pipe', 'end': 'start'},
                        'insertion_mm': 30, 'locked': True}]}


def test_socket_penetration_is_allowed_only_near_seated_end(tmp_path):
    seat = validate(Assembly.from_doc(socket_with_extra_obstacle(), tmp_path))
    assert not any(i['code'] == 'INTERSECTION' for i in seat['issues']), seat
    obstructed = validate(Assembly.from_doc(socket_with_extra_obstacle(extra=True), tmp_path))
    assert any(i['code'] == 'INTERSECTION' and i['severity'] == 'error'
               for i in obstructed['issues']), obstructed
