import copy
import json
import threading
import urllib.error
from pathlib import Path
import numpy as np
import pytest
import yaml
from test_server import editor

from pipesim.document import Assembly, DocumentError, write
from pipesim.drafting import connect, finalize, preview, repair, reopen, runs


def design(second_axis=(-1, 0, 0), second_y=0):
    def fitting(identifier, x, y, axis):
        return {'id': identifier, 'body': {'kind': 'connector', 'mass_kg': .2,
                'geometry': [{'type': 'sphere', 'radius_mm': 6, 'position_mm': [-80*a for a in axis]}],
                'ports': {'socket': {'type': 'socket', 'position_mm': [0, 0, 0], 'axis': list(axis),
                                     'diameter_mm': 42.4, 'profile': 'round',
                                     'engagement_mm': 40, 'min_engagement_mm': 15}}},
                'pose': {'position_mm': [x, y, 100]}}
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': 'Draft fixture',
            'parts': [fitting('left', 0, 0, (1, 0, 0)), fitting('right', 1000, second_y, second_axis)],
            'joints': [], 'draft_subassemblies': [{'id': 'frame', 'runs': [
                {'id': 'tube-1', 'catalog': 'tubeclamp.tube-C', 'start_mm': [0, 0, 100],
                 'end_mm': [1000, 0, 100]}]}]}


def test_draft_connections_materialize_one_exact_cut_length():
    doc = design()
    before = copy.deepcopy(doc)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'right', 'socket', 'end', 20)
    assert before['draft_subassemblies'][0]['runs'][0]['start_mm'] == [0, 0, 100]
    assert len(Assembly.from_doc(doc).parts) == 2
    assert preview(Assembly.from_doc(doc))[0]['length_mm'] == 1040
    result = finalize(Assembly.from_doc(doc))
    assert result['status'] == 'finalized', result
    assert result['lengths_mm']['tube-1'] == 1040
    finished = Assembly.from_doc(result['document'])
    assert not result['document'].get('draft_subassemblies')
    assert len(finished.joints) == 2


def test_one_draft_pipe_cannot_use_two_sockets_of_one_connector():
    doc = design()
    doc['parts'].append({'id': 'double', 'body': {'kind': 'connector', 'mass_kg': .2,
                         'geometry': [{'type': 'sphere', 'radius_mm': 6}],
                         'ports': {name: {'type': 'socket', 'through': True, 'profile': 'round',
                                          'diameter_mm': 42.4, 'position_mm': [x, 0, 0],
                                          'axis': [1, 0, 0], 'engagement_mm': 30}
                                   for name, x in [('first', -30), ('second', 30)]}},
                         'pose': {'position_mm': [500, 0, 100]}})
    ambiguous = Assembly.from_doc(copy.deepcopy(doc))
    assert any(c['code'] == 'AMBIGUOUS_CONNECTOR' for c in repair(ambiguous)['conflicts'])
    assert any(c['code'] == 'AMBIGUOUS_CONNECTOR' for c in finalize(ambiguous)['conflicts'])
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'double', 'first')
    with pytest.raises(DocumentError, match='one pipe cannot occupy two sockets'):
        connect(Assembly.from_doc(doc), 'tube-1', 'double', 'second')
    # Existing invalid drafts remain openable and savable so their attachment can be removed.
    doc['draft_subassemblies'][0]['runs'][0]['attachments'].append(
        {'connector': 'double', 'port': 'second'})
    assembly = Assembly.from_doc(doc)
    assert any(c['code'] == 'DUPLICATE_CONNECTOR' for c in preview(assembly)[0]['conflicts'])
    assert any(c['code'] == 'DUPLICATE_CONNECTOR' for c in repair(assembly)['conflicts'])
    assert any(c['code'] == 'DUPLICATE_CONNECTOR' for c in finalize(assembly)['conflicts'])


def test_repair_uses_unrounded_through_gap_at_display_tolerance():
    doc = design()
    doc['parts'] = [{'id': 'through', 'body': {'kind': 'connector', 'mass_kg': .2,
                     'geometry': [{'type': 'sphere', 'radius_mm': 6,
                                   'position_mm': [0, 80, 0]}],
                     'ports': {'bore': {'type': 'socket', 'through': True,
                                        'profile': 'round', 'diameter_mm': 42.4,
                                        'axis': [1, 0, 0], 'engagement_mm': 40}}},
                     'pose': {'position_mm': [500, 0, 100]}}]
    doc['anchors'] = [{'part': 'through', 'surface': 'fixture'}]
    run = doc['draft_subassemblies'][0]['runs'][0]
    run['start_mm'] = [0, 1.0004, 100]
    run['end_mm'] = [1000, 1.0004, 100]
    run['attachments'] = [{'connector': 'through', 'port': 'bore'}]
    conflict = preview(Assembly.from_doc(doc))[0]['conflicts'][0]
    assert conflict['code'] == 'THROUGH_FIT'
    assert conflict['radial_gap_mm'] == 1.0
    assert conflict['radial_gap_exact_mm'] > 1.0

    result = repair(Assembly.from_doc(doc))
    assert result['status'] == 'repaired', result
    assert not preview(Assembly.from_doc(result['document']))[0]['conflicts']
    finalized = finalize(Assembly.from_doc(result['document']))
    assert finalized['status'] == 'finalized', finalized


def test_finished_pipe_returns_to_draft_with_its_span_and_socket_graph():
    doc = design()
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'right', 'socket', 'end', 20)
    exact = finalize(Assembly.from_doc(doc), check_collisions=False)['document']
    result = reopen(Assembly.from_doc(exact), ['left', 'tube-1', 'right'])
    assert result['status'] == 'reopened'
    assert {p['id'] for p in result['document']['parts']} == {'left', 'right'}
    assert result['document']['joints'] == []
    run = runs(result['document'])[0]
    assert run['id'] == 'tube-1'
    assert 'locked_length_mm' not in run
    assert pytest.approx(1040) == sum((a-b)**2 for a,b in zip(run['start_mm'],run['end_mm']))**.5
    assert run['attachments'] == [
        {'connector': 'left', 'port': 'socket', 'end': 'start', 'insertion_mm': 20},
        {'connector': 'right', 'port': 'socket', 'end': 'end', 'insertion_mm': 20}]
    assert preview(Assembly.from_doc(result['document']))[0]['conflicts'] == []
    again = finalize(Assembly.from_doc(result['document']), check_collisions=False)
    assert again['status'] == 'finalized', again
    assert again['lengths_mm']['tube-1'] == 1040


def test_reopen_preserves_other_draft_structure_and_derives_through_station():
    doc = design()
    doc['parts'].append({'id': 'through', 'body': {'kind': 'connector', 'mass_kg': .2,
                         'geometry': [{'type': 'sphere', 'radius_mm': 6, 'position_mm': [0, 80, 0]}],
                         'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                                            'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                                            'axis': [1, 0, 0], 'engagement_mm': 34}}},
                         'pose': {'position_mm': [500, 0, 100]}})
    doc['draft_subassemblies'].append({'id': 'other', 'runs': [
        {'id': 'tube-2', 'catalog': 'tubeclamp.tube-C', 'start_mm': [0, 500, 100], 'end_mm': [1000, 500, 100]}]})
    for fitting, end in [('left', 'start'), ('right', 'end')]:
        doc = connect(Assembly.from_doc(doc), 'tube-1', fitting, 'socket', end, 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'through', 'bore')
    exact = finalize(Assembly.from_doc(doc), check_collisions=False, run_id='tube-1')['document']
    result = reopen(Assembly.from_doc(exact), ['tube-1'])
    assert [group['id'] for group in result['document']['draft_subassemblies']] == ['other', 'draft-1']
    assert {run['id'] for run in runs(result['document'])} == {'tube-1', 'tube-2'}
    attachment = next(a for a in runs(result['document'])[1]['attachments'] if a['connector'] == 'through')
    assert attachment == {'connector': 'through', 'port': 'bore'}
    again = finalize(Assembly.from_doc(result['document']), check_collisions=False, run_id='tube-1')
    assert again['status'] == 'finalized', again
    assert next(j for j in again['document']['joints'] if j['a']['part'] == 'through')['b']['at_mm'] == 520


def test_reopen_rejects_unsupported_constraints_without_modifying_exact_design():
    doc = design()
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    exact = finalize(Assembly.from_doc(doc), check_collisions=False)['document']
    before = copy.deepcopy(exact)
    exact['anchors'] = [{'part': 'tube-1', 'surface': 'fixture'}]
    with pytest.raises(DocumentError, match='anchors'):
        reopen(Assembly.from_doc(exact), ['tube-1'])
    assert before['parts'] == exact['parts']
    exact.pop('anchors')
    exact['joints'][0]['locked'] = False
    with pytest.raises(DocumentError, match='cannot represent'):
        reopen(Assembly.from_doc(exact), ['tube-1'])


def test_reopen_keeps_free_pipe_roll_on_preview_and_refinalization():
    doc = design()
    doc.pop('draft_subassemblies')
    doc['parts'].append({'id': 'tube-1', 'catalog': 'tubeclamp.tube-C',
                         'parameters': {'length_mm': 750},
                         'pose': {'position_mm': [300, 200, 600], 'rotation_deg': [0, 0, 37]}})
    original = Assembly.from_doc(doc).parts['tube-1'].matrix
    draft = reopen(Assembly.from_doc(doc), ['tube-1'])['document']
    draft_pose = preview(Assembly.from_doc(draft))[0]['pose']
    from pipesim.math3d import transform
    assert np.allclose(transform(draft_pose), original, atol=1e-5)
    exact = finalize(Assembly.from_doc(draft), check_collisions=False)
    assert exact['status'] == 'finalized', exact
    assert np.allclose(Assembly.from_doc(exact['document']).parts['tube-1'].matrix, original, atol=1e-5)


def test_reopen_converts_multiple_members_in_one_draft_group():
    doc = design()
    doc.pop('draft_subassemblies')
    for pid, y in [('tube-1', 200), ('tube-2', 400)]:
        doc['parts'].append({'id': pid, 'catalog': 'tubeclamp.tube-C',
                             'parameters': {'length_mm': 600}, 'pose': {'position_mm': [0, y, 500]}})
    result = reopen(Assembly.from_doc(doc), ['tube-1', 'tube-2'])
    assert result['converted_parts'] == ['tube-1', 'tube-2']
    assert {run['id'] for run in result['document']['draft_subassemblies'][0]['runs']} == {'tube-1', 'tube-2'}
    assert {part['id'] for part in result['document']['parts']} == {'left', 'right'}


@pytest.mark.parametrize('separate_groups', [False, True])
def test_finalizing_a_selected_run_preserves_other_draft_structures(separate_groups):
    doc = design()
    for part in copy.deepcopy(doc['parts']):
        part['id'] += '-other'
        part['pose']['position_mm'][1] = 500
        doc['parts'].append(part)
    doc['anchors'] = [{'part': 'left', 'surface': 'fixture'}]
    other = {'id': 'tube-2', 'catalog': 'tubeclamp.tube-C',
             'start_mm': [0, 500, 100], 'end_mm': [1000, 500, 100]}
    if separate_groups:
        doc['draft_subassemblies'].append({'id': 'other', 'runs': [other]})
    else:
        doc['draft_subassemblies'][0]['runs'].append(other)
    for run_id, suffix in [('tube-1', ''), ('tube-2', '-other')]:
        doc = connect(Assembly.from_doc(doc), run_id, 'left'+suffix, 'socket', 'start', 20)
        doc = connect(Assembly.from_doc(doc), run_id, 'right'+suffix, 'socket', 'end', 20)
    result = finalize(Assembly.from_doc(doc), check_collisions=False, run_id='tube-1')
    assert result['status'] == 'finalized', result
    assert {p['id'] for p in result['document']['parts']} == {'left', 'right', 'left-other', 'right-other', 'tube-1'}
    assert [run['id'] for run in runs(result['document'])] == ['tube-2']
    assert preview(Assembly.from_doc(result['document']))[0]['id'] == 'tube-2'
    second = finalize(Assembly.from_doc(result['document']), check_collisions=False, run_id='tube-2')
    assert second['status'] == 'finalized', second
    assert not second['document'].get('draft_subassemblies')


def test_selected_finalization_includes_draft_runs_sharing_a_fitting():
    doc = design()
    doc['anchors'] = [{'part': 'left', 'surface': 'fixture'}]
    doc['parts'].append({'id': 'hub', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 6}], 'ports': {
            'west': {'type': 'socket', 'position_mm': [0, 0, 0], 'axis': [-1, 0, 0],
                     'diameter_mm': 42.4, 'profile': 'round', 'engagement_mm': 40, 'min_engagement_mm': 15},
            'east': {'type': 'socket', 'position_mm': [0, 0, 0], 'axis': [1, 0, 0],
                     'diameter_mm': 42.4, 'profile': 'round', 'engagement_mm': 40, 'min_engagement_mm': 15}}},
        'pose': {'position_mm': [500, 0, 100]}})
    doc['draft_subassemblies'][0]['runs'][0]['end_mm'] = [500, 0, 100]
    doc['draft_subassemblies'][0]['runs'].append({'id': 'tube-2', 'catalog': 'tubeclamp.tube-C',
        'start_mm': [500, 0, 100], 'end_mm': [1000, 0, 100]})
    for run_id, fitting, port, end in [('tube-1', 'left', 'socket', 'start'),
                                       ('tube-1', 'hub', 'west', 'end'),
                                       ('tube-2', 'hub', 'east', 'start'),
                                       ('tube-2', 'right', 'socket', 'end')]:
        doc = connect(Assembly.from_doc(doc), run_id, fitting, port, end, 20)
    result = finalize(Assembly.from_doc(doc), check_collisions=False, run_id='tube-1')
    assert result['status'] == 'finalized', result
    assert {'tube-1', 'tube-2'} <= {part['id'] for part in result['document']['parts']}
    assert not result['document'].get('draft_subassemblies')


def test_selected_finalization_recovers_unrecorded_through_fit_across_draft_groups():
    doc = design()
    doc['anchors'] = [{'part': 'left', 'surface': 'fixture'}]
    doc['parts'].append({'id': 'hub', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 6, 'position_mm': [0, 80, 0]}],
        'ports': {
            'through': {'type': 'socket', 'through': True, 'profile': 'round',
                        'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                        'axis': [1, 0, 0], 'engagement_mm': 34},
            'branch': {'type': 'socket', 'profile': 'round', 'diameter_mm': 42.4,
                       'position_mm': [0, 0, 0], 'axis': [0, 1, 0],
                       'engagement_mm': 40, 'min_engagement_mm': 15}}},
        'pose': {'position_mm': [500, 0, 100]}})
    doc['draft_subassemblies'].append({'id': 'other', 'runs': [
        {'id': 'branch-run', 'catalog': 'tubeclamp.tube-C',
         'start_mm': [500, 0, 100], 'end_mm': [500, 500, 100]}]})
    for fitting, end in [('left', 'start'), ('right', 'end')]:
        doc = connect(Assembly.from_doc(doc), 'tube-1', fitting, 'socket', end, 20)
    doc = connect(Assembly.from_doc(doc), 'branch-run', 'hub', 'branch', 'start', 20)
    assert not any(a['connector'] == 'hub' for a in runs(doc)[0]['attachments'])
    result = finalize(Assembly.from_doc(doc), check_collisions=False, run_id='tube-1')
    assert result['status'] == 'finalized', result
    assert result['inferred_through_connections'] == 1
    assert set(result['lengths_mm']) == {'tube-1', 'branch-run'}
    assert not result['document'].get('draft_subassemblies')
    assert any(j['a'] == {'part': 'hub', 'port': 'through'} and j['b']['part'] == 'tube-1'
               for j in result['document']['joints'])


@pytest.mark.parametrize('x,y', [(500, 6), (-10, 0)])
def test_finalization_reports_an_unattached_near_through_fit(x, y):
    doc = design()
    doc['parts'].append({'id': 'through', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 6, 'position_mm': [0, 80, 0]}],
        'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                           'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                           'axis': [1, 0, 0], 'engagement_mm': 34}}},
        'pose': {'position_mm': [x, y, 100]}})
    for fitting, end in [('left', 'start'), ('right', 'end')]:
        doc = connect(Assembly.from_doc(doc), 'tube-1', fitting, 'socket', end, 20)
    result = finalize(Assembly.from_doc(doc), check_collisions=False, run_id='tube-1')
    assert result['status'] == 'conflict'
    assert any(c['code'] == 'UNATTACHED_THROUGH' and c['connector'] == 'through'
               for c in result['conflicts'])
    assert runs(doc)[0]['id'] == 'tube-1'


def test_finalization_does_not_guess_between_two_pipes_in_one_through_socket():
    doc = design()
    doc['parts'].append({'id': 'through', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 6, 'position_mm': [0, 80, 0]}],
        'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                           'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                           'axis': [1, 0, 0], 'engagement_mm': 34}}},
        'pose': {'position_mm': [500, 0, 100]}})
    other = copy.deepcopy(runs(doc)[0]);other['id'] = 'tube-2'
    doc['draft_subassemblies'].append({'id': 'other', 'runs': [other]})
    result = finalize(Assembly.from_doc(doc), check_collisions=False, run_id='tube-1')
    assert result['status'] == 'conflict'
    assert any(c['code'] == 'AMBIGUOUS_THROUGH' and set(c['runs']) == {'tube-1', 'tube-2'}
               for c in result['conflicts'])


def test_repair_records_every_unattached_through_socket_along_a_draft_pipe():
    doc = design()
    for name, x, y, angle in [('first', 250, 0, 0), ('second', 500, 4, 3), ('third', 750, -3, -2)]:
        doc['parts'].append({'id': name, 'body': {'kind': 'connector', 'mass_kg': .2,
            'geometry': [{'type': 'sphere', 'radius_mm': 6, 'position_mm': [0, 80, 0]}],
            'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                               'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                               'axis': [1, 0, 0], 'engagement_mm': 34}}},
            'pose': {'position_mm': [x, y, 100], 'rotation_deg': [0, 0, angle]}})
    for fitting, end in [('left', 'start'), ('right', 'end')]:
        doc = connect(Assembly.from_doc(doc), 'tube-1', fitting, 'socket', end, 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'first', 'bore')
    assert len(runs(doc)[0]['attachments']) == 3
    result = repair(Assembly.from_doc(doc), run_id='tube-1')
    assert result['status'] == 'repaired', result
    assert result['inferred_through_connections'] == 2
    restored = Assembly.from_doc(result['document'])
    assert {a['connector'] for a in runs(restored.doc)[0]['attachments']} == {
        'left', 'right', 'first', 'second', 'third'}
    assert preview(restored)[0]['conflicts'] == []
    exact = finalize(restored, check_collisions=False)
    assert exact['status'] == 'finalized', exact
    assert len(exact['document']['joints']) == 5


def test_repair_does_not_claim_a_through_socket_shared_by_two_near_pipes():
    doc = design()
    doc['parts'].append({'id': 'through', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 6, 'position_mm': [0, 80, 0]}],
        'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                           'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                           'axis': [1, 0, 0], 'engagement_mm': 34}}},
        'pose': {'position_mm': [500, 4, 100]}})
    other = copy.deepcopy(runs(doc)[0]); other['id'] = 'tube-2'
    doc['draft_subassemblies'].append({'id': 'other', 'runs': [other]})
    result = repair(Assembly.from_doc(doc), run_id='tube-1')
    assert result['status'] == 'conflict', result
    assert any(c['code'] == 'AMBIGUOUS_THROUGH' and set(c['runs']) == {'tube-1', 'tube-2'}
               for c in result['conflicts'])


def test_incompatible_closed_path_stays_draft_with_residual():
    doc = design(second_axis=(0, -1, 0), second_y=20)
    doc['anchors'] = [{'part': 'left', 'surface': 'fixture'}, {'part': 'right', 'surface': 'fixture'}]
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'right', 'socket', 'end', 20)
    result = finalize(Assembly.from_doc(doc), check_collisions=False)
    assert result['status'] == 'conflict'
    assert any(c['code'] == 'AXIS_MISMATCH' for c in result['conflicts'])
    assert doc['draft_subassemblies'][0]['runs'][0]['id'] == 'tube-1'


def test_finalize_relaxes_free_connector_and_holds_anchor():
    doc = design(second_y=40)
    doc['anchors'] = [{'part': 'left', 'surface': 'fixture'}]
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'right', 'socket', 'end', 20)
    result = finalize(Assembly.from_doc(doc), check_collisions=False)
    assert result['status'] == 'finalized', result
    parts = {p['id']: p for p in result['document']['parts']}
    assert parts['left']['pose']['position_mm'] == [0, 0, 100]
    assert abs(parts['right']['pose']['position_mm'][1]) < .01
    assert result['moved_parts_mm']['right'] > 39


def test_repair_closes_a_small_socket_mismatch_without_finalizing():
    doc = design(second_y=7)
    doc['anchors'] = [{'part': 'left', 'surface': 'fixture'}]
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'right', 'socket', 'end', 20)
    assert any(c['code'] == 'POSITION_MISMATCH' for c in preview(Assembly.from_doc(doc))[0]['conflicts'])
    result = repair(Assembly.from_doc(doc), 'frame')
    assert result['status'] == 'repaired', result
    assert result['document']['draft_subassemblies'] == doc['draft_subassemblies']
    assert result['document']['joints'] == []
    assert len(result['document']['parts']) == 2
    assert abs(next(p for p in result['document']['parts'] if p['id'] == 'right')['pose']['position_mm'][1]) < .01
    assert preview(Assembly.from_doc(result['document']))[0]['conflicts'] == []
    assert finalize(Assembly.from_doc(result['document']), check_collisions=False)['status'] == 'finalized'


def test_repair_keeps_an_unmovable_draft_unchanged():
    doc = design(second_y=7)
    doc['anchors'] = [{'part': 'left', 'surface': 'fixture'}, {'part': 'right', 'surface': 'fixture'}]
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'right', 'socket', 'end', 20)
    before = copy.deepcopy(doc)
    result = repair(Assembly.from_doc(doc), 'frame')
    assert result['status'] == 'conflict'
    assert any(c['code'] == 'POSITION_MISMATCH' for c in result['conflicts'])
    assert doc == before


def test_finalize_rotates_free_connector_to_close_axis():
    doc = design(second_axis=(0, -1, 0))
    doc['anchors'] = [{'part': 'left', 'surface': 'fixture'}]
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'right', 'socket', 'end', 20)
    result = finalize(Assembly.from_doc(doc), check_collisions=False)
    assert result['status'] == 'finalized', result
    assert result['rotated_parts_deg']['right'] > 80


def test_anchored_impossible_closure_reports_conflict_without_moving_fittings():
    doc = design(second_y=40)
    doc['anchors'] = [{'part': 'left', 'surface': 'fixture'}, {'part': 'right', 'surface': 'fixture'}]
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'right', 'socket', 'end', 20)
    before = copy.deepcopy(doc)
    result = finalize(Assembly.from_doc(doc), check_collisions=False)
    assert result['status'] == 'conflict'
    assert any(c['code'] == 'AXIS_MISMATCH' for c in result['conflicts'])
    assert doc == before


def test_through_fitting_is_station_on_one_continuous_run():
    doc = design()
    doc['parts'].append({'id': 'through', 'body': {'kind': 'connector', 'mass_kg': .2,
                         'geometry': [{'type': 'sphere', 'radius_mm': 6, 'position_mm': [0, 80, 0]}],
                         'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                                            'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                                            'axis': [1, 0, 0], 'engagement_mm': 34}}},
                         'pose': {'position_mm': [500, 0, 100]}})
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'through', 'bore')
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'right', 'socket', 'end', 20)
    result = finalize(Assembly.from_doc(doc), check_collisions=False)
    assert result['status'] == 'finalized', result
    assert len(result['document']['parts']) == 4
    joint = next(j for j in result['document']['joints'] if j['a']['part'] == 'through')
    assert joint['b'] == {'part': 'tube-1', 'at_mm': 520}


def test_draft_through_fit_follows_fine_tuned_fitting_and_repair_keeps_its_station():
    doc = design()
    doc['anchors'] = [{'part': 'left', 'surface': 'fixture'}, {'part': 'right', 'surface': 'fixture'}]
    doc['parts'].append({'id': 'through', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 6, 'position_mm': [0, 80, 0]}],
        'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                           'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                           'axis': [1, 0, 0], 'engagement_mm': 34}}},
        'pose': {'position_mm': [650, 0, 100]}})
    for fitting, end in [('left', 'start'), ('right', 'end')]:
        doc = connect(Assembly.from_doc(doc), 'tube-1', fitting, 'socket', end, 20)
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'through', 'bore', at_mm=520)
    attachment = runs(doc)[0]['attachments'][-1]
    assert 'at_mm' not in attachment
    attachment['at_mm'] = 520  # A saved draft from before stations followed the fitting pose.
    assert preview(Assembly.from_doc(doc))[0]['conflicts'] == []
    assert repair(Assembly.from_doc(doc))['status'] == 'aligned'

    doc['parts'][-1]['pose']['position_mm'][1] = 7
    assert [c['code'] for c in preview(Assembly.from_doc(doc))[0]['conflicts']] == ['THROUGH_FIT']
    repaired = repair(Assembly.from_doc(doc))
    assert repaired['status'] == 'repaired', repaired
    pose = next(p['pose'] for p in repaired['document']['parts'] if p['id'] == 'through')
    assert abs(pose['position_mm'][0]-650) < .01
    assert abs(pose['position_mm'][1]) < .01
    assert preview(Assembly.from_doc(repaired['document']))[0]['conflicts'] == []
    exact = finalize(Assembly.from_doc(repaired['document']), check_collisions=False)
    assert exact['status'] == 'finalized', exact
    joint = next(j for j in exact['document']['joints'] if j['a']['part'] == 'through')
    assert abs(joint['b']['at_mm']-670) < .01


@pytest.mark.parametrize('station',[16.9, 983.1])
def test_repair_clears_tiny_through_engagement_shortfall(station):
    doc = design()
    doc['parts'] = [{'id': 'through', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 6}],
        'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                           'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                           'axis': [1, 0, 0], 'engagement_mm': 34}}},
        'pose': {'position_mm': [station, .001, 100]}}]
    doc['draft_subassemblies'][0]['runs'][0]['attachments'] = [{'connector': 'through', 'port': 'bore'}]
    assembly = Assembly.from_doc(doc)
    initial = preview(assembly)[0]['conflicts']
    assert initial[0]['code'] == 'THROUGH_FIT'
    assert 'engagement' in initial[0]['message']
    repaired = repair(assembly)
    assert repaired['status'] == 'repaired', repaired
    after = Assembly.from_doc(repaired['document'])
    assert preview(after)[0]['conflicts'] == []
    assert finalize(after, check_collisions=False)['status'] == 'finalized'

    doc['draft_subassemblies'][0]['runs'][0]['attachments'] = []
    inferred = finalize(Assembly.from_doc(doc), check_collisions=False)
    assert inferred['status'] == 'finalized', inferred
    assert inferred['inferred_through_connections'] == 1


def test_repair_extends_an_unlocked_through_pipe_when_its_fitting_cannot_move():
    doc = design()
    doc['parts'] = [{'id': 'through', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 6}],
        'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                           'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                           'axis': [1, 0, 0], 'engagement_mm': 34}}},
        'pose': {'position_mm': [16.9, 0, 100]}}]
    doc['anchors'] = [{'part': 'through', 'surface': 'fixture'}]
    run = runs(doc)[0]
    run['attachments'] = [{'connector': 'through', 'port': 'bore'}]
    repaired = repair(Assembly.from_doc(doc))
    assert repaired['status'] == 'repaired', repaired
    assert repaired['moved_parts_mm'] == {}
    assert repaired['resized_runs_mm']['tube-1'] == pytest.approx(.2)
    assert runs(repaired['document'])[0]['start_mm'][0] == pytest.approx(-.2)
    assert preview(Assembly.from_doc(repaired['document']))[0]['conflicts'] == []

    run['locked_length_mm'] = 1000
    locked = repair(Assembly.from_doc(doc))
    assert locked['status'] == 'conflict'
    assert runs(doc)[0]['start_mm'][0] == 0


def test_repair_resizes_centered_draft_when_shared_through_fitting_moves():
    doc = design()
    doc['parts'] = [{'id': 'junction', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 6}],
        'ports': {
            'through': {'type': 'socket', 'through': True, 'profile': 'round',
                        'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                        'axis': [0, 1, 0], 'engagement_mm': 34},
            'end': {'type': 'socket', 'profile': 'round', 'diameter_mm': 42.4,
                    'position_mm': [0, 0, 0], 'axis': [-1, 0, 0],
                    'engagement_mm': 40, 'min_engagement_mm': 15}}},
        'pose': {'position_mm': [280, 500, 100]}}]
    group = doc['draft_subassemblies'][0]
    group['runs'] = [
        {'id': 'cross', 'catalog': 'tubeclamp.tube-C', 'start_mm': [-300, 500, 100],
         'end_mm': [300, 500, 100], 'attachments': [
             {'connector': 'junction', 'port': 'end', 'end': 'end', 'insertion_mm': 20}]},
        {'id': 'rail', 'catalog': 'tubeclamp.tube-C', 'start_mm': [278.9, 0, 100],
         'end_mm': [278.9, 1000, 100], 'attachments': [
             {'connector': 'junction', 'port': 'through'}]}]
    group['mirrors'] = [{'id': 'midline', 'axis': 'x', 'offset_mm': 0,
                         'run_modes': {'cross': 'centered'}}]
    assembly = Assembly.from_doc(doc)
    assert preview(assembly)[1]['conflicts'][0]['code'] == 'THROUGH_FIT'
    result = repair(assembly, run_id='rail')
    assert result['status'] == 'repaired', result
    assert result['resized_runs_mm']['cross'] == pytest.approx(-2.2, abs=.01)
    run = next(run for run in runs(result['document']) if run['id'] == 'cross')
    assert run['start_mm'][0] == pytest.approx(-run['end_mm'][0], abs=.01)
    assert np.linalg.norm(np.array(run['end_mm'])-run['start_mm']) < 600
    assert all(not part['conflicts'] for part in preview(Assembly.from_doc(result['document'])))


def test_repair_keeps_mirror_axis_while_closing_a_connected_end_mismatch():
    path = Path(__file__).parent/'fixtures'/'draft-mirror-repair.pipe.yaml'
    doc = yaml.safe_load(path.read_text(encoding='utf-8'))
    original = copy.deepcopy(doc)
    assembly = Assembly.from_doc(doc)
    conflict = next(part for part in preview(assembly) if part['id'] == 'tube-c-2-copy-15')
    assert conflict['conflicts'][0]['code'] == 'POSITION_MISMATCH'
    result = repair(assembly, run_id='tube-c-2-copy-15')
    assert result['status'] == 'repaired', result
    assert doc == original
    repaired = Assembly.from_doc(result['document'])
    assert all(not part['conflicts'] for part in preview(repaired))
    centered = next(run for run in runs(repaired.doc) if run['id'] == 'tube-c-2-copy-14')
    layout = next(part for part in preview(repaired) if part['id'] == centered['id'])
    assert abs(layout['pose']['position_mm'][0]) < .05
    assert 'tube-c-2-copy-14' in result['resized_runs_mm']


def test_repair_rotates_crossed_fittings_and_through_only_bridge_together():
    path = Path(__file__).parent/'fixtures'/'draft-cross-repair.pipe.yaml'
    doc = yaml.safe_load(path.read_text(encoding='utf-8'))
    original = copy.deepcopy(doc)
    assembly = Assembly.from_doc(doc)
    before = next(part for part in preview(assembly) if part['id'] == 'bridge')
    assert before['conflicts'][0]['code'] == 'THROUGH_FIT'
    assert before['conflicts'][0]['residual_mm'] > 18
    direct = finalize(assembly, run_id='bridge', check_collisions=False)
    assert direct['status'] == 'finalized', direct
    result = repair(assembly, run_id='bridge')
    assert result['status'] == 'repaired', result
    assert doc == original
    repaired = Assembly.from_doc(result['document'])
    assert all(not part['conflicts'] for part in preview(repaired))
    assert 'bridge' in result['moved_runs']
    for part in ('cross0', 'cross1'):
        assert 4 < result['rotated_parts_deg'][part] < 6
    for part in ('anchor0', 'anchor1'):
        assert np.allclose(repaired.parts[part].matrix, assembly.parts[part].matrix)
    exact = finalize(repaired, run_id='bridge', check_collisions=False)
    assert exact['status'] == 'finalized', exact


def test_repair_moves_a_through_only_draft_pipe_between_fixed_fittings():
    doc = design()
    doc['parts'] = [{'id': pid, 'body': {'kind': 'connector', 'mass_kg': .2,
                     'geometry': [{'type': 'sphere', 'radius_mm': 6}],
                     'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                                        'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                                        'axis': [1, 0, 0], 'engagement_mm': 34}}},
                     'pose': {'position_mm': [x, 0, 100]}}
                    for pid, x in [('first', 200), ('second', 800)]]
    doc['anchors'] = [{'part': pid, 'surface': 'fixture'} for pid in ('first', 'second')]
    run = runs(doc)[0]
    run['start_mm'] = [0, 5, 100]
    run['end_mm'] = [1000, 5, 100]
    run['attachments'] = [{'connector': pid, 'port': 'bore'} for pid in ('first', 'second')]
    result = repair(Assembly.from_doc(doc))
    assert result['status'] == 'repaired', result
    assert result['moved_parts_mm'] == {}
    assert result['moved_runs'] == ['tube-1']
    assert preview(Assembly.from_doc(result['document']))[0]['conflicts'] == []


def test_partial_finalization_removes_mirror_modes_for_finished_runs():
    doc = design()
    group = doc['draft_subassemblies'][0]
    second = copy.deepcopy(group['runs'][0])
    second['id'] = 'tube-2'
    second['start_mm'][1] = second['end_mm'][1] = 500
    group['runs'].append(second)
    group['mirrors'] = [{'id': 'scene-x', 'axis': 'x', 'offset_mm': 0,
                         'run_modes': {'tube-1': 'free', 'tube-2': 'free'}}]
    result = finalize(Assembly.from_doc(doc), run_id='tube-1', check_collisions=False)
    assert result['status'] == 'finalized', result
    remaining = result['document']['draft_subassemblies'][0]
    assert remaining['mirrors'][0]['run_modes'] == {'tube-2': 'free'}
    assert [run['id'] for run in remaining['runs']] == ['tube-2']
    Assembly.from_doc(result['document'])


def test_draft_through_axis_allowance_matches_exact_validation():
    doc = design()
    doc['parts'] = [{'id': 'through', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 6}],
        'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                           'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                           'axis': [1, 0, 0], 'engagement_mm': 34}}},
        'pose': {'position_mm': [500, .001, 100], 'rotation_deg': [0, 0, 2.3]}}]
    doc['draft_subassemblies'][0]['runs'][0]['attachments'] = [{'connector': 'through', 'port': 'bore'}]
    assembly = Assembly.from_doc(doc)
    assert preview(assembly)[0]['conflicts'] == []
    assert finalize(assembly, check_collisions=False)['status'] == 'finalized'


def test_draft_ids_and_socket_occupancy_are_checked_on_load():
    doc = design()
    doc['draft_subassemblies'][0]['runs'].append(copy.deepcopy(doc['draft_subassemblies'][0]['runs'][0]))
    with pytest.raises(DocumentError, match='unique'):
        Assembly.from_doc(doc)
    doc['draft_subassemblies'][0]['runs'][1]['id'] = 'tube-2'
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    with pytest.raises(DocumentError, match='occupied'):
        connect(Assembly.from_doc(doc), 'tube-2', 'left', 'socket', 'start', 20)


def test_unfinished_draft_is_not_silently_omitted_from_analysis():
    assembly = Assembly.from_doc(design())
    from pipesim.validation import validate
    from pipesim.exporting import bom, cutting_plan
    from pipesim.fea import analyse
    from pipesim.planning import plan_build
    from pipesim.physics import World
    for operation in (validate, bom, cutting_plan, analyse, plan_build, World):
        with pytest.raises(DocumentError, match='Finalize draft subassemblies'):
            operation(assembly)


def test_draft_graph_survives_save_and_load(tmp_path):
    doc = design()
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    path = tmp_path / 'draft.pipe.yaml'
    write(path, doc)
    loaded = Assembly.load(path)
    assert loaded.doc['draft_subassemblies'] == doc['draft_subassemblies']
    assert len(loaded.parts) == 2
    assert preview(loaded)[0]['attachments'][0]['connector'] == 'left'


def test_locked_length_with_one_free_end_sets_the_cut_length():
    doc = design()
    doc = connect(Assembly.from_doc(doc), 'tube-1', 'left', 'socket', 'start', 20)
    doc['draft_subassemblies'][0]['runs'][0]['locked_length_mm'] = 750
    assert preview(Assembly.from_doc(doc))[0]['length_mm'] == 750
    result = finalize(Assembly.from_doc(doc), check_collisions=False)
    assert result['status'] == 'finalized', result
    assert result['lengths_mm']['tube-1'] == 750


def test_old_selected_finalize_request_is_rejected(editor):
    from test_server import request

    with pytest.raises(urllib.error.HTTPError) as error:
        request(editor, '/api/draft-finalize', {'document': design(), 'subassembly': 'frame'})
    assert error.value.code == 400
    assert 'out of date' in json.loads(error.value.read())['error']


@pytest.mark.parametrize('operation', ['draft-finalize', 'draft-repair'])
def test_editor_cancels_draft_operation_before_committing(editor, monkeypatch, operation):
    from pipesim.server import Handler
    from test_server import request

    entered, release = threading.Event(), threading.Event()
    original = Handler.assembly
    def paused_assembly(handler, document, base, **kwargs):
        entered.set()
        assert release.wait(10)
        return original(handler, document, base, **kwargs)
    monkeypatch.setattr(Handler, 'assembly', paused_assembly)
    result = []
    def run():
        try:
            result.append(request(editor, '/api/'+operation,
                                  {'document': design(), 'job_id': 'draftcancel01'}))
        except urllib.error.HTTPError as exc:
            result.append((exc.code, json.loads(exc.read())))
    thread = threading.Thread(target=run)
    thread.start()
    try:
        assert entered.wait(5)
        status, raw = request(editor, '/api/'+operation+'-cancel', {'job_id': 'draftcancel01'})
        assert status == 200 and json.loads(raw)['cancelled']
    finally:
        release.set()
        thread.join(10)
    assert not thread.is_alive()
    assert result[0][0] == 400
    assert 'cancelled' in result[0][1]['error']
