"""Mirror closure is part of draft repair, not a hidden mirror-bake exception."""

import copy
import json

import numpy as np
import pytest

from pipesim.document import Assembly
from pipesim.drafting import _layout, preview, repair
from pipesim.symmetry import materialize_mirror
from test_server import editor, request


def mirrored_free_run(mode='centered'):
    start, end = ([-90, 0, 100], [110, 0, 100]) if mode == 'centered' else (
        [-90, 10, 100], [110, 10, 100])
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
            'name': 'Recoverable mirrored draft', 'parts': [], 'joints': [],
            'draft_subassemblies': [{'id': 'draft', 'runs': [{
                'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
                'start_mm': start, 'end_mm': end}],
                'mirrors': [{'id': 'mirror', 'axis': 'x' if mode == 'centered' else 'y',
                             'offset_mm': 0, 'run_modes': {'pipe': mode}}]}]}


@pytest.mark.parametrize('mode', ['centered', 'in_plane'])
def test_repair_realigns_free_mirrored_run_without_changing_its_length(mode):
    doc = mirrored_free_run(mode)
    original = copy.deepcopy(doc)
    assembly = Assembly.from_doc(doc, validate_mirror_geometry=False)
    assert preview(assembly)[0]['conflicts'][0]['code'] == 'MIRROR_ALIGNMENT'

    result = repair(assembly)
    assert result['status'] == 'repaired', result
    assert result['moved_runs'] == ['pipe']
    assert doc == original
    corrected = Assembly.from_doc(result['document'])
    assert preview(corrected)[0]['conflicts'] == []
    before, after = original['draft_subassemblies'][0]['runs'][0], result['document']['draft_subassemblies'][0]['runs'][0]
    assert np.linalg.norm(np.subtract(after['end_mm'], after['start_mm'])) == pytest.approx(
        np.linalg.norm(np.subtract(before['end_mm'], before['start_mm'])))


def test_finalize_api_repairs_free_centered_run_before_mirror_bake(editor):
    doc = mirrored_free_run()
    status, raw = request(editor, '/api/draft-finalize', {'document': doc})
    result = json.loads(raw)
    assert status == 200
    assert result['status'] == 'finalized', result
    assert not result['document'].get('draft_subassemblies')
    assert [part['id'] for part in result['document']['parts']] == ['pipe']
    assert result['document']['parts'][0]['pose']['position_mm'][0] == pytest.approx(0)


def test_selected_finalize_ignores_unrelated_misaligned_mirror_group(editor):
    doc = mirrored_free_run()
    good = doc['draft_subassemblies'][0]
    good['id'] = 'good-draft'
    good['runs'][0]['id'] = 'good-pipe'
    good['runs'][0]['start_mm'][0] = -100
    good['runs'][0]['end_mm'][0] = 100
    good['mirrors'][0]['run_modes'] = {'good-pipe': 'centered'}
    bad = copy.deepcopy(mirrored_free_run()['draft_subassemblies'][0])
    bad['id'] = 'bad-draft'
    bad['runs'][0]['id'] = 'bad-pipe'
    bad['mirrors'][0]['run_modes'] = {'bad-pipe': 'centered'}
    doc['draft_subassemblies'].append(bad)

    status, raw = request(editor, '/api/draft-finalize-selected',
                          {'document': doc, 'run': 'good-pipe'})
    result = json.loads(raw)
    assert status == 200
    assert result['status'] == 'finalized', result
    assert [part['id'] for part in result['document']['parts']] == ['good-pipe']
    assert result['document']['draft_subassemblies'][0]['runs'][0]['id'] == 'bad-pipe'
    assert preview(Assembly.from_doc(result['document'],
                   validate_mirror_geometry=False))[0]['conflicts'][0]['code'] == 'MIRROR_ALIGNMENT'


def test_irreconcilable_mirror_alignment_returns_conflict_not_http_error(editor):
    doc = mirrored_free_run()
    parts = []
    for name, x, axis in [('left', -80, [1, 0, 0]), ('right', 120, [-1, 0, 0])]:
        parts.append({'id': name, 'body': {'kind': 'connector', 'mass_kg': .2,
                      'geometry': [{'type': 'sphere', 'radius_mm': 6,
                                    'position_mm': [0, 80, 0]}],
                      'ports': {'end': {'type': 'socket', 'profile': 'round',
                                        'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                                        'axis': axis, 'engagement_mm': 40,
                                        'min_engagement_mm': 15}}},
                      'pose': {'position_mm': [x, 0, 100]}})
    doc['parts'] = parts
    doc['anchors'] = [{'part': item['id'], 'surface': 'fixture'} for item in parts]
    run = doc['draft_subassemblies'][0]['runs'][0]
    run['attachments'] = [{'connector': 'left', 'port': 'end', 'end': 'start',
                           'insertion_mm': 20},
                          {'connector': 'right', 'port': 'end', 'end': 'end',
                           'insertion_mm': 20}]
    for operation in ('draft-repair', 'draft-finalize'):
        status, raw = request(editor, '/api/' + operation, {'document': doc})
        result = json.loads(raw)
        assert status == 200
        assert result['status'] == 'conflict', result
        assert any(item['code'] == 'MIRROR_ALIGNMENT' for item in result['conflicts'])


def test_centered_scaffold_crossbar_repair_keeps_through_leg_upright():
    """A reversed side socket must turn about its vertical bore, not tip the leg."""
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': 'Scaffold crossbar mirror repair', 'parts': [
               {'id': 'corner', 'catalog': 'tubeclamp.TC116C',
                'pose': {'position_mm': [466, 500, 672]}}], 'joints': [],
           'draft_subassemblies': [{'id': 'scaffold', 'runs': [
               {'id': 'leg', 'catalog': 'tubeclamp.tube-C',
                'start_mm': [468, 500, 0], 'end_mm': [468, 500, 1300],
                'attachments': [{'connector': 'corner', 'port': 'through'}]},
               {'id': 'crossbar', 'catalog': 'tubeclamp.tube-C',
                'start_mm': [-465, 500, 672], 'end_mm': [465, 500, 672],
                'attachments': [{'connector': 'corner', 'port': 'x',
                                 'end': 'end', 'insertion_mm': 30}]}],
               'mirrors': [{'id': 'left-right', 'axis': 'x', 'offset_mm': 0,
                            'scope': 'scene', 'run_modes': {'crossbar': 'centered'}}]}]}
    assembly = Assembly.from_doc(doc, validate_mirror_geometry=False)
    result = repair(assembly, run_id='crossbar')
    assert result['status'] == 'repaired', result
    corrected = Assembly.from_doc(result['document'])
    layouts = {run['id']: _layout(corrected, run)
               for run in result['document']['draft_subassemblies'][0]['runs']}
    leg = layouts['leg']
    assert np.linalg.norm(np.cross(leg['end']-leg['start'], [0, 0, 1])) < 1e-3
    assert abs((layouts['crossbar']['start'][0]+layouts['crossbar']['end'][0])/2) < .05
    assert np.linalg.norm(layouts['crossbar']['end']-layouts['crossbar']['start']) > 800
    baked = materialize_mirror(corrected, 'scaffold', 'left-right')
    crossbar = next(run for run in baked['draft_subassemblies'][0]['runs']
                    if run['id'] == 'crossbar')
    assert {item['end'] for item in crossbar['attachments']} == {'start', 'end'}
    assert len({item['connector'] for item in crossbar['attachments']}) == 2
