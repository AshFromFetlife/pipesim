"""An anchored through support must not prevent editing either occupied end."""
import copy
from pathlib import Path

import numpy as np
import pytest

from pipesim.document import read
from pipesim.drafting import finalize, preview, reopen, runs
from pipesim.resize_drag import _frame, resize_drag
from pipesim.validation import validate


FIXTURE = Path(__file__).parent / 'fixtures/resize-anchored-crossbar.pipe.yaml'


@pytest.mark.parametrize('mode', ['draft', 'exact', 'mixed'])
@pytest.mark.parametrize('side', ['start', 'end'])
@pytest.mark.parametrize('change', [-10, 10])
@pytest.mark.parametrize('payload', [False, True])
def test_occupied_pipe_end_resizes_beside_world_fixed_crossbar(factory, mode, side, change, payload):
    doc = read(FIXTURE)
    if payload:
        fitting = factory(doc).parts['tc104c-1']
        mouth, axis = fitting.frame({'port': 'through'})
        doc['draft_subassemblies'][0]['runs'].append({
            'id': 'end-crossbar', 'catalog': 'tubeclamp.tube-C',
            'start_mm': (mouth-axis*250).tolist(), 'end_mm': (mouth+axis*250).tolist(),
            'attachments': [{'connector': fitting.id, 'port': 'through'}]})
        doc['parts'].append({'id': 'payload', 'catalog': 'generic.box',
                             'parameters': {'mass_kg': 37},
                             'pose': {'position_mm': fitting.matrix[:3, 3].tolist()}})
        doc['joints'].append({'id': 'payload-mount', 'type': 'fixed',
                              'a': {'part': fitting.id}, 'b': {'part': 'payload'}})
    if mode != 'draft':
        result = finalize(factory(doc), check_collisions=False)
        assert result['status'] == 'finalized', result
        doc = result['document']
        if mode == 'mixed':
            doc = reopen(factory(doc), ['tube-c-4'])['document']
    before = factory(doc)
    saved = copy.deepcopy(before.doc)
    first, last, _ = _frame(before, 'tube-c-4')
    length = np.linalg.norm(last-first)
    outward = (first-last if side == 'start' else last-first)/length
    moving = 'tc132c-1' if side == 'start' else 'tc104c-1'
    result = resize_drag(before, 'tube-c-4', float(length+change), side, auto_connect=False)
    assert result['status'] == 'resized'
    after = factory(result['document'])
    new_first, new_last, _ = _frame(after, 'tube-c-4')
    assert np.linalg.norm(new_last-new_first) == pytest.approx(length+change, abs=.001)
    for old, new, selected in [(first, new_first, side == 'start'), (last, new_last, side == 'end')]:
        assert np.allclose(new, old + (outward*change if selected else 0), atol=.001)
    for pid, part in before.parts.items():
        if pid == 'tube-c-4':
            continue
        expected = part.matrix.copy()
        if pid == moving or payload and side == 'end' and pid in ('payload', 'end-crossbar'):
            expected[:3, 3] += outward*change
        assert np.allclose(after.parts[pid].matrix, expected, atol=.001), pid
        if part.kind == 'member':
            assert after.parts[pid].length == part.length
    if payload:
        assert after.parts['payload'].mass == 37
    assert after.doc['anchors'] == before.doc['anchors']
    for run in runs(doc):
        changed = next(r for r in runs(after.doc) if r['id'] == run['id'])
        assert changed['attachments'] == run['attachments']
        if run['id'] == 'end-crossbar' and side == 'end':
            for field in ('start_mm', 'end_mm'):
                assert np.allclose(changed[field], np.array(run[field])+outward*change)
        elif run['id'] != 'tube-c-4':
            assert changed == run
    assert [j['id'] for j in after.joints] == [j['id'] for j in before.joints]
    assert all(not item['conflicts'] for item in preview(after))
    exact = finalize(after, check_collisions=False) if runs(after.doc) else {'document': after.doc}
    report = validate(factory(exact['document']), collisions=False)
    assert report['valid'], report['issues']
    assert before.doc == saved
