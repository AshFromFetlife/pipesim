"""Valid geometry alone does not prove a resize honored the requested edit."""
import copy

import numpy as np
import pytest

from pipesim.drafting import finalize, reopen, runs
from pipesim.resize_drag import resize_drag
from pipesim.validation import validate


@pytest.mark.parametrize('side', ['start', 'end'])
@pytest.mark.parametrize('reverse', [False, True])
@pytest.mark.parametrize('finished_neighbors', [False, True])
def test_shortening_one_free_grid_end_preserves_every_other_span(factory, blank, side, reverse, finished_neighbors):
    doc = copy.deepcopy(blank)
    doc['parts'] = [{'id': f'cross-{x}-{z}', 'catalog': 'tubeclamp.TC161C',
                     'pose': {'position_mm': [x*500, 0, 1000+z*500]}}
                    for x in range(2) for z in range(2)]
    definitions = []
    for x in range(2):
        endpoints = [[x*500, 0, 800], [x*500, 0, 1700]]
        if reverse:
            endpoints.reverse()
        definitions.append({'id': f'upright-{x}', 'catalog': 'tubeclamp.tube-C',
            'start_mm': endpoints[0], 'end_mm': endpoints[1],
            'attachments': [{'connector': f'cross-{x}-{z}', 'port': 'through'} for z in range(2)]})
    for z in range(2):
        definitions.append({'id': f'crossbar-{z}', 'catalog': 'tubeclamp.tube-C',
            'start_mm': [-200, 42.4, 1000+z*500], 'end_mm': [700, 42.4, 1000+z*500],
            'attachments': [{'connector': f'cross-{x}-{z}', 'port': 'cross'} for x in range(2)]})
    doc['draft_subassemblies'] = [{'id': 'frame', 'runs': definitions}]
    if finished_neighbors:
        finished = finalize(factory(doc))
        assert finished['status'] == 'finalized'
        doc = reopen(factory(finished['document']), ['upright-0'])['document']
    result = resize_drag(factory(doc), 'upright-0', 890, side, auto_connect=False)
    assert result['status'] == 'resized'
    after = result['document']
    assert after['parts'] == doc['parts']
    for original, changed in zip(runs(doc), runs(after)):
        if original['id'] != 'upright-0':
            assert changed == original, 'a different pipe was silently resized or moved'
        else:
            fixed = 'end_mm' if side == 'start' else 'start_mm'
            assert changed[fixed] == original[fixed]
            assert np.linalg.norm(np.subtract(changed['end_mm'], changed['start_mm'])) == pytest.approx(890)
            assert changed['attachments'] == original['attachments']
    exact = finalize(factory(after))
    assert exact['status'] == 'finalized'
    assert validate(factory(exact['document']))['valid']
