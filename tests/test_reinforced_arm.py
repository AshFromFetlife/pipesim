import copy
import json
from pathlib import Path

import numpy as np
import pytest

from pipesim.document import Assembly
from pipesim.snapping import connection_options

FIXTURE=Path(__file__).parent/'fixtures'/'four-way-swing'


def test_duplicate_reinforcement_closes_without_searching_payload_or_merging_arms(library):
    before=Assembly.from_doc(json.loads((FIXTURE/'reinforced-arm.json').read_text(encoding='utf-8')),FIXTURE,library)
    original=copy.deepcopy(before.doc)
    result=connection_options(before,'tube-c-4-copy','tc104c-1-copy','branch',end='end',first_valid=True)
    chosen=next((o for o in result['options'] if o['available']),None)
    assert chosen is not None,[(o['move'],o.get('reason')) for o in result['options']]
    after=Assembly.from_doc(chosen['document'],FIXTURE,library)
    assert np.linalg.norm(after.parts['tube-c-4'].matrix[:3,3]-after.parts['tube-c-4-copy'].matrix[:3,3])>200
    assert len(after.joints)==len(before.joints)+1
    a,b,_=after.joint_frames(chosen.get('joint',result['joint']))
    assert np.linalg.norm(a-b)<result['joint']['fit_tolerance_mm']
    assert not chosen.get('requires_preview')
    for anchor in before.anchors:
        np.testing.assert_allclose(after.parts[anchor['part']].matrix,before.parts[anchor['part']].matrix,atol=1e-6,rtol=0)
    assert before.doc==original
    for joint in before.joints:
        updated=next(j for j in after.joints if j['id']==joint['id'])
        assert updated.get('locked')==joint.get('locked')
        for end in ('a','b'):
            assert updated[end]['part']==joint[end]['part']
        if joint.get('locked') or joint['type']=='fixed' or (joint['a']['part'].startswith(('human-','chain-')) and joint['b']['part'].startswith(('human-','chain-'))):
            x,y=(joint[end]['part'] for end in ('a','b'))
            np.testing.assert_allclose(np.linalg.inv(after.parts[x].matrix)@after.parts[y].matrix,
                                       np.linalg.inv(before.parts[x].matrix)@before.parts[y].matrix,atol=1e-5,rtol=0)
    for pid,part in before.parts.items():
        assert after.parts[pid].length==part.length


def test_reinforcement_only_searches_the_structural_path(library,monkeypatch):
    import pipesim.fitting as fitting
    before=Assembly.from_doc(json.loads((FIXTURE/'reinforced-arm.json').read_text(encoding='utf-8')),FIXTURE,library)
    original=fitting.least_squares
    sizes=[]
    def solve(function,values,**kwargs):
        sizes.append(len(values))
        return original(function,values,**kwargs)
    monkeypatch.setattr(fitting,'least_squares',solve)
    # Crossing a clock deadline must not abort an interactive job: the caller
    # owns cancellation, independent of machine speed or design size.
    clock=iter(range(0,1000000000,1000))
    monkeypatch.setattr(fitting.time,'monotonic',lambda:next(clock))
    result=connection_options(before,'tube-c-4-copy','tc104c-1-copy','branch',end='end',first_valid=True,cancelled=lambda:False)
    assert result['recommended']
    assert sizes and max(sizes)==6


def test_reinforcement_cancellation_interrupts_numerical_search(library,monkeypatch):
    import pipesim.fitting as fitting
    from pipesim.connection_jobs import ConnectionCancelled
    before=Assembly.from_doc(json.loads((FIXTURE/'reinforced-arm.json').read_text(encoding='utf-8')),FIXTURE,library)
    cancelled=False
    def solve(function,values,**kwargs):
        nonlocal cancelled
        cancelled=True
        function(values)
        pytest.fail('Cancelled residual continued')
    monkeypatch.setattr(fitting,'least_squares',solve)
    with pytest.raises(ConnectionCancelled):
        connection_options(before,'tube-c-4-copy','tc104c-1-copy','branch',end='end',cancelled=lambda:cancelled)
