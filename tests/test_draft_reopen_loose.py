import copy

import numpy as np
import pytest

from pipesim.drafting import finalize, reopen, runs
from pipesim.duplication import duplicate_draft
from pipesim.math3d import pose_of
from reopen_cases import check_loose_reopen, loose_socket_case


@pytest.mark.parametrize('terminal',[False,True])
@pytest.mark.parametrize('seed',[0,1])
def test_return_body_to_draft_preserves_a_loose_socket_in_another_body(factory,terminal,seed):
    check_loose_reopen(factory,seed,terminal=terminal)


def test_reopened_loose_socket_uses_the_edited_station_and_keeps_its_settings(factory):
    before=factory(loose_socket_case())
    draft=reopen(before,['tube-c-5'])['document']
    source=copy.deepcopy(runs(draft)[0]['attachments'][0]['joint'])
    axis=before.parts['tube-c-5'].matrix[:3,2]
    for part in draft['parts']:
        matrix=before.parts[part['id']].matrix.copy()
        matrix[:3,3]+=axis*25
        part['pose']=pose_of(matrix)
    result=finalize(factory(draft))
    assert result['status']=='finalized',result
    joint=next(j for j in result['document']['joints'] if j['id']==source['id'])
    assert joint['b']['at_mm']==pytest.approx(source['b']['at_mm']+25,abs=1e-5)
    assert joint['locked'] is False
    assert joint['metadata']==source['metadata'] and joint['limits']==source['limits']


def test_copying_reopened_draft_keeps_loose_state_and_remaps_joint_endpoints(factory):
    before=factory(loose_socket_case())
    draft=reopen(before,['tube-c-5'])['document']
    copied=duplicate_draft(factory(draft),'tube-c-5',offset_mm=[5000,0,0])
    result=finalize(factory(copied['document']))
    assert result['status']=='finalized',result
    after=factory(result['document'])
    joints=[j for j in after.joints if not j.get('locked')]
    assert len(joints)==2 and len({j['id'] for j in after.joints})==len(after.joints)
    endpoints={(j['a']['part'],j['b']['part']) for j in joints}
    assert endpoints=={('tc161c-1','tube-c-5'),
        (copied['copies'][0]['parts']['tc161c-1'],copied['selected'])}
    for joint in joints: assert joint['metadata']==before.joints[0]['metadata']
