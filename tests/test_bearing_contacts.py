import copy
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.document import Assembly
from pipesim.math3d import pose_of
from pipesim.physics import simulate


@pytest.mark.parametrize('starting_angle',[-20,0,20])
def test_carriage_keeps_swinging_instead_of_sticking_in_its_bearings(starting_angle):
    assembly=Assembly.load(Path(__file__).parent/'fixtures/swing-carriage.pipe.yaml')
    joint=next(j for j in assembly.joints if j['id']=='tc101c-1-through-tube-c-3')
    pivot,_,axis=assembly.joint_frames(joint)
    rotation=Rotation.from_rotvec(np.deg2rad(starting_angle)*axis).as_matrix()
    members=next(g for g in assembly.rigid_groups() if joint['a']['part'] in g)
    doc=copy.deepcopy(assembly.doc)
    for part in doc['parts']:
        if part['id'] not in members: continue
        matrix=assembly.parts[part['id']].matrix.copy()
        matrix[:3,3]=pivot+rotation@(matrix[:3,3]-pivot)
        matrix[:3,:3]=rotation@matrix[:3,:3]
        part['pose']=pose_of(matrix)
    result=simulate(Assembly.from_doc(doc,assembly.base),3,30)
    angles=np.array([f['joints'][joint['id']]['angle_deg'] for f in result['frames']])
    # The old contact persistence trapped all starting poses at the same raised
    # position after about ten frames, with near-zero subsequent motion.
    assert np.ptp(angles[30:])>20
    assert not result['settled']
    assert result['final_max_speed_m_s']>.1
    assert not any(e['type'] in ('joint_break','socket_disengaged') for e in result['events'])
