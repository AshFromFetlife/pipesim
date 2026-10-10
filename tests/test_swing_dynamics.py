"""Connected rope geometry and beam damage are independent physics oracles."""
import copy
import json
from pathlib import Path

import numpy as np
import pybullet as pb
import pytest

from pipesim.beam_dynamics import prepare_beams
from pipesim.document import Assembly
from pipesim.math3d import point,transform
from pipesim.physics import World, _matrix


FIXTURE=Path(__file__).parent/'fixtures'/'four-way-swing'/'simulation.json'


def swing(library):
    return Assembly.from_doc(json.loads(FIXTURE.read_text()),FIXTURE.parent,library)


def assert_connected_lines(world, tolerance_mm=8):
    for joint in world._chain_joints+world._beam_joints:
        ends=[point(world.part_matrix(joint[end]['part']),
                    world.assembly.parts[joint[end]['part']].local_frame(joint[end])[0])
              for end in ('a','b')]
        gap=float(np.linalg.norm(ends[0]-ends[1]))
        assert gap<tolerance_mm, f"{world.elapsed:.4f}s: unbroken {joint['id']} has {gap:.3f} mm gap"


def assert_local_line_pivots(world):
    for joint in world._chain_joints:
        constraint=pb.getConstraintInfo(world._chain_constraints[joint['id']][0],physicsClientId=world.client)
        for end,local in [('a',constraint[6]),('b',constraint[7])]:
            pid=joint[end]['part'];body,link,_=world.part_map[pid]
            actual=point(_matrix(*world.link_pose(body,link)),np.array(local)*1000)
            expected=point(world.part_matrix(pid),world.assembly.parts[pid].local_frame(joint[end])[0])
            np.testing.assert_allclose(actual,expected,atol=1e-6)


def assert_recorded_line_continuity(assembly,recording):
    maximum=0.
    for frame in recording['frames']:
        matrices={pid:transform(pose) for pid,pose in frame['parts'].items()}
        for joint in assembly.joints:
            if not joint.get('metadata',{}).get('chain_link') or joint['id'] in frame['broken_joints']: continue
            ends=[point(matrices[joint[end]['part']],assembly.parts[joint[end]['part']].local_frame(joint[end])[0]) for end in ('a','b')]
            gap=float(np.linalg.norm(ends[0]-ends[1]));maximum=max(maximum,gap)
            assert gap<8, f"recording {frame['time_s']:.4f}s: unbroken {joint['id']} has {gap:.3f} mm gap"
    return maximum


def test_saved_swing_rope_contacts_do_not_explode(library):
    # Keep the original rigid arm here to isolate rope integration from the
    # separate beam-selection fix. The old solver fails before its first break.
    with World(swing(library)) as world:
        for step in range(84):
            world.step()
            if step%4==3: assert_connected_lines(world)
        assert not world.broken


def test_continuity_oracle_rejects_scatter_without_break_events(factory,blank,monkeypatch):
    blank['objects']=[{'id':'line','template':'chain','parameters':{'length_mm':60}}]
    with World(factory(blank)) as world:
        assert_connected_lines(world)
        original=world.part_matrix
        def scattered(pid):
            matrix=original(pid).copy()
            if pid=='line/link-2': matrix[0,3]+=20
            return matrix
        monkeypatch.setattr(world,'part_matrix',scattered)
        assert not world.events and not world.broken
        with pytest.raises(AssertionError,match='unbroken .* gap'):
            assert_connected_lines(world)


def test_recording_oracle_rejects_finite_scattered_geometry(factory,blank):
    blank['objects']=[{'id':'line','template':'chain','parameters':{'length_mm':60}}]
    assembly=factory(blank)
    with World(assembly) as world:
        recording={'frames':[world.snapshot()]}
    assert_recorded_line_continuity(assembly,recording)
    recording['frames'][0]['parts']['line/link-2']['position_mm'][0]+=20
    assert not recording['frames'][0]['broken_joints']
    with pytest.raises(AssertionError,match='recording .* unbroken .* gap'):
        assert_recorded_line_continuity(assembly,recording)


def test_large_swing_retains_bending_and_detects_arm_yield(library):
    dynamic,model,analysis=prepare_beams(swing(library))
    assert analysis['status']=='beam_model_too_large'
    assert model['tube-c-4']['selection']=='dynamic_fallback'
    with World(dynamic,1/4000) as world:
        closed=[j for j in world._beam_joints if j['id'] not in world.joint_map]
        assert closed, 'the retained design exercises closed-frame bending too'
        for step in range(6400):
            world.step()
            if step%40==39: assert_connected_lines(world,1 if world.elapsed<.25 else 8)
        assert any(e['type']=='beam_yield' and e['part']=='tube-c-4' for e in world.events)
        assert all(j['id'] in world.snapshot()['joints'] for j in closed)


def test_topology_rebuild_keeps_local_rope_pivots(factory,blank):
    doc=copy.deepcopy(blank)
    doc['objects']=[{'id':'line','template':'chain','parameters':{'length_mm':60}}]
    doc['environment']={'ground':False}
    with World(factory(doc)) as world:
        body,link,_=world.part_map['line/link-2']
        assert link==-1
        position,rotation=pb.getBasePositionAndOrientation(body,physicsClientId=world.client)
        pb.resetBasePositionAndOrientation(body,np.array(position)+[.003,0,0],rotation,physicsClientId=world.client)
        world._detach(set())
        assert_local_line_pivots(world)


def test_severed_line_has_no_remaining_support_spring(factory,blank,library):
    from test_rope_drop import hanging_weight
    with World(factory(hanging_weight(blank,library))) as world:
        assert world._chain_spans
        world._detach({'line/join-10'})
        assert 'line/join-10' not in world._chain_constraints
        assert 'line/join-10' not in {j['id'] for j in world.assembly.joints}
        assert not world._chain_spans
        for _ in range(24): world.step()
        assert_connected_lines(world)
