"""Preview must simplify real rated lines without losing their physical load path."""
import copy
import json
import time
import os
import random
from pathlib import Path

import numpy as np
import pytest

from pipesim.document import Assembly
from pipesim.physics import World,simulate,simplify_chains
from pipesim.math3d import transform
from test_rope_drop import hanging_weight
from test_swing_dynamics import FIXTURE,assert_connected_lines,assert_recorded_line_continuity


@pytest.mark.parametrize('catalog',['generic.rope-nylon-10','generic.rope-jute-6','generic.chain-link','generic.strap-seatbelt-65'])
def test_rated_line_grouping_preserves_load_path_mass_and_breaking(factory,blank,library,catalog):
    doc=hanging_weight(blank,library,catalog=catalog)
    from pipesim.chain import generate
    last=generate(doc['objects'][0]['parameters'],library)['parts'][-1]['id']
    doc['joints'][1]['a']['part']='line/'+last
    assembly=factory(doc)
    # The generic metal chain has no catalogue rating; assign a uniform test
    # rating so this case exercises overload handling as well as geometry.
    for joint in assembly.joints:
        if joint.get('metadata',{}).get('chain_link'):joint.setdefault('break_force_n',1000)
    grouped,frozen=simplify_chains(assembly,8)
    assert frozen, 'rated lines silently bypassed grouping'
    assert assembly.doc==doc
    with World(assembly) as full,World(grouped) as preview:
        assert len(preview.body_ids)<len(full.body_ids)
        assert preview.part_map['line/link-1'][:2]==preview.part_map['line/link-2'][:2]
        assert [j for j in grouped.joints if j['id'] in ('top','bottom')]==doc['joints']
        assert len(preview._chain_spans)==len(full._chain_spans)==1
        assert preview._chain_spans[0][4:]==full._chain_spans[0][4:]
        assert all(preview.part_matrix(pid)==pytest.approx(part.matrix,abs=.002) for pid,part in assembly.parts.items())
        assert sum(p.mass for p in grouped.parts.values())==pytest.approx(sum(p.mass for p in assembly.parts.values()))
        # The grouped line must still fail under an actual overload, exactly
        # once, and remove its parallel support spring when it does.
        preview._span_tensions={j['id']:1e9 for j in preview._chain_joints}
        preview._break_events()
        assert len(preview.broken)==1
        assert not preview._chain_spans
        assert all(j['id'] not in preview._chain_constraints for j in assembly.joints if j['id'] in preview.broken)
        for _ in range(8): preview.step()
        assert_connected_lines(preview)


def test_real_swing_groups_rated_links_and_preserves_all_four_support_spans(library):
    assembly=Assembly.from_doc(json.loads(FIXTURE.read_text()),FIXTURE.parent,library)
    grouped,frozen=simplify_chains(assembly,8)
    assert len(frozen)>80
    with World(grouped) as world:
        assert len(world._chain_spans)==4
        assert len(world._chain_joints)<80


@pytest.mark.parametrize('slack,swing',[(0,25),(100,0)])
def test_preview_pendulum_trajectory_tracks_full_physics(factory,blank,library,slack,swing):
    assembly=factory(hanging_weight(blank,library,slack_mm=slack,swing_deg=swing))
    original=copy.deepcopy(assembly.doc)
    full=simulate(assembly,.3)
    preview=simulate(assembly,.3,mode='preview')
    assert preview['chain_simplification']['frozen_joints']
    assert preview['mode']=='preview' and full['mode']=='full'
    assert not any(e['type']=='joint_break' for e in preview['events'])
    assert_recorded_line_continuity(assembly,preview)
    errors=[np.linalg.norm(transform(a['parts']['weight'])[:3,3]-transform(b['parts']['weight'])[:3,3])
            for a,b in zip(full['frames'],preview['frames'])]
    assert max(errors)<30, f'Preview payload trajectory differs by {max(errors):.1f} mm'
    assert assembly.doc==original


def test_full_quality_is_independent_of_preview_preferences(factory,blank):
    first=simulate(factory(blank),.02)
    second=simulate(factory(blank),.02,preview_steps_per_second=240,preview_solver_iterations=40,preview_beam_segment_mm=2000)
    assert first['frames']==second['frames']
    assert first['quality']==second['quality']


def test_preview_discards_unconverged_attempt_and_returns_full_recording(factory,blank,monkeypatch):
    step=World.step
    def drift(world):
        step(world)
        if world.preview_iterations is not None:world.preview_gap_mm=9
    monkeypatch.setattr(World,'step',drift)
    assembly=factory(blank);original=copy.deepcopy(assembly.doc);updates=[]
    result=simulate(assembly,.02,mode='preview',progress=updates.append)
    full=simulate(assembly,.02)
    assert result['frames']==full['frames']
    assert result['mode']=='preview' and result['quality']['resolved_mode']=='full'
    assert result['quality']['refinement']['joint_gap_mm']==9
    assert result['quality']['requested']['preview_steps_per_second']==1000
    assert any('Refining Preview with Full physics' in update['message'] for update in updates)
    assert assembly.doc==original


def test_preview_refinement_remains_cancellable(factory,blank,monkeypatch):
    from pipesim.simulation_control import SimulationCancelled
    step=World.step
    def drift(world):
        step(world)
        if world.preview_iterations is not None:world.preview_gap_mm=9
    monkeypatch.setattr(World,'step',drift)
    cancel=[False]
    def progress(update):
        if 'Refining Preview with Full physics' in update['message']:cancel[0]=True
    with pytest.raises(SimulationCancelled):
        simulate(factory(blank),.02,mode='preview',progress=progress,cancelled=lambda:cancel[0])


def test_preview_beam_detail_keeps_the_long_arm_flexible(library):
    assembly=Assembly.from_doc(json.loads(FIXTURE.read_text()),FIXTURE.parent,library)
    result=simulate(assembly,.001,mode='preview',preview_beam_segment_mm=1000)
    # This setting makes the short frame members rigid, but must not disable
    # the long loaded arm's bending or change its mass/length in playback.
    assert set(result['beam_model'])=={'tube-c-4'}
    assert len(result['beam_model']['tube-c-4']['segments'])==4
    assert result['quality']['beam_segment_mm']==1000


def test_preview_keeps_ordinary_rigid_models_at_their_normal_time_step(factory,blank):
    result=simulate(factory(blank),.02,mode='preview')
    assert result['dt_s']==pytest.approx(1/240)
    assert result['substeps']==2


@pytest.mark.parametrize('options',[{'mode':'quick'},{'preview_steps_per_second':0},
    {'preview_solver_iterations':True},{'preview_beam_segment_mm':float('nan')}])
def test_invalid_preview_preferences_are_rejected_before_start(factory,blank,options):
    with pytest.raises(ValueError):simulate(factory(blank),**options)


def test_preview_randomized_loaded_lines_preserve_motion(factory,blank,library):
    root=int(os.environ.get('PIPESIM_PREVIEW_FUZZ_SEED',str(random.SystemRandom().randrange(2**32))))
    rng=random.Random(root)
    print(f'Preview line root_seed={root}',flush=True)
    for index in range(12):
        seed=index if index<4 else rng.randrange(2**32)
        case=random.Random(seed)
        slack=100 if seed%3==0 else 0
        doc=hanging_weight(blank,library,mass_kg=case.uniform(1,24),slack_mm=slack,
                           swing_deg=0 if slack else case.uniform(-45,45))
        options={'mode':'preview','chain_links_per_body':case.choice([1,2,4,8,12]),
                 'preview_steps_per_second':case.choice([1000,2000]),'preview_solver_iterations':case.choice([240,480])}
        try:
            assembly=factory(doc)
            full=simulate(assembly,.3)
            preview=simulate(assembly,.3,**options)
            assert (options['chain_links_per_body']==1 or preview.get('quality',{}).get('resolved_mode')=='full'
                    or preview['chain_simplification']['frozen_joints'])
            assert_recorded_line_continuity(assembly,preview)
            # Close to the tensile rating, coarse geometry can change whether
            # a transient crosses the threshold. Compare trajectories only
            # while both systems retain the same topology. Clear overloads
            # and known subcritical loads have separate outcome assertions.
            cutoff=min([e['time_s'] for result in (full,preview) for e in result['events'] if e['type']=='joint_break']+[float('inf')])
            error=max(np.linalg.norm(transform(a['parts']['weight'])[:3,3]-transform(b['parts']['weight'])[:3,3])
                      for a,b in zip(full['frames'],preview['frames']) if max(a['time_s'],b['time_s'])<cutoff)
            assert error<30, f'Payload differs by {error:.2f} mm'
        except Exception as error:
            directory=Path('fuzz-runs/preview')/f'{root}-{seed}-{time.time_ns()}'
            directory.mkdir(parents=True,exist_ok=True)
            (directory/'design.json').write_text(json.dumps(doc))
            (directory/'failure.json').write_text(json.dumps({'root':root,'seed':seed,'options':options,'error':str(error)}))
            raise AssertionError(f'Preview case {seed}; reproduction {directory}') from error


@pytest.mark.parametrize('mode',['full','preview'])
def test_clear_tensile_overload_still_severs_a_grouped_line(factory,blank,library,mode):
    assembly=factory(hanging_weight(blank,library,mass_kg=80,slack_mm=100))
    result=simulate(assembly,.3,mode=mode)
    breaks=[e for e in result['events'] if e['type']=='joint_break']
    assert len(breaks)==1
    assert_recorded_line_continuity(assembly,result)
