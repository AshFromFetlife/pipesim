"""Preview must simplify real rated lines without losing their physical load path."""
import copy
import json
import time
import os
import random
from pathlib import Path

import numpy as np
import pybullet as pb
import pytest

from pipesim.document import Assembly
from pipesim.physics import World,simulate,simplify_chains
from pipesim.math3d import transform
from test_rope_drop import hanging_weight
from test_swing_dynamics import FIXTURE,assert_connected_lines,assert_recorded_line_continuity


WHEELED_DOLLY=Path(__file__).parent/'fixtures'/'wheeled-inversion-dolly'/'design.json'


def _dolly_wheel_centres(recording,ground_z_mm=0):
    return [pose['position_mm'][2]-ground_z_mm for frame in recording['frames']
            for pid,pose in frame['parts'].items() if pid.startswith('wheel-1')]


def test_preview_wheeled_dolly_stays_on_floor_during_human_motion():
    assembly=Assembly.load(WHEELED_DOLLY)
    recording=simulate(assembly,5,fps=10,mode='preview')
    wheel_heights=_dolly_wheel_centres(recording)
    # All four wheel centres should remain near their 50 mm radius, including
    # after the suspended human's arms start to swing. The old four-substep
    # solve had launched them metres above the floor by 0.2 s.
    assert min(wheel_heights)>35
    assert max(wheel_heights)<90
    assert recording['final_max_speed_m_s']<10


@pytest.mark.parametrize('rate,ground_z,friction',[
    (30,2.3571295109997292,0.888408532488334),
    (240,-3.9431890249128196,0.7544639831823877),
    (30,-3.2330991701225886,0.8553855765555709),
    (120,3.5961082240370494,0.593125075308008),
    (120,2.8101413974706793,0.417589191517601),
    (120,-0.01970802448440878,0.8366594321574126),
])
def test_preview_wheel_contact_survives_rate_and_floor_variations(rate,ground_z,friction):
    baseline=Assembly.load(WHEELED_DOLLY)
    doc=copy.deepcopy(baseline.doc)
    doc['environment']={'ground':True,'ground_z_mm':ground_z,
                        'friction':friction}
    recording=simulate(Assembly.from_doc(doc,baseline.base,baseline.library),.6,
                       fps=10,mode='preview',preview_steps_per_second=rate)
    heights=_dolly_wheel_centres(recording,ground_z)
    assert min(heights)>30 and max(heights)<90
    assert recording['final_max_speed_m_s']<10


def test_preview_wheeled_floor_contacts_fuzz_rate_clearance_and_friction():
    baseline=Assembly.load(WHEELED_DOLLY)
    root=int(os.environ.get('PIPESIM_PREVIEW_WHEEL_FUZZ_SEED',str(random.SystemRandom().randrange(2**32))))
    rng=random.Random(root)
    print(f'Preview wheel root_seed={root}',flush=True)
    cases=[(60,0,.7),(30,-3,.5),(120,3,.9),(240,0,.7),
           (30,2.3571295109997292,0.888408532488334),
           (240,-3.9431890249128196,0.7544639831823877),
           (30,-3.2330991701225886,0.8553855765555709),
           (120,3.5961082240370494,0.593125075308008),
           (120,2.8101413974706793,0.417589191517601),
           (120,-0.01970802448440878,0.8366594321574126)]
    cases.extend((rng.choice((30,60,120,240)),rng.uniform(-5,5),rng.uniform(.4,1.0))
                 for _ in range(4))
    for index,(rate,ground_z,friction) in enumerate(cases):
        doc=copy.deepcopy(baseline.doc)
        doc['environment']={'ground':True,'ground_z_mm':ground_z,'friction':friction}
        assembly=Assembly.from_doc(doc,baseline.base,baseline.library)
        recording=simulate(assembly,.6,fps=10,mode='preview',preview_steps_per_second=rate)
        assert recording['quality']['physics_steps_per_second']==max(60,rate)
        heights=_dolly_wheel_centres(recording,ground_z)
        # Perturbed friction/clearance can lift one wheel transiently. Detect
        # the runaway metres-high launch, rather than demanding all four stay
        # in contact during the human's motion.
        assert min(heights)>30 and max(heights)<250 and recording['final_max_speed_m_s']<15, (
            f'root={root} case={index} rate={rate} ground_z={ground_z:.2f} '
            f'friction={friction:.2f} wheel_range={min(heights):.1f}..{max(heights):.1f} '
            f'speed={recording["final_max_speed_m_s"]:.1f}')


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


def test_preview_ballast_cannot_make_coarse_rope_heavier_than_its_payload(factory,blank,library):
    doc=hanging_weight(blank,library)
    ballast=copy.deepcopy(doc['parts'][1]);ballast['id']='ballast'
    ballast['body']['mass_kg']=2000;ballast['pose']['position_mm']=[5000,0,0]
    doc['parts'].append(ballast)
    grouped,_=simplify_chains(factory(doc),1000,target_groups=4,straightness=.98)
    with World(grouped,1/60,preview_iterations=60) as world:
        line_links={world.part_map[pid][:2] for pid in world.part_map if pid.startswith('line/')}
        mass=sum(pb.getDynamicsInfo(body,link,physicsClientId=world.client)[0] for body,link in line_links)
        assert mass<1, 'a remote ballast inflated the numerical rope mass above 10% of its payload'


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


def test_preview_keeps_its_budget_without_replaying_full(factory,blank,monkeypatch):
    step=World.step; builds=[]
    build=World._build
    def observe(world,assembly):
        builds.append(world.preview_iterations);build(world,assembly)
    def drift(world):
        step(world);world.preview_gap_mm=90
    monkeypatch.setattr(World,'_build',observe);monkeypatch.setattr(World,'step',drift)
    result=simulate(factory(blank),.1,mode='preview')
    assert builds==[60]
    assert result['quality']['resolved_mode']=='preview'
    assert result['dt_s']==pytest.approx(1/60)
    assert result['substeps']==4 and result['solver_iterations']==60
    assert result['quality']['contact_refinements']==0
    assert not result['beam_model']


def test_preview_remains_cancellable(factory,blank):
    from pipesim.simulation_control import SimulationCancelled
    cancel=[False]
    def progress(update):
        if update.get('phase')=='simulating':cancel[0]=True
    with pytest.raises(SimulationCancelled):
        simulate(factory(blank),.2,mode='preview',progress=progress,cancelled=lambda:cancel[0])


@pytest.mark.parametrize('line_index',range(4))
def test_preview_extra_detail_applies_only_to_selected_line(library,line_index):
    assembly=Assembly.from_doc(json.loads(FIXTURE.read_text()),FIXTURE.parent,library)
    root=f'chain-{line_index+1}'
    result=simulate(assembly,.001,mode='preview',preview_detail_parts=[root])
    assert result['quality']['resolved_mode']=='preview'
    remaining=result['chain_simplification']['frozen_joints']
    assert remaining and all(not jid.startswith(root+'/') for jid in remaining)
    assert all(pid.startswith(root+'/') for pid in result['quality']['detail_parts'])
    assert not result['beam_model']
    assert result['substeps']==4


def test_preview_beams_are_rigid_unless_selected(library):
    assembly=Assembly.from_doc(json.loads(FIXTURE.read_text()),FIXTURE.parent,library)
    original=copy.deepcopy(assembly.doc)
    coarse=simulate(assembly,.001,mode='preview')
    assert not coarse['beam_model'] and not coarse['beam_elements']
    detail=simulate(assembly,.1,mode='preview',preview_detail_parts=['tube-c-4'],preview_beam_segment_mm=1000)
    assert set(detail['beam_model'])=={'tube-c-4'}
    assert len(detail['beam_model']['tube-c-4']['segments'])==4
    assert detail['quality']['rigid_beams'] is False
    assert detail['quality']['resolved_mode']=='preview'
    assert detail['solver_iterations']==60 and detail['substeps']==4
    assert assembly.doc==original


@pytest.mark.parametrize('rate',[30,60,120,240])
def test_preview_honours_requested_rate_for_rigid_scenes(factory,blank,rate):
    result=simulate(factory(blank),.1,mode='preview',preview_steps_per_second=rate)
    assert result['dt_s']==pytest.approx(1/rate)
    assert result['substeps']==4


@pytest.mark.parametrize('options',[{'mode':'quick'},{'preview_steps_per_second':0},
    {'preview_solver_iterations':True},{'preview_beam_segment_mm':float('nan')}])
def test_invalid_preview_preferences_are_rejected_before_start(factory,blank,options):
    with pytest.raises(ValueError):simulate(factory(blank),**options)


def test_preview_randomized_loaded_lines_preserve_motion(factory,blank,library):
    root=int(os.environ.get('PIPESIM_PREVIEW_FUZZ_SEED',str(random.SystemRandom().randrange(2**32))))
    rng=random.Random(root)
    print(f'Preview line root_seed={root}',flush=True)
    for index in range(24):
        seed=index if index<4 else rng.randrange(2**32)
        case=random.Random(seed)
        slack=100 if seed%3==0 else 0
        doc=hanging_weight(blank,library,mass_kg=case.uniform(1,24),slack_mm=slack,
                           swing_deg=0 if slack else case.uniform(-45,45))
        options={'mode':'preview','chain_links_per_body':case.choice([1,2,4,8,12]),
                 'preview_steps_per_second':case.choice([30,60,120,240]),'preview_solver_iterations':case.choice([40,60,100])}
        try:
            assembly=factory(doc)
            full=simulate(assembly,.3)
            preview=simulate(assembly,.3,**options)
            assert not any(e['type']=='joint_break' for e in preview['events'])
            assert (options['chain_links_per_body']==1
                    or preview['chain_simplification']['frozen_joints'])
            assert_recorded_line_continuity(assembly,preview)
            # Close to the tensile rating, coarse geometry can change whether
            # a transient crosses the threshold. Compare trajectories only
            # while both systems retain the same topology. Clear overloads
            # and known subcritical loads have separate outcome assertions.
            cutoff=min([e['time_s'] for result in (full,preview) for e in result['events'] if e['type']=='joint_break']+[float('inf')])
            error=max(np.linalg.norm(transform(a['parts']['weight'])[:3,3]-transform(b['parts']['weight'])[:3,3])
                      for a,b in zip(full['frames'],preview['frames']) if max(a['time_s'],b['time_s'])<cutoff)
            # Fast Preview at 30 Hz promises approximate motion, not Full's
            # millimetre trajectory. Retain a 20% line-length error bound.
            assert error<80, f'Payload differs by {error:.2f} mm'
        except Exception as error:
            directory=Path('fuzz-runs/preview')/f'{root}-{seed}-{time.time_ns()}'
            directory.mkdir(parents=True,exist_ok=True)
            (directory/'design.json').write_text(json.dumps(doc))
            (directory/'failure.json').write_text(json.dumps({'root':root,'seed':seed,'options':options,'error':str(error)}))
            raise AssertionError(f'Preview case {seed}; reproduction {directory}') from error


def test_full_physics_clear_tensile_overload_still_severs_a_line(factory,blank,library):
    assembly=factory(hanging_weight(blank,library,mass_kg=80,slack_mm=100))
    result=simulate(assembly,.3,mode='full')
    breaks=[e for e in result['events'] if e['type']=='joint_break']
    assert len(breaks)==1
    assert_recorded_line_continuity(assembly,result)


@pytest.mark.parametrize('rate',[30,60,120,240])
def test_preview_retains_overloaded_line_and_reports_strength_limitation(factory,blank,library,rate):
    assembly=factory(hanging_weight(blank,library,mass_kg=80,slack_mm=100))
    result=simulate(assembly,.3,mode='preview',preview_steps_per_second=rate)
    assert not any(e['type']=='joint_break' for e in result['events'])
    assert any('breakage is disabled' in limitation for limitation in result['limitations'])
    assert_recorded_line_continuity(assembly,result)
