"""Exercise physical invariants, not just finite output or absence of events."""
import copy
import json
import math
import os
from pathlib import Path
import random
import time

import numpy as np
import pybullet as pb
import pytest
from scipy.spatial.transform import Rotation

from pipesim.beam_dynamics import prepare_beams
from pipesim.document import Assembly
from pipesim.grouping import update_object_parameters
from pipesim.math3d import pose_of,transform
from pipesim.physics import World,simulate,simplify_chains
from test_swing_dynamics import FIXTURE,assert_connected_lines,assert_local_line_pivots,assert_recorded_line_continuity


def run_case(seed,library):
    rng=random.Random(seed)
    fragment=seed%4==2
    beam=seed%4==1
    replay=os.environ.get('PIPESIM_DYNAMICS_FUZZ_REPLAY')
    if replay:
        doc=json.loads((Path(replay)/'design.json').read_text())
    elif fragment:
        doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[],'joints':[],
             'anchors':[],'objects':[{'id':'line','template':'chain',
              'parameters':{'length_mm':400,'link_catalog':'generic.rope-jute-6'},
              'pose':{'position_mm':[0,0,700],'rotation_deg':[0,25,rng.uniform(-180,180)]}}]}
    else:
        doc=json.loads(FIXTURE.read_text())
        if seed>3 and seed!=7:
            human=next(o for o in doc['objects'] if o['id']=='human-1')
            human['parameters']['mass_kg']=rng.choice([30,50,80,100])
            weight=next(p for p in doc['parts'] if p['id']=='box-1')
            weight.setdefault('parameters',{})['mass_kg']=rng.choice([20,200,2000])
            assembly=Assembly.from_doc(doc,FIXTURE.parent,library)
            profiles=['generic.rope-nylon-10','generic.rope-jute-6',
                      'generic.chain-link','generic.strap-seatbelt-65']
            profile=rng.choice(profiles)
            for line in ('chain-1','chain-2','chain-3','chain-4'):
                material=rng.choice(profiles) if seed%8>=4 else profile
                doc=update_object_parameters(assembly,line,{'length_mm':1000,'link_catalog':material})
                assembly=Assembly.from_doc(doc,FIXTURE.parent,library)
            delta=np.eye(4)
            delta[:3,:3]=Rotation.from_euler('z',rng.uniform(-180,180),degrees=True).as_matrix()
            delta[:3,3]=[rng.uniform(-1000,1000),rng.uniform(-1000,1000),rng.choice([0,200])]
            for item in doc['parts']+doc['objects']:
                item['pose']=pose_of(delta@transform(item.get('pose')))
    original=copy.deepcopy(doc)
    world=None
    case_options={}
    try:
        assembly=Assembly.from_doc(doc,FIXTURE.parent,library)
        if seed in (1,3) or seed%4==3:
            # Use the production recording path and the complete animation.
            # A short prefix missed constraint drift during the 1.5 s impact.
            last_progress=[-.5]
            def progress(info):
                elapsed=info.get('simulated_s',0)
                if elapsed>=last_progress[0]+.5:
                    last_progress[0]=elapsed
                    print(f'Dynamics full swing: {elapsed:.2f}/3.00 simulated seconds',flush=True)
            preview=seed!=1
            duration=3 if seed in (1,3) else .95 if seed==7 else .35
            case_options={'mode':'preview' if preview else 'full',
                          'chain_links_per_body':8 if seed==3 else rng.choice([1,2,4,8,12]) if preview else 1}
            if seed==7:
                case_options.update(chain_links_per_body=8,preview_steps_per_second=4000)
            elif preview and seed!=3:
                case_options.update(preview_steps_per_second=rng.choice([1000,2000,4000]),
                                    preview_solver_iterations=rng.choice([240,480]),
                                    preview_beam_segment_mm=rng.choice([400,800,1000]))
            if replay: case_options=json.loads((Path(replay)/'failure.json').read_text()).get('options') or case_options
            if preview and case_options['chain_links_per_body']>1:
                _,grouped=simplify_chains(assembly,case_options['chain_links_per_body'])
                assert grouped, 'preview silently skipped line grouping'
            recording=simulate(assembly,duration,30,progress=progress,**case_options)
            directory=Path(os.environ.get('PIPESIM_DYNAMICS_FUZZ_REPRO_DIR','fuzz-runs/dynamics'))
            directory.mkdir(parents=True,exist_ok=True)
            (directory/('last-preview-recording.json' if preview else 'last-swing-recording.json')).write_text(json.dumps(recording))
            assert 'tube-c-4' in recording['beam_model'], 'moving arm silently became rigid'
            if seed in (1,3,7):
                assert any(e['type']=='beam_yield' and e['part']=='tube-c-4' for e in recording['events'])
                assert not any(e['type']=='joint_break' for e in recording['events']), 'saved nylon swing developed a spurious rupture'
                assert not any(frame['broken_joints'] for frame in recording['frames']), 'saved nylon swing lost a connection'
            if preview:
                refined=recording.get('quality',{}).get('resolved_mode')=='full'
                assert refined or case_options['chain_links_per_body']==1 or recording['chain_simplification']['frozen_joints'], 'preview silently skipped line grouping'
                assert recording['mode']=='preview'
                if seed==3:assert not refined, 'default Preview lost its coarse-model performance'
                if refined:print(f'Dynamics Preview refined to Full: {recording["quality"]["refinement"]}',flush=True)
            maximum=assert_recorded_line_continuity(assembly,recording)
            print(f'Dynamics full swing recording: maximum rope joint gap {maximum:.3f} mm',flush=True)
            assert original==doc
            return
        if beam:
            assembly,model,_=prepare_beams(assembly)
            assert 'tube-c-4' in model, 'moving arm silently became rigid'
        with World(assembly,1/4000 if beam else 1/240) as world:
            if fragment:
                body,link,_=world.part_map['line/link-5']
                assert link==-1
                position,rotation=pb.getBasePositionAndOrientation(body,physicsClientId=world.client)
                offset=np.array([rng.uniform(-.003,.003) for _ in range(3)])
                pb.resetBasePositionAndOrientation(body,np.array(position)+offset,rotation,physicsClientId=world.client)
                world._detach({'line/join-4','line/join-14'})
                assert_local_line_pivots(world)
                assert not world._chain_spans
                start=np.mean([world.part_matrix(pid)[2,3] for pid in world.part_map])
            duration=.8 if fragment else (1.2 if seed>2 and seed%16==13 else .15) if beam else .35
            for step in range(round(duration/world.dt)):
                world.step()
                if step%max(1,round(.02/world.dt))==0:
                    assert_connected_lines(world,8)
                if fragment and abs(world.elapsed-.2)<world.dt/2:
                    height=np.mean([world.part_matrix(pid)[2,3] for pid in world.part_map])
                    assert abs(height-(start-9810*.2**2/2))<3, 'free fragments are not falling ballistically'
            if fragment:
                assert max(world.part_matrix(pid)[2,3] for pid in world.part_map)<80, 'free fragments remain suspended above the ground'
                assert len(world.broken)==2, 'one cut caused the remaining rope to shatter'
            if beam and seed==1:
                assert any(e['type']=='beam_yield' and e['part']=='tube-c-4' for e in world.events)
            assert original==doc, 'simulation changed the authored design'
    except Exception as error:
        directory=Path(os.environ.get('PIPESIM_DYNAMICS_FUZZ_REPRO_DIR','fuzz-runs/dynamics'))/f'{seed}-{time.time_ns()}'
        directory.mkdir(parents=True,exist_ok=True)
        (directory/'design.json').write_text(json.dumps(original,indent=2))
        (directory/'failure.json').write_text(json.dumps({'seed':seed,'beam':beam,'fragment':fragment,'options':case_options,'error':str(error)},indent=2))
        raise AssertionError(f'Dynamics seed={seed}; reproduction: {directory}') from error


def test_dynamics_long_randomized(library):
    minutes=float(os.environ.get('PIPESIM_DYNAMICS_FUZZ_MINUTES',os.environ.get('PIPESIM_FUZZ_MINUTES','45')))
    count=int(os.environ.get('PIPESIM_DYNAMICS_FUZZ_CASES','1000000'))
    assert math.isfinite(minutes) and minutes>0 and count>0
    root=int(os.environ.get('PIPESIM_DYNAMICS_FUZZ_SEED',str(random.SystemRandom().randrange(2**32))))
    assert 0<=root<2**32
    single=os.environ.get('PIPESIM_DYNAMICS_FUZZ_CASE_SEED')
    replay=os.environ.get('PIPESIM_DYNAMICS_FUZZ_REPLAY')
    if replay:
        single=str(json.loads((Path(replay)/'failure.json').read_text())['seed'])
    rng=random.Random(root);started=time.monotonic();completed=[]
    # Exercise the cheaper coarse model before the long Full reference, so a
    # broken preview cannot spend most of a campaign waiting to be exercised.
    retained=[0,3,7,1,2,3032479272,3198599736]
    minimum_cases=len(retained)+4
    print(f'Dynamics root_seed={root} budget_minutes={minutes}',flush=True)
    while len(completed)<count and (len(completed)<minimum_cases or time.monotonic()-started<minutes*60):
        if single is not None:seed=int(single)
        elif len(completed)<len(retained):seed=retained[len(completed)]
        else:
            while True:
                seed=rng.randrange(2**32)
                if len(completed)<minimum_cases:
                    # A slow retained replay must not consume the budget and
                    # silently turn a campaign into fixed regressions only.
                    seed=(seed&~3)|((len(completed)-len(retained))%4)
                # A previously discovered seed may since have been retained.
                # Replaying its original root must still produce fresh cases.
                if seed not in retained and seed not in completed:break
        print(f'Dynamics case_seed={seed} starting',flush=True)
        run_case(seed,library)
        completed.append(seed)
        print(f'Dynamics case_seed={seed} passed',flush=True)
        if single is not None: break
    print(f'Dynamics completed={len(completed)} elapsed_s={time.monotonic()-started:.1f}',flush=True)


def test_chain_material_keeps_embedded_hand_attachment_stable(library):
    # Discovered by the physical-invariant campaign: chain rings inside the
    # gripping hand had mount contacts enabled while nylon did not.
    run_case(3032479272,library)


def test_high_step_preview_keeps_swing_ropes_connected(library):
    run_case(7,library)


def test_saved_swing_oracle_rejects_false_breaks(library,monkeypatch,tmp_path):
    import sys
    module=sys.modules[__name__]
    recording={'beam_model':{'tube-c-4':{}},'frames':[],
               'events':[{'type':'beam_yield','part':'tube-c-4'},
                         {'type':'joint_break','joint':'chain-1/join-1'}]}
    monkeypatch.setattr(module,'simulate',lambda *args,**kwargs:recording)
    monkeypatch.delenv('PIPESIM_DYNAMICS_FUZZ_REPLAY',raising=False)
    monkeypatch.setenv('PIPESIM_DYNAMICS_FUZZ_REPRO_DIR',str(tmp_path))
    with pytest.raises(AssertionError) as failure:
        run_case(1,library)
    assert 'spurious rupture' in str(failure.value.__cause__)


def test_jute_motion_keeps_joint_closure_with_a_light_counterweight(library):
    run_case(3198599736,library)


@pytest.mark.parametrize('root',[42,4182449475])
def test_slow_retained_replay_cannot_skip_fresh_families(monkeypatch,root):
    import sys
    from types import SimpleNamespace
    module=sys.modules[__name__]
    completed=[]
    ticks=iter([0.,1000.,1000.])
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:next(ticks)))
    monkeypatch.setattr(module,'run_case',lambda seed,library:completed.append(seed))
    monkeypatch.delenv('PIPESIM_DYNAMICS_FUZZ_CASE_SEED',raising=False)
    monkeypatch.delenv('PIPESIM_DYNAMICS_FUZZ_REPLAY',raising=False)
    monkeypatch.setenv('PIPESIM_DYNAMICS_FUZZ_MINUTES','0.001')
    monkeypatch.setenv('PIPESIM_DYNAMICS_FUZZ_CASES','100')
    monkeypatch.setenv('PIPESIM_DYNAMICS_FUZZ_SEED',str(root))
    test_dynamics_long_randomized(None)
    assert completed[:7]==[0,3,7,1,2,3032479272,3198599736]
    assert len(completed)==11
    assert len(set(completed))==11
    assert [seed%4 for seed in completed[7:]]==[0,1,2,3]
