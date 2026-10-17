"""Retained failures and a timed campaign for coordinated build planning.

The child process deadline catches work hidden inside one insertion sweep, which
the old search-state budget did not limit. Every failure saves its actual input,
result and seeds. Replay with PIPESIM_PLANNING_FUZZ_CASE_SEED=<seed>.
"""
from __future__ import annotations

import json
import copy
import os
from pathlib import Path
import random
import secrets
import subprocess
import sys
import time

if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest

from fuzz_runtime import budget
from pipesim.document import Assembly
from pipesim.planning import plan_build
from planning_cases import (FIXTURES, assert_complete_plan, assert_gravity_support,
                            assert_sweep_samples,
                            anchored_cantilever, capture_frame, cube, fixture, heavy_workbench, ladder,
                            remote_bore_obstruction, rotated_split_clamp, transformed)
from test_planning import capped_socket


RETAINED_SEEDS = (0, 1, 2, 3, 4, 35, 216, 487, 18, 19, 14, 29, 436,
                  221317699909015487, 44, 66, 131, 161, 2853301854875761127)
# The planner itself has a 300-second deadline. Allow assembly resolution and
# the independent collision replay their own time outside that search budget.
CASE_TIMEOUT_SECONDS = 600


def case_document(seed):
    family = seed % 5
    if family == 0:
        doc = cube(seed)
    elif family == 1:
        doc = (heavy_workbench(seed) if seed & 128 and not seed & (16 | 64) else
               capture_frame(seed) if seed & 64 and not seed & 16 else
               rotated_split_clamp(seed) if seed & 16 else
               ladder(seed, levels=1 + random.Random(seed).randrange(7)))
        if seed & 128 and not seed & (16 | 64) and seed & 32:
            doc['build']['max_lift_mass_kg'] = 15
    elif family == 2:
        doc = transformed(fixture('complicated-metal'), seed)
    elif family == 3:
        doc = capped_socket(obstructed=True)
        obstacle = next(p for p in doc['parts'] if p['id'] == 'obstacle')
        rng = random.Random(seed)
        obstacle['parameters']['height_mm'] = rng.uniform(.2, 2)
        obstacle['pose']['position_mm'][2] = rng.uniform(-400, -100)
        doc = transformed(doc, seed, ground=False)
    else:
        doc = (anchored_cantilever(seed) if seed & 32 else
               remote_bore_obstruction(seed))
    if seed & 1:
        # Naming and JSON insertion order must not determine feasibility.
        names = list(range(len(doc['parts'])))
        random.Random(seed + 100).shuffle(names)
        rename = {part['id']: f'part-{name:04}'
                  for part, name in zip(doc['parts'], names)}
        for part in doc['parts']:
            part['id'] = rename[part['id']]
        for joint in doc['joints']:
            for end in ('a', 'b'):
                joint[end]['part'] = rename[joint[end]['part']]
        for anchor in doc.get('anchors', []):
            anchor['part'] = rename[anchor['part']]
        if 'sequence' in doc.get('build', {}):
            doc['build']['sequence'] = [rename[p] for p in doc['build']['sequence']]
    return doc


def check_case(seed, repro_dir, *, saved=None):
    doc = saved['document'] if saved is not None else case_document(seed)
    family = saved.get('family', seed % 5) if saved is not None else seed % 5
    expected = saved.get('expected_status') if saved is not None else None
    expected = expected or ('blocked' if family in (3, 4) else
        'indeterminate' if family == 1 and doc.get('build', {}).get('max_lift_mass_kg') == 15
        else 'buildable')
    grouped = saved.get('grouped', family in (0, 2)) if saved is not None else family in (0, 2)
    result = None
    original = copy.deepcopy(doc)
    started = time.monotonic()
    try:
        assembly = Assembly.from_doc(doc, FIXTURES)
        print(f'Planning case_seed={seed} phase=search', flush=True)
        result = plan_build(assembly)
        print(f'Planning case_seed={seed} phase=replay status={result["status"]} '
              f'planner_seconds={result.get("elapsed_s", 0)}', flush=True)
        assert result.get('elapsed_s', 0) <= doc.get('build', {}).get('time_limit_s', 300)+1, result
        if expected != 'buildable':
            assert result['status'] == expected and not result['steps'], result
        else:
            assert_complete_plan(assembly, result, grouped=grouped)
            assert_gravity_support(assembly, result)
            assert_sweep_samples(assembly, result)
            if family in (0, 1) and max(part.mass for part in assembly.parts.values()) <= 15:
                assert not result['certificate'].get('fallback_used', False), result
                assert all(step.get('group_mass_kg', 0) <= 15 for step in result['steps'])
        assert doc == original, 'Planning changed the authored design'
    except Exception:
        repro_dir = Path(repro_dir)
        repro_dir.mkdir(parents=True, exist_ok=True)
        path = repro_dir / f'planning-{seed}.json'
        path.write_text(json.dumps({'case_seed': seed, 'family': family,
            'expected_status': expected, 'grouped': grouped,
            'document': doc, 'result': result,
            'elapsed_seconds': time.monotonic()-started}, indent=2), encoding='utf-8')
        raise


def run_case(seed, repro_dir, *, replay=None):
    command = [sys.executable, str(Path(__file__).resolve()),
               '--replay' if replay else '--case', str(replay or seed), str(repro_dir)]
    try:
        completed = subprocess.run(command, capture_output=True, text=True,
            cwd=Path(__file__).resolve().parents[1], timeout=CASE_TIMEOUT_SECONDS)
    except subprocess.TimeoutExpired as exc:
        path = Path(repro_dir) / f'planning-{seed}-timeout.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({'case_seed': seed,
            'document': (json.loads(Path(replay).read_text(encoding='utf-8'))['document']
                         if replay else case_document(seed)),
            'timeout_seconds': CASE_TIMEOUT_SECONDS,
            'worker_stdout': (exc.stdout.decode(errors='replace')
                              if isinstance(exc.stdout, bytes) else exc.stdout),
            'worker_stderr': (exc.stderr.decode(errors='replace')
                              if isinstance(exc.stderr, bytes) else exc.stderr)},
            indent=2), encoding='utf-8')
        raise AssertionError(f'Planner made no bounded progress: case_seed={seed}; '
            f'repro={path}; PIPESIM_PLANNING_FUZZ_CASE_SEED={seed}') from exc
    assert completed.returncode == 0, (
        f'Planning fuzz case_seed={seed}; replay with '
        f'PIPESIM_PLANNING_FUZZ_CASE_SEED={seed}\n'
        f'{completed.stdout}\n{completed.stderr}')
    assert f'Planning case_seed={seed} passed' in completed.stdout, completed.stdout
    print(f'Planning case_seed={seed} family={seed % 5} passed', flush=True)


@pytest.mark.parametrize('seed', RETAINED_SEEDS)
def test_retained_parallel_frame_and_tuple_planning(seed, tmp_path):
    run_case(seed, tmp_path)


def test_fresh_parallel_assembly_planning_campaign():
    repro_dir = Path(os.environ.get('PIPESIM_PLANNING_FUZZ_REPRO_DIR',
        str(Path(__file__).resolve().parents[1] / 'fuzz-runs' / 'planning')))
    replay_file = os.environ.get('PIPESIM_PLANNING_FUZZ_REPLAY')
    if replay_file:
        saved = json.loads(Path(replay_file).read_text(encoding='utf-8'))
        run_case(saved['case_seed'], repro_dir, replay=Path(replay_file).resolve())
        return
    replay = os.environ.get('PIPESIM_PLANNING_FUZZ_CASE_SEED')
    if replay is not None:
        run_case(int(replay), repro_dir)
        return
    root_seed = int(os.environ.get('PIPESIM_PLANNING_FUZZ_SEED',
                                  str(secrets.randbits(64))))
    rng = random.Random(root_seed)
    minutes, started, deadline = budget(default_minutes=30)
    count = 0
    print(f'Planning fuzz root_seed={root_seed} budget_minutes={minutes}', flush=True)
    while time.monotonic() < deadline or not count:
        # Cycle families deliberately: fresh randomness must not silently omit
        # the large reported closed-loop topology for an entire campaign.
        seed = rng.randrange(2**60) * 5 + count % 5
        try:
            run_case(seed, repro_dir)
        except Exception as exc:
            raise AssertionError(f'Planning fuzz root_seed={root_seed} '
                f'case_seed={seed} case_index={count}') from exc
        count += 1
    print(f'Planning fuzz passed {count} fresh cases across all five families '
          f'in {time.monotonic()-started:.1f}s; root_seed={root_seed}', flush=True)


if __name__ == '__main__':
    assert len(sys.argv) == 4 and sys.argv[1] in ('--case', '--replay')
    saved = (json.loads(Path(sys.argv[2]).read_text(encoding='utf-8'))
             if sys.argv[1] == '--replay' else None)
    case_seed = saved['case_seed'] if saved is not None else int(sys.argv[2])
    check_case(case_seed, sys.argv[3], saved=saved)
    print(f'Planning case_seed={case_seed} passed', flush=True)
