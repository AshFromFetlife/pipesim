"""Fresh-seed variations of the three-mirror, connected cube topology."""
import os
import random
import secrets
import subprocess
import sys
import json
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.document import Assembly
from pipesim.drafting import finalize
from pipesim.math3d import pose_of, transform
from pipesim.symmetry import materialize_all
from pipesim.validation import validate
from test_symmetry_cube_collisions import cube_corner


def _variant(base, seed):
    rng = random.Random(seed)
    # Unequal spans exercise the three axes independently; a uniform cube can
    # conceal an axis mix-up because swapping two edges leaves it unchanged.
    reach = [rng.uniform(180, 480) for _ in range(3)]
    origin = np.array([rng.uniform(-900, 900), rng.uniform(-900, 900),
                       rng.uniform(700, 1700)])
    rotation = Rotation.from_euler('xyz',
        [rng.choice((0, 90, 180, 270)) for _ in range(3)], degrees=True)
    basis = np.rint(rotation.as_matrix()).astype(int)
    assert np.all(np.abs(basis).sum(axis=0) == 1)
    original_center = np.array([0., 0., 1000.])
    doc = cube_corner(base, reach=reach)
    for part in doc['parts']:
        matrix = transform(part['pose'])
        matrix[:3, 3] = origin + rotation.apply(matrix[:3, 3]-original_center)
        matrix[:3, :3] = rotation.as_matrix() @ matrix[:3, :3]
        part['pose'] = pose_of(matrix)
    group = doc['draft_subassemblies'][0]
    for run in group['runs']:
        for end in ('start_mm', 'end_mm'):
            run[end] = (origin + rotation.apply(
                np.asarray(run[end])-original_center)).tolist()
    run_for_plane = {}
    for local_index, local_axis in enumerate('xyz'):
        world_index = int(np.argmax(np.abs(basis[:, local_index])))
        run_for_plane['xyz'[world_index]] = f'{local_axis}-edge'
    for mirror in group['mirrors']:
        mirror['offset_mm'] = float(origin['xyz'.index(mirror['axis'])])
        mirror['run_modes'] = {run_for_plane[mirror['axis']]: 'centered'}
    rng.shuffle(group['mirrors'])
    return doc


def _check_cube_case(base, case_seed, *, root_seed, index, repro_dir):
    context = f'root_seed={root_seed} case_seed={case_seed} index={index}'
    doc = baked_doc = result = None
    stage = 'generate'
    try:
        doc = _variant(base, case_seed)
        stage = 'materialize'
        before = Assembly.from_doc(doc, base)
        baked_doc = materialize_all(before)
        baked = Assembly.from_doc(baked_doc, base)
        assert len(baked_doc['parts']) == 8, context
        assert len(baked_doc['draft_subassemblies'][0]['runs']) == 12, context
        attachments = [attachment for run in baked_doc['draft_subassemblies'][0]['runs']
                       for attachment in run['attachments']]
        assert len(attachments) == 24, context
        assert len({(attachment['connector'], attachment['port'])
                    for attachment in attachments}) == 24, context
        stage = 'finalize'
        result = finalize(baked)
        assert result['status'] == 'finalized', f'{context}: {result}'
        stage = 'validate'
        exact = Assembly.from_doc(result['document'], base)
        assert len(exact.joints) == 24, context
        report = validate(exact)
        assert report['valid'], f'{context}: {report["issues"]}'
    except Exception as exc:
        path = repro_dir / f'cube-fuzz-{case_seed}.json'
        path.write_text(json.dumps({
            'root_seed': root_seed, 'case_seed': case_seed, 'case_index': index,
            'stage': stage, 'source_document': doc,
            'materialized_document': baked_doc,
            'finalize_result': result,
        }, indent=2, default=str), encoding='utf-8')
        raise AssertionError(f'Cube mirror fuzz {context} stage={stage} '
                             f'repro={path}; replay with '
                             f'PIPESIM_CUBE_FUZZ_CASE_SEED={case_seed}') from exc


def test_randomized_orthogonal_mirror_cubes_finalize_without_collisions(tmp_path):
    root_seed = int(os.environ.get('PIPESIM_CUBE_FUZZ_SEED', secrets.randbits(64)))
    replay = os.environ.get('PIPESIM_CUBE_FUZZ_CASE_SEED')
    listed = tuple(int(value) for value in os.environ.get('PIPESIM_CUBE_FUZZ_CASE_SEEDS', '').split(',') if value)
    count = len(listed) if listed else 1 if replay else int(os.environ.get('PIPESIM_CUBE_FUZZ_CASES', 6))
    rng = random.Random(root_seed)
    print(f'cube mirror root_seed={root_seed}', flush=True)
    index_base = int(os.environ.get('PIPESIM_CUBE_FUZZ_CASE_INDEX_BASE', 0))
    for index in range(count):
        case_seed = listed[index] if listed else int(replay) if replay else rng.getrandbits(64)
        _check_cube_case(tmp_path, case_seed, root_seed=root_seed,
                         index=index_base+index, repro_dir=tmp_path)


@pytest.mark.skipif(os.environ.get('PIPESIM_CUBE_FUZZ_LONG') != '1',
                    reason='opt-in multi-hour orthogonal mirror cube campaign')
def test_cube_long_randomized(tmp_path):
    root_seed = int(os.environ.get('PIPESIM_CUBE_FUZZ_SEED', secrets.randbits(64)))
    case_limit = int(os.environ.get('PIPESIM_CUBE_FUZZ_CASES', 1000000))
    deadline = time.monotonic() + float(os.environ.get('PIPESIM_CUBE_FUZZ_SECONDS', 7200))
    rng = random.Random(root_seed)
    print(f'cube mirror long root_seed={root_seed} case_limit={case_limit}', flush=True)
    completed = 0
    started = time.monotonic()
    try:
        while completed < case_limit and time.monotonic() < deadline:
            batch = [rng.getrandbits(64) for _ in range(min(10, case_limit-completed))]
            batch_dir = tmp_path / f'batch-{completed//10:06d}'
            child_env = os.environ.copy()
            child_env['PIPESIM_CUBE_FUZZ_CASE_SEEDS'] = ','.join(map(str, batch))
            child_env['PIPESIM_CUBE_FUZZ_CASE_INDEX_BASE'] = str(completed)
            child_env.pop('PIPESIM_CUBE_FUZZ_LONG', None)
            child = subprocess.run(
                [sys.executable, '-m', 'pytest', '-q', '-s', f'--basetemp={batch_dir}',
                 'tests/test_symmetry_cube_fuzz.py', '-k', 'randomized_orthogonal_mirror_cubes_finalize_without_collisions'],
                env=child_env, capture_output=True, text=True, check=False)
            if child.returncode:
                pytest.fail(f'Cube fuzz root_seed={root_seed}, completed={completed}, '
                            f'batch_seeds={batch}, repro_dir={batch_dir}\n'
                            f'{child.stdout}\n{child.stderr}', pytrace=False)
            completed += len(batch)
            print(f'cube mirror long completed_models={completed} '
                  f'elapsed_s={time.monotonic()-started:.0f} last_case_seed={batch[-1]}', flush=True)
    finally:
        print(f'cube mirror long completed_models={completed}', flush=True)
    assert completed > 0, 'Long cube fuzz budget did not permit one model'
