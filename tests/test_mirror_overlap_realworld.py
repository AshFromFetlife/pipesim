"""Finalization cases taken from a mirrored connector frame edited in the UI."""

import copy
import json
import os
from pathlib import Path
import random
import secrets
import shutil

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.document import Assembly
from pipesim.drafting import finalize
from pipesim.math3d import pose_of, transform
from pipesim.symmetry import materialize_mirror
from pipesim.validation import CollisionWorld, validate


FIXTURE = Path(__file__).parent / 'fixtures' / 'draft-mirror-overlap.json'
THROUGH_FIXTURE = Path(__file__).parent / 'fixtures' / 'tc173mc-mirrored-through.json'
LISTED_SEEDS = tuple(int(value) for value in
                     os.environ.get('PIPESIM_REALWORLD_MIRROR_CASE_SEEDS', '').split(',') if value)
CASE_SEEDS = LISTED_SEEDS or (13, 139, secrets.randbits(64), secrets.randbits(64))


def _case(tmp_path):
    shutil.copytree(FIXTURE.parent / 'assets', tmp_path / 'assets', dirs_exist_ok=True)
    return json.loads(FIXTURE.read_text(encoding='utf-8'))


def _before_bake(document):
    document = copy.deepcopy(document)
    document['parts'] = [part for part in document['parts']
                         if '-mirror-mirror-1' not in part['id']]
    group = document['draft_subassemblies'][0]
    group['runs'] = [run for run in group['runs']
                     if '-mirror-mirror-1' not in run['id']]
    for run in group['runs']:
        run['attachments'] = [attachment for attachment in run.get('attachments', [])
                              if '-mirror-mirror-1' not in attachment['connector']]
    group['mirrors'] = [{'id': 'mirror-1', 'axis': 'x', 'offset_mm': 0, 'scope': 'scene'}]
    return document


def test_saved_mirror_frame_finalizes_without_false_collisions(tmp_path, library):
    document = _case(tmp_path)
    original = copy.deepcopy(document)
    result = finalize(Assembly.from_doc(document, tmp_path, library))
    assert result['status'] == 'finalized', result
    assert document == original
    assert 'tube-c-2-copy-2-mirror-mirror-1' not in {
        part['id'] for part in result['document']['parts']}
    assert validate(Assembly.from_doc(result['document'], tmp_path, library))['valid']


def test_real_non_socket_overlap_remains_visible(tmp_path, library):
    document = _case(tmp_path)
    document['parts'].append({'id': 'obstruction', 'catalog': 'generic.box',
        'parameters': {'width_mm': 100, 'depth_mm': 100, 'height_mm': 100, 'mass_kg': 1},
        'pose': {'position_mm': [-640, -800, 189.04]}})
    result = finalize(Assembly.from_doc(document, tmp_path, library))
    assert result['status'] == 'finalized', result
    issues = validate(Assembly.from_doc(result['document'], tmp_path, library))['issues']
    assert any(issue['code'] == 'INTERSECTION' and
               set(issue['parts']) == {'obstruction', 'tube-c-3'} for issue in issues)


def test_saved_four_way_swing_through_socket_accepts_its_inserted_pipe(tmp_path, library):
    shutil.copytree(FIXTURE.parent / 'assets', tmp_path / 'assets', dirs_exist_ok=True)
    document = json.loads(THROUGH_FIXTURE.read_text(encoding='utf-8'))
    assembly = Assembly.from_doc(document, tmp_path, library)
    pair = ('tc173mc-1-mirror-mirror-1', 'tube-c-2')
    with CollisionWorld(assembly) as world:
        assert max(-contact[8]*1000 for contact in world.contacts(*pair)) > 40
    assert not any(issue['code'] == 'INTERSECTION' for issue in validate(assembly)['issues'])


@pytest.mark.parametrize('case_seed', CASE_SEEDS)
def test_realworld_through_socket_case(case_seed, tmp_path, library):
    shutil.copytree(FIXTURE.parent / 'assets', tmp_path / 'assets', dirs_exist_ok=True)
    document = json.loads(THROUGH_FIXTURE.read_text(encoding='utf-8'))
    rng = random.Random(case_seed)
    rotation = Rotation.from_euler('xyz', [rng.uniform(-180, 180) for _ in range(3)],
                                   degrees=True).as_matrix()
    translation = np.array([rng.uniform(-300, 300) for _ in range(3)])
    for part in document['parts']:
        matrix = transform(part['pose'])
        matrix[:3, 3] = rotation @ matrix[:3, 3] + translation
        matrix[:3, :3] = rotation @ matrix[:3, :3]
        part['pose'] = pose_of(matrix)
    try:
        assembly = Assembly.from_doc(document, tmp_path, library)
        assert validate(assembly, collisions=False)['valid']
        assert validate(assembly)['valid']
    except Exception as error:
        repro = tmp_path / f'through-socket-{case_seed}.json'
        repro.write_text(json.dumps({'case_seed': case_seed, 'document': document}, indent=2),
                         encoding='utf-8')
        pytest.fail(f'case_seed={case_seed}, repro={repro}: {error}')


@pytest.mark.parametrize('case_seed', CASE_SEEDS)
def test_realworld_mirror_overlap_case(case_seed, tmp_path, library):
    document = _before_bake(_case(tmp_path))
    free_end = random.Random(case_seed).uniform(40, 606.88)
    run = next(run for run in document['draft_subassemblies'][0]['runs']
               if run['id'] == 'tube-c-2-copy-2')
    run['end_mm'][0] = free_end
    try:
        baked = materialize_mirror(
            Assembly.from_doc(document, tmp_path, library), 'draft-1', 'mirror-1')
        baked_runs = baked['draft_subassemblies'][0]['runs']
        assert 'tube-c-2-copy-2-mirror-mirror-1' not in {run['id'] for run in baked_runs}
        result = finalize(Assembly.from_doc(baked, tmp_path, library))
        assert result['status'] == 'finalized', result
        assert validate(Assembly.from_doc(result['document'], tmp_path, library))['valid']
    except Exception as error:
        repro = tmp_path / f'mirror-overlap-{case_seed}.json'
        repro.write_text(json.dumps({'case_seed': case_seed, 'source_document': document},
                                    indent=2), encoding='utf-8')
        pytest.fail(f'case_seed={case_seed}, free_end_mm={free_end}, repro={repro}: {error}')
