"""A connector must carry a free flexible line attached by a spherical joint.

The fixture includes both an independent draft graph and, in selected cases,
a draft run attached to the same connector. Both must remain valid after edits.
"""
import copy
import os
import random
import secrets

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipesim.drafting import preview
from pipesim.document import DocumentError
from pipesim.grouping import attach_part, move_object
from pipesim.math3d import pose_of
from pipesim.posing import transform_part


PROFILES = ('generic.chain-link', 'generic.rope-jute-6', 'generic.strap-seatbelt-65')
PITCH = {'generic.chain-link': 20, 'generic.rope-jute-6': 20,
         'generic.strap-seatbelt-65': 25}


def scene(factory, blank, catalog='generic.chain-link', end='start', *, anchored=False,
          link_count=12, extra_runs=0, draft_connected=False):
    doc = copy.deepcopy(blank)
    doc['parts'] = [
        {'id': 'hanger', 'pose': {'position_mm': [140, -220, 880]},
         'body': {'kind': 'connector', 'mass_kg': .25,
                  'geometry': [{'type': 'sphere', 'radius_mm': 15}],
                  'ports': {'eye': {'type': 'eye', 'position_mm': [0, 0, 35],
                                    'axis': [0, 1, 0], 'assembly': 'hook'},
                            'bore': {'type': 'socket', 'profile': 'round',
                                     'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                                     'axis': [1, 0, 0], 'through': True,
                                     'engagement_mm': 40, 'min_engagement_mm': 15}}}},
        {'id': 'pipe-fitting', 'pose': {'position_mm': [500, 350, 250]},
         'body': {'kind': 'connector', 'mass_kg': .2,
                  'geometry': [{'type': 'sphere', 'radius_mm': 12}],
                  'ports': {'bore': {'type': 'socket', 'profile': 'round',
                                     'diameter_mm': 42.4, 'position_mm': [0, 0, 0],
                                     'axis': [1, 0, 0], 'through': True,
                                     'engagement_mm': 40, 'min_engagement_mm': 15}}}},
    ]
    runs = [{'id': 'draft-pipe', 'catalog': 'tubeclamp.tube-C',
             'start_mm': [0, 350, 250], 'end_mm': [1000, 350, 250],
             'attachments': [{'connector': 'pipe-fitting', 'port': 'bore'}]}]
    if draft_connected:
        runs.append({'id': 'hanger-pipe', 'catalog': 'tubeclamp.tube-C',
                     'start_mm': [-360, -220, 880], 'end_mm': [640, -220, 880],
                     'attachments': [{'connector': 'hanger', 'port': 'bore'}]})
    for index in range(extra_runs):
        y = 600 + index * 180
        runs.append({'id': f'draft-spare-{index}', 'catalog': 'tubeclamp.tube-C',
                     'start_mm': [0, y, 250], 'end_mm': [600, y, 250]})
    doc['draft_subassemblies'] = [{'id': 'unfinished-frame', 'runs': runs}]
    doc['objects'] = [{'id': 'line', 'template': 'chain',
                       'parameters': {'length_mm': link_count * PITCH[catalog],
                                      'link_catalog': catalog},
                       'pose': {'position_mm': [140, -220, 1000]}}]
    if anchored:
        doc['anchors'] = [{'part': 'hanger', 'surface': 'fixture'}]
    before = factory(doc)
    link = 'line/link-1' if end == 'start' else f'line/link-{link_count}'
    attached = attach_part(before, 'line', link,
                           {'part': 'hanger', 'port': 'eye'}, 'spherical')
    after = factory(attached['document'])
    assert_attachment(after)
    return after, attached['joint']


def assert_attachment(assembly, joint=None):
    attached = [j for j in assembly.joints if j['id'].endswith('-attachment')]
    assert len(attached) == 1
    if joint is not None:
        assert attached[0]['id'] == joint['id']
    for connection in assembly.joints:
        first, second, _ = assembly.joint_frames(connection)
        assert np.linalg.norm(first - second) < .05, connection['id']
    assert all(not part['conflicts'] for part in preview(assembly)), 'draft graph gained conflicts'


def moved_connector(factory, before, joint, *, shift=(0, 0, 0), rotation_deg=(0, 0, 0), mode='translate'):
    expected = before.parts['hanger'].matrix.copy()
    expected[:3, 3] += shift
    expected[:3, :3] = (Rotation.from_euler('xyz', rotation_deg, degrees=True).as_matrix()
                         @ expected[:3, :3])
    result = transform_part(before, 'hanger', pose_of(expected), mode)
    after = factory(result['document'])
    assert np.allclose(after.parts['hanger'].matrix, expected, atol=.5, rtol=0), result
    assert_attachment(after, joint)
    return after, result


@pytest.mark.parametrize('catalog', PROFILES)
@pytest.mark.parametrize('end', ('start', 'end'))
def test_free_connector_translation_carries_attached_flexible_line(factory, blank, catalog, end):
    before, joint = scene(factory, blank, catalog, end)
    old_eye = before.joint_frames(joint)[0]
    after, result = moved_connector(factory, before, joint, shift=(15, -12, 85))
    new_eye = after.joint_frames(joint)[0]
    assert np.allclose(new_eye - old_eye, [15, -12, 85], atol=.05)
    assert any(part.startswith('line/link-') for part in result['moved'])


@pytest.mark.parametrize('catalog', PROFILES)
@pytest.mark.parametrize('end', ('start', 'end'))
def test_free_connector_rotation_carries_offset_attachment(factory, blank, catalog, end):
    before, joint = scene(factory, blank, catalog, end)
    old_eye = before.joint_frames(joint)[0]
    after, result = moved_connector(factory, before, joint, rotation_deg=(0, 25, 0), mode='rotate')
    new_eye = after.joint_frames(joint)[0]
    assert np.linalg.norm(new_eye - old_eye) > 10
    assert any(part.startswith('line/link-') for part in result['moved'])


@pytest.mark.parametrize('catalog', PROFILES)
@pytest.mark.parametrize('end', ('start', 'end'))
def test_connector_first_edits_preserve_attachment_over_multiple_steps(factory, blank, catalog, end):
    current, joint = scene(factory, blank, catalog, end, extra_runs=2)
    for shift, angle, mode in [((0, 0, 65), (0, 0, 0), 'translate'),
                               ((0, 0, 0), (0, 18, 0), 'rotate'),
                               ((20, 10, -30), (0, 0, 0), 'translate')]:
        current, _ = moved_connector(factory, current, joint, shift=shift,
                                     rotation_deg=angle, mode=mode)


@pytest.mark.parametrize('catalog', PROFILES)
@pytest.mark.parametrize('end', ('start', 'end'))
def test_connector_then_whole_line_move_preserves_visible_motion_and_attachment(factory, blank, catalog, end):
    current, joint = scene(factory, blank, catalog, end)
    current, _ = moved_connector(factory, current, joint, shift=(0, 0, 90))
    before = {part: body.matrix.copy() for part, body in current.parts.items()}
    target = next(item for item in current.doc['objects'] if item['id'] == 'line')['pose'].copy()
    target['position_mm'] = (np.asarray(target['position_mm']) + [25, -15, 45]).tolist()
    result = move_object(current, 'line', target)
    after = factory(result['document'])
    for part in ('hanger', 'line/link-1', f'line/link-{len([p for p in before if p.startswith("line/link-")])}'):
        assert np.allclose(after.parts[part].matrix[:3, 3] - before[part][:3, 3],
                           [25, -15, 45], atol=.5), (part, result)
    assert_attachment(after, joint)


@pytest.mark.parametrize('end', ('start', 'end'))
def test_draft_connected_connector_carries_pipe_and_line_through_both_edits(factory, blank, end):
    current, joint = scene(factory, blank, end=end, draft_connected=True)
    pipe = lambda assembly: next(run for group in assembly.doc['draft_subassemblies']
                                 for run in group['runs'] if run['id'] == 'hanger-pipe')
    original_start = np.asarray(pipe(current)['start_mm'])
    current, _ = moved_connector(factory, current, joint, shift=(0, 0, 80))
    assert np.allclose(np.asarray(pipe(current)['start_mm']) - original_start, [0, 0, 80], atol=.05)
    target = next(item for item in current.doc['objects'] if item['id'] == 'line')['pose'].copy()
    target['position_mm'] = (np.asarray(target['position_mm']) + [15, 0, 40]).tolist()
    result = move_object(current, 'line', target)
    after = factory(result['document'])
    assert np.allclose(np.asarray(pipe(after)['start_mm']) - original_start, [15, 0, 120], atol=.05)
    assert_attachment(after, joint)


@pytest.mark.parametrize('end', ('start', 'end'))
def test_world_anchored_connector_rejects_motion_without_moving_chain(factory, blank, end):
    before, joint = scene(factory, blank, end=end, anchored=True)
    old = {part: body.matrix.copy() for part, body in before.parts.items()}
    target = before.parts['hanger'].matrix.copy()
    target[:3, 3] += [0, 0, 75]
    try:
        result = transform_part(before, 'hanger', pose_of(target))
    except DocumentError:
        return
    after = factory(result['document'])
    assert all(np.allclose(body.matrix, old[part], atol=1e-6, rtol=0)
               for part, body in after.parts.items())
    assert_attachment(after, joint)


def test_randomized_connector_first_sequences_keep_flexible_line_attached(factory, blank):
    seed = int(os.environ.get('PIPESIM_CHAIN_ATTACHMENT_SEED') or secrets.randbits(64))
    count = int(os.environ.get('PIPESIM_CHAIN_ATTACHMENT_CASES', '8'))
    rng = random.Random(seed)
    for case in range(count):
        catalog = rng.choice(PROFILES)
        end = rng.choice(('start', 'end'))
        links = rng.randint(7, 18)
        draft_connected = rng.choice((False, True))
        current, joint = scene(factory, blank, catalog, end,
                               link_count=links, extra_runs=rng.randint(0, 3),
                               draft_connected=draft_connected)
        try:
            for _ in range(rng.randint(2, 4)):
                if rng.random() < .25:
                    shift = [rng.uniform(-40, 40) for _ in range(3)]
                    old_hanger = current.parts['hanger'].matrix[:3, 3].copy()
                    target = next(item for item in current.doc['objects']
                                  if item['id'] == 'line')['pose'].copy()
                    target['position_mm'] = (np.asarray(target['position_mm']) + shift).tolist()
                    result = move_object(current, 'line', target)
                    current = factory(result['document'])
                    assert np.allclose(current.parts['hanger'].matrix[:3, 3] - old_hanger,
                                       shift, atol=.5)
                    assert_attachment(current, joint)
                elif rng.choice((False, True)):
                    shift = [rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(-50, 100)]
                    angle, mode = (0, 0, 0), 'translate'
                    current, _ = moved_connector(factory, current, joint,
                                                 shift=shift, rotation_deg=angle, mode=mode)
                else:
                    shift = (0, 0, 0)
                    angle = [rng.uniform(-20, 20) for _ in range(3)]
                    mode = 'rotate'
                    current, _ = moved_connector(factory, current, joint,
                                                 shift=shift, rotation_deg=angle, mode=mode)
        except Exception as error:
            pytest.fail(f'chain attachment seed={seed} case={case} '
                        f'catalog={catalog} end={end} links={links} '
                        f'draft_connected={draft_connected}: {error}')
