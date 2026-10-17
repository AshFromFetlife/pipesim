"""Presentation can combine loose-part placement without hiding assembly work."""
import copy
import json
import os
import random
import secrets

import pytest

from pipesim.build_presentation import instruction_steps


def _assembly(factory, *, mass=1):
    return factory({'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
        'parts': [
            {'id': 'fitting', 'catalog': 'tubeclamp.TC116C', 'mass_kg': mass},
            {'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
             'parameters': {'length_mm': 173.875}, 'mass_kg': 1},
            {'id': 'custom', 'body': {'kind': 'rigid', 'name': 'Corner plate',
             'mass_kg': 1, 'geometry': [{'type': 'box', 'size_mm': [20, 30, 10]}]}},
        ], 'joints': []})


def _pair(pid='fitting', *, numbers=(17, 29), context='subassembly-1', floor=0):
    start = {'position_mm': [0, 0, floor+40], 'rotation_deg': [0, 0, 0]}
    end = {'position_mm': [100, 50, floor+40], 'rotation_deg': [0, 0, 90]}
    initial = {'number': numbers[0], 'kind': 'position', 'part': pid,
        'parts': [pid], 'introduced_parts': [pid], 'moving_parts': [pid],
        'target_parts': [], 'installed_parts': [pid], 'subassembly': context,
        'instruction': 'Place the loose part on the floor.',
        'pose': copy.deepcopy(start), 'poses': {pid: copy.deepcopy(start)},
        'floor_z_mm': floor, 'grounded_after': True, 'group_mass_kg': 1,
        'manual_support': False, 'held_parts': [], 'fasten': [], 'fixture': None,
        'world_anchor': None, 'world_anchors': [], 'prepared_subassembly': None,
        'approach_direction': [0, 0, 1], 'start_offset_mm': [0, 0, 0]}
    lifting = copy.deepcopy(initial)
    lifting.update(number=numbers[1], kind='lift', parts=[], introduced_parts=[],
        instruction='Lift the loose part and turn it into its working pose.',
        pose=copy.deepcopy(end), poses={pid: copy.deepcopy(end)},
        start_poses={pid: copy.deepcopy(start)},
        motion_waypoints=[{pid: {'position_mm': [0, 0, floor+300],
                                 'rotation_deg': [0, 0, 90]}}])
    return [initial, lifting]


def _plan(steps):
    return {'status': 'buildable', 'steps': steps,
            'certificate': {'algorithm_version': 3, 'sentinel': ['unchanged']}}


def _identity_presentation(assembly, plan):
    """Negative control: the old book gives every raw action its own page."""
    result = copy.deepcopy(plan['steps'])
    for number, step in enumerate(result, 1):
        step.update(source_steps=[step['number']], image_number=step['number'], number=number)
    return result


def _check(assembly, plan, expected_groups, presenter=instruction_steps):
    original = copy.deepcopy(plan)
    serialized = json.dumps(plan, sort_keys=True)
    shown = presenter(assembly, plan)
    assert len(shown) == len(expected_groups), 'loose singleton placement still has a redundant page'
    assert [step['source_steps'] for step in shown] == expected_groups
    assert [step['number'] for step in shown] == list(range(1, len(shown)+1))
    assert [number for step in shown for number in step['source_steps']] == [
        step['number'] for step in plan['steps']]
    raw = {step['number']: step for step in plan['steps']}
    for step, sources in zip(shown, expected_groups):
        final = raw[sources[-1]]
        assert step['image_number'] == sources[-1]
        assert step['poses'] == final['poses']
        assert step['pose'] == final['pose']
        if len(sources) == 2:
            assert step['kind'] == 'position'
            assert step['introduced_parts'] == raw[sources[0]]['introduced_parts']
            assert step['parts'] == raw[sources[0]]['parts']
            assert step['start_poses'] == final['start_poses']
            assert step['motion_waypoints'] == final['motion_waypoints']
        else:
            restored = copy.deepcopy(step)
            restored['number'] = sources[0]
            restored.pop('source_steps')
            restored.pop('image_number')
            assert restored == final
    assert plan == original
    assert json.dumps(plan, sort_keys=True) == serialized
    # Display edits must not alias either the verified input or another call.
    if shown:
        pid = next(iter(shown[0]['poses']))
        shown[0]['poses'][pid]['position_mm'][0] += 12345
        shown[0]['source_steps'].append(-1)
        assert plan == original
    return presenter(assembly, plan)


@pytest.mark.parametrize('pid,title', [('fitting', 'Position TC116C'),
                                     ('pipe', 'Position pipe'),
                                     ('custom', 'Position Corner plate')])
def test_initial_loose_part_uses_one_page_at_its_final_working_pose(factory, pid, title):
    assembly = _assembly(factory)
    shown = _check(assembly, _plan(_pair(pid)), [[17, 29]])
    assert shown[0]['display_title'] == title
    assert 'on the floor in the shown orientation' in shown[0]['instruction']
    assert pid in shown[0]['instruction']
    if pid == 'pipe':
        assert '173.875 mm' in shown[0]['instruction']


UNSAFE_FIELDS = [
    ('group_mass_kg', 15.0001), ('helper_required', True),
    ('assistance_required', True), ('fixture', {'type': 'stand'}),
    ('fasten', [{'joint': 'socket-1'}]), ('world_anchor', {'part': 'fitting'}),
    ('world_anchors', [{'part': 'fitting'}]), ('target_parts', ['pipe']),
    ('manual_support', True), ('held_parts', ['fitting']),
    ('prepared_subassembly', 'subassembly-2'),
]


@pytest.mark.parametrize('side', [0, 1], ids=['initial', 'lift'])
@pytest.mark.parametrize('field,value', UNSAFE_FIELDS, ids=[item[0] for item in UNSAFE_FIELDS])
def test_handling_and_assembly_requirements_keep_both_pages(factory, side, field, value):
    pair = _pair()
    pair[side][field] = copy.deepcopy(value)
    _check(_assembly(factory), _plan(pair), [[17], [29]])


@pytest.mark.parametrize('reason', ['multipart', 'context', 'not-grounded', 'floor',
    'different-moving-part', 'not-introduced', 'initial-kind', 'following-kind',
    'unknown-part', 'not-adjacent'])
def test_only_an_immediate_matching_grounded_singleton_reorientation_is_combined(factory, reason):
    pair = _pair()
    if reason == 'multipart':
        for step in pair:
            step['moving_parts'] = ['fitting', 'pipe']
        pair[0]['introduced_parts'] = ['fitting', 'pipe']
    elif reason == 'context':
        pair[1]['subassembly'] = 'main'
    elif reason == 'not-grounded':
        pair[1]['grounded_after'] = False
    elif reason == 'floor':
        pair[1]['floor_z_mm'] = 10
    elif reason == 'different-moving-part':
        pair[1]['moving_parts'] = ['pipe']
    elif reason == 'not-introduced':
        pair[0]['introduced_parts'] = []
    elif reason == 'initial-kind':
        pair[0]['kind'] = 'insert'
    elif reason == 'following-kind':
        pair[1]['kind'] = 'merge'
    elif reason == 'unknown-part':
        pair = _pair('missing')
    elif reason == 'not-adjacent':
        barrier = copy.deepcopy(pair[0])
        barrier.update(number=23, kind='insert', introduced_parts=[])
        pair.insert(1, barrier)
    _check(_assembly(factory), _plan(pair), [[step['number']] for step in pair])


@pytest.mark.parametrize('mass,merged', [(15, True), (15.0001, False)])
def test_actual_part_mass_and_reported_mass_both_respect_handling_boundary(factory, mass, merged):
    assembly = _assembly(factory, mass=mass)
    # Do not trust a low group_mass_kg if the resolved part itself is heavy.
    _check(assembly, _plan(_pair()), [[17, 29]] if merged else [[17], [29]])
    pair = _pair()
    for step in pair:
        step['group_mass_kg'] = mass
    _check(_assembly(factory), _plan(pair), [[17, 29]] if merged else [[17], [29]])


def test_empty_and_unpaired_final_steps_are_retained(factory):
    assembly = _assembly(factory)
    assert instruction_steps(assembly, {}) == []
    _check(assembly, _plan(_pair()[:1]), [[17]])


PRESENTATION_SEED = int(os.environ.get('PIPESIM_PRESENTATION_FUZZ_SEED', secrets.randbits(64)))
PRESENTATION_CASES = int(os.environ.get('PIPESIM_PRESENTATION_FUZZ_CASES', 160))


def _mixed_case(rng, index):
    steps, groups = [], []
    next_number = rng.randrange(1, 100)
    for item in range(rng.randrange(3, 15)):
        pid = rng.choice(['fitting', 'pipe', 'custom'])
        numbers = [next_number, next_number+rng.randrange(1, 9)]
        next_number = numbers[-1]+rng.randrange(1, 9)
        pair = _pair(pid, numbers=numbers, context=f'preparation-{index}-{item}',
                     floor=rng.uniform(-500, 500))
        for step in pair:
            step['poses'][pid]['position_mm'][:2] = [rng.uniform(-5000, 5000) for _ in range(2)]
            step['poses'][pid]['rotation_deg'] = [rng.uniform(-180, 180) for _ in range(3)]
            step['pose'] = copy.deepcopy(step['poses'][pid])
        pair[1]['start_poses'] = copy.deepcopy(pair[0]['poses'])
        # Every case includes a positive witness, so identity-only display is
        # guaranteed to fail instead of passing a random all-unsafe sample.
        unsafe = None if item == 0 else rng.choice([None, *range(len(UNSAFE_FIELDS)),
                                                   'context', 'floor', 'not-grounded', 'multipart'])
        if isinstance(unsafe, int):
            field, value = UNSAFE_FIELDS[unsafe]
            pair[rng.randrange(2)][field] = copy.deepcopy(value)
        elif unsafe == 'context':
            pair[1]['subassembly'] = 'main'
        elif unsafe == 'floor':
            pair[1]['floor_z_mm'] += 100
        elif unsafe == 'not-grounded':
            pair[1]['grounded_after'] = False
        elif unsafe == 'multipart':
            for step in pair:
                step['moving_parts'] = [pid, 'other']
            pair[0]['introduced_parts'] = [pid, 'other']
        groups.extend([numbers] if unsafe is None else [[number] for number in numbers])
        steps.extend(pair)
    return _plan(steps), groups


def test_seeded_mixed_presentation_fuzz_preserves_every_raw_action(factory, tmp_path):
    assembly = _assembly(factory)
    rng = random.Random(PRESENTATION_SEED)
    print(f'build presentation seed={PRESENTATION_SEED}; cases={PRESENTATION_CASES}', flush=True)
    for index in range(PRESENTATION_CASES):
        plan, expected = _mixed_case(rng, index)
        try:
            _check(assembly, plan, expected)
        except AssertionError:
            repro = tmp_path / f'presentation-{PRESENTATION_SEED}-{index}.json'
            repro.write_text(json.dumps({'plan': plan, 'expected_source_groups': expected}, indent=2))
            pytest.fail(f'seed={PRESENTATION_SEED}; case={index}; repro={repro}')


def test_identity_only_negative_control_fails_the_regression_and_fuzz_oracles(factory):
    assembly = _assembly(factory)
    with pytest.raises(AssertionError, match='redundant page'):
        _check(assembly, _plan(_pair()), [[17, 29]], _identity_presentation)
    rng = random.Random(730442)
    for index in range(12):
        plan, expected = _mixed_case(rng, index)
        with pytest.raises(AssertionError, match='redundant page'):
            _check(assembly, plan, expected, _identity_presentation)
