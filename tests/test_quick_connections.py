import copy
import random

from pipesim.document import Assembly
from pipesim.drafting import connect
from pipesim.grouping import move_object
from pipesim.quick_connections import attach_nearby, attach_similar, attached_connector_ports, detach_all


def fitting(identifier, position, axis):
    return {'id': identifier, 'body': {'kind': 'connector', 'mass_kg': .2,
            'geometry': [{'type': 'sphere', 'radius_mm': 6}],
            'ports': {'socket': {'type': 'socket', 'position_mm': [0, 0, 0],
                'axis': axis, 'diameter_mm': 42.4, 'profile': 'round',
                'engagement_mm': 40, 'min_engagement_mm': 15}}},
            'pose': {'position_mm': position}}


def draft_design():
    return {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': 'Quick draft',
            'parts': [fitting('left', [0, 0, 100], [1, 0, 0]),
                      fitting('right', [1000, 0, 100], [-1, 0, 0])],
            'joints': [], 'draft_subassemblies': [{'id': 'frame', 'runs': [
                {'id': 'rail', 'catalog': 'tubeclamp.tube-C',
                 'start_mm': [0, 0, 100], 'end_mm': [1000, 0, 100]}]}]}


def test_draft_connector_and_run_detach_then_attach_all_nearby():
    doc = draft_design()
    doc = connect(Assembly.from_doc(doc), 'rail', 'left', 'socket', 'start', 20)
    doc = connect(Assembly.from_doc(doc), 'rail', 'right', 'socket', 'end', 20)
    initial = copy.deepcopy(doc)
    detached = detach_all(Assembly.from_doc(doc), 'left')
    assert detached['count'] == 1
    run = detached['document']['draft_subassemblies'][0]['runs'][0]
    assert [a['connector'] for a in run['attachments']] == ['right']
    detached['document']['parts'][0]['pose']['position_mm'][0] += 5
    reattached = attach_nearby(Assembly.from_doc(detached['document']), 'left')
    assert reattached['count'] == 1
    assert {a['connector'] for a in reattached['document']['draft_subassemblies'][0]['runs'][0]['attachments']} == {'left', 'right'}
    assert next(a for a in reattached['document']['draft_subassemblies'][0]['runs'][0]['attachments']
                if a['connector'] == 'left')['insertion_mm'] == 20
    assert reattached['document']['parts'][0]['pose']['position_mm'][0] == initial['parts'][0]['pose']['position_mm'][0] + 5
    detached = detach_all(Assembly.from_doc(reattached['document']), 'rail')
    assert detached['count'] == 2
    assert not detached['document']['draft_subassemblies'][0]['runs'][0]['attachments']
    reattached = attach_nearby(Assembly.from_doc(detached['document']), 'rail')
    assert reattached['count'] == 2
    Assembly.from_doc(reattached['document'])


def test_exact_member_reconnects_two_sockets_without_moving_parts():
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': 'Exact length',
           'parts': [{'id': 'rail', 'catalog': 'tubeclamp.tube-C',
                      'parameters': {'length_mm': 1000}, 'pose': {'position_mm': [0, 0, 500]}},
                     fitting('low', [0, 0, 20], [0, 0, 1]),
                     fitting('high', [0, 0, 980], [0, 0, -1])],
           'joints': []}
    original_parts = copy.deepcopy(doc['parts'])
    attached = attach_nearby(Assembly.from_doc(doc), 'rail')
    assert attached['count'] == 2
    assert attached['document']['parts'] == original_parts
    detached = detach_all(Assembly.from_doc(attached['document']), 'rail')
    assert detached['count'] == 2
    assert not detached['document']['joints']
    restored = attach_nearby(Assembly.from_doc(detached['document']), 'rail')
    assert restored['count'] == 2
    assert restored['document']['parts'] == original_parts


def test_exact_shortcut_does_not_save_a_skew_joint_from_visual_proximity():
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': 'Offset length',
           'parts': [{'id': 'rail', 'catalog': 'tubeclamp.tube-C',
                      'parameters': {'length_mm': 1000}, 'pose': {'position_mm': [5, 0, 500]}},
                     fitting('low', [0, 0, 20], [0, 0, 1])],
           'joints': []}
    result = attach_nearby(Assembly.from_doc(doc), 'rail')
    assert result['count'] == 0
    assert result['document'] == doc


def test_chain_detaches_only_external_mount_and_restores_its_joint():
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': 'Chain mount',
           'parts': [], 'joints': [], 'objects': [{'id': 'line', 'template': 'chain',
               'parameters': {'length_mm': 60}, 'pose': {'position_mm': [0, 0, 200]}}]}
    chain = Assembly.from_doc(doc)
    point, axis = chain.parts['line/link-1'].frame({'port': 'b'})
    doc['parts'].append({'id': 'eye', 'body': {'kind': 'connector', 'mass_kg': .1,
        'geometry': [{'type': 'sphere', 'radius_mm': 5}],
        'ports': {'ring': {'type': 'eye', 'assembly': 'hook',
                           'position_mm': [0, 0, 0], 'axis': axis.tolist()}}},
        'pose': {'position_mm': point.tolist()}})
    doc['joints'].append({'id': 'mount', 'type': 'spherical',
                          'a': {'part': 'line/link-1', 'port': 'b'},
                          'b': {'part': 'eye', 'port': 'ring'}})
    before = Assembly.from_doc(doc)
    internal_count = len(before.joints) - 1
    detached = detach_all(before, 'line/link-1')
    assert detached['count'] == 1
    assert not detached['document']['joints']
    assert len(Assembly.from_doc(detached['document']).joints) == internal_count
    target = copy.deepcopy(detached['document']['objects'][0]['pose'])
    target['position_mm'][1] += 5
    moved = move_object(Assembly.from_doc(detached['document']), 'line', target)['document']
    attached = attach_nearby(Assembly.from_doc(moved), 'line/link-1')
    assert attached['count'] == 1
    assert attached['document']['joints'][0]['id'] == 'mount'
    assert attached['document']['objects'][0]['pose']['position_mm'][1] == doc['objects'][0]['pose']['position_mm'][1]
    assert len(Assembly.from_doc(attached['document']).joints) == internal_count + 1
    detached = detach_all(Assembly.from_doc(attached['document']), 'line/link-1')['document']
    new_eye = copy.deepcopy(detached['parts'][0])
    new_eye['id'] = 'new-eye'
    new_eye['pose']['position_mm'][0] += 100
    detached['parts'].append(new_eye)
    target = copy.deepcopy(detached['objects'][0]['pose'])
    target['position_mm'][0] += 105
    moved = move_object(Assembly.from_doc(detached), 'line', target)['document']
    reconnected = attach_nearby(Assembly.from_doc(moved), 'line/link-1')
    assert reconnected['count'] == 1
    assert {end['part'] for end in reconnected['document']['joints'][0].values()
            if isinstance(end, dict) and 'part' in end} == {'line/link-1', 'new-eye'}
    assert reconnected['document']['objects'][0]['pose']['position_mm'][0] == doc['objects'][0]['pose']['position_mm'][0] + 100
    assert not reconnected['document'].get('metadata', {}).get('quick_detached'), 'old mount is no longer offered after reconnecting elsewhere'


def test_two_nearby_bolt_port_pairs_join_in_one_action():
    def part(identifier, ports):
        return {'id': identifier, 'body': {'kind': 'connector', 'mass_kg': .1,
            'geometry': [{'type': 'sphere', 'radius_mm': 5}], 'ports': ports}}
    def bolt(kind, x):
        return {'type': kind, 'assembly': 'bolt', 'diameter_mm': 12,
                'position_mm': [x, 0, 0], 'axis': [0, 0, 1]}
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': 'Two bolts',
           'parts': [part('hub', {'left': bolt('eye', -100), 'right': bolt('eye', 100)}),
                     part('eye-only', {'hole': bolt('eye', -100)}),
                     part('left', {'hole': bolt('clevis', -100)}),
                     part('right', {'hole': bolt('clevis', 100)})], 'joints': []}
    attached = attach_nearby(Assembly.from_doc(doc), 'hub')
    assert attached['count'] == 2
    assert {j['type'] for j in attached['document']['joints']} == {'revolute'}
    assert not any(end['part'] == 'eye-only' for joint in attached['document']['joints']
                   for end in (joint['a'], joint['b']))
    detached = detach_all(Assembly.from_doc(attached['document']), 'hub')
    assert detached['count'] == 2
    assert attach_nearby(Assembly.from_doc(detached['document']), 'hub')['count'] == 2


def test_draft_through_socket_reattaches_at_its_new_station():
    doc = draft_design()
    doc['parts'] = [{'id': 'hook', 'body': {'kind': 'connector', 'mass_kg': .2,
        'geometry': [{'type': 'sphere', 'radius_mm': 8}],
        'ports': {'bore': {'type': 'socket', 'through': True, 'profile': 'round',
                           'diameter_mm': 42.4, 'engagement_mm': 40,
                           'position_mm': [0, 0, 0], 'axis': [1, 0, 0]}}},
        'pose': {'position_mm': [500, 0, 100]}}]
    doc['draft_subassemblies'][0]['runs'][0]['attachments'] = [
        {'connector': 'hook', 'port': 'bore'}]
    detached = detach_all(Assembly.from_doc(doc), 'hook')
    assert detached['count'] == 1
    detached['document']['parts'][0]['pose']['position_mm'][0] += 5
    attached = attach_nearby(Assembly.from_doc(detached['document']), 'hook')
    assert attached['count'] == 1
    attachment = attached['document']['draft_subassemblies'][0]['runs'][0]['attachments'][0]
    assert attachment == {'connector': 'hook', 'port': 'bore'}
    Assembly.from_doc(attached['document'])


def test_seeded_draft_shortcut_reconnects_small_layout_offsets():
    rng = random.Random(737)
    for seed in range(12):
        doc = draft_design()
        run = doc['draft_subassemblies'][0]['runs'][0]
        offset = [rng.uniform(-8, 8), rng.uniform(-4, 4), rng.uniform(-4, 4)]
        run['start_mm'] = [a + b for a, b in zip(run['start_mm'], offset)]
        run['end_mm'] = [a + b for a, b in zip(run['end_mm'], offset)]
        result = attach_nearby(Assembly.from_doc(doc), 'rail')
        assert result['count'] == 2, (seed, offset)
        Assembly.from_doc(result['document'])


def test_similar_socket_action_attaches_only_matching_open_nearby_ports():
    doc = draft_design()
    doc['parts'] = []
    doc['draft_subassemblies'][0]['runs'] = []
    for index in range(6):
        y = index * 120
        doc['parts'].append(fitting(f'left-{index}', [0, y, 100], [1, 0, 0]))
        if index < 5:
            doc['draft_subassemblies'][0]['runs'].append({'id': f'rail-{index}',
                'catalog': 'tubeclamp.tube-C', 'start_mm': [0, y, 100],
                'end_mm': [1000, y, 100]})
    doc = connect(Assembly.from_doc(doc), 'rail-1', 'left-1', 'socket', 'start', 20)
    first = attach_nearby(Assembly.from_doc(doc), 'left-0')
    assert first['count'] == 1
    assert attached_connector_ports(doc, first['document'], 'left-0') == [('left-0', 'socket')]
    assembly = Assembly.from_doc(first['document'])
    preview = attach_similar(assembly, 'left-0', 'socket')
    assert preview['count'] == 3
    assert preview['candidates'] == ['left-2', 'left-3', 'left-4']
    assert 'document' not in preview
    applied = attach_similar(assembly, 'left-0', 'socket', apply=True)
    assert applied['count'] == 3
    assert applied['document']['draft_subassemblies'][0]['runs'][1]['attachments'] == [
        {'connector': 'left-1', 'port': 'socket', 'end': 'start', 'insertion_mm': 20}]
    for index in (0, 2, 3, 4):
        assert [item['connector'] for item in applied['document']['draft_subassemblies'][0]['runs'][index]['attachments']] == [f'left-{index}']
    assert attach_similar(Assembly.from_doc(applied['document']), 'left-0', 'socket')['count'] == 0


def test_similar_action_handles_finalized_sockets_and_bolt_ports():
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': 'Repeated mounts',
           'parts': [], 'joints': []}
    for index in range(3):
        y = index * 150
        doc['parts'].extend([
            {'id': f'rail-{index}', 'catalog': 'tubeclamp.tube-C',
             'parameters': {'length_mm': 1000}, 'pose': {'position_mm': [0, y, 500]}},
            fitting(f'socket-{index}', [0, y, 20], [0, 0, 1]),
        ])
    first = attach_nearby(Assembly.from_doc(doc), 'socket-0')
    assert first['count'] == 1
    rest = attach_similar(Assembly.from_doc(first['document']), 'socket-0', 'socket', apply=True)
    assert rest['count'] == 2
    assert len(rest['document']['joints']) == 3
    Assembly.from_doc(rest['document'])

    def bolt(identifier, kind, y):
        return {'id': identifier, 'body': {'kind': 'connector', 'mass_kg': .1,
            'geometry': [{'type': 'sphere', 'radius_mm': 5}],
            'ports': {'hole': {'type': kind, 'assembly': 'bolt', 'diameter_mm': 12,
                               'position_mm': [0, 0, 0], 'axis': [0, 0, 1]}}},
            'pose': {'position_mm': [0, y, 100]}}
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg', 'name': 'Repeated hinges',
           'parts': [part for i in range(3) for part in
                     (bolt(f'eye-{i}', 'eye', i * 150), bolt(f'clevis-{i}', 'clevis', i * 150))],
           'joints': []}
    first = attach_nearby(Assembly.from_doc(doc), 'eye-0')
    assert first['count'] == 1
    rest = attach_similar(Assembly.from_doc(first['document']), 'eye-0', 'hole', apply=True)
    assert rest['count'] == 2
    assert {joint['type'] for joint in rest['document']['joints']} == {'revolute'}
