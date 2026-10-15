"""Cuts, splices and slack edits retain the referenced physical links."""
import copy
import random

import numpy as np
import pytest

from pipesim.chain import _fit_span, dimensions, generate
from pipesim.chain_edit import adjust_span, cut, resize_free_end, splice
from pipesim.document import Assembly
from pipesim.math3d import align_axis, pose_of
from pipesim.resize_drag import resize_drag


def line(factory, blank, library, count=12, profile='generic.rope-jute-6', curved=False):
    doc = copy.deepcopy(blank)
    parameters = {'length_mm': count*20, 'link_catalog': profile}
    obj = {'id': 'line', 'template': 'chain', 'parameters': parameters,
           'layout_mode': 'posable', 'pose': {'position_mm': [0, 0, 500]}}
    if curved:
        info = dimensions(parameters, library)
        components = generate(parameters, library)
        nodes = _fit_span(np.linspace([0., 0., 0.], [120., 0., 0.], count+1),
                          np.full(count, info['pitch_mm']), np.zeros(3),
                          np.array([120., 0., 0.]))
        for index, part in enumerate(components['parts']):
            matrix = np.eye(4)
            matrix[:3, :3] = align_axis(info['a']-info['b'], nodes[index+1]-nodes[index])
            matrix[:3, 3] = nodes[index]-matrix[:3, :3]@info['b']
            part['pose'] = pose_of(matrix)
        obj['components'] = components
    doc['objects'] = [obj]
    if curved:
        doc['anchors'] = [{'part': 'line/link-1', 'surface': 'fixture'},
                          {'part': f'line/link-{count}', 'surface': 'fixture'}]
    return factory(doc)


def aligned(assembly):
    for joint in assembly.joints:
        a, b, _ = assembly.joint_frames(joint)
        assert np.linalg.norm(a-b) < 1e-4, joint['id']


def test_cut_selected_link_preserves_both_pieces_and_external_references(factory, blank, library):
    before = line(factory, blank, library)
    before.doc['anchors'] = [{'part': 'line/link-2', 'surface': 'fixture'},
                             {'part': 'line/link-10', 'surface': 'fixture'}]
    before = factory(before.doc)
    original = copy.deepcopy(before.doc)
    result = cut(before, 'line', 6)
    after = factory(result['document'])
    tail = result['tail']
    assert len(after.doc['objects']) == 2
    assert len(after.parts) == 12 and len(after.joints) == 10
    assert after.anchors[1]['part'] == tail+'/link-4'
    for old in range(1, 13):
        new = f'line/link-{old}' if old <= 6 else f'{tail}/link-{old-6}'
        assert np.allclose(before.parts[f'line/link-{old}'].matrix,
                           after.parts[new].matrix, atol=1e-6)
    assert before.doc == original
    aligned(after)


def test_splice_cut_line_restores_one_compact_line(factory, blank, library):
    before = line(factory, blank, library)
    split = factory(cut(before, 'line', 5)['document'])
    tail_id = next(o['id'] for o in split.doc['objects'] if o['id'] != 'line')
    result = splice(split, 'line', tail_id)
    after = factory(result['document'])
    assert len(after.doc['objects']) == 1
    assert len(after.parts) == 12 and len(after.joints) == 11
    assert result['message'].endswith('0 bridging segment(s).')
    for i in range(1, 13):
        assert np.allclose(before.parts[f'line/link-{i}'].matrix,
                           after.parts[f'line/link-{i}'].matrix, atol=1e-6)
    aligned(after)


def test_splice_short_gap_adds_bendable_bridge(factory, blank, library):
    before = line(factory, blank, library)
    split = cut(before, 'line', 5)['document']
    tail = next(o for o in split['objects'] if o['id'] != 'line')
    tail['pose']['position_mm'][0] += 10
    separated = factory(split)
    result = splice(separated, 'line', tail['id'])
    after = factory(result['document'])
    assert len(after.parts) == 14
    aligned(after)


@pytest.mark.parametrize('first_end,second_end',
                         [('end','start'), ('start','start'), ('end','end'), ('start','end')])
def test_splice_all_end_pairings_preserves_attachment_ports(
        factory, blank, library, first_end, second_end):
    doc = copy.deepcopy(blank)
    doc['objects'] = [
        {'id': name, 'template': 'chain',
         'parameters': {'length_mm': 100, 'link_catalog': 'generic.rope-jute-6'},
         'pose': {'position_mm': [x, 0, 500]}}
        for name,x in [('left',0),('right',200)]]
    initial = factory(doc)
    def endpoint(assembly, name, end):
        return assembly.parts[f'{name}/link-{1 if end == "start" else 5}'].frame(
            {'port': 'b' if end == 'start' else 'a'})[0]
    offset = endpoint(initial, 'left', first_end)-endpoint(initial, 'right', second_end)
    doc['objects'][1]['pose']['position_mm'] = (np.array(doc['objects'][1]['pose']['position_mm'])+offset).tolist()
    positioned = factory(doc)
    attached = [('left',2,'b'),('right',4,'a')]
    for name,index,port in attached:
        position = positioned.parts[f'{name}/link-{index}'].frame({'port':port})[0]
        doc['parts'].append({'id':f'eye-{name}', 'body':{
            'kind':'connector', 'mass_kg':.1,
            'geometry':[{'type':'sphere','radius_mm':5}],
            'ports':{'eye':{'type':'eye','position_mm':[0,0,0],'axis':[1,0,0]}}},
            'pose':{'position_mm':position.tolist()}})
        doc['joints'].append({'id':f'attach-{name}', 'type':'spherical',
                              'a':{'part':f'eye-{name}','port':'eye'},
                              'b':{'part':f'{name}/link-{index}','port':port}})
    before = factory(doc)
    result = splice(before,'left','right',first_end,second_end)
    after = factory(result['document'])
    assert len(after.parts) == 12
    assert len(after.joints) == 11
    assert len(after.doc['objects']) == 1
    aligned(after)
    for joint in after.doc['joints']:
        name = joint['id'].removeprefix('attach-')
        old_point, old_axis = before.parts[f'{name}/link-{2 if name == "left" else 4}'].frame(
            {'port':'b' if name == 'left' else 'a'})
        new_point, new_axis = after.parts[joint['b']['part']].frame(joint['b'])
        assert np.allclose(new_point, old_point, atol=1e-5)
        assert np.allclose(new_axis, old_axis, atol=1e-5)


def test_cut_and_splice_bake_saved_joint_state(factory, blank, library):
    before = line(factory, blank, library)
    before.doc['state'] = {'joints': {'line/join-2': {'rotation_deg': [20, 0, 0]},
                                    'line/join-9': {'rotation_deg': [0, -10, 0]}}}
    before = factory(before.doc)
    split = cut(before, 'line', 6)
    tail = split['tail']
    assert not split['document'].get('state', {}).get('joints')
    for old in range(1, 13):
        new = f'line/link-{old}' if old <= 6 else f'{tail}/link-{old-6}'
        assert np.allclose(before.parts[f'line/link-{old}'].matrix,
                           factory(split['document']).parts[new].matrix, atol=1e-5)
    joined = splice(factory(split['document']), 'line', tail)['document']
    assert not joined.get('state', {}).get('joints')
    aligned(factory(joined))


@pytest.mark.parametrize('change', [-1, 1])
def test_adjust_slack_between_fixed_links_keeps_attachments(factory, blank, library, change):
    before = line(factory, blank, library, curved=True)
    first = before.parts['line/link-1'].matrix.copy()
    last = before.parts['line/link-12'].matrix.copy()
    result = adjust_span(before, 'line', 6, change)
    assert result['changed']
    after = factory(result['document'])
    assert len(after.parts) == 12+change
    assert np.allclose(after.parts['line/link-1'].matrix, first, atol=1e-5)
    assert np.allclose(after.parts[f'line/link-{12+change}'].matrix, last, atol=1e-5)
    assert after.anchors[1]['part'] == f'line/link-{12+change}'
    aligned(after)


def test_adjust_slack_retains_external_joint_frames(factory, blank, library):
    before = line(factory, blank, library, curved=True)
    before.doc['anchors'] = []
    ends = [before.parts['line/link-1'].frame({'port': 'b'})[0],
            before.parts['line/link-12'].frame({'port': 'a'})[0]]
    for number, position in enumerate(ends):
        before.doc['parts'].append({'id': f'eye-{number}', 'body': {
            'kind': 'connector', 'mass_kg': .1,
            'geometry': [{'type': 'sphere', 'radius_mm': 5}],
            'ports': {'eye': {'type': 'eye', 'position_mm': [0, 0, 0], 'axis': [1, 0, 0]}}},
            'pose': {'position_mm': position.tolist()}})
        before.doc['anchors'].append({'part': f'eye-{number}', 'surface': 'fixture'})
    before.doc['joints'] = [
        {'id': f'attach-{number}', 'type': 'spherical',
         'a': {'part': f'eye-{number}', 'port': 'eye'},
         'b': {'part': f'line/link-{1 if number == 0 else 12}',
               'port': 'b' if number == 0 else 'a'}}
        for number in range(2)]
    before = factory(before.doc)
    result = adjust_span(before, 'line', 6, -1)
    assert result['changed']
    after = factory(result['document'])
    assert after.doc['joints'][1]['b']['part'] == 'line/link-11'
    for eye in ('eye-0', 'eye-1'):
        assert np.allclose(after.parts[eye].matrix, before.parts[eye].matrix)
    aligned(after)


def test_taut_span_reports_no_change_instead_of_detaching(factory, blank, library):
    before = line(factory, blank, library)
    before.doc['anchors'] = [{'part': 'line/link-1', 'surface': 'fixture'},
                             {'part': 'line/link-12', 'surface': 'fixture'}]
    before = factory(before.doc)
    result = adjust_span(before, 'line', 6, -1)
    assert not result['changed']
    assert result['document'] == before.doc


def test_start_resize_uses_the_start_free_tail(factory, blank, library):
    before = line(factory, blank, library)
    before.doc['anchors'] = [{'part': 'line/link-6', 'surface': 'fixture'}]
    before = factory(before.doc)
    fixed = before.parts['line/link-6'].matrix.copy()
    end = before.parts['line/link-12'].matrix.copy()
    old_start = before.parts['line/link-1'].frame({'port': 'b'})[0]
    result = resize_drag(before, 'line', 280, 'start')
    after = factory(result['document'])
    assert after.anchors[0]['part'] == 'line/link-8'
    assert np.allclose(after.parts['line/link-8'].matrix, fixed, atol=1e-5)
    assert np.allclose(after.parts['line/link-14'].matrix, end, atol=1e-5)
    assert not np.allclose(after.parts['line/link-1'].frame({'port': 'b'})[0], old_start)
    aligned(after)


def test_free_end_cannot_trim_past_an_attachment(factory, blank, library):
    before = line(factory, blank, library)
    before.doc['anchors'] = [{'part': 'line/link-6', 'surface': 'fixture'}]
    before = factory(before.doc)
    result = resize_free_end(before, 'line', 80, 'start')
    assert not result['changed']
    assert result['document'] == before.doc


@pytest.mark.parametrize('seed', range(12))
def test_fuzz_cut_splice_and_slack_preserve_contiguous_line(factory, blank, library, seed):
    rng = random.Random(seed)
    count = rng.randint(10, 30)
    before = line(factory, blank, library, count=count)
    cut_at = rng.randint(2, count-2)
    split = factory(cut(before, 'line', cut_at)['document'])
    tail = next(o['id'] for o in split.doc['objects'] if o['id'] != 'line')
    joined = factory(splice(split, 'line', tail)['document'])
    assert len(joined.parts) == count
    aligned(joined)
    for change in (1, -1):
        result = adjust_span(joined, 'line', rng.randint(2, count-1), change)
        if result['changed']:
            joined = factory(result['document'])
            aligned(joined)
