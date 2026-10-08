"""Editing a flexible line keeps the connections that define its design."""
import copy

import numpy as np
import pytest

from pipesim.document import Assembly
from pipesim.grouping import attach_part, update_object_parameters
from pipesim.math3d import pose_of
from pipesim.posing import transform_part
from pipesim.resize_drag import resize_drag


def attached_line(factory, blank, catalog='generic.rope-jute-6', length=200):
    doc = copy.deepcopy(blank)
    doc['objects'] = [{'id':'line', 'template':'chain',
                       'parameters':{'length_mm':length, 'link_catalog':catalog},
                       'layout_mode':'posable', 'pose':{'position_mm':[0,0,400]}}]
    doc['parts'] = [{'id':name, 'catalog':'generic.d-link',
                     'pose':{'position_mm':[0,0,height]}}
                    for name,height in [('support',400),('weight',400-length)]]
    count = round(length / (25 if catalog == 'generic.rope-nylon-10' else 20))
    doc['joints'] = [
        {'id':'top', 'type':'spherical', 'a':{'part':'support','port':'eye'},
         'b':{'part':'line/link-1','port':'b'}},
        {'id':'bottom', 'type':'spherical', 'a':{'part':'weight','port':'eye'},
         'b':{'part':f'line/link-{count}','port':'a'}}]
    doc['anchors'] = [{'part':'support','surface':'fixture'}]
    return factory(doc)


def assert_attached(assembly):
    for joint in assembly.doc['joints']:
        a,b,_ = assembly.joint_frames(joint)
        assert np.linalg.norm(a-b) < .05, joint['id']


@pytest.mark.parametrize('catalog,count', [('generic.rope-nylon-10',8),
                                           ('generic.strap-seatbelt-65',8),
                                           ('generic.chain-heavy-100',3)])
def test_change_profile_keeps_existing_attachment_ids_and_support(factory, blank, catalog, count):
    before = attached_line(factory, blank)
    doc = update_object_parameters(before, 'line',
                                   {'length_mm':200, 'link_catalog':catalog})
    after = factory(doc)
    assert [j['id'] for j in doc['joints']] == ['top','bottom']
    assert len([p for p in after.parts if p.startswith('line/')]) == count
    assert all(after.parts[f'line/link-{i}'].spec['catalog']==catalog
               for i in range(1,count+1))
    assert doc['anchors'] == before.doc['anchors']
    assert_attached(after)


def test_shrink_line_moves_free_attached_weight_and_preserves_both_joints(factory, blank):
    before = attached_line(factory, blank)
    old_weight = before.parts['weight'].matrix[:3,3].copy()
    doc = update_object_parameters(before, 'line',
                                   {'length_mm':120, 'link_catalog':'generic.rope-jute-6'})
    after = factory(doc)
    assert [j['id'] for j in doc['joints']] == ['top','bottom']
    assert next(j for j in doc['joints'] if j['id']=='bottom')['b']['part']=='line/link-6'
    assert np.allclose(after.parts['weight'].matrix[:3,3]-old_weight, [0,0,80], atol=.05)
    assert_attached(after)


def test_shrink_moves_structure_connected_to_the_attached_weight(factory, blank):
    before = attached_line(factory, blank)
    before.doc['parts'].append({'id':'pendant','catalog':'generic.d-link',
                                'pose':{'position_mm':[0,0,200]}})
    before.doc['joints'].append({'id':'weight-pendant','type':'fixed',
                                 'a':{'part':'weight','port':'eye'},
                                 'b':{'part':'pendant','port':'eye'}})
    before = factory(before.doc)
    after = factory(update_object_parameters(before, 'line',
                    {'length_mm':120, 'link_catalog':'generic.rope-jute-6'}))
    assert np.allclose(after.parts['pendant'].matrix[:3,3], [0,0,280], atol=.05)
    assert_attached(after)


def test_change_profile_resamples_a_saved_bend(factory, blank):
    before = attached_line(factory, blank)
    before.doc['joints'] = []
    before = factory(before.doc)
    target = pose_of(before.parts['line/link-10'].matrix)
    target['position_mm'][0] += 20
    bent = factory(transform_part(before, 'line/link-10', target)['document'])
    assert 'components' in bent.doc['objects'][0]
    after = factory(update_object_parameters(bent, 'line',
                    {'length_mm':200, 'link_catalog':'generic.rope-nylon-10'}))
    assert len([p for p in after.parts if p.startswith('line/')]) == 8
    for joint in after.joints:
        a,b,_ = after.joint_frames(joint)
        assert np.linalg.norm(a-b) < .05, joint['id']
    assert after.parts['line/link-8'].matrix[0,3] > 1


def test_dragging_attached_free_end_shorter_keeps_its_fitting(factory, blank):
    before = attached_line(factory, blank)
    result = resize_drag(before, 'line', 120, 'end')
    after = factory(result['document'])
    assert result['status'] == 'resized'
    assert_attached(after)
    assert np.allclose(after.parts['weight'].matrix[:3,3], [0,0,280], atol=.05)


def test_shorter_line_bends_between_two_fixed_supports_when_it_has_slack(factory, blank):
    before = attached_line(factory, blank)
    doc = copy.deepcopy(before.doc)
    doc['joints'] = []
    doc['parts'][1]['pose']['position_mm'] = [0,0,240]
    doc['anchors'].append({'part':'weight','surface':'fixture'})
    before = factory(doc)
    first = factory(attach_part(before,'line','line/link-1',
                                {'part':'support','port':'eye'},'spherical')['document'])
    posed = factory(attach_part(first,'line','line/link-10',
                                {'part':'weight','port':'eye'},'spherical')['document'])
    assert_attached(posed)
    after = factory(update_object_parameters(posed,'line',
                    {'length_mm':180,'link_catalog':'generic.rope-jute-6'}))
    assert_attached(after)
    assert np.allclose(after.parts['weight'].matrix,posed.parts['weight'].matrix)


def test_dragging_start_keeps_other_end_and_carries_free_attachments(factory, blank):
    before = attached_line(factory, blank)
    before.doc['anchors'] = []
    before = factory(before.doc)
    result = resize_drag(before, 'line', 120, 'start')
    after = factory(result['document'])
    assert_attached(after)
    assert np.allclose(after.parts['weight'].matrix[:3,3], [0,0,200], atol=.05)
    assert np.allclose(after.parts['support'].matrix[:3,3], [0,0,320], atol=.05)


def test_extending_attached_end_keeps_attachment_at_new_tip(factory, blank):
    before = attached_line(factory, blank)
    after = factory(update_object_parameters(before, 'line',
                    {'length_mm':280,'link_catalog':'generic.rope-jute-6'}))
    bottom = next(j for j in after.doc['joints'] if j['id']=='bottom')
    assert bottom['b']['part']=='line/link-14'
    assert np.allclose(after.parts['weight'].matrix[:3,3], [0,0,120], atol=.05)
    assert_attached(after)


def test_changing_profile_preserves_an_interior_attachment_offset(factory, blank):
    before = attached_line(factory, blank)
    before.doc['joints'] = []
    before = factory(before.doc)
    point,_ = before.parts['line/link-5'].frame({'frame':{'position_mm':[3,0,0]}})
    before.doc['parts'].append({'id':'tag','catalog':'generic.d-link',
                                'pose':{'position_mm':point.tolist()}})
    before.doc['joints'] = [{'id':'tag-joint','type':'spherical',
                             'a':{'part':'tag','port':'eye'},
                             'b':{'part':'line/link-5',
                                  'frame':{'position_mm':[3,0,0]}}}]
    before = factory(before.doc)
    after = factory(update_object_parameters(before,'line',
                    {'length_mm':200,'link_catalog':'generic.rope-nylon-10'}))
    assert_attached(after)
    endpoint = after.doc['joints'][0]['b']
    assert endpoint['part']=='line/link-4'
    assert np.linalg.norm(np.array(endpoint['frame']['position_mm'])[:2]) > 2.9


def test_switching_link_type_rotates_a_free_fixed_fitting_with_the_line(factory, blank):
    before = attached_line(factory, blank, catalog='generic.chain-link')
    before.doc['joints'][1]['type'] = 'fixed'
    before = factory(before.doc)
    old_relative = (np.linalg.inv(before.parts['line/link-10'].matrix)@
                    before.parts['weight'].matrix)
    after = factory(update_object_parameters(before,'line',
                    {'length_mm':200,'link_catalog':'generic.chain-heavy-100'}))
    new_relative = (np.linalg.inv(after.parts['line/link-3'].matrix)@
                    after.parts['weight'].matrix)
    assert np.allclose(new_relative[:3,:3],old_relative[:3,:3],atol=1e-6)
    assert_attached(after)
