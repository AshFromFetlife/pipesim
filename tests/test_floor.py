import copy

import numpy as np
import pytest
import trimesh

from pipesim.document import Assembly,DocumentError
from pipesim.geometry import lowest_z,mesh_for_part
from pipesim.math3d import pose_of
from pipesim.posing import drop_to_floor
from pipesim.snapping import _movement_coordinates
from floor_cases import check_free_floor


def box(pid,position,size=(100,80,60)):
    return {'id':pid,'body':{'kind':'rigid','mass_kg':1,'geometry':[{'type':'box','size_mm':list(size)}]},'pose':{'position_mm':position}}


@pytest.mark.parametrize('z',[500,-500,0])
def test_drop_or_lift_rotated_compound_to_actual_lowest_point(blank,factory,z):
    part=box('part',[125,-75,z]);part['pose']['rotation_deg']=[27,43,19]
    part['body']['geometry'].append({'type':'box','size_mm':[20,30,40],'position_mm':[60,10,-110],'rotation_deg':[14,32,7]})
    blank['parts']=[part,box('other',[400,0,-1000])]
    before=factory(blank);original=copy.deepcopy(before.doc)
    result=drop_to_floor(before,'part');after=factory(result['document'])
    assert mesh_for_part(after.parts['part'],True).bounds[0,2]==pytest.approx(0,abs=1e-7)
    assert np.allclose(after.parts['part'].matrix[:3,:3],before.parts['part'].matrix[:3,:3])
    assert np.allclose(after.parts['part'].matrix[:2,3],[125,-75])
    assert np.array_equal(after.parts['other'].matrix,before.parts['other'].matrix)
    assert before.doc==original
    again=drop_to_floor(after,'part')
    assert not again['moved'] and again['document']==after.doc


def test_rotated_round_pipe_uses_exact_radius_not_its_origin(blank,factory):
    blank['parts']=[{'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},
                     'pose':{'position_mm':[80,40,-600],'rotation_deg':[33,42,19]}}]
    before=factory(blank);after=factory(drop_to_floor(before,'pipe')['document'])
    axis=after.parts['pipe'].matrix[2,:3]
    expected=500*abs(axis[2])+21.2*np.linalg.norm(axis[:2])
    assert after.parts['pipe'].matrix[2,3]==pytest.approx(expected)
    assert mesh_for_part(after.parts['pipe'],True).bounds[0,2]>=-1e-7


def test_imported_mesh_includes_scale_and_local_and_world_transforms(blank,tmp_path):
    mesh=trimesh.creation.box([20,40,60]);mesh.apply_translation([50,-60,70]);mesh.export(tmp_path/'part.stl')
    blank['parts']=[{'id':'mesh','body':{'kind':'rigid','mass_kg':2,'geometry':[{'type':'mesh','file':'part.stl','scale':2,
        'position_mm':[10,0,-30],'rotation_deg':[10,27,40]}]},'pose':{'position_mm':[40,80,-200],'rotation_deg':[28,33,17]}}]
    before=Assembly.from_doc(blank,tmp_path)
    after=Assembly.from_doc(drop_to_floor(before,'mesh')['document'],tmp_path,before.library)
    assert mesh_for_part(after.parts['mesh'],True).bounds[0,2]==pytest.approx(0,abs=1e-7)


@pytest.mark.parametrize('joint_type',['fixed','spherical'])
def test_drop_whole_moving_assembly_uses_lowest_attached_part(blank,factory,joint_type):
    blank['parts']=[box('top',[0,0,500]),box('foot',[0,0,200],(50,50,40))]
    blank['joints']=[{'id':'attachment','type':joint_type,'a':{'part':'top','frame':{'position_mm':[0,0,-300]}},'b':{'part':'foot'}}]
    before=factory(blank);after=factory(drop_to_floor(before,'top')['document'])
    assert min(mesh_for_part(p,True).bounds[0,2] for p in after.parts.values())==pytest.approx(0,abs=1e-7)
    for pid in before.parts:
        assert after.parts[pid].matrix[2,3]-before.parts[pid].matrix[2,3]==pytest.approx(-180)
    _movement_coordinates(before,after)


def slider(blank,factory,travel=1000):
    blank['parts']=[box('support',[1000,0,400]),box('part',[0,0,400],(100,100,100))]
    blank['anchors']=[{'part':'support','surface':'ceiling'}]
    blank['joints']=[{'id':'slider','type':'prismatic','a':{'part':'support','frame':{'position_mm':[-1000,0,0]}},
                     'b':{'part':'part'},'limits':{'slide_mm':[-travel,travel]}}]
    return factory(blank)


def test_drop_respects_available_joint_motion_and_fixed_world_support(blank,factory):
    before=slider(blank,factory);after=factory(drop_to_floor(before,'part')['document'])
    assert after.parts['part'].matrix[2,3]==pytest.approx(50,abs=.01)
    assert np.array_equal(after.parts['support'].matrix,before.parts['support'].matrix)
    _movement_coordinates(before,after)


def test_limited_or_anchored_drop_does_not_commit_partial_movement(blank,factory):
    before=slider(blank,factory,travel=100);original=copy.deepcopy(before.doc)
    with pytest.raises(DocumentError,match='Cannot reach the floor'):
        drop_to_floor(before,'part')
    assert before.doc==original
    with pytest.raises(DocumentError,match='support is fixed to the world'):
        drop_to_floor(before,'support')
    assert before.doc==original


def test_grouped_person_keeps_pose_and_grouping_while_lifting_to_floor(blank,factory):
    blank['objects']=[{'id':'person','template':'human','parameters':{'pose':'seated'},
                      'pose':{'position_mm':[2000,100,-700],'rotation_deg':[14,37,180]}}]
    before=factory(blank);original=copy.deepcopy(before.doc)
    after=factory(drop_to_floor(before,'person/left_hand','person')['document'])
    assert len(after.doc['objects'])==1 and not after.doc['parts']
    assert min(lowest_z(p) for p in after.parts.values())==pytest.approx(0,abs=1e-7)
    delta=np.eye(4);delta[2,3]=-min(lowest_z(p) for p in before.parts.values())
    for pid in before.parts: assert np.allclose(after.parts[pid].matrix,delta@before.parts[pid].matrix,atol=1e-7,rtol=0)
    assert before.doc==original


def test_hinged_part_corrects_its_changed_extent_when_lowered(blank,factory):
    blank['parts']=[box('support',[0,0,30]),box('part',[100,0,30],(20,20,20))]
    blank['anchors']=[{'part':'support','surface':'fixture'}]
    blank['joints']=[{'id':'hinge','type':'revolute','a':{'part':'support','frame':{'axis':[0,1,0]}},
                     'b':{'part':'part','frame':{'position_mm':[-100,0,0]}},'limits':{'angle_deg':[-60,60]}}]
    before=factory(blank);after=factory(drop_to_floor(before,'part')['document'])
    assert mesh_for_part(after.parts['part'],True).bounds[0,2]==pytest.approx(0,abs=.01)
    _movement_coordinates(before,after)


@pytest.mark.parametrize('seed',[0,1,2,3])
@pytest.mark.parametrize('selected',['rail','foot-1','sleeve-2'])
def test_free_closed_frame_drops_independently_of_selected_body(factory,seed,selected):
    from pipesim.preview import PreparedAssembly
    check_free_floor(factory,seed,selected=selected,prepared=PreparedAssembly)


def test_free_floor_placement_does_not_search_joint_travel(factory,monkeypatch):
    def unexpected(*args,**kwargs):
        raise AssertionError('Free floor placement must not run inverse kinematics')
    monkeypatch.setattr('pipesim.posing.Mechanism',unexpected)
    monkeypatch.setattr('pipesim.posing.least_squares',unexpected)
    check_free_floor(factory,1)


def test_drop_free_draft_attachment_uses_lowest_run_and_carries_other_body(blank,factory):
    from pipesim.drafting import preview
    for pid,z in [('lower',500),('upper',800)]:
        blank['parts'].append({'id':pid,'body':{'kind':'connector','mass_kg':1,
            'geometry':[{'type':'box','size_mm':[60,60,60]}],
            'ports':{'bore':{'type':'socket','profile':'round','diameter_mm':42.4,
                            'through':True,'engagement_mm':60,'axis':[0,0,1],
                            'position_mm':[0,0,0]}}},'pose':{'position_mm':[0,0,z]}})
    blank['draft_subassemblies']=[{'id':'draft','runs':[{'id':'run','catalog':'tubeclamp.tube-C',
        'start_mm':[0,0,100],'end_mm':[0,0,1000],
        'attachments':[{'connector':p,'port':'bore'} for p in ('lower','upper')]}]}]
    before=factory(blank);result=drop_to_floor(before,'lower');after=factory(result['document'])
    assert set(result['moved'])=={'lower','upper'}
    run=after.doc['draft_subassemblies'][0]['runs'][0]
    assert np.allclose(run['start_mm'],[0,0,0],atol=1e-7)
    assert np.allclose(run['end_mm'],[0,0,900],atol=1e-7)
    assert not preview(after)[0]['conflicts']
    assert after.parts['lower'].matrix[2,3]==pytest.approx(400)
    assert after.parts['upper'].matrix[2,3]==pytest.approx(700)
    assert not drop_to_floor(after,'upper')['moved']


def test_drop_whole_object_uses_attached_lower_body(blank,factory):
    blank['objects']=[{'id':'object','template':'custom','pose':{'position_mm':[0,0,500]},
        'components':{'parts':[box('body',[0,0,0])],'joints':[]}}]
    blank['parts']=[box('foot',[0,0,200],(40,40,40))]
    blank['joints']=[{'id':'attachment','type':'spherical',
        'a':{'part':'object/body','frame':{'position_mm':[0,0,-300]}},'b':{'part':'foot'}}]
    before=factory(blank);result=drop_to_floor(before,'object/body','object');after=factory(result['document'])
    assert set(result['moved'])=={'object/body','foot'}
    assert min(lowest_z(p) for p in after.parts.values())==pytest.approx(0,abs=1e-7)
    assert after.parts['object/body'].matrix[2,3]==pytest.approx(320)
    assert after.doc['objects'][0]['components']==before.doc['objects'][0]['components']


def test_unreachable_anchored_drop_names_external_fixing_and_remaining_gap(blank,factory):
    before=slider(blank,factory,travel=100)
    with pytest.raises(DocumentError,match=r'support is fixed to the world.*250.00 mm above Z = 0'):
        drop_to_floor(before,'part')
