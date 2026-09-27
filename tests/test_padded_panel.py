import copy

import numpy as np
import pytest

from pipesim.document import DocumentError
from pipesim.geometry import bounds, mesh_for_part
from pipesim.human import seat_fit


def padded(blank, factory, **parameters):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'pad','catalog':'generic.padded-panel','parameters':parameters}]
    return factory(doc)


def test_padded_panel_has_distinct_backing_and_contact_surface(blank,factory):
    assembly=padded(blank,factory,width_mm=500,depth_mm=300,thickness_mm=70,
                    padding_mm=50,edge_inset_mm=10)
    part=assembly.parts['pad']
    backing,cushion=part.shapes
    assert part.kind=='panel'
    assert backing['size_mm']==[500,300,20]
    assert backing['position_mm']==[0,0,-25]
    assert cushion['size_mm']==[480,280,50]
    assert cushion['position_mm']==[0,0,10]
    assert backing['color']!=cushion['color']
    assert np.allclose(bounds(part),[[-250,-150,-35],[250,150,35]])
    assert len(np.unique(mesh_for_part(part).visual.face_colors[:,:3],axis=0))==2
    expected=(500*300*20*650+480*280*50*60)*1e-9
    assert part.mass==pytest.approx(expected)
    assert part.center_of_mass[2]<0  # Denser backing sits below the cushion.
    assert all(value>0 for value in part.definition['inertia_kg_m2'])
    from pipesim.exporting import bom
    record=bom(assembly)['items'][0]
    assert record['panel_dimensions_mm']==pytest.approx([500,300,70])
    assert record['mass_kg']==pytest.approx(expected)


def test_padded_panel_dimensions_rebuild_both_layers_and_mass(blank,factory):
    small=padded(blank,factory,width_mm=400,depth_mm=300)
    large=padded(blank,factory,width_mm=800,depth_mm=300)
    assert large.parts['pad'].shapes[0]['size_mm'][0]==800
    assert large.parts['pad'].shapes[1]['size_mm'][0]==790
    assert large.parts['pad'].mass>small.parts['pad'].mass


def test_padded_panel_has_both_contact_layers_in_physics(blank,factory):
    import pybullet as pb
    from pipesim.physics import World
    assembly=padded(blank,factory)
    with World(assembly) as world:
        body,link,_=world.part_map['pad']
        assert len(pb.getCollisionShapeData(body,link,physicsClientId=world.client))==2


@pytest.mark.parametrize('parameters',[
    {'padding_mm':60}, {'padding_mm':-1}, {'edge_inset_mm':200},
    {'thickness_mm':0}, {'width_mm':float('inf')},
])
def test_invalid_padded_dimensions_are_rejected(blank,factory,parameters):
    with pytest.raises(DocumentError): padded(blank,factory,**parameters)


def test_padded_seat_uses_cushion_top_and_usable_width(load,factory):
    doc=copy.deepcopy(load('seated-human').doc)
    seat=doc['parts'][0]
    seat['catalog']='generic.padded-panel'
    seat['parameters']={'width_mm':400,'depth_mm':300,'thickness_mm':70,
                        'padding_mm':50,'edge_inset_mm':20}
    assembly=factory(doc)
    result=seat_fit(assembly,'person','seat')
    assert result['available_width_mm']==pytest.approx(360)
    assert result['available_depth_mm']==pytest.approx(260)
    assert result['seat_height_mm']==pytest.approx(460)
    assert not result['checks']['thigh_support']
