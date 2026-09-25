"""Real fitting geometry must admit its intended pipes, in both mesh and physics."""
import copy

import numpy as np
import pytest

from pipesim.editing import connect_member
from pipesim.geometry import shape_mesh
from pipesim.physics import simulate
from pipesim.validation import validate


def crossover_design(blank,size='C'):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'fitting','catalog':f'tubeclamp.TC161{size}','pose':{'position_mm':[0,0,500]}},
                  {'id':'upright','catalog':f'tubeclamp.tube-{size}','parameters':{'length_mm':1000}},
                  {'id':'crossbar','catalog':f'tubeclamp.tube-{size}','parameters':{'length_mm':800}}]
    doc=connect_member(doc,'.','upright','fitting','through',at_mm=500)
    doc=connect_member(doc,'.','crossbar','fitting','cross',at_mm=400)
    doc['anchors']=[{'part':'upright','surface':'fixture'}]
    return doc


def ray_hits(triangles,origin,direction):
    """Direct triangle intersections, independent of collision proxies or R-tree extras."""
    edge1=triangles[:,1]-triangles[:,0]; edge2=triangles[:,2]-triangles[:,0]
    h=np.cross(direction,edge2); determinant=np.einsum('ij,ij->i',edge1,h)
    usable=np.abs(determinant)>1e-9
    inverse=np.zeros_like(determinant); inverse[usable]=1/determinant[usable]
    offset=origin-triangles[:,0]
    u=inverse*np.einsum('ij,ij->i',offset,h)
    q=np.cross(offset,edge1)
    v=inverse*(q@direction)
    distance=inverse*np.einsum('ij,ij->i',edge2,q)
    return np.any(usable&(u>=0)&(v>=0)&(u+v<=1)&(distance>=0))


@pytest.mark.parametrize('size,offset,mass',[
    ('A',26.9,.222),('B',33.7,.308),('C',42.4,.49),('D',48.3,.538),('E',60.3,.864)
])
def test_offset_cross_accepts_two_continuous_pipes(blank,factory,size,offset,mass):
    assembly=factory(crossover_design(blank,size)); fitting=assembly.parts['fitting']
    assert fitting.mass==pytest.approx(mass)
    assert fitting.ports['cross']['position_mm']==[0,offset,0]
    assert all(p['through'] and p['assembly']=='slide' for p in fitting.ports.values())
    report=validate(assembly)
    assert report['valid'],report['issues']
    assert len(assembly.rigid_groups())==1
    mesh=shape_mesh(fitting.shapes[0],fitting.base)
    assert mesh.is_volume
    for socket in fitting.ports.values():
        axis=np.array(socket['axis'],float); center=np.array(socket['position_mm'],float)
        across=np.cross(axis,[0,1,0]); other=np.cross(axis,across)
        # A pipe's centre and outer skin must pass through the actual render mesh.
        for radius in (0,socket['diameter_mm']/2):
            for angle in np.linspace(0,2*np.pi,12,endpoint=False):
                origin=center-axis*200+radius*(np.cos(angle)*across+np.sin(angle)*other)
                assert not ray_hits(mesh.triangles,origin,axis),socket


def test_offset_cross_loose_socket_slides_without_plugged_bore(blank,factory):
    doc=crossover_design(blank)
    doc['joints'][0]['locked']=False
    assembly=factory(doc)
    assert len(assembly.rigid_groups())==2
    result=simulate(assembly,.2,10)
    pose=result['frames'][-1]['parts']
    assert pose['upright']['position_mm'][2]==pytest.approx(500,abs=.01)
    assert 250<pose['fitting']['position_mm'][2]<400
    assert pose['crossbar']['position_mm'][2]==pytest.approx(pose['fitting']['position_mm'][2],abs=1)


@pytest.mark.parametrize('size',['B','C','D'])
def test_split_tee_distinct_casting_preserves_socket_fit(blank,factory,size):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'split','catalog':f'tubeclamp.TC136{size}','pose':{'position_mm':[0,0,500]}},
                  {'id':'upright','catalog':f'tubeclamp.tube-{size}','parameters':{'length_mm':1000}},
                  {'id':'rail','catalog':f'tubeclamp.tube-{size}','parameters':{'length_mm':500}}]
    doc=connect_member(doc,'.','upright','split','through',at_mm=500)
    doc=connect_member(doc,'.','rail','split','branch')
    doc['anchors']=[{'part':'upright','surface':'fixture'}]
    assembly=factory(doc); fitting=assembly.parts['split']
    report=validate(assembly)
    assert report['valid'],report['issues']
    assert fitting.ports['through']['assembly']=='radial'
    assert fitting.ports['through']['diameter_mm']==fitting.ports['branch']['diameter_mm']
    mesh=shape_mesh(fitting.shapes[0],fitting.base)
    assert mesh.is_volume and len(mesh.split())==2  # Separate cap and main casting.
    assert not ray_hits(mesh.triangles,np.array([0,0,-200]),np.array([0,0,1]))
