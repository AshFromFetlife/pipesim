"""Build hollow fitting meshes and bore-preserving convex collision compounds.

manifold3d is a catalogue build dependency only; generated STL files ship with
the library, so editing, rendering and simulation need no boolean engine.
"""
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'.pipesim/build-tools'))
import trimesh


def cylinder(diameter,length,position=(0,0,0),axis=(0,0,1)):
    mesh=trimesh.creation.cylinder(radius=diameter/2,height=length,sections=64)
    matrix=trimesh.geometry.align_vectors([0,0,1],axis)
    matrix[:3,3]=position
    mesh.apply_transform(matrix)
    return mesh


def block(size,position):
    mesh=trimesh.creation.box(size)
    mesh.apply_translation(position)
    return mesh


def ring_collision(outer,bore,length,position=(0,0,0),axis=(0,0,1),other_bores=()):
    """Tangent boxes keep the primary bore open; trim away crossing bores.

    Each other bore is perpendicular to this socket. Trimming uses the closest
    point of each box to that bore, conservatively preserving tube clearance.
    """
    result=[]; inner=bore/2; radial=(outer-bore)/2; count=32
    tangent=2*inner*math.tan(math.pi/count)
    axis=np.asarray(axis,float); position=np.asarray(position,float)
    align=trimesh.geometry.align_vectors([0,0,1],axis)[:3,:3]
    for index in range(count):
        angle=(index+.5)*2*math.pi/count
        rotation=align@Rotation.from_euler('z',angle).as_matrix()
        center=position+rotation@np.array([inner+radial/2,0,0])
        intervals=[(-length/2,length/2)]
        for other_center,other_axis,other_diameter in other_bores:
            other_center=np.asarray(other_center,float); other_axis=np.asarray(other_axis,float)
            normal=np.cross(axis,other_axis)
            if not np.isclose(np.linalg.norm(normal),1): raise ValueError('Crossing bores must be perpendicular')
            separation=abs(np.dot(center-other_center,normal))
            extent=abs(np.dot(rotation[:,0],normal))*radial/2+abs(np.dot(rotation[:,1],normal))*tangent/2
            distance=max(0,separation-extent)
            radius=other_diameter/2
            if distance>=radius: continue
            half_gap=math.sqrt(radius*radius-distance*distance)+.02
            station=np.dot(other_center-position,axis)
            cut_lo,cut_hi=station-half_gap,station+half_gap
            kept=[]
            for lo,hi in intervals:
                if cut_hi<=lo or cut_lo>=hi: kept.append((lo,hi))
                else:
                    if cut_lo>lo: kept.append((lo,cut_lo))
                    if cut_hi<hi: kept.append((cut_hi,hi))
            intervals=kept
        for lo,hi in intervals:
            result.append({'type':'box','size_mm':[radial,tangent,float(hi-lo)],
                           'position_mm':(center+axis*(lo+hi)/2).round(6).tolist(),
                           'rotation_deg':Rotation.from_matrix(rotation).as_euler('xyz',degrees=True).round(6).tolist()})
    return result


def save_mesh(code,solids,cutters,collision):
    directory=ROOT/'pipesim/data/libraries/meshes'
    directory.mkdir(parents=True,exist_ok=True)
    shell=trimesh.boolean.union(solids,engine='manifold')
    mesh=trimesh.boolean.difference([shell,*cutters],engine='manifold')
    if not mesh.is_volume: raise ValueError(f'{code}: fitting mesh must be watertight')
    filename=code.lower()+'.stl'
    mesh.export(directory/filename)
    return {'type':'mesh','file':'meshes/'+filename,'collision_geometry':collision}


def standard_crossover(code,od,dimensions):
    bore=od+1
    offset=dimensions['i']; outer=dimensions['h']-offset
    first=[0,0,0]; second=[0,offset,0]
    solids=[cylinder(outer,dimensions['g'],first),cylinder(outer,dimensions['f'],second,[1,0,0])]
    cutters=[cylinder(bore,outer*4,first),cylinder(bore,outer*4,second,[1,0,0])]
    collision=ring_collision(outer,bore,dimensions['g'],first,other_bores=[(second,[1,0,0],bore)])
    collision+=ring_collision(outer,bore,dimensions['f'],second,[1,0,0],[(first,[0,0,1],bore)])
    return save_mesh(code,solids,cutters,collision)


def split_tee(code,od,dimensions,reach):
    # Keep existing authored port frames. Only f/g/h are supplier dimensions;
    # casting OD, seam width and bolt envelopes remain explicit approximations.
    bore=od+1; outer=od+9; width=dimensions['h']; branch_outer=dimensions['h']
    branch_length=reach-outer/2; branch_center=(reach+outer/2)/2
    solids=[cylinder(outer,width),cylinder(branch_outer,reach-bore/2,[(reach+bore/2)/2,0,0],[1,0,0])]
    lug_diameter=14; lug_y=(dimensions['g']-lug_diameter)/2
    # Opposing ears accept bolts across the split plane; all remain outside the bore.
    bridge_length=lug_y-(outer/2-.5)
    for sign in (-1,1):
        solids.append(cylinder(lug_diameter,14,[0,sign*lug_y,0],[1,0,0]))
        solids.append(block([14,bridge_length,8],[0,sign*(lug_y-bridge_length/2),0]))
    cutters=[cylinder(bore,width+20),cylinder(bore,branch_length+2,[branch_center,0,0],[1,0,0]),
             block([.9,dimensions['g']+20,width+20],[0,0,0])]
    collision=ring_collision(outer,bore,width)
    collision+=ring_collision(branch_outer,bore,branch_length,[branch_center,0,0],[1,0,0])
    land_length=(outer-bore)/2
    collision.append({'type':'cylinder','diameter_mm':branch_outer,'length_mm':land_length,'position_mm':[(outer+bore)/4,0,0],'axis':[1,0,0]})
    for sign in (-1,1):
        collision.append({'type':'cylinder','diameter_mm':lug_diameter,'length_mm':14,'position_mm':[0,sign*lug_y,0],'axis':[1,0,0]})
        collision.append({'type':'box','size_mm':[14,bridge_length,8],'position_mm':[0,sign*(lug_y-bridge_length/2),0]})
    return save_mesh(code,solids,cutters,collision),lug_y


def swivel_short_tee(code,od,dimensions):
    """One stepped TC148 casting, with a short collar and a coplanar pair socket.

    Two castings nest with opposite collar axes, collar centres f apart, and
    branch axes at the common collar face. The tapered web is inferred from
    supplier photographs; only f/h/i below are dimensioned in the drawing.
    """
    f,h,i=(dimensions[key] for key in ('f','h','i'))
    outer=od+9;radius=outer/2;bore=od+1;branch_z=f/2
    collar=cylinder(outer,f)
    barrel=cylinder(outer,i,[h+i/2,0,branch_z],[1,0,0])
    # A narrow web meets the collar on its occupied side. The opposite side
    # opens out towards the terminal barrel, leaving room for the second tee.
    start=radius*.55
    vertices=[[start,y,z] for y in (-radius*.45,radius*.45) for z in (-f/2,branch_z)]
    vertices += [[h,radius*math.cos(a),branch_z+radius*math.sin(a)] for a in np.linspace(0,2*math.pi,64,endpoint=False)]
    shoulder=trimesh.convex.convex_hull(np.array(vertices))
    cutters=[cylinder(bore,outer*4),cylinder(bore,i+2,[h+i/2+1,0,branch_z],[1,0,0])]
    collision=ring_collision(outer,bore,f)
    collision+=ring_collision(outer,bore,i,[h+i/2,0,branch_z],[1,0,0])
    # Interior boxes approximate the solid web without closing either bore.
    for left,right in zip(np.linspace(radius,h,9)[:-1],np.linspace(radius,h,9)[1:]):
        t=(left-start)/(h-start);half_width=radius*(.45+.25*t)
        bottom=-f/2*(1-t)+(branch_z-radius*.7)*t;top=branch_z+radius*.7*t
        collision.append({'type':'box','size_mm':[float(right-left),float(2*half_width),float(top-bottom)],
                          'position_mm':[float((left+right)/2),0,float((top+bottom)/2)]})
    return save_mesh(code,[collar,shoulder,barrel],cutters,collision)
