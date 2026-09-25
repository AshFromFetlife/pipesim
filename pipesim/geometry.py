"""One geometry pipeline for the editor, mesh exports, collision checks and renderers."""
from __future__ import annotations
from pathlib import Path
import math
import numpy as np
import trimesh
from .math3d import transform, align_axis, point

def shape_transform(shape):
    matrix=transform(shape)
    if "axis" in shape: matrix[:3,:3]=matrix[:3,:3]@align_axis([0,0,1],shape["axis"])
    return matrix

def volume(s):
    kind=s["type"]
    r=s.get("radius_mm",s.get("diameter_mm",0)/2)
    length=s.get("length_mm",0)
    if kind in ("box","extrusion"): return float(np.prod(s["size_mm"]))
    if kind=="sphere": return 4*math.pi*r**3/3
    if kind=="capsule": return math.pi*r*r*length+4*math.pi*r**3/3
    if kind=="tube": return math.pi*(r*r-max(0,r-s["wall_mm"])**2)*length
    if kind=="cylinder": return math.pi*r*r*length
    if kind=="mesh": return 0.
    raise ValueError(f"Unknown shape {kind}")

def shape_mesh(s,base=Path("."),sections=32):
    kind=s["type"]
    r=s.get("radius_mm",s.get("diameter_mm",0)/2)
    length=s.get("length_mm",0)
    if kind=="box": mesh=trimesh.creation.box(s["size_mm"])
    elif kind=="extrusion":
        # An intentionally simplified T-slot envelope; section properties come from the supplier.
        w,d,l=s["size_mm"]
        t=min(w,d)*.16
        pieces=[trimesh.creation.box([w*.46,d*.46,l])]
        for x in (-1,1):
            for y in (-1,1):
                corner=trimesh.creation.box([w*.27,d*.27,l])
                corner.apply_translation([x*w*.365,y*d*.365,0])
                pieces.append(corner)
        for x,y,sx,sy in ((0,1,t,d*.35),(0,-1,t,d*.35),(1,0,w*.35,t),(-1,0,w*.35,t)):
            web=trimesh.creation.box([sx,sy,l]); web.apply_translation([x*w*.24,y*d*.24,0]); pieces.append(web)
        mesh=trimesh.util.concatenate(pieces)
    elif kind=="tube":
        inner=r-s["wall_mm"]
        if inner<=0: raise ValueError("Tube wall must be smaller than its radius")
        mesh=trimesh.creation.annulus(r_min=inner,r_max=r,height=length,sections=sections)
    elif kind=="cylinder": mesh=trimesh.creation.cylinder(radius=r,height=length,sections=sections)
    elif kind=="sphere": mesh=trimesh.creation.icosphere(subdivisions=2,radius=r)
    elif kind=="capsule": mesh=trimesh.creation.capsule(height=length,radius=r,count=[16,24])
    elif kind=="mesh":
        path=(Path(base)/s["file"]).resolve()
        if not path.is_file(): raise ValueError(f"Mesh file does not exist: {path}")
        mesh=trimesh.load(path,force="mesh",process=False)
        # STL stores separate vertices for every triangle. Join shared corners
        # so watertight parts retain their volume-based COM and inertia.
        if path.suffix.lower()=='.stl': mesh.merge_vertices()
        mesh.apply_scale(s.get("scale",1))
    else: raise ValueError(f"Unknown shape {kind}")
    mesh.apply_transform(shape_transform(s))
    return mesh

def mesh_for_part(part,world=False):
    pieces=[]
    for shape in part.shapes:
        mesh=shape_mesh(shape,part.base)
        from PIL import ImageColor
        color=ImageColor.getrgb(shape.get("color",part.definition.get("color","#a1aab0")))
        mesh.visual.face_colors=[*color,255]
        pieces.append(mesh)
    result=trimesh.util.concatenate(pieces)
    if world: result.apply_transform(part.matrix)
    return result

def bounds(part):
    return mesh_for_part(part,True).bounds

def lowest_z(part, matrix=None):
    """Lowest world Z, with exact curved-primitive support and actual mesh vertices."""
    world=part.matrix if matrix is None else matrix
    heights=[]
    for shape in part.shapes:
        local=world@shape_transform(shape);axis=local[2,:3];kind=shape['type']
        r=shape.get('radius_mm',shape.get('diameter_mm',0)/2)
        if kind in ('box','extrusion'): extent=np.abs(axis)@np.array(shape['size_mm'])/2
        elif kind in ('tube','cylinder'): extent=abs(axis[2])*shape['length_mm']/2+r*np.linalg.norm(axis[:2])
        elif kind=='sphere': extent=r
        elif kind=='capsule': extent=abs(axis[2])*shape['length_mm']/2+r
        else:
            mesh=shape_mesh(shape,part.base)
            if len(mesh.vertices): heights.append(float(np.min(mesh.vertices@world[2,:3]+world[2,3])))
            continue
        heights.append(float(local[2,3]-extent))
    if not heights or not np.isfinite(heights).all(): raise ValueError(f'{part.id}: no finite geometry to place on the floor')
    return min(heights)

def assembly_bounds(assembly):
    if not assembly.parts: return np.array([[-500,-500,0],[500,500,1000]],float)
    boxes=np.array([bounds(p) for p in assembly.parts.values()])
    return np.array([boxes[:,0].min(axis=0),boxes[:,1].max(axis=0)])

def collision_primitives(part):
    """Convex compounds with hollow sockets. No solid convex hull across a pipe bore.

    Annuli are approximated with tangent boxes; the hole clearance is preserved.
    Mesh imports use an explicit convex hull or a static concave triangle mesh.
    """
    result=[]
    for shape in part.shapes:
        local=shape_transform(shape)
        kind=shape['type']
        r=shape.get('radius_mm',shape.get('diameter_mm',0)/2)
        if kind=='tube' and part.kind!='member':
            inner=r-shape['wall_mm']
            n=24
            radial=shape['wall_mm']
            tangent=2*inner*math.tan(math.pi/n)
            for i in range(n):
                a=(i+.5)*2*math.pi/n
                matrix=np.eye(4)
                matrix[:3,:3]=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
                matrix[:3,3]=[(inner+radial/2)*math.cos(a),(inner+radial/2)*math.sin(a),0]
                result.append(({'type':'box','size_mm':[radial,tangent,shape['length_mm']]},local@matrix))
        elif kind=='tube': result.append(({'type':'cylinder','diameter_mm':2*r,'length_mm':shape['length_mm']},local))
        elif kind=='extrusion': result.append(({'type':'box','size_mm':shape['size_mm']},local))
        elif kind=='mesh' and shape.get('collision_geometry'):
            for simple in shape['collision_geometry']: result.append((simple,local@shape_transform(simple)))
        else: result.append((shape,local))
    return result

def export_mesh(assembly,path):
    scene=trimesh.Scene()
    for p in assembly.parts.values(): scene.add_geometry(mesh_for_part(p),node_name=p.id,geom_name=p.id,transform=p.matrix)
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.suffix.lower() in ('.glb','.gltf'): scene.export(str(path))
    else: scene.to_geometry().export(str(path))
    return path
