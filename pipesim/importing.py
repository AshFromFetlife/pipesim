"""Import user geometry while retaining explicit units, mass and editable interfaces."""
from pathlib import Path
import re
import shutil
import trimesh
from .document import write,DocumentError

def import_mesh(mesh_path,library_path,id,mass_kg,scale=1):
    mesh_path=Path(mesh_path).resolve(); library_path=Path(library_path).resolve()
    if mesh_path.suffix.lower() not in ('.stl','.obj','.glb','.ply'): raise DocumentError('Import supports STL, OBJ, GLB and PLY meshes')
    if not re.fullmatch(r'[A-Za-z0-9_.-]+',id): raise DocumentError('Part id must use letters, numbers, dots, hyphens or underscores')
    if mass_kg<=0 or scale<=0: raise DocumentError('Mass and scale must be positive')
    mesh=trimesh.load(mesh_path,force='mesh',process=False)
    if len(mesh.vertices)<4 or len(mesh.faces)<4: raise DocumentError('Mesh needs a closed three-dimensional collision envelope')
    mesh.apply_scale(scale)
    if mesh.extents.min()<=0: raise DocumentError('Mesh has zero thickness')
    target=library_path.parent/'assets'/f'{id}.glb'; target.parent.mkdir(parents=True,exist_ok=True)
    # Bake into a portable GLB; this also removes external OBJ material dependencies.
    mesh.export(target)
    relative=target.relative_to(library_path.parent).as_posix()
    part={'name':id,'kind':'rigid','geometry':[{'type':'mesh','file':relative,'scale':1,'collision':'convex'}],'mass_kg':float(mass_kg),'ports':{},'source':{'geometry_status':'user-imported','original_filename':mesh_path.name,'units':'mm after baking input scale','collision_note':'Convex hull; add collision_geometry for hollow or concave moving parts'}}
    library={'format':'pipesim-library/1','name':id+' mesh library','parts':{id:part}}
    write(library_path,library)
    return {'library':str(library_path),'mesh':str(target),'part':id,'bounds_mm':mesh.bounds.tolist(),'vertices':len(mesh.vertices),'faces':len(mesh.faces),'watertight':bool(mesh.is_watertight)}
