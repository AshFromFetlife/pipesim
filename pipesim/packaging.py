"""Create a portable design directory with resolved catalogue data and local meshes."""
import copy
from pathlib import Path
from .document import Assembly,write
from .editing import expand_objects

def bundle(assembly,directory):
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=True)
    doc=expand_objects(assembly); doc['libraries']=[]; doc['definitions']={}
    doc.pop('results',None); doc.pop('build_plan',None)
    materials={}
    for index,spec in enumerate(doc['parts']):
        part=assembly.parts[spec['id']]; definition=copy.deepcopy(part.definition)
        definition.pop('parameters',None)
        material=definition.pop('material_data',None)
        if material:
            name='bundle.material-'+str(index); materials[name]=material; definition['material']=name
        for shape_index,shape in enumerate(definition['geometry']):
            if shape['type']=='mesh':
                from .geometry import shape_mesh
                mesh=shape_mesh({k:v for k,v in shape.items() if k not in ('position_mm','rotation_deg','axis')},part.base)
                assets=directory/'assets'; assets.mkdir(exist_ok=True)
                filename=f'part-{index}-{shape_index}.glb'; mesh.export(assets/filename)
                shape['file']='assets/'+filename; shape.pop('scale',None)
        catalog='bundle.part-'+str(index); doc['definitions'][catalog]=definition
        spec['catalog']=catalog; spec.pop('body',None); spec.pop('parameters',None)
    if materials:
        write(directory/'materials.yaml',{'format':'pipesim-library/1','name':'Resolved build materials','materials':materials})
        doc['libraries']=['materials.yaml']
    doc.setdefault('metadata',{})['bundled_from_input_sha256']=assembly.input_hash
    Assembly.from_doc(doc,directory)  # Verify references before writing the deliverable.
    write(directory/'design.pipe.yaml',doc)
    return directory/'design.pipe.yaml'
