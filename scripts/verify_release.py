"""Check the built wheel contains its offline assets and runs outside the checkout."""
from pathlib import Path
import hashlib,json,subprocess,sys,zipfile

root=Path(__file__).resolve().parents[1]
wheel=next((root/'output/wheels').glob('pipesim-*.whl'))
destination=root/'output/wheel-check'; destination.mkdir(parents=True,exist_ok=True)
with zipfile.ZipFile(wheel) as archive:
    required=['pipesim/web/index.html','pipesim/web/theme.css','pipesim/web/vendor/three.core.js','pipesim/web/vendor/addons/controls/TransformControls.js','pipesim/data/libraries/tubeclamp.yaml','pipesim/data/schemas/design.schema.json','pipesim/human.py','pipesim/loadcases.py','pipesim/motion.py']
    required += [f'pipesim/data/libraries/meshes/tc161{size}.stl' for size in 'abcde']
    required += [f'pipesim/data/libraries/meshes/tc136{size}.stl' for size in 'bcd']
    required += ['pipesim/web/snapping.js','pipesim/web/snap-settings.js','pipesim/web/placement.css','pipesim/snapping.py','pipesim/connections.py']
    required += ['pipesim/web/files.css','pipesim/resizing.py','pipesim/sliding.py','pipesim/posing.py','pipesim/grouping.py','pipesim/duplication.py','pipesim/fitting.py','pipesim/chain.py','pipesim/preview.py','pipesim/fit_adjustments.py']
    required += [f'pipesim/data/libraries/meshes/tc148{size}.stl' for size in 'abcde']
    for path in required: assert path in archive.namelist(),path
    for name in archive.namelist(): assert (destination/name).resolve().is_relative_to(destination.resolve())
    archive.extractall(destination)
code="""
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
import pipesim
from pipesim.cli import main
from pipesim.document import Assembly
from pipesim.geometry import shape_mesh
assert Path(pipesim.__file__).resolve().is_relative_to(Path(sys.argv[1]).resolve())
codes=['TC161'+size for size in 'ABCDE']+['TC136'+size for size in 'BCD']+['TC148'+size for size in 'ABCDE']
assembly=Assembly.from_doc({'format':'pipesim/1','units':'mm-kg-s-N-deg',
    'parts':[{'id':code,'catalog':'tubeclamp.'+code} for code in codes]})
for part in assembly.parts.values():
    assert shape_mesh(part.shapes[0],part.base).is_volume,part.id
from pipesim.grouping import move_object,regroup_object
from pipesim.editing import expand_objects
human=Assembly.from_doc({'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[],
    'objects':[{'id':'person','template':'human','parameters':{'pose':'seated'}}]})
grouped=regroup_object(Assembly.from_doc(expand_objects(human)),'person')
moved=move_object(Assembly.from_doc(grouped),'person',{'position_mm':[3000,0,0],'rotation_deg':[0,0,180]})
assert len(moved['moved'])==19 and len(moved['document']['objects'])==1
from pipesim.duplication import duplicate
copies=duplicate(Assembly.from_doc(moved['document']),'person/right_hand','subassembly',count=2)
assert len(Assembly.from_doc(copies['document']).parts)==57
limb=duplicate(Assembly.from_doc(moved['document']),'person/right_hand')
assert len(Assembly.from_doc(limb['document']).parts)==20
from pipesim.snapping import connection_options
from pipesim.validation import validate
triangle=Assembly.load(sys.argv[2])
count=len(triangle.joints)
for fitting,station in [('upper-back',800),('upper-left',500)]:
    result=connection_options(triangle,'top-bar',fitting,'through',at_mm=station,locked=False,force=True)
    assert result['recommended'],result
    triangle=Assembly.from_doc(next(o['document'] for o in result['options'] if o['move']==result['recommended']),triangle.base)
assert len(triangle.joints)==count+2 and validate(triangle)['valid']
adjustment=Assembly.load(sys.argv[3])
for permissions in ({'unlock_connectors':1},{'resize_members':1,'max_length_change_mm':2}):
    result=connection_options(adjustment,'bottom-arm','elbow','x',end='end',insertion_mm=30,force=True,force_options=permissions)
    assert result['recommended'],result
    option=next(o for o in result['options'] if o['move']==result['recommended'])
    assert option['gap_mm']<.03 and validate(Assembly.from_doc(option['document'],adjustment.base))['valid']
    assert option['adjustments']['unlocked' if 'unlock_connectors' in permissions else 'resized']
from pipesim.grouping import update_object_parameters,set_object_layout
from pipesim.posing import transform_part
from pipesim.math3d import pose_of
chain=Assembly.from_doc({'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[],
    'objects':[{'id':'chain','template':'chain','parameters':{'length_mm':4000},'pose':{'position_mm':[0,0,4500]}}]})
assert len(chain.editor_groups())==1 and len(chain.rigid_groups())==200
chain=Assembly.from_doc(set_object_layout(chain,'chain','posable'))
target=pose_of(chain.parts['chain/link-200'].matrix);target['position_mm'][0]+=30;target['position_mm'][2]+=30
posed=transform_part(chain,'chain/link-200',target)
assert posed['position_error_mm']<.05 and len(posed['document']['objects'])==1
from pipesim.preview import PreviewCache
from pipesim.posing import commit_transform
cache=PreviewCache();prepared=cache.remember(chain)
preview=transform_part(chain,'chain/link-200',target,preview=True,prepared=prepared)
assert 'document' not in preview and preview['poses']==posed['poses']
assert len(commit_transform(chain,preview['poses'])['objects'])==1
from pipesim.posing import drop_to_floor
from pipesim.geometry import lowest_z
grounded=Assembly.from_doc(drop_to_floor(chain,'chain/link-200','chain')['document'])
assert abs(min(lowest_z(p) for p in grounded.parts.values()))<1e-6
assert len(grounded.doc['objects'])==1 and not grounded.doc['parts']
chain=Assembly.from_doc(update_object_parameters(Assembly.from_doc(posed['document']),'chain',{'length_mm':4100}))
assert len(chain.parts)==205 and len(chain.doc['objects'])==1
raise SystemExit(main(['library','--query','TC161','-o','catalog.json']))
"""
subprocess.run([sys.executable,'-c',code,str(destination),str(root/'examples/hinged-triangle.pipe.yaml'),str(root/'examples/assembly-adjustments.pipe.yaml')],cwd=destination,check=True)
assert 'tubeclamp.TC161C' in json.loads((destination/'catalog.json').read_text())['parts']
record={'wheel':wheel.name,'sha256':hashlib.sha256(wheel.read_bytes()).hexdigest(),'bytes':wheel.stat().st_size,'offline_assets_checked':required}
(root/'output/release-check.json').write_text(json.dumps(record,indent=2))
print(json.dumps(record,indent=2))
