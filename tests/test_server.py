import copy
import json
import threading
import urllib.request
import urllib.error
import pytest
from pipesim.server import EditorServer,EDITOR_API_VERSION
from pipesim.document import write

@pytest.fixture
def editor(tmp_path,load):
    write(tmp_path/'design.pipe.yaml',load('workbench').doc)
    server=EditorServer(tmp_path,'design.pipe.yaml',0)
    thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
    yield server
    server.shutdown(); server.server_close(); thread.join()

def request(server,path,data=None,token=True,headers=None):
    values=headers or {}
    if token: values['X-PipeSim-Token']=server.token
    if data is not None: values['Content-Type']='application/json'
    req=urllib.request.Request(f'http://127.0.0.1:{server.server_port}'+path,data=json.dumps(data).encode() if data is not None else None,headers=values)
    with urllib.request.urlopen(req) as response: return response.status,response.read()

def test_bootstrap_serves_real_library_and_offline_editor(editor):
    status,raw=request(editor,'/api/bootstrap'); data=json.loads(raw)
    assert status==200 and len(data['scene']['parts'])==13
    assert data['api_version']==EDITOR_API_VERSION
    assert 'porta.DOW-19' in data['library']
    assert request(editor,'/vendor/three.module.js')[0]==200


def test_chain_snapshot_keeps_grouping_length_controls_and_physical_joints(editor,blank):
    from pipesim.document import Assembly
    from pipesim.physics import simulate
    doc=copy.deepcopy(blank);doc['objects']=[{'id':'chain','template':'chain','parameters':{'length_mm':160},'pose':{'position_mm':[0,0,500],'rotation_deg':[0,30,0]}}]
    doc['anchors']=[{'part':'chain/link-1','surface':'ceiling'}]
    frame=simulate(Assembly.from_doc(doc),.1,20)['frames'][-1]
    _,raw=request(editor,'/api/snapshot',{'document':doc,'frame':frame});captured=json.loads(raw)
    assert len(captured['objects'])==1 and not captured['parts']
    assert len(captured['objects'][0]['components']['parts'])==8
    _,raw=request(editor,'/api/resolve',{'document':captured});scene=json.loads(raw)
    assert len(scene['groups'])==1 and len(scene['joints'])==7
    assert scene['chains'][0]['length_mm']==160
    _,raw=request(editor,'/api/object-parameters',{'document':captured,'object':'chain','parameters':{'length_mm':200}})
    assert len(json.loads(raw)['scene']['parts'])==10

def test_mutations_require_same_origin_session_token(editor):
    for headers,token in (({},False),({'Origin':'https://untrusted.example'},True)):
        with pytest.raises(urllib.error.HTTPError) as error: request(editor,'/api/save',{},token,headers)
        assert error.value.code==403


def test_simulation_runs_as_a_cancellable_job_without_resending_design(editor,blank):
    import time
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'box','catalog':'generic.box','pose':{'position_mm':[0,0,500]}}]
    _,raw=request(editor,'/api/simulate',{'document':doc,'duration':.02,'chain_links_per_body':5})
    job=json.loads(raw);assert job['status']=='running'
    deadline=time.monotonic()+30
    while job['status']=='running':
        assert time.monotonic()<deadline
        time.sleep(.05)
        job=json.loads(request(editor,'/api/simulation-status',{'job_id':job['id']})[1])
    assert job['status']=='completed',job
    assert 'result' not in job
    result=json.loads(request(editor,'/api/simulation-result',{'job_id':job['id']})[1])['result']
    assert result['frames']
    job=json.loads(request(editor,'/api/simulate',{'document':doc,'duration':30})[1])
    with pytest.raises(urllib.error.HTTPError) as error:
        request(editor,'/api/simulation-cancel',{'job_id':job['id']},token=False)
    assert error.value.code==403
    assert request(editor,'/api/bootstrap')[0]==200
    cancelled=json.loads(request(editor,'/api/simulation-cancel',{'job_id':job['id']})[1])
    assert cancelled['status']=='cancelled'
    assert 'result' not in cancelled

@pytest.mark.parametrize('path',['../outside.yaml','.git/config','.codex/settings.json'])
def test_workspace_boundaries_apply_to_open_and_save(editor,path):
    with pytest.raises(urllib.error.HTTPError) as error: request(editor,'/api/open?path='+urllib.parse.quote(path))
    assert error.value.code==400

def test_external_library_reference_is_rejected(editor):
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[],'libraries':['../outside.yaml']}
    with pytest.raises(urllib.error.HTTPError) as error: request(editor,'/api/resolve',{'document':doc})
    assert error.value.code==400
    assert 'workspace' in error.value.read().decode()

def test_save_reload_preserves_properties(editor,load):
    doc=copy.deepcopy(load('workbench').doc); doc['name']='Roundtrip edit'
    assert request(editor,'/api/save',{'document':doc,'path':'saved.pipe.yaml'})[0]==200
    saved=json.loads(request(editor,'/api/open?path=saved.pipe.yaml')[1])
    assert saved['document']['name']=='Roundtrip edit'


def test_design_list_roundtrips_saved_files_and_subfolders(editor,blank):
    assert json.loads(request(editor,'/api/designs')[1])=={'designs':[]}
    for path in ('designs/bench.pipe.yaml','designs/frames/My cube.YML','designs/frame.JSON'):
        request(editor,'/api/save',{'document':blank,'path':path})
    (editor.root/'designs'/'notes.txt').write_text('Not a design')
    (editor.root/'designs'/'.hidden.json').write_text('{}')
    write(editor.root/'designs'/'.git'/'config.json',blank)
    files=json.loads(request(editor,'/api/designs')[1])['designs']
    assert [item['path'] for item in files]==['designs/bench.pipe.yaml','designs/frame.JSON','designs/frames/My cube.YML']
    assert all(item['size_bytes']>0 for item in files)
    assert json.loads((editor.root/'designs'/'frame.JSON').read_text())==blank
    for item in files:
        opened=json.loads(request(editor,'/api/open?path='+urllib.parse.quote(item['path']))[1])
        assert opened['document']==blank and opened['path']==item['path']
        assert 'tubeclamp.TC101C' in opened['library']


def test_design_list_does_not_follow_external_links(editor,blank,tmp_path):
    directory=editor.root/'designs';directory.mkdir()
    outside=tmp_path.parent/(tmp_path.name+'-outside');outside.mkdir()
    write(outside/'secret.yaml',blank)
    try:
        (directory/'external').symlink_to(outside,target_is_directory=True)
        (directory/'external.yaml').symlink_to(outside/'secret.yaml')
    except OSError: pytest.skip('Creating symbolic links requires Windows Developer Mode')
    assert json.loads(request(editor,'/api/designs')[1])=={'designs':[]}


def test_open_loads_custom_library_and_relative_mesh_from_design_directory(editor,blank):
    from pipesim.server import BUNDLED_MESHES
    folder=editor.root/'designs'/'custom';folder.mkdir(parents=True)
    mesh=next(BUNDLED_MESHES.glob('*.stl'))
    (folder/'fitting.stl').write_bytes(mesh.read_bytes())
    definition={'name':'My fitting','kind':'connector','mass_kg':1,'geometry':[{'type':'mesh','file':'fitting.stl'}]}
    write(folder/'parts.yaml',{'format':'pipesim-library/1','name':'Custom parts','parts':{'custom.fitting':definition}})
    doc={**blank,'libraries':['parts.yaml'],'parts':[{'id':'fitting','catalog':'custom.fitting'}]}
    write(folder/'frame.pipe.yaml',doc)
    opened=json.loads(request(editor,'/api/open?path=designs/custom/frame.pipe.yaml')[1])
    assert opened['library']['custom.fitting']==definition
    shape=opened['scene']['parts'][0]['geometry'][0]
    assert request(editor,shape['url'])[1]==mesh.read_bytes()


@pytest.mark.parametrize('filename,text',[
    ('from-os.pipe.yaml','format: pipesim/1\nunits: mm-kg-s-N-deg\nname: From disk\nparts: []'),
    ('from-os.JSON',json.dumps({'format':'pipesim/1','units':'mm-kg-s-N-deg','name':'From disk','parts':[]})),
])
def test_file_open_is_in_memory_until_saved(editor,filename,text):
    result=json.loads(request(editor,'/api/open-file',{'filename':filename,'text':text})[1])
    assert result['imported'] is True and result['path']=='designs/'+filename
    assert result['document']['name']=='From disk' and result['scene']['parts']==[]
    assert 'tubeclamp.TC101C' in result['library']
    assert not (editor.root/result['path']).exists()
    request(editor,'/api/save',{'document':result['document'],'path':result['path']})
    assert (editor.root/result['path']).is_file()


def test_file_open_avoids_overwriting_names_and_ignores_os_path(editor,blank):
    write(editor.root/'designs'/'existing.pipe.yaml',blank)
    result=json.loads(request(editor,'/api/open-file',{'filename':'C:\\Users\\Ash\\existing.pipe.yaml','text':json.dumps({**blank,'name':'Imported'})})[1])
    assert result['path']=='designs/existing.pipe-imported-1.yaml'
    assert not (editor.root/result['path']).exists()
    assert json.loads(request(editor,'/api/open?path=designs/existing.pipe.yaml')[1])['document']==blank


@pytest.mark.parametrize('filename,text',[
    ('not-a-design.yaml','just: [broken'),('not-a-design.json','{}'),('mesh.stl','{}'),
    ('unknown.pipe.yaml',json.dumps({'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[{'id':'bad','catalog':'missing.part'}]})),
])
def test_file_open_rejects_invalid_designs_without_writing(editor,filename,text):
    with pytest.raises(urllib.error.HTTPError) as error:
        request(editor,'/api/open-file',{'filename':filename,'text':text})
    assert error.value.code==400
    assert not (editor.root/'designs').exists()


def test_file_open_reports_missing_companion_assets(editor,blank):
    doc={**blank,'parts':[{'id':'mesh','body':{'geometry':[{'type':'mesh','file':'missing.stl'}],'mass_kg':1}}]}
    with pytest.raises(urllib.error.HTTPError) as error:
        request(editor,'/api/open-file',{'filename':'frame.yaml','text':json.dumps(doc)})
    message=json.loads(error.value.read())['error']
    assert error.value.code==400 and 'missing.stl' in message and 'companion folders' in message

def test_exported_html_relative_assets_are_served(editor):
    path=editor.root/'output'/'book'; path.mkdir(parents=True)
    (path/'instructions.html').write_text('<img src="overview.png">')
    (path/'overview.png').write_bytes(b'image')
    assert b'overview.png' in request(editor,'/files/output/book/instructions.html')[1]
    assert request(editor,'/files/output/book/overview.png')[1]==b'image'

def test_nonfinite_json_geometry_is_rejected(editor):
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[{'id':'p','catalog':'generic.box','mass_kg':float('nan')}]}
    with pytest.raises(urllib.error.HTTPError) as error: request(editor,'/api/resolve',{'document':doc})
    assert error.value.code==400

def test_bundled_fitting_mesh_served_outside_design_workspace(editor):
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','parts':[{'id':'cross','catalog':'tubeclamp.TC161C'}]}
    scene=json.loads(request(editor,'/api/resolve',{'document':doc})[1])
    shape=scene['parts'][0]['geometry'][0]
    assert shape['url'].startswith('/builtin-mesh?')
    assert len(request(editor,shape['url'])[1])>1000

@pytest.mark.parametrize('path',['../tubeclamp.yaml','../../../server.py','tc161c.stl/../../tubeclamp.yaml'])
def test_bundled_mesh_route_cannot_escape_asset_directory(editor,path):
    with pytest.raises(urllib.error.HTTPError) as error:
        request(editor,'/builtin-mesh?path='+urllib.parse.quote(path))
    assert error.value.code==400


def test_connection_preview_and_commit_document_roundtrip(editor,blank):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'pipe','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},'pose':{'position_mm':[0,0,500]}},
                  {'id':'tee','catalog':'tubeclamp.TC104C','pose':{'position_mm':[200,0,500]}}]
    doc['anchors']=[{'part':'pipe','surface':'fixture'}]
    data=json.loads(request(editor,'/api/snap-options',{'document':doc,'member':'pipe','connector':'tee','port':'through','at_mm':500})[1])
    choice=next(o for o in data['options'] if o['move']==data['recommended'])
    assert choice['available'] and data['recommended']=='connector'
    assert not doc['joints']
    scene=json.loads(request(editor,'/api/resolve',{'document':choice['document']})[1])
    assert len(scene['groups'])==1
    assert next(p for p in scene['parts'] if p['id']=='tee')['pose']['position_mm']==[0,0,500]
    assert next(p for p in scene['parts'] if p['id']=='pipe')['length_mm']==1000


def test_move_endpoint_preserves_world_anchor(editor,blank):
    doc=copy.deepcopy(blank);doc['parts']=[{'id':'pipe','catalog':'tubeclamp.tube-C'}];doc['anchors']=[{'part':'pipe','surface':'fixture'}]
    with pytest.raises(urllib.error.HTTPError) as error:
        request(editor,'/api/move',{'document':doc,'poses':{'pipe':{'position_mm':[50,0,0]}}})
    assert error.value.code==400
    assert 'fixed to the world' in error.value.read().decode()
