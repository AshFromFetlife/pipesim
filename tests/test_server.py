import copy
import json
import socket
import threading
import urllib.request
import urllib.error
import urllib.parse
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


def test_editor_port_cannot_be_shared_by_another_server(editor):
    with pytest.raises(OSError):
        EditorServer(editor.root,editor.design,editor.server_port)


def test_editor_accepts_scene_asset_burst_while_editing(editor):
    # Hold accept briefly while a scene's asset requests and an edit queue up.
    # With the HTTPServer default backlog of five, the excess connections are
    # refused even though the server is still running.
    entered=threading.Event()
    release=threading.Event()
    original=editor.get_request

    def held_accept():
        entered.set()
        release.wait(timeout=10)
        return original()

    editor.get_request=held_accept
    clients=[]
    address=('127.0.0.1',editor.server_port)
    get=f'GET /index.html HTTP/1.0\r\nHost: 127.0.0.1:{editor.server_port}\r\n\r\n'.encode()
    try:
        first=socket.create_connection(address,timeout=2)
        first.settimeout(10)
        clients.append(first)
        assert entered.wait(timeout=2)
        for _ in range(24):
            client=socket.create_connection(address,timeout=2)
            client.settimeout(10)
            clients.append(client)
        release.set()
        for client in clients: client.sendall(get)
        for client in clients:
            response=b''
            while chunk:=client.recv(65536): response+=chunk
            assert response.startswith(b'HTTP/1.0 200')
    finally:
        release.set()
        editor.get_request=original
        for client in clients: client.close()
    assert request(editor,'/api/bootstrap')[0]==200


def test_autosave_keeps_bounded_recovery_copies_without_replacing_design(editor,blank):
    directory='designs/.autosaves'
    source='designs/work.pipe.yaml'
    for number in range(3):
        doc={**blank,'name':f'Revision {number}'}
        request(editor,'/api/autosave',{'document':doc,'path':source,'source_path':source,
                                       'directory':directory,'keep':2})
    records=json.loads(request(editor,'/api/autosaves?directory=designs%2F.autosaves')[1])['autosaves']
    assert len(records)==2
    assert not (editor.root/source).exists()
    latest=json.loads(request(editor,'/api/autosave?directory=designs%2F.autosaves&path='+
                               urllib.parse.quote(records[0]['path']))[1])
    assert latest['document']['name']=='Revision 2'
    assert latest['path']==source and latest['autosaved'] is True
    with pytest.raises(urllib.error.HTTPError) as error:
        request(editor,'/api/autosave',{'document':blank,'path':source,'source_path':source,
                                       'directory':'../outside','keep':2})
    assert error.value.code==400


def test_misaligned_mirror_draft_can_be_saved_reopened_and_moved_into_alignment(editor):
    from pipesim.document import Assembly, DocumentError
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','name':'Recoverable draft',
         'parts':[{'id':'connector','body':{'kind':'connector','mass_kg':.2,
             'geometry':[{'type':'sphere','radius_mm':5,'position_mm':[0,-80,0]}],
             'ports':{'socket':{'type':'socket','profile':'round','diameter_mm':42.4,
                 'position_mm':[0,0,0],'axis':[1,0,0],
                 'engagement_mm':40,'min_engagement_mm':15}}},
             'pose':{'position_mm':[300,0,100]}}],
         'joints':[],'draft_subassemblies':[{'id':'frame','runs':[{
             'id':'pipe','catalog':'tubeclamp.tube-C','start_mm':[-300,0,100],
             'end_mm':[300,0,100],
             'attachments':[{'connector':'connector','port':'socket',
                             'end':'start','insertion_mm':20}]}],
             'mirrors':[{'id':'middle','axis':'x','offset_mm':0,
                         'run_modes':{'pipe':'centered'}}]}]}
    with pytest.raises(DocumentError,match='centre the perpendicular pipe'):
        Assembly.from_doc(doc)
    scene=json.loads(request(editor,'/api/resolve',{'document':doc})[1])
    pipe=next(part for part in scene['parts'] if part['id']=='pipe')
    assert any(conflict['code']=='MIRROR_ALIGNMENT' for conflict in pipe['conflicts'])

    path='designs/recover.pipe.yaml'
    request(editor,'/api/save',{'document':doc,'path':path})
    opened=json.loads(request(editor,'/api/open?path='+path)[1])
    assert opened['document']['name']=='Recoverable draft'
    assert next(part for part in opened['scene']['parts'] if part['id']=='pipe')['conflicts']
    request(editor,'/api/autosave',{'document':doc,'path':path,'source_path':path,
                                   'directory':'designs/.autosaves','keep':2})
    autosaves=json.loads(request(editor,'/api/autosaves?directory=designs%2F.autosaves')[1])['autosaves']
    recovered=json.loads(request(editor,'/api/autosave?directory=designs%2F.autosaves&path='+
                                 urllib.parse.quote(autosaves[0]['path']))[1])
    assert recovered['document']['name']=='Recoverable draft'
    assert next(part for part in recovered['scene']['parts'] if part['id']=='pipe')['conflicts']
    request(editor,'/api/save',{'document':doc,'path':'design.pipe.yaml'})
    restarted=json.loads(request(editor,'/api/bootstrap')[1])
    assert next(part for part in restarted['scene']['parts'] if part['id']=='pipe')['conflicts']

    intermediate=json.loads(request(editor,'/api/move',{'document':doc,
        'poses':{'connector':{'position_mm':[250,0,100]}}})[1])
    assert next(part for part in intermediate['scene']['parts'] if part['id']=='pipe')['conflicts']
    corrected=json.loads(request(editor,'/api/move',{'document':intermediate['document'],
        'poses':{'connector':{'position_mm':[0,0,100]}}})[1])
    assert not next(part for part in corrected['scene']['parts'] if part['id']=='pipe')['conflicts']
    Assembly.from_doc(corrected['document'])


def test_save_and_autosave_preserve_a_document_even_when_it_cannot_be_resolved(editor,blank):
    doc=copy.deepcopy(blank)
    doc['parts']=[{'id':'unresolved','catalog':'missing.catalog',
                   'pose':{'position_mm':[12,34,56]}}]
    path='designs/unresolved.pipe.yaml'
    with pytest.raises(urllib.error.HTTPError):
        request(editor,'/api/resolve',{'document':doc})
    assert request(editor,'/api/save',{'document':doc,'path':path})[0]==200
    from pipesim.document import read
    assert read(editor.root/path)==doc
    request(editor,'/api/autosave',{'document':doc,'path':path,'source_path':path,
                                   'directory':'designs/.autosaves','keep':2})
    assert json.loads(request(editor,'/api/autosaves?directory=designs%2F.autosaves')[1])['autosaves']


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


def test_browser_video_api_exports_recording_and_serves_download(editor,load):
    from pipesim.math3d import pose_of
    from PIL import Image
    import io
    assembly=load('workbench')
    poses={pid:pose_of(p.matrix) for pid,p in assembly.parts.items()}
    moved=copy.deepcopy(poses)
    for pose in moved.values(): pose['position_mm'][0]+=200
    recording={'fps':10,'frames':[{'time_s':0,'parts':poses},{'time_s':.1,'parts':moved}]}
    payload={'document':assembly.doc,'source':'recording','recording':recording,'format':'gif','options':{'width':160,'height':160}}
    with pytest.raises(urllib.error.HTTPError) as error: request(editor,'/api/render-video',payload,token=False)
    assert error.value.code==403
    result=json.loads(request(editor,'/api/render-video',payload)[1])
    assert result['format']=='gif' and result['frames']==2 and result['fps']==10
    image=Image.open(io.BytesIO(request(editor,result['url'])[1]))
    assert image.n_frames==2 and image.size==(160,160)


def test_browser_video_api_rejects_missing_motion_and_invalid_format(editor,load):
    for extra in ({'source':'recording'}, {'format':'../escape'}, {'source':'animation','duration':301}):
        with pytest.raises(urllib.error.HTTPError) as error:
            request(editor,'/api/render-video',{'document':load('workbench').doc,**extra})
        assert error.value.code==400


def test_browser_video_api_renders_authored_animation_tracks(editor,load):
    doc=load('sliding-collar').doc
    assert doc['animation']['tracks']
    result=json.loads(request(editor,'/api/render-video',{'document':doc,'source':'animation','format':'gif',
        'duration':1,'fps':3,'options':{'width':100,'height':100}})[1])
    assert result['frames']==4 and result['fps']==3
    assert request(editor,result['url'])[0]==200


def test_human_import_is_textured_portable_and_survives_pose_snapshots(editor,blank):
    from test_human_assets import skinned_fixture,encoded
    data,_=skinned_fixture(vrm=True)
    payload={'filename':'person.gltf','content_base64':encoded(json.dumps(data).encode()),'path':'design.pipe.yaml'}
    with pytest.raises(urllib.error.HTTPError) as error: request(editor,'/api/import-human-model',payload,token=False)
    assert error.value.code==403
    imported=json.loads(request(editor,'/api/import-human-model',payload)[1])
    assert request(editor,imported['url'])[0]==200
    doc=copy.deepcopy(blank)
    doc['objects']=[{'id':'person','template':'human','render_model':imported['render_model']}]
    scene=json.loads(request(editor,'/api/resolve',{'document':doc})[1])
    assert len(scene['parts'])==19 and len(scene['humans'])==1
    assert scene['humans'][0]['render_model']['url']==imported['url']
    saved=json.loads(request(editor,'/api/save',{'document':doc,'source_path':'design.pipe.yaml','path':'designs/people/person.yaml'})[1])
    assert saved['document']['objects'][0]['render_model']['file'].startswith('../../.pipesim/')
    reopened=json.loads(request(editor,'/api/open?path=designs/people/person.yaml')[1])
    assert reopened['scene']['humans'][0]['render_model']['metadata']['bones']==imported['metadata']['bones']
    frame={'time_s':0,'parts':{p['id']:p['pose'] for p in scene['parts']},'joints':{}}
    snapshot=json.loads(request(editor,'/api/snapshot',{'document':doc,'frame':frame})[1])
    snapscene=json.loads(request(editor,'/api/resolve',{'document':snapshot})[1])
    assert snapscene['humans'][0]['render_model']['file']==imported['render_model']['file']
    regrouped=json.loads(request(editor,'/api/regroup',{'document':snapshot,'object':'person'})[1])
    assert regrouped['document']['objects'][0]['render_model']==imported['render_model']
    regrouped['document']['objects'][0]['render_model']['enabled']=False
    disabled=json.loads(request(editor,'/api/resolve',{'document':regrouped['document']})[1])
    assert disabled['humans'][0]['render_model']['metadata']==imported['metadata']


def test_human_assets_cannot_escape_workspace_or_load_remote_resources(editor,blank):
    from test_human_assets import skinned_fixture,encoded
    doc=copy.deepcopy(blank)
    for name in ('../outside.glb','.git/avatar.glb'):
        doc['objects']=[{'id':'person','template':'human','render_model':{'file':name}}]
        with pytest.raises(urllib.error.HTTPError) as error: request(editor,'/api/resolve',{'document':doc})
        assert error.value.code==400
    data,_=skinned_fixture()
    data['images'][0]['uri']='https://example.org/track.png'
    asset=editor.root/'unsafe.gltf';asset.write_text(json.dumps(data))
    doc['objects'][0]['render_model']['file']='unsafe.gltf'
    result=json.loads(request(editor,'/api/resolve',{'document':doc})[1])
    config=result['humans'][0]['render_model']
    assert 'selected model folder' in config['load_error'] and 'url' not in config
    assert len(result['parts'])==19


@pytest.mark.parametrize('missing', [True, False])
def test_disabled_human_model_can_resolve_save_and_reopen_without_reading_asset(editor,blank,missing):
    doc=copy.deepcopy(blank)
    config={'file':'optional.glb','enabled':False,'bone_map':{'pelvis':0},
            'extra_bones':{'Hair':{'mode':'damped_spring','mass':.1}}}
    if not missing: (editor.root/'optional.glb').write_bytes(b'incompatible appearance data')
    doc['objects']=[{'id':'person','template':'human','render_model':config}]
    scene=json.loads(request(editor,'/api/resolve',{'document':doc})[1])
    assert len(scene['parts'])==19 and not scene['humans'][0]['render_model']['enabled']
    request(editor,'/api/save',{'document':doc,'path':'saved.yaml'})
    reopened=json.loads(request(editor,'/api/open?path=saved.yaml')[1])
    assert reopened['document']['objects'][0]['render_model']==config
    doc['objects'][0]['render_model']['enabled']=True
    fallback=json.loads(request(editor,'/api/resolve',{'document':doc})[1])
    assert 'unavailable' in fallback['humans'][0]['render_model']['load_error']
    assert 'url' not in fallback['humans'][0]['render_model']
    assert len(fallback['parts'])==19


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
