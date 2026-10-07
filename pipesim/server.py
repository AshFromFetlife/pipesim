"""Local-only editor service. No account, CDN, remote execution or cloud storage."""
from __future__ import annotations
import base64
import copy
import errno
import hashlib
import json
import mimetypes
import os
import re
import secrets
import socket
import threading
import time
import traceback
import urllib.parse
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from .document import Assembly,Library,DocumentError,read,write,parse,plain,DATA

WEB=Path(__file__).parent/'web'
BUNDLED_MESHES=(DATA/'libraries'/'meshes').resolve()
DESIGN_SUFFIXES={'.yaml','.yml','.json'}
EDITOR_API_VERSION=25

def bundled_mesh(relative):
    path=(BUNDLED_MESHES/relative).resolve()
    if not path.is_relative_to(BUNDLED_MESHES) or path.suffix.lower()!='.stl':
        raise DocumentError('Invalid bundled mesh path')
    return path

class EditorServer(ThreadingHTTPServer):
    daemon_threads=True
    allow_reuse_address=False
    # A scene may request many mesh assets while an edit POST is in flight.
    # The HTTPServer default backlog of five refuses otherwise valid local
    # connections on Windows during those short bursts.
    request_queue_size=128
    def server_bind(self):
        # Windows SO_REUSEADDR permits a second process to listen on the same
        # address. Requests can then reach either version after a "restart".
        if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):
            self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
        super().server_bind()
    def __init__(self,root,design,port):
        self.root=Path(root).resolve(); self.design=design; self.token=secrets.token_urlsafe(32)
        from .preview import PreviewCache
        self.previews=PreviewCache()
        from .simulation_jobs import SimulationJobs
        self.simulations=SimulationJobs()
        self.draft_jobs={}; self.draft_jobs_lock=threading.Lock()
        self.autosave_lock=threading.Lock()
        self.human_model_cache={}
        super().__init__(('127.0.0.1',port),Handler)
    def server_close(self):
        with self.draft_jobs_lock:
            for event in self.draft_jobs.values(): event.set()
        self.simulations.close()
        super().server_close()
    def path(self,relative):
        path=(self.root/relative).resolve()
        if not path.is_relative_to(self.root): raise DocumentError('Path must stay inside the editor workspace')
        if any(p in ('.git','.agents','.codex') for p in path.relative_to(self.root).parts): raise DocumentError('Repository control directories are not editor assets')
        return path
    def designs(self):
        directory=self.path('designs')
        result=[]
        def subdirectory(path):
            try:
                resolved=self.path(path)
                return resolved==path and resolved.is_relative_to(directory)
            except (ValueError,OSError): return False
        # Do not traverse linked directories or repository control directories.
        for parent,dirs,files in os.walk(directory,followlinks=False):
            dirs[:]=[name for name in dirs if not name.startswith('.') and subdirectory(Path(parent)/name)]
            for name in files:
                candidate=Path(parent)/name
                if candidate.suffix.lower() not in DESIGN_SUFFIXES or name.startswith('.'): continue
                try:
                    path=self.path(candidate)
                    if not path.is_relative_to(directory) or not path.is_file(): continue
                    result.append({'path':candidate.relative_to(self.root).as_posix(),'size_bytes':path.stat().st_size})
                except (ValueError,OSError): continue
        return sorted(result,key=lambda item:item['path'].casefold())

    def autosave_directory(self,relative):
        if not isinstance(relative,str): raise DocumentError('Choose an autosave folder')
        pieces=relative.replace('\\','/').split('/')
        if Path(relative).is_absolute() or not pieces or any(p.casefold() in ('','.','..','.git','.agents','.codex') for p in pieces):
            raise DocumentError('Autosaves must stay in a workspace folder')
        path=self.path(relative)
        if path==self.root: raise DocumentError('Choose an autosave folder')
        return path

    def autosaves(self,relative):
        directory=self.autosave_directory(relative)
        if not directory.exists(): return []
        return sorted((p for p in directory.glob('*.autosave.json') if p.is_file() and not p.is_symlink()),
                      key=lambda p:p.stat().st_mtime_ns,reverse=True)

class Handler(BaseHTTPRequestHandler):
    server: EditorServer
    def log_message(self,format,*args): pass
    def send_data(self,status,data,mime='application/json'):
        if mime=='application/json': data=json.dumps(plain(data),allow_nan=False).encode()
        elif isinstance(data,str): data=data.encode('utf-8')
        self.send_response(status); self.send_header('Content-Type',mime); self.send_header('Content-Length',str(len(data)))
        self.send_header('X-Content-Type-Options','nosniff'); self.send_header('Cache-Control','no-store'); self.end_headers(); self.wfile.write(data)
    def trusted_host(self):
        return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}')
    def file(self,path):
        if not path.is_file(): self.send_data(404,{'error':'File not found'}); return
        mime=mimetypes.guess_type(path)[0] or 'application/octet-stream'
        if path.suffix=='.js': mime='text/javascript'
        self.send_data(200,path.read_bytes(),mime)
    def scene(self,assembly):
        from .grouping import regroup_candidates
        result=assembly.scene()
        for human in result.get('humans',[]):
            config=human['render_model']
            path=self.server.path(str((assembly.base/config['file']).resolve()))
            error=getattr(assembly,'human_model_errors',{}).get(human['id'])
            if error:
                config.pop('url',None)
                config['load_error']=error
            else:
                config.pop('load_error',None)
                config['url']='/asset?path='+urllib.parse.quote(path.relative_to(self.server.root).as_posix())
        from .naming import part_name
        for p in result['parts']: p['label']=part_name(assembly,p['id'])
        from .chain import summary
        result['groups']=assembly.editor_groups()
        result['chains']=[summary(o,assembly.library) for o in assembly.doc.get('objects',[]) if o['template']=='chain']
        result['regroupable_objects']=regroup_candidates(assembly)
        from .drafting import preview, runs
        drafts=preview(assembly)
        result['parts'].extend(drafts)
        result['groups'].extend([[p['id']] for p in drafts])
        result['draft_attachments']=[{**a,'run':run['id']} for run in runs(assembly.doc)
                                     for a in run.get('attachments',[])]
        for part in result['parts']:
            for shape in part['geometry']:
                if shape['type']=='mesh':
                    base=assembly.parts[part['id']].base if part['id'] in assembly.parts else assembly.library.paths.get(part['catalog'],assembly.base)
                    path=(base/shape['file']).resolve()
                    if path.is_relative_to(self.server.root): shape['url']='/asset?path='+urllib.parse.quote(path.relative_to(self.server.root).as_posix())
                    elif path.is_relative_to(BUNDLED_MESHES): shape['url']='/builtin-mesh?path='+urllib.parse.quote(path.relative_to(BUNDLED_MESHES).as_posix())
        self.server.previews.remember(assembly)
        return result
    def assembly(self,doc,base,*,validate_mirror_geometry=True):
        for ref in doc.get('libraries',[]): self.server.path(str((base/ref).resolve()))
        from .human_assets import validate_render_model,validate_stored_asset
        model_errors={}
        instances=list(doc.get('objects',[]))+[entry['instance'] for entry in doc.get('expanded_objects',[])]
        for instance in instances:
            config=instance.get('render_model')
            if not config: continue
            validate_render_model(config)
            path=self.server.path(str((base/config['file']).resolve()))
            # The simplified mannequin remains editable/simulatable when an
            # optional appearance asset is offline, missing or incompatible.
            if config.get('enabled') is False: continue
            try:
                info=path.stat(); signature=(info.st_mtime_ns,info.st_size)
                if self.server.human_model_cache.get(path)!=signature:
                    validate_stored_asset(path)
                    self.server.human_model_cache[path]=signature
            except (DocumentError,OSError) as exc:
                model_errors[instance['id']]='Imported appearance unavailable; showing the simple mannequin. '+str(exc)
        assembly=Assembly.from_doc(doc,base,self.server.previews.library(doc,base),
                                   validate_mirror_geometry=validate_mirror_geometry)
        assembly.human_model_errors=model_errors
        for part in assembly.parts.values():
            for shape in part.shapes:
                if shape['type']=='mesh':
                    path=(part.base/shape['file']).resolve()
                    if path.is_relative_to(BUNDLED_MESHES): bundled_mesh(path.relative_to(BUNDLED_MESHES))
                    else: self.server.path(str(path))
                    if not path.is_file(): raise FileNotFoundError(f'Mesh file not found: {shape["file"]}')
        return assembly
    def scene_human_document(self,assembly):
        from .human_symmetry import sync_scene_humans
        errors=[]
        try:
            document=sync_scene_humans(assembly,errors=errors)
        except DocumentError as exc:
            # An existing attachment can prevent moving the person onto the
            # line. Keep the design open and report the unresolved constraint.
            return assembly.doc,assembly,False,str(exc)
        error=' '.join(errors) if errors else None
        if document is assembly.doc:
            return document,assembly,False,error
        return document,self.assembly(document,assembly.base,validate_mirror_geometry=False),True,error
    def opened_design(self,doc,path):
        assembly=self.assembly(doc,path.parent,validate_mirror_geometry=False)
        doc,assembly,updated,error=self.scene_human_document(assembly)
        return {'document':doc,'scene':self.scene(assembly),'library':assembly.library.parts,
                'path':path.relative_to(self.server.root).as_posix(),'auto_symmetry':updated,
                'mirror_pose_error':error}
    def do_GET(self):
        if not self.trusted_host(): self.send_data(403,{'error':'Untrusted host'}); return
        request=urllib.parse.urlparse(self.path); query=urllib.parse.parse_qs(request.query)
        try:
            if request.path=='/api/bootstrap':
                path=self.server.path(self.server.design)
                if path.exists(): doc=read(path)
                else: doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','name':'Untitled creation','parts':[],'joints':[]}
                assembly=self.assembly(doc,path.parent,validate_mirror_geometry=False)
                doc,assembly,updated,error=self.scene_human_document(assembly)
                self.send_data(200,{'token':self.server.token,'api_version':EDITOR_API_VERSION,'path':path.relative_to(self.server.root).as_posix(),'document':doc,'scene':self.scene(assembly),'library':assembly.library.parts,'auto_symmetry':updated,'mirror_pose_error':error,'examples':[p.relative_to(self.server.root).as_posix() for p in sorted((self.server.root/'examples').glob('*.pipe.yaml'))]})
            elif request.path=='/api/open':
                path=self.server.path(query['path'][0])
                if path.suffix.lower() not in DESIGN_SUFFIXES: raise DocumentError('Open a .yaml, .yml or .json design')
                self.send_data(200,self.opened_design(read(path),path))
            elif request.path=='/api/designs':
                self.send_data(200,{'designs':self.server.designs()})
            elif request.path=='/api/autosaves':
                directory=query.get('directory',['designs/.autosaves'])[0]
                records=[]
                for path in self.server.autosaves(directory):
                    try:
                        payload=json.loads(path.read_text(encoding='utf-8'))
                        records.append({'path':path.relative_to(self.server.root).as_posix(),
                                        'source_path':payload['source_path'],'saved_at':path.stat().st_mtime})
                    except (ValueError,KeyError,OSError): continue
                self.send_data(200,{'autosaves':records})
            elif request.path=='/api/autosave':
                directory=self.server.autosave_directory(query.get('directory',['designs/.autosaves'])[0])
                path=self.server.path(query['path'][0])
                if path.parent!=directory or not path.name.endswith('.autosave.json'):
                    raise DocumentError('Choose an autosave from the configured folder')
                payload=json.loads(path.read_text(encoding='utf-8'))
                source=self.server.path(payload['source_path'])
                assembly=self.assembly(payload['document'],source.parent,validate_mirror_geometry=False)
                document,assembly,updated,error=self.scene_human_document(assembly)
                self.send_data(200,{'document':document,'scene':self.scene(assembly),
                                    'library':assembly.library.parts,'path':payload['source_path'],'autosaved':True,
                                    'auto_symmetry':updated,'mirror_pose_error':error})
            elif request.path=='/asset': self.file(self.server.path(query['path'][0]))
            elif request.path=='/builtin-mesh': self.file(bundled_mesh(query['path'][0]))
            elif request.path.startswith('/files/'):
                self.file(self.server.path(urllib.parse.unquote(request.path[7:])))
            else:
                relative=request.path.lstrip('/') or 'index.html'; path=(WEB/relative).resolve()
                if not path.is_relative_to(WEB.resolve()): raise DocumentError('Invalid resource path')
                self.file(path)
        except (ValueError,OSError,KeyError,TypeError) as exc: self.send_data(400,{'error':str(exc)})
    def do_POST(self):
        if not self.trusted_host() or self.headers.get('X-PipeSim-Token')!=self.server.token:
            self.send_data(403,{'error':'Invalid editor session token'}); return
        origin=self.headers.get('Origin')
        if origin and origin not in (f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}'):
            self.send_data(403,{'error':'Cross-origin writes are not permitted'}); return
        try:
            size=int(self.headers.get('Content-Length','0'))
            if size<=0 or size>32*1024*1024: raise DocumentError('Request must be between 1 byte and 32 MiB')
            data=json.loads(self.rfile.read(size))
            route=urllib.parse.urlparse(self.path).path
            base=self.server.path(data.get('path',self.server.design)).parent
            if route in ('/api/simulation-status','/api/simulation-cancel','/api/simulation-result'):
                result=self.server.simulations.get(data['job_id'],cancel=route=='/api/simulation-cancel',
                                                   include_result=route=='/api/simulation-result')
            elif route in ('/api/draft-finalize-cancel','/api/draft-repair-cancel'):
                with self.server.draft_jobs_lock:
                    event=self.server.draft_jobs.get(data['job_id'])
                    if event: event.set()
                result={'cancelled':bool(event)}
            elif route in ('/api/draft-finalize','/api/draft-repair',
                           '/api/draft-finalize-selected','/api/draft-repair-selected'):
                from .drafting import finalize, repair, _check_cancelled
                job_id=data.get('job_id')
                event=threading.Event()
                if job_id:
                    if not re.fullmatch(r'[A-Za-z0-9_-]{8,80}',job_id): raise DocumentError('Invalid draft job id')
                    with self.server.draft_jobs_lock:
                        if job_id in self.server.draft_jobs: raise DocumentError('Draft job id already running')
                        self.server.draft_jobs[job_id]=event
                try:
                    assembly=self.assembly(data['document'],base,validate_mirror_geometry=False)
                    _check_cancelled(event.is_set)
                    operation=repair if route.startswith('/api/draft-repair') else finalize
                    selected=route.endswith('-selected')
                    run_id=data.get('run')
                    if selected and not run_id: raise DocumentError('Choose a draft run to finalize or repair')
                    if not selected and data.get('subassembly') is not None:
                        raise DocumentError('This editor tab is out of date. Save your work and refresh before finalizing a selected draft structure')
                    include_run_ids=()
                    blocking=None
                    if operation is finalize and any(g.get('mirrors') for g in assembly.doc.get('draft_subassemblies', [])):
                        from .symmetry import materialize_all
                        # A draft mirror is only a preview. Close recoverable
                        # mirror residuals while the original run is still
                        # editable, before materializing its reflected copies.
                        preflight=repair(assembly,cancelled=event.is_set,
                                         run_id=run_id if selected else None)
                        if preflight['status']=='conflict':
                            blocking=preflight
                        elif preflight['status']=='repaired':
                            assembly=self.assembly(preflight['document'],base,
                                                   validate_mirror_geometry=False)
                        owner=next((g['id'] for g in assembly.doc['draft_subassemblies'] if any(r['id']==run_id for r in g['runs'])),None) if selected else None
                        if selected:
                            from .drafting import _scope, runs
                            selected_before={r['id'] for g in _scope(assembly,run_id=run_id) for r in g['runs']}
                            existing={r['id'] for r in runs(assembly.doc)}
                        if blocking is None:
                            try:
                                document=materialize_all(assembly,{owner} if selected else None)
                            except DocumentError as exc:
                                blocking={'status':'conflict','conflicts':[{
                                    'code':'MIRROR_BAKE','message':str(exc)}]}
                            else:
                                if selected:
                                    include_run_ids=tuple(r['id'] for r in runs(document) if r['id'] not in existing and
                                        any(r['id'].startswith(source+'-mirror-') for source in selected_before))
                                assembly=self.assembly(document,base,validate_mirror_geometry=False)
                    result=blocking if blocking is not None else operation(
                        assembly,data.get('subassembly'),cancelled=event.is_set,
                        run_id=run_id if selected else None,
                        **({'include_run_ids':include_run_ids} if operation is finalize else {}))
                    if result['status'] in ('finalized','repaired'):
                        result['scene']=self.scene(self.assembly(result['document'],base,
                                                               validate_mirror_geometry=False))
                finally:
                    if job_id:
                        with self.server.draft_jobs_lock: self.server.draft_jobs.pop(job_id,None)
            elif route=='/api/parse':
                doc=parse(data['text']); assembly=self.assembly(doc,base,validate_mirror_geometry=False)
                doc,assembly,_,error=self.scene_human_document(assembly)
                result={'document':doc,'scene':self.scene(assembly),'library':assembly.library.parts,
                        'mirror_pose_error':error}
            elif route=='/api/open-file':
                # A browser supplies contents and a basename, never its original
                # OS directory. Resolve companion assets relative to designs/.
                name=re.sub(r'[^\w .-]','_',data['filename'].replace('\\','/').rsplit('/',1)[-1]).lstrip('. ')
                if Path(name).suffix.lower() not in DESIGN_SUFFIXES: raise DocumentError('Open a .yaml, .yml or .json design')
                path=self.server.path('designs/'+name)
                stem,suffix=path.stem,path.suffix
                index=1
                while path.exists():
                    path=path.with_name(f'{stem}-imported-{index}{suffix}'); index+=1
                doc=parse(data['text'])
                try: result=self.opened_design(doc,path)
                except (DocumentError,OSError) as exc:
                    raise DocumentError(f'{exc}\nFile open reads one file. For separate libraries or meshes, keep the design and its companion folders inside designs/ and open it from Load.') from exc
                result['imported']=True
            elif route=='/api/save':
                path=self.server.path(data['path'])
                if path.suffix.lower() not in DESIGN_SUFFIXES: raise DocumentError('Save a .yaml, .yml or .json design')
                from .editing import relocate_design
                from .document import check_values
                source=self.server.path(data.get('source_path',data['path'])).parent
                if not isinstance(data['document'],dict): raise DocumentError('The design must be a mapping')
                check_values(data['document'])
                document=relocate_design(data['document'],source,path.parent)
                write(path,document); result={'saved':path.relative_to(self.server.root).as_posix(),'document':document}
            elif route=='/api/autosave':
                directory=self.server.autosave_directory(data['directory'])
                source=self.server.path(data['source_path'])
                from .document import check_values
                if not isinstance(data['document'],dict): raise DocumentError('The design must be a mapping')
                check_values(data['document'])
                keep=data['keep']
                if isinstance(keep,bool) or not isinstance(keep,int) or not 1<=keep<=100:
                    raise DocumentError('Keep between 1 and 100 autosaves')
                digest=hashlib.sha256(data['source_path'].encode()).hexdigest()[:16]
                with self.server.autosave_lock:
                    directory.mkdir(parents=True,exist_ok=True)
                    path=directory/f'{digest}-{time.time_ns()}.autosave.json'
                    temp=directory/f'.{path.name}.{secrets.token_hex(4)}.tmp'
                    try:
                        temp.write_text(json.dumps({'source_path':data['source_path'],
                                                    'document':plain(data['document'])},allow_nan=False),encoding='utf-8')
                        temp.replace(path)
                    finally:
                        if temp.exists(): temp.unlink()
                    files=sorted(directory.glob(f'{digest}-*.autosave.json'),key=lambda p:p.stat().st_mtime_ns,reverse=True)
                    for old in files[keep:]: old.unlink()
                result={'saved':path.relative_to(self.server.root).as_posix()}
            elif route=='/api/import-human-model':
                from .human_assets import import_human_model
                directory=self.server.path('.pipesim/human-models')
                result=import_human_model(directory,base,data['filename'],data['content_base64'],data.get('files'))
                path=result.pop('asset')
                result['url']='/asset?path='+urllib.parse.quote(path.relative_to(self.server.root).as_posix())
            elif route=='/api/import-mesh':
                name=re.sub(r'[^A-Za-z0-9_.-]','_',data['filename'])
                directory=self.server.root/'.pipesim'/'imports'/secrets.token_hex(6); directory.mkdir(parents=True)
                source=directory/name; source.write_bytes(base64.b64decode(data['data'],validate=True))
                from .importing import import_mesh
                id=data.get('id','custom.'+source.stem)
                record=import_mesh(source,directory/'part.yaml',id,float(data['mass_kg']),float(data.get('scale',1)))
                doc=copy.deepcopy(data['document'])
                doc.setdefault('libraries',[]).append(Path(os.path.relpath(directory/'part.yaml',base)).as_posix())
                assembly=self.assembly(doc,base)
                result={'document':doc,'library':assembly.library.parts,'import':record}
            else:
                doc=data['document']; prepared=None
                if route in ('/api/move-object','/api/drop-to-floor') or (route=='/api/move' and data.get('selected')):
                    prepared=self.server.previews.prepared(doc,base,lambda:self.assembly(
                        doc,base,validate_mirror_geometry=route=='/api/drop-to-floor'))
                    assembly=prepared.assembly
                else: assembly=self.assembly(doc,base,validate_mirror_geometry=route not in ('/api/resolve','/api/move','/api/resize-drag'))
                if route=='/api/resolve':
                    document,assembly,updated,error=self.scene_human_document(assembly)
                    result=self.scene(assembly)
                    if updated: result['document']=document
                    if error: result['mirror_pose_error']=error
                elif route=='/api/draft-connect':
                    from .drafting import connect
                    document=connect(assembly,data['run'],data['connector'],data['port'],data.get('end','start'),data.get('insertion_mm'),data.get('at_mm'))
                    result={'document':document,'scene':self.scene(self.assembly(document,base))}
                elif route=='/api/draft-reopen':
                    from .drafting import reopen
                    result=reopen(assembly,data.get('members'))
                    result['scene']=self.scene(self.assembly(result['document'],base,validate_mirror_geometry=False))
                elif route=='/api/draft-mirror-bake':
                    from .symmetry import materialize_mirror
                    document=materialize_mirror(assembly,data['group'],data['plane'])
                    result={'document':document,'scene':self.scene(self.assembly(document,base))}
                elif route=='/api/draft-update':
                    from .drafting import runs
                    document=copy.deepcopy(doc)
                    run=next((r for r in runs(document) if r['id']==data['run']),None)
                    if run is None: raise DocumentError('Unknown draft run')
                    for key in ('start_mm','end_mm','locked_length_mm'):
                        if key in data:
                            if key=='locked_length_mm' and data[key] is None: run.pop(key,None)
                            else: run[key]=data[key]
                    document.pop('results',None); document.pop('build_plan',None)
                    result={'document':document,'scene':self.scene(self.assembly(document,base))}
                elif route=='/api/rename':
                    from .naming import rename
                    result=rename(assembly,data['name'],object_id=data.get('object'),part_id=data.get('part'),members=data.get('members'))
                    result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/delete':
                    from .deletion import delete_parts
                    result=delete_parts(assembly,data.get('members'),data.get('object'))
                    result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/duplicate':
                    from .duplication import duplicate, duplicate_draft
                    from .drafting import runs
                    if any(run['id']==data['selected'] for run in runs(doc)):
                        if data.get('scope','part')!='subassembly':
                            raise DocumentError('Choose the connected draft structure to duplicate')
                        result=duplicate_draft(assembly,data['selected'],data.get('count',1),data.get('grid_mm',1),data.get('offset_mm'))
                    else:
                        result=duplicate(assembly,data['selected'],data.get('scope','part'),data.get('count',1),data.get('grid_mm',1),data.get('offset_mm'))
                    result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/move':
                    if data.get('selected'):
                        from .posing import transform_part,commit_transform
                        # Older tabs still receive full preview documents. New
                        # tabs request only poses, then accept the exact stored
                        # solution on release without solving it a second time.
                        light=bool(data.get('preview') and data.get('pose_only'))
                        recalled=prepared.recalled(data) if not data.get('preview') else None
                        if recalled is not None:
                            result={**recalled,'document':commit_transform(assembly,recalled['poses'])}
                        else:
                            result=transform_part(assembly,data['selected'],data['target'],data.get('mode','translate'),data.get('seed'),preview=light,prepared=prepared)
                        if light: result=prepared.remember(data,result)
                        if not data.get('preview'): result['scene']=self.scene(self.assembly(result['document'],base))
                    else:
                        from .snapping import move_document
                        result=move_document(assembly,data['poses'])
                        result={'document':result,'scene':self.scene(self.assembly(
                            result,base,validate_mirror_geometry=False))}
                elif route=='/api/drop-to-floor':
                    from .posing import drop_to_floor
                    result=drop_to_floor(assembly,data['selected'],data.get('object'),prepared=prepared)
                    result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/resize':
                    from .resizing import resize_member
                    result=resize_member(assembly,data['member'],data['length_mm'],data.get('releases'))
                    if result['status']=='resized': result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/resize-drag':
                    from .resize_drag import resize_drag
                    result=resize_drag(assembly,data['member'],data['length_mm'],data['endpoint'],
                                       data.get('behavior','follow'),data.get('capture_mm',40),
                                       data.get('capture_deg',15),data.get('locked',True),data.get('auto_connect',True))
                    result['scene']=self.scene(self.assembly(result['document'],base,validate_mirror_geometry=False))
                elif route=='/api/move-object':
                    from .grouping import move_object
                    result=move_object(assembly,data['object'],data['target'],preview=bool(data.get('preview') and data.get('pose_only')))
                    if not data.get('preview'): result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/regroup':
                    from .grouping import regroup_object
                    document=regroup_object(assembly,data['object'])
                    result={'document':document,'scene':self.scene(self.assembly(document,base))}
                elif route=='/api/object-parameters':
                    from .grouping import update_object_parameters
                    document=update_object_parameters(assembly,data['object'],data['parameters'])
                    result={'document':document,'scene':self.scene(self.assembly(document,base))}
                elif route=='/api/human-symmetry':
                    from .human_symmetry import set_human_symmetry
                    document=set_human_symmetry(assembly,data['object'],data.get('group'),data.get('plane'))
                    result={'document':document,'scene':self.scene(self.assembly(document,base))}
                elif route=='/api/object-layout':
                    from .grouping import set_object_layout
                    document=set_object_layout(assembly,data['object'],data['mode'])
                    result={'document':document,'scene':self.scene(self.assembly(document,base))}
                elif route=='/api/detach-attachment':
                    from .grouping import detach_attachment
                    document=detach_attachment(assembly,data['object'],data['joint'])
                    result={'document':document,'scene':self.scene(self.assembly(document,base))}
                elif route=='/api/release-object-anchor':
                    from .grouping import release_object_anchor
                    document=release_object_anchor(assembly,data['object'],data['part'])
                    result={'document':document,'scene':self.scene(self.assembly(document,base))}
                elif route=='/api/attach-part':
                    from .grouping import attach_part
                    result=attach_part(assembly,data['object'],data.get('part'),data.get('target'),data.get('type','revolute'),data.get('reconnect'),data.get('part_port'))
                    if not data.get('preview'): result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/connect-ports':
                    from .port_connections import connect_ports
                    result=connect_ports(assembly,data.get('a'),data.get('b'),
                        data.get('type','revolute'),data.get('move','auto'),
                        data.get('poses'),bool(data.get('preview')))
                    if not data.get('preview'):
                        result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/add-wheel':
                    from .wheels import add_wheel
                    result=add_wheel(assembly,data.get('catalog','generic.wheel'),
                        data.get('parameters'),data.get('target'),data.get('pose'))
                    result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/mount-wheel':
                    from .wheels import mount_wheel
                    result=mount_wheel(assembly,data.get('wheel'),data.get('target'))
                    result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/snap-options':
                    from .snapping import connection_options
                    result=connection_options(assembly,data['member'],data['connector'],data['port'],
                        data.get('end','start'),data.get('insertion_mm'),data.get('at_mm'),data.get('locked',True),
                        data.get('poses'),data.get('replace_joint'),data.get('force',False),data.get('tolerance_mm',2.),data.get('force_options'))
                elif route=='/api/validate':
                    if doc.get('draft_subassemblies'): raise DocumentError('Finalize draft subassemblies before validation')
                    from .validation import validate
                    result=validate(assembly,build=data.get('build',False))
                elif route=='/api/analyse':
                    if doc.get('draft_subassemblies'): raise DocumentError('Finalize draft subassemblies before analysis')
                    from .fea import analyse
                    result=analyse(assembly)
                elif route=='/api/simulate':
                    if doc.get('draft_subassemblies'): raise DocumentError('Finalize draft subassemblies before simulation')
                    duration=float(data.get('duration',3))
                    if duration>30: raise DocumentError('Editor simulations are limited to 30 seconds; use the CLI for longer runs')
                    result=self.server.simulations.start(assembly,duration=duration,fps=30,
                        chain_links_per_body=data.get('chain_links_per_body',1),
                        deflection_warning_mm=float(data.get('deflection_warning_mm',10)))
                elif route=='/api/plan':
                    if doc.get('draft_subassemblies'): raise DocumentError('Finalize draft subassemblies before build planning')
                    from .planning import plan_build
                    result=plan_build(assembly)
                elif route=='/api/fit':
                    assembly.require_finished('fit tests')
                    from .human import reach,seat_fit,run_fit_tests
                    if data.get('target'): result=reach(assembly,data['human'],data['target'],data.get('hand','right'))
                    elif data.get('seat'): result=seat_fit(assembly,data['human'],data['seat'])
                    else: result=run_fit_tests(assembly)
                elif route=='/api/connect':
                    from .editing import connect_member
                    result=connect_member(doc,base,data['member'],data['connector'],data['port'],data.get('end','start'),data.get('insertion_mm'),data.get('at_mm'),data.get('locked',True))
                    result={'document':result,'scene':self.scene(self.assembly(result,base))}
                elif route=='/api/expand':
                    from .editing import expand_objects
                    result=expand_objects(assembly,data.get('object'))
                elif route=='/api/snapshot':
                    assembly.require_finished('capturing a simulation frame')
                    from .editing import snapshot_design
                    result=snapshot_design(assembly,data['frame'])
                    from .grouping import restore_objects
                    result=restore_objects({'objects':[o for o in assembly.doc.get('objects',[]) if o['template']=='chain']},result,assembly.base,assembly.library)
                elif route in ('/api/export','/api/render','/api/render-video'):
                    if route=='/api/export' and doc.get('draft_subassemblies'): raise DocumentError('Finalize draft subassemblies before exporting a build book')
                    directory=self.server.root/'output'/('export-'+secrets.token_hex(4)); directory.mkdir(parents=True)
                    if route=='/api/export':
                        from .exporting import build_export
                        result=build_export(assembly,directory,engineering=data.get('engineering',False))
                        relative=(directory/'instructions.html').relative_to(self.server.root).as_posix()
                    elif route=='/api/render-video':
                        from .rendering import render_video,animation_frames
                        format=data.get('format','mp4')
                        if format not in ('mp4','gif'): raise DocumentError('Choose MP4 or GIF for browser video export')
                        source=data.get('source','recording')
                        if source=='recording':
                            recording=data.get('recording') or doc.get('results',{}).get('simulate')
                            if not recording or not recording.get('frames'): raise DocumentError('Run a simulation before exporting its recording')
                        elif source=='animation':
                            duration=float(data.get('duration',4)); fps=float(data.get('fps',30))
                            if not 0<duration<=300 or not 1<=fps<=120: raise DocumentError('Use a duration up to 300 seconds and a frame rate from 1 to 120')
                            recording=animation_frames(assembly,duration=duration,fps=fps)
                        else: raise DocumentError('Choose a simulation recording or authored animation')
                        target=directory/('render.'+format)
                        render_video(assembly,target,recording,**data.get('options',{}))
                        relative=target.relative_to(self.server.root).as_posix()
                        result={'format':format,'frames':len(recording['frames']),'fps':recording.get('fps',30)}
                    else:
                        from .rendering import render_image
                        render_image(assembly,directory/'render.png',**data.get('options',{}))
                        relative=(directory/'render.png').relative_to(self.server.root).as_posix(); result={}
                    result['url']='/files/'+urllib.parse.quote(relative)
                else:
                    self.send_data(404,{'error':'Unknown operation'}); return
            self.send_data(200,result)
        except (ValueError,OSError,KeyError,TypeError,RuntimeError) as exc:
            self.send_data(400,{'error':str(exc)})
        except Exception as exc:
            traceback.print_exc(); self.send_data(500,{'error':f'Operation failed: {exc}'})

def serve(root,design,port=8765,open_browser=False):
    try:
        editor_server=EditorServer(root,design,port)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE or getattr(exc,'winerror',None) == 10048:
            raise OSError(f'Port {port} is already in use. Stop the PipeSim server using that port before starting another one.') from exc
        raise
    with editor_server as server:
        url=f'http://127.0.0.1:{server.server_port}'
        print(f'PipeSim editor: {url}\nWorkspace: {server.root}\nPress Ctrl+C to stop.',flush=True)
        if open_browser:
            import webbrowser
            webbrowser.open(url)
        try: server.serve_forever()
        except KeyboardInterrupt: pass
