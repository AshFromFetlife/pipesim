"""Local-only editor service. No account, CDN, remote execution or cloud storage."""
from __future__ import annotations
import base64
import copy
import json
import mimetypes
import os
import re
import secrets
import threading
import traceback
import urllib.parse
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from .document import Assembly,Library,DocumentError,read,write,parse,plain,DATA

WEB=Path(__file__).parent/'web'
BUNDLED_MESHES=(DATA/'libraries'/'meshes').resolve()
DESIGN_SUFFIXES={'.yaml','.yml','.json'}
EDITOR_API_VERSION=13

def bundled_mesh(relative):
    path=(BUNDLED_MESHES/relative).resolve()
    if not path.is_relative_to(BUNDLED_MESHES) or path.suffix.lower()!='.stl':
        raise DocumentError('Invalid bundled mesh path')
    return path

class EditorServer(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,root,design,port):
        self.root=Path(root).resolve(); self.design=design; self.token=secrets.token_urlsafe(32)
        from .preview import PreviewCache
        self.previews=PreviewCache()
        from .simulation_jobs import SimulationJobs
        self.simulations=SimulationJobs()
        super().__init__(('127.0.0.1',port),Handler)
    def server_close(self):
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
        from .naming import part_name
        for p in result['parts']: p['label']=part_name(assembly,p['id'])
        from .chain import summary
        result['groups']=assembly.editor_groups()
        result['chains']=[summary(o,assembly.library) for o in assembly.doc.get('objects',[]) if o['template']=='chain']
        result['regroupable_objects']=regroup_candidates(assembly)
        for part in result['parts']:
            for shape in part['geometry']:
                if shape['type']=='mesh':
                    path=(assembly.parts[part['id']].base/shape['file']).resolve()
                    if path.is_relative_to(self.server.root): shape['url']='/asset?path='+urllib.parse.quote(path.relative_to(self.server.root).as_posix())
                    elif path.is_relative_to(BUNDLED_MESHES): shape['url']='/builtin-mesh?path='+urllib.parse.quote(path.relative_to(BUNDLED_MESHES).as_posix())
        self.server.previews.remember(assembly)
        return result
    def assembly(self,doc,base):
        for ref in doc.get('libraries',[]): self.server.path(str((base/ref).resolve()))
        assembly=Assembly.from_doc(doc,base,self.server.previews.library(doc,base))
        for part in assembly.parts.values():
            for shape in part.shapes:
                if shape['type']=='mesh':
                    path=(part.base/shape['file']).resolve()
                    if path.is_relative_to(BUNDLED_MESHES): bundled_mesh(path.relative_to(BUNDLED_MESHES))
                    else: self.server.path(str(path))
                    if not path.is_file(): raise FileNotFoundError(f'Mesh file not found: {shape["file"]}')
        return assembly
    def opened_design(self,doc,path):
        assembly=self.assembly(doc,path.parent)
        return {'document':doc,'scene':self.scene(assembly),'library':assembly.library.parts,
                'path':path.relative_to(self.server.root).as_posix()}
    def do_GET(self):
        if not self.trusted_host(): self.send_data(403,{'error':'Untrusted host'}); return
        request=urllib.parse.urlparse(self.path); query=urllib.parse.parse_qs(request.query)
        try:
            if request.path=='/api/bootstrap':
                path=self.server.path(self.server.design)
                if path.exists(): doc=read(path)
                else: doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','name':'Untitled creation','parts':[],'joints':[]}
                assembly=self.assembly(doc,path.parent)
                self.send_data(200,{'token':self.server.token,'api_version':EDITOR_API_VERSION,'path':path.relative_to(self.server.root).as_posix(),'document':doc,'scene':self.scene(assembly),'library':assembly.library.parts,'examples':[p.relative_to(self.server.root).as_posix() for p in sorted((self.server.root/'examples').glob('*.pipe.yaml'))]})
            elif request.path=='/api/open':
                path=self.server.path(query['path'][0])
                if path.suffix.lower() not in DESIGN_SUFFIXES: raise DocumentError('Open a .yaml, .yml or .json design')
                self.send_data(200,self.opened_design(read(path),path))
            elif request.path=='/api/designs':
                self.send_data(200,{'designs':self.server.designs()})
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
            elif route=='/api/parse':
                doc=parse(data['text']); assembly=self.assembly(doc,base)
                result={'document':doc,'scene':self.scene(assembly),'library':assembly.library.parts}
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
                source=self.server.path(data.get('source_path',data['path'])).parent
                self.assembly(data['document'],source)
                document=relocate_design(data['document'],source,path.parent)
                self.assembly(document,path.parent)
                write(path,document); result={'saved':path.relative_to(self.server.root).as_posix(),'document':document}
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
                    prepared=self.server.previews.prepared(doc,base,lambda:self.assembly(doc,base))
                    assembly=prepared.assembly
                else: assembly=self.assembly(doc,base)
                if route=='/api/resolve': result=self.scene(assembly)
                elif route=='/api/rename':
                    from .naming import rename
                    result=rename(assembly,data['name'],object_id=data.get('object'),part_id=data.get('part'),members=data.get('members'))
                    result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/delete':
                    from .deletion import delete_parts
                    result=delete_parts(assembly,data.get('members'),data.get('object'))
                    result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/duplicate':
                    from .duplication import duplicate
                    result=duplicate(assembly,data['selected'],data.get('scope','part'),data.get('count',1),data.get('grid_mm',1))
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
                        result={'document':result,'scene':self.scene(self.assembly(result,base))}
                elif route=='/api/drop-to-floor':
                    from .posing import drop_to_floor
                    result=drop_to_floor(assembly,data['selected'],data.get('object'),prepared=prepared)
                    result['scene']=self.scene(self.assembly(result['document'],base))
                elif route=='/api/resize':
                    from .resizing import resize_member
                    result=resize_member(assembly,data['member'],data['length_mm'],data.get('releases'))
                    if result['status']=='resized': result['scene']=self.scene(self.assembly(result['document'],base))
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
                elif route=='/api/snap-options':
                    from .snapping import connection_options
                    result=connection_options(assembly,data['member'],data['connector'],data['port'],
                        data.get('end','start'),data.get('insertion_mm'),data.get('at_mm'),data.get('locked',True),
                        data.get('poses'),data.get('replace_joint'),data.get('force',False),data.get('tolerance_mm',2.),data.get('force_options'))
                elif route=='/api/validate':
                    from .validation import validate
                    result=validate(assembly,build=data.get('build',False))
                elif route=='/api/analyse':
                    from .fea import analyse
                    result=analyse(assembly)
                elif route=='/api/simulate':
                    duration=float(data.get('duration',3))
                    if duration>30: raise DocumentError('Editor simulations are limited to 30 seconds; use the CLI for longer runs')
                    result=self.server.simulations.start(assembly,duration=duration,fps=30,
                        chain_links_per_body=data.get('chain_links_per_body',1))
                elif route=='/api/plan':
                    from .planning import plan_build
                    result=plan_build(assembly)
                elif route=='/api/fit':
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
                    from .editing import snapshot_design
                    result=snapshot_design(assembly,data['frame'])
                    from .grouping import restore_objects
                    result=restore_objects({'objects':[o for o in assembly.doc.get('objects',[]) if o['template']=='chain']},result,assembly.base,assembly.library)
                elif route in ('/api/export','/api/render'):
                    directory=self.server.root/'output'/('export-'+secrets.token_hex(4)); directory.mkdir(parents=True)
                    if route=='/api/export':
                        from .exporting import build_export
                        result=build_export(assembly,directory,engineering=data.get('engineering',False))
                        relative=(directory/'instructions.html').relative_to(self.server.root).as_posix()
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
    with EditorServer(root,design,port) as server:
        url=f'http://127.0.0.1:{server.server_port}'
        print(f'PipeSim editor: {url}\nWorkspace: {server.root}\nPress Ctrl+C to stop.',flush=True)
        if open_browser:
            import webbrowser
            webbrowser.open(url)
        try: server.serve_forever()
        except KeyboardInterrupt: pass
