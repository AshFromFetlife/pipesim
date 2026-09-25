"""Versioned YAML/JSON I/O, source libraries, object expansion and resolved assemblies."""
from __future__ import annotations
import copy
import hashlib
import json
import math
import os
import tempfile
from functools import lru_cache
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np
import yaml
import jsonschema
from .math3d import transform, point, unit, pose_of, UnionFind

DATA = Path(__file__).parent / "data"

class DocumentError(ValueError):
    pass

class UniqueLoader(yaml.SafeLoader):
    pass

# Dates are metadata strings, not implicit Python objects in an engineering file.
UniqueLoader.yaml_implicit_resolvers={key:[r for r in rules if r[0]!='tag:yaml.org,2002:timestamp'] for key,rules in yaml.SafeLoader.yaml_implicit_resolvers.items()}

def _mapping(loader, node, deep=False):
    loader.flatten_mapping(node)
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise DocumentError(f"Duplicate YAML key {key!r} at line {key_node.start_mark.line+1}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result

UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)

def _json_mapping(pairs):
    result={}
    for key,value in pairs:
        if key in result: raise DocumentError(f'Duplicate JSON key {key!r}')
        result[key]=value
    return result

def parse(text):
    if len(text) > 64*1024*1024:
        raise DocumentError("Document exceeds 64 MiB")
    try:
        if text.lstrip().startswith(('{','[')):
            # JSON scientific notation such as 1e-08 is a number. PyYAML's
            # YAML 1.1 resolver otherwise silently reads it as a string.
            try: result=json.loads(text,object_pairs_hook=_json_mapping)
            except json.JSONDecodeError: result=yaml.load(text,Loader=UniqueLoader)
        else: result = yaml.load(text, Loader=UniqueLoader)
    except (yaml.YAMLError,RecursionError) as exc:
        raise DocumentError(str(exc)) from exc
    if not isinstance(result, dict):
        raise DocumentError("The document must be a mapping")
    check_values(result)
    return result

def check_values(result):
    count=0
    def check(v, stack, depth=0):
        nonlocal count
        count+=1
        if count>1000000: raise DocumentError('Document exceeds one million values after alias expansion')
        if depth > 80:
            raise DocumentError("Document nesting exceeds 80 levels")
        if isinstance(v, float) and not math.isfinite(v):
            raise DocumentError("NaN and Infinity are not valid engineering values")
        if isinstance(v,(dict,list)):
            if id(v) in stack:
                raise DocumentError("Recursive YAML aliases are not supported")
            for item in (v.values() if isinstance(v,dict) else v):
                check(item, stack|{id(v)}, depth+1)
        elif v is not None and not isinstance(v,(str,int,float,bool)):
            raise DocumentError(f'Unsupported document value type: {type(v).__name__}')
    check(result,set())

def read(path):
    return parse(Path(path).read_text(encoding="utf-8-sig"))

def plain(value):
    if isinstance(value,np.ndarray): return value.tolist()
    if isinstance(value,np.generic): return value.item()
    if isinstance(value,Path): return str(value)
    if isinstance(value,dict): return {str(k):plain(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)): return [plain(v) for v in value]
    return value

def write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    data = plain(data)
    text = json.dumps(data, indent=2, allow_nan=False) if path.suffix.lower() == ".json" else yaml.safe_dump(data, sort_keys=False, allow_unicode=True, width=110)
    fd, temporary = tempfile.mkstemp(prefix="."+path.name, dir=path.parent)
    try:
        with os.fdopen(fd,"w",encoding="utf-8",newline="\n") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary,path)
    finally:
        if os.path.exists(temporary): os.unlink(temporary)

def fingerprint(doc):
    inputs = {k:v for k,v in doc.items() if k not in ("results","build_plan")}
    return hashlib.sha256(json.dumps(plain(inputs),sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()

@lru_cache(maxsize=2)
def _validator(name):
    schema = json.loads((DATA/"schemas"/f"{name}.schema.json").read_text())
    return jsonschema.Draft202012Validator(schema)

def check_schema(doc, name="design"):
    check_values(doc)
    errors = sorted(_validator(name).iter_errors(doc), key=lambda e: str(list(e.path)))
    if errors:
        raise DocumentError("\n".join(f"{'/'.join(map(str,e.path)) or '$'}: {e.message}" for e in errors[:20]))

def substitute(value, params):
    if isinstance(value, str) and value.startswith("$"):
        key = value[1:]
        if key not in params: raise DocumentError(f"Missing parameter {key}")
        return copy.deepcopy(params[key])
    if isinstance(value, dict): return {k:substitute(v,params) for k,v in value.items()}
    if isinstance(value, list): return [substitute(v,params) for v in value]
    return value

class Library:
    def __init__(self):
        self.parts, self.materials, self.objects = {}, {}, {}
        self.paths = {}
    def add(self, doc, base=None):
        check_schema(doc,"library")
        for key in ("parts","materials","objects"):
            target = getattr(self,key)
            for name, definition in doc.get(key,{}).items():
                if name in target: raise DocumentError(f"Duplicate library definition: {name}")
                target[name] = copy.deepcopy(definition)
                self.paths[name] = Path(base or DATA)
        return self
    @classmethod
    def load(cls, refs=(), base=None):
        library = cls()
        for path in sorted((DATA/"libraries").glob("*.yaml")):
            library.add(read(path),path.parent)
        for ref in refs:
            path = (Path(base or ".")/ref).resolve()
            library.add(read(path),path.parent)
        return library

@dataclass
class Part:
    id: str
    spec: dict
    definition: dict
    matrix: np.ndarray
    base: Path

    @property
    def kind(self): return self.definition.get("kind","rigid")
    @property
    def shapes(self): return self.definition.get("geometry",[])
    @property
    def ports(self): return self.definition.get("ports",{})
    @property
    def length(self): return float(self.definition.get("length_mm",self.spec.get("parameters",{}).get("length_mm",0)))
    @property
    def section(self): return self.definition.get("section",{})
    @property
    def center_of_mass(self):
        if 'center_of_mass_mm' in self.definition: return np.array(self.definition['center_of_mass_mm'],float)
        from .geometry import mesh_for_part
        mesh=mesh_for_part(self)
        center=mesh.center_mass if mesh.is_volume else mesh.centroid
        return np.array(center,float)
    @property
    def mass(self):
        if "mass_kg" in self.definition: return float(self.definition["mass_kg"])
        if "mass_per_m_kg" in self.definition: return self.length/1000*self.definition["mass_per_m_kg"]
        from .geometry import volume
        return sum(volume(s) for s in self.shapes)*1e-9*self.definition.get("material_data",{}).get("density_kg_m3",1000)
    def local_frame(self, endpoint):
        if "port" in endpoint:
            if endpoint["port"] not in self.ports: raise DocumentError(f"{self.id}: unknown port {endpoint['port']}")
            port = self.ports[endpoint["port"]]
            return np.array(port.get("position_mm",[0,0,0]),float),unit(port.get("axis",[0,0,1]))
        if self.kind == "member" and ("end" in endpoint or "at_mm" in endpoint):
            station = endpoint.get("at_mm",0 if endpoint.get("end") == "start" else self.length)
            return np.array([0.,0.,float(station)-self.length/2]),np.array([0.,0.,-1 if endpoint.get("end") == "end" else 1])
        frame = endpoint.get("frame",{})
        return np.array(frame.get("position_mm",[0,0,0]),float),unit(frame.get("axis",[0,0,1]))
    def frame(self, endpoint):
        pos,axis = self.local_frame(endpoint)
        return point(self.matrix,pos),self.matrix[:3,:3]@axis

@dataclass
class Assembly:
    doc: dict
    library: Library
    parts: dict[str,Part]
    joints: list[dict]
    base: Path
    anchors: list[dict] = field(default_factory=list)

    @property
    def input_hash(self):
        """Identify the resolved design, including external catalogue and mesh edits."""
        meshes={}
        for p in self.parts.values():
            for s in p.shapes:
                if s['type']=='mesh':
                    path=(p.base/s['file']).resolve()
                    meshes[p.id+':'+s['file']]=hashlib.sha256(path.read_bytes()).hexdigest()
        return fingerprint({'design':{k:v for k,v in self.doc.items() if k not in ('results','build_plan')},
            'parts':{pid:{'definition':p.definition,'pose':pose_of(p.matrix)} for pid,p in self.parts.items()},
            'joints':self.joints,'anchors':self.anchors,'meshes':meshes})

    @classmethod
    def load(cls,path):
        path=Path(path).resolve()
        return cls.from_doc(read(path),path.parent)
    @classmethod
    def from_doc(cls,doc,base=None,library=None):
        check_schema(doc)
        base=Path(base or ".").resolve()
        library = library or Library.load(doc.get("libraries",[]),base)
        library = copy.deepcopy(library)
        for name,definition in doc.get("definitions",{}).items():
            library.parts[name]=copy.deepcopy(definition)
            library.paths.setdefault(name,base)
        parts=copy.deepcopy(doc.get("parts",[]))
        joints=copy.deepcopy(doc.get("joints",[]))
        anchors=copy.deepcopy(doc.get("anchors",[]))
        object_ids=[o['id'] for o in doc.get('objects',[])]
        if len(set(object_ids))!=len(object_ids): raise DocumentError('Duplicate object id')
        for instance in doc.get("objects",[]):
            if 'components' in instance:
                from .grouping import object_components
                expanded=object_components(instance,library)
            elif instance["template"] == "human":
                from .human import humanoid
                expanded=humanoid(**instance.get("parameters",{}))
            elif instance['template'] == 'chain':
                from .chain import generate
                expanded=generate(instance.get('parameters',{}),library)
            else:
                if instance["template"] not in library.objects: raise DocumentError(f"Unknown object {instance['template']}")
                template=library.objects[instance["template"]]
                expanded=substitute(template,{**template.get("parameters",{}),**instance.get("parameters",{})})
            prefix=instance["id"]+"/"
            parent=transform(instance.get("pose"))
            for part in copy.deepcopy(expanded["parts"]):
                part["id"]=prefix+part["id"]
                part["pose"]=pose_of(parent@transform(part.get("pose")))
                if instance['template'] not in ('human','chain') and 'components' not in instance:
                    for shape in part.get('body',{}).get('geometry',[]):
                        if shape['type']=='mesh': shape['file']=str((library.paths[instance['template']]/shape['file']).resolve())
                parts.append(part)
            for joint in copy.deepcopy(expanded.get("joints",[])):
                joint["id"]=prefix+joint["id"]
                for end in ("a","b"): joint[end]["part"]=prefix+joint[end]["part"]
                joints.append(joint)
            for anchor in copy.deepcopy(expanded.get("anchors",[])):
                anchor["part"]=prefix+anchor["part"]
                anchors.append(anchor)
        resolved={}
        for spec in parts:
            if spec["id"] in resolved: raise DocumentError(f"Duplicate part id {spec['id']}")
            ref=spec.get("catalog")
            if ref and ref not in library.parts: raise DocumentError(f"Unknown catalog part {ref}")
            definition=copy.deepcopy(library.parts.get(ref,{}))
            params={**definition.get("parameters",{}),**spec.get("parameters",{})}
            definition=substitute(definition,params)
            definition.update(copy.deepcopy(spec.get("body",{})))
            for k in ("mass_kg","color","friction","restitution","center_of_mass_mm","inertia_kg_m2"):
                if k in spec: definition[k]=spec[k]
            material=definition.get("material")
            if material:
                if material not in library.materials: raise DocumentError(f"Unknown material {material}")
                definition["material_data"]=library.materials[material]
            if not definition.get("geometry"): raise DocumentError(f"{spec['id']}: body needs geometry")
            if "length_mm" in params: definition["length_mm"]=params["length_mm"]
            resolved[spec["id"]]=Part(spec["id"],spec,definition,transform(spec.get("pose")),library.paths.get(ref,base))
        ids=set()
        for joint in joints:
            if joint["id"] in ids: raise DocumentError(f"Duplicate joint id {joint['id']}")
            ids.add(joint["id"])
            for end in ("a","b"):
                ref=joint[end]["part"]
                if ref not in resolved: raise DocumentError(f"{joint['id']}: unknown part {ref}")
                resolved[ref].local_frame(joint[end])
            if joint["a"]["part"] == joint["b"]["part"]: raise DocumentError(f"{joint['id']}: a joint cannot connect a part to itself")
        for anchor in anchors:
            if anchor["part"] not in resolved: raise DocumentError(f"Anchor refers to unknown part {anchor['part']}")
        result=cls(copy.deepcopy(doc),library,resolved,joints,base,anchors)
        result.apply_coordinates(doc.get("state",{}).get("joints",{}))
        return result

    def rigid_groups(self):
        groups=UnionFind(self.parts)
        for joint in self.joints:
            if joint_kind(joint)=="fixed": groups.union(joint["a"]["part"],joint["b"]["part"])
        return groups.groups()

    def editor_groups(self):
        """Hold grouped chains in their current shape for layout, never physics."""
        groups=UnionFind(self.parts)
        for group in self.rigid_groups():
            for pid in group[1:]: groups.union(group[0],pid)
        for instance in self.doc.get('objects',[]):
            if instance['template']!='chain' or instance.get('layout_mode','rigid')!='rigid': continue
            members=[p for p in self.parts if p.startswith(instance['id']+'/')]
            for pid in members[1:]: groups.union(members[0],pid)
        return groups.groups()

    def joint_frames(self,joint):
        a,b=self.parts[joint["a"]["part"]],self.parts[joint["b"]["part"]]
        pa,axis=a.frame(joint["a"])
        pb,_=b.frame(joint["b"])
        if joint.get("type")=="socket": pa=pa-axis*joint.get("insertion_mm",0)
        return pa,pb,axis

    def apply_coordinates(self,coordinates):
        """Move the downstream connected component; reject coordinates on closed loops."""
        from scipy.spatial.transform import Rotation
        for jid,values in coordinates.items():
            joint=next((j for j in self.joints if j["id"]==jid),None)
            if joint is None: raise DocumentError(f"State refers to unknown joint {jid}")
            kind=joint_kind(joint)
            if kind=="fixed" and any(np.any(np.array(v)!=0) for v in values.values()): raise DocumentError(f"{jid} is locked")
            downstream={joint["b"]["part"]}
            for _ in self.parts:
                before=len(downstream)
                for other in self.joints:
                    if other["id"]==jid: continue
                    ends={other['a']['part'],other['b']['part']}
                    if ends&downstream: downstream|=ends
                if len(downstream)==before: break
            if joint["a"]["part"] in downstream: raise DocumentError(f"{jid}: motion in a closed loop requires explicit part poses")
            reverse=False
            if any(a["part"] in downstream for a in self.anchors):
                upstream={joint['a']['part']}
                for _ in self.parts:
                    for other in self.joints:
                        if other['id']==jid: continue
                        ends={other['a']['part'],other['b']['part']}
                        if ends&upstream: upstream|=ends
                if any(a['part'] in upstream for a in self.anchors): raise DocumentError(f"{jid}: motion would move a world anchor")
                downstream=upstream; reverse=True
            pa,pb,axis=self.joint_frames(joint)
            angle=float(values.get("twist_deg",values.get("angle_deg",0)))
            slide=float(values.get("slide_mm",0))
            if angle and kind not in ("revolute","cylindrical","spherical"): raise DocumentError(f"{jid} does not rotate")
            if slide and kind not in ("prismatic","cylindrical"): raise DocumentError(f"{jid} does not slide")
            from .math3d import axis_frame
            if "rotation_deg" in values:
                if kind!="spherical": raise DocumentError(f"{jid} does not have three rotational coordinates")
                basis=self.parts[joint['a']['part']].matrix[:3,:3]@Rotation.from_euler('xyz',joint['a'].get('frame',{}).get('rotation_deg',[0,0,0]),degrees=True).as_matrix()
                change=Rotation.from_euler('XYZ',values['rotation_deg'],degrees=True).as_matrix()
                r=basis@change@basis.T
            else: r=Rotation.from_rotvec(axis*np.deg2rad(angle)).as_matrix()
            delta=np.eye(4)
            delta[:3,:3]=r
            delta[:3,3]=pa-r@pa+axis*slide
            if reverse: delta=np.linalg.inv(delta)
            for pid in downstream: self.parts[pid].matrix=delta@self.parts[pid].matrix

    def scene(self):
        return {"name":self.doc.get("name","Untitled"), "parts":[{"id":p.id,"catalog":p.spec.get("catalog"),"kind":p.kind,"pose":pose_of(p.matrix),"geometry":p.shapes,"ports":p.ports,"section":p.section,"length_mm":p.length,"mass_kg":p.mass,"color":p.definition.get("color","#929faa"),"source":p.definition.get("source",{})} for p in self.parts.values()],"groups":self.rigid_groups(),"joints":self.joints,"anchors":self.anchors}

def joint_kind(joint):
    if joint.get("locked",joint.get("type")=="fixed"): return "fixed"
    return "cylindrical" if joint.get("type")=="socket" else joint.get("type","fixed")
