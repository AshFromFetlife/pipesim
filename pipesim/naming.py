"""Editable display names; connection IDs remain stable."""
import copy

from .document import DocumentError


def part_name(assembly,pid):
    part=assembly.parts[pid]
    label=assembly.doc.get('metadata',{}).get('part_labels',{}).get(pid) or part.spec.get('label')
    if label: return label
    if not any(p['id']==pid for p in assembly.doc.get('parts',[])):
        owner=next((o for o in assembly.doc.get('objects',[]) if pid.startswith(o['id']+'/')),None)
        if owner and owner.get('label'): return owner['label']+' / '+pid[len(owner['id'])+1:]
    return pid


def rename(assembly,name,*,object_id=None,part_id=None,members=None):
    if not isinstance(name,str) or not name.strip() or len(name.strip())>120 or any(ord(c)<32 for c in name):
        raise DocumentError('Enter a name of 1–120 characters on one line')
    name=name.strip()
    if sum(v is not None for v in (object_id,part_id,members))!=1:
        raise DocumentError('Choose one object, part or Body to rename')
    doc=copy.deepcopy(assembly.doc)
    if object_id is not None:
        obj=next((o for o in doc.get('objects',[]) if o['id']==object_id),None)
        if obj is None: raise DocumentError('The object no longer exists')
        before=obj.get('label',object_id);obj['label']=name
    elif part_id is not None:
        if part_id not in assembly.parts: raise DocumentError('The part no longer exists')
        before=part_name(assembly,part_id)
        spec=next((p for p in doc['parts'] if p['id']==part_id),None)
        if spec is not None:
            spec['label']=name
            doc.get('metadata',{}).get('part_labels',{}).pop(part_id,None)
        else: doc.setdefault('metadata',{}).setdefault('part_labels',{})[part_id]=name
    else:
        if not isinstance(members,list) or not members or not all(isinstance(p,str) for p in members):
            raise DocumentError('Choose a Body to rename')
        members=set(members);direct={p['id'] for p in doc['parts']}
        bodies=[set(g)&direct for g in assembly.editor_groups()]
        if members not in bodies: raise DocumentError('This Body changed; select it again before renaming')
        labels=doc.setdefault('metadata',{}).setdefault('body_labels',[])
        record=next((r for r in labels if set(r['parts'])==members),None)
        before=record['label'] if record else None
        if record: record['label']=name
        else: labels.append({'parts':sorted(members),'label':name})
    if before==name: return {'document':copy.deepcopy(assembly.doc),'changed':False,'name':name}
    doc.pop('results',None);doc.pop('build_plan',None)
    return {'document':doc,'changed':True,'name':name}
