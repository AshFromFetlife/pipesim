"""Delete explicit tree entries and clean references in one document edit."""
import copy

from .document import Assembly, DocumentError


def delete_parts(assembly, members=None, object_id=None):
    direct={p['id'] for p in assembly.doc['parts']}
    owners={o['id']:{pid for pid in assembly.parts if pid not in direct and pid.startswith(o['id']+'/')}
            for o in assembly.doc.get('objects',[])}
    if object_id is not None:
        if object_id not in owners: raise DocumentError('Select an existing object to delete')
        members=owners[object_id]
    else:
        if not isinstance(members,list) or not members or not all(isinstance(p,str) for p in members):
            raise DocumentError('Select parts to delete')
        members=set(members)
    if not members<=assembly.parts.keys(): raise DocumentError('A selected part no longer exists')
    for oid,owned in owners.items():
        if owned&members and not owned<=members:
            raise DocumentError(f'Expand {oid} before deleting individual parts, or delete the whole object')
    removed_objects={oid for oid,owned in owners.items() if owned<=members}
    removed_joints={j['id'] for j in assembly.joints if {j['a']['part'],j['b']['part']}&members}
    doc=copy.deepcopy(assembly.doc)
    if doc.get('state',{}).get('joints'):
        # Removing a posed attachment or anchor must not make survivors jump.
        # Reuse the editor's pose capture, including motor/track limit rebasing.
        from .posing import _editable
        doc=copy.deepcopy(_editable(assembly,members).doc)
    doc['parts']=[p for p in doc['parts'] if p['id'] not in members]
    doc['objects']=[o for o in doc.get('objects',[]) if o['id'] not in removed_objects]
    doc['joints']=[j for j in doc.get('joints',[]) if j['id'] not in removed_joints]
    for key in ('anchors','loads'):
        if key in doc: doc[key]=[item for item in doc[key] if item['part'] not in members]
    if 'state' in doc:
        doc['state']['joints']={jid:v for jid,v in doc['state'].get('joints',{}).items() if jid not in removed_joints}
    if 'animation' in doc:
        doc['animation']['tracks']=[t for t in doc['animation'].get('tracks',[]) if t['joint'] not in removed_joints]
    if 'drives' in doc:
        doc['drives']=[d for d in doc['drives'] if not {d['driver'],d['follower']}&removed_joints]
    build=doc.get('build',{})
    if 'sequence' in build: build['sequence']=[pid for pid in build['sequence'] if pid not in members]
    if 'fixtures' in build: build['fixtures']=[f for f in build['fixtures'] if f['part'] not in members]
    if 'tests' in doc:
        doc['tests']=[t for t in doc['tests'] if t.get('human') not in removed_objects
                      and t.get('seat') not in members and not any(p.startswith(t.get('human','')+'/') for p in members)]
    if 'expanded_objects' in doc:
        doc['expanded_objects']=[r for r in doc['expanded_objects'] if any(
            p['id'].startswith(r['instance']['id']+'/') for p in doc['parts'])]
    detached=doc.get('metadata',{}).get('detached_attachments')
    names=doc.get('metadata',{})
    if 'part_labels' in names: names['part_labels']={pid:label for pid,label in names['part_labels'].items() if pid not in members}
    if 'body_labels' in names: names['body_labels']=[r for r in names['body_labels'] if not set(r['parts'])&members]
    if detached is not None:
        doc['metadata']['detached_attachments']=[r for r in detached if not
            {r['joint']['a']['part'],r['joint']['b']['part']}&members]
    doc.pop('results',None);doc.pop('build_plan',None)
    if assembly.doc.get('state',{}).get('joints'):
        from .grouping import restore_objects
        doc=restore_objects({'objects':[o for o in assembly.doc.get('objects',[]) if o['id'] not in removed_objects]},
                            doc,assembly.base,assembly.library)
    Assembly.from_doc(doc,assembly.base,assembly.library)
    return {'document':doc,'deleted_parts':sorted(members),'deleted_joints':sorted(removed_joints)}
