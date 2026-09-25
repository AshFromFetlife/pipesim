"""Constraint-aware authoring helpers shared by GUI and CLI."""
import copy
import os
from pathlib import Path
import numpy as np
from .document import Assembly, DocumentError
from .math3d import align_axis, pose_of

def reference_path(path,base):
    """Prefer a readable relative path; Windows cannot relativize between drives."""
    path,base=Path(path).resolve(),Path(base).resolve()
    try: return Path(os.path.relpath(path,base)).as_posix()
    except ValueError: return path.as_posix()

def connect_member(doc,base,member,connector,port,end='start',insertion_mm=None,at_mm=None,locked=True):
    doc=copy.deepcopy(doc); a=Assembly.from_doc(doc,base)
    from .connections import socket_attachment
    b,insertion=socket_attachment(a,member,connector,port,end,insertion_mm,at_mm)
    tube=a.parts[member]; fitting=a.parts[connector]
    socket=fitting.ports.get(port,{})
    if not any(p['id']==member for p in doc['parts']): raise DocumentError('Expand this object before editing its internal parts')
    existing=[j for j in a.joints if j.get('type')=='socket' and j['b']['part']==member]
    target,axis=fitting.frame({'part':connector,'port':port}); target-=axis*insertion
    spec=next(p for p in doc['parts'] if p['id']==member)
    if existing:
        if len(existing)!=1 or socket.get('through') or existing[0]['b'].get('end')==end or 'end' not in existing[0]['b']:
            raise DocumentError('This move would disturb existing constraints. Use two opposite end sockets or detach the affected connection first.')
        old=existing[0]; start,_,oldaxis=a.joint_frames(old)
        vector=target-start; length=np.linalg.norm(vector)
        if length<=0 or abs(oldaxis@axis+1)>.002 or np.linalg.norm(np.cross(vector,oldaxis))>1:
            raise DocumentError('The two end sockets must face each other on one line')
        direction=vector/length if old['b']['end']=='start' else -vector/length
        matrix=tube.matrix.copy(); matrix[:3,:3]=align_axis(matrix[:3,2],direction)@matrix[:3,:3]; matrix[:3,3]=(start+target)/2
        spec.setdefault('parameters',{})['length_mm']=float(length)
    else:
        local,local_axis=tube.local_frame(b)
        matrix=tube.matrix.copy(); matrix[:3,:3]=align_axis(matrix[:3,:3]@local_axis,axis)@matrix[:3,:3]
        matrix[:3,3]=target-matrix[:3,:3]@local
    spec['pose']=pose_of(matrix)
    jid=f'{connector}-{port}-{member}'
    doc.setdefault('joints',[]).append({'id':jid,'type':'socket','a':{'part':connector,'port':port},'b':b,'insertion_mm':float(insertion),'locked':locked})
    doc.pop('results',None); doc.pop('build_plan',None)
    return doc

def expand_objects(assembly,object_id=None):
    doc=copy.deepcopy(assembly.doc); reference=copy.deepcopy(doc); reference.pop('state',None)
    neutral=Assembly.from_doc(reference,assembly.base,assembly.library)
    instances=[o for o in doc.get('objects',[]) if object_id is None or o['id']==object_id]
    if object_id is not None and not instances: raise DocumentError('Select an object to expand')
    if not instances: return doc
    prefixes=tuple(o['id']+'/' for o in instances)
    inside=lambda pid:pid.startswith(prefixes)
    records=doc.setdefault('expanded_objects',[])
    for instance in instances:
        members=[p for p in neutral.parts if p.startswith(instance['id']+'/')]
        if not members: continue
        root=instance['id']+'/pelvis' if instance['id']+'/pelvis' in members else members[0]
        records[:]=[r for r in records if r['instance']['id']!=instance['id']]
        records.append({'instance':{k:copy.deepcopy(v) for k,v in instance.items() if k!='components'},
                        'reference_part':root,'reference_pose':pose_of(neutral.parts[root].matrix),
                        'index':doc['objects'].index(instance)})
    doc['objects']=[o for o in doc.get('objects',[]) if o not in instances]
    if not doc['objects']: doc.pop('objects')
    direct={p['id'] for p in doc['parts']}
    doc['parts']=[{**copy.deepcopy(p.spec),'pose':pose_of(p.matrix)} for p in neutral.parts.values() if p.id in direct or inside(p.id)]
    for spec in doc['parts']:
        for shape in spec.get('body',{}).get('geometry',[]):
            if shape['type']=='mesh' and Path(shape['file']).is_absolute(): shape['file']=reference_path(shape['file'],assembly.base)
    known={j['id'] for j in doc.get('joints',[])}
    doc['joints']=[copy.deepcopy(j) for j in neutral.joints if j['id'] in known or inside(j['a']['part']) or inside(j['b']['part'])]
    doc['anchors']=[copy.deepcopy(a) for a in neutral.anchors if a in doc.get('anchors',[]) or inside(a['part'])]
    return doc

def relocate_design(doc,old_base,new_base):
    """Keep referenced assets valid when Save As changes the design directory."""
    from .document import Library
    old_base,new_base=Path(old_base).resolve(),Path(new_base).resolve()
    if old_base==new_base: return copy.deepcopy(doc)
    result=copy.deepcopy(doc); library=Library.load(doc.get('libraries',[]),old_base)
    def relative(path): return reference_path(old_base/path,new_base)
    result['libraries']=[relative(ref) for ref in doc.get('libraries',[])]
    for name,definition in result.get('definitions',{}).items():
        if name in library.parts: continue  # Overrides retain their source library's asset base.
        for shape in definition.get('geometry',[]):
            if shape['type']=='mesh': shape['file']=relative(shape['file'])
    specs=list(result.get('parts',[]))
    for instance in result.get('objects',[]): specs.extend(instance.get('components',{}).get('parts',[]))
    for spec in specs:
        if spec.get('catalog') in library.parts: continue
        for shape in spec.get('body',{}).get('geometry',[]):
            if shape['type']=='mesh': shape['file']=relative(shape['file'])
    result.pop('results',None); result.pop('build_plan',None)
    return result

def snapshot_design(assembly,frame):
    """Make the selected pose the editable zero position; keep readable limit offsets."""
    from .math3d import transform,point
    doc=expand_objects(assembly)
    doc.setdefault('joints',[])
    doc.pop('state',None); doc.pop('results',None); doc.pop('build_plan',None); doc.pop('animation',None)
    if set(frame['parts'])!=set(assembly.parts): raise DocumentError('Recording parts do not match this design')
    for p in doc['parts']: p['pose']=copy.deepcopy(frame['parts'][p['id']])
    broken=set(frame.get('broken_joints',[]))
    doc['joints']=[j for j in doc['joints'] if j['id'] not in broken]
    for j in doc['joints']:
        q=frame.get('joints',{}).get(j['id'],{})
        for coordinate,limit in j.get('limits',{}).items():
            value=q.get(coordinate,0)
            if coordinate=='rotation_deg':
                j['limits'][coordinate]=[[lo-v,hi-v] for (lo,hi),v in zip(limit,value if isinstance(value,list) else [0,0,0])]
            else: j['limits'][coordinate]=[v-value for v in limit]
        motor=j.get('motor',{})
        if motor.get('mode','position')=='position':
            offset=q.get('slide_mm',q.get('angle_deg',0))
            if 'target' in motor: motor['target']-=offset
            for key in motor.get('schedule',[]): key['target']-=offset
        if j['type']=='socket':
            a=assembly.parts[j['a']['part']]; b=assembly.parts[j['b']['part']]
            ma=transform(frame['parts'][a.id]); mb=transform(frame['parts'][b.id]); port=a.ports[j['a']['port']]
            mouth=point(ma,port['position_mm']); axis=ma[:3,:3]@np.array(port['axis'])
            if port.get('through'): j['b']={'part':b.id,'at_mm':float(point(np.linalg.inv(mb),mouth)[2]+b.length/2)}
            else: j['insertion_mm']=max(0,float((mouth-point(mb,b.local_frame(j['b'])[0]))@axis))
        j.setdefault('metadata',{})['snapshot_reference_coordinates']=q
    doc['drives']=[d for d in doc.get('drives',[]) if d['driver'] not in broken and d['follower'] not in broken]
    # Preserve the coupling's existing elastic displacement at the new origin.
    for drive in doc['drives']:
        driver=frame.get('joints',{}).get(drive['driver'],{}).get('angle_deg',0)
        follower=frame.get('joints',{}).get(drive['follower'],{})
        ratio=drive.get('ratio',1)*drive.get('sign',1)
        if 'slide_mm' in follower:
            if drive['type'] in ('gt2','rack'): ratio=drive.get('pitch_mm',2)*drive.get('teeth',20)/360*drive.get('sign',1)
            drive['offset_mm']=drive.get('offset_mm',0)+ratio*driver-follower['slide_mm']
        else: drive['offset_deg']=drive.get('offset_deg',0)+ratio*driver-follower.get('angle_deg',0)
    doc.setdefault('metadata',{})['snapshot']={'time_s':frame['time_s'],'source_input_sha256':assembly.input_hash,'coordinates':frame.get('joints',{}),'note':'Positions and velocities are separate: this design restarts at rest. Spherical joint axes and rectangular Euler limit ranges rebase at the snapshot; review anatomical limits after a large compound rotation.'}
    return doc
