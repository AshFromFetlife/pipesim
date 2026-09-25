"""Bounded assembly edits for Force: release screws, fit, then retighten.

A resizable pipe is represented in the search by two end frames joined by a
bounded virtual slider. This lets the existing hinge/loop IK solve cut lengths
and poses together. Virtual parts never enter the saved design or collision
model; final checks use the actual resized geometry.
"""
import copy
import itertools
import math
import time

import numpy as np

from .document import Assembly, DocumentError
from .math3d import pose_of, transform


def settings(tolerance_mm=2., options=None):
    values={'unlock_connectors':0,'resize_members':0,'max_length_change_mm':20.}
    if options is not None:
        if not isinstance(options,dict) or set(options)-set(values): raise DocumentError('Invalid Force options')
        values.update(options)
    values['tolerance_mm']=tolerance_mm
    for key,maximum in [('unlock_connectors',6),('resize_members',4)]:
        value=values[key]
        if isinstance(value,bool) or not isinstance(value,int) or not 0<=value<=maximum:
            raise DocumentError(f'{key} must be a whole number from 0 to {maximum}')
    for key,lo,hi in [('tolerance_mm',.01,20),('max_length_change_mm',.01,1000)]:
        value=values[key]
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi:
            raise DocumentError(f'{key} must be from {lo:g} to {hi:g} mm')
    return values


def _candidates(assembly,joint):
    distance={joint[e]['part']:0 for e in ('a','b')};queue=list(distance)
    adjacent={pid:set() for pid in assembly.parts}
    for j in assembly.joints:
        a,b=(j[e]['part'] for e in ('a','b'));adjacent[a].add(b);adjacent[b].add(a)
    for pid in queue:
        for other in sorted(adjacent[pid]):
            if other not in distance: distance[other]=distance[pid]+1;queue.append(other)
    editable={p['id'] for p in assembly.doc['parts']}
    screws={j['a']['part'] for j in assembly.doc.get('joints',[]) if j['type']=='socket' and j.get('locked') and j['a']['part'] in distance}
    anchors={a['part'] for a in assembly.anchors}
    members={pid for pid in distance if pid in editable and pid not in anchors and assembly.parts[pid].kind=='member' and assembly.parts[pid].length>0}
    # The virtual end model supports pipe-start stations and real end seats.
    # Custom centre frames/ports need a richer parameterized object definition.
    for j in [*assembly.joints,joint]:
        for e in ('a','b'):
            if j[e]['part'] in members and not ('end' in j[e] or 'at_mm' in j[e]): members.discard(j[e]['part'])
    order=lambda pid:(distance[pid],pid)
    return sorted(screws,key=order)[:12],sorted(members,key=order)[:12]


def _plans(connectors,members,controls):
    # Try the fewest edited parts first, then the combinations nearest the new
    # connection. A total time budget bounds combinatorial searches.
    for total in range(1,controls['unlock_connectors']+controls['resize_members']+1):
        choices=[]
        for nr in range(min(total,controls['resize_members'],len(members))+1):
            nu=total-nr
            if nu>min(controls['unlock_connectors'],len(connectors)): continue
            for unlocked in itertools.combinations(connectors,nu):
                for resized in itertools.combinations(members,nr):
                    rank=sum(connectors.index(p) for p in unlocked)+sum(members.index(p) for p in resized)
                    choices.append((rank,nr,unlocked,resized))
        for _,_,unlocked,resized in sorted(choices)[:64]: yield unlocked,resized


def _proxy(assembly,joint,placed,unlocked,resized,maximum):
    document=copy.deepcopy(assembly.doc)
    released=[]
    for j in document.get('joints',[]):
        if j['type']=='socket' and j.get('locked') and j['a']['part'] in unlocked:
            j['locked']=False;released.append(j['id'])
    working=Assembly.from_doc(document,assembly.base,assembly.library) if released else assembly
    proxy=copy.copy(working);proxy.parts={pid:copy.copy(p) for pid,p in working.parts.items()}
    proxy.joints=copy.deepcopy(working.joints);stretch={}
    adapted=copy.deepcopy(joint)
    for index,pid in enumerate(resized):
        part=working.parts[pid];tag=f'__fit_length_{index}'
        while tag in proxy.parts or any(j['id']==tag for j in proxy.joints): tag+='x'
        # Reject fixed meshes or definitions whose geometry ignores length.
        probe=copy.deepcopy(document)
        next(p for p in probe['parts'] if p['id']==pid).setdefault('parameters',{})['length_mm']=part.length+1
        if Assembly.from_doc(probe,assembly.base,assembly.library).parts[pid].shapes==part.shapes:
            raise DocumentError(f'{pid}: geometry does not support changing its cut length')
        end=copy.copy(part);end.id=tag;end.spec={**part.spec,'id':tag};proxy.parts[tag]=end
        for j in [*proxy.joints,adapted]:
            for e in ('a','b'):
                if j[e]['part']==pid and j[e].get('end')=='end': j[e]['part']=tag
        proxy.joints.append({'id':tag,'type':'prismatic','a':{'part':pid,'frame':{'axis':[0,0,1]}},
                             'b':{'part':tag},'limits':{'slide_mm':[-min(maximum,part.length-.01),maximum]}})
        stretch[pid]=tag
    intent=copy.copy(proxy);intent.parts={pid:copy.copy(p) for pid,p in proxy.parts.items()}
    for pid in intent.parts:
        if pid in placed.parts: intent.parts[pid].matrix=placed.parts[pid].matrix
    return working,proxy,adapted,intent,released,stretch


def _materialize(working,proxy,joint,fit,released,stretch,controls):
    from .editing import snapshot_design
    from .snapping import connection_collisions
    from .validation import validate
    poses,fitted,motions,context,angle=fit
    matrices={pid:transform(poses[pid]) if pid in poses else part.matrix.copy() for pid,part in proxy.parts.items()}
    document=copy.deepcopy(working.doc);resizes=[]
    for pid,virtual in stretch.items():
        start,end=matrices[pid],matrices[virtual]
        change=float((end[:3,3]-start[:3,3])@start[:3,2]);length=working.parts[pid].length+change
        if abs(change)>controls['max_length_change_mm']+1e-5 or length<=0: raise DocumentError('Requested cut-length limit exceeded')
        matrices[pid]=start.copy();matrices[pid][:3,3]+=start[:3,2]*(change/2)
        if abs(change)>1e-6:
            next(p for p in document['parts'] if p['id']==pid).setdefault('parameters',{})['length_mm']=length
            resizes.append({'part':pid,'before_mm':working.parts[pid].length,'after_mm':length,'change_mm':change})
        for e in ('a','b'):
            if fitted[e]['part']==virtual: fitted[e]['part']=pid
    dimensions=Assembly.from_doc(document,working.base,working.library)
    actual={pid:pose_of(matrices[pid]) for pid in working.parts}
    coordinates={m['joint']:{k:v for k,v in m.items() if k!='joint'} for m in motions if m['joint'] not in stretch.values()}
    document=snapshot_design(dimensions,{'time_s':0,'parts':actual,'joints':coordinates})
    animation=copy.deepcopy(working.doc.get('animation'))
    if animation:
        for track in animation.get('tracks',[]):
            shift=coordinates.get(track['joint'],{}).get(track['coordinate'],0)
            for key in track['keyframes']: key['value']-=shift
        document['animation']=animation
    relocked=[]
    for j in document['joints']:
        if j['id'] in released:
            j['locked']=True
            values=coordinates.get(j['id'],{})
            if any(np.max(np.abs(value))>1e-6 for value in values.values()):
                relocked.append({'connector':j['a']['part'],'joint':j['id'],'port':j['a'].get('port'),'action':'loosen_then_retighten'})
    adjusted={'unlocked':relocked,'resized':resizes}
    fitted.setdefault('metadata',{})['assembly_adjustments']=copy.deepcopy(adjusted)
    candidate=copy.deepcopy(document);candidate.setdefault('joints',[]).append(fitted)
    after=Assembly.from_doc(candidate,working.base,working.library)
    key=lambda issue:(issue['code'],tuple(sorted(issue['parts'])),issue.get('joint'))
    old_errors={key(i) for i in validate(working,collisions=False)['issues'] if i['severity']=='error'}
    for issue in validate(after,collisions=False)['issues']:
        if issue['severity']=='error' and key(issue) not in old_errors: raise DocumentError(issue['message'])
    changed={pid:pose for pid,pose in actual.items() if not np.allclose(matrices[pid],working.parts[pid].matrix,atol=1e-7,rtol=0)}
    relevant={*changed,*stretch,joint['a']['part'],joint['b']['part']}
    collisions=connection_collisions(working,after,relevant,{joint['a']['part'],joint['b']['part']})
    if collisions: raise DocumentError(f'{collisions[0][0]} would intersect {collisions[0][1]}')
    return {'document':document,'fit':(changed,fitted,[{'joint':jid,**values} for jid,values in coordinates.items()],
                                      sorted(set(context)&set(working.parts)),angle),'adjustments':adjusted}


def adjust_connection(assembly,joint,placed,controls):
    from .fitting import fit_connection
    from .snapping import move_document
    deadline=time.monotonic()+30
    fallback=None;best_gap=float('inf')
    def gap(result):
        poses,fitted,*_=result['fit']
        candidate=Assembly.from_doc(result['document'],assembly.base,assembly.library)
        for pid,pose in poses.items(): candidate.parts[pid].matrix=transform(pose)
        pa,pb,_=candidate.joint_frames(fitted)
        return float(np.linalg.norm(pb-pa))
    # Prefer an exact fit. A permitted residual is a fallback, not a reason to
    # skip screw/length adjustments that the user explicitly requested.
    try:
        fit=fit_connection(assembly,joint,placed,True,tolerance_mm=controls['tolerance_mm'],deadline=min(deadline,time.monotonic()+4))
        fallback={'document':move_document(assembly,fit[0]),'fit':fit,'adjustments':{'unlocked':[],'resized':[]}}
        best_gap=gap(fallback)
        if best_gap<=.03: return fallback
    except DocumentError as exc: failure=str(exc)
    else: failure=f'Closest unadjusted fit leaves {best_gap:.2f} mm.'
    connectors,members=_candidates(assembly,joint)
    attempts=0
    for unlocked,resized in _plans(connectors,members,controls):
        if attempts>=64 or time.monotonic()>deadline: break
        attempts+=1
        try:
            working,proxy,adapted,intent,released,stretch=_proxy(assembly,joint,placed,unlocked,resized,controls['max_length_change_mm'])
            fit=fit_connection(proxy,adapted,intent,True,tolerance_mm=controls['tolerance_mm'],
                               deadline=min(deadline,time.monotonic()+4),check_collisions=not stretch)
            result=_materialize(working,proxy,joint,fit,released,stretch,controls)
            remaining=gap(result)
            if remaining<=.03: return result
            if remaining<best_gap: fallback=result;best_gap=remaining
        except DocumentError as exc: failure=str(exc)
    if fallback is not None: return fallback
    raise DocumentError(f'No fit found within the allowed edits ({controls["unlock_connectors"]} connectors to loosen/retighten; '
                        f'{controls["resize_members"]} pipes to resize by at most {controls["max_length_change_mm"]:g} mm each). '
                        f'Tried {attempts} combinations. {failure}')
