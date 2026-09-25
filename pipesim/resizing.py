"""Resize cut members while preserving connections and fixed world supports.

Length edits translate connected parts without changing their rotations or other
cut lengths. Sliding joints provide bounded freedom; rigid loops remain closed.
"""
import copy
import itertools
import math
import numpy as np
from scipy.linalg import null_space
from scipy.optimize import linprog, minimize, LinearConstraint
from .document import Assembly, DocumentError, joint_kind
from .math3d import pose_of

TOL=1e-5


def _component(assembly,member):
    connected={member}; editable={p['id'] for p in assembly.doc['parts']}
    objects=[{pid for pid in assembly.parts if pid.startswith(o['id']+'/') and pid not in editable} for o in assembly.doc.get('objects',[])]
    edges=[{j['a']['part'],j['b']['part']} for j in assembly.joints]+objects
    while True:
        previous=len(connected)
        for edge in edges:
            if edge&connected: connected|=edge
        if len(connected)==previous: return connected,objects


def _translations(before,after,member):
    connected,objects=_component(after,member)
    names=sorted(connected); index={pid:i*3 for i,pid in enumerate(names)}; size=len(names)*3
    rows=[]; targets=[]; labels=[]; inequalities=[]; lower=[]; upper=[]; bounds_labels=[]
    def row(a,b,axis):
        result=np.zeros(size)
        if a is not None: result[index[a]:index[a]+3]-=axis
        if b is not None: result[index[b]:index[b]+3]+=axis
        return result
    def equal(a,b,axis,value,label):
        rows.append(row(a,b,axis));targets.append(value);labels.append(label)
    def bounded(a,b,axis,constant,lo,hi,label):
        inequalities.append(row(a,b,axis));lower.append(lo-constant);upper.append(hi-constant);bounds_labels.append(label)
    identity=np.eye(3); old_joints={j['id']:j for j in before.joints}
    for j in after.joints:
        aid,bid=j['a']['part'],j['b']['part']
        if aid not in connected: continue
        pa,pb,axis=after.joint_frames(j); olda,oldb,_=before.joint_frames(old_joints[j['id']])
        delta=(pb-pa)-(oldb-olda);kind=joint_kind(j)
        # Socket metadata defines the intended seat, including repairing an old
        # length edit that left an otherwise valid fitting behind.
        alignment=pb-pa if j['type']=='socket' else delta
        directions=identity-np.outer(axis,axis) if kind in ('prismatic','cylindrical') else identity
        for direction in directions: equal(aid,bid,direction,-direction@alignment,j['id'])
        if kind in ('prismatic','cylindrical'):
            lo,hi=j.get('limits',{}).get('slide_mm',[-np.inf,np.inf])
            if np.isfinite([lo,hi]).any(): bounded(aid,bid,axis,axis@delta,lo,hi,j['id'])
        if j['type']=='socket':
            fitting,tube=after.parts[aid],after.parts[bid];port=fitting.ports[j['a']['port']]
            mouth,_=fitting.frame(j['a'])
            if port.get('through'):
                along=tube.matrix[:3,2];station=(mouth-tube.matrix[:3,3])@along+tube.length/2
                half=port.get('engagement_mm',0)/2
                bounded(bid,aid,along,station,half,tube.length-half,j['id'])
            else:
                end,_=tube.frame(j['b']);depth=(mouth-end)@axis
                bounded(bid,aid,axis,depth,port.get('min_engagement_mm',0),port.get('engagement_mm',0),j['id'])
    # Keep reusable objects intact. Their poses can move without expanding their
    # definitions or silently changing an anatomical pose during a length edit.
    for ids in objects:
        ids=sorted(ids&connected)
        for pid in ids[1:]:
            for axis in identity: equal(ids[0],pid,axis,0,'object:'+pid.rsplit('/',1)[0])
    anchors=[a for a in after.anchors if a['part'] in connected]
    for anchor in anchors:
        for axis in identity: equal(None,anchor['part'],axis,0,'anchor:'+anchor['part'])
    if not anchors:
        for axis in identity: equal(None,member,axis,0,'reference:'+member)
    A=np.array(rows);b=np.array(targets);x=np.linalg.lstsq(A,b,rcond=None)[0]
    residual=np.abs(A@x-b)
    if residual.max(initial=0)>TOL:
        conflicts=list(dict.fromkeys(label for label,error in zip(labels,residual) if error>TOL))
        reason='World anchors prevent the connected parts from following this length change.' if any(c.startswith('anchor:') for c in conflicts) else 'These connections form a closed path. Changing this side alone cannot preserve the other lengths and angles.'
        return None,conflicts,reason
    if inequalities:
        G=np.array(inequalities);lo=np.array(lower);hi=np.array(upper);values=G@x
        failed=(values<lo-TOL)|(values>hi+TOL)
        if failed.any():
            free=null_space(A);reasons=list(dict.fromkeys(label for label,bad in zip(bounds_labels,failed) if bad))
            reason='The requested length exceeds the available slide travel or socket engagement. A fitting must remain on the pipe and within its insertion limits.'
            if not free.shape[1] or np.any(lo>hi+TOL): return None,reasons,reason
            C=G@free;low=lo-values;high=hi-values
            upper_mask=np.isfinite(high);lower_mask=np.isfinite(low)
            feasible=linprog(np.zeros(free.shape[1]),A_ub=np.vstack((C[upper_mask],-C[lower_mask])),
                             b_ub=np.r_[high[upper_mask],-low[lower_mask]],bounds=[(None,None)]*free.shape[1],method='highs')
            if not feasible.success: return None,reasons,reason
            nearest=minimize(lambda z:.5*z@z,feasible.x,jac=lambda z:z,
                             constraints=[LinearConstraint(C,low,high)],method='SLSQP',options={'ftol':1e-10,'maxiter':200})
            x+=free@(nearest.x if nearest.success else feasible.x)
            if np.any(G@x<lo-TOL) or np.any(G@x>hi+TOL): return None,reasons,reason
    return {pid:x[index[pid]:index[pid]+3] for pid in names},[],None


def _release(assembly,releases):
    doc=copy.deepcopy(assembly.doc)
    for release in releases:
        if not isinstance(release,dict) or release.get('action') not in ('loosen','detach'):
            raise DocumentError('Choose a socket to loosen or detach')
        joint=next((j for j in doc.get('joints',[]) if j['id']==release.get('joint')),None)
        if not joint or joint.get('type')!='socket': raise DocumentError('Only an editable socket connection can be released here')
        if release['action']=='loosen': joint['locked']=False
        else:
            doc['joints'].remove(joint)
            doc['drives']=[d for d in doc.get('drives',[]) if joint['id'] not in (d['driver'],d['follower'])]
            if 'animation' in doc: doc['animation']['tracks']=[t for t in doc['animation'].get('tracks',[]) if t['joint']!=joint['id']]
    return doc


def _rebase(before,after,doc,translations):
    changes={};old={j['id']:j for j in before.joints};resolved={j['id']:j for j in after.joints}
    for j in doc.get('joints',[]):
        if j['a']['part'] not in translations or joint_kind(j) not in ('prismatic','cylindrical'): continue
        pa0,pb0,_=before.joint_frames(old[j['id']]);pa,pb,axis=after.joint_frames(resolved[j['id']])
        slide=float(((pb-pa)-(pb0-pa0))@axis)
        if abs(slide)<TOL: continue
        changes[j['id']]=slide
        if 'slide_mm' in j.get('limits',{}): j['limits']['slide_mm']=[v-slide for v in j['limits']['slide_mm']]
        motor=j.get('motor',{})
        if motor.get('mode','position')=='position' and 'rotation_deg' not in motor:
            if 'target' in motor: motor['target']-=slide
            for key in motor.get('schedule',[]): key['target']-=slide
        if j['type']=='socket':
            fitting,tube=after.parts[j['a']['part']],after.parts[j['b']['part']]
            mouth,axis=fitting.frame(j['a'])
            if fitting.ports[j['a']['port']].get('through'):
                j['b']={'part':tube.id,'at_mm':float((mouth-tube.matrix[:3,3])@tube.matrix[:3,2]+tube.length/2)};j['insertion_mm']=0
            else: j['insertion_mm']=float((mouth-tube.frame(j['b'])[0])@axis)
    for track in doc.get('animation',{}).get('tracks',[]):
        if track['coordinate']=='slide_mm':
            for key in track['keyframes']: key['value']-=changes.get(track['joint'],0)
    for drive in doc.get('drives',[]):
        if drive['follower'] in changes: drive['offset_mm']=drive.get('offset_mm',0)-changes[drive['follower']]


def _check(before,after,relevant,collisions):
    from .validation import validate,CollisionWorld
    key=lambda i:(i['code'],tuple(sorted(i['parts'])),i.get('joint'))
    old_errors={key(i) for i in validate(before,collisions=False)['issues'] if i['severity']=='error'}
    for issue in validate(after,collisions=False)['issues']:
        if issue['severity']=='error' and key(issue) not in old_errors and relevant.intersection(issue['parts']):
            return {'reason':issue['message'],'conflicts':[issue['joint']] if issue.get('joint') else [],'parts':issue['parts']}
    if collisions:
        with CollisionWorld(after) as current,CollisionWorld(before) as previous:
            for left,right in itertools.combinations(after.parts,2):
                if not relevant.intersection((left,right)): continue
                if after.parts[left].kind==after.parts[right].kind=='human': continue
                if current.intersect(left,right,1) and not previous.intersect(left,right,1):
                    return {'reason':f'{left} would intersect {right}. Move the obstruction or change the requested length.',
                            'conflicts':[],'parts':[left,right]}
    return None


def _attempt(assembly,member,length,releases=(),collisions=True):
    doc=_release(assembly,releases)
    baseline=Assembly.from_doc(doc,assembly.base,assembly.library) if releases else assembly
    next(p for p in doc['parts'] if p['id']==member).setdefault('parameters',{})['length_mm']=length
    resized=Assembly.from_doc(doc,assembly.base,assembly.library)
    if abs(resized.parts[member].length-length)>TOL or (abs(baseline.parts[member].length-length)>TOL and resized.parts[member].shapes==baseline.parts[member].shapes):
        raise DocumentError('This part definition fixes its geometry length; make its geometry use $length_mm before resizing')
    moves,conflicts,reason=_translations(baseline,resized,member)
    if moves is None:
        return {'status':'blocked','reason':reason,'conflicts':conflicts,'parts':[member]}
    for spec in doc['parts']:
        shift=moves.get(spec['id'])
        if shift is not None and np.linalg.norm(shift)>TOL:
            matrix=resized.parts[spec['id']].matrix.copy();matrix[:3,3]+=shift;spec['pose']=pose_of(matrix)
    for instance in doc.get('objects',[]):
        ids=[pid for pid in moves if pid.startswith(instance['id']+'/') and pid not in {p['id'] for p in doc['parts']}]
        if ids and np.linalg.norm(moves[ids[0]])>TOL:
            pose=instance.setdefault('pose',{});pose['position_mm']=(np.array(pose.get('position_mm',[0,0,0]))+moves[ids[0]]).tolist()
    posed=Assembly.from_doc(doc,assembly.base,assembly.library)
    _rebase(baseline,posed,doc,moves)
    final=Assembly.from_doc(doc,assembly.base,assembly.library)
    moved=[pid for pid,delta in moves.items() if np.linalg.norm(delta)>TOL]
    problem=_check(baseline,final,{member,*moved},collisions)
    if problem: return {'status':'blocked',**problem}
    doc.pop('results',None);doc.pop('build_plan',None)
    return {'status':'resized','document':doc,'moved':moved,'member':member,'length_mm':length,'releases':list(releases)}


def resize_member(assembly,member,length_mm,releases=None,collisions=True,suggest=True):
    """Return a complete edit or a diagnostic; never modify the input assembly."""
    if member not in assembly.parts or assembly.parts[member].kind!='member': raise DocumentError('Select a pipe, dowel or extrusion')
    if not any(p['id']==member for p in assembly.doc['parts']): raise DocumentError('Expand the object before changing an internal member length')
    if isinstance(length_mm,bool) or not isinstance(length_mm,(int,float)) or not math.isfinite(length_mm) or length_mm<=0: raise DocumentError('Pipe length must be a finite positive number')
    if assembly.doc.get('state',{}).get('joints'): raise DocumentError('Capture the motion frame before resizing a member in this pose')
    connected,_=_component(assembly,member);releases=releases or []
    allowed={j['id'] for j in assembly.joints if j['a']['part'] in connected}
    if any(not isinstance(r,dict) or r.get('joint') not in allowed for r in releases): raise DocumentError('Release a connection in this member’s connected structure')
    result=_attempt(assembly,member,float(length_mm),releases,collisions)
    if result['status']=='resized': return result
    editable={j['id']:j for j in assembly.doc.get('joints',[])}
    conflicts=set(result.pop('conflicts',[]));blockers=[]
    for j in assembly.joints:
        if j['id'] in conflicts:
            blockers.append({'joint':j['id'],'parts':[j['a']['part'],j['b']['part']],
                             'connector':j['a']['part'],'port':j['a'].get('port'),
                             'locked':joint_kind(j)=='fixed','editable':j['id'] in editable})
    for label in conflicts:
        if label.startswith('anchor:'): blockers.append({'anchor':label[7:],'parts':[label[7:]]})
    candidates=[j for j in editable.values() if j.get('type')=='socket' and j['a']['part'] in connected]
    candidates.sort(key=lambda j:(member not in (j['a']['part'],j['b']['part']),j['id'] not in conflicts,j['id']))
    result.update(member=member,length_mm=float(length_mm),blockers=blockers,suggestions=[])
    if not suggest or releases: return result
    locked=[j for j in candidates if j.get('locked')][:10]
    choices=[[j] for j in locked]+list(itertools.combinations(locked[:6],2))
    for joints in choices:
        actions=[{'joint':j['id'],'action':'loosen'} for j in joints]
        trial=_attempt(assembly,member,float(length_mm),actions,False)
        if trial['status']=='resized' and (not collisions or _attempt(assembly,member,float(length_mm),actions,True)['status']=='resized'):
            result['suggestions'].append({'action':'loosen','joints':[j['id'] for j in joints],
                                          'releases':actions,'message':'Loosen these sockets, then resize while keeping every connection engaged.'})
            if len(result['suggestions'])>=2: break
    if not result['suggestions']:
        for j in candidates[:6]:
            actions=[{'joint':j['id'],'action':'detach'}]
            trial=_attempt(assembly,member,float(length_mm),actions,False)
            if trial['status']=='resized' and (not collisions or _attempt(assembly,member,float(length_mm),actions,True)['status']=='resized'):
                result['suggestions'].append({'action':'detach','joints':[j['id']],'releases':actions,
                    'message':'Loosening alone does not provide enough travel. Disconnect this socket to open the frame, then resize.'})
                if len(result['suggestions'])>=2: break
    return result
