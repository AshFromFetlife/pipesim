"""Batch manufacturing adjustments, with an immutable source and checked preview.

Socket-connected members act as editable edges during the fit. All socket
origins are solved together in a sparse linear system after aligning the axes.
No collision worlds or individual resize searches are created inside the fit.
"""
from __future__ import annotations

import copy
import json
import math
import threading
from collections import OrderedDict

import numpy as np
from scipy.optimize import least_squares, lsq_linear
from scipy.sparse import lil_matrix, csr_matrix, vstack
from scipy.sparse.linalg import LinearOperator
from scipy.spatial.transform import Rotation

from .document import Assembly, DocumentError, UnionFind, fingerprint
from .math3d import pose_of
from .connection_jobs import check_cancelled
from .exporting import bom
from .symmetry import canonical_catalog


def _number(value, name, low=0., high=1000000.):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low < value <= high:
        raise DocumentError(f'{name} must be a finite number above {low:g} and at most {high:g}')
    return float(value)


def _state_parts(assembly):
    coordinates=assembly.doc.get('state',{}).get('joints',{})
    if not coordinates:return set()
    graph=UnionFind(assembly.parts);held=set()
    for j in assembly.joints:
        graph.union(j['a']['part'],j['b']['part'])
        if j['id'] in coordinates:held.update((j['a']['part'],j['b']['part']))
    return {pid for group in graph.groups() if held.intersection(group) for pid in group}


def _editable(assembly, pid):
    part=assembly.parts[pid]
    if pid not in {p['id'] for p in assembly.doc.get('parts', [])} or pid in _state_parts(assembly): return False
    definition=assembly.library.parts.get(part.spec.get('catalog'), {})
    return (part.kind=='member' and '$length_mm' in str(definition.get('geometry', []))
            and 'geometry' not in part.spec.get('body', {})
            and all(j['type']=='socket' for j in assembly.joints if pid in (j['a']['part'],j['b']['part'])))


def analyse(assembly, tolerance_mm=1., precision_mm=None):
    tolerance=_number(tolerance_mm, 'Similar-length tolerance', -1e-12, 1000.)
    estimated=bool(assembly.doc.get('draft_subassemblies'))
    if estimated:
        from .drafting import runs, _layout
        doc=copy.deepcopy(assembly.doc)
        for run in runs(doc):
            layout=_layout(assembly,run)
            doc.setdefault('parts',[]).append({'id':run['id'],'catalog':run['catalog'],
                'parameters':{**run.get('parameters',{}),'length_mm':layout['length']},'pose':layout['pose']})
        doc.pop('draft_subassemblies',None)
        working=Assembly.from_doc(doc,assembly.base,assembly.library)
    else: working=assembly
    report=bom(working,precision_mm)
    buckets={}
    for part in working.parts.values():
        if part.kind!='member': continue
        catalog,definition=canonical_catalog(part,working)
        key=json.dumps([catalog or part.id,part.section,definition.get('material')],sort_keys=True)
        buckets.setdefault(key,[]).append(part)
    groups=[]
    for parts in buckets.values():
        cluster=[]
        for part in sorted(parts,key=lambda p:(p.length,p.id)):
            if cluster and part.length-cluster[0].length>tolerance+1e-9:
                groups.append(cluster);cluster=[]
            cluster.append(part)
        if cluster: groups.append(cluster)
    lengths=[]
    for index,parts in enumerate(groups):
        catalog,definition=canonical_catalog(parts[0],working)
        lengths.append({'id':str(index),'catalog':catalog or 'custom','name':definition.get('name',parts[0].id),
            'minimum_mm':min(p.length for p in parts),'maximum_mm':max(p.length for p in parts),
            'suggested_mm':float(np.median([p.length for p in parts])),
            'parts':[{'id':p.id,'length_mm':p.length,'editable':not estimated and _editable(working,p.id)} for p in parts]})
    direct={p['id'] for p in assembly.doc.get('parts',[])}
    anchored={a['part'] for a in assembly.anchors}
    anchored|=_state_parts(assembly)
    angles=[{'part':pid,'rotation_deg':pose_of(p.matrix)['rotation_deg'],
             'editable':not estimated and pid in direct and pid not in anchored}
            for pid,p in working.parts.items() if p.kind!='human']
    stations=[]
    for joint in assembly.joints:
        if joint['type']!='socket' or 'at_mm' not in joint['b']: continue
        connector=assembly.parts[joint['a']['part']]
        if not connector.ports[joint['a']['port']].get('through'): continue
        pipe=assembly.parts[joint['b']['part']]
        half=connector.ports[joint['a']['port']].get('engagement_mm',0)/2
        stations.append({'joint':joint['id'],'part':pipe.id,'connector':connector.id,
            'station_mm':joint['b']['at_mm'],'length_mm':pipe.length,'minimum_mm':half,'maximum_mm':pipe.length-half,
            'editable':not estimated and _editable(assembly,pipe.id)})
    return {'bom':report,'length_groups':lengths,'angles':angles,'stations':stations,
            'estimated':estimated,'source_hash':assembly.input_hash}


def _targets(assembly, requested):
    if not isinstance(requested,dict) or set(requested)-{'lengths','angles','stations'}:
        raise DocumentError('Choose pipe lengths, part angles or through-socket stations')
    lengths,angles,stations={},{},{}
    if any(not isinstance(requested.get(kind,{}),dict) for kind in ('lengths','angles','stations')):
        raise DocumentError('Adjustment lists must identify each part or socket')
    for pid,value in requested.get('lengths',{}).items():
        if pid not in assembly.parts or not _editable(assembly,pid):
            raise DocumentError(f'{pid}: this member retains its authored geometry')
        lengths[pid]=_number(value,'Pipe length')
    direct={p['id'] for p in assembly.doc.get('parts',[])}
    for pid,values in requested.get('angles',{}).items():
        if pid not in direct or not isinstance(values,list) or len(values)!=3:
            raise DocumentError('Part angles need three values for an editable part')
        angles[pid]=[_number(v,'Angle',-360001,360000) for v in values]
    joints={j['id']:j for j in assembly.joints}
    for jid,value in requested.get('stations',{}).items():
        j=joints.get(jid)
        if not j or j['type']!='socket' or 'at_mm' not in j['b'] or not _editable(assembly,j['b']['part']):
            raise DocumentError('Choose an existing through-socket station')
        stations[jid]=_number(value,'Distance down pipe',-1e-12)
    return {'lengths':lengths,'angles':angles,'stations':stations}


def _groups(assembly):
    graph=UnionFind(assembly.parts)
    for joint in assembly.joints:
        if joint['type']!='socket': graph.union(joint['a']['part'],joint['b']['part'])
    groups=graph.groups(); index={pid:i for i,g in enumerate(groups) for pid in g}
    direct={p['id'] for p in assembly.doc.get('parts',[])}
    fixed={index[a['part']] for a in assembly.anchors}
    fixed.update(index[pid] for pid in _state_parts(assembly))
    fixed.update(i for i,g in enumerate(groups) if not set(g)<=direct)
    return groups,index,fixed


def _rotations(assembly, targets, groups, index, fixed, cancelled):
    original={pid:p.matrix[:3,:3] for pid,p in assembly.parts.items()}
    if not targets: return original
    variables=[i for i in range(len(groups)) if i not in fixed]
    slot={gid:i for i,gid in enumerate(variables)}
    desired=np.zeros((len(variables),3))
    for gid in variables:
        rotations=[Rotation.from_euler('xyz',targets[pid],degrees=True).as_matrix()@original[pid].T
                   for pid in groups[gid] if pid in targets]
        if rotations: desired[slot[gid]]=Rotation.from_matrix(rotations).mean().as_rotvec()
    sockets=[]
    for j in assembly.joints:
        if j['type']!='socket':continue
        a,b=j['a']['part'],j['b']['part']
        ax=assembly.parts[a].local_frame(j['a'])[1];bx=assembly.parts[b].local_frame(j['b'])[1]
        sign=1 if (original[a]@ax)@(original[b]@bx)>=0 else -1
        sockets.append((a,b,ax,bx,sign))
    def matrices(q):
        delta=Rotation.from_rotvec(q.reshape(-1,3)).as_matrix() if len(q) else []
        return {pid:(delta[slot[index[pid]]]@matrix if index[pid] in slot else matrix) for pid,matrix in original.items()}
    def alignment(m):return np.concatenate([m[a]@ax-sign*m[b]@bx for a,b,ax,bx,sign in sockets]) if sockets else np.zeros(0)
    first=matrices(desired.ravel())
    if not len(variables) or np.max(np.abs(alignment(first)),initial=0)<1e-7:return first
    pattern=lil_matrix((3*len(sockets)+3*len(variables),3*len(variables)),dtype=int)
    for row,(a,b,*_) in enumerate(sockets):
        for pid in (a,b):
            if index[pid] in slot:pattern[3*row:3*row+3,3*slot[index[pid]]:3*slot[index[pid]]+3]=1
    for i in range(3*len(variables)):pattern[3*len(sockets)+i,i]=1
    def residual(q):
        check_cancelled(cancelled)
        return np.r_[alignment(matrices(q))*1000,(q-desired.ravel())*.01]
    result=least_squares(residual,desired.ravel(),jac_sparsity=pattern.tocsr(),max_nfev=60,
                         ftol=1e-9,xtol=1e-9,gtol=1e-9)
    return matrices(result.x)


def _fit(assembly, targets, cancelled=None):
    """Solve all pipe/socket origins once; rotations are independent of lengths."""
    check_cancelled(cancelled)
    targets=copy.deepcopy(targets)
    # Rounding at a pipe end must not abandon every unrelated edit in a batch.
    # Keep the requested station as intent and fit its nearest supported value;
    # the preview explicitly reports any target that was not reached.
    for j in assembly.joints:
        if j['type']!='socket':continue
        socket=assembly.parts[j['a']['part']].ports[j['a']['port']]
        if not socket.get('through'):continue
        pid=j['b']['part'];half=socket.get('engagement_mm',0)/2
        if targets['lengths'].get(pid,assembly.parts[pid].length)<2*half:
            targets['lengths'].pop(pid,None)
        length=targets['lengths'].get(pid,assembly.parts[pid].length)
        if j['id'] in targets['stations']:
            targets['stations'][j['id']]=float(np.clip(targets['stations'][j['id']],half,length-half))
    groups,index,fixed=_groups(assembly)
    rotations=_rotations(assembly,targets['angles'],groups,index,fixed,cancelled)
    origins={}
    for group in groups:
        first=assembly.parts[group[0]]
        delta=rotations[first.id]@first.matrix[:3,:3].T
        pivot=first.matrix[:3,3]
        for pid in group:origins[pid]=pivot+delta@(assembly.parts[pid].matrix[:3,3]-pivot)
    # Translation variables are measured in mm, depth/station variables in mm.
    lower=[];upper=[];nominal=[];weight=[];slots={}
    def variable(key,value,lo=-np.inf,hi=np.inf,regularization=1e-4):
        if hi-lo<1e-8:return None
        slots[key]=len(nominal);nominal.append(float(np.clip(value,lo,hi)))
        lower.append(lo);upper.append(hi);weight.append(regularization)
        return slots[key]
    for gid in range(len(groups)):
        if gid not in fixed:
            for axis in range(3):variable(('position',gid,axis),0.)
    authored=_state_parts(assembly)
    sockets=[j for j in assembly.joints if j['type']=='socket' and j['a']['part'] not in authored]
    for j in sockets:
        socket=assembly.parts[j['a']['part']].ports[j['a']['port']]
        if socket.get('through'):
            length=targets['lengths'].get(j['b']['part'],assembly.parts[j['b']['part']].length)
            half=socket.get('engagement_mm',0)/2
            if length<2*half: return None
            if j['id'] in targets['stations']:
                if not half<=targets['stations'][j['id']]<=length-half:return None
            else:variable(('station',j['id']),j['b']['at_mm'],half,length-half,.0003)
        else:
            variable(('depth',j['id']),j.get('insertion_mm',0),socket.get('min_engagement_mm',0),socket['engagement_mm'],.0005)
    matrix=lil_matrix((3*len(sockets),len(nominal)),dtype=float);rhs=np.zeros(3*len(sockets));fixed_stations={}
    for row,j in enumerate(sockets):
        check_cancelled(cancelled)
        a,b=j['a']['part'],j['b']['part']; socket=assembly.parts[a].ports[j['a']['port']]
        axis=rotations[a]@assembly.parts[a].local_frame(j['a'])[1]
        direction=rotations[b][:,2]; length=targets['lengths'].get(b,assembly.parts[b].length)
        mouth=rotations[a]@assembly.parts[a].local_frame(j['a'])[0]
        constant=origins[b]-origins[a]-mouth
        for dim in range(3):
            for pid,sign in ((b,1),(a,-1)):
                col=slots.get(('position',index[pid],dim))
                if col is not None:matrix[3*row+dim,col]+=sign
        if socket.get('through'):
            col=slots.get(('station',j['id']))
            half=socket.get('engagement_mm',0)/2
            station=targets['stations'].get(j['id'],float(np.clip(j['b']['at_mm'],half,length-half)))
            constant-=direction*length/2
            if col is None:constant+=direction*station;fixed_stations[j['id']]=station
            else:matrix[3*row:3*row+3,col]=direction.reshape(3,1)
        else:
            constant+=direction*((-.5 if j['b'].get('end')=='start' else .5)*length)
            col=slots.get(('depth',j['id']))
            if col is None:constant+=axis*j.get('insertion_mm',0)
            else:matrix[3*row:3*row+3,col]=axis.reshape(3,1)
        rhs[3*row:3*row+3]=-constant
    if nominal:
        regular=csr_matrix((weight,(range(len(nominal)),range(len(nominal)))),shape=(len(nominal),len(nominal)))
        design=vstack([matrix.tocsr(),regular]).tocsr()
        def matvec(x):check_cancelled(cancelled);return design@x
        def rmatvec(x):check_cancelled(cancelled);return design.T@x
        operator=LinearOperator(design.shape,matvec=matvec,rmatvec=rmatvec,dtype=float)
        result=lsq_linear(operator,np.r_[rhs,np.array(nominal)*weight],bounds=(lower,upper),
                          tol=1e-10,lsmr_tol=1e-12,max_iter=100,lsmr_maxiter=4000)
        values=result.x
    else:values=np.zeros(0)
    check_cancelled(cancelled)
    gaps=(matrix@values-rhs).reshape(-1,3)
    if any(np.linalg.norm(gap)>j.get('fit_tolerance_mm',1.1)+1e-7 for j,gap in zip(sockets,gaps)):return None
    doc=copy.deepcopy(assembly.doc)
    for spec in doc.get('parts',[]):
        pid=spec['id'];m=assembly.parts[pid].matrix.copy();m[:3,:3]=rotations[pid]
        m[:3,3]=origins[pid]+[values[slots[('position',index[pid],dim)]] if ('position',index[pid],dim) in slots else 0. for dim in range(3)]
        if not np.allclose(m,assembly.parts[pid].matrix,atol=1e-9,rtol=0):spec['pose']=pose_of(m)
        if pid in targets['lengths']:spec.setdefault('parameters',{})['length_mm']=targets['lengths'][pid]
    for j in doc.get('joints',[]):
        if ('depth',j['id']) in slots:j['insertion_mm']=float(values[slots[('depth',j['id'])]])
        if ('station',j['id']) in slots:j['b']['at_mm']=float(values[slots[('station',j['id'])]])
        if j['id'] in fixed_stations:j['b']['at_mm']=fixed_stations[j['id']]
        if j['id'] in targets['stations']:j['b']['at_mm']=targets['stations'][j['id']]
    doc.pop('results',None);doc.pop('build_plan',None)
    doc.get('metadata',{}).pop('draft_mirror_reopen',None)
    return doc


def _changes(before, after, targets):
    rows=[]
    joints={j['id']:j for j in after.joints}
    originals={j['id']:j for j in before.joints}
    for pid,value in targets['lengths'].items():
        rows.append({'kind':'length','id':pid,'before':before.parts[pid].length,'requested':value,'after':after.parts[pid].length,
                     'matched':abs(after.parts[pid].length-value)<1e-5})
    for jid,value in targets['stations'].items():
        rows.append({'kind':'station','id':jid,'before':originals[jid]['b']['at_mm'],'requested':value,'after':joints[jid]['b']['at_mm'],
                     'matched':abs(joints[jid]['b']['at_mm']-value)<1e-5})
    for pid,values in targets['angles'].items():
        error=np.linalg.norm(Rotation.from_matrix(Rotation.from_euler('xyz',values,degrees=True).as_matrix()@after.parts[pid].matrix[:3,:3].T).as_rotvec())
        rows.append({'kind':'angle','id':pid,'before':pose_of(before.parts[pid].matrix)['rotation_deg'],'requested':values,
                     'after':pose_of(after.parts[pid].matrix)['rotation_deg'],'matched':bool(error<1e-5)})
    return rows


def normalize(assembly, requested, cancelled=None):
    assembly.require_finished('normalising finished dimensions')
    targets=_targets(assembly,requested)
    if not any(targets.values()):
        return {'status':'unchanged','document':copy.deepcopy(assembly.doc),'source_hash':assembly.input_hash,
                'changes':[],'matched':0,'requested':0,'retained_targets':[],'message':'Choose adjustments to preview.'}
    unchanged=_changes(assembly,assembly,targets)
    if all(row['matched'] for row in unchanged):
        return {'status':'unchanged','document':copy.deepcopy(assembly.doc),'source_hash':assembly.input_hash,
                'changes':unchanged,'matched':len(unchanged),'requested':len(unchanged),'retained_targets':[],
                'message':'These targets already match the design.'}
    from .validation import validate
    # An independent final check decides whether the fitted candidate is usable.
    # If a combined request is incompatible, retain checked subsets, never call
    # a failed numerical search a proof that the author's topology is impossible.
    attempts=[targets]
    if targets['angles']:
        nearby={pid:values for pid,values in targets['angles'].items() if
            Rotation.from_matrix(Rotation.from_euler('xyz',values,degrees=True).as_matrix()@assembly.parts[pid].matrix[:3,:3].T).magnitude()<np.deg2rad(1)}
        if nearby and len(nearby)<len(targets['angles']):attempts.append({**targets,'angles':nearby})
    if targets['angles'] and (targets['lengths'] or targets['stations']):attempts.append({**targets,'angles':{}})
    if targets['stations'] and targets['lengths']:attempts.append({'lengths':targets['lengths'],'angles':{},'stations':{}})
    candidate=None
    for subset in attempts:
        check_cancelled(cancelled)
        try:
            document=_fit(assembly,subset,cancelled)
            if document is None:continue
            fitted=Assembly.from_doc(document,assembly.base,assembly.library)
            report=validate(fitted,cancelled=cancelled)
        except DocumentError:
            # An invalid candidate is a failed fit, not an invalid user design.
            continue
        if report['valid']:
            candidate=fitted;break
    if candidate is None:candidate=assembly
    changes=_changes(assembly,candidate,targets)
    moved=any(abs(row['before']-row['after'])>1e-6 for row in changes if row['kind']!='angle')
    moved|=any(Rotation.from_matrix(candidate.parts[pid].matrix[:3,:3]@assembly.parts[pid].matrix[:3,:3].T).magnitude()>1e-6
               for pid in targets['angles'])
    if not moved:
        candidate=assembly;changes=unchanged
    changed=fingerprint(candidate.doc)!=fingerprint(assembly.doc)
    return {'status':'preview' if changed else 'unchanged','document':copy.deepcopy(candidate.doc),
            'source_hash':assembly.input_hash,'changes':changes,
            'matched':sum(row['matched'] for row in changes),'requested':len(changes),
            'message':('Checked batch preview. Apply to keep these adjustments.' if changed else
                'No checked change from this batch yet. The original design is retained; try fewer targets or a smaller rounding increment.'),
            'retained_targets':[row['id'] for row in changes if not row['matched']]}


class BomPreviews:
    """Keep checked previews server-side; applying one must match its source."""
    def __init__(self):self.lock=threading.Lock();self.results=OrderedDict()
    def remember(self,key,result):
        with self.lock:
            self.results[key]=copy.deepcopy(result)
            while len(self.results)>8:self.results.popitem(last=False)
    def apply(self,assembly,key):
        with self.lock:result=copy.deepcopy(self.results.get(key))
        if result is None:
            return {'status':'stale','message':'This preview has expired. Preview the batch again before applying it.'}
        if result['source_hash']!=assembly.input_hash:
            return {'status':'stale','message':'The design changed since this preview. Preview the batch again before applying it.'}
        return {**result,'status':'applied' if result['status']=='preview' else 'unchanged'}
