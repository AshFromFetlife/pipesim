"""Close a socket by sliding connected rigid bodies along existing joints.

Bodies retain their rotations and cut lengths. A surrounding support body is a
reference, so closing an inner corner does not drag its outer frame around.
"""
import copy
import itertools
import numpy as np
from scipy.linalg import null_space
from scipy.optimize import LinearConstraint, linprog, minimize

from .document import DocumentError, joint_kind
from .math3d import pose_of


def sliding_context(assembly, first, second):
    """Find the mechanism and its editing reference without adding world anchors."""
    groups=assembly.editor_groups()
    group_of={pid:i for i,group in enumerate(groups) for pid in group}
    targets={group_of[first],group_of[second]}
    if len(targets)!=2: return None
    adjacency={i:set() for i in range(len(groups))}
    for joint in assembly.joints:
        a,b=(group_of[joint[end]['part']] for end in ('a','b'))
        if a!=b: adjacency[a].add(b);adjacency[b].add(a)
    all_groups=set();references=set()
    anchored={group_of[a['part']] for a in assembly.anchors}
    for start in sorted(targets):
        if start in all_groups: continue
        parents={start:None};queue=[start]
        for current in queue:
            for other in sorted(adjacency[current]):
                if other not in parents: parents[other]=current;queue.append(other)
        component=set(queue);all_groups|=component
        if component&anchored: references|=component&anchored;continue
        # With no world anchor, use a shared support between the connection
        # sides. For separate mechanisms use each side's largest support body.
        candidates=component-targets
        other=next(iter(targets-{start}))
        if other in component:
            path=set();current=parents[other]
            while current is not None: path.add(current);current=parents[current]
            candidates=path-targets
        if candidates: references.add(max(candidates,key=lambda gid:(len(groups[gid]),-gid)))
    sliders=[j for j in assembly.joints if joint_kind(j) in ('prismatic','cylindrical') and
             group_of[j['a']['part']]!=group_of[j['b']['part']] and group_of[j['a']['part']] in all_groups]
    if not sliders: return None
    return groups,group_of,sorted(all_groups-references)


def slide_connection(assembly, joint, placed, context):
    groups,group_of,selected=context
    index={gid:i*3 for i,gid in enumerate(selected)};size=3*len(selected)
    moving={pid for gid in selected for pid in groups[gid]}
    editable={p['id'] for p in assembly.doc['parts']}
    rows=[];targets=[];labels=[];bounds=[];lower=[];upper=[];bound_labels=[]
    def row(a,b,axis):
        result=np.zeros(size)
        for pid,sign in ((a,-1),(b,1)):
            if pid is not None and group_of[pid] in index:
                start=index[group_of[pid]];result[start:start+3]+=sign*axis
        return result
    def equal(a,b,axis,target,label):
        rows.append(row(a,b,axis));targets.append(target);labels.append(label)
    def bounded(a,b,axis,constant,lo,hi,label):
        bounds.append(row(a,b,axis));lower.append(lo-constant);upper.append(hi-constant);bound_labels.append(label)
    identity=np.eye(3)
    reference=np.zeros(size)
    for gid in selected:
        # A drag supplies a preferred translation, not permission to leave a
        # rail. Project that intent onto the allowed joint motion.
        shifts=[]
        for pid in groups[gid]:
            old,new=assembly.parts[pid].matrix,placed.parts[pid].matrix
            if not np.allclose(old[:3,:3],new[:3,:3],atol=1e-7,rtol=0):
                raise DocumentError('Sliding together keeps the current angles. Rotate the pipe or connector into alignment first')
            shifts.append(new[:3,3]-old[:3,3])
        reference[index[gid]:index[gid]+3]=np.mean(shifts,axis=0)
    for pid in assembly.parts:
        if pid not in moving and not np.allclose(assembly.parts[pid].matrix,placed.parts[pid].matrix,atol=1e-7,rtol=0):
            raise DocumentError('Slide the two connecting bodies without moving the surrounding structure')
    for anchor in assembly.anchors:
        for axis in identity: equal(None,anchor['part'],axis,0,'World anchor: '+anchor['part'])
    for pid in moving-editable:
        for axis in identity: equal(None,pid,axis,0,'Expand the reusable object containing '+pid)
    context=set();sliders=[]
    for existing in assembly.joints:
        a,b=existing['a']['part'],existing['b']['part']
        if not moving.intersection((a,b)): continue
        _,_,axis=assembly.joint_frames(existing);kind=joint_kind(existing)
        directions=identity-np.outer(axis,axis) if kind in ('prismatic','cylindrical') else identity
        for direction in directions: equal(a,b,direction,0,existing['id'])
        if kind in ('prismatic','cylindrical') and group_of[a]!=group_of[b]:
            sliders.append(existing);context.update({a,b}-moving)
            if 'slide_mm' in existing.get('limits',{}):
                lo,hi=existing['limits']['slide_mm'];bounded(a,b,axis,0,lo,hi,existing['id'])
        if existing['type']=='socket':
            fitting,tube=assembly.parts[a],assembly.parts[b];port=fitting.ports[existing['a']['port']]
            mouth,_=fitting.frame(existing['a'])
            if port.get('through'):
                along=tube.matrix[:3,2];station=(mouth-tube.matrix[:3,3])@along+tube.length/2
                half=port.get('engagement_mm',0)/2
                bounded(b,a,along,station,half,tube.length-half,existing['id'])
            else:
                end,_=tube.frame(existing['b']);depth=(mouth-end)@axis
                bounded(b,a,axis,depth,port.get('min_engagement_mm',0),port['engagement_mm'],existing['id'])
    if not sliders: raise DocumentError('Neither connecting body has an existing sliding joint')
    pa,pb,axis=assembly.joint_frames(joint);_,tube_axis=assembly.parts[joint['b']['part']].frame(joint['b'])
    port=assembly.parts[joint['a']['part']].ports[joint['a']['port']]
    alignment=abs(axis@tube_axis) if port.get('through') else axis@tube_axis
    if alignment<1-1e-7: raise DocumentError('The pipe and socket axes need rotation before they can slide together')
    for direction in identity:
        equal(joint['a']['part'],joint['b']['part'],direction,-direction@(pb-pa),'Requested socket alignment')
    A=np.array(rows);target=np.array(targets)
    unconstrained=reference+np.linalg.lstsq(A,target-A@reference,rcond=None)[0]
    residual=np.abs(A@unconstrained-target)
    if residual.max(initial=0)>1e-6:
        conflicts=list(dict.fromkeys(label for label,error in zip(labels,residual) if error>1e-6))
        raise DocumentError('The available slide axes cannot close this connection: '+', '.join(conflicts))
    def solve():
        x=unconstrained.copy()
        if not bounds: return x
        G=np.array(bounds);lo=np.array(lower);hi=np.array(upper);values=G@x
        failed=(values<lo-1e-6)|(values>hi+1e-6)
        if failed.any():
            message='Slide travel or socket engagement prevents alignment: '+', '.join(dict.fromkeys(label for label,bad in zip(bound_labels,failed) if bad))
            free=null_space(A)
            if not free.shape[1] or np.any(lo>hi): raise DocumentError(message)
            C=G@free;low=lo-values;high=hi-values
            finite_hi=np.isfinite(high);finite_lo=np.isfinite(low)
            feasible=linprog(np.zeros(free.shape[1]),A_ub=np.vstack((C[finite_hi],-C[finite_lo])),
                             b_ub=np.r_[high[finite_hi],-low[finite_lo]],bounds=[(None,None)]*free.shape[1],method='highs')
            if not feasible.success: raise DocumentError(message)
            nearest=minimize(lambda z:.5*z@z,feasible.x,jac=lambda z:z,method='SLSQP',
                             constraints=[LinearConstraint(C,low,high)],options={'ftol':1e-10,'maxiter':100})
            x+=free@(nearest.x if nearest.success else feasible.x)
            if np.any(G@x<lo-1e-6) or np.any(G@x>hi+1e-6): raise DocumentError(message)
        return x
    # Contact planes guide the remaining slide freedom away from an obstruction.
    # Every candidate is still checked with the actual compound collision model.
    from .validation import CollisionWorld
    reference_pairs={frozenset((j['a']['part'],j['b']['part'])) for j in assembly.joints}
    for iteration in range(12):
        x=solve();translations={pid:x[index[group_of[pid]]:index[group_of[pid]]+3] for pid in moving}
        fitted=copy.copy(assembly);fitted.parts={pid:copy.copy(part) for pid,part in assembly.parts.items()}
        for pid,delta in translations.items():
            fitted.parts[pid].matrix=assembly.parts[pid].matrix.copy();fitted.parts[pid].matrix[:3,3]+=delta
        collisions=[]
        with CollisionWorld(fitted) as world:
            for left,right in itertools.combinations(fitted.parts,2):
                if not moving.intersection((left,right)): continue
                relative=translations.get(left,np.zeros(3))-translations.get(right,np.zeros(3))
                if np.linalg.norm(relative)<1e-6: continue
                if fitted.parts[left].kind==fitted.parts[right].kind=='human' and frozenset((left,right)) in reference_pairs: continue
                contacts=[c for c in world.contacts(left,right) if c[8]*1000<-1]
                if not contacts: continue
                # The deepest contact for each pair supplies the local separation
                # direction; further contacts are checked on the next iteration.
                contact=min(contacts,key=lambda c:c[8]);direction=np.array(contact[7]);constraint=row(right,left,direction)
                label=f'{left} would intersect {right}'
                bounds.append(constraint);lower.append(float(constraint@x-contact[8]*1000+.25));upper.append(np.inf);bound_labels.append(label)
                collisions.append(label)
        if not collisions: break
    else: raise DocumentError('No clear placement found within the available sliding travel: '+', '.join(collisions))
    poses={}
    for pid,delta in translations.items():
        if np.linalg.norm(delta)<1e-7: continue
        matrix=assembly.parts[pid].matrix.copy();matrix[:3,3]+=delta;poses[pid]=pose_of(matrix)
    slides=[]
    for slider in sliders:
        a,b=slider['a']['part'],slider['b']['part'];_,_,axis=assembly.joint_frames(slider)
        travel=float((translations.get(b,np.zeros(3))-translations.get(a,np.zeros(3)))@axis)
        if abs(travel)>1e-6: slides.append({'joint':slider['id'],'slide_mm':travel})
    return poses,slides,sorted(context)
