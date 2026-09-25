"""Bounded reverse assembly search with swept insertion checks and support tests."""
from __future__ import annotations
import math
import numpy as np
import pybullet as pb
from .document import fingerprint, joint_kind
from .math3d import unit,pose_of
from .validation import CollisionWorld, support

def _directions(assembly,pid,remaining):
    axes=[]; restricted=[]
    for j in assembly.joints:
        a,b=j['a']['part'],j['b']['part']
        if pid not in (a,b) or not {a,b}<=remaining: continue
        pa,pb,axis=assembly.joint_frames(j)
        if j.get('type')=='socket':
            port=assembly.parts[a].ports[j['a']['port']]
            if port.get('assembly') in ('slide','glue') and j.get('assembly',port.get('assembly'))!='radial':
                # Either slide a fitting along a pipe, or withdraw a pipe along the socket.
                restricted.append(axis)
        axes.extend([axis,-axis])
    axes.extend([np.eye(3)[i]*s for i in range(3) for s in (-1,1)])
    unique=[]
    for axis in axes:
        axis=unit(axis)
        if restricted and any(abs(axis@unit(r))<.999 for r in restricted): continue
        if not any(np.dot(axis,a)>.999 for a in unique): unique.append(axis)
    return unique

def plan_build(assembly,search_limit=None):
    options=assembly.doc.get('build',{})
    limit=search_limit or options.get('search_limit',3000)
    remaining=frozenset(assembly.parts)
    visited=set(); explored=0; blocked={}; support_cache={}
    fixtures={f['part'] for f in options.get('fixtures',[])}
    allow_fixtures=options.get('allow_temporary_supports',False)
    require_stability=options.get('require_stability',True)
    if fixtures and not allow_fixtures:
        return {'status':'blocked','reason':'Temporary fixtures are declared but allow_temporary_supports is false','steps':[],'input_sha256':assembly.input_hash}

    def stable(subset):
        if not require_stability or not subset: return True
        if subset not in support_cache:
            evidence=support(assembly,subset)
            support_cache[subset]=evidence['stable'] or (allow_fixtures and all(g['stable'] or bool(set(g['parts'])&fixtures) for g in evidence['components']))
        return support_cache[subset]

    if not stable(remaining):
        return {'status':'blocked','reason':'Final structure has unsupported components under the gravity support test','steps':[],'input_sha256':assembly.input_hash}
    with CollisionWorld(assembly) as world:
        boxes=world.boxes
        span=max((float(np.linalg.norm(box[1]-box[0])) for box in boxes.values()),default=100)
        all_bounds=np.array(list(boxes.values())) if boxes else np.array([[[0,0,0],[1,1,1]]])
        escape=float(np.linalg.norm(all_bounds[:,1].max(axis=0)-all_bounds[:,0].min(axis=0)))+span+100
        # Separation is 1-Lipschitz under translation: advancing less than the
        # current clearance cannot cross an obstacle, regardless of its thickness.
        # A 0.5 mm internal threshold leaves margin inside the reported 1 mm
        # drawing/contact tolerance, including the 0.05 mm minimum advance.
        increment=100.

        def escape_path(pid,subset):
            for axis in _directions(assembly,pid,subset):
                hit=None
                swept=np.array([np.minimum(boxes[pid][0],boxes[pid][0]+axis*escape),np.maximum(boxes[pid][1],boxes[pid][1]+axis*escape)])
                candidates=[other for other in subset if other!=pid and np.all(swept[1]>=boxes[other][0]) and np.all(boxes[other][1]>=swept[0])]
                # Split fittings open around their mating pipe. Model the opened halves
                # by excluding that pipe only for the explicitly declared radial stage.
                for joint in assembly.joints:
                    if joint.get('type')=='socket' and joint['a']['part']==pid:
                        port=assembly.parts[pid].ports[joint['a']['port']]
                        _,_,socket_axis=assembly.joint_frames(joint)
                        if port.get('assembly')=='radial' and abs(axis@socket_axis)<.01:
                            candidates=[c for c in candidates if c!=joint['b']['part']]
                floor=assembly.doc.get('environment',{}).get('ground_z_mm',0)
                if assembly.doc.get('environment',{}).get('ground',True) and boxes[pid][0,2]+axis[2]*escape<floor-1: continue
                if not candidates: return axis,escape
                distance=0.
                while distance<escape:
                    delta=axis*distance; world.move(pid,delta)
                    clearance=increment
                    for other in candidates:
                        contacts=pb.getClosestPoints(world.world.part_map[pid][0],world.world.part_map[other][0],distance=increment/1000,physicsClientId=world.world.client)
                        separation=min((c[8]*1000 for c in contacts),default=increment)
                        clearance=min(clearance,separation)
                        if separation<-.5: hit=other; break
                    if hit: break
                    distance+=max(.05,min(increment,.5*(clearance+.5)))
                world.move(pid,[0,0,0])
                if hit is None: return axis,escape
                blocked.setdefault(pid,set()).add(hit)
            return None

        sequence=options.get('sequence')
        if sequence is not None:
            if len(sequence)!=len(remaining) or set(sequence)!=remaining:
                return {'status':'blocked','reason':'Explicit sequence must contain every part exactly once','steps':[],'input_sha256':assembly.input_hash}
            removed=[]; subset=remaining
            for pid in reversed(sequence):
                after=subset-{pid}
                if not stable(after):
                    return {'status':'blocked','reason':f'Explicit sequence is unstable before adding {pid}','steps':[],'input_sha256':assembly.input_hash}
                path=escape_path(pid,subset)
                if path is None:
                    return {'status':'blocked','reason':f'{pid} has no straight insertion route at its requested step','blocked_by':sorted(blocked.get(pid,[])),'steps':[],'input_sha256':assembly.input_hash}
                removed.append((pid,*path)); subset=after
        else:
            def search(subset):
                nonlocal explored
                if not subset: return []
                if subset in visited or explored>=limit: return None
                visited.add(subset); explored+=1
                # Start by removing leaves and upper parts; backtrack if they trap a fitting.
                degrees={p:sum(p in (j['a']['part'],j['b']['part']) and {j['a']['part'],j['b']['part']}<=subset for j in assembly.joints) for p in subset}
                for pid in sorted(subset,key=lambda p:(degrees[p],-assembly.parts[p].matrix[2,3],p)):
                    after=subset-{pid}
                    if not stable(after): continue
                    path=escape_path(pid,subset)
                    if path is None: continue
                    rest=search(after)
                    if rest is not None: return [(pid,*path),*rest]
                return None
            removed=search(remaining)
        if removed is None:
            return {'status':'indeterminate','reason':'Search budget exhausted' if explored>=limit else 'No stable straight-insertion sequence found; coordinated placement or declared fixtures may be needed','states_explored':explored,'blocked_by':{p:sorted(v) for p,v in blocked.items()},'steps':[],'input_sha256':assembly.input_hash}
    added=set(); steps=[]
    for number,(pid,axis,distance) in enumerate(reversed(removed),1):
        p=assembly.parts[pid]; connections=[]
        for j in assembly.joints:
            ends={j['a']['part'],j['b']['part']}
            if pid in ends and (ends-{pid})<=added: connections.append(j)
        action=f'Position {p.spec.get("label",p.definition.get("name",pid))} ({pid}).'
        if p.kind=='member': action=f'Insert {pid}, cut to {p.length:g} mm, along the shown direction.'
        elif any(port.get('through') and port.get('assembly')=='slide' for port in p.ports.values()): action=f'Slide {pid} onto the accessible tube end before fitting the next stop.'
        elif any(port.get('assembly')=='radial' for port in p.ports.values()): action=f'Open the split clamp {pid}, fit it around the tube, and close its halves.'
        fasten=[]
        for j in connections:
            if joint_kind(j)=='fixed':
                torque=j.get('torque_nm')
                fasten.append({'joint':j['id'],'action':'Secure connection'+(f' to the specified {torque:g} N·m' if torque is not None else '; use the torque specified for the purchased hardware')})
            else: fasten.append({'joint':j['id'],'action':'Retain the moving connection; leave the intended slide/rotation free'})
        added.add(pid)
        for f,j in zip(fasten,connections):
            if j.get('type')=='socket':
                if j.get('insertion_mm',0): f['action']+=f"; insert the tube {j['insertion_mm']:g} mm into this socket"
                elif 'at_mm' in j['b']: f['action']+=f"; align the socket centre {j['b']['at_mm']:g} mm from the tube start"
        steps.append({'number':number,'part':pid,'instruction':action,'pose':pose_of(p.matrix),'approach_direction':(-axis).tolist(),'start_offset_mm':(axis*distance).tolist(),'fasten':fasten,'installed_parts':sorted(added),'world_anchor':next((a for a in assembly.anchors if a['part']==pid),None),'fixture':next((f for f in options.get('fixtures',[]) if f['part']==pid),None)})
    return {'status':'buildable','input_sha256':assembly.input_hash,'states_explored':explored,'steps':steps,'certificate':{'method':'reverse search, individual straight translations, conservative clearance advancement and gravity support polygons','maximum_sweep_step_mm':increment,'collision_tolerance_mm':1.,'stability_checked':require_stability,'temporary_fixtures':options.get('fixtures',[])},'limitations':['Clearance is evaluated on the supplied collision geometry, including documented fitting and mesh approximations','No tool access, fastener thread motion, human handling or manufacturing tolerance stack-up','No claim of general assembly impossibility when the search is indeterminate']}
