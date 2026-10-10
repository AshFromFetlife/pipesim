"""End-handle length edits for draft runs, cut members, and flexible lines.

The chosen end is only an input to the edit. Geometry is solved in world space
relative to the opposite end, so reversing a member does not change the tool.
"""
import copy
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .document import Assembly, DocumentError
from .drafting import _layout as draft_layout, preview as draft_preview, runs
from .math3d import align_axis, pose_of
from .resizing import _check, _component, resize_member


def _frame(assembly, identifier):
    run=next((r for r in runs(assembly.doc) if r['id']==identifier), None)
    if run is not None:
        layout=draft_layout(assembly,run)
        return layout['start'],layout['end'],run
    part=assembly.parts.get(identifier)
    if part is None or part.kind!='member': raise DocumentError('Select a pipe or flexible line')
    axis=part.matrix[:3,2];center=part.matrix[:3,3]
    return center-axis*part.length/2,center+axis*part.length/2,None


def _graph(assembly, identifier, draft):
    if not draft: return _component(assembly,identifier)[0]
    connected={identifier}
    edges=[{r['id'],a['connector']} for r in runs(assembly.doc) for a in r.get('attachments',[])]
    edges += [{j['a']['part'],j['b']['part']} for j in assembly.joints if j.get('locked') or j['type']=='fixed']
    while True:
        previous=len(connected)
        for edge in edges:
            if edge&connected: connected|=edge
        if len(connected)==previous:return connected


def _end_followers(assembly, identifier, side, draft):
    """Bodies carried by one cut end, with the edited pipe removed as a bridge.

    A through support joins the same pipe to a separate branch of the graph.
    Its world anchor does not require the end fitting to stay at that station.
    """
    if draft:
        connected={a['connector'] for a in draft.get('attachments',[]) if a.get('end')==side}
    else:
        connected={joint[other]['part'] for joint in assembly.joints
                   for end,other in (('a','b'),('b','a'))
                   if joint[end]['part']==identifier and joint[end].get('end')==side}
    edges=[{r['id'],a['connector']} for r in runs(assembly.doc) for a in r.get('attachments',[])]
    edges += [{j['a']['part'],j['b']['part']} for j in assembly.joints]
    edges=[edge for edge in edges if identifier not in edge]
    while True:
        previous=len(connected)
        for edge in edges:
            if edge&connected:connected|=edge
        if len(connected)==previous:return connected


def _valid_against(before,after,members,*,ignore_draft_runs=()):
    if before.doc.get('draft_subassemblies') or after.doc.get('draft_subassemblies'):
        # Exact validation intentionally refuses draft runs: their cut lengths
        # have not been finalized. A length edit to an exact member or flexible
        # line must still work in that scene, so validate the exact bodies and
        # then check draft geometry with its own preview constraints.
        def exact(assembly):
            doc=copy.deepcopy(assembly.doc)
            doc.pop('draft_subassemblies',None)
            return Assembly.from_doc(doc,assembly.base,assembly.library)
        checked_before,checked_after=exact(before),exact(after)
    else:
        checked_before,checked_after=before,after
    # A draft edit can move exact fittings outside ``members`` when it captures
    # a socket. Check every exact part for newly broken joints; the selected
    # draft run alone is allowed to remain orange during an edit.
    relevant=set(checked_after.parts) if ignore_draft_runs else set(members)
    problem=_check(checked_before,checked_after,relevant,False)
    if problem: raise DocumentError(problem['reason'])
    if before.doc.get('draft_subassemblies') or after.doc.get('draft_subassemblies'):
        def key(conflict):
            return (conflict.get('run'),conflict.get('code'),conflict.get('connector'),
                    conflict.get('port'),conflict.get('plane'))
        previous={key(conflict):conflict for item in draft_preview(before)
                  for conflict in item['conflicts']}
        for item in draft_preview(after):
            if item['id'] in ignore_draft_runs:continue
            for conflict in item['conflicts']:
                old=previous.get(key(conflict))
                if old is None or any(float(conflict.get(field,0))>float(old.get(field,0))+1e-6
                                      for field in ('residual_mm','residual_deg')):
                    raise DocumentError(conflict['message'])


def _axial_extent(part,direction):
    """Conservative occupied half-width along a pipe, from the connector solids."""
    low,high=math.inf,-math.inf
    for shape in part.shapes:
        center=part.matrix[:3,:3]@np.array(shape.get('position_mm',[0,0,0]),float)+part.matrix[:3,3]
        shape_rotation=Rotation.from_euler('xyz',shape.get('rotation_deg',[0,0,0]),degrees=True).as_matrix()
        orientation=part.matrix[:3,:3]@shape_rotation
        kind=shape.get('type')
        if kind=='sphere':radius=float(shape.get('radius_mm',shape.get('diameter_mm',0)/2))
        elif kind in ('tube','cylinder','capsule'):
            axis=np.array(shape.get('axis',[0,0,1]),float);axis/=max(np.linalg.norm(axis),1e-9)
            axis=orientation@axis;parallel=abs(float(axis@direction))
            radius=float(shape.get('radius_mm',shape.get('diameter_mm',0)/2))
            radius=parallel*float(shape.get('length_mm',0))/2+(
                radius if kind=='capsule' else math.sqrt(max(0,1-parallel**2))*radius)
        elif kind in ('box','extrusion'):
            dimensions=np.array(shape.get('size_mm',[0,0,0]),float)
            radius=float(np.abs(orientation.T@direction)@dimensions/2)
        else:continue
        station=float(center@direction);low=min(low,station-radius);high=max(high,station+radius)
    origin=float(part.matrix[:3,3]@direction)
    return max(origin-low,high-origin) if math.isfinite(low) else 0.0


def _spacing_length(assembly,identifier,length,side):
    """Stop through fittings just before their bodies would overlap."""
    first,last,draft=_frame(assembly,identifier)
    old_length=float(np.linalg.norm(last-first))
    fixed=last if side=='start' else first
    outward=(first-last)/old_length if side=='start' else (last-first)/old_length
    attached=([{'connector':a['connector'],'port':a['port']} for a in draft.get('attachments',[])] if draft else
              [{'connector':j['a']['part'],'port':j['a']['port']} for j in assembly.joints
               if j['type']=='socket' and j['b']['part']==identifier])
    fittings={a['connector'] for a in attached}
    positions=[]
    for pid in fittings:
        part=assembly.parts.get(pid)
        if part is None:continue
        station=float((part.matrix[:3,3]-fixed)@outward)
        if 0<=station<=old_length:
            positions.append((station,_axial_extent(part,outward)))
    positions.sort()
    minimum=0.0
    # Proportional shrinking also draws every through station toward the fixed
    # end. A body-spacing check alone can leave a socket too close to either
    # cut face even while the fitting bodies remain separated.
    for attachment in attached:
        part=assembly.parts.get(attachment['connector'])
        if part is None:continue
        socket=part.ports.get(attachment['port'],{})
        if not socket.get('through'):continue
        mouth,_=part.frame({'port':attachment['port']})
        station=float((mouth-fixed)@outward)
        if not 0<station<old_length:continue
        half=float(socket.get('engagement_mm',0))/2
        if half>0:
            minimum=max(minimum,old_length*(half+1e-6)/min(station,old_length-station))
    for (a,ra),(b,rb) in zip(positions,positions[1:]):
        gap=b-a;required=ra+rb+2
        if gap>1e-6 and gap>=required-1e-6:
            minimum=max(minimum,old_length*required/gap)
    return min(old_length,max(length,minimum)) if length<old_length else length


def _stationary_shrink_length(assembly,identifier,length,side):
    """Keep already seated through fittings engaged while a free cut end moves."""
    first,last,run=_frame(assembly,identifier)
    old_length=float(np.linalg.norm(last-first))
    direction=(last-first)/old_length
    centered=bool(run) and any(
        plane.get('run_modes',{}).get(identifier)=='centered'
        for group in assembly.doc.get('draft_subassemblies',[])
        if any(item['id']==identifier for item in group['runs'])
        for plane in group.get('mirrors',[]))
    fixed=last if side=='start' else first
    outward=-direction if side=='start' else direction
    middle=(first+last)/2
    minimum=1e-6
    for attachment in run.get('attachments',[]) if run else []:
        fitting=assembly.parts[attachment['connector']]
        socket=fitting.ports[attachment['port']]
        if not socket.get('through'):continue
        mouth,_=fitting.frame({'port':attachment['port']})
        station=float((mouth-first)@direction)
        half=float(socket.get('engagement_mm',0))/2
        # An already short fitting stays editable in draft and is shown orange.
        # Otherwise leave enough material at both sides of its existing bore.
        if station<half-1e-3 or station>old_length-half+1e-3:continue
        required=(2*(abs(float((mouth-middle)@direction))+half+.1) if centered else
                  float((mouth-fixed)@outward)+half+.1)
        minimum=max(minimum,required)
    return min(old_length,max(length,minimum))


def _automatic_through(doc,assembly,identifier,old_start,old_end,new_start,new_end,side,capture_mm,capture_deg,locked):
    """Capture compatible sockets reached by an extending free pipe end."""
    direction=(new_end-new_start)/np.linalg.norm(new_end-new_start)
    length=np.linalg.norm(new_end-new_start);old_length=np.linalg.norm(old_end-old_start)
    if length<=old_length+1e-6:return
    old_tip=old_start if side=='start' else old_end
    new_tip=new_start if side=='start' else new_end
    outward=-direction if side=='start' else direction
    draft=next((r for r in runs(doc) if r['id']==identifier),None)
    member=assembly.parts.get(identifier)
    definition=assembly.library.parts[draft['catalog']] if draft else None
    section=(member.section if member else definition.get('section',{}))
    diameter=float(section.get('diameter_mm',0))
    profile='round' if section.get('type') in ('tube','round','circle') else section.get('profile',section.get('type'))
    used={a['connector'] for a in draft.get('attachments',[])} if draft else {
        j['a']['part'] for j in doc.get('joints',[]) if j.get('type')=='socket' and j['b']['part']==identifier}
    for fitting in assembly.parts.values():
        if fitting.id==identifier or fitting.id in used or fitting.kind=='member':continue
        spec=next((p for p in doc['parts'] if p['id']==fitting.id),None)
        if not spec or any(a['part']==fitting.id for a in doc.get('anchors',[])):continue
        attached=any(fitting.id in (j['a']['part'],j['b']['part']) for j in assembly.joints) or any(
            a['connector']==fitting.id for r in runs(doc) for a in r.get('attachments',[]))
        for port_name,socket in fitting.ports.items():
            if socket.get('type')!='socket':continue
            if any(j['type']=='socket' and j['a']['part']==fitting.id and j['a'].get('port')==port_name
                   for j in assembly.joints) or any(a['connector']==fitting.id and a['port']==port_name
                   for r in runs(doc) for a in r.get('attachments',[])):continue
            if socket.get('profile','round')!=profile or abs(socket.get('diameter_mm',0)-diameter)>.6:continue
            mouth,axis=fitting.frame({'port':port_name})
            station=float((mouth-new_start)@direction)
            old_distance=float((mouth-old_tip)@outward)
            new_distance=float((mouth-new_tip)@outward)
            if old_distance < -capture_mm or new_distance > capture_mm:continue
            target=new_start+station*direction
            through=bool(socket.get('through'))
            aligned=(direction if axis@direction>=0 else -direction) if through else (
                direction if side=='start' else -direction)
            radial=(float(np.linalg.norm(mouth-target)) if through else
                    float(np.linalg.norm((new_tip-mouth)-axis*((new_tip-mouth)@axis))))
            angular=float(axis@aligned)
            if radial>capture_mm or angular<math.cos(math.radians(capture_deg)):continue
            if attached and not draft and (radial>1e-3 or angular<1-1e-6):continue
            depth=None
            if through:
                # Draft connections may be claimed before the tip spans the
                # entire bore; preview shows the short engagement in orange.
                if not -capture_mm<=station<=length+capture_mm:continue
            else:
                raw_depth=float((mouth-new_tip)@axis)
                minimum=float(socket.get('min_engagement_mm',0))
                maximum=float(socket.get('engagement_mm',0))
                if raw_depth < -capture_mm or raw_depth > maximum+capture_mm:continue
                depth=float(np.clip(raw_depth,minimum,maximum))
            if draft:
                attachment={'connector':fitting.id,'port':port_name}
                if not through:attachment.update(end=side,insertion_mm=depth)
                from .drafting import _check_attachment
                try:_check_attachment(assembly,draft,attachment,runs(doc))
                except DocumentError:continue
            if not attached:
                rotation=align_axis(axis,aligned)
                matrix=fitting.matrix.copy();matrix[:3,:3]=rotation@matrix[:3,:3]
                mouth_target=target if through else new_tip+aligned*depth
                matrix[:3,3]=mouth_target-matrix[:3,:3]@np.array(socket.get('position_mm',[0,0,0]))
                spec['pose']=pose_of(matrix)
            if draft:
                draft.setdefault('attachments',[]).append(attachment)
            else:
                joint_id=f'{fitting.id}-{port_name}-{identifier}'
                if any(j['id']==joint_id for j in doc.get('joints',[])):joint_id+='-resize'
                doc.setdefault('joints',[]).append({'id':joint_id,'type':'socket',
                    'a':{'part':fitting.id,'port':port_name},
                    'b':{'part':identifier,**({'at_mm':station} if through else {'end':side})},
                    'insertion_mm':0 if through else depth,'locked':locked,'fit_tolerance_mm':2})
            used.add(fitting.id)
            break


def _stretch(assembly,identifier,length,side,behavior,capture_mm,capture_deg,locked,auto_connect,*,
             propagate=True,follow_endpoint=False):
    first,last,draft=_frame(assembly,identifier)
    old_length=float(np.linalg.norm(last-first))
    if length<=0 or length>1e6:raise DocumentError('Choose a positive length no greater than 1,000,000 mm')
    fixed=last if side=='start' else first
    outward=(first-last)/old_length if side=='start' else (last-first)/old_length
    new_moving=fixed+outward*length
    new_first,new_last=(new_moving,fixed) if side=='start' else (fixed,new_moving)
    centered=bool(draft) and any(plane.get('run_modes',{}).get(identifier)=='centered'
        for group in assembly.doc.get('draft_subassemblies',[]) if any(r['id']==identifier for r in group['runs'])
        for plane in group.get('mirrors',[]))
    direction=(last-first)/old_length
    middle=(first+last)/2
    if centered:
        new_first=middle-direction*length/2
        new_last=middle+direction*length/2
    scale=length/old_length;delta=length-old_length
    members=_graph(assembly,identifier,bool(draft)) if behavior=='follow' and propagate else {identifier}
    if follow_endpoint:
        members={identifier}|_end_followers(assembly,identifier,side,draft)
    doc=copy.deepcopy(assembly.doc)
    active_runs={r['id']:r for r in runs(doc)}
    def fraction(point):return float((point-(first if centered else fixed))@(direction if centered else outward)/old_length)
    def transformed(point):
        if follow_endpoint:return point+outward*delta
        if centered:return point+direction*float((point-middle)@direction)*(scale-1)
        return point+outward*delta*np.clip(fraction(point),0,1)
    if behavior=='detach':
        if draft:
            edit=active_runs[identifier]
            kept=[]
            for attachment in edit.get('attachments',[]):
                fitting=assembly.parts[attachment['connector']]
                mouth,_=fitting.frame({'port':attachment['port']})
                if attachment.get('end')==side:
                    if length<old_length or not fitting.ports[attachment['port']].get('through'):continue
                    attachment.pop('end',None);attachment.pop('insertion_mm',None)
                if 'end' not in attachment and not 0<=float((mouth-new_first)@((new_last-new_first)/length))<=length:continue
                kept.append(attachment)
            edit['attachments']=kept
        else:
            keep=[]
            for joint in doc.get('joints',[]):
                endpoint=joint['b'] if joint['b']['part']==identifier else joint['a']
                if joint.get('type')=='socket' and endpoint['part']==identifier:
                    fitting=assembly.parts[joint['a']['part']]
                    if endpoint.get('end')==side:
                        if length<old_length or not fitting.ports[joint['a']['port']].get('through'):continue
                        endpoint.pop('end',None);endpoint['at_mm']=0
                    mouth,_=fitting.frame(joint['a'])
                    axis=(new_last-new_first)/length
                    station=float((mouth-new_first)@axis)
                    half=fitting.ports[joint['a']['port']].get('engagement_mm',0)/2
                    if 'at_mm' in endpoint and not half<=station<=length-half:continue
                keep.append(joint)
            removed={j['id'] for j in doc.get('joints',[])}-{j['id'] for j in keep}
            doc['joints']=keep
            doc['drives']=[d for d in doc.get('drives',[]) if d['driver'] not in removed and d['follower'] not in removed]
            if 'animation' in doc:doc['animation']['tracks']=[t for t in doc['animation'].get('tracks',[]) if t['joint'] not in removed]
    for run in runs(doc):
        if run['id'] not in members:continue
        if run['id']==identifier:
            run['start_mm']=new_first.tolist();run['end_mm']=new_last.tolist()
        else:
            run['start_mm']=transformed(np.array(run['start_mm'])).tolist()
            run['end_mm']=transformed(np.array(run['end_mm'])).tolist()
        if run.get('locked_length_mm') is not None:
            run['locked_length_mm']=float(np.linalg.norm(np.array(run['end_mm'])-run['start_mm']))
    resized={}
    for spec in doc['parts']:
        pid=spec['id']
        if pid not in members:continue
        part=assembly.parts[pid]
        before=part.matrix[:3,3]
        after=transformed(before) if behavior=='follow' or pid==identifier else before
        if pid==identifier and not draft:after=(new_first+new_last)/2
        if any(a['part']==pid for a in assembly.anchors) and np.linalg.norm(after-before)>1e-5:
            raise DocumentError(f'{pid} is anchored and cannot follow this endpoint')
        matrix=part.matrix.copy();matrix[:3,3]=after
        spec['pose']=pose_of(matrix)
        if part.kind=='member' and (pid==identifier or behavior=='follow' and not follow_endpoint and
                abs(part.matrix[:3,2]@outward)>.999 and
                0-1e-5<=fraction(before-part.matrix[:3,2]*part.length/2)<=1+1e-5 and
                0-1e-5<=fraction(before+part.matrix[:3,2]*part.length/2)<=1+1e-5):
            new_length=length if pid==identifier else part.length*scale
            if '$length_mm' not in str(assembly.library.parts.get(spec.get('catalog'),spec.get('body',{})).get('geometry',[])):
                raise DocumentError(f'{pid} has fixed geometry and cannot follow the length change')
            spec.setdefault('parameters',{})['length_mm']=new_length
            resized[pid]=new_length
    if draft is None:
        candidate=Assembly.from_doc(doc,assembly.base,assembly.library)
        for joint in doc.get('joints',[]):
            for endpoint in (joint['a'],joint['b']):
                pid=endpoint['part']
                if pid not in resized or 'at_mm' not in endpoint:continue
                if joint.get('type')=='socket' and joint['a']['part'] in candidate.parts:
                    mouth,_=candidate.parts[joint['a']['part']].frame(joint['a'])
                    member=candidate.parts[pid]
                    endpoint['at_mm']=float((mouth-member.matrix[:3,3])@member.matrix[:3,2]+member.length/2)
                else:endpoint['at_mm']=float((endpoint['at_mm']-assembly.parts[pid].length/2)*resized[pid]/assembly.parts[pid].length+resized[pid]/2)
    candidate=Assembly.from_doc(doc,assembly.base,assembly.library,validate_mirror_geometry=False)
    if auto_connect:
        _automatic_through(doc,candidate,identifier,first,last,new_first,new_last,side,capture_mm,capture_deg,locked)
    after=Assembly.from_doc(doc,assembly.base,assembly.library,validate_mirror_geometry=False)
    _valid_against(assembly,after,members,ignore_draft_runs={identifier} if draft else ())
    actual_first,actual_last,_=_frame(after,identifier)
    if (np.linalg.norm(actual_first-new_first)>1e-3 or
            np.linalg.norm(actual_last-new_last)>1e-3):
        raise DocumentError('The connected structure cannot keep the pipe handle at the requested position')
    doc.pop('results',None);doc.pop('build_plan',None)
    return {'status':'resized','document':doc,'member':identifier,'length_mm':length,
            'moved':sorted(members-{identifier}),'endpoint':side}


def resize_drag(assembly,identifier,length_mm,side,behavior='follow',capture_mm=40,capture_deg=15,locked=True,auto_connect=True):
    if side not in ('start','end') or behavior not in ('follow','detach'):
        raise DocumentError('Choose an end handle and connector behavior')
    if not isinstance(length_mm,(int,float)) or isinstance(length_mm,bool) or not math.isfinite(length_mm):
        raise DocumentError('Choose a finite length')
    if length_mm<=0 or length_mm>1e6:
        raise DocumentError('Choose a positive length no greater than 1,000,000 mm')
    if not 0<=capture_mm<=100 or not 0<=capture_deg<=45:
        raise DocumentError('Choose a valid connection capture window')
    if any(o['id']==identifier and o['template']=='chain' for o in assembly.doc.get('objects',[])):
        from .grouping import update_object_parameters
        instance=next(o for o in assembly.doc['objects'] if o['id']==identifier)
        from .chain import summary
        info=summary(instance,assembly.library)
        if length_mm>info['pitch_mm']*1000:
            raise DocumentError('A flexible line supports at most 1,000 segments')
        fixed_id=info['end_part'] if side=='start' else info['start_part']
        fixed_port=info['end_port'] if side=='start' else info['start_port']
        old_point,_=assembly.parts[fixed_id].frame({'port':fixed_port})
        doc=update_object_parameters(assembly,identifier,{**instance['parameters'],'length_mm':float(length_mm)})
        after=Assembly.from_doc(doc,assembly.base,assembly.library)
        new_info=summary(next(o for o in doc['objects'] if o['id']==identifier),assembly.library)
        new_id=new_info['end_part'] if side=='start' else new_info['start_part']
        new_port=new_info['end_port'] if side=='start' else new_info['start_port']
        new_point,_=after.parts[new_id].frame({'port':new_port})
        delta=old_point-new_point
        if np.linalg.norm(delta)>1e-6:
            for anchor in after.anchors:
                if anchor['part'].startswith(identifier+'/'):
                    raise DocumentError(f'{anchor["part"]} is anchored and cannot follow this endpoint')
            from .grouping import move_object
            obj=next(o for o in doc['objects'] if o['id']==identifier)
            pose=copy.deepcopy(obj.get('pose',{}))
            pose['position_mm']=(np.array(pose.get('position_mm',[0,0,0]))+delta).tolist()
            doc=move_object(after,identifier,pose)['document']
            after=Assembly.from_doc(doc,assembly.base,assembly.library)
        for anchor in assembly.anchors:
            pid=anchor['part']
            if not pid.startswith(identifier+'/'):continue
            if pid not in after.parts or not np.allclose(assembly.parts[pid].matrix,after.parts[pid].matrix,atol=1e-4):
                raise DocumentError(f'{pid} is anchored and cannot follow this endpoint')
        _valid_against(assembly,after,{p for p in after.parts if p.startswith(identifier+'/')})
        return {'status':'resized','document':doc,'member':identifier,'length_mm':length_mm,'moved':[],'endpoint':side}
    first,last,draft=_frame(assembly,identifier)
    if draft is None and not any(part['id']==identifier for part in assembly.doc['parts']):
        raise DocumentError('Expand the object before changing an internal member length')
    requested_length=float(length_mm)
    if behavior=='follow' and any(anchor['part'] in _graph(assembly,identifier,bool(draft))
                                  for anchor in assembly.anchors):
        # An anchored through branch stays at its world station while the cut
        # end and its own attached branch move. Do not scale the whole graph or
        # silently release a support merely because the selected end is occupied.
        try:
            return _stretch(assembly,identifier,requested_length,side,behavior,
                            capture_mm,capture_deg,locked,auto_connect,
                            follow_endpoint=True)
        except DocumentError:
            # A loop or another joint may require the existing constrained solve.
            pass
    if behavior=='follow':length_mm=_spacing_length(assembly,identifier,length_mm,side)
    # A through socket can slide on its pipe. Extending a free end leaves every
    # existing fitting at its station and changes only the selected pipe. An
    # axial deformation of the whole attachment graph would move unrelated
    # pipes and could violate crossed sockets or mirror constraints.
    old_length=float(np.linalg.norm(last-first))
    occupied_end=(any(a.get('end')==side for a in draft.get('attachments',[])) if draft else
                  any(endpoint['part']==identifier and endpoint.get('end')==side
                      for joint in assembly.joints for endpoint in (joint['a'],joint['b'])))
    stationary_extension=behavior=='follow' and length_mm>old_length+1e-6 and not occupied_end
    if stationary_extension:
        try:
            return _stretch(assembly,identifier,float(length_mm),side,behavior,
                            capture_mm,capture_deg,locked,auto_connect,propagate=False)
        except DocumentError:
            # A different joint may depend on the member's pose. The regular
            # follow solve below can still find a valid constrained edit.
            pass
    if draft is not None and behavior=='follow' and requested_length<old_length-1e-6 and not occupied_end:
        local_length=_stationary_shrink_length(assembly,identifier,requested_length,side)
        connected=_graph(assembly,identifier,True)
        other_members=(any(run['id']!=identifier and run['id'] in connected
                           for run in runs(assembly.doc)) or
                       any(pid!=identifier and pid in connected and part.kind=='member'
                           for pid,part in assembly.parts.items()))
        # Prefer the requested cut over a graph deformation that has to stop
        # early to keep every fitting in its old proportional position. In a
        # connected frame, proportional deformation can also silently shorten
        # other pipes and move crossbars while passing every geometry check.
        # A free cut must preserve those other authored spans when possible.
        if other_members or local_length<length_mm-1e-6:
            try:
                return _stretch(assembly,identifier,local_length,side,behavior,
                                capture_mm,capture_deg,locked,auto_connect,
                                propagate=False)
            except DocumentError:
                pass
        try:
            return _stretch(assembly,identifier,float(length_mm),side,behavior,
                            capture_mm,capture_deg,locked,auto_connect)
        except DocumentError:
            return _stretch(assembly,identifier,local_length,side,behavior,
                            capture_mm,capture_deg,locked,auto_connect,
                            propagate=False)
    if draft is None and behavior=='follow':
        try:
            return _stretch(assembly,identifier,float(length_mm),side,behavior,capture_mm,capture_deg,locked,auto_connect)
        except DocumentError as stretch_error:
            # Angled branches and sliding assemblies can be solved by translating
            # their rigid members even when a proportional stretch is impossible.
            stretch_reason=str(stretch_error)
        fixed='end' if side=='start' else 'start'
        result=resize_member(assembly,identifier,length_mm,collisions=False,suggest=False,fixed_end=fixed)
        if result['status']=='resized':
            doc=result['document'];after=Assembly.from_doc(doc,assembly.base,assembly.library)
            new_first,new_last,_=_frame(after,identifier)
            if auto_connect:
                _automatic_through(doc,after,identifier,first,last,new_first,new_last,side,capture_mm,capture_deg,locked)
            _valid_against(assembly,Assembly.from_doc(doc,assembly.base,assembly.library),
                           {identifier,*result['moved']})
            result['document']=doc;result['endpoint']=side
            return result
        raise DocumentError(result.get('reason') or stretch_reason)
    return _stretch(assembly,identifier,float(length_mm),side,behavior,capture_mm,capture_deg,locked,auto_connect)
