"""Transfer recorded external contact wrenches into an explicit frame-FEA load case."""
import copy
import numpy as np
from .document import Assembly,DocumentError,joint_kind
from .editing import snapshot_design
from .math3d import transform

def contact_load_case(assembly,recording,parts=None,window_s=.25):
    if recording.get('input_sha256') and recording['input_sha256']!=assembly.input_hash:
        raise DocumentError('Simulation recording is stale for this design or its libraries')
    if window_s<=0: raise DocumentError('Contact averaging window must be positive')
    selected=set(parts or [pid for pid,p in assembly.parts.items() if p.kind=='member'])
    if selected-set(assembly.parts): raise DocumentError('Unknown structural part in contact load case')
    for _ in assembly.parts:
        for j in assembly.joints:
            ends={j['a']['part'],j['b']['part']}
            if joint_kind(j)=='fixed' and ends&selected: selected|=ends
    if not selected: raise DocumentError('Select structural parts, or add members connected to the structure')
    frames=recording['frames']; last=frames[-1]
    samples=[f for f in frames if f['time_s']>=last['time_s']-window_s]
    doc=snapshot_design(assembly,last)
    doc['parts']=[p for p in doc['parts'] if p['id'] in selected]
    doc['joints']=[j for j in doc['joints'] if {j['a']['part'],j['b']['part']}<=selected]
    doc['anchors']=[a for a in doc.get('anchors',[]) if a['part'] in selected]
    doc['loads']=[l for l in doc.get('loads',[]) if l['part'] in selected]
    doc.pop('tests',None); doc.pop('drives',None)
    # For compound Bullet bodies, identify the contacted constituent from its
    # actual triangle surface rather than assigning the entire load to a random leg.
    from .geometry import mesh_for_part
    from trimesh.proximity import closest_point_naive
    local_meshes={pid:mesh_for_part(assembly.parts[pid]) for pid in selected}
    wrenches={}; contact_count=0
    for frame in samples:
        for contact in frame.get('contacts',[]):
            aa=set(contact['a'])&selected; bb=set(contact['b'])&selected
            if bool(aa)==bool(bb): continue
            external=contact['b'] if aa else contact['a']
            if external==['world']: continue
            candidates=aa or bb
            position=np.array(contact['position_a_mm'] if aa else contact['position_b_mm'])
            force=np.array(contact['force_on_a_n'])*(1 if aa else -1)/len(samples)
            def separation(pid):
                matrix=transform(frame['parts'][pid]); local=matrix[:3,:3].T@(position-matrix[:3,3])
                return closest_point_naive(local_meshes[pid],local[None,:])[1][0]
            pid=min(candidates,key=separation)
            origin=np.array(last['parts'][pid]['position_mm'])
            f,m=wrenches.setdefault(pid,[np.zeros(3),np.zeros(3)])
            f+=force; m+=np.cross((position-origin)/1000,force); contact_count+=1
    for pid,(force,moment) in wrenches.items():
        doc['loads'].append({'part':pid,'point_mm':[0,0,0],'force_n':force.tolist(),'moment_nm':moment.tolist()})
    doc.setdefault('metadata',{})['contact_load_case']={'source_input_sha256':assembly.input_hash,'samples':len(samples),'contacts':contact_count,'window_s':window_s,'resultants':{pid:{'force_n':f.tolist(),'moment_nm':m.tolist()} for pid,(f,m) in wrenches.items()},'limitations':['Contact forces are averaged over the selected time window; use a short window for impact envelopes','Each resultant is applied at the part origin and preserves net force and moment; distributed local bending requires explicit attachment stations','Original real world anchors are retained; ground contacts are not silently converted to clamps']}
    doc['name']=assembly.doc.get('name','Design')+' · recorded contact load case'
    return doc
