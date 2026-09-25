"""Linear 3D Euler–Bernoulli frame elements: axial, bending, torsion and rigid offsets.

Each beam element has twelve degrees of freedom. Socket locations split members;
rigid connectors transfer force and moment through exact kinematic offset maps.
Constraint nullspaces expose mechanisms instead of hiding them with weak springs.
"""
from __future__ import annotations
import copy
import math
import numpy as np
from scipy.linalg import null_space, eigh
from .document import joint_kind, fingerprint, DocumentError
from .math3d import point, skew, axis_frame, unit

def section_properties(part):
    s=part.section; kind=s.get('type')
    if kind in ('tube','solid_round'):
        d=s['diameter_mm']/1000
        inside=d-2*s['wall_mm']/1000 if kind=='tube' else 0
        if not 0<=inside<d: raise DocumentError(f'{part.id}: invalid cross-section')
        area=math.pi*(d*d-inside*inside)/4
        inertia=math.pi*(d**4-inside**4)/64
        return {'area':area,'iy':inertia,'iz':inertia,'j':2*inertia,'cy':d/2,'cz':d/2,'round':True}
    if kind=='rectangular':
        w,d=s['width_mm']/1000,s['depth_mm']/1000
        a,b=max(w,d),min(w,d)
        return {'area':w*d,'iy':w*d**3/12,'iz':d*w**3/12,'j':a*b**3*(1/3-.21*b/a*(1-b**4/(12*a**4))),'cy':w/2,'cz':d/2,'round':False}
    if kind=='explicit':
        return {'area':s['area_mm2']*1e-6,'iy':s['iy_mm4']*1e-12,'iz':s['ix_mm4']*1e-12,'j':s['j_mm4']*1e-12,'cy':s['cx_mm']/1000,'cz':s['cy_mm']/1000,'round':False}
    raise DocumentError(f'{part.id}: member needs tube, solid_round, rectangular or explicit section properties')

def beam_stiffness(length,youngs,shear,section):
    L=length; s=section; k=np.zeros((12,12))
    def insert(indices,values): k[np.ix_(indices,indices)]+=values
    insert([0,6],youngs*s['area']/L*np.array([[1,-1],[-1,1]]))
    insert([3,9],shear*s['j']/L*np.array([[1,-1],[-1,1]]))
    insert([1,5,7,11],youngs*s['iz']/L**3*np.array([[12,6*L,-12,6*L],[6*L,4*L*L,-6*L,2*L*L],[-12,-6*L,12,-6*L],[6*L,2*L*L,-6*L,4*L*L]]))
    insert([2,4,8,10],youngs*s['iy']/L**3*np.array([[12,-6*L,-12,-6*L],[-6*L,4*L*L,6*L,2*L*L],[-12,6*L,12,6*L],[-6*L,2*L*L,6*L,4*L*L]]))
    return k

def analyse(assembly,include_self_weight=True):
    members={pid:p for pid,p in assembly.parts.items() if p.kind=='member'}
    if not members:
        return {'status':'no_members','input_sha256':assembly.input_hash,'members':[],'message':'Add a structural member with a material and section definition'}
    stations={pid:{0.,float(p.length)} for pid,p in members.items()}
    for j in assembly.joints:
        for end in ('a','b'):
            e=j[end]; pid=e['part']
            if pid in stations:
                p=members[pid]; local,_=p.local_frame(e)
                stations[pid].add(round(float(local[2]+p.length/2),6))
    for load in assembly.doc.get('loads',[]):
        if load['part'] in stations:
            p=members[load['part']]
            stations[p.id].add(round(float(load.get('at_mm',load.get('point_mm',[0,0,0])[2]+p.length/2)),6))
    nodes=[]; node_map={}; rigid_nodes={}
    for pid,p in members.items():
        for station in sorted(stations[pid]):
            if station<0 or station>p.length: raise DocumentError(f'{pid}: FEA attachment/load lies outside the member')
            node_map[(pid,station)]=len(nodes)
            nodes.append(point(p.matrix,[0,0,station-p.length/2])/1000)
    for pid,p in assembly.parts.items():
        if pid not in members:
            rigid_nodes[pid]=len(nodes); nodes.append(p.matrix[:3,3]/1000)
    nodes=np.array(nodes); ndof=len(nodes)*6
    K=np.zeros((ndof,ndof)); F=np.zeros(ndof); elements=[]
    gravity=np.array(assembly.doc.get('environment',{}).get('gravity_m_s2',[0,0,-9.81])) if include_self_weight else np.zeros(3)
    for pid,p in members.items():
        material=p.definition.get('material_data',{})
        if 'youngs_modulus_pa' not in material: raise DocumentError(f'{pid}: Young modulus is unknown')
        E=material['youngs_modulus_pa']; nu=material.get('poisson_ratio',.3); G=E/(2*(1+nu)); s=section_properties(p)
        if min(s[k] for k in ('area','iy','iz','j'))<=0: raise DocumentError(f'{pid}: section properties must be positive')
        basis=np.column_stack([p.matrix[:3,2],p.matrix[:3,0],p.matrix[:3,1]])
        T=np.zeros((12,12))
        for i in range(4): T[3*i:3*i+3,3*i:3*i+3]=basis.T
        ordered=sorted(stations[pid])
        for a,b in zip(ordered,ordered[1:]):
            L=(b-a)/1000
            if L<1e-7: continue
            n1,n2=node_map[(pid,a)],node_map[(pid,b)]
            ids=np.r_[np.arange(n1*6,n1*6+6),np.arange(n2*6,n2*6+6)]
            local=beam_stiffness(L,E,G,s); K[np.ix_(ids,ids)]+=T.T@local@T
            q=basis.T@gravity*(p.mass/(p.length/1000))
            load=np.zeros(12); load[:3]=q*L/2; load[6:9]=q*L/2
            load[4]=-q[2]*L*L/12; load[5]=q[1]*L*L/12
            load[10]=q[2]*L*L/12; load[11]=-q[1]*L*L/12
            F[ids]+=T.T@load
            elements.append({'part':pid,'a_mm':a,'b_mm':b,'nodes':[n1,n2],'indices':ids,'k':local,'T':T,'fixed_end_load':load,'section':s,'youngs':E})
    for pid,node in rigid_nodes.items():
        p=assembly.parts[pid]; force=p.mass*gravity
        com=point(p.matrix,p.center_of_mass)/1000
        F[node*6:node*6+3]+=force
        F[node*6+3:node*6+6]+=np.cross(com-nodes[node],force)

    def endpoint(e,target=None):
        pid=e['part']; p=assembly.parts[pid]; local,_=p.local_frame(e)
        pos=point(p.matrix,local)/1000
        if pid in members:
            station=round(float(local[2]+p.length/2),6)
            node=node_map[(pid,station)]
        else: node=rigid_nodes[pid]
        offset=(target if target is not None else pos)-nodes[node]
        J=np.eye(6); J[:3,3:]=-skew(offset)
        return node,J

    for load in assembly.doc.get('loads',[]):
        pid=load['part']
        if pid not in assembly.parts: raise DocumentError(f'Unknown loaded part {pid}')
        p=assembly.parts[pid]; station=float(load.get('at_mm',load.get('point_mm',[0,0,0])[2]+p.length/2))
        e={'part':pid,'at_mm':station} if pid in members else {'part':pid}
        local=load.get('point_mm',[0,0,station-p.length/2] if pid in members else [0,0,0])
        target=point(p.matrix,local)/1000
        node,J=endpoint(e,target)
        force=np.r_[load.get('force_n',[0,0,0]),load.get('moment_nm',[0,0,0])]
        F[node*6:node*6+6]+=J.T@force
    constraints=[]; labels=[]
    for j in assembly.joints:
        pa,pb,axis=assembly.joint_frames(j); target=(pa+pb)/2000
        a,Ja=endpoint(j['a'],target); b,Jb=endpoint(j['b'],target)
        kind=joint_kind(j)
        if kind=='fixed': selectors=np.eye(6)
        elif kind=='spherical': selectors=np.eye(6)[:3]
        else:
            basis=axis_frame(axis)
            if kind=='revolute': selectors=np.vstack([np.eye(6)[:3],np.c_[np.zeros((2,3)),basis[:,:2].T]])
            elif kind=='prismatic': selectors=np.vstack([np.c_[basis[:,:2].T,np.zeros((2,3))],np.eye(6)[3:]])
            elif kind=='cylindrical': selectors=np.vstack([np.c_[basis[:,:2].T,np.zeros((2,3))],np.c_[np.zeros((2,3)),basis[:,:2].T]])
            else:
                return {'status':'unsupported_constraint','joint':j['id'],'message':'Tension-only and distance members need nonlinear cable analysis or contact forces from a rigid-body simulation','input_sha256':assembly.input_hash}
        for row in selectors:
            C=np.zeros(ndof); C[a*6:a*6+6]+=row@Ja; C[b*6:b*6+6]-=row@Jb
            constraints.append(C); labels.append(('joint',j['id'],row))
    for anchor in assembly.anchors:
        pid=anchor['part']; p=assembly.parts[pid]
        if pid in members:
            # Anchor a member's start unless an explicit world point identifies another station.
            station=0.
            if 'position_mm' in anchor: station=float((np.array(anchor['position_mm'])-p.matrix[:3,3])@p.matrix[:3,2]+p.length/2)
            station=min(stations[pid],key=lambda v:abs(v-station)); e={'part':pid,'at_mm':station}
        else: e={'part':pid}
        target=np.array(anchor.get('position_mm',p.matrix[:3,3]))/1000
        node,J=endpoint(e,target)
        for dof in anchor.get('dofs',['x','y','z','rx','ry','rz']):
            index=['x','y','z','rx','ry','rz'].index(dof)
            C=np.zeros(ndof); C[node*6:node*6+6]=J[index]
            constraints.append(C); labels.append(('anchor',pid,np.eye(6)[index]))
    C=np.array(constraints) if constraints else np.empty((0,ndof))
    Q=null_space(C,rcond=1e-10) if len(C) else np.eye(ndof)
    reduced=Q.T@K@Q; reduced=(reduced+reduced.T)/2
    rhs=Q.T@F
    if reduced.size:
        diagonal=np.sqrt(np.maximum(np.abs(np.diag(reduced)),1e-14))
        scaled=reduced/diagonal[:,None]/diagonal[None,:]
        eigenvalues,eigenvectors=eigh(scaled)
        threshold=max(eigenvalues.max(),1)*1e-10
        free=eigenvalues<threshold
        if np.any(free):
            unbalanced=eigenvectors[:,free].T@(rhs/diagonal)
            return {'status':'mechanism','input_sha256':assembly.input_hash,'free_modes':int(free.sum()),'unbalanced_mode_load':float(np.linalg.norm(unbalanced)),'message':'Structure is underconstrained at this pose. Add real anchors/bracing or settle and lock moving joints before static frame analysis.','members':[]}
        solution=(eigenvectors@((eigenvectors.T@(rhs/diagonal))/eigenvalues))/diagonal
        displacement=Q@solution
    else: displacement=np.zeros(ndof)
    residual=K@displacement-F
    multipliers=np.linalg.lstsq(C.T,-residual,rcond=1e-10)[0] if len(C) else np.zeros(0)
    balance=float(np.linalg.norm(residual+(C.T@multipliers if len(C) else 0)))
    member_results={}; assumptions=set(); joint_forces={}; anchors={}
    for label,multiplier in zip(labels,multipliers):
        kind,id,row=label
        target=joint_forces if kind=='joint' else anchors
        target.setdefault(id,np.zeros(6)); target[id]+=row*multiplier
    for element in elements:
        pid=element['part']; p=members[pid]; s=element['section']
        forces=element['k']@element['T']@displacement[element['indices']]-element['fixed_end_load']
        ends=forces.reshape(2,6)
        stress=0.; compression=0.
        for i,force in enumerate(ends):
            N,Vy,Vz,T,My,Mz=force
            normal=abs(N)/s['area']
            if s['round']: normal+=math.hypot(My,Mz)*s['cy']/s['iy']
            else: normal+=abs(My)*s['cz']/s['iy']+abs(Mz)*s['cy']/s['iz']
            tau=abs(T)*max(s['cy'],s['cz'])/s['j']+4/3*math.hypot(Vy,Vz)/s['area']
            stress=max(stress,math.sqrt(normal**2+3*tau**2))
            compression=max(compression,N if i==0 else -N)
        yield_strength=p.definition['material_data'].get('yield_strength_pa')
        effective=p.section.get('effective_length_factor',2)*(p.length/1000)
        buckling=math.pi**2*element['youngs']*min(s['iy'],s['iz'])/effective**2
        result=member_results.setdefault(pid,{'part':pid,'max_von_mises_mpa':0.,'yield_utilisation':0. if yield_strength else None,'buckling_utilisation':0.,'max_displacement_mm':0.,'elements':[],'material_status':p.definition['material_data'].get('status','unknown')})
        result['max_von_mises_mpa']=max(result['max_von_mises_mpa'],stress/1e6)
        if yield_strength: result['yield_utilisation']=max(result['yield_utilisation'],stress/yield_strength)
        result['buckling_utilisation']=max(result['buckling_utilisation'],max(0,compression)/buckling)
        result['max_displacement_mm']=max(result['max_displacement_mm'],*(np.linalg.norm(displacement[n*6:n*6+3])*1000 for n in element['nodes']))
        result['elements'].append({'from_mm':element['a_mm'],'to_mm':element['b_mm'],'end_forces_local_n_nm':ends.tolist(),'von_mises_mpa':stress/1e6,'euler_buckling_n':buckling})
        assumptions.add(p.definition['material_data'].get('status','unknown material status'))
    connectors=[]
    for j in assembly.joints:
        force=joint_forces.get(j['id'],np.zeros(6))
        p=assembly.parts[j['a']['part']]; strength=p.definition.get('strength',{})
        _,_,axis=assembly.joint_frames(j)
        axial=abs(float(force[:3]@axis)); rating=strength.get('axial_slip_n')
        applicable=rating is not None and j.get('torque_nm',0)>=strength.get('required_torque_nm',math.inf) and joint_kind(j)=='fixed'
        connectors.append({'joint':j['id'],'part':p.id,'force_world_n':force[:3].tolist(),'moment_world_nm':force[3:].tolist(),'axial_n':axial,'axial_slip_utilisation':axial/rating if applicable else None,'capacity_status':strength.get('rating_status','unknown'),'unknown_modes':['casting fracture','bending capacity','torsional capacity'],'rating_condition_met':applicable})
    failures=[{'part':r['part'],'mode':mode,'utilisation':r[field]} for r in member_results.values() for field,mode in [('yield_utilisation','yield'),('buckling_utilisation','euler_buckling')] if r[field] is not None and r[field]>1]
    failures += [{'joint':c['joint'],'part':c['part'],'mode':'conditional_axial_slip','utilisation':c['axial_slip_utilisation']} for c in connectors if c['axial_slip_utilisation'] is not None and c['axial_slip_utilisation']>1]
    return {'status':'solved','input_sha256':assembly.input_hash,'method':'3D Euler–Bernoulli frame elements with rigid offsets and constraint nullspace','members':list(member_results.values()),'connectors':connectors,'anchor_reactions':{pid:(-f).tolist() for pid,f in anchors.items()},'nodes':[{'position_mm':(pos*1000).tolist(),'translation_mm':(displacement[6*i:6*i+3]*1000).tolist(),'rotation_deg':np.rad2deg(displacement[6*i+3:6*i+6]).tolist()} for i,pos in enumerate(nodes)],'equilibrium_residual_n_nm':balance,'failures':failures,'certified':False,'assumptions':sorted(assumptions),'limitations':['Small deflections; linear elastic isotropic beams; no shell/panel stress, contact, plasticity or casting fracture','Euler buckling uses effective length factor 2 unless overridden in section data','Connector axial ratings are conditional supplier statements; missing bending/fracture capacities remain unknown','Rigid panels and humanoids transfer gravity through explicit joints; contact loads require a separate load case']}

def destruction_test(assembly,part,maximum_force_n=10000,directions=24,steps=12,seed=1,rigid_duration_s=0):
    """Seeded multiaxial force ramp; reports thresholds and untested capacity modes."""
    if part not in assembly.parts: raise DocumentError(f'Unknown load target {part}')
    if maximum_force_n<=0 or directions<1 or steps<1: raise ValueError('Force, directions and steps must be positive')
    from .document import Assembly
    rng=np.random.default_rng(seed)
    vectors=[np.eye(3)[i]*s for i in range(3) for s in (-1,1)]
    vectors += [unit(v) for v in rng.normal(size=(max(0,directions-6),3))]
    trials=[]
    for direction in vectors[:max(6,directions)]:
        first=None; last_force=0
        for magnitude in np.linspace(maximum_force_n/steps,maximum_force_n,steps):
            doc=copy.deepcopy(assembly.doc)
            doc.setdefault('loads',[]).append({'part':part,'force_n':(direction*magnitude).tolist(),'at_mm':assembly.parts[part].length if assembly.parts[part].kind=='member' else 0})
            result=analyse(Assembly.from_doc(doc,assembly.base))
            if result['status']!='solved' or result['failures']:
                first={'force_n':float(magnitude),'previous_pass_force_n':float(last_force),'status':result['status'],'failures':result.get('failures',[])}; break
            last_force=magnitude
        trials.append({'direction':direction.tolist(),'first_failure':first,'last_tested_force_n':float(magnitude),'unknown_capacity_modes':True})
    result={'input_sha256':assembly.input_hash,'seed':seed,'part':part,'trials':trials,'method':'linear frame analysis with increasing loads in six cardinal and seeded random directions','limitations':['These are model-predicted yield, buckling and conditional axial slip thresholds, not measured destructive test data','Unknown connector fracture is not inferred from an absent rating']}
    if rigid_duration_s:
        from .physics import rigid_load_trials
        result['rigid_body_trials']=rigid_load_trials(assembly,part,vectors[:max(6,directions)],maximum_force_n,steps,rigid_duration_s)
    else: result['limitations'].append('Rigid-body tipping and sliding not tested in this run; use --rigid')
    return result
