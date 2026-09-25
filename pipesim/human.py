"""19-segment articulated human, configurable measurements, IK and seated-fit tests.

The default body is an engineering mannequin, not a validated individual digital
twin. It uses explicit segment masses, geometric inertia, anatomical joint ranges,
self-collision and tunable dimensions. All generated parts/joints can be exported
and edited as ordinary PipeSim objects.
"""
from __future__ import annotations
import copy
import math
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from .math3d import transform, pose_of, align_axis, point
from .document import DocumentError

ARM_JOINTS=tuple(side+'_'+joint for side in ('left','right') for joint in ('clavicle_joint','shoulder','elbow','wrist'))
TORSO_JOINTS=('lumbar_flex','thoracic_flex','neck_base','neck_head')
LEG_JOINTS=tuple(side+'_'+joint for side in ('left','right') for joint in ('hip','knee','ankle'))
POSTURE_GROUPS={'arms':ARM_JOINTS,'torso':TORSO_JOINTS,'upper_body':TORSO_JOINTS+ARM_JOINTS,'legs':LEG_JOINTS}


def humanoid(stature_mm=1750,mass_kg=75,pose='standing',measurements=None,joint_angles_deg=None,hold_pose=False,strength_scale=1,hold_joints=None,grip_diameter_mm=None,joint_damping_nms_rad=.08):
    if not 500<=stature_mm<=2500 or not 5<=mass_kg<=350: raise DocumentError('Human stature must be 500–2500 mm and mass 5–350 kg')
    H=float(stature_mm); measures=measurements or {}
    allowed={'shoulder_width_mm','hip_width_mm','thigh_length_mm','shin_length_mm','upper_arm_length_mm','forearm_length_mm','hand_length_mm','foot_length_mm'}
    if set(measures)-allowed: raise DocumentError('Unknown human measurement: '+', '.join(sorted(set(measures)-allowed)))
    if any(not isinstance(v,(int,float)) or not np.isfinite(v) or v<=0 for v in measures.values()): raise DocumentError('Human measurements must be finite and positive')
    if not 0<strength_scale<=10: raise DocumentError('strength_scale must be between 0 and 10')
    if not isinstance(hold_pose,bool): raise DocumentError('hold_pose must be true or false; use hold_joints for selective posture control')
    if hold_joints is not None and (not isinstance(hold_joints,list) or any(not isinstance(j,str) for j in hold_joints)):
        raise DocumentError('hold_joints must be a list of anatomical joint names or posture groups')
    if grip_diameter_mm is not None and (not isinstance(grip_diameter_mm,(int,float)) or not 8<=grip_diameter_mm<=80):
        raise DocumentError('grip_diameter_mm must be between 8 and 80 mm')
    if not isinstance(joint_damping_nms_rad,(int,float)) or not np.isfinite(joint_damping_nms_rad) or joint_damping_nms_rad<0:
        raise DocumentError('joint_damping_nms_rad must be finite and nonnegative')
    shoulder=measures.get('shoulder_width_mm',.264*H)/2
    hip=measures.get('hip_width_mm',.11*H)/2
    thigh=measures.get('thigh_length_mm',.235*H)
    shin=measures.get('shin_length_mm',.235*H)
    upper=measures.get('upper_arm_length_mm',.175*H)
    forearm=measures.get('forearm_length_mm',.145*H)
    hand=measures.get('hand_length_mm',.105*H)
    foot=measures.get('foot_length_mm',.145*H)
    ankle_z=.05*H; knee_z=ankle_z+shin; hip_z=knee_z+thigh
    torso_offset=hip_z-.52*H
    parts=[]; joints=[]; weights={}
    def body(name,shape,position,weight,rotation=None):
        parts.append({'id':name,'pose':{'position_mm':list(position),'rotation_deg':list(rotation or [0,0,0])},'body':{'name':name.replace('_',' ').title(),'kind':'human','geometry':[shape],'material':'human-tissue','mass_kg':weight,'color':'#ddb087' if name in ('head','neck') or name.endswith('_hand') else '#648b91' if name in ('pelvis','lumbar','thorax') else '#7fa2a5','friction':.7,'restitution':.01,'source':{'geometry_status':'anthropometric mannequin approximation','mass_basis':'Normalised engineering segment fractions; individual calibration required'}}})
        weights[name]=weight
    def limb(name,a,b,radius,weight):
        a,b=np.array(a),np.array(b); length=np.linalg.norm(b-a)
        matrix=np.eye(4); matrix[:3,:3]=align_axis([0,0,1],b-a); matrix[:3,3]=(a+b)/2
        body(name,{'type':'capsule','radius_mm':radius,'length_mm':max(1,length-1.8*radius)},(a+b)/2,weight,pose_of(matrix)['rotation_deg'])
    def join(name,a,b,pivot,kind='spherical',axis=(1,0,0),limits=None):
        byid={p['id']:p for p in parts}
        ends=[]
        for pid in (a,b):
            matrix=transform(byid[pid]['pose']); inv=np.linalg.inv(matrix)
            ends.append({'part':pid,'frame':{'position_mm':point(inv,pivot).tolist(),'axis':(inv[:3,:3]@np.array(axis)).tolist(),'rotation_deg':pose_of(inv)['rotation_deg']}})
        joint={'id':name,'type':kind,'a':ends[0],'b':ends[1],'damping':joint_damping_nms_rad,'limits':limits or {'rotation_deg':[[-20,20],[-20,20],[-30,30]]},'metadata':{'anatomical':True,'limits_status':'engineering default, not person-specific'}}
        joints.append(joint)
    z=lambda f:f*H+torso_offset
    body('pelvis',{'type':'box','size_mm':[.18*H,.12*H,.11*H]},[0,0,z(.54)],.142)
    body('lumbar',{'type':'box','size_mm':[.14*H,.115*H,.10*H]},[0,0,z(.632)],.12)
    body('thorax',{'type':'box','size_mm':[.20*H,.13*H,.17*H]},[0,0,z(.745)],.20)
    body('neck',{'type':'capsule','radius_mm':.024*H,'length_mm':.035*H},[0,0,z(.856)],.011)
    body('head',{'type':'capsule','radius_mm':.045*H,'length_mm':.03*H},[0,0,z(.94)],.0694)
    join('lumbar_flex','pelvis','lumbar',[0,0,z(.585)],limits={'rotation_deg':[[-30,45],[-20,20],[-30,30]]})
    join('thoracic_flex','lumbar','thorax',[0,0,z(.67)],limits={'rotation_deg':[[-20,35],[-20,20],[-35,35]]})
    join('neck_base','thorax','neck',[0,0,z(.827)],limits={'rotation_deg':[[-35,40],[-25,25],[-40,40]]})
    join('neck_head','neck','head',[0,0,z(.89)],limits={'rotation_deg':[[-25,30],[-20,20],[-40,40]]})
    for side,sign in (('left',-1),('right',1)):
        shoulder_pos=[sign*shoulder,0,z(.80)]; collar=[sign*.035*H,0,z(.805)]
        elbow=[sign*shoulder,0,z(.80)-upper]; wrist=[sign*shoulder,0,elbow[2]-forearm]
        palm=[sign*shoulder,0,wrist[2]-hand]
        limb(side+'_clavicle',collar,shoulder_pos,.019*H,.008)
        limb(side+'_upper_arm',shoulder_pos,elbow,.027*H,.0271)
        limb(side+'_forearm',elbow,wrist,.022*H,.0162)
        limb(side+'_hand',wrist,palm,.023*H,.0061)
        parts[-1]['body']['ports']={'grip':{'type':'mount','label':'Palm grip centre','position_mm':[0,0,0],'axis':[1,0,0],'assembly':'hook','capacity':1}}
        if grip_diameter_mm is not None:
            # A curled rigid hand: the fingers surround a clear bore, and the
            # palm joins them to the wrist. No bar penetrates a solid hand capsule.
            inner=grip_diameter_mm/2+1; thickness=.008*H; outer=inner+thickness
            width=.043*H; palm_length=max(10,hand/2-outer+thickness)
            parts[-1]['body']['geometry']=[
                {'type':'box','size_mm':[width,.017*H,palm_length],'position_mm':[0,0,-hand/2+palm_length/2]},
                *[{'type':'tube','diameter_mm':2*outer,'wall_mm':thickness,'length_mm':width*.22,
                   'axis':[1,0,0],'position_mm':[(i-1.5)*width/4,0,0]} for i in range(4)]]
            parts[-1]['body']['ports']['grip']['diameter_mm']=grip_diameter_mm
            parts[-1]['body']['source']['grip_model']='Rigid curled fingers with 1 mm radial clearance; attachment supplies the assumed no-slip grip'
        join(side+'_clavicle_joint','thorax',side+'_clavicle',collar,limits={'rotation_deg':[[-15,15],[-15,15],[-20,20]]})
        join(side+'_shoulder',side+'_clavicle',side+'_upper_arm',shoulder_pos,limits={'rotation_deg':[[-60,180],[-110,110],[-90,90]]})
        join(side+'_elbow',side+'_upper_arm',side+'_forearm',elbow,'revolute',limits={'angle_deg':[0,150]})
        join(side+'_wrist',side+'_forearm',side+'_hand',wrist,limits={'rotation_deg':[[-70,80],[-25,35],[-75,75]]})
        hp=[sign*hip,0,hip_z]; knee=[sign*hip,0,knee_z]; ankle=[sign*hip,0,ankle_z]
        limb(side+'_thigh',hp,knee,.037*H,.1416)
        limb(side+'_shin',knee,ankle,.027*H,.0433)
        body(side+'_foot',{'type':'box','size_mm':[.055*H,foot,.045*H]},[sign*hip,foot*.25,.0225*H],.0137)
        join(side+'_hip','pelvis',side+'_thigh',hp,limits={'rotation_deg':[[-25,125],[-45,45],[-45,45]]})
        join(side+'_knee',side+'_thigh',side+'_shin',knee,'revolute',limits={'angle_deg':[-155,0]})
        join(side+'_ankle',side+'_shin',side+'_foot',ankle,limits={'rotation_deg':[[-45,25],[-20,20],[-15,15]]})
    total=sum(weights.values())
    for p in parts: p['body']['mass_kg']=mass_kg*weights[p['id']]/total
    angles={}
    if pose=='seated':
        for side in ('left','right'): angles.update({side+'_hip':[90,0,0],side+'_knee':-90,side+'_elbow':80})
    elif pose=='crouching':
        angles['lumbar_flex']=[30,0,0]
        for side in ('left','right'): angles.update({side+'_hip':[100,0,0],side+'_knee':-125,side+'_ankle':[25,0,0]})
    elif pose=='pull-up':
        for side in ('left','right'):
            angles.update({side+'_shoulder':[80,0,0],side+'_elbow':135,side+'_wrist':[-35,0,0],
                           side+'_hip':[4,0,0],side+'_knee':-8,side+'_ankle':[4,0,0]})
    elif pose!='standing': raise DocumentError('Human pose must be standing, seated, crouching or pull-up')
    angles.update(joint_angles_deg or {})
    joint_names={j['id'] for j in joints}
    if set(angles)-joint_names: raise DocumentError('Unknown anatomical joint: '+', '.join(sorted(set(angles)-joint_names)))
    held=set(joint_names) if hold_pose else set()
    for name in hold_joints or []:
        if name in POSTURE_GROUPS: held.update(POSTURE_GROUPS[name])
        elif name in joint_names: held.add(name)
        else: raise DocumentError('Unknown posture joint or group: '+name)
    matrices={p['id']:transform(p['pose']) for p in parts}
    # Anatomical angles are in the world-aligned neutral body axes. Store joint axes
    # in the actual parent frame so simulation and IK use the same coordinates.
    for j in joints:
        if j['id'] not in angles: continue
        value=angles[j['id']]
        vector=[value,0,0] if isinstance(value,(float,int)) else value
        if j['type']=='revolute':
            lo,hi=j['limits']['angle_deg']
            if not lo<=vector[0]<=hi: raise DocumentError(f"{j['id']}: anatomical angle outside limits")
        else:
            if any(not lo<=v<=hi for v,(lo,hi) in zip(vector,j['limits']['rotation_deg'])): raise DocumentError(f"{j['id']}: anatomical angle outside limits")
        parent=matrices[j['a']['part']]; pivot=point(parent,j['a']['frame']['position_mm'])
        # Flexion axis is global X, transformed by preceding joint movement.
        neutral_parent=transform(next(p['pose'] for p in parts if p['id']==j['a']['part']))
        basis=parent[:3,:3]@neutral_parent[:3,:3].T
        r=basis@Rotation.from_euler('XYZ',vector,degrees=True).as_matrix()@basis.T
        descendants={j['b']['part']}
        for _ in parts:
            for child in joints:
                if child['a']['part'] in descendants: descendants.add(child['b']['part'])
        delta=np.eye(4); delta[:3,:3]=r; delta[:3,3]=pivot-r@pivot
        for pid in descendants: matrices[pid]=delta@matrices[pid]
        if j['type']=='revolute': j['limits']['angle_deg']=[v-vector[0] for v in j['limits']['angle_deg']]
        else: j['limits']['rotation_deg']=[[lo-v,hi-v] for v,(lo,hi) in zip(vector,j['limits']['rotation_deg'])]
        j['metadata']['reference_anatomical_angles_deg']=vector
    for p in parts: p['pose']=pose_of(matrices[p['id']])
    if held:
        for j in joints:
            if j['id'] not in held: continue
            capacity=120 if '_hip' in j['id'] else 90 if '_knee' in j['id'] else 40 if '_ankle' in j['id'] else 35 if '_shoulder' in j['id'] else 20 if '_elbow' in j['id'] else 5 if '_wrist' in j['id'] else 10 if 'neck' in j['id'] else 60
            j['motor']={'mode':'position','target':0,'max_torque_nm':capacity*strength_scale*mass_kg/75,'kp':35,'kd':1}
            if j['type']=='spherical': j['motor']['rotation_deg']=[0,0,0]
            j['metadata']['actuation']='bounded posture servo; assumed strength, no balance or muscle physiology model'
    # Re-express joint pivots in their transformed bodies after posing. Child and
    # parent attachment points already follow local coordinates; both still coincide.
    return {'parts':parts,'joints':joints,'metadata':{'segments':19,'stature_mm':stature_mm,'mass_kg':mass_kg,'pose':pose,'measurements':measures,'held_joints':sorted(held),'status':'configurable engineering mannequin; calibrate to the intended person','sources':['https://pubmed.ncbi.nlm.nih.gov/8872282/','https://opensimconfluence.atlassian.net/wiki/spaces/OpenSim/pages/53089158/How+Scaling+Works']}}

def reach(assembly,human,target_mm,hand='right',check_collision=True):
    if hand not in ('left','right'): raise DocumentError('hand must be left or right')
    prefix=human+'/'
    shoulder=next((j for j in assembly.joints if j['id']==prefix+hand+'_shoulder'),None)
    elbow=next((j for j in assembly.joints if j['id']==prefix+hand+'_elbow'),None)
    wrist=next((j for j in assembly.joints if j['id']==prefix+hand+'_wrist'),None)
    if any(j is None for j in (shoulder,elbow,wrist)): raise DocumentError(f'{human}: no articulated {hand} arm')
    target=np.array(target_mm,float)
    chain=[shoulder,elbow,wrist]
    initial={pid:p.matrix.copy() for pid,p in assembly.parts.items()}
    descendants=[]
    for joint in chain:
        affected={joint['b']['part']}
        for _ in assembly.parts:
            for child in assembly.joints:
                if child['a']['part'] in affected and child['b']['part'].startswith(prefix): affected.add(child['b']['part'])
        descendants.append(affected)
    lower=[]; upper=[]
    for j in chain:
        limits=j['limits']['rotation_deg'] if j['type']=='spherical' else [j['limits']['angle_deg']]
        lower.extend(lo for lo,hi in limits); upper.extend(hi for lo,hi in limits)
    def posed(values):
        matrices={pid:m.copy() for pid,m in initial.items()}; index=0
        for j,affected in zip(chain,descendants):
            parent=matrices[j['a']['part']]
            pivot=point(parent,j['a']['frame']['position_mm'])
            if j['type']=='spherical':
                angles=values[index:index+3]; index+=3
                basis=parent[:3,:3]@Rotation.from_euler('xyz',j['a']['frame'].get('rotation_deg',[0,0,0]),degrees=True).as_matrix()
                r=basis@Rotation.from_euler('XYZ',angles,degrees=True).as_matrix()@basis.T
            else:
                axis=parent[:3,:3]@np.array(j['a']['frame']['axis'])
                r=Rotation.from_rotvec(axis*math.radians(values[index])).as_matrix(); index+=1
            delta=np.eye(4); delta[:3,:3]=r; delta[:3,3]=pivot-r@pivot
            for pid in affected: matrices[pid]=delta@matrices[pid]
        return matrices
    handid=prefix+hand+'_hand'
    def residual(values):
        matrices=posed(values)
        return np.r_[(matrices[handid][:3,3]-target),np.array(values)*.002]
    best=None
    for seed in (np.zeros(7),(np.array(lower)+np.array(upper))/2,np.array([90,0,0,90,0,0,0])):
        result=least_squares(residual,np.clip(seed,lower,upper),bounds=(lower,upper),max_nfev=250,ftol=1e-9,xtol=1e-9,gtol=1e-9)
        if best is None or np.linalg.norm(result.fun[:3])<np.linalg.norm(best.fun[:3]): best=result
    matrices=posed(best.x); error=float(np.linalg.norm(matrices[handid][:3,3]-target))
    collisions=[]
    if check_collision and error<=5:
        from .validation import CollisionWorld
        candidate=copy.deepcopy(assembly)
        for pid,m in matrices.items(): candidate.parts[pid].matrix=m
        adjacent={frozenset([j['a']['part'],j['b']['part']]) for j in assembly.joints}
        moving=descendants[0]
        with CollisionWorld(candidate) as world:
            for a in sorted(moving):
                for b in candidate.parts:
                    if a==b or frozenset([a,b]) in adjacent or (b in moving and a>b): continue
                    if world.intersect(a,b,3): collisions.append([a,b])
    return {'human':human,'hand':hand,'reachable':error<=5 and not collisions,'position_reachable':error<=5,'target_mm':target.tolist(),'palm_mm':matrices[handid][:3,3].tolist(),'error_mm':error,'collisions':collisions,'angles_deg':{shoulder['id']:best.x[:3].tolist(),elbow['id']:float(best.x[3]),wrist['id']:best.x[4:].tolist()},'parts':{pid:pose_of(matrices[pid]) for pid in descendants[0]},'limitations':['Static pose feasibility; no collision-free reach trajectory is implied','Results depend on individual segment dimensions and joint ranges']}

def seat_fit(assembly,human,seat):
    from .geometry import bounds,mesh_for_part
    if seat not in assembly.parts or human+'/pelvis' not in assembly.parts: raise DocumentError('Seat or human does not exist')
    panel=assembly.parts[seat]; box=bounds(panel); pelvis=assembly.parts[human+'/pelvis']; pb=bounds(pelvis)
    if abs(panel.matrix[2,2])<.98: raise DocumentError('Seat fit currently requires a nearly horizontal seat')
    local_box=mesh_for_part(panel).bounds
    width=float(local_box[1,0]-local_box[0,0]); depth=float(local_box[1,1]-local_box[0,1]); hip_width=float(mesh_for_part(pelvis).extents[0])
    thigh=assembly.parts[human+'/right_thigh']; shin=assembly.parts[human+'/right_shin']; foot=assembly.parts[human+'/right_foot']
    def joint_point(name):
        j=next(j for j in assembly.joints if j['id']==human+'/'+name)
        return assembly.parts[j['a']['part']].frame(j['a'])[0]
    hip,knee,ankle=[joint_point('right_'+name) for name in ('hip','knee','ankle')]
    thigh_length=float(np.linalg.norm(knee-hip)); shin_length=float(np.linalg.norm(ankle-knee))
    height=float(box[1,2]-assembly.doc.get('environment',{}).get('ground_z_mm',0))
    required_depth=thigh_length*.65
    comfortable_height=shin_length+float(bounds(foot)[1,2]-bounds(foot)[0,2])/2
    width_ok=width>=hip_width+40; depth_ok=depth>=required_depth
    ground=assembly.doc.get('environment',{}).get('ground_z_mm',0)
    feet_gaps=[float(bounds(assembly.parts[human+'/'+side+'_foot'])[0,2]-ground) for side in ('left','right')]
    return {'human':human,'seat':seat,'fits':bool(width_ok and depth_ok),'available_width_mm':width,'required_width_mm':hip_width+40,'available_depth_mm':depth,'required_depth_mm':required_depth,'seat_height_mm':height,'estimated_popliteal_height_mm':comfortable_height,'feet_supported':all(-5<=gap<=20 for gap in feet_gaps),'checks':{'hip_clearance':bool(width_ok),'thigh_support':bool(depth_ok)},'current_pose':{'thigh_nearly_horizontal':abs(float((knee-hip)[2]))<thigh_length*.35,'foot_ground_gaps_mm':feet_gaps,'pelvis_seat_gap_mm':float(pb[0,2]-box[1,2])},'suggested_pelvis_position_mm':[float((box[0,0]+box[1,0])/2),float(box[1,1]-required_depth*.5),float(box[1,2]+(pb[1,2]-pb[0,2])/2)],'limitations':['Dimensional screening only; verify a seated pose using rigid-body contact simulation','Soft tissue compression, comfort and personal mobility are not inferred']}

def clearance(assembly,human,clearance_mm=0):
    from .validation import CollisionWorld
    import pybullet as pb
    humans=[pid for pid in assembly.parts if pid.startswith(human+'/')]
    if not humans: raise DocumentError('Unknown human '+human)
    if clearance_mm<0: raise DocumentError('Clearance must be nonnegative')
    violations=[]
    with CollisionWorld(assembly) as world:
        for pid in humans:
            for other in assembly.parts.keys()-set(humans):
                points=pb.getClosestPoints(world.world.part_map[pid][0],world.world.part_map[other][0],distance=(clearance_mm+1)/1000,physicsClientId=world.world.client)
                separation=min((c[8]*1000 for c in points),default=clearance_mm+1)
                if separation<clearance_mm-.5: violations.append({'parts':[pid,other],'clearance_mm':separation})
    return {'human':human,'clear':not violations,'requested_clearance_mm':clearance_mm,'violations':violations,'limitations':['Checks the current geometric pose; padding, clothing and personal movement allowances must be supplied explicitly']}

def mesh_vertices(part):
    from .geometry import mesh_for_part
    return mesh_for_part(part).vertices

def run_fit_tests(assembly):
    results=[]
    for test in assembly.doc.get('tests',[]):
        if test['type']=='reach':
            result=reach(assembly,test['human'],test['target_mm'],test.get('hand','right')); observed=result['reachable']
        elif test['type']=='seat':
            result=seat_fit(assembly,test['human'],test['seat']); observed=result['fits']
        elif test['type']=='clearance':
            result=clearance(assembly,test['human'],test.get('clearance_mm',0)); observed=result['clear']
        results.append({'id':test['id'],'passed':observed==test.get('expect',True),'result':result})
    return {'passed':all(t['passed'] for t in results),'tests':results}
