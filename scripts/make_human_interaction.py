"""Regenerate only the pull-up interaction examples, using catalogue sockets."""
from pathlib import Path
import copy
import sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from pipesim.document import Assembly,Library,write
from pipesim.human import humanoid
from pipesim.math3d import align_axis,point,pose_of,transform

REFERENCE='https://in.pinterest.com/pin/808466570608639102/'


def pull_up_design():
    library=Library.load(); specs={}; matrices={}
    doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','name':'Human pull-up · held arms, swinging legs',
         'description':'Photo-inspired C42 pull-up cage with overhead ladder rungs. Both hands grip the front bar. '
                       'Bounded upper-body posture motors hold the bent arms; hips, knees and ankles remain passive. '
                       'Initially bent legs swing under gravity, without animation tracks. Floor flanges are explicitly anchored.',
         'parts':[],'joints':[],'anchors':[],'environment':{'ground':True,'gravity_m_s2':[0,0,-9.81]},
         'metadata':{'reference_url':REFERENCE,'dimensions_status':'Example dimensions chosen for this mannequin; not measured from the photograph',
                     'grip_assumption':'Hands cannot slide along or release the bar; each may rotate about the bar axis',
                     'posture_assumption':'Finite joint-torque controllers approximate isometric upper-body effort; strengths are adjustable assumptions',
                     'damping_assumption':'Passive viscous resistance of 1 N m s/rad per anatomical axis; adjustable, not measured physiology',
                     'simulation_duration_s':4,'camera':{'eye_mm':[3000,-4300,2750],'target_mm':[0,0,1250]}}}

    def part(pid,code,position,rotation=None,length=None):
        matrix=np.eye(4);matrix[:3,3]=position
        if rotation is not None: matrix[:3,:3]=rotation
        spec={'id':pid,'catalog':'tubeclamp.'+code,'pose':pose_of(matrix)}
        if length is not None: spec['parameters']={'length_mm':round(float(length),6)}
        doc['parts'].append(spec);specs[pid]=spec;matrices[pid]=matrix
        return pid

    def socket(pid,direction):
        for name,p in library.parts[specs[pid]['catalog']]['ports'].items():
            if p['type']=='socket' and not p.get('through') and np.dot(matrices[pid][:3,:3]@p['axis'],direction)>.999:
                return name,p
        raise ValueError('No terminal socket for '+pid)

    def attach(fitting,port,tube,**end):
        depth=end.pop('insertion_mm',0)
        doc['joints'].append({'id':fitting+'-'+port,'type':'socket','a':{'part':fitting,'port':port},
                             'b':{'part':tube,**end},'insertion_mm':depth,'locked':True,'torque_nm':40})

    def beam(pid,a,b,depth_a=30,depth_b=30):
        delta=matrices[b][:3,3]-matrices[a][:3,3];direction=delta/np.linalg.norm(delta)
        pa,port_a=socket(a,direction);pb,port_b=socket(b,-direction)
        start=point(matrices[a],port_a['position_mm'])-direction*depth_a
        end=point(matrices[b],port_b['position_mm'])+direction*depth_b
        part(pid,'tube-C',(start+end)/2,align_axis([0,0,1],direction),np.linalg.norm(end-start))
        attach(a,pa,pid,end='start',insertion_mm=depth_a);attach(b,pb,pid,end='end',insertion_mm=depth_b)

    def through(fitting,tube):
        center=point(np.linalg.inv(matrices[tube]),matrices[fitting][:3,3])
        attach(fitting,'through',tube,at_mm=round(float(center[2]+specs[tube]['parameters']['length_mm']/2),6))

    corners={name:(x,y) for name,x,y in [('front-left',-500,-550),('front-right',500,-550),('back-left',-500,550),('back-right',500,550)]}
    for name,(x,y) in corners.items():
        foot=part('foot-'+name,'TC132C',[x,y,0])
        directions=np.column_stack(([-np.sign(x),0,0],[0,-np.sign(y),0],[0,0,-1]))
        if np.linalg.det(directions)<0: directions=directions[:,[1,0,2]]
        top=part('top-'+name,'TC128C',[x,y,2300],directions)
        beam('post-'+name,foot,top,60,30)
        doc['anchors'].append({'part':foot,'surface':'floor','label':'Bolted flange; anchor capacity must be selected for the actual floor'})
    edges=[('front','front-left','front-right'),('back','back-left','back-right'),('left','front-left','back-left'),('right','front-right','back-right')]
    for side,a,b in edges: beam('top-bar-'+side,'top-'+a,'top-'+b)
    for level,height in [('lower',200),('middle',1250)]:
        for name,(x,y) in corners.items():
            inward_x=[-np.sign(x),0,0];inward_y=[0,-np.sign(y),0]
            basis=np.column_stack((inward_x,inward_y,np.cross(inward_x,inward_y)))
            fitting=part(level+'-'+name,'TC116C',[x,y,height],basis)
            through(fitting,'post-'+name)
        for side,a,b in edges:
            if level=='middle' and side=='front': continue  # Clear the legs' swing volume.
            beam(level+'-bar-'+side,level+'-'+a,level+'-'+b)
    for number,y in enumerate([-250,100,400],1):
        for side,x in [('left',-500),('right',500)]:
            branch=np.array([-np.sign(x),0,0]);run=np.array([0,1,0])
            fitting=part(f'rung-{number}-{side}','TC101C',[x,y,2300],np.column_stack((branch,np.cross(run,branch),run)))
            through(fitting,'top-bar-'+side)
        beam(f'rung-{number}',f'rung-{number}-left',f'rung-{number}-right',24,24)

    parameters={'stature_mm':1750,'mass_kg':75,'pose':'pull-up','grip_diameter_mm':42.4,
                'hold_joints':['upper_body'],'strength_scale':6,'joint_damping_nms_rad':1,'joint_angles_deg':{'right_hip':[6,0,0]}}
    human=humanoid(**parameters);body=transform({'rotation_deg':[0,0,180]})
    # Start with the total body COM below the bar, so releasing the legs excites
    # their passive joints rather than a large whole-body pendulum fall.
    neutral_com=np.average([p['pose']['position_mm'] for p in human['parts']],axis=0,weights=[p['body']['mass_kg'] for p in human['parts']])
    neutral_grip=np.mean([p['pose']['position_mm'] for p in human['parts'] if p['id'].endswith('_hand')],axis=0)
    arm=body[:3,:3]@(neutral_com-neutral_grip)
    body[:3,:3]=Rotation.from_rotvec([np.arctan2(-arm[1],-arm[2]),0,0]).as_matrix()@body[:3,:3]
    palms={side:body@transform(next(p for p in human['parts'] if p['id']==side+'_hand')['pose']) for side in ('left','right')}
    body[:3,3]=np.array([0,-550,2300])-np.mean([m[:3,3] for m in palms.values()],axis=0)
    doc['objects']=[{'id':'person','template':'human','parameters':parameters,'pose':pose_of(body)}]
    for side in ('left','right'):
        palm=palms[side][:3,3]+body[:3,3]
        station=point(np.linalg.inv(matrices['top-bar-front']),palm)[2]+specs['top-bar-front']['parameters']['length_mm']/2
        doc['joints'].append({'id':side+'-hand-grip','type':'revolute',
                             'a':{'part':'top-bar-front','at_mm':round(float(station),6)},
                             'b':{'part':'person/'+side+'_hand','port':'grip'},'damping':.02,
                             'metadata':{'human_grip':True,'assumption':'No-slip grasp with free rotation about the bar; no grip fatigue or release model'}})
    Assembly.from_doc(doc,ROOT/'examples',library)
    return doc


if __name__=='__main__':
    held=pull_up_design();write(ROOT/'examples'/'human-pull-up.pipe.yaml',held)
    relaxed=copy.deepcopy(held);relaxed['name']='Human pull-up · relaxed arms comparison'
    relaxed['description']='Same frame, grips and initial pose as human-pull-up. Posture motors are disabled; the person drops towards a passive hanging pose while both hands remain attached.'
    relaxed['objects'][0]['parameters']['hold_joints']=[]
    write(ROOT/'examples'/'human-pull-up-relaxed.pipe.yaml',relaxed)
    print('Wrote the held and relaxed pull-up examples.')
