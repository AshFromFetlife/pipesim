"""Three tee-ended legs on a square, ready to thread a common top pipe."""
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from pipesim.document import Assembly, Library, read, write
from pipesim.math3d import pose_of, align_axis, point


def build():
    doc=read(ROOT/'tests/fixtures/sliding-corner.pipe.yaml')
    frame={'bottom','right','top','left',*[f'corner-{i}' for i in range(4)]}
    doc['parts']=[p for p in doc['parts'] if p['id'] in frame]
    doc['joints']=[j for j in doc['joints'] if {j['a']['part'],j['b']['part']}<=frame]
    for part in doc['parts']: part['pose']['position_mm'][2]-=440
    doc['name']='Hinged triangle · thread the final pipe'
    doc['description']='A square supports three 1000 mm pipes, each with a tee at both ends. The different tee offsets require continuous angles rather than exact 15 degree increments. The top pipe is in the front socket: connect it through upper-back and then upper-left, keeping the screws loose until the frame is fitted.'
    doc['environment']={'ground':True}
    doc['anchors']=[{'part':'corner-0','surface':'fixture'}]
    library=Library.load(); base=Assembly.from_doc(doc,ROOT/'examples',library)
    def socket(jid,tee,port,pipe,endpoint,locked,insertion=0):
        doc['joints'].append({'id':jid,'type':'socket','a':{'part':tee,'port':port},'b':{'part':pipe,**endpoint},'insertion_mm':insertion,'locked':locked})
    for name,host,y,angle,upper in [('front','right',200,60,'TC148C'),('back','right',850,75,'TC148C'),('left','left',500,60,'TC101C')]:
        direction=np.array([(-1 if host=='right' else 1)*np.cos(np.deg2rad(angle)),0,np.sin(np.deg2rad(angle))])
        along=np.array([0.,1.,0.]); pivot=np.array([1065.6 if host=='right' else 0.,y,60.])
        lower_matrix=np.eye(4);lower_matrix[:3,:3]=np.column_stack((direction,np.cross(along,direction),along));lower_matrix[:3,3]=pivot
        doc['parts'].append({'id':'lower-'+name,'catalog':'tubeclamp.TC101C','pose':pose_of(lower_matrix)})
        station=(pivot-base.parts[host].matrix[:3,3])@base.parts[host].matrix[:3,2]+500
        socket('hinge-'+name,'lower-'+name,'through',host,{'at_mm':float(station)},False)
        start=pivot+direction*(59.5-24)
        pipe_matrix=np.eye(4);pipe_matrix[:3,:3]=align_axis([0,0,1],direction);pipe_matrix[:3,3]=start+direction*500
        doc['parts'].append({'id':'leg-'+name,'catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},'pose':pose_of(pipe_matrix)})
        socket('lower-seat-'+name,'lower-'+name,'branch','leg-'+name,{'end':'start'},True,24)
        upper_matrix=np.eye(4);upper_matrix[:3,:3]=np.column_stack((-direction,np.cross(along,-direction),along))
        upper_port=library.parts['tubeclamp.'+upper]['ports']['branch'];depth=min(24,upper_port['engagement_mm']*.8)
        upper_matrix[:3,3]=start+direction*(1000-depth)-upper_matrix[:3,:3]@upper_port['position_mm']
        doc['parts'].append({'id':'upper-'+name,'catalog':'tubeclamp.'+upper,'pose':pose_of(upper_matrix)})
        socket('upper-seat-'+name,'upper-'+name,'branch','leg-'+name,{'end':'end'},True,depth)
        if name=='front':
            bar=np.eye(4);bar[:3,:3]=align_axis([0,0,1],along);bar[:3,3]=upper_matrix[:3,3]+along*350
            doc['parts'].append({'id':'top-bar','catalog':'tubeclamp.tube-C','parameters':{'length_mm':1000},'pose':pose_of(bar)})
            socket('thread-front','upper-front','through','top-bar',{'at_mm':150},False)
    return doc


if __name__=='__main__':
    write(ROOT/'examples/hinged-triangle.pipe.yaml',build())
