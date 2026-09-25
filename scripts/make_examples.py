from pathlib import Path
import copy
import yaml

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'examples'; OUT.mkdir(exist_ok=True)
def base(name): return {'format':'pipesim/1','units':'mm-kg-s-N-deg','name':name,'parts':[],'joints':[],'anchors':[],'environment':{'ground':True,'gravity_m_s2':[0,0,-9.81]}}
def put(doc,name): (OUT/name).write_text(yaml.safe_dump(doc,sort_keys=False,width=110),encoding='utf-8')
def part(id,catalog,pos=(0,0,0),rot=(0,0,0),**params): return {'id':id,'catalog':catalog,'pose':{'position_mm':list(pos),'rotation_deg':list(rot)},**({'parameters':params} if params else {})}
def socket(id,connector,port,member,end=None,at=None,insert=0,locked=True):
    return {'id':id,'type':'socket','a':{'part':connector,'port':port},'b':{'part':member,**({'end':end} if end else {'at_mm':at})},'insertion_mm':insert,'locked':locked,**({'torque_nm':40} if locked else {})}

bench=base('Workshop bench · C42 steel + timber')
bench['description']='A 1400 × 800 mm work surface on four flange-mounted steel legs. Floor anchors are explicit; bolt the top flanges to the timber.'
for i,(x,y) in enumerate([(-550,-280),(550,-280),(-550,280),(550,280)],1):
    bench['parts'] += [part(f'foot-{i}','tubeclamp.TC132C',[x,y,0]),part(f'leg-{i}','tubeclamp.tube-C',[x,y,(26+796.7)/2],length_mm=770.7),part(f'top-flange-{i}','tubeclamp.TC131C',[x,y,820],[180,0,0])]
    bench['anchors'].append({'part':f'foot-{i}','surface':'floor','label':'Two selected floor anchors through the TC132 bolt holes'})
    bench['joints'] += [socket(f'lower-{i}',f'foot-{i}','socket',f'leg-{i}','start',insert=60),socket(f'upper-{i}',f'top-flange-{i}','socket',f'leg-{i}','end',insert=30)]
    bench['joints'].append({'id':f'timber-{i}','type':'fixed','a':{'part':f'top-flange-{i}','port':'mount'},'b':{'part':'worktop','frame':{'position_mm':[x,y,-15],'axis':[0,0,-1]}},'assembly':'bolt','metadata':{'hardware':'Four appropriately selected timber fasteners through the TC131 mounting holes; verify pull-out for this panel'}})
bench['parts'].append(part('worktop','generic.panel',[0,0,835],width_mm=1400,depth_mm=800,thickness_mm=30))
bench['loads']=[{'part':'worktop','force_n':[0,0,-1000],'point_mm':[350,0,0]}]
bench['build']={'kerf_mm':3,'end_trim_mm':5,'stock_lengths_mm':[6000],'require_stability':True}
put(bench,'workbench.pipe.yaml')

beam=base('Analytical benchmark · cantilever')
beam['parts']=[part('beam','tubeclamp.tube-C',[500,0,1000],[0,90,0],length_mm=1000)]
beam['anchors']=[{'part':'beam','surface':'wall','position_mm':[0,0,1000]}]
beam['loads']=[{'part':'beam','at_mm':1000,'force_n':[0,0,-1000]}]
beam['environment']['gravity_m_s2']=[0,0,0]
put(beam,'cantilever.pipe.yaml')

slider=base('Loose collar · gravity and a physical stop')
slider['parts']=[part('guide','tubeclamp.tube-C',[0,0,1000],length_mm=2000),part('stop','tubeclamp.TC179C',[0,0,40]),part('slider','tubeclamp.TC101C',[0,0,1300])]
slider['anchors']=[{'part':'guide','surface':'fixture'}]
slider['joints']=[socket('stop-screw','stop','through','guide',at=40),socket('loose-screw','slider','through','guide',at=1300,locked=False)]
slider['joints'][1]['limits']={'slide_mm':[-600,1250],'angle_deg':[-180,180]}
slider['animation']={'duration_s':4,'fps':30,'tracks':[{'joint':'loose-screw','coordinate':'slide_mm','keyframes':[{'time_s':0,'value':0},{'time_s':2,'value':1000},{'time_s':4,'value':0}]}]}
put(slider,'sliding-collar.pipe.yaml')

motor=base('GT2 linear stage · coupled motor and carriage')
motor['parts']=[part('base','generic.box',[0,0,100],width_mm=600,depth_mm=160,height_mm=40,mass_kg=10),part('pulley','adafruit.GT2-20T-5',[-220,0,140]),part('carriage','generic.box',[-150,0,180],width_mm=80,depth_mm=80,height_mm=35,mass_kg=2)]
motor['anchors']=[{'part':'base','surface':'fixture'}]
motor['joints']=[{'id':'motor','type':'revolute','a':{'part':'base','frame':{'position_mm':[-220,0,40],'axis':[0,0,1]}},'b':{'part':'pulley'},'motor':{'mode':'position','target':0,'max_torque_nm':2,'kp':50,'kd':1,'schedule':[{'time_s':0,'target':0},{'time_s':3,'target':1800}]}},
                 {'id':'travel','type':'prismatic','a':{'part':'base','frame':{'position_mm':[-150,0,80],'axis':[1,0,0]}},'b':{'part':'carriage'},'limits':{'slide_mm':[0,300]}}]
motor['drives']=[{'id':'belt','type':'gt2','driver':'motor','follower':'travel','teeth':20,'pitch_mm':2,'belt_width_mm':6,'belt_length_mm':1164,'max_force_n':200,'route_mm':[[-220,-20,140],[220,-20,140],[220,20,140],[-220,20,140],[-220,-20,140]]}]
put(motor,'gt2-stage.pipe.yaml')

human=base('Human fit lab · 19 rigid segments')
human['objects']=[{'id':'person','template':'human','parameters':{'stature_mm':1750,'mass_kg':75,'pose':'standing'},'pose':{'position_mm':[0,0,0]}}]
put(human,'human.pipe.yaml')
seated=copy.deepcopy(human); seated['name']='Seated human · contact and reach'
seated['objects'][0]['parameters']['pose']='seated'
seated['objects'][0]['pose']['position_mm']=[0,0,-398.75]
seated['parts']=[part('seat','generic.panel',[0,-100,425],width_mm=650,depth_mm=550,thickness_mm=50)]
seated['anchors']=[{'part':'seat','surface':'fixture','label':'Bench supported by a fixed test fixture'}]
seated['tests']=[{'id':'seat-width','type':'seat','human':'person','seat':'seat','expect':True},{'id':'reach-far','type':'reach','human':'person','hand':'right','target_mm':[200,2500,1200],'expect':False}]
put(seated,'seated-human.pipe.yaml')

chain=base('Chain by length')
chain['objects']=[{'id':'chain','template':'chain','parameters':{'length_mm':1000,'link_catalog':'generic.chain-link'},'pose':{'position_mm':[0,0,1300],'rotation_deg':[0,30,0]}}]
chain['anchors']=[{'part':'chain/link-1','surface':'ceiling'}]
put(chain,'chain.pipe.yaml')
print('Created',len(list(OUT.glob('*.yaml'))),'example designs')
