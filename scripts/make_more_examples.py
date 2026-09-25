"""Additional material, load and reusable mechanism examples."""
from pathlib import Path
from pipesim.document import read,write

root=Path('examples')
def blank(name): return {'format':'pipesim/1','units':'mm-kg-s-N-deg','name':name,'parts':[],'joints':[],'anchors':[]}
doc=blank('Three materials · equal 500 mm cantilevers')
doc['environment']={'gravity_m_s2':[0,0,0],'ground':True}; doc['loads']=[]
for i,(pid,catalog) in enumerate([('steel','tubeclamp.tube-C'),('aluminium','minitec.20.1006'),('timber','porta.DOW-19')]):
    y=i*350
    doc['parts'].append({'id':pid,'catalog':catalog,'parameters':{'length_mm':500},'pose':{'position_mm':[250,y,800],'rotation_deg':[0,90,0]}})
    doc['anchors'].append({'part':pid,'surface':'wall','position_mm':[0,y,800]})
    doc['loads'].append({'part':pid,'at_mm':500,'force_n':[0,0,-50]})
doc['description']='Same force and length, different sections. This is not a comparison at equal mass. Timber and metal strength defaults are assumptions.'
write(root/'material-comparison.pipe.yaml',doc)

doc=read(root/'workbench.pipe.yaml'); doc['name']='Workbench · external contact load'; doc['loads']=[]
doc['parts'].append({'id':'payload','catalog':'generic.box','parameters':{'mass_kg':20},'pose':{'position_mm':[200,0,960]}})
doc['description']='The free 20 kg box drops 10 mm onto the tabletop. Record simulation contacts and use contact-loads to make the corresponding frame load case.'
write(root/'contact-load.pipe.yaml',doc)

doc=read(root/'seated-human.pipe.yaml'); doc['name']='Seated human · bounded posture control'
doc['objects'][0]['parameters']['hold_pose']=True
doc['objects'][0]['pose']['position_mm'][2]+=5
doc['description']='Finite joint torques hold a seated reference posture. This is an engineering posture controller, without active balance or muscle physiology.'
write(root/'seated-posture.pipe.yaml',doc)

caster={'parts':[
 {'id':'plate','pose':{'position_mm':[0,0,159]},'body':{'kind':'rigid','mass_kg':.4,'geometry':[{'type':'box','size_mm':[70,70,6]}],'color':'#8597a3'}},
 {'id':'fork','pose':{'position_mm':[0,0,100]},'body':{'kind':'rigid','mass_kg':.5,'geometry':[{'type':'box','size_mm':[15,8,100],'position_mm':[0,-23,0]},{'type':'box','size_mm':[15,8,100],'position_mm':[0,23,0]},{'type':'box','size_mm':[60,56,8],'position_mm':[0,0,50]}],'color':'#8597a3'}},
 {'id':'wheel','catalog':'generic.wheel','pose':{'position_mm':[0,0,50]}}],
 'joints':[
 {'id':'swivel','type':'revolute','a':{'part':'plate','frame':{'position_mm':[0,0,-3],'axis':[0,0,1]}},'b':{'part':'fork','frame':{'position_mm':[0,0,56],'axis':[0,0,1]}},'limits':{'angle_deg':[-180,180]},'damping':.05},
 {'id':'axle','type':'revolute','a':{'part':'fork','frame':{'position_mm':[0,0,-50],'axis':[0,1,0]}},'b':{'part':'wheel','frame':{'axis':[0,1,0]}},'damping':.005}],
 'metadata':{'status':'generic caster mechanism; replace dimensions and capacities with purchased hardware'}}
write(root/'libraries'/'mechanisms.yaml',{'format':'pipesim-library/1','name':'Reusable user-defined mechanisms','objects':{'demo.caster':caster}})
doc=blank('Four casters · reusable articulated objects'); doc['libraries']=['libraries/mechanisms.yaml']
doc['parts']=[{'id':'platform','catalog':'generic.panel','parameters':{'width_mm':800,'depth_mm':500,'thickness_mm':36},'pose':{'position_mm':[0,0,180]}}]
doc['objects']=[]
for i,(x,y) in enumerate([(-300,-180),(300,-180),(300,180),(-300,180)]):
    name=f'caster-{i+1}'; doc['objects'].append({'id':name,'template':'demo.caster','pose':{'position_mm':[x,y,0]}})
    doc['joints'].append({'id':f'mount-{i+1}','type':'fixed','a':{'part':'platform','frame':{'position_mm':[x,y,-18]}},'b':{'part':name+'/plate','frame':{'position_mm':[0,0,3]}},'metadata':{'hardware':'Four appropriately sized caster mounting bolts, nuts and washers'}})
doc['loads']=[{'part':'platform','force_n':[10,0,0],'start_s':.5,'end_s':1.5}]
doc['description']='Each caster contains a fork, wheel and plate joined by two revolute bearings. Apply a horizontal push to observe rolling. Gravity-polygon validation is conservative for articulated wheels.'
write(root/'caster-platform.pipe.yaml',doc)
