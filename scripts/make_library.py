"""Reproducible dimension-derived starter library. Tables were read from supplier drawings.

Only dimensions in `drawing_dimensions_mm` are transcribed measurements. Every
inferred bore clearance, casting wall, socket depth and fillet omission is labelled.
The resulting YAML is the editable source used by the application.
"""
from pathlib import Path
import json
import math
import yaml
from fitting_meshes import standard_crossover,split_tee,swivel_short_tee

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'pipesim/data/libraries'
OUT.mkdir(parents=True,exist_ok=True)
OD={'T':21.3,'A':26.9,'B':33.7,'C':42.4,'D':48.3,'E':60.3}

def tube(od,length,pos=(0,0,0),axis=(0,0,1),bore=None):
    return {'type':'tube','diameter_mm':od,'wall_mm':round((od-(bore or od-6))/2,4),'length_mm':round(length,4),'position_mm':list(pos),'axis':list(axis)}
def box(size,pos=(0,0,0)):
    return {'type':'box','size_mm':list(size),'position_mm':list(pos)}
def port(od,pos,axis,through=False,depth=30,assembly='slide'):
    return {'type':'socket','position_mm':list(pos),'axis':list(axis),'diameter_mm':od,'profile':'round','through':through,'engagement_mm':round(depth,3),'min_engagement_mm':round(min(depth*.65,25),3),'assembly':assembly,'capacity':1,'clearance_mm':1.0,'geometry_status':'inferred-engagement'}

tables={
 '101':('f g h mass',{'T':[34,51,36,.14],'A':[40,60,43,.21],'B':[44,66,48,.35],'C':[56,89,59,.50],'D':[65,100,67,.65],'E':[76,122,80,1.0]}),
 '104':('f g mass',{'A':[82.8,60.5,.354],'B':[94.8,70.2,.54],'C':[120.5,88.1,.84],'D':[134.8,98.7,.95],'E':[170,120.9,1.518]}),
 # TC116E is withheld: the drawing's f=61.5 mm cannot accommodate its 60.3 mm bore
 # and two terminal sockets. TC116ZD has no dimension table on the product page.
 '116':('f mass',{'T':[50.3,.166],'A':[60.95,.258],'B':[74.5,.39],'C':[88.9,.634],'D':[96.7,.688]}),
 '125':('f g mass',{'T':[50.9,25,.152],'A':[66,28,.252],'B':[69.5,30,.346],'C':[89,38,.555],'D':[96.8,46,.652],'E':[120.4,53,1.174]}),
 '128':('f mass',{'T':[50.5,.21],'A':[60.2,.344],'B':[73.2,.518],'C':[88.5,.778],'D':[97.7,1.008],'E':[125.8,1.722]}),
 '131':('f g h i j mass',{'T':[32.4,74.1,4.4,39.3,8,.196],'A':[46,80.3,6.8,43.3,9,.304],'B':[48.4,90.1,6.5,45,9,.4],'C':[53.3,100.2,7.2,55.9,9,.494],'D':[58.8,114.8,8.3,64,9,.63],'E':[64.6,127.5,6.4,68,9,1.034]}),
 '132':('f g h i j k mass',{'A':[65,8,105,72,53,11,.4],'B':[75,8,125,84,60,15,.75],'C':[86,10,138,101,80,15,1.15],'D':[96,10,150,114,80,15,1.35],'E':[128,10,165,125,101,17,1.8]}),
 '136':('f g h mass',{'B':[81,78,47,.5],'C':[95,75,58,.65],'D':[105,83,61,.85]}),
 '138':('f g h i mass',{'A':[25.5,61.3,30.5,14.2,.164],'B':[25.5,69.3,33.3,14.2,.194],'C':[25.5,78.5,38.5,14.2,.21],'D':[25.5,83.2,41.1,14.2,.23]}),
 '140':('f g h i j mass',{'A':[64.1,25.6,62.3,30.5,13.4,.256],'B':[64.1,25.6,69.5,33.3,13.4,.266],'C':[64.1,25.6,79.4,38.5,13.4,.306],'D':[64.1,25.6,85,41.1,13.4,.316]}),
 '148':('f g h i mass',{'A':[20.3,73.8,30.1,24.7,.184],'B':[25.2,82.7,33.5,24.3,.288],'C':[28.5,101.3,49.9,27,.438],'D':[33.7,124.3,66.7,34.3,.636],'E':[36,145,105,40,.95]}),
 '161':('f g h i mass',{'A':[32.2,32.2,76.6,26.9,.222],'B':[39.5,39.5,87.4,33.7,.308],'C':[45.1,45.1,106.4,42.4,.49],'D':[55.8,55.8,111.4,48.3,.538],'E':[62.7,62.7,134.23,60.3,.864]}),
 '173m':('f g h i j mass',{'A':[73.4,8.1,32.8,8.2,32.8,.164],'B':[85.6,9.4,38.7,10.5,38.7,.258],'C':[95,8.9,38.6,10.6,45,.316],'D':[101.7,9.4,38.5,10.8,45,.332],'E':[120.6,8.3,49.3,10.6,49.7,.474]}),
 '173f':('f g h i j k mass',{'A':[34.3,68.8,32.3,8.4,6.3,10.5,.222],'B':[41.9,80.1,38.7,10.7,7.2,10.4,.344],'C':[41.6,87.2,37.9,10.7,7.2,10.6,.466],'D':[40.4,96.2,37.9,11,7.2,10.7,.5],'E':[50,110,38,9.3,6,10.5,.75]}),
 '179':('f mass',{'A':[22.8,.088],'B':[26.2,.116],'C':[26.2,.146],'D':[26.8,.152]})
}

parts={}
for code,(columns,rows) in tables.items():
    source=json.loads((ROOT/f'sources/tubeclamp/tc{code}/source.json').read_text())
    for size,values in rows.items():
        d=dict(zip(columns.split(),values)); od=OD[size]; bore=od+1; outer=od+9
        geometry=[]; ports={}
        if code in ('101','104'):
            width=d['f']
            outer=d['h'] if code=='101' else od+9
            total=d['g']
            reach=total-outer/2
            geometry=[tube(outer,width,bore=bore),tube(outer,reach-outer/2,(reach/2+outer/4,0,0),(1,0,0),bore)]
            ports={'through':port(od,[0,0,0],[0,0,1],True,width),
                   'branch':port(od,[reach,0,0],[1,0,0],depth=reach-outer/2)}
            if code=='104':
                ports={'through':ports['through'],'run_start':port(od,[0,0,-width/2],[0,0,-1],depth=width/2-1),'run_end':port(od,[0,0,width/2],[0,0,1],depth=width/2-1),'branch':ports['branch']}
                ports['through'].update(label='Main run · continuous pipe',excludes=['run_start','run_end'])
                for name in ('run_start','run_end'):
                    ports[name].update(label=name.replace('_',' ')+' · pipe end',excludes=['through'])
        elif code=='136':
            reach=d['f']-outer/2
            mesh,lug_y=split_tee(f'TC136{size}',od,d,reach)
            geometry=[mesh]
            for sign in (-1,1):
                geometry.append({'type':'cylinder','diameter_mm':9,'length_mm':18,'position_mm':[0,sign*lug_y,0],'axis':[1,0,0],'color':'#58636b'})
            geometry.append({'type':'cylinder','diameter_mm':12,'length_mm':4,'position_mm':[(reach+outer/2)/2,0,d['h']/2+2],'color':'#58636b'})
            ports={'through':port(od,[0,0,0],[0,0,1],True,d['h'],assembly='radial'),
                   'branch':port(od,[reach,0,0],[1,0,0],depth=reach-outer/2)}
        elif code=='148':
            geometry=[swivel_short_tee(f'TC148{size}',od,d)]
            ports={'through':port(od,[0,0,0],[0,0,1],True,d['f']),
                   'branch':port(od,[d['h']+d['i'],0,d['f']/2],[1,0,0],depth=d['i'])}
            ports['through']['label']='Short collar · continuous pipe'
            ports['branch']['label']='Offset branch · pipe end'
            for position in ([0,-outer/2-2,0],[d['h']+d['i']*.6,-outer/2-2,d['f']/2]):
                geometry.append({'type':'cylinder','diameter_mm':12,'length_mm':4,'position_mm':position,'axis':[0,1,0],'color':'#58636b'})
        elif code=='161':
            geometry=[standard_crossover(f'TC161{size}',od,d)]
            for position in ([0,-(d['h']-d['i'])/2-2,0],[0,d['i']+(d['h']-d['i'])/2+2,0]):
                geometry.append({'type':'cylinder','diameter_mm':12,'length_mm':4,'position_mm':position,'axis':[0,1,0],'color':'#58636b'})
            ports={'through':port(od,[0,0,0],[0,0,1],True,d['g']),
                   'cross':port(od,[0,d['i'],0],[1,0,0],True,d['f'])}
        elif code=='116':
            # Local Z is the continuous upright. The X/Y rails terminate before
            # its bore, so neither rail obstructs the pipe passing through it.
            reach=d['f']-outer/2
            geometry=[tube(outer,outer,bore=bore)]
            ports={'through':port(od,[0,0,0],[0,0,1],True,outer)}
            for axis,label in [([1,0,0],'x'),([0,1,0],'y')]:
                geometry.append(tube(outer,reach-outer/2,[a*d['f']/2 for a in axis],axis,bore))
                ports[label]=port(od,[a*reach for a in axis],axis,depth=reach-outer/2)
                geometry.append({'type':'cylinder','diameter_mm':12,'length_mm':4,'position_mm':[axis[0]*d['f']/2,axis[1]*d['f']/2,outer/2+2],'color':'#58636b'})
            offset=(outer/2+2)/math.sqrt(2)
            geometry.append({'type':'cylinder','diameter_mm':12,'length_mm':4,'position_mm':[-offset,-offset,0],'axis':[-1,-1,0],'color':'#58636b'})
        elif code in ('125','128'):
            if code=='125': outer=d['f']-d['g']
            reach=d['f']-outer/2
            axes=[[1,0,0],[0,0,1]] if code=='125' else [[1,0,0],[0,1,0],[0,0,1]]
            for axis,label in zip(axes,['x','z'] if code=='125' else ['x','y','z']):
                pos=[a*reach/2 for a in axis]
                geometry.append(tube(outer,reach,pos,axis,bore))
                ports[label]=port(od,[a*reach for a in axis],axis,depth=min(d.get('g',reach-outer/2),reach-outer/2))
        elif code in ('131','132'):
            height=d['f']; thickness=d['h'] if code=='131' else d['g']
            if code=='131':
                geometry=[tube(d['g'],thickness,[0,0,thickness/2],bore=bore)]
                for i,(x,y) in enumerate([(-1,-1),(-1,1),(1,1),(1,-1)]):
                    ports[f'bolt{i+1}']={'type':'bolt','position_mm':[x*d['i']/2,y*d['i']/2,0],'axis':[0,0,-1],'diameter_mm':d['j'],'assembly':'bolt','capacity':1}
            else:
                # Four strips preserve the central bore; bolt holes are annotated ports.
                w,l=d['h'],d['j']; hole=bore/2
                geometry=[box([w,(l-bore)/2,thickness],[0,s*(l+bore)/4,thickness/2]) for s in (-1,1)]
                geometry += [box([(w-bore)/2,bore,thickness],[s*(w+bore)/4,0,thickness/2]) for s in (-1,1)]
                for i,s in enumerate((-1,1)):
                    ports[f'bolt{i+1}']={'type':'bolt','position_mm':[s*d['i']/2,0,0],'axis':[0,0,-1],'diameter_mm':d['k'],'assembly':'bolt','capacity':1}
            geometry.append(tube(outer,height-thickness,[0,0,(height+thickness)/2],bore=bore))
            ports['socket']=port(od,[0,0,height],[0,0,1],depth=height-thickness)
            ports['mount']={'type':'mount','position_mm':[0,0,0],'axis':[0,0,-1],'assembly':'bolt','capacity':1}
        elif code=='179':
            geometry=[tube(outer,d['f'],bore=bore)]
            ports['through']=port(od,[0,0,0],[0,0,1],True,d['f'])
        elif code in ('138','140'):
            length=d['f'] if code=='138' else d['g']
            offset=d['h'] if code=='138' else d['i']
            geometry=[tube(outer,length,bore=bore)]
            if code=='138': geometry.append(tube(d['i']+10,length,[offset,0,0],bore=d['i']))
            else: geometry.append({'type':'cylinder','diameter_mm':d['j'],'length_mm':d['f'],'position_mm':[offset,0,(d['f']-length)/2]})
            ports={'through':port(od,[0,0,0],[0,0,1],True,length),'hinge':{'type':'eye' if code=='138' else 'pin','position_mm':[offset,0,0],'axis':[0,0,1],'diameter_mm':d['i'] if code=='138' else d['j'],'assembly':'hook','capacity':1}}
        elif code=='173m':
            length=d['j']; offset=d['f']-outer/2-d['h']/2
            geometry=[tube(outer,length,bore=bore),box([max(1,offset-outer/2),d['g'],d['h']],[(offset+outer/2)/2,0,0]),tube(d['h'],d['g'],[offset,0,0],[0,1,0],d['i'])]
            ports={'through':port(od,[0,0,0],[0,0,1],True,length),'hinge':{'type':'eye','position_mm':[offset,0,0],'axis':[0,1,0],'diameter_mm':d['i'],'assembly':'bolt','capacity':1}}
        elif code=='173f':
            height=d['g']-d['f']; offset=d['g']-d['h']/2
            geometry=[tube(outer,height,[0,0,height/2],bore=bore)]
            for sign in (-1,1):
                geometry.append(box([d['h'],d['j'],d['f']],[0,sign*(d['k']+d['j'])/2,height+d['f']/2]))
            ports={'socket':port(od,[0,0,0],[0,0,-1],depth=height),'hinge':{'type':'clevis','position_mm':[0,0,offset],'axis':[0,1,0],'diameter_mm':d['i'],'gap_mm':d['k'],'assembly':'bolt','capacity':1}}
        # Set-screw bosses are visible primitives, kept clear of the hollow socket.
        if code not in ('116','136','138','140','148','161','173f'):
            geometry.append({'type':'cylinder','diameter_mm':12,'length_mm':4,'position_mm':[0,outer/2+2,0 if code not in ('131','132') else d['f']*.65],'axis':[0,1,0],'color':'#58636b'})
        ident=f'tubeclamp.TC{code.upper()}{size}'
        parts[ident]={'name':source['title']+f' · {size} ({od} mm)','kind':'connector','material':'cast-iron','color':'#9aabb7','mass_kg':d['mass'],
                     'geometry':geometry,'ports':ports,'drawing_dimensions_mm':{k:v for k,v in d.items() if k!='mass'},
                     'source':{'supplier':'Tubeclamp AU','sku':f'TC{code.upper()}{size}','url':source['url'],'drawing_url':source.get('downloads',[{'url':source['images'][1]}])[0]['url'],'retrieved':source['retrieved'],'dimensions':'supplier drawing, nominal ±1 mm','geometry_status':'dimension-derived approximation','mass_basis':'drawing table, may differ from Shopify shipping weight','assumptions':['1 mm diametral bore clearance','Unspecified casting walls and socket engagement estimated','Fillets, screw threads and bolt holes simplified; verify physical sample before fabrication']},
                     'strength':{'axial_slip_n':5650,'required_torque_nm':40,'source_url':source['url'],'basis':'Supplier family-level axial statement at 40 Nm; applicability to this size is unverified','rating_status':'conditional','bending_nm':None,'torsion_nm':None,'fracture_n':None}}
        if code in ('136','148','161'):
            import hashlib
            parts[ident]['source']['model_sha256']=hashlib.sha256((OUT/geometry[0]['file']).read_bytes()).hexdigest()
        if code=='136':
            parts[ident]['source']['assumptions'].append('Split collar seam and bolt ears are approximations; g controls ear span, h controls axial width and branch envelope. Existing port positions retained; both bores accept the same nominal tube OD')
        if code=='161':
            parts[ident]['source']['assumptions'].append('f/g used as axial socket lengths, i as axis separation and h-i as casting OD. Both bores are cut through the combined casting; exact external blend and screw bosses approximated')
        if code=='148':
            parts[ident]['source']['assumptions'] += [
                'Single rigid casting, not a built-in hinge. Swivel action comes from two independent collars on a common pipe; supplier gives a 60-180 degree paired arrangement',
                'f is collar length; branch mouth uses h+i from the through axis, with socket engagement i. Branch centre is at the collar face f/2 to allow an inverted mate; its precise axial offset and tapered web are inferred from photos',
                'Casting OD is tube OD + 9 mm. The g envelope does not consistently agree with h+i and the bore radius in the supplier table (especially D/E); g is retained as published but does not set the model envelope']
            parts[ident]['pairing']={'collar_spacing_mm':d['f'],'relative_angle_range_deg':[60,180],
                'instructions':'Place two through collars f apart on one pipe; invert the second around local X, then rotate it about the pipe. Their branch socket axes meet the shared collar face. Tighten the pipe screws to lock the chosen angle.',
                'status':'Supplier arrangement; exact casting contact stops are approximated, not certified motion limits'}

# Steel tube product size is sourced; default wall/material grade are explicit user assumptions.
for size,od in OD.items():
    parts[f'tubeclamp.tube-{size}']={'name':f'Galvanised steel tube · {od} mm','kind':'member','material':'steel-assumed','color':'#778b99','parameters':{'length_mm':1000,'wall_mm':3.2 if size in 'CDE' else 2.6},'geometry':[{'type':'tube','diameter_mm':od,'wall_mm':'$wall_mm','length_mm':'$length_mm'}],'section':{'type':'tube','diameter_mm':od,'wall_mm':'$wall_mm'},'stock_lengths_mm':[6000], 'source':{'supplier':'Tubeclamp AU','url':'https://www.tubeclamp.com.au/pages/fitting-size-guide','dimensions':'outside diameter verified; wall thickness must be specified for purchased tube','geometry_status':'parametric','assumptions':['Steel grade and default wall thickness are assumptions, not supplier certification']}}

library={'format':'pipesim-library/1','name':'Tubeclamp AU — dimension-derived catalogue','description':'Offline, editable models reconstructed from published dimension drawings. Check source.assumptions on every part.','parts':parts}
(OUT/'tubeclamp.yaml').write_text(yaml.safe_dump(library,sort_keys=False,width=110),encoding='utf-8')

materials={
 'steel-assumed':{'density_kg_m3':7850,'youngs_modulus_pa':200e9,'poisson_ratio':.3,'yield_strength_pa':250e6,'status':'assumed-grade','notes':'Isotropic elastic model; replace with purchased tube grade and certificate.'},
 'cast-iron':{'density_kg_m3':7200,'youngs_modulus_pa':170e9,'poisson_ratio':.27,'status':'approximate-elastic; fracture strength unknown'},
 'aluminium-6063-assumed':{'density_kg_m3':2700,'youngs_modulus_pa':69e9,'poisson_ratio':.33,'yield_strength_pa':160e6,'status':'assumed-temper','notes':'6063 reported by MiniTec; temper and yield assumption need verification.'},
 'hardwood-assumed':{'density_kg_m3':650,'youngs_modulus_pa':11e9,'poisson_ratio':.3,'yield_strength_pa':40e6,'status':'assumed-longitudinal','notes':'Beam-only longitudinal approximation; grain, knots, moisture and joint splitting are unmodelled.'},
 'plywood-assumed':{'density_kg_m3':650,'youngs_modulus_pa':7e9,'poisson_ratio':.3,'status':'rigid-panel-only'},
 'polymer-assumed':{'density_kg_m3':1240,'youngs_modulus_pa':2e9,'poisson_ratio':.35,'status':'unrated; print anisotropy unmodelled'},
 'rubber':{'density_kg_m3':1100,'status':'rigid contact approximation'},
 'human-tissue':{'density_kg_m3':1000,'status':'rigid segment approximation'}
}
generic={
 'porta.DOW-19':{'name':'Porta Eucalyptus Grandis dowel · 19 mm','kind':'member','material':'hardwood-assumed','color':'#c9a06e','parameters':{'length_mm':1000},'geometry':[{'type':'cylinder','diameter_mm':19,'length_mm':'$length_mm'}],'section':{'type':'solid_round','diameter_mm':19},'stock_lengths_mm':[1200,1800,2400,3000],'source':{'supplier':'Porta','sku':'DOW-19','url':'https://www.porta.com.au/products/mouldings/dowels/dowel-19','retrieved':'2026-09-07','geometry_status':'supplier dimensions; timber stiffness and strength assumed','assumptions':['19 mm is not compatible with the 18 mm printable reference','Replace material data with species, moisture and grade measurements']}},
 'porta.DAR-3018':{'name':'Porta Eucalyptus Grandis DAR · 30 × 18 mm','kind':'member','material':'hardwood-assumed','color':'#c9a06e','parameters':{'length_mm':1000},'geometry':[{'type':'box','size_mm':[30,18,'$length_mm']}],'section':{'type':'rectangular','width_mm':30,'depth_mm':18},'stock_lengths_mm':[1200,1800,2400,2700,3000],'source':{'supplier':'Porta','sku':'DAR-3018','url':'https://www.porta.com.au/products/mouldings/dressed-boards/dar-3018','retrieved':'2026-09-07','geometry_status':'supplier dimensions; longitudinal beam material approximation'}},
 'minitec.21.1018':{'name':'MiniTec Power-Lock 45 · 21.1018/0','kind':'connector','material':'steel-assumed','mass_kg':.029,'color':'#92a6b1','geometry':[{'type':'box','size_mm':[18,12,5]},{'type':'cylinder','diameter_mm':8,'length_mm':25,'position_mm':[0,0,-15]}],'ports':{'end_screw':{'type':'mount','position_mm':[0,0,-2.5],'axis':[0,0,-1],'assembly':'bolt','thread':'M8'},'slot':{'type':'mount','position_mm':[0,0,2.5],'axis':[0,0,1],'assembly':'slide','profile':'MiniTec-45'}},'source':{'supplier':'MiniTec Framing','sku':'21.1018/0','url':'https://www.minitecframing.com/PDF/Catalog_Sections/MiniTec_Fasteners.pdf','assembly_url':'https://www.minitecframing.com/Products/Profile_Fasteners/Profile_Fasteners_PDF/MiniTec_Fastener_Technical_Data.pdf','retrieved':'2026-09-07','geometry_status':'approximate fastener envelope; not machining geometry','assumptions':['M8 × 25 screw and 0.029 kg are catalogue data; connecting-element envelope is estimated','Requires M8 tapping of profile end; slide second profile over gripping element before locking at 12 Nm','Use supplier assembly drawing for actual slot engagement and tool access']},'strength':{'rating_status':'supplier lists 6000 N static connection load; application and load direction must be verified','supplier_static_load_n':6000,'required_torque_nm':12}},
 'minitec.20.1006':{'name':'MiniTec 45 × 45 · 20.1006','kind':'member','material':'aluminium-6063-assumed','color':'#a7b7c2','parameters':{'length_mm':1000},'geometry':[{'type':'extrusion','size_mm':[45,45,'$length_mm']}],'section':{'type':'explicit','area_mm2':2192/2.7,'iy_mm4':159340,'ix_mm4':159340,'j_mm4':134160,'cy_mm':22.5,'cx_mm':22.5,'width_mm':45,'depth_mm':45},'mass_per_m_kg':2.192,'stock_lengths_mm':[6000],'source':{'supplier':'MiniTec Framing','sku':'20.1006','url':'https://www.minitecframing.com/Products/Aluminum_Profiles/Aluminum_Profile_Catalog_Pages/20.1006_Aluminum_Profile_45x45.html','torsion_source_url':'https://shop.minitec.co.uk/product/45-x-45-profile/','geometry_status':'simplified T-slot envelope','dimensions':'45 × 45 mm; Ix=Iy=15.934 cm4; 2.192 kg/m from US catalogue','assumptions':['Mass differs in UK catalogue (2.205 kg/m); US value used','Visual cross-section approximate; FEA uses published section properties']}},
 'generic.dowel-18':{'name':'Hardwood dowel · 18 mm','kind':'member','material':'hardwood-assumed','color':'#c9a06e','parameters':{'length_mm':1000},'geometry':[{'type':'cylinder','diameter_mm':18,'length_mm':'$length_mm'}],'section':{'type':'solid_round','diameter_mm':18},'stock_lengths_mm':[1200,2400],'source':{'geometry_status':'generic-parametric','status':'Select actual timber species and supplier before fabrication'}},
 'generic.dowel-elbow-18':{'name':'18 mm dowel elbow · printable concept','kind':'connector','material':'polymer-assumed','color':'#ef9b50','geometry':[tube(28,40,[20,0,0],[1,0,0],18.4),tube(28,40,[0,0,20],bore=18.4)],'ports':{'x':port(18,[40,0,0],[1,0,0],depth=24,assembly='glue'),'z':port(18,[0,0,40],[0,0,1],depth=24,assembly='glue')},'source':{'reference_url':'https://www.printables.com/model/1320655-connectors-for-18mm-tubedowel','geometry_status':'original illustrative design, not a copy of the linked model','status':'Linked Printables files could not be retrieved; import the original mesh with its licence to use it'},'strength':{'rating_status':'unknown'}},
 'generic.panel':{'name':'Timber panel / plank / sleeper','kind':'panel','material':'plywood-assumed','color':'#c29a66','parameters':{'width_mm':1200,'depth_mm':600,'thickness_mm':30},'geometry':[{'type':'box','size_mm':['$width_mm','$depth_mm','$thickness_mm']}],'source':{'geometry_status':'user-dimensioned'}},
 'generic.box':{'name':'Abstract load · box','kind':'load','color':'#de795c','parameters':{'width_mm':200,'depth_mm':200,'height_mm':200,'mass_kg':20},'geometry':[{'type':'box','size_mm':['$width_mm','$depth_mm','$height_mm']}],'mass_kg':'$mass_kg','source':{'geometry_status':'user-defined rigid load'}},
 'generic.wheel':{'name':'Wheel · configurable axle and flange','kind':'wheel','material':'rubber','color':'#333e48','parameters':{'diameter_mm':100,'width_mm':30,'mass_kg':1},'geometry':[{'type':'cylinder','diameter_mm':'$diameter_mm','length_mm':'$width_mm','axis':[0,1,0]}],'mass_kg':'$mass_kg','friction':.85,'ports':{'axle':{'type':'pin','position_mm':[0,0,0],'axis':[0,1,0],'assembly':'bolt'},'mount':{'type':'mount','position_mm':[0,0,65],'axis':[0,0,1],'assembly':'bolt'}},'source':{'geometry_status':'generic envelope; supply actual wheel mount dimensions'}},
 'generic.chain-link':{'name':'Chain link · configurable ring proxy','kind':'chain','material':'steel-assumed','color':'#87949e','parameters':{'diameter_mm':30,'wire_mm':5},'geometry':[{'type':'tube','diameter_mm':'$diameter_mm','wall_mm':'$wire_mm','length_mm':'$wire_mm','axis':[0,1,0]}],'ports':{'a':{'type':'eye','position_mm':[0,0,-10],'axis':[0,1,0],'assembly':'hook'},'b':{'type':'eye','position_mm':[0,0,10],'axis':[0,1,0],'assembly':'hook'}},'source':{'geometry_status':'generic ring contact proxy; no load rating'}},
 'generic.d-link':{'name':'D shackle · generic','kind':'connector','material':'steel-assumed','geometry':[tube(40,8,axis=[0,1,0],bore=24)],'ports':{'eye':{'type':'eye','position_mm':[0,0,0],'axis':[0,1,0],'assembly':'hook'}},'source':{'geometry_status':'generic ring proxy; no load rating'}},
 'adafruit.GT2-20T-5':{'name':'GT2 pulley · 20 teeth · 5 mm bore','kind':'rigid','material':'aluminium-6063-assumed','mass_kg':.006,'color':'#b8c6ce','geometry':[tube(16,16,bore=5)],'ports':{'shaft':{'type':'socket','diameter_mm':5,'profile':'round','position_mm':[0,0,0],'axis':[0,0,1],'through':True,'engagement_mm':16,'min_engagement_mm':10,'assembly':'slide'},'belt':{'type':'belt','pitch_mm':2,'teeth':20,'width_mm':6,'position_mm':[0,0,0],'axis':[0,0,1]}},'source':{'supplier':'Adafruit','sku':'1251','url':'https://www.adafruit.com/product/1251','status':'archived / no longer stocked','geometry_status':'dimension-derived envelope; tooth profile omitted'}},
 'adafruit.GT2-belt':{'name':'GT2 timing belt · 2 mm pitch · 6 mm wide','kind':'belt','mass_kg':.03,'color':'#293b42','parameters':{'length_mm':1164},'geometry':[{'type':'box','size_mm':['$length_mm',6,1.5]}],'source':{'supplier':'Adafruit','sku':'1184','url':'https://www.adafruit.com/product/1184','geometry_status':'straight stock representation; routed belt described by drive.route_mm','assumptions':['Mass and thickness estimated; tooth strength unknown']}}
}
(OUT/'general.yaml').write_text(yaml.safe_dump({'format':'pipesim-library/1','name':'Materials, framing, motion and user-defined primitives','materials':materials,'parts':generic},sort_keys=False,width=110),encoding='utf-8')
print(f'Generated {len(parts)+len(generic)} catalogue parts and {len(materials)} materials')
