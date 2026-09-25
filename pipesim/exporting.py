"""BOM, stock cutting, print-ready illustrated assembly instructions and evidence."""
from __future__ import annotations
import csv
import hashlib
import html
import json
from pathlib import Path
from .document import write, fingerprint, DocumentError
from .planning import plan_build

def bom(assembly):
    groups={}
    for p in assembly.parts.values():
        if p.kind=='human': continue
        spec=json.dumps({'catalog':p.spec.get('catalog',p.id),'section':p.section,'geometry':p.shapes if p.kind!='member' else None,'length_mm':p.length if p.kind=='member' else None},sort_keys=True)
        entry=groups.setdefault(spec,{'catalog':p.spec.get('catalog','custom'),'name':p.definition.get('name',p.id),'quantity':0,'length_mm':p.length if p.kind=='member' else None,'mass_kg':0,'parts':[],'supplier_url':p.definition.get('source',{}).get('url',''),'source_status':p.definition.get('source',{}).get('geometry_status','custom')})
        entry['quantity']+=1; entry['mass_kg']+=p.mass; entry['parts'].append(p.id)
        entry['section']=p.section; entry['material']=p.definition.get('material')
        if p.kind=='panel':
            from .geometry import mesh_for_part
            entry['panel_dimensions_mm']=mesh_for_part(p).extents.tolist()
    hardware=[{'connection':j['id'],'description':j['metadata']['hardware']} for j in assembly.joints if j.get('metadata',{}).get('hardware')]
    hardware += [{'connection':a['part'],'description':a.get('label','Select mounting fasteners for this world anchor')} for a in assembly.anchors]
    return {'items':list(groups.values()),'additional_hardware':hardware,'total_mass_kg':sum(g['mass_kg'] for g in groups.values()),'input_sha256':assembly.input_hash}

def cutting_plan(assembly,kerf_mm=None,stock_lengths_mm=None,end_trim_mm=None):
    build=assembly.doc.get('build',{})
    kerf=build.get('kerf_mm',3) if kerf_mm is None else kerf_mm
    trim=build.get('end_trim_mm',0) if end_trim_mm is None else end_trim_mm
    if kerf<0 or trim<0: raise ValueError('Kerf and end trim must be nonnegative')
    groups={}
    for p in assembly.parts.values():
        if p.kind!='member': continue
        key=json.dumps([p.spec.get('catalog',p.id),p.section,p.definition.get('material')],sort_keys=True)
        groups.setdefault(key,[]).append(p)
    bars=[]
    for members in groups.values():
        groupbars=[]
        stock=sorted(stock_lengths_mm or build.get('stock_lengths_mm') or members[0].definition.get('stock_lengths_mm',[6000]))
        if any(s<=2*trim for s in stock): raise ValueError('Stock must be longer than both end trims')
        def needed(remaining,length): return length if abs(remaining-length)<1e-7 else length+kerf
        for p in sorted(members,key=lambda p:(-p.length,p.id)):
            candidates=[b for b in groupbars if needed(b['remaining_mm'],p.length)<=b['remaining_mm']+1e-7]
            if candidates: bar=min(candidates,key=lambda b:b['remaining_mm'])
            else:
                sizes=[s for s in stock if needed(s-2*trim,p.length)<=s-2*trim+1e-7]
                if not sizes: raise DocumentError(f'{p.id}: {p.length:g} mm plus cut allowance exceeds the available stock lengths')
                bar={'bar':len(bars)+1,'catalog':p.spec.get('catalog','custom'),'section':p.section,'stock_length_mm':sizes[0],'end_trim_mm':trim,'remaining_mm':sizes[0]-2*trim,'cuts':[],'kerf_loss_mm':0}
                bars.append(bar); groupbars.append(bar)
            cost=needed(bar['remaining_mm'],p.length)
            offset=bar['stock_length_mm']-trim-bar['remaining_mm']
            cutkerf=cost-p.length
            bar['cuts'].append({'part':p.id,'length_mm':p.length,'start_mm':offset,'kerf_mm':cutkerf})
            bar['kerf_loss_mm']+=cutkerf; bar['remaining_mm']-=cost
    return {'method':'best-fit decreasing heuristic; not a proof of minimum stock use','kerf_mm':kerf,'bars':bars,'stock_count':len(bars),'stock_total_mm':sum(b['stock_length_mm'] for b in bars),'finished_total_mm':sum(c['length_mm'] for b in bars for c in b['cuts']),'kerf_total_mm':sum(b['kerf_loss_mm'] for b in bars),'trim_total_mm':sum(b['end_trim_mm']*2 for b in bars),'offcut_total_mm':sum(b['remaining_mm'] for b in bars),'input_sha256':assembly.input_hash}

def build_export(assembly,directory,engineering=False,illustrations=True):
    from .validation import validate
    checks=validate(assembly)
    if not checks['valid']: raise DocumentError('Build export requires a valid design: '+', '.join(i['code'] for i in checks['issues'] if i['severity']=='error'))
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=True)
    plan=assembly.doc.get('build_plan',{})
    if plan.get('input_sha256')!=assembly.input_hash or plan.get('status')!='buildable' or any('pose' not in s for s in plan.get('steps',[])): plan=plan_build(assembly)
    if plan['status']!='buildable':
        write(directory/'planning-report.json',plan)
        raise DocumentError(f"Build planning is {plan['status']}: {plan.get('reason')}. See planning-report.json.")
    parts=bom(assembly); cuts=cutting_plan(assembly)
    write(directory/'bill-of-materials.json',parts); write(directory/'cutting-plan.json',cuts); write(directory/'build-plan.json',plan)
    with (directory/'bill-of-materials.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        writer=csv.writer(stream); writer.writerow(['Part','Description','Quantity','Cut length (mm)','Panel dimensions (mm)','Section','Material','Mass (kg)','Instances','Supplier'])
        for r in parts['items']: writer.writerow([r['catalog'],r['name'],r['quantity'],r['length_mm'],' × '.join(f'{v:g}' for v in r.get('panel_dimensions_mm',[])),json.dumps(r['section']),r['material'],round(r['mass_kg'],3),', '.join(r['parts']),r['supplier_url']])
    with (directory/'cutting-plan.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        writer=csv.writer(stream); writer.writerow(['Stock bar','Catalog','Stock length (mm)','Part','Cut length (mm)','Start (mm)','Kerf (mm)'])
        for bar in cuts['bars']:
            for cut in bar['cuts']: writer.writerow([bar['bar'],bar['catalog'],bar['stock_length_mm'],cut['part'],cut['length_mm'],cut['start_mm'],cut['kerf_mm']])
    if illustrations:
        from .rendering import Renderer
        from .engineering_plot import layout_plot
        layout_plot(assembly,directory/'layout.png')
        with Renderer(assembly,width=1100,height=680,background='#ffffff',lighting='technical') as renderer:
            renderer.frame().convert('RGB').save(directory/'overview.png')
            for step in plan['steps']:
                renderer.frame(visible=step['installed_parts'],highlight=step['part'],arrow=step['approach_direction']).convert('RGB').save(directory/f"step-{step['number']:03}.png")
    esc=html.escape
    sections=[f'<section class="cover"><div class="eyebrow">PIPESIM / BUILD BOOK</div><h1>{esc(assembly.doc.get("name","Pipe creation"))}</h1><p>{len(assembly.parts)} parts · {len(plan["steps"])} steps · {parts["total_mass_kg"]:.1f} kg</p>'+('<img src="overview.png" alt="Completed structure">' if illustrations else '')+'<p class="note">All dimensions in millimetres. Confirm the listed hardware and inferred fitting dimensions against the purchased parts. The sequence uses declared world anchors.</p></section>']
    rows=''.join(f'<tr><td>{esc(r["catalog"])}</td><td>{esc(r["name"])}</td><td>{r["quantity"]}</td><td>{r["length_mm"] or " × ".join(f"{v:g}" for v in r.get("panel_dimensions_mm",[])) or "—"}</td></tr>' for r in parts['items'])
    sections.append('<section><div class="eyebrow">01 / GET READY</div><h2>Parts & materials</h2><table><thead><tr><th>Code</th><th>Part</th><th>Qty</th><th>Cut mm</th></tr></thead><tbody>'+rows+'</tbody></table><h3>Mounting hardware</h3><ul>'+''.join(f'<li><b>{esc(h["connection"])}</b> — {esc(h["description"])}</li>' for h in parts['additional_hardware'])+'</ul></section>')
    diagrams=[]
    for bar in cuts['bars']:
        pieces=''.join(f'<div style="width:{c["length_mm"]/bar["stock_length_mm"]*100:.3f}%" title="{esc(c["part"])}">{esc(c["part"])}<br>{c["length_mm"]:g}</div>' for c in bar['cuts'])
        diagrams.append(f'<h3>Bar {bar["bar"]} · {esc(bar["catalog"])} · {bar["stock_length_mm"]:g} mm</h3><div class="bar">{pieces}</div><p>{bar["remaining_mm"]:g} mm offcut · {bar["kerf_loss_mm"]:g} mm saw loss · {2*bar["end_trim_mm"]:g} mm end trim</p>')
    sections.append(f'<section><div class="eyebrow">02 / CUT ONCE</div><h2>Cutting plan</h2><p>{cuts["stock_count"]} stock lengths · {cuts["kerf_mm"]:g} mm blade kerf</p>'+''.join(diagrams)+'</section>')
    if illustrations:
        sections.append('<section><div class="eyebrow">03 / SET OUT THE BUILD</div><h2>Layout & mounting positions</h2><img src="layout.png" alt="Top, front and side projections with millimetre axes and mounting origins"><p class="note">Dots mark part origins for world-mounted parts. Coordinates use the shared design origin. Check actual mounting-hole patterns against purchased hardware.</p></section>')
    for step in plan['steps']:
        image=f'<img src="step-{step["number"]:03}.png" alt="Step {step["number"]}: highlighted new part and insertion arrow">' if illustrations else ''
        instructions=''.join('<li>'+esc(f['action'])+f' <span class="ref">({esc(f["joint"])})</span></li>' for f in step['fasten'])
        if step['world_anchor']: instructions+='<li>Fix this mounting plate to the declared '+esc(step['world_anchor'].get('surface','fixture'))+' using the selected anchors.</li>'
        if step['fixture']: instructions+='<li>Temporary support: '+esc(step['fixture']['description'])+'</li>'
        coordinates=' · '.join(f'{axis} {value:g}' for axis,value in zip('XYZ',step['pose']['position_mm']))
        angles=' / '.join(f'{value:g}°' for value in step['pose']['rotation_deg'])
        sections.append(f'<section><div class="step-number">{step["number"]:02}</div><div class="eyebrow">ASSEMBLY / {len(plan["steps"])} STEPS</div><h2>{esc(step["part"])}</h2>{image}<h3>{esc(step["instruction"])}</h3><p class="note">Part origin: {coordinates} mm. Orientation X/Y/Z: {angles}.</p><ul>{instructions}</ul><footer>PipeSim · {esc(assembly.doc.get("name",""))} <span>{step["number"]} / {len(plan["steps"])}</span></footer></section>')
    if engineering:
        from .fea import analyse
        result=analyse(assembly); write(directory/'engineering.json',result)
        if result['status']=='solved' and illustrations:
            from .engineering_plot import engineering_plot
            engineering_plot(assembly,result,directory/'engineering.png')
            sections.append('<section><div class="eyebrow">ENGINEERING / LOAD CASE</div><h2>Frame response</h2><img src="engineering.png" alt="Stress colours and magnified frame deflection"><p>Solid lines show the supplied geometry. Dashed lines show magnified computed displacement; the scale is printed on the diagram.</p></section>')
        rows=''.join(f'<tr><td>{esc(r["part"])}</td><td>{r["max_von_mises_mpa"]:.2f}</td><td>{r["max_displacement_mm"]:.3f}</td><td>{format(r["yield_utilisation"],".3g") if r["yield_utilisation"] is not None else "Unknown"}</td></tr>' for r in result.get('members',[]))
        sections.append('<section><div class="eyebrow">ENGINEERING / MODEL RESULTS</div><h2>Structural response</h2><p>Status: '+esc(result['status'])+'</p><table><tr><th>Member</th><th>Stress MPa</th><th>Deflection mm</th><th>Yield ratio</th></tr>'+rows+'</table><p>These calculations use the material assumptions in the design. Missing connector fracture and bending capacities remain unknown.</p><ul>'+''.join('<li>'+esc(s)+'</li>' for s in result.get('limitations',[]))+'</ul></section>')
    css='''*{box-sizing:border-box}body{margin:0;background:#dce2e5;color:#24313a;font:15px system-ui,sans-serif}section{position:relative;background:white;width:210mm;min-height:290mm;margin:20px auto;padding:18mm;break-after:page}h1{font-size:38px;line-height:1.12;max-width:600px}h2{font-size:30px;margin:12px 0}h3{font-size:17px}p,li{line-height:1.65}.eyebrow{font-size:11px;letter-spacing:2px;color:#667983;font-weight:700}img{width:100%;object-fit:contain}table{width:100%;border-collapse:collapse;font-size:12px}td,th{padding:11px 6px;border-bottom:1px solid #dce2e5;text-align:left}th{color:#607681}.step-number{position:absolute;right:18mm;top:10mm;font-size:62px;font-weight:300;color:#d7dfe3}.bar{height:55px;background:#e9edf0;display:flex;border:1px solid #a9b8c0}.bar>div{border-right:3px solid white;background:#b7d6d0;text-align:center;font-size:11px;padding-top:9px;overflow:hidden}.ref,.note{color:#6a7d86;font-size:12px}footer{position:absolute;bottom:12mm;left:18mm;right:18mm;font-size:11px;color:#6a7d86}footer span{float:right}@media print{body{background:white}section{margin:0;min-height:290mm;padding:15mm;box-shadow:none}}@page{size:A4;margin:0}'''
    (directory/'instructions.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>'+esc(assembly.doc.get('name','Assembly instructions'))+'</title><style>'+css+'</style>'+''.join(sections)+'</html>',encoding='utf-8')
    from .packaging import bundle
    bundle(assembly,directory)
    return {'directory':str(directory),'instructions':str(directory/'instructions.html'),'steps':len(plan['steps']),'stock_bars':cuts['stock_count'],'files':sorted(p.name for p in directory.iterdir())}
