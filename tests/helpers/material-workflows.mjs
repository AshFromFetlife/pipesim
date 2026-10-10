// The hosts and attachment points are the oracle, independently of resampling.
import assert from 'node:assert/strict';
import {readFile,cp,mkdir,writeFile} from 'node:fs/promises';
import {join} from 'node:path';
import {randomBytes} from 'node:crypto';
import * as THREE from 'three';
import {matrix,closePoint,random,v} from './geometry-workflow-model.mjs';

export function registerMaterialWorkflows({test,editor,workspace}) {
  const fixture='tests/fixtures/four-way-swing';
  const profiles=['generic.chain-link','generic.chain-heavy-100','generic.rope-jute-6','generic.strap-seatbelt-65','generic.rope-polypropylene-12'];
  function worldPoint(scene,endpoint) {
    const part=scene.parts.find(p=>p.id===endpoint.part);
    assert.ok(part,`missing attachment part ${endpoint.part}`);
    const local=endpoint.frame?.position_mm??part.ports[endpoint.port]?.position_mm??[0,0,0];
    return v(local).applyMatrix4(matrix(part.pose)).toArray();
  }
  function verify(before,ui,line,catalog) {
    const object=ui.state.doc.objects.find(o=>o.id===line);
    assert.equal(object.parameters.link_catalog,catalog,'material change was rejected');
    assert.equal(object.parameters.length_mm,1000,'requested length changed');
    assert.deepEqual(ui.state.doc.anchors,before.document.anchors);
    assert.deepEqual(ui.state.doc.joints.map(j=>j.id),before.document.joints.map(j=>j.id),'lost attachment');
    for(const old of before.scene.parts.filter(p=>!p.id.startsWith(line+'/'))) {
      const part=ui.state.scene.parts.find(p=>p.id===old.id);
      closePoint(part.pose.position_mm,old.pose.position_mm,`host ${old.id}`,.002);
      for(const axis of [[1,0,0],[0,1,0],[0,0,1]])closePoint(v(axis).transformDirection(matrix(part.pose)).toArray(),v(axis).transformDirection(matrix(old.pose)).toArray(),`host orientation ${old.id}`,1e-6);
    }
    const links=ui.state.scene.parts.filter(p=>p.id.startsWith(line+'/'));
    const pitch=({'generic.chain-link':20,'generic.chain-heavy-100':80,'generic.rope-jute-6':20,'generic.strap-seatbelt-65':25,'generic.rope-nylon-10':25,'generic.rope-polypropylene-12':25})[catalog];
    assert.equal(links.length,Math.ceil(1000/pitch),'wrong link count');
    for(const part of links) {
      assert.equal(part.catalog,catalog);
      const a=v(worldPoint(ui.state.scene,{part:part.id,port:'a'})),b=v(worldPoint(ui.state.scene,{part:part.id,port:'b'}));
      assert.ok(Math.abs(a.distanceTo(b)-pitch)<.001,`stretched ${part.id}`);
    }
    for(const joint of ui.state.scene.joints) {
      if(![joint.a,joint.b].some(e=>e.part.startsWith(line+'/')))continue;
      closePoint(worldPoint(ui.state.scene,joint.a),worldPoint(ui.state.scene,joint.b),`attachment ${joint.id}`,.05);
    }
    for(const old of before.scene.joints.filter(j=>[j.a,j.b].some(e=>e.part.startsWith(line+'/'))&&!j.id.startsWith(line+'/'))) {
      const joint=ui.state.scene.joints.find(j=>j.id===old.id);
      for(const side of ['a','b'])closePoint(worldPoint(ui.state.scene,joint[side]),worldPoint(before.scene,old[side]),`station ${old.id}`, .05);
    }
  }
  async function exercise(spec,{fault=null,retain=true}={}) {
    await cp(join(fixture,'assets'),join(workspace(),'output','assets'),{recursive:true});
    let ui;const failures=[],requests=[],responses=[];
    try {
      ui=await editor(structuredClone(spec.document),null,{
        requestTimeoutMs:15000,onError:e=>failures.push(String(e)),onToast:(text,error)=>{if(error)failures.push(text);},
        onRequest:(path,body)=>requests.push({path,body}),onResponse:(path,status,data)=>responses.push({path,status,data}),
        interceptFetch:path=>fault==='reject'&&path==='/api/object-parameters'?Promise.resolve(new Response(JSON.stringify({error:'Injected material rejection'}),{status:400})):null,
      });
      await ui.drain();const before={document:structuredClone(ui.state.doc),scene:structuredClone(ui.state.scene)};
      for(const [index,catalog] of spec.profiles.entries()) {
        const group=ui.document.querySelector(`[data-object-tree="${spec.line}"]`);
        group.open=true;group.dispatchEvent(new ui.window.Event('toggle'));await ui.drain();
        ui.select(spec.line+'/link-1');
        const input=ui.document.querySelector('#chain-profile');input.value=catalog;input.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
        await ui.drain();
        assert.deepEqual(failures,[],'material edit emitted an error');
        if(fault==='move-host')ui.state.scene.parts.find(p=>p.id==='human-1/pelvis').pose.position_mm[0]+=10;
        if(fault==='drop-joint')ui.state.doc.joints.pop();
        if(fault==='stretch')ui.state.scene.parts.find(p=>p.id===spec.line+'/link-1').ports.a.position_mm[2]-=3;
        verify(before,ui,spec.line,catalog);assert.equal(ui.state.undo.length,index+1);
        const accepted=structuredClone(ui.state.doc);
        ui.document.querySelector('#undo').click();await ui.drain();
        if(index===0)assert.deepEqual(ui.state.doc,before.document);
        ui.document.querySelector('#redo').click();await ui.drain();assert.deepEqual(ui.state.doc,accepted);
        verify(before,ui,spec.line,catalog);
      }
    } catch(error) {
      if(retain) {
        const directory=`fuzz-runs/material/${spec.seed}-${Date.now()}`;await mkdir(directory,{recursive:true});
        await writeFile(join(directory,'replay.json'),JSON.stringify(spec,null,2));
        await writeFile(join(directory,'failure.json'),JSON.stringify({error:error.stack,failures,requests,responses,document:ui?.state.doc},null,2));
        throw new Error(`${error.message}\nMaterial seed=${spec.seed}; replay=${directory}/replay.json`,{cause:error});
      }
      throw error;
    } finally {await ui?.close();}
  }
  async function variant(seed) {
    const document=JSON.parse(await readFile(join(fixture,'design.json'),'utf8')),rng=random(seed);
    const line=`chain-${seed%4+1}`;
    document.objects.find(o=>o.id===line).layout_mode=Math.floor(seed/4)%2?'posable':'rigid';
    if(seed>=8) {
      const delta=new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(rng()*2,rng()*2,rng()*6,'ZYX')).setPosition(v([rng()*2000,rng()*2000,5000]));
      for(const part of [...document.parts,...document.objects]) {
        const m=delta.clone().multiply(matrix(part.pose)),e=new THREE.Euler().setFromRotationMatrix(m,'ZYX');
        part.pose={position_mm:new THREE.Vector3().setFromMatrixPosition(m).toArray(),rotation_deg:[e.x,e.y,e.z].map(THREE.MathUtils.radToDeg)};
      }
    }
    return {version:1,seed,line,profiles:['generic.chain-link',profiles[1+Math.floor(seed/8)%4],'generic.rope-nylon-10'],document};
  }
  test('seeded material workflows preserve slack line attachments through profile changes',async()=>{
    if(process.env.PIPESIM_MATERIAL_REPLAY){await exercise(JSON.parse(await readFile(process.env.PIPESIM_MATERIAL_REPLAY,'utf8')));return;}
    const root=Number(process.env.PIPESIM_MATERIAL_SEED??randomBytes(4).readUInt32LE()),single=process.env.PIPESIM_MATERIAL_CASE_SEED;
    const count=Number(process.env.PIPESIM_MATERIAL_CASES??8);
    assert.ok(Number.isSafeInteger(count)&&count>=0);
    for(const seed of [root,...(single===undefined?[]:[Number(single)])])assert.ok(Number.isInteger(seed)&&seed>=0&&seed<=0xffffffff);
    const rng=random(root),seeds=single===undefined?[0,1,2,3,4,5,6,7,8,16,24,...Array.from({length:count},()=>Math.floor(rng()*0x100000000))]:[Number(single)];
    console.log(`Material root_seed=${root} cases=${seeds.length}`);
    for(const seed of seeds){await exercise(await variant(seed));console.log(`Material case_seed=${seed} passed`);}
    console.log(`Material cases_passed=${seeds.length} profile_changes=${seeds.length*3}`);
  });
  test('material workflow oracles reject false errors, moved hosts, lost joints and stretched links',async()=>{
    for(const [fault,pattern] of [['reject',/emitted an error/],['move-host',/host human-1\/pelvis/],['drop-joint',/lost attachment/],['stretch',/stretched|attachment/]])
      await assert.rejects(exercise(await variant(0),{fault,retain:false}),pattern);
  });
}
