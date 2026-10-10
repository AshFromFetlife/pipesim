// Rigid placement with articulated payloads: the independent oracle transports
// every member of one attachment component by the same transform.
import assert from 'node:assert/strict';
import {readFile,cp,mkdir,writeFile} from 'node:fs/promises';
import {join} from 'node:path';
import {randomBytes} from 'node:crypto';
import * as THREE from 'three';
import {matrix,endpoints,closePoint,random,v} from './geometry-workflow-model.mjs';

export function registerFreeAssemblyWorkflows({test,editor,post,workspace}) {
  const fixture='tests/fixtures/four-way-swing';
  function pose(m) {
    const q=new THREE.Quaternion().setFromRotationMatrix(m),e=new THREE.Euler().setFromQuaternion(q,'ZYX');
    return {position_mm:new THREE.Vector3().setFromMatrixPosition(m).toArray(),rotation_deg:[e.x,e.y,e.z].map(THREE.MathUtils.radToDeg)};
  }
  function component(scene,selected) {
    const members=new Set([selected]);let size;
    do {size=members.size;for(const joint of scene.joints)if(members.has(joint.a.part)||members.has(joint.b.part)){
      members.add(joint.a.part);members.add(joint.b.part);
    }}while(size!==members.size);
    return members;
  }
  async function exercise(spec,{fault=null,retain=true}={}) {
    await cp(join(fixture,'assets'),join(workspace(),'output','assets'),{recursive:true});
    let ui;const failures=[],requests=[],responses=[];
    try {
      ui=await editor(structuredClone(spec.document),null,{
        requestTimeoutMs:10000,
        onError:error=>failures.push(String(error)),
        onToast:(text,error)=>{if(error)failures.push(text);},
        onRequest:(path,body)=>requests.push({path,body}),
        onResponse:(path,status,data)=>{responses.push({path,status,data});if(status>=400||data?.status==='conflict')failures.push({path,status,data});},
        interceptFetch:path=>fault==='reject'&&path==='/api/snap-options'?Promise.resolve(new Response(JSON.stringify({error:'Injected rigid connection rejection'}),{status:400})):null,
      });
      await ui.drain();
      const original=structuredClone(ui.state.doc),scene=structuredClone(ui.state.scene);
      const selected=spec.side==='member'?'tube-c-4':'tc104c-1',members=component(scene,selected);
      assert.ok(members.size>=7,'the regression requires attached payloads');
      const fitting=scene.parts.find(p=>p.id==='tc104c-1'),pipe=scene.parts.find(p=>p.id==='tube-c-4');
      const socket=fitting.ports.branch,mouth=v(socket.position_mm).applyMatrix4(matrix(fitting.pose));
      const axis=v(socket.axis).transformDirection(matrix(fitting.pose));
      const end=endpoints(pipe)[spec.end==='start'?0:1],depth=Math.min(30,socket.engagement_mm*.8);
      const delta=mouth.clone().addScaledVector(axis,-depth).sub(v(end));
      if(spec.side==='connector')delta.negate();
      const focus=spec.side==='member'?mouth:v(end);
      ui.orbit.target.copy(focus);ui.camera.position.copy(focus).add(v([2200,-2700,1400]));ui.orbit.update();ui.tick();
      ui.select(selected);ui.document.querySelector('[data-tool="translate"]').click();ui.state.snap=false;
      ui.gizmo.axis='XYZ';ui.gizmo.dispatchEvent({type:'mouseDown'});
      ui.gizmo.object.position.add(delta);ui.gizmo.object.updateMatrix();
      const started=performance.now();ui.gizmo.dispatchEvent({type:'objectChange'});
      await ui.drain();
      const previewMs=performance.now()-started;
      assert.ok(previewMs<2500,'rigid drag preview exceeded the interactive time budget');
      for(const part of scene.parts) {
        const expected=v(part.pose.position_mm).add(members.has(part.id)?delta:v([0,0,0]));
        closePoint(ui.partObjects.get(part.id).position.toArray(),expected.toArray(),`drag ${part.id}`,.02);
      }
      ui.gizmo.dispatchEvent({type:'mouseUp'});
      await ui.drain();await ui.wait(()=>!ui.state.placementPending||ui.document.querySelector('#modal').open,10000);
      assert.deepEqual(failures,[],'free component connection was rejected');
      assert.equal(ui.document.querySelector('#modal').open,false,'a trivial rigid connection opened a review/Force dialog');
      assert.equal(ui.state.undo.length,1,'drag and connection must be one edit');
      assert.equal(ui.state.doc.joints.length,original.joints.length+1);
      const joint=ui.state.doc.joints.at(-1);
      assert.deepEqual(joint.a,{part:'tc104c-1',port:'branch'});assert.deepEqual(joint.b,{part:'tube-c-4',end:spec.end});
      if(fault==='leave-payload')ui.state.scene.parts.find(p=>p.id==='box-1').pose.position_mm[2]+=20;
      if(fault==='drop-joint')ui.state.doc.joints.splice(0,1);
      for(const part of scene.parts) {
        const actual=ui.state.scene.parts.find(p=>p.id===part.id),expected=v(part.pose.position_mm).add(members.has(part.id)?delta:v([0,0,0]));
        closePoint(actual.pose.position_mm,expected.toArray(),`committed ${part.id}`,.02);
        for(const direction of [[1,0,0],[0,1,0],[0,0,1]])closePoint(v(direction).transformDirection(matrix(actual.pose)).toArray(),v(direction).transformDirection(matrix(part.pose)).toArray(),`${part.id} orientation`,1e-4);
      }
      assert.deepEqual(ui.state.doc.joints.slice(0,-1),original.joints,'internal joints or limits were changed');
      for(let i=0;i<original.objects.length;i++) {
        const withoutPose=o=>Object.fromEntries(Object.entries(o).filter(([key])=>key!=='pose'));
        assert.deepEqual(withoutPose(ui.state.doc.objects[i]),withoutPose(original.objects[i]),'payload internals were rebaked');
      }
      const connected=structuredClone(ui.state.doc);
      ui.document.querySelector('#undo').click();await ui.drain();assert.deepEqual(ui.state.doc,original);
      ui.document.querySelector('#redo').click();await ui.drain();assert.deepEqual(ui.state.doc,connected);
      const options=await post('snap-options',original,{member:'tube-c-4',connector:'tc104c-1',port:'branch',end:spec.end,force:true});
      assert.ok(options.options.some(o=>o.available&&!o.requires_preview),'Force must also recognize the rigid solution');
      return {previewMs:Math.round(previewMs)};
    } catch(error) {
      if(retain) {
        const directory=`fuzz-runs/free-assembly/${spec.seed}-${Date.now()}`;await mkdir(directory,{recursive:true});
        await writeFile(join(directory,'replay.json'),JSON.stringify(spec,null,2));
        await writeFile(join(directory,'failure.json'),JSON.stringify({error:error.stack,failures,requests,responses,currentDocument:ui?.state.doc},null,2));
        throw new Error(`${error.message}\nRigid assembly seed=${spec.seed}; replay=${directory}/replay.json`,{cause:error});
      }
      throw error;
    } finally {await ui?.close();}
  }
  async function variant(seed) {
    const document=JSON.parse(await readFile(join(fixture,'design.json'),'utf8')),rng=random(seed);
    const end=seed%4>=2?'start':'end';
    if(end==='start') {
      const pipe=document.parts.find(p=>p.id==='tube-c-4');
      pipe.pose=pose(matrix(pipe.pose).multiply(new THREE.Matrix4().makeRotationX(Math.PI)));
      for(const joint of document.joints)for(const key of ['a','b'])if(joint[key].part==='tube-c-4') {
        if(joint[key].end)joint[key].end=joint[key].end==='start'?'end':'start';
        if(joint[key].at_mm!=null)joint[key].at_mm=pipe.parameters.length_mm-joint[key].at_mm;
      }
    }
    if(seed>=4) {
      const turn=new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler((rng()-.5)*.4,(rng()-.5)*.4,rng()*6,'ZYX'));
      turn.setPosition(v([rng()*1000-500,rng()*1000-500,5000]));
      for(const part of [...document.parts,...document.objects])part.pose=pose(turn.clone().multiply(matrix(part.pose)));
      // Vary the gap while preserving each source component's rigid geometry.
      const shift=v([rng()*100-50,rng()*100-50,rng()*100-50]);
      const source=['tube-c-4','tube-c-5','tc131c-1','tc131c-2','tc161c-1','tc132c-1','box-1'];
      for(const part of document.parts)if(source.includes(part.id))part.pose.position_mm=v(part.pose.position_mm).add(shift).toArray();
    }
    // Independent bits cover both dragged sides and pipe ends in each mode.
    for(const object of document.objects)if(object.template==='chain')object.layout_mode=Math.floor(seed/4)%2?'posable':'rigid';
    return {version:1,seed,side:seed%2?'connector':'member',end,document};
  }
  test('seeded free assembly workflows connect articulated payloads by rigid placement without dialogs',async()=>{
    if(process.env.PIPESIM_FREE_ASSEMBLY_REPLAY){await exercise(JSON.parse(await readFile(process.env.PIPESIM_FREE_ASSEMBLY_REPLAY,'utf8')));return;}
    const root=Number(process.env.PIPESIM_FREE_ASSEMBLY_SEED??randomBytes(4).readUInt32LE());
    const count=Number(process.env.PIPESIM_FREE_ASSEMBLY_CASES??8),single=process.env.PIPESIM_FREE_ASSEMBLY_CASE_SEED;
    for(const seed of [root,...(single===undefined?[]:[Number(single)])])assert.ok(Number.isInteger(seed)&&seed>=0&&seed<=0xffffffff,'seed must be a uint32');
    assert.ok(Number.isSafeInteger(count)&&count>=0);const rng=random(root);
    const seeds=single!==undefined?[Number(single)]:[0,1,2,3,4,5,6,7,1112331263,...Array.from({length:count},()=>Math.floor(rng()*0x100000000))];
    console.log(`Free assembly root_seed=${root} cases=${seeds.length}`);
    for(const seed of seeds){await exercise(await variant(seed));console.log(`Free assembly case_seed=${seed} passed`);}
    console.log(`Free assembly cases_passed=${seeds.length} rigid_connections=${seeds.length}`);
  });
  test('free assembly workflow oracles reject false failures and detached payloads',async()=>{
    for(const [fault,pattern] of [['reject',/rejected/],['leave-payload',/box-1/],['drop-joint',/internal joints/]])
      await assert.rejects(exercise(await variant(0),{fault,retain:false}),pattern);
  });
}
