// Replay the second socket of a duplicated reinforcement arm with its top
// collar already secured to a shared rail. Use the editor's connection handler
// and real API; a rejected feasible operation is always a failing test.
import assert from 'node:assert/strict';
import {readFile,cp,mkdir,writeFile} from 'node:fs/promises';
import {join} from 'node:path';
import {randomBytes} from 'node:crypto';
import * as THREE from 'three';
import {matrix,random,v} from './geometry-workflow-model.mjs';

export function registerReinforcementWorkflows({test,editor,post,workspace}) {
  const fixture='tests/fixtures/four-way-swing';
  const match={member:'tube-c-4-copy',connector:'tc104c-1-copy',port:'branch',end:'end',insertion_mm:29.36,distance:0};
  function pose(m) {
    const e=new THREE.Euler().setFromRotationMatrix(m,'ZYX');
    return {position_mm:new THREE.Vector3().setFromMatrixPosition(m).toArray(),rotation_deg:[e.x,e.y,e.z].map(THREE.MathUtils.radToDeg)};
  }
  async function variant(seed) {
    const document=JSON.parse(await readFile(join(fixture,'reinforced-arm.json'),'utf8')),rng=random(seed);
    const end=seed%2?'start':'end';
    if(seed>=4) {
      // A user can secure a duplicate at different stations along the rail.
      // Vary that spacing and the loose tee's starting station independently,
      // preserving the already-connected geometry by construction.
      for(const [connector,rail,members,travel] of [
        ['tc161c-1-copy','tube-c-5',['tc161c-1-copy','tube-c-4-copy','tc132c-1-copy'],rng()*130-100],
        ['tc104c-1-copy','tube-c-2-mirror-mirror-3-mirror-mirror-2',['tc104c-1-copy'],rng()*240-120],
      ]) {
        const host=document.parts.find(p=>p.id===rail),shift=v([0,0,1]).transformDirection(matrix(host.pose)).multiplyScalar(travel);
        for(const part of document.parts)if(members.includes(part.id))part.pose.position_mm=v(part.pose.position_mm).add(shift).toArray();
        const joint=document.joints.find(j=>j.a.part===connector&&j.b.part===rail);joint.b.at_mm+=travel;
      }
    }
    if(end==='start') {
      const pipe=document.parts.find(p=>p.id===match.member);
      pipe.pose=pose(matrix(pipe.pose).multiply(new THREE.Matrix4().makeRotationX(Math.PI)));
      for(const j of document.joints)for(const key of ['a','b'])if(j[key].part===pipe.id) {
        if(j[key].end)j[key].end=j[key].end==='start'?'end':'start';
        if(j[key].at_mm!=null)j[key].at_mm=pipe.parameters.length_mm-j[key].at_mm;
      }
    }
    if(seed>=2) {
      const turn=new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler((rng()-.5)*2,(rng()-.5)*2,rng()*6,'ZYX'));
      turn.setPosition(v([rng()*4000-2000,rng()*4000-2000,5000]));
      for(const part of [...document.parts,...document.objects])part.pose=pose(turn.clone().multiply(matrix(part.pose)));
    }
    for(const object of document.objects)if(object.template==='chain')object.layout_mode=seed%4>=2?'rigid':'posable';
    return {version:1,seed,document,match:{...match,end}};
  }
  async function exercise(spec,{fault=null,retain=true}={}) {
    await cp(join(fixture,'assets'),join(workspace(),'output','assets'),{recursive:true});
    let ui;const failures=[],requests=[];
    try {
      ui=await editor(structuredClone(spec.document),null,{
        onError:e=>failures.push(String(e)),onToast:(text,error)=>{if(error)failures.push(text);},
        onRequest:(path,body)=>requests.push({path,body}),
        onResponse:(path,status,data)=>{if(status>=400)failures.push({path,status,data});},
        interceptFetch:path=>fault==='reject'&&path==='/api/snap-options'?Promise.resolve(new Response(JSON.stringify({error:'Injected connection rejection'}),{status:400})):null,
      });
      await ui.drain();const original=structuredClone(ui.state.doc),scene=structuredClone(ui.state.scene);
      const fitting=ui.state.scene.parts.find(p=>p.id===spec.match.connector),member=ui.state.scene.parts.find(p=>p.id===spec.match.member);
      const axis=v(fitting.ports.branch.axis).transformDirection(matrix(fitting.pose));
      const endAxis=v([0,0,spec.match.end==='end'?-1:1]).transformDirection(matrix(member.pose));
      const angle=THREE.MathUtils.radToDeg(Math.acos(THREE.MathUtils.clamp(axis.dot(endAxis),-1,1)));
      let poses={};
      if(spec.seed%3) {
        // Renderer alignment is intent, not permission to break the loose
        // collar's existing rail connection. Cover both direct connection and
        // the drag/drop path which supplies this proposed pose to the fitter.
        const rotation=new THREE.Matrix4().makeRotationFromQuaternion(new THREE.Quaternion().setFromUnitVectors(axis,endAxis));
        const mouth=v(fitting.ports.branch.position_mm).applyMatrix4(matrix(fitting.pose));
        const end=v([0,0,(spec.match.end==='end'?1:-1)*member.length_mm/2]).applyMatrix4(matrix(member.pose));
        rotation.setPosition(end.addScaledVector(endAxis,spec.match.insertion_mm).sub(mouth.applyMatrix4(rotation)));
        poses={[spec.match.connector]:pose(rotation.multiply(matrix(fitting.pose)))};
      }
      const start=performance.now();await ui.offerConnection([{...spec.match,angle,score:0}],poses);await ui.drain();
      assert.deepEqual(failures,[],'reinforcement connection was rejected');
      assert.equal(ui.document.querySelector('#modal').open,false,'reinforcement unexpectedly needs review or Force');
      assert.ok(performance.now()-start<10000,'reinforcement took more than ten seconds');
      assert.equal(ui.state.doc.joints.length,original.joints.length+1,'reinforcement did not connect');
      assert.equal(ui.state.undo.length,1);
      const added=ui.state.doc.joints.at(-1);
      assert.deepEqual(added.a,{part:spec.match.connector,port:'branch'});
      assert.deepEqual(added.b,{part:spec.match.member,end:spec.match.end});
      if(fault==='merge-arms')ui.state.scene.parts.find(p=>p.id===match.member).pose=structuredClone(ui.state.scene.parts.find(p=>p.id==='tube-c-4').pose);
      const oldParts=new Map(scene.parts.map(p=>[p.id,p])),parts=new Map(ui.state.scene.parts.map(p=>[p.id,p]));
      assert.ok(v(parts.get('tube-c-4').pose.position_mm).distanceTo(v(parts.get(match.member).pose.position_mm))>200,'reinforcement merged onto original arm');
      for(const a of original.anchors)assert.deepEqual(parts.get(a.part).pose,oldParts.get(a.part).pose,'world fixing moved');
      if(fault==='drop-joint')ui.state.doc.joints.splice(0,1);
      assert.deepEqual(ui.state.doc.joints.slice(0,-1).map(j=>[j.id,j.a.part,j.b.part,j.locked]),original.joints.map(j=>[j.id,j.a.part,j.b.part,j.locked]),'existing attachments changed');
      for(const j of scene.joints)if(j.locked||j.type==='fixed') {
        const old=matrix(oldParts.get(j.a.part).pose).invert().multiply(matrix(oldParts.get(j.b.part).pose));
        const actual=matrix(parts.get(j.a.part).pose).invert().multiply(matrix(parts.get(j.b.part).pose));
        assert.ok(old.elements.every((n,i)=>Math.abs(n-actual.elements[i])<.002),`${j.id} lost its rigid relationship`);
      }
      const base='tube-c-2-mirror-mirror-3-mirror-mirror-2';
      const inverseOld=matrix(oldParts.get(base).pose).invert(),inverseNew=matrix(parts.get(base).pose).invert();
      if(fault==='leave-payload')parts.get('human-1/head').pose.position_mm[0]+=20;
      for(const p of scene.parts) {
        assert.equal(parts.get(p.id).length_mm,p.length_mm,'cut length changed');
        if(!/^(human-|chain-)/.test(p.id))continue;
        const before=inverseOld.clone().multiply(matrix(p.pose)),after=inverseNew.clone().multiply(matrix(parts.get(p.id).pose));
        assert.ok(before.elements.every((n,i)=>Math.abs(n-after.elements[i])<.002),`${p.id} did not travel rigidly with its support`);
      }
      const socket=parts.get(spec.match.connector),pipe=parts.get(spec.match.member),bore=socket.ports.branch;
      const boreAxis=v(bore.axis).transformDirection(matrix(socket.pose));
      const expected=v(bore.position_mm).applyMatrix4(matrix(socket.pose)).addScaledVector(boreAxis,-added.insertion_mm);
      const end=v([0,0,(spec.match.end==='end'?1:-1)*pipe.length_mm/2]).applyMatrix4(matrix(pipe.pose));
      assert.ok(end.distanceTo(expected)<added.fit_tolerance_mm,'new socket has an excessive gap');
      assert.ok(boreAxis.dot(v([0,0,spec.match.end==='end'?-1:1]).transformDirection(matrix(pipe.pose)))>=.999,'new socket axes miss');
      const committed=structuredClone(ui.state.doc);
      ui.document.querySelector('#undo').click();await ui.drain();assert.deepEqual(ui.state.doc,original);
      ui.document.querySelector('#redo').click();await ui.drain();assert.deepEqual(ui.state.doc,committed);
    } catch(error) {
      if(retain) {
        const directory=`fuzz-runs/reinforcement/${spec.seed}-${Date.now()}`;await mkdir(directory,{recursive:true});
        await writeFile(join(directory,'replay.json'),JSON.stringify(spec,null,2));
        await writeFile(join(directory,'failure.json'),JSON.stringify({error:error.stack,failures,requests,document:ui?.state.doc},null,2));
        throw new Error(`${error.message}\nReinforcement seed=${spec.seed}; replay=${directory}/replay.json`,{cause:error});
      }
      throw error;
    } finally {await ui?.close();}
  }
  test('seeded reinforcement workflows close duplicated arms on shared rails',async()=>{
    if(process.env.PIPESIM_REINFORCEMENT_REPLAY){await exercise(JSON.parse(await readFile(process.env.PIPESIM_REINFORCEMENT_REPLAY,'utf8')));return;}
    const root=Number(process.env.PIPESIM_REINFORCEMENT_SEED??randomBytes(4).readUInt32LE()),single=process.env.PIPESIM_REINFORCEMENT_CASE_SEED;
    const count=Number(process.env.PIPESIM_REINFORCEMENT_CASES??8),rng=random(root);
    assert.ok(Number.isSafeInteger(count)&&count>=0);
    for(const seed of [root,...(single===undefined?[]:[Number(single)])])assert.ok(Number.isInteger(seed)&&seed>=0&&seed<=0xffffffff);
    const seeds=single===undefined?[0,1,2,3,...Array.from({length:count},()=>Math.floor(rng()*0x100000000))]:[Number(single)];
    console.log(`Reinforcement root_seed=${root} cases=${seeds.length}`);
    for(const seed of seeds){await exercise(await variant(seed));console.log(`Reinforcement case_seed=${seed} passed`);}
    console.log(`Reinforcement cases_passed=${seeds.length} connections=${seeds.length}`);
  });
  test('reinforcement workflow oracles reject errors, merged arms and detached payloads',async()=>{
    for(const [fault,pattern] of [['reject',/was rejected/],['merge-arms',/merged onto/],['drop-joint',/attachments changed/],['leave-payload',/did not travel rigidly/]])
      await assert.rejects(exercise(await variant(0),{fault,retain:false}),pattern);
  });
}
