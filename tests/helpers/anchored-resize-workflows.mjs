// Occupied pipe ends + a through crossbar + remote world anchors must coexist
// in every case. Random isolated pipes cannot exercise this failure.
import assert from 'node:assert/strict';
import {readFile, mkdir, writeFile} from 'node:fs/promises';
import {randomBytes} from 'node:crypto';
import * as THREE from 'three';
import {random, matrix, endpoints, closePoint, connections, edgeKey, runs, v} from './geometry-workflow-model.mjs';
import {DEFAULT_PREFERENCES} from '../../pipesim/web/preferences.js';

export function registerAnchoredResizeWorkflows({test, editor, post}) {
  const blank={format:'pipesim/1',units:'mm-kg-s-N-deg',parts:[],joints:[],anchors:[]};
  async function fixture() {
    return (await post('parse',blank,{text:await readFile('tests/fixtures/resize-anchored-crossbar.pipe.yaml','utf8')})).document;
  }
  async function exercise(doc,{seed=0,mode='draft',reverse=false,step=10,fault=null,retain=true,prepared=false}={}) {
    if(reverse&&!prepared) for(const run of runs(doc)) {
      [run.start_mm,run.end_mm]=[run.end_mm,run.start_mm];
      for(const a of run.attachments||[])if(a.end)a.end=a.end==='start'?'end':'start';
    }
    // The same connections must work before and after finalization, including
    // a draft selected pipe whose anchored neighbors are already exact.
    if(mode!=='draft'&&!prepared) {
      const finished=await post('draft-finalize',doc);assert.equal(finished.status,'finalized');doc=finished.document;
      if(mode==='mixed')doc=(await post('draft-reopen',doc,{members:['tube-c-4']})).document;
    }
    const initial=structuredClone(doc),failures=[],actions=[];
    let ui;
    try {
      ui=await editor(doc,null,{
        storedPreferences:JSON.stringify({...DEFAULT_PREFERENCES,resizeKeyStepMm:step}),
        onError:error=>failures.push(String(error)),
        onToast:(text,error)=>{if(error)failures.push(text);},
        onResponse:(path,status,data)=>{if(status>=400||data?.status==='conflict')failures.push({path,status,data});},
        interceptFetch:path=>fault==='reject'&&path==='/api/resize-drag'?
          Promise.resolve(new Response(JSON.stringify({error:'Injected anchored resize rejection'}),{status:400})):null
      });
      const settled=async()=>{await ui.drain();await ui.wait(()=>!ui.state.placementPending&&!ui.state.busy);await ui.drain();assert.deepEqual(failures,[],'feasible anchored endpoint edit was rejected');};
      ui.select('tube-c-4');
      assert.equal(ui.document.querySelector('#resize-label-second').textContent,'U/J');
      const edges=connections(doc).map(edgeKey).sort();
      const clean=doc=>{const d=structuredClone(doc);delete d.results;return d;};
      // Always exercise the reported bottom pair first, then the opposite end.
      for(const [key,side,sign] of [['u','end',1],['j','end',-1],['h','start',-1],['y','start',1]]) {
        const before=structuredClone(ui.state.doc),scene=structuredClone(ui.state.scene),undo=ui.state.undo.length;
        const span=endpoints(scene.parts.find(p=>p.id==='tube-c-4'));
        const moving=side==='start'?0:1,fixed=1-moving;
        const delta=v(span[moving]).sub(v(span[fixed])).normalize().multiplyScalar(sign*step);
        const follower=(side==='start')!==reverse?'tc132c-1':'tc104c-1';
        actions.push({key,side,step});
        ui.document.dispatchEvent(new ui.window.KeyboardEvent('keydown',{key,bubbles:true,cancelable:true}));
        await settled();
        assert.equal(ui.state.undo.length,undo+1,'shortcut must commit exactly one edit');
        if(fault==='move-anchor')ui.state.scene.parts.find(p=>p.id==='tc131c-1').pose.position_mm[0]+=10;
        if(fault==='drop-connection')runs(ui.state.doc)[0].attachments.pop();
        if(fault==='wrong-end')ui.state.scene.parts.find(p=>p.id==='tube-c-4').pose.position_mm[0]+=10;
        const actual=endpoints(ui.state.scene.parts.find(p=>p.id==='tube-c-4'));
        closePoint(actual[fixed],span[fixed],'opposite endpoint',.002);
        closePoint(actual[moving],v(span[moving]).add(delta).toArray(),'requested endpoint',.002);
        for(const old of scene.parts) {
          if(old.id==='tube-c-4')continue;
          const part=ui.state.scene.parts.find(p=>p.id===old.id);
          const expected=v(old.pose.position_mm).add(old.id===follower?delta:v([0,0,0]));
          closePoint(part.pose.position_mm,expected.toArray(),old.id,.002);
          if(old.length_mm)assert.equal(part.length_mm,old.length_mm,'neighbor pipe was resized');
          const a=matrix(old.pose),b=matrix(part.pose);
          for(const axis of [[1,0,0],[0,1,0],[0,0,1]])closePoint(v(axis).transformDirection(a).toArray(),v(axis).transformDirection(b).toArray(),'orientation',1e-6);
        }
        assert.deepEqual(connections(ui.state.doc).map(edgeKey).sort(),edges,'connection was lost');
        assert.deepEqual(ui.state.doc.anchors,initial.anchors,'world anchor was released');
        assert.ok(ui.state.scene.parts.every(p=>!p.conflicts?.length),'resize introduced a draft conflict');
        const after=structuredClone(ui.state.doc);
        ui.document.querySelector('#undo').click();await settled();assert.deepEqual(clean(ui.state.doc),clean(before));
        ui.document.querySelector('#redo').click();await settled();assert.deepEqual(clean(ui.state.doc),clean(after));
      }
      const saved=structuredClone(ui.state.doc);await ui.close();ui=await editor(saved);
      const exact=runs(saved).length?await post('draft-finalize',saved):{document:saved};
      assert.equal((await post('validate',exact.document)).valid,true,'result must still be buildable');
      assert.deepEqual(connections(ui.state.doc).map(edgeKey).sort(),edges,'reopening lost a connection');
    } catch(error) {
      if(retain) {
        const dir=`fuzz-runs/anchored-resize/${seed}-${Date.now()}`;await mkdir(dir,{recursive:true});
        await writeFile(`${dir}/replay.json`,JSON.stringify({seed,mode,reverse,step,document:initial,actions,failures},null,2));
        error.message+=`\nanchored resize seed=${seed} mode=${mode} reverse=${reverse} step=${step}; replay=${dir}/replay.json`;
      }
      throw error;
    } finally {await ui?.close();}
  }
  test('U/J grows and shrinks an occupied pipe end connected to a world-fixed crossbar',async()=>{
    await exercise(await fixture());
  });
  test('seeded anchored resize workflows preserve remote anchors and occupied ends',async()=>{
    if(process.env.PIPESIM_ANCHORED_RESIZE_REPLAY) {
      const replay=JSON.parse(await readFile(process.env.PIPESIM_ANCHORED_RESIZE_REPLAY,'utf8'));
      await exercise(replay.document,{...replay,prepared:true});return;
    }
    const root=Number(process.env.PIPESIM_ANCHORED_RESIZE_SEED??randomBytes(4).readUInt32LE());
    const count=Number(process.env.PIPESIM_ANCHORED_RESIZE_CASES??24),rng=random(root);
    assert.ok(Number.isInteger(count)&&count>=0);assert.ok(Number.isInteger(root)&&root>=0&&root<=0xffffffff);
    const base=await fixture(),caseSeed=process.env.PIPESIM_ANCHORED_RESIZE_CASE_SEED;
    const seeds=caseSeed!==undefined?[Number(caseSeed)]:[0,1,2,3,4,5,...Array.from({length:count},()=>Math.floor(rng()*0x100000000))];
    console.log(`Anchored resize root_seed=${root}, cases=${seeds.length}`);
    for(const seed of seeds) {
      assert.ok(Number.isInteger(seed)&&seed>=0&&seed<=0xffffffff);
      // A case must have the same topology, mode and anchors in an ordinary
      // batch and when replayed alone by seed.
      const index=seed;
      const doc=structuredClone(base),r=random(seed);
      if(index>=6) {
        // Vary the actual cut length and the support's station along it, not
        // just the camera/world orientation of an otherwise identical case.
        const scene=await post('resolve',doc);
        const span=endpoints(scene.parts.find(p=>p.id==='tube-c-4')),axis=v(span[1]).sub(v(span[0])).normalize();
        const run=runs(doc).find(r=>r.id==='tube-c-4');
        for(const [end,field,fitting] of [[0,'start_mm','tc132c-1'],[1,'end_mm','tc104c-1']]) {
          const shift=axis.clone().multiplyScalar((r()-.5)*400);
          run[field]=v(span[end]).add(shift).toArray();
          const part=doc.parts.find(p=>p.id===fitting);part.pose.position_mm=v(part.pose.position_mm).add(shift).toArray();
        }
        const turn=new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(r()*2-1,r()*2-1,r()*6-3,'ZYX'));
        turn.setPosition(v([r()*1000-500,r()*1000-500,4000]));
        for(const p of doc.parts) {
          const transformed=turn.clone().multiply(matrix(p.pose)),q=new THREE.Quaternion().setFromRotationMatrix(transformed),e=new THREE.Euler().setFromQuaternion(q,'ZYX');
          p.pose={position_mm:new THREE.Vector3().setFromMatrixPosition(transformed).toArray(),rotation_deg:[e.x,e.y,e.z].map(THREE.MathUtils.radToDeg)};
        }
        for(const run of runs(doc))for(const field of ['start_mm','end_mm'])run[field]=v(run[field]).applyMatrix4(turn).toArray();
        if(index%2)doc.anchors.pop();
      }
      await exercise(doc,{seed,mode:['draft','exact','mixed'][index%3],reverse:index%2===1,step:index<6?10:1+Math.floor(r()*60)});
    }
    console.log(`Anchored resize cases_passed=${seeds.length} keyboard_edits=${seeds.length*4}`);
  });
  test('anchored resize workflow oracles reject false errors and silent geometry damage',async()=>{
    for(const [fault,pattern] of [['reject',/rejected/],['move-anchor',/tc131c-1/],['drop-connection',/connection was lost/],['wrong-end',/endpoint/]])
      await assert.rejects(exercise(await fixture(),{fault,retain:false}),pattern,`oracle survived ${fault}`);
  });
}
