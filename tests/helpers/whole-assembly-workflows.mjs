import assert from 'node:assert/strict';
import {mkdir,readFile,writeFile} from 'node:fs/promises';
import {randomBytes} from 'node:crypto';
import * as THREE from 'three';
import {matrix,closePoint,random,v} from './geometry-workflow-model.mjs';

export function registerWholeAssemblyWorkflows({test,editor,post}) {
  async function variant(seed) {
    const rng=random(seed),count=seed<6?50:12+Math.floor(rng()*90),station=seed<6?25:2+Math.floor(rng()*(count-4));
    const document={format:'pipesim/1',units:'mm-kg-s-N-deg',name:'Free midpoint chains',parts:[],joints:[],anchors:[],
      objects:['left','right','unconnected'].map((id,i)=>({id,template:'chain',parameters:{length_mm:count*20},layout_mode:'posable',
        pose:{position_mm:[i===2?4000:0,0,1600],rotation_deg:[0,i===0?-35:35,seed<6?0:rng()*180]}}))};
    const scene=await post('resolve',document);
    const point=id=>{const p=scene.parts.find(p=>p.id===`${id}/link-${station}`);return v(p.ports.a.position_mm).applyMatrix4(matrix(p.pose));};
    document.objects[1].pose.position_mm=v(document.objects[1].pose.position_mm).add(point('left').sub(point('right'))).toArray();
    document.joints=[{id:'midpoint',type:['spherical','revolute','fixed'][Math.floor(seed/2)%3],
      a:{part:`left/link-${station}`,port:'a'},b:{part:`right/link-${station}`,port:'a'}}];
    return {version:1,seed,count,document,selected:seed%2?'right':'left',
      shift:[(rng()-.5)*2400,(rng()-.5)*2400,100+rng()*900],angle:(rng()-.5)*2};
  }
  async function exercise(spec,{fault=null,retain=true}={}) {
    let ui;const failures=[],requests=[];
    try {
      ui=await editor(structuredClone(spec.document),null,{
        onError:e=>failures.push(String(e)),onToast:(text,error)=>{if(error)failures.push(text);},
        onRequest:(path,body)=>requests.push({path,body}),
        onResponse:(path,status,data)=>{if(status>=400)failures.push({path,status,data});},
        interceptFetch:path=>fault==='reject'&&path==='/api/move-object'?Promise.resolve(new Response(JSON.stringify({error:'Injected whole-object rejection'}),{status:400})):null,
      });
      await ui.drain();ui.state.snap=false;ui.state.connectionSnap=false;ui.state.snapSettings.alignEnabled=false;
      const group=ui.document.querySelector(`[data-object-tree="${spec.selected}"]`);group.open=true;group.dispatchEvent(new ui.window.Event('toggle'));await ui.drain();
      const selected=`${spec.selected}/link-${spec.count}`;ui.select(selected);
      ui.document.querySelector('[data-tool="translate"]').click();
      assert.equal(ui.document.querySelector('[data-object-mode="limb"]').getAttribute('aria-pressed'),'true');
      const tail=ui.partObjects.get(selected).position.clone();
      ui.gizmo.axis='XYZ';ui.gizmo.dispatchEvent({type:'mouseDown'});
      ui.gizmo.object.position.add(v([20,0,35]));ui.gizmo.object.updateMatrix();
      ui.gizmo.dispatchEvent({type:'objectChange'});await ui.drain();ui.gizmo.dispatchEvent({type:'mouseUp'});
      await ui.wait(()=>!ui.state.placementPending&&ui.state.undo.length===1);await ui.drain();
      assert.ok(ui.partObjects.get(selected).position.distanceTo(tail)>1,'segment posing did not move the tail');
      ui.document.querySelector('[data-object-mode="whole"]').click();await ui.drain();
      assert.equal(ui.document.querySelector('[data-object-mode="whole"]').getAttribute('aria-pressed'),'true');
      for(const mode of ['translate','rotate']) {
        ui.document.querySelector(`[data-tool="${mode}"]`).click();
        const original=structuredClone(ui.state.doc),scene=structuredClone(ui.state.scene),undo=ui.state.undo.length;
        ui.gizmo.object.updateMatrix();const handle=ui.gizmo.object.matrix.clone();
        ui.gizmo.axis=mode==='translate'?'XYZ':'Z';ui.gizmo.dispatchEvent({type:'mouseDown'});
        if(mode==='translate')ui.gizmo.object.position.add(v(spec.shift));
        else ui.gizmo.object.quaternion.premultiply(new THREE.Quaternion().setFromAxisAngle(v([0,0,1]),spec.angle));
        ui.gizmo.object.updateMatrix();const delta=ui.gizmo.object.matrix.clone().multiply(handle.invert());
        const started=performance.now();ui.gizmo.dispatchEvent({type:'objectChange'});await ui.drain();
        assert.deepEqual(failures,[],'whole-object preview emitted an error');
        assert.ok(performance.now()-started<2500,'whole-object preview exceeded interactive budget');
        function verify(preview) {
          for(const part of scene.parts) {
            const expected=matrix(part.pose);if(!part.id.startsWith('unconnected/'))expected.premultiply(delta);
            const actual=preview?ui.partObjects.get(part.id).matrix:matrix(ui.state.scene.parts.find(p=>p.id===part.id).pose);
            closePoint(new THREE.Vector3().setFromMatrixPosition(actual).toArray(),new THREE.Vector3().setFromMatrixPosition(expected).toArray(),`${mode} ${preview?'preview':'commit'} ${part.id}`,.002);
            for(const axis of [[1,0,0],[0,1,0],[0,0,1]])closePoint(v(axis).transformDirection(actual).toArray(),v(axis).transformDirection(expected).toArray(),`${part.id} orientation`,1e-6);
          }
        }
        if(fault==='leave-payload')ui.partObjects.get(`${spec.selected==='left'?'right':'left'}/link-1`).matrix.elements[12]+=20;
        verify(true);
        ui.gizmo.dispatchEvent({type:'mouseUp'});
        await ui.wait(()=>!ui.state.placementPending&&(ui.state.undo.length===undo+1||failures.length));await ui.drain();
        assert.deepEqual(failures,[],'whole-object commit emitted an error');
        assert.equal(ui.document.querySelector('#modal').open,false,'free placement opened a dialog');
        assert.equal(ui.state.undo.length,undo+1);verify(false);
        if(fault==='drop-joint')ui.state.doc.joints.pop();
        assert.deepEqual(ui.state.doc.joints,original.joints,'whole movement changed attachments');
        assert.deepEqual(ui.state.doc.objects.map(({pose,...rest})=>rest),original.objects.map(({pose,...rest})=>rest),'whole movement rebaked object shapes');
        const committed=structuredClone(ui.state.doc);
        ui.document.querySelector('#undo').click();await ui.drain();assert.deepEqual(ui.state.doc,original);
        ui.document.querySelector('#redo').click();await ui.drain();assert.deepEqual(ui.state.doc,committed);verify(false);
      }
      assert.ok(requests.some(r=>r.path==='/api/move'&&r.body.selected===selected),'segment pose route was not exercised');
      assert.equal(requests.filter(r=>r.path==='/api/move-object'&&r.body.preview).length,2);
      assert.equal(requests.filter(r=>r.path==='/api/move-object'&&!r.body.preview).length,2);
    } catch(error) {
      if(retain) {
        const directory=`fuzz-runs/whole-assembly/${spec.seed}-${Date.now()}`;await mkdir(directory,{recursive:true});
        await writeFile(`${directory}/replay.json`,JSON.stringify(spec,null,2));
        await writeFile(`${directory}/failure.json`,JSON.stringify({error:error.stack,failures,requests,document:ui?.state.doc},null,2));
        throw new Error(`${error.message}\nWhole assembly seed=${spec.seed}; replay=${directory}/replay.json`,{cause:error});
      }
      throw error;
    } finally {await ui?.close();}
  }
  test('seeded whole assembly workflows pose then rigidly move midpoint-connected chains',async()=>{
    if(process.env.PIPESIM_WHOLE_ASSEMBLY_REPLAY){await exercise(JSON.parse(await readFile(process.env.PIPESIM_WHOLE_ASSEMBLY_REPLAY,'utf8')));return;}
    const root=Number(process.env.PIPESIM_WHOLE_ASSEMBLY_SEED??randomBytes(4).readUInt32LE()),single=process.env.PIPESIM_WHOLE_ASSEMBLY_CASE_SEED;
    const count=Number(process.env.PIPESIM_WHOLE_ASSEMBLY_CASES??8);
    assert.ok(Number.isSafeInteger(count)&&count>=0);
    for(const seed of [root,...(single===undefined?[]:[Number(single)])])assert.ok(Number.isInteger(seed)&&seed>=0&&seed<=0xffffffff);
    const rng=random(root),seeds=single===undefined?[0,1,2,3,4,5,...Array.from({length:count},()=>Math.floor(rng()*0x100000000))]:[Number(single)];
    console.log(`Whole assembly root_seed=${root} cases=${seeds.length}`);
    for(const seed of seeds){await exercise(await variant(seed));console.log(`Whole assembly case_seed=${seed} passed`);}
    console.log(`Whole assembly cases_passed=${seeds.length} rigid_moves=${seeds.length*2}`);
  });
  test('whole assembly workflow oracles reject errors, stationary payloads and lost attachments',async()=>{
    for(const [fault,pattern] of [['reject',/emitted an error/],['leave-payload',/right\/link-1/],['drop-joint',/changed attachments/]])
      await assert.rejects(exercise(await variant(0),{fault,retain:false}),pattern);
  });
}
