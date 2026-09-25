// Exercise the real editor handlers and Python API in a DOM harness. WebGL and
// orbit widgets are replaced; picking, projections, DOM events and documents are real.
import test,{before,after} from 'node:test';
import assert from 'node:assert/strict';
import {readFile,mkdtemp,rm,realpath} from 'node:fs/promises';
import {join,dirname,basename} from 'node:path';
import {tmpdir} from 'node:os';
import {spawn} from 'node:child_process';
import {setTimeout as pause} from 'node:timers/promises';
import {JSDOM} from 'jsdom';
import * as Three from 'three';
import {STLLoader as ThreeSTLLoader} from 'three/addons/loaders/STLLoader.js';
import * as snapping from '../pipesim/web/snapping.js';
import * as settings from '../pipesim/web/snap-settings.js';

let server,url,bootstrap,workspace;
before(async()=>{
  workspace=await mkdtemp(join(tmpdir(),'pipesim-editor-ui-'));
  server=spawn(process.env.PYTHON||'python',['-m','pipesim','editor','--port','0','--root',workspace],{cwd:process.cwd(),windowsHide:true});
  let stdout='',stderr='';server.stderr.on('data',chunk=>stderr+=chunk);
  await new Promise((resolve,reject)=>{const timer=setTimeout(()=>reject(new Error('Editor server did not start: '+stderr)),15000);
    server.on('error',reject);server.stdout.on('data',chunk=>{stdout+=chunk;const match=stdout.match(/http:\/\/127\.0\.0\.1:\d+/);if(match){url=match[0];clearTimeout(timer);resolve();}});});
  bootstrap=await (await fetch(url+'/api/bootstrap')).json();
});
after(async()=>{
  if(server&&server.exitCode===null&&server.signalCode===null)await new Promise(resolve=>{server.once('exit',resolve);server.kill();});
  if(workspace){
    const target=await realpath(workspace);
    assert.equal(dirname(target),await realpath(tmpdir()));assert.ok(basename(target).startsWith('pipesim-editor-ui-'));
    await rm(target,{recursive:true,force:true});
  }
});

async function post(route,doc,extra={}){
  const response=await fetch(url+'/api/'+route,{method:'POST',headers:{'Content-Type':'application/json','X-PipeSim-Token':bootstrap.token},body:JSON.stringify({document:doc,path:'output/ui-test.pipe.yaml',...extra})});
  const data=await response.json();assert.ok(response.ok,data.error);return data;
}
const fixture=(catalog='TC101C')=>({format:'pipesim/1',units:'mm-kg-s-N-deg',name:'Interaction test',parts:[
  {id:'pipe',catalog:'tubeclamp.tube-C',parameters:{length_mm:1000},pose:{position_mm:[0,0,500]}},
  {id:'tee',catalog:'tubeclamp.'+catalog,pose:{position_mm:[300,0,500]}}],joints:[],anchors:[{part:'pipe',surface:'fixture'}]});

async function editor(doc,storedDefaults=null,{beforeResponse=()=>{},bootstrapOverride=()=>({}),interceptFetch=()=>null}={}){
  const html=await readFile('pipesim/web/index.html','utf8');const dom=new JSDOM(html,{url});const window=dom.window;
  const document=window.document,timers=[],captures=new Set();let frame,inFlight=0;
  if(storedDefaults)window.localStorage.setItem('pipesim.snap-defaults.v1',storedDefaults);
  window.matchMedia=()=>({matches:false,addEventListener(){}});
  window.HTMLDialogElement.prototype.showModal=function(){this.setAttribute('open','');};
  window.HTMLDialogElement.prototype.close=function(){this.removeAttribute('open');};
  window.HTMLElement.prototype.getBoundingClientRect=()=>({x:0,y:0,left:0,top:0,width:900,height:600,right:900,bottom:600});
  window.HTMLElement.prototype.setPointerCapture=id=>captures.add(id);
  window.HTMLElement.prototype.hasPointerCapture=id=>captures.has(id);
  window.HTMLElement.prototype.releasePointerCapture=id=>captures.delete(id);
  class Renderer {constructor(){this.domElement=document.createElement('canvas');this.shadowMap={};}setPixelRatio(){}setSize(){}render(scene,camera){scene.updateMatrixWorld(true);camera.updateMatrixWorld(true);this.domElement.dataset.rendered='true';}dispose(){}}
  class Controls extends Three.EventDispatcher {constructor(camera){super();this.camera=camera;this.target=new Three.Vector3();this.enabled=true;}update(){this.camera.lookAt(this.target);this.camera.updateMatrixWorld();}dispose(){}}
  class Gizmo extends Three.EventDispatcher {constructor(){super();this.axis=null;this.dragging=false;}getHelper(){return new Three.Group();}setSize(){}setTranslationSnap(v){this.translationSnap=v;}setRotationSnap(v){this.rotationSnap=v;}attach(o){this.object=o;}detach(){this.object=null;}setMode(m){this.mode=m;}}
  const resolved=await post('resolve',doc);
  const fetchEditor=async(path,options)=>{
    if(path==='/api/bootstrap')return new Response(JSON.stringify({...bootstrap,path:'output/ui-test.pipe.yaml',document:doc,scene:resolved,...bootstrapOverride()}));
    const intercepted=interceptFetch(path,options);if(intercepted)return intercepted;
    inFlight++;try{const response=await fetch(new URL(path,url),options),body=await response.arrayBuffer();await beforeResponse(path);return new Response(body,{status:response.status,headers:response.headers});}finally{inFlight--;}
  };
  const source=(await readFile('pipesim/web/app.js','utf8')).replace(/^import .*;\r?\n/gm,'');
  class MeshLoader {load(path,success,progress,failure){fetchEditor(path).then(response=>response.arrayBuffer()).then(bytes=>success(new ThreeSTLLoader().parse(bytes))).catch(failure);}}
  const AsyncFunction=Object.getPrototypeOf(async function(){}).constructor;
  const evaluate=new AsyncFunction('THREE','OrbitControls','TransformControls','GLTFLoader','STLLoader','OBJLoader','connectionCandidates','clearConnectionIntent','alignmentDelta','socketOccupied','rotationAlignment',
    'DEFAULT_SNAP_SETTINGS','loadSnapDefaults','saveSnapDefaults','validateSnapSettings',
    'window','document','devicePixelRatio','getComputedStyle','ResizeObserver','requestAnimationFrame','fetch','setTimeout','clearTimeout',
    source+'\nreturn {state, camera, scene, partObjects, viewport, gizmo};');
  const app=await evaluate({...Three,WebGLRenderer:Renderer},Controls,Gizmo,class{},MeshLoader,class{},
    snapping.connectionCandidates,snapping.clearConnectionIntent,snapping.alignmentDelta,snapping.socketOccupied,snapping.rotationAlignment,
    settings.DEFAULT_SNAP_SETTINGS,settings.loadSnapDefaults,settings.saveSnapDefaults,settings.validateSnapSettings,
    window,document,1,()=>({getPropertyValue:()=> '#dce5e9'}),class{constructor(callback){this.callback=callback;}observe(){this.callback();}},callback=>{frame=callback;},fetchEditor,
    (callback,delay)=>{const timer=setTimeout(callback,delay);timers.push(timer);return timer;},clearTimeout);
  const tick=()=>frame?.(performance.now());tick();
  const wait=async(predicate,timeout=12000)=>{const deadline=Date.now()+timeout;while(!predicate()){if(Date.now()>deadline)throw new Error('Timed out: '+document.querySelector('#toast').textContent+' / '+document.querySelector('#snap-review-status')?.textContent+' / '+document.querySelector('#opening-status')?.textContent);await pause(20);tick();}};
  const select=id=>document.querySelector(`[data-select="${id}"]`).click();
  const pixel=point=>{tick();const p=point.clone().project(app.camera);return {x:(p.x+1)*450,y:(1-p.y)*300};};
  const pointer=(type,point)=>{tick();const event=new window.MouseEvent(type,{clientX:point.x,clientY:point.y,button:0,bubbles:true});Object.defineProperty(event,'pointerId',{value:1});document.querySelector('#viewport canvas').dispatchEvent(event);};
  const drag=(from,to)=>{pointer('pointerdown',from);pointer('pointermove',{x:(from.x+to.x)/2,y:(from.y+to.y)/2});pointer('pointermove',to);pointer('pointerup',to);};
  const drain=async()=>{await wait(()=>inFlight===0,45000);await pause(30);};
  return {...app,document,window,wait,select,pixel,pointer,drag,tick,drain,async close(){await drain();timers.forEach(clearTimeout);dom.window.close();}};
}

for(const z of [500,-10])test(`Drop to floor places a horizontal pipe from Z=${z} and supports Undo`,async()=>{
  const doc=fixture();doc.parts=[{...doc.parts[0],pose:{position_mm:[125,75,z],rotation_deg:[0,90,0]}}];doc.anchors=[];
  const ui=await editor(doc);try{
    ui.select('pipe');const before=structuredClone(ui.state.doc);
    const button=ui.document.querySelector('[data-drop-floor="part"]');
    assert.equal(button.previousElementSibling.querySelector('[data-pose]').dataset.pose,'position_mm');
    button.click();await ui.wait(()=>!ui.state.placementPending);
    assert.equal(ui.state.undo.length,1);
    const pose=ui.state.scene.parts[0].pose;assert.ok(Math.abs(pose.position_mm[2]-21.2)<1e-5);
    assert.deepEqual(pose.position_mm.slice(0,2),[125,75]);assert.ok(pose.rotation_deg.every((v,i)=>Math.abs(v-[0,90,0][i])<1e-5));
    const accepted=structuredClone(ui.state.doc);ui.document.querySelector('[data-drop-floor="part"]').click();
    await ui.wait(()=>!ui.state.placementPending);assert.equal(ui.state.undo.length,1);
    assert.match(ui.document.querySelector('#toast').textContent,/Already resting/);
    ui.document.querySelector('#undo').click();await ui.wait(()=>Math.abs(ui.state.scene.parts[0].pose.position_mm[2]-z)<1e-5);
    assert.deepEqual({...ui.state.doc,results:{}},{...before,results:{}});
    ui.document.querySelector('#redo').click();await ui.wait(()=>Math.abs(ui.state.scene.parts[0].pose.position_mm[2]-21.2)<1e-5);
    assert.deepEqual({...ui.state.doc,results:{}},{...accepted,results:{}});
  }finally{await ui.close();}
});

test('Drop to floor in object position moves the whole person and preserves the pose',async()=>{
  const doc=seatedHuman();doc.objects[0].pose={position_mm:[2000,100,-700],rotation_deg:[0,0,180]};
  const ui=await editor(doc);try{
    ui.document.querySelector('[data-object-select="person"]').click();
    const before=new Map(ui.state.scene.parts.map(p=>[p.id,structuredClone(p.pose)]));
    assert.equal(ui.document.querySelector('[data-drop-floor="part"]').closest('.inspect-section').hidden,true);
    ui.document.querySelector('[data-drop-floor="object"]').click();await ui.wait(()=>!ui.state.placementPending);
    assert.equal(ui.state.undo.length,1);assert.equal(ui.state.doc.objects.length,1);assert.equal(ui.state.doc.parts.length,0);
    const shift=ui.state.scene.parts[0].pose.position_mm[2]-before.get(ui.state.scene.parts[0].id).position_mm[2];assert.ok(shift>0);
    for(const p of ui.state.scene.parts){
      assert.ok(Math.abs(p.pose.position_mm[2]-before.get(p.id).position_mm[2]-shift)<1e-5);
      assert.ok(p.pose.rotation_deg.every((v,i)=>Math.abs(v-before.get(p.id).rotation_deg[i])<1e-5));
    }
    ui.document.querySelector('[data-drop-floor="object"]').click();await ui.wait(()=>!ui.state.placementPending);
    assert.equal(ui.state.undo.length,1);assert.match(ui.document.querySelector('#toast').textContent,/Already resting/);
  }finally{await ui.close();}
});

test('Drop to floor reports a world fixing without creating an Undo entry',async()=>{
  const doc=fixture();doc.parts=[doc.parts[0]];doc.parts[0].pose.position_mm[2]=800;
  const ui=await editor(doc);try{
    ui.select('pipe');const before=structuredClone(ui.state.doc);ui.document.querySelector('[data-drop-floor="part"]').click();
    await ui.wait(()=>!ui.state.placementPending);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);
    assert.match(ui.document.querySelector('#toast').textContent,/pipe is fixed to the world/);
    assert.equal(ui.document.querySelector('[data-drop-floor="part"]').disabled,false);
  }finally{await ui.close();}
});

async function checkFindings(ui){
  ui.document.querySelector('[data-mode="check"]').click();ui.document.querySelector('[data-run="validate"]').click();
  await ui.wait(()=>!ui.state.busy&&ui.state.checks);
}

test('double-clicking a loose-screw finding opens the precise Properties control and lets it be fixed',async()=>{
  const result=await post('snap-options',fixture(),{member:'pipe',connector:'tee',port:'through',at_mm:500,locked:false});
  const doc=result.options.find(o=>o.move===result.recommended).document;
  const ui=await editor(doc);try{
    await checkFindings(ui);
    const index=ui.state.checks.issues.findIndex(i=>i.code==='LOOSE_SCREW'),issue=ui.state.checks.issues[index];
    const button=ui.document.querySelector(`[data-result-finding="${index}"]`),text=button.querySelector('p');
    const original=structuredClone(ui.state.doc);
    text.dispatchEvent(new ui.window.MouseEvent('click',{bubbles:true,detail:1}));
    assert.equal(ui.state.mode,'check');assert.equal(ui.state.selected,'tee');assert.equal(button.isConnected,true);
    text.dispatchEvent(new ui.window.MouseEvent('click',{bubbles:true,detail:2}));
    text.dispatchEvent(new ui.window.MouseEvent('dblclick',{bubbles:true,detail:2}));
    assert.equal(ui.state.mode,'design');assert.equal(ui.document.querySelector('#inspector-title').textContent,'PROPERTIES');
    assert.equal(ui.document.activeElement.dataset.lock,issue.joint);
    assert.equal(ui.document.querySelector('[data-mode="design"]').getAttribute('aria-selected'),'true');
    assert.deepEqual(ui.state.doc,original);assert.equal(ui.state.undo.length,0);
    ui.document.activeElement.click();await ui.wait(()=>ui.state.scene.joints.find(j=>j.id===issue.joint).locked);
    await checkFindings(ui);assert.ok(!ui.state.checks.issues.some(i=>i.code==='LOOSE_SCREW'&&i.joint===issue.joint));
  }finally{await ui.close();}
});

test('an unsupported finding with no part ID opens Position and Drop to floor using its support component',async()=>{
  const doc=blankDesign('Floating box');doc.parts=[{id:'box',body:{kind:'rigid',mass_kg:1,geometry:[{type:'box',size_mm:[100,100,100]}]},pose:{position_mm:[0,0,400]}}];
  const ui=await editor(doc);try{
    await checkFindings(ui);
    const index=ui.state.checks.issues.findIndex(i=>i.code==='UNSUPPORTED');assert.deepEqual(ui.state.checks.issues[index].parts,[]);
    ui.document.querySelector(`[data-result-finding="${index}"]`).dispatchEvent(new ui.window.MouseEvent('dblclick',{bubbles:true}));
    assert.equal(ui.state.mode,'design');assert.equal(ui.state.selected,'box');
    assert.equal(ui.document.activeElement.dataset.dropFloor,'part');assert.equal(ui.state.undo.length,0);
    ui.document.activeElement.click();await ui.wait(()=>!ui.state.placementPending);
    await checkFindings(ui);assert.ok(!ui.state.checks.issues.some(i=>i.code==='UNSUPPORTED'));
  }finally{await ui.close();}
});

test('Enter on a grouped-part finding opens its part properties without expanding or moving it',async()=>{
  const doc=blankDesign('Grouped collision');doc.objects=[{id:'object',template:'custom',components:{parts:['a','b'].map(id=>({id,body:{kind:'rigid',mass_kg:1,geometry:[{type:'box',size_mm:[100,100,100]}]}})),joints:[]}}];
  const ui=await editor(doc);try{
    await checkFindings(ui);
    const index=ui.state.checks.issues.findIndex(i=>i.code==='INTERSECTION');assert.ok(index>=0);
    const original=structuredClone(ui.state.doc);
    ui.document.querySelector(`[data-result-finding="${index}"]`).dispatchEvent(new ui.window.KeyboardEvent('keydown',{key:'Enter',bubbles:true,cancelable:true}));
    assert.equal(ui.state.mode,'design');assert.equal(ui.state.selected,'object/a');
    assert.equal(ui.document.activeElement.dataset.partPose,'position_mm');
    assert.equal(ui.document.activeElement.closest('.inspect-section').hidden,false);
    assert.deepEqual(ui.state.doc,original);assert.equal(ui.state.undo.length,0);
  }finally{await ui.close();}
});

test('explicit connection previews, centres the tee and undoes as one edit',async()=>{
  const ui=await editor(fixture('TC104C'));try{
    ui.select('pipe');ui.document.querySelector('#connect-selected').click();ui.select('tee');
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    assert.equal(ui.document.querySelector('#snap-station').value,'500');
    assert.ok(ui.document.querySelector('#snap-preview-view canvas[data-rendered]'));
    assert.equal(ui.state.doc.joints.length,0);
    ui.document.querySelector('#modal-actions .primary').click();await ui.wait(()=>ui.state.doc.joints.length===1&&!ui.state.placementPending);
    assert.deepEqual(ui.state.doc.parts.find(p=>p.id==='tee').pose.position_mm,[0,0,500]);assert.equal(ui.state.undo.length,1);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.doc.joints.length===0);assert.equal(ui.state.doc.parts[1].pose.position_mm[0],300);
  }finally{await ui.close();}
});

test('dragging an aligned fitting connects on release without a dialog',async()=>{
  const ui=await editor(fixture());try{
    ui.drag(ui.pixel(new Three.Vector3(300,0,500)),ui.pixel(new Three.Vector3(0,0,500)));
    await ui.wait(()=>ui.state.doc.joints.length===1);
    assert.equal(ui.document.querySelector('#modal').open,false);assert.equal(ui.state.doc.joints[0].b.at_mm,500);
  }finally{await ui.close();}
});

test('misaligned drag requires preview and leaves the anchored pipe fixed',async()=>{
  const ui=await editor(fixture('TC128C'));try{
    const source=ui.pixel(new Three.Vector3(300,0,510)),target=ui.pixel(new Three.Vector3(0,0,1000));
    ui.drag(source,target);await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    assert.equal(ui.state.doc.joints.length,0);assert.equal(ui.document.querySelector('#snap-move').value,'connector');
    ui.document.querySelector('#modal-actions .primary').click();await ui.wait(()=>ui.state.doc.joints.length===1);
    assert.deepEqual(ui.state.doc.parts.find(p=>p.id==='pipe').pose.position_mm,[0,0,500]);
  }finally{await ui.close();}
});

test('connection toggle leaves ordinary dragging available',async()=>{
  const ui=await editor(fixture());try{
    ui.document.querySelector('#connection-snap-button').click();assert.equal(ui.document.querySelector('#connection-snap-button').getAttribute('aria-pressed'),'false');
    ui.drag(ui.pixel(new Three.Vector3(300,0,500)),ui.pixel(new Three.Vector3(0,0,500)));
    await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    assert.equal(ui.state.doc.joints.length,0);assert.ok(new Three.Vector3(...ui.state.doc.parts[1].pose.position_mm).distanceTo(new Three.Vector3(300,0,500))>100);
  }finally{await ui.close();}
});

test('cancelling a preview changes neither document nor undo history',async()=>{
  const ui=await editor(fixture('TC128C'));try{
    ui.select('pipe');ui.document.querySelector('#connect-selected').click();ui.select('tee');
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    ui.document.querySelector('#modal-close').click();
    assert.equal(ui.state.doc.joints.length,0);assert.equal(ui.state.undo.length,0);assert.equal(ui.state.placementPending,false);
  }finally{await ui.close();}
});

test('editing an old 500 mm insertion offers the continuous main run',async()=>{
  const doc=fixture('TC104C');doc.joints=[{id:'old','type':'socket',a:{part:'tee',port:'run_end'},b:{part:'pipe',end:'start'},insertion_mm:500,locked:true}];
  const ui=await editor(doc);try{
    ui.select('tee');ui.document.querySelector('[data-joint-edit="old"]').click();
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    assert.equal(ui.document.querySelector('#snap-station').value,'500');
    ui.document.querySelector('#modal-actions .primary').click();await ui.wait(()=>ui.state.doc.joints[0].a.port==='through');
    assert.equal(ui.state.doc.joints[0].id,'old');assert.equal(ui.state.doc.joints[0].b.at_mm,500);
  }finally{await ui.close();}
});

test('snap settings apply to controls and saved defaults load in a fresh editor',async()=>{
  let stored;
  const ui=await editor(fixture());try{
    ui.document.querySelector('#snap-settings-button').click();assert.equal(ui.document.querySelector('#settings-angle').value,'90');
    ui.document.querySelector('[data-snap-preset="45"]').click();ui.document.querySelector('#settings-position').value='25';
    ui.document.querySelector('#settings-capture').value='16';ui.document.querySelector('#settings-align').checked=false;
    ui.document.querySelector('#modal-actions .primary').click();
    assert.equal(ui.gizmo.translationSnap,25);assert.equal(ui.gizmo.rotationSnap,Math.PI/4);
    stored=ui.window.localStorage.getItem('pipesim.snap-defaults.v1');assert.ok(stored);
    assert.equal(ui.state.undo.length,0);assert.equal(ui.state.dirty,false);
  }finally{await ui.close();}
  const fresh=await editor(fixture(),stored);try{
    assert.equal(fresh.state.snapSettings.rotationDeg,45);assert.equal(fresh.state.snapSettings.connectionPixels,16);
    assert.equal(fresh.gizmo.translationSnap,25);assert.equal(fresh.gizmo.rotationSnap,Math.PI/4);
    fresh.document.querySelector('#snap-settings-button').click();fresh.document.querySelector('[data-snap-preset="90"]').click();
    [...fresh.document.querySelectorAll('#modal-actions button')].find(b=>b.textContent==='Apply').click();
    assert.equal(fresh.gizmo.rotationSnap,Math.PI/2);assert.equal(fresh.window.localStorage.getItem('pipesim.snap-defaults.v1'),stored);
  }finally{await fresh.close();}
});

test('rotation corrects a small pointing error to exactly 90 degrees',async()=>{
  const ui=await editor(fixture());try{
    ui.select('tee');ui.document.querySelector('[data-tool="rotate"]').click();ui.gizmo.axis='Z';ui.gizmo.dispatchEvent({type:'mouseDown'});
    ui.partObjects.get('tee').quaternion.setFromAxisAngle(new Three.Vector3(0,0,1),Three.MathUtils.degToRad(88.8));
    ui.gizmo.dispatchEvent({type:'objectChange'});
    assert.ok(Math.abs(new Three.Euler().setFromQuaternion(ui.partObjects.get('tee').quaternion).z-Math.PI/2)<1e-7);
    ui.gizmo.dispatchEvent({type:'mouseUp'});await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    assert.equal(ui.state.doc.parts[1].pose.rotation_deg[2],90);
  }finally{await ui.close();}
});

test('rotation can align to an existing angled frame between grid increments',async()=>{
  const doc=fixture();doc.parts.push({id:'reference',catalog:'tubeclamp.TC101C',pose:{position_mm:[-500,0,500],rotation_deg:[0,0,17]}});
  const ui=await editor(doc);try{
    ui.select('tee');ui.document.querySelector('[data-tool="rotate"]').click();ui.gizmo.axis='Z';ui.gizmo.dispatchEvent({type:'mouseDown'});
    ui.partObjects.get('tee').quaternion.setFromAxisAngle(new Three.Vector3(0,0,1),Three.MathUtils.degToRad(16));
    ui.gizmo.dispatchEvent({type:'objectChange'});
    assert.ok(Math.abs(new Three.Euler().setFromQuaternion(ui.partObjects.get('tee').quaternion).z-Three.MathUtils.degToRad(17))<1e-7);
    ui.gizmo.dispatchEvent({type:'mouseUp'});await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    assert.equal(ui.state.doc.parts[1].pose.rotation_deg[2],17);
  }finally{await ui.close();}
});

function freeTube(){const doc=fixture();doc.parts=doc.parts.slice(0,1);doc.anchors=[];return doc;}
async function rotateTube(ui){
  ui.select('pipe');ui.document.querySelector('[data-tool="rotate"]').click();ui.gizmo.axis='X';
  ui.gizmo.dispatchEvent({type:'mouseDown'});
  ui.partObjects.get('pipe').quaternion.setFromAxisAngle(new Three.Vector3(1,0,0),Three.MathUtils.degToRad(88.8));
  ui.gizmo.dispatchEvent({type:'objectChange'});ui.gizmo.dispatchEvent({type:'mouseUp'});
  await ui.wait(()=>!ui.state.placementPending);
}

test('rotating a tube commits 90 degrees, keeps its length and undoes once',async()=>{
  const ui=await editor(freeTube());try{
    const before=structuredClone(ui.state.doc);await rotateTube(ui);
    assert.deepEqual(ui.state.doc.parts[0].pose.rotation_deg,[90,0,0]);
    assert.deepEqual(ui.state.doc.parts[0].pose.position_mm,[0,0,500]);
    assert.equal(ui.state.doc.parts[0].parameters.length_mm,1000);assert.equal(ui.state.undo.length,1);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.doc.parts[0].pose.rotation_deg===undefined);
    assert.deepEqual(ui.state.doc.parts,before.parts);
  }finally{await ui.close();}
});

test('an old server reports the required restart and a failed tube rotation preserves the design',async()=>{
  const ui=await editor(freeTube(),null,{bootstrapOverride:()=>({api_version:undefined}),interceptFetch:path=>path==='/api/move'?new Response(JSON.stringify({error:'Unknown operation'}),{status:404}):null});
  try{
    assert.match(ui.document.querySelector('#status-text').textContent,/server update required/);
    const before=structuredClone(ui.state.doc);await rotateTube(ui);
    assert.match(ui.document.querySelector('#toast').textContent,/server is out of date.*Save your work, restart/);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);
    assert.deepEqual(ui.partObjects.get('pipe').quaternion.toArray(),[0,0,0,1]);
  }finally{await ui.close();}
});

test('rotation recovers an expired server session without replacing unsaved work',async()=>{
  let bootstrapCalls=0,moveCalls=0;
  const ui=await editor(freeTube(),null,{bootstrapOverride:()=>{bootstrapCalls++;return {};},interceptFetch:(path,options)=>{
    if(path==='/api/move'&&++moveCalls===1)options.headers['X-PipeSim-Token']='expired-test-session';return null;
  }});
  try{
    ui.document.querySelector('#rename').click();ui.document.querySelector('#design-name').value='Unsaved frame';modalAction(ui,'Rename');
    await ui.wait(()=>ui.state.dirty&&!ui.document.querySelector('#modal').open);
    const camera=ui.camera.position.toArray();await rotateTube(ui);
    assert.equal(bootstrapCalls,2);assert.equal(moveCalls,2);
    assert.equal(ui.state.doc.name,'Unsaved frame');assert.deepEqual(ui.state.doc.parts[0].pose.rotation_deg,[90,0,0]);
    assert.equal(ui.state.undo.length,2);assert.equal(ui.state.dirty,true);assert.deepEqual(ui.camera.position.toArray(),camera);
  }finally{await ui.close();}
});

test('dropping from the toolbox connects and one undo removes the complete addition',async()=>{
  const ui=await editor(fixture());try{
    const point=ui.pixel(new Three.Vector3(0,0,500));
    const event=new ui.window.MouseEvent('drop',{clientX:point.x,clientY:point.y,bubbles:true,cancelable:true});
    Object.defineProperty(event,'dataTransfer',{value:{getData:()=> 'tubeclamp.TC101C'}});
    ui.document.querySelector('#viewport canvas').dispatchEvent(event);
    await ui.wait(()=>ui.state.doc.joints.length===1&&!ui.state.placementPending);
    assert.equal(ui.state.doc.parts.length,3);assert.equal(ui.state.undo.length,1);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.doc.parts.length===2);
    assert.equal(ui.state.doc.joints.length,0);
  }finally{await ui.close();}
});

test('Escape cancels direct dragging before any document change',async()=>{
  const ui=await editor(fixture());try{
    const from=ui.pixel(new Three.Vector3(300,0,500)),to=ui.pixel(new Three.Vector3(100,0,600));
    ui.pointer('pointerdown',from);ui.pointer('pointermove',to);
    ui.document.dispatchEvent(new ui.window.KeyboardEvent('keydown',{key:'Escape',bubbles:true}));
    assert.equal(ui.state.undo.length,0);assert.equal(ui.state.dirty,false);
    assert.deepEqual(ui.partObjects.get('tee').position.toArray(),[300,0,500]);
  }finally{await ui.close();}
});

const blankDesign=name=>({format:'pipesim/1',units:'mm-kg-s-N-deg',name,parts:[],joints:[]});
const modalAction=(ui,label)=>[...ui.document.querySelectorAll('#modal-actions button')].find(b=>b.textContent===label).click();
function fileInput(ui,file){
  const input=ui.document.querySelector('#design-file');Object.defineProperty(input,'files',{value:[file],configurable:true});
  input.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
}
function fileDrag(ui,type,files,target=ui.document.body){
  const event=new ui.window.MouseEvent(type,{bubbles:true,cancelable:true});
  Object.defineProperty(event,'dataTransfer',{value:{types:['Files'],files,getData:()=>''}});target.dispatchEvent(event);return event;
}

test('Load shows an empty folder and opens the OS file chooser',async()=>{
  const ui=await editor(fixture());try{
    ui.document.querySelector('#load-button').click();
    await ui.wait(()=>ui.document.querySelector('#saved-design-list')?.textContent.includes('No saved designs yet'));
    let chooserOpened=false;ui.document.querySelector('#design-file').addEventListener('click',event=>{event.preventDefault();chooserOpened=true;});
    ui.document.querySelector('#open-file-button').click();assert.equal(chooserOpened,true);
  }finally{await ui.close();}
});

test('Save then Load lists subfolders and replaces the scene and library cleanly',async()=>{
  const original=fixture();original.definitions={'custom.old':{name:'Old fitting',kind:'connector',mass_kg:1,geometry:[{type:'box',size_mm:[10,10,10]}]}};
  const ui=await editor(original);try{
    ui.document.querySelector('#save-button').click();ui.document.querySelector('#save-path').value='designs/frames/Saved cube.pipe.yaml';
    modalAction(ui,'Save file');await ui.wait(()=>ui.state.path==='designs/frames/Saved cube.pipe.yaml');
    const loaded={...fixture('TC104C'),name:'Cube from disk',definitions:{'custom.new':{name:'New fitting',kind:'connector',mass_kg:1,geometry:[{type:'box',size_mm:[20,20,20]}]}}};
    await post('save',loaded,{path:'designs/frames/Other cube.pipe.yaml'});
    ui.select('tee');ui.state.checks={valid:false};ui.state.mode='check';ui.state.undo=[structuredClone(original)];
    ui.document.querySelector('#load-button').click();await ui.wait(()=>ui.document.querySelectorAll('#saved-design-list [data-open]').length===2);
    const search=ui.document.querySelector('#design-search');search.value='Other cube';search.dispatchEvent(new ui.window.Event('input'));
    assert.equal(ui.document.querySelectorAll('#saved-design-list [data-open]').length,1);
    ui.document.querySelector('#saved-design-list [data-open]').click();await ui.wait(()=>ui.state.doc.name==='Cube from disk');
    assert.equal(ui.state.scene.parts[1].catalog,'tubeclamp.TC104C');assert.equal(ui.state.path,'designs/frames/Other cube.pipe.yaml');
    assert.ok(ui.state.library['custom.new']);assert.equal(ui.state.library['custom.old'],undefined);
    assert.equal(ui.state.selected,null);assert.equal(ui.state.checks,null);assert.equal(ui.state.mode,'design');
    assert.equal(ui.state.undo.length,0);assert.equal(ui.state.dirty,false);
  }finally{await ui.close();}
});

test('OS file picker loads YAML and Ctrl+O invokes the same chooser',async()=>{
  const ui=await editor(fixture());try{
    let chooserOpened=false;ui.document.querySelector('#design-file').addEventListener('click',event=>{event.preventDefault();chooserOpened=true;});
    const shortcut=new ui.window.KeyboardEvent('keydown',{key:'o',ctrlKey:true,bubbles:true,cancelable:true});ui.document.dispatchEvent(shortcut);
    assert.equal(shortcut.defaultPrevented,true);assert.equal(chooserOpened,true);
    fileInput(ui,new File(['\uFEFFformat: pipesim/1\nunits: mm-kg-s-N-deg\nname: Opened YAML\nparts: []'],'opened.pipe.yml'));
    await ui.wait(()=>ui.state.doc.name==='Opened YAML');
    assert.equal(ui.state.path,'designs/opened.pipe.yml');assert.equal(ui.state.dirty,true);assert.equal(ui.state.scene.parts.length,0);
    assert.equal(ui.document.querySelector('#modal').open,false);
    const files=await (await fetch(url+'/api/designs')).json();assert.ok(!files.designs.some(f=>f.path===ui.state.path));
  }finally{await ui.close();}
});

test('OS drop on the canvas opens JSON and suppresses browser navigation',async()=>{
  const ui=await editor(fixture());try{
    const file=new File([JSON.stringify(blankDesign('Dropped design'))],'drop.JSON');
    fileDrag(ui,'dragenter',[file]);assert.equal(ui.document.querySelector('#design-drop-overlay').classList.contains('hidden'),false);
    fileDrag(ui,'dragenter',[file],ui.document.querySelector('#viewport canvas'));fileDrag(ui,'dragleave',[file]);
    assert.equal(ui.document.querySelector('#design-drop-overlay').classList.contains('hidden'),false);
    const event=fileDrag(ui,'drop',[file],ui.document.querySelector('#viewport canvas'));
    assert.equal(event.defaultPrevented,true);assert.equal(ui.document.querySelector('#design-drop-overlay').classList.contains('hidden'),true);
    await ui.wait(()=>ui.state.doc.name==='Dropped design');assert.equal(ui.state.path,'designs/drop.JSON');
    assert.equal(ui.state.dirty,true);assert.equal(ui.state.undo.length,0);
  }finally{await ui.close();}
});

test('dirty design stays intact on cancel, then Save and open persists it first',async()=>{
  const ui=await editor(fixture());try{
    ui.document.querySelector('#rename').click();ui.document.querySelector('#design-name').value='Unsaved work';modalAction(ui,'Rename');
    await ui.wait(()=>ui.state.dirty&&!ui.document.querySelector('#modal').open);
    const before=structuredClone(ui.state.doc),history=ui.state.undo.length,file=new File([JSON.stringify(blankDesign('Next design'))],'next.json');
    fileInput(ui,file);await ui.wait(()=>ui.document.querySelector('#modal-title').textContent==='Save changes before opening?');
    modalAction(ui,'Cancel');assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.dirty,true);assert.equal(ui.state.undo.length,history);
    fileInput(ui,file);await ui.wait(()=>ui.document.querySelector('#modal-title').textContent==='Save changes before opening?');modalAction(ui,'Save and open');
    ui.document.querySelector('#save-path').value='designs/kept.pipe.yaml';modalAction(ui,'Save file');await ui.wait(()=>ui.state.doc.name==='Next design');
    const saved=await (await fetch(url+'/api/open?path=designs/kept.pipe.yaml')).json();
    assert.equal(saved.document.name,before.name);assert.deepEqual(saved.document.parts,before.parts);
    assert.deepEqual(saved.document.joints,before.joints);assert.deepEqual(saved.document.anchors,before.anchors);
    assert.equal(ui.state.path,'designs/next.json');assert.equal(ui.state.dirty,true);assert.equal(ui.state.undo.length,0);
  }finally{await ui.close();}
});

test('invalid files and failed saves keep the current design and undo history',async()=>{
  const ui=await editor(fixture());try{
    ui.state.dirty=true;ui.state.undo=[structuredClone(ui.state.doc)];const before=structuredClone(ui.state.doc),path=ui.state.path;
    fileInput(ui,new File(['parts: [broken'],'invalid.yaml'));await ui.wait(()=>ui.document.querySelector('#modal-title').textContent==='Could not open design');
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.path,path);assert.equal(ui.state.dirty,true);assert.equal(ui.state.undo.length,1);
    ui.document.querySelector('#modal-close').click();fileInput(ui,new File([JSON.stringify(blankDesign('Next'))],'next.json'));
    await ui.wait(()=>ui.document.querySelector('#modal-title').textContent==='Save changes before opening?');modalAction(ui,'Save and open');
    ui.document.querySelector('#save-path').value='../outside.yaml';modalAction(ui,'Save file');await ui.wait(()=>ui.document.querySelector('#save-error')?.textContent.includes('workspace'));
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.dirty,true);assert.equal(ui.state.path,path);
  }finally{await ui.close();}
});

test('Open without saving is explicit and multiple dropped files are rejected',async()=>{
  const ui=await editor(fixture());try{
    const file=new File([JSON.stringify(blankDesign('Discard selected'))],'discard.json');const before=structuredClone(ui.state.doc);
    fileDrag(ui,'drop',[file,file]);assert.match(ui.document.querySelector('#toast').textContent,/one design file/);assert.deepEqual(ui.state.doc,before);
    fileDrag(ui,'drop',[new File(['mesh'],'fitting.stl')]);assert.match(ui.document.querySelector('#toast').textContent,/Import a 3D part/);assert.deepEqual(ui.state.doc,before);
    ui.state.dirty=true;fileDrag(ui,'drop',[file]);await ui.wait(()=>ui.document.querySelector('#modal-title').textContent==='Save changes before opening?');
    modalAction(ui,'Open without saving');assert.equal(ui.state.doc.name,'Discard selected');assert.equal(ui.state.dirty,true);
  }finally{await ui.close();}
});

test('cancelling a slow load discards its late response',async()=>{
  let release,responseStarted=false;const hold=new Promise(resolve=>release=resolve);
  const ui=await editor(fixture(),null,{beforeResponse:async path=>{if(path==='/api/open-file'){responseStarted=true;await hold;}}});
  try{
    const before=structuredClone(ui.state.doc);fileInput(ui,new File([JSON.stringify(blankDesign('Late response'))],'late.json'));
    await ui.wait(()=>responseStarted);modalAction(ui,'Cancel');release();await pause(100);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.dirty,false);assert.equal(ui.document.querySelector('#modal').open,false);
  }finally{release();await ui.close();}
});

test('Save and open reloads the newly saved version of the same workspace file',async()=>{
  const ui=await editor(fixture());try{
    ui.document.querySelector('#save-button').click();ui.document.querySelector('#save-path').value='designs/reload.pipe.yaml';
    modalAction(ui,'Save file');await ui.wait(()=>ui.state.path==='designs/reload.pipe.yaml');
    ui.document.querySelector('#rename').click();ui.document.querySelector('#design-name').value='Newest version';modalAction(ui,'Rename');
    await ui.wait(()=>ui.state.dirty&&!ui.document.querySelector('#modal').open);
    ui.document.querySelector('#load-button').click();await ui.wait(()=>ui.document.querySelector('[data-open="designs/reload.pipe.yaml"]'));
    ui.document.querySelector('[data-open="designs/reload.pipe.yaml"]').click();await ui.wait(()=>ui.document.querySelector('#modal-title').textContent==='Save changes before opening?');
    modalAction(ui,'Save and open');modalAction(ui,'Save file');await ui.wait(()=>!ui.document.querySelector('#modal').open&&ui.state.undo.length===0);
    assert.equal(ui.state.doc.name,'Newest version');assert.equal(ui.state.path,'designs/reload.pipe.yaml');assert.equal(ui.state.dirty,false);
  }finally{await ui.close();}
});

test('cancelling Save and open while saving does not later switch the design',async()=>{
  let release,responseStarted=false;const hold=new Promise(resolve=>release=resolve);
  const ui=await editor(fixture(),null,{beforeResponse:async path=>{if(path==='/api/save'){responseStarted=true;await hold;}}});
  try{
    const before=structuredClone(ui.state.doc);ui.state.dirty=true;
    fileInput(ui,new File([JSON.stringify(blankDesign('Cancelled switch'))],'cancelled.json'));
    await ui.wait(()=>ui.document.querySelector('#modal-title').textContent==='Save changes before opening?');modalAction(ui,'Save and open');
    modalAction(ui,'Save file');await ui.wait(()=>responseStarted);modalAction(ui,'Cancel');release();await ui.wait(()=>!ui.state.dirty);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.path,'output/ui-test.pipe.yaml');assert.equal(ui.document.querySelector('#modal').open,false);
  }finally{release();await ui.close();}
});

test('human inspector edits selective posture controls and preserves undo',async()=>{
  const doc=blankDesign('Selective muscles');doc.objects=[{id:'person',template:'human',parameters:{pose:'pull-up',hold_joints:['upper_body'],strength_scale:6,grip_diameter_mm:42.4}}];
  const ui=await editor(doc);try{
    ui.select('person/pelvis');assert.equal(ui.document.querySelector('#object-hold').value,'upper_body');
    assert.equal(ui.document.querySelector('#object-pose').value,'pull-up');assert.equal(ui.document.querySelector('#object-grip').value,'42.4');
    const set=async(mode,count)=>{
      const select=ui.document.querySelector('#object-hold');select.value=mode;select.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
      await ui.wait(()=>ui.state.scene.joints.filter(j=>j.motor).length===count);
    };
    await set('arms',8);assert.deepEqual(ui.state.doc.objects[0].parameters.hold_joints,['arms']);
    await set('all',18);assert.equal(ui.state.doc.objects[0].parameters.hold_pose,true);assert.equal(ui.state.doc.objects[0].parameters.hold_joints,undefined);
    await set('relaxed',0);assert.equal(ui.state.doc.objects[0].parameters.hold_pose,false);
    const select=ui.document.querySelector('#object-hold');select.value='custom';select.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
    assert.equal(ui.document.querySelector('#object-held-joints-field').hidden,false);
    const joints=ui.document.querySelector('#object-held-joints');joints.value='left_elbow, right_elbow';joints.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
    await ui.wait(()=>ui.state.scene.joints.filter(j=>j.motor).length===2);
    assert.equal(ui.document.querySelector('#object-hold').value,'custom');assert.deepEqual(ui.state.doc.objects[0].parameters.hold_joints,['left_elbow','right_elbow']);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.joints.every(j=>!j.motor));
    assert.equal(ui.document.querySelector('#object-hold').value,'relaxed');
  }finally{await ui.close();}
});

test('add human dialog creates a pull-up pose with gripping hands and free legs',async()=>{
  const ui=await editor(blankDesign('Human creation'));try{
    ui.document.querySelector('#human-button').click();ui.document.querySelector('#human-pose').value='pull-up';
    ui.document.querySelector('#human-hold').value='upper_body';ui.document.querySelector('#human-strength').value='6';ui.document.querySelector('#human-grip').value='42.4';
    modalAction(ui,'Add human');await ui.wait(()=>ui.state.scene.parts.length===19);
    const parameters=ui.state.doc.objects[0].parameters;
    assert.equal(parameters.pose,'pull-up');assert.equal(parameters.grip_diameter_mm,42.4);assert.deepEqual(parameters.hold_joints,['upper_body']);
    assert.equal(ui.state.scene.joints.filter(j=>j.motor).length,12);assert.equal(ui.state.scene.joints.filter(j=>j.id.includes('_hip')&&j.motor).length,0);
    assert.equal(ui.state.undo.length,1);
  }finally{await ui.close();}
});

test('pull-up example runs physics and plays recorded independent leg motion in the editor',async()=>{
  const opened=await post('parse',blankDesign('Read example'),{text:await readFile('examples/human-pull-up.pipe.yaml','utf8')});
  const ui=await editor(opened.document);try{
    assert.equal(ui.state.scene.parts.length,59);
    ui.document.querySelector('[data-mode="simulate"]').click();assert.equal(ui.document.querySelector('#sim-duration').value,'4');
    ui.document.querySelector('[data-run="simulate"]').click();await ui.wait(()=>ui.state.recording&&!ui.state.busy,60000);
    const recording=ui.state.recording;assert.equal(recording.frames.length,121);assert.equal(ui.state.doc.results.simulate,recording);
    assert.equal(ui.state.dirty,true);assert.equal(recording.events.length,0);
    const initial=recording.frames[0].parts,last=recording.frames.at(-1).parts;
    assert.ok(new Three.Vector3(...initial['person/left_hand'].position_mm).distanceTo(new Three.Vector3(...last['person/left_hand'].position_mm))<2);
    assert.ok(new Three.Vector3(...initial['person/left_foot'].position_mm).distanceTo(new Three.Vector3(...last['person/left_foot'].position_mm))>40);
    const slider=ui.document.querySelector('#time-slider');slider.value=recording.frames.length-1;slider.dispatchEvent(new ui.window.Event('input',{bubbles:true}));
    assert.equal(ui.state.frame,recording.frames.length-1);
    assert.ok(ui.partObjects.get('person/left_foot').position.distanceTo(new Three.Vector3(...last['person/left_foot'].position_mm))<.001);
  }finally{await ui.close();}
});

test('Duplicate copies the selected part independently and supports undo and redo',async()=>{
  const doc=fixture();Object.assign(doc.parts[0],{label:'Custom upright',color:'#305f80',parameters:{length_mm:1234,wall_mm:3.2},pose:{position_mm:[50,70,700],rotation_deg:[12,34,56]}});
  doc.joints=[{id:'original-connection',type:'fixed',a:{part:'pipe'},b:{part:'tee'}}];
  const ui=await editor(doc);try{
    ui.select('pipe');const original=structuredClone(ui.state.doc.parts[0]);
    ui.document.querySelector('#duplicate-selected').click();await ui.wait(()=>ui.state.selected==='pipe-copy'&&!ui.state.placementPending);
    const copy=structuredClone(ui.state.doc.parts.find(p=>p.id==='pipe-copy'));
    assert.equal(ui.state.doc.parts.length,3);assert.deepEqual(ui.state.doc.parts[0],original);
    assert.deepEqual(copy.parameters,original.parameters);assert.equal(copy.color,original.color);assert.equal(copy.label,original.label);
    assert.deepEqual(copy.pose.rotation_deg,original.pose.rotation_deg);assert.deepEqual(copy.pose.position_mm.slice(1),original.pose.position_mm.slice(1));
    assert.ok(copy.pose.position_mm[0]>original.pose.position_mm[0]+100);
    assert.equal(ui.state.doc.joints.length,1);assert.equal(ui.state.doc.anchors.length,1);assert.equal(ui.state.undo.length,1);assert.equal(ui.state.tool,'translate');
    const length=ui.document.querySelector('[data-param="length_mm"]');length.value='1500';length.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
    await ui.wait(()=>ui.state.scene.parts.find(p=>p.id==='pipe-copy').length_mm===1500);assert.equal(ui.state.doc.parts[0].parameters.length_mm,1234);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.parts.find(p=>p.id==='pipe-copy').length_mm===1234);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.parts.length===2);
    ui.document.querySelector('#redo').click();await ui.wait(()=>ui.state.scene.parts.length===3);
    assert.deepEqual(ui.state.doc.parts.find(p=>p.id==='pipe-copy'),copy);
  }finally{await ui.close();}
});

test('Duplicate preserves inline body properties and avoids existing object namespaces',async()=>{
  const doc=blankDesign('Inline bodies');doc.parts=[{id:'load',body:{kind:'rigid',mass_kg:5,color:'#805030',geometry:[{type:'box',size_mm:[120,60,40]}]},pose:{position_mm:[0,0,20]}}];
  doc.objects=[{id:'load-copy',template:'human',pose:{position_mm:[2000,0,0]}}];
  const ui=await editor(doc);try{
    ui.select('load');ui.document.querySelector('#duplicate-selected').click();await ui.wait(()=>ui.state.selected==='load-copy-2'&&!ui.state.placementPending);
    const copy=ui.state.doc.parts.find(p=>p.id==='load-copy-2');assert.deepEqual(copy.body,doc.parts[0].body);assert.notEqual(copy.body,ui.state.doc.parts[0].body);
    ui.document.querySelector('#duplicate-selected').click();await ui.wait(()=>ui.state.selected==='load-copy-3'&&!ui.state.placementPending);
    assert.equal(ui.state.doc.objects.length,1);assert.equal(ui.state.doc.parts.length,3);
    assert.ok(ui.state.doc.parts[2].pose.position_mm[0]>copy.pose.position_mm[0]);
    const reopened=await post('parse',blankDesign('Reopen'),{text:JSON.stringify(ui.state.doc)});assert.equal(reopened.scene.parts.length,22);
  }finally{await ui.close();}
});

test('Duplicate subassembly copies a whole human and its joint state without external attachments',async()=>{
  const doc=fixture();doc.objects=[{id:'person',template:'human',parameters:{hold_joints:['arms'],stature_mm:1800},pose:{position_mm:[900,300,0],rotation_deg:[0,0,35]}}];
  doc.state={joints:{'person/right_elbow':{angle_deg:30}}};
  doc.animation={tracks:[{joint:'person/right_elbow',coordinate:'angle_deg',keyframes:[{time_s:0,value:30},{time_s:1,value:60}]}]};
  doc.drives=[{id:'exercise',type:'gear',driver:'person/right_elbow',follower:'person/left_elbow',ratio:1,route_mm:[[900,300,500],[1200,300,500]]}];
  doc.joints=[{id:'grip',type:'spherical',a:{part:'tee'},b:{part:'person/left_hand',port:'grip'}}];
  const ui=await editor(doc);try{
    ui.select('person/right_hand');const original=structuredClone(ui.state.doc.objects[0]),poses=new Map(ui.state.scene.parts.filter(p=>p.id.startsWith('person/')).map(p=>[p.id,p.pose]));
    ui.document.querySelector('#duplicate-menu-toggle').click();ui.document.querySelector('[data-duplicate="subassembly"]').click();await ui.wait(()=>ui.state.selected==='person-copy/right_hand'&&!ui.state.placementPending);
    assert.equal(ui.state.doc.objects.length,2);assert.equal(ui.state.scene.parts.length,40);assert.deepEqual(ui.state.doc.objects[0],original);
    const copy=ui.state.doc.objects[1],offset=copy.pose.position_mm[0]-original.pose.position_mm[0];assert.deepEqual(copy.parameters,original.parameters);
    assert.equal(ui.state.scene.joints.filter(j=>j.id.startsWith('person-copy/')).length,18);
    assert.equal(ui.state.scene.joints.filter(j=>j.id.startsWith('person-copy/')&&j.motor).length,8);
    assert.equal(ui.state.doc.joints.length,1);assert.deepEqual(ui.state.doc.state.joints['person-copy/right_elbow'],{angle_deg:30});
    assert.equal(ui.state.doc.animation.tracks.length,2);assert.equal(ui.state.doc.animation.tracks[1].joint,'person-copy/right_elbow');
    assert.deepEqual(ui.state.doc.animation.tracks[1].keyframes,doc.animation.tracks[0].keyframes);
    assert.equal(ui.state.doc.drives[1].driver,'person-copy/right_elbow');assert.equal(ui.state.doc.drives[1].follower,'person-copy/left_elbow');
    assert.deepEqual(ui.state.doc.drives[1].route_mm,[[900+offset,300,500],[1200+offset,300,500]]);
    for(const [id,pose] of poses){
      const copied=ui.state.scene.parts.find(p=>p.id===id.replace('person/','person-copy/')).pose;
      assert.ok(new Three.Vector3(...copied.position_mm).distanceTo(new Three.Vector3(...pose.position_mm).add(new Three.Vector3(offset,0,0)))<.001);
      assert.deepEqual(copied.rotation_deg,pose.rotation_deg);
    }
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.doc.objects.length===1&&ui.state.scene.parts.length===21);
    assert.equal(ui.state.doc.state.joints['person-copy/right_elbow'],undefined);
  }finally{await ui.close();}
});

test('failed duplication restores the original document and selection',async()=>{
  let rejectNext=false;
  const ui=await editor(fixture(),null,{interceptFetch:path=>{if(path==='/api/duplicate'&&rejectNext){rejectNext=false;return new Response(JSON.stringify({error:'Could not resolve the copy'}),{status:400});}}});
  try{
    ui.select('pipe');const before=structuredClone(ui.state.doc);rejectNext=true;
    ui.document.querySelector('#duplicate-selected').click();await ui.wait(()=>!ui.state.placementPending);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.selected,'pipe');assert.equal(ui.state.undo.length,0);
    assert.equal(ui.document.querySelector('#duplicate-selected').disabled,false);assert.match(ui.document.querySelector('#toast').textContent,/Could not resolve the copy/);
  }finally{await ui.close();}
});

async function resizeFixture(path){return (await post('parse',blankDesign('Resize fixture'),{text:await readFile(path,'utf8')})).document;}
function changeLength(ui,length){
  const field=ui.document.querySelector('[data-param="length_mm"]');field.value=String(length);field.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
}

test('pipe length edits move attached branches and undo together',async()=>{
  const doc=await resizeFixture('examples/tee-midpoint.pipe.yaml');doc.parts[0].parameters.wall_mm=3.2;
  const ui=await editor(doc);try{
    ui.select('spine');const before=structuredClone(ui.state.doc);changeLength(ui,1200);
    await ui.wait(()=>!ui.state.placementPending);
    assert.equal(ui.state.doc.parts[0].parameters.length_mm,1200);assert.equal(ui.state.doc.parts[0].parameters.wall_mm,3.2);
    assert.equal(ui.state.selected,'spine');assert.equal(ui.state.undo.length,1);
    for(const id of ['upper-tee','upper-crossbar']){
      const old=before.parts.find(p=>p.id===id),now=ui.state.scene.parts.find(p=>p.id===id);
      assert.ok(Math.abs(now.pose.position_mm[2]-old.pose.position_mm[2]-100)<1e-5);
    }
    assert.deepEqual(ui.state.doc.joints,before.joints);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.parts.find(p=>p.id==='spine').length_mm===1000);
    assert.deepEqual(ui.state.doc,{...before,results:{}});
    ui.document.querySelector('#redo').click();await ui.wait(()=>ui.state.scene.parts.find(p=>p.id==='spine').length_mm===1200);
    assert.equal((await post('validate',ui.state.doc)).valid,true);
  }finally{await ui.close();}
});

test('blocked square length explains sockets, preserves edits, and navigates to the constraint',async()=>{
  const ui=await editor(await resizeFixture('tests/fixtures/resize-square.pipe.yaml'));try{
    ui.select('bottom');const before=structuredClone(ui.state.doc);changeLength(ui,900);
    await ui.wait(()=>ui.document.querySelector('#modal-title').textContent==='Length change blocked');
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);assert.equal(ui.state.dirty,false);
    assert.equal(ui.document.querySelector('[data-param="length_mm"]').value,'1000');
    assert.match(ui.document.querySelector('#modal-content').textContent,/closed path/);
    assert.match(ui.document.querySelector('[data-resize-release]').textContent,/Disconnect and resize/);
    modalAction(ui,'Keep current length');assert.deepEqual(ui.state.doc,before);
    changeLength(ui,900);await ui.wait(()=>ui.document.querySelector('#modal').open&&!ui.state.placementPending);
    ui.document.querySelector('#modal-content details').open=true;
    const show=ui.document.querySelector('[data-resize-show]'),card=show.closest('.joint-card');
    const joint=card.querySelector('.joint-card-top').textContent;
    show.click();assert.equal(ui.state.selected,before.joints.find(j=>j.id===joint).a.part);
    assert.ok(ui.document.querySelector(`[data-joint-edit="${joint}"]`));assert.equal(ui.state.undo.length,0);
  }finally{await ui.close();}
});

test('an explicit loosen-and-resize keeps every socket engaged and is one undoable edit',async()=>{
  const ui=await editor(await resizeFixture('tests/fixtures/resize-square.pipe.yaml'));try{
    ui.select('bottom');const before=structuredClone(ui.state.doc);changeLength(ui,995);
    await ui.wait(()=>ui.document.querySelector('[data-resize-release]'));
    const release=ui.document.querySelector('[data-resize-release]');assert.equal(release.textContent,'Loosen and resize');
    release.click();await ui.wait(()=>!ui.state.placementPending&&ui.state.doc.parts.find(p=>p.id==='bottom').parameters.length_mm===995);
    assert.equal(ui.state.doc.joints.length,8);assert.equal(ui.state.doc.joints.filter(j=>!j.locked).length,1);
    assert.equal(ui.state.undo.length,1);assert.equal(ui.state.selected,'bottom');assert.equal(ui.document.querySelector('#modal').open,false);
    assert.equal((await post('validate',ui.state.doc)).valid,true);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.parts.find(p=>p.id==='bottom').length_mm===1000);
    assert.deepEqual(ui.state.doc,{...before,results:{}});
  }finally{await ui.close();}
});

test('a large resize disconnects only the socket explicitly chosen in the dialog',async()=>{
  const ui=await editor(await resizeFixture('tests/fixtures/resize-square.pipe.yaml'));try{
    ui.select('bottom');changeLength(ui,900);await ui.wait(()=>ui.document.querySelector('[data-resize-release]'));
    ui.document.querySelector('[data-resize-release]').click();
    await ui.wait(()=>!ui.state.placementPending&&ui.state.doc.parts.find(p=>p.id==='bottom').parameters.length_mm===900);
    assert.equal(ui.state.doc.joints.length,7);assert.ok(ui.state.doc.joints.every(j=>j.locked));assert.equal(ui.state.undo.length,1);
    assert.equal((await post('validate',ui.state.doc)).valid,true);
  }finally{await ui.close();}
});

test('a failed length request restores the input without changing the document or undo history',async()=>{
  const ui=await editor(freeTube(),null,{interceptFetch:path=>path==='/api/resize'?new Response(JSON.stringify({error:'Resize failed'}),{status:400}):null});
  try{
    ui.select('pipe');const before=structuredClone(ui.state.doc);changeLength(ui,1400);await ui.wait(()=>!ui.state.placementPending);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);assert.equal(ui.state.dirty,false);
    assert.equal(ui.document.querySelector('[data-param="length_mm"]').value,'1000');assert.equal(ui.document.querySelector('[data-param="length_mm"]').disabled,false);
    assert.match(ui.document.querySelector('#toast').textContent,/Resize failed/);
  }finally{await ui.close();}
});

async function openSlidingConnection(ui){
  ui.select('bottom-arm');ui.document.querySelector('#connect-selected').click();ui.select('elbow');
  await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
}

test('connecting two sliding tees previews both arms and commits their motion as one edit',async()=>{
  const ui=await editor(await resizeFixture('tests/fixtures/sliding-corner.pipe.yaml'));try{
    const before=structuredClone(ui.state.doc);await openSlidingConnection(ui);
    assert.equal(ui.document.querySelector('#snap-move').value,'both');
    assert.match(ui.document.querySelector('#snap-review-status').textContent,/2 sliding joints/);
    assert.ok(ui.document.querySelector('#snap-preview-view canvas[data-rendered]'));
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);
    ui.document.querySelector('#modal-actions .primary').click();
    await ui.wait(()=>ui.state.doc.joints.length===before.joints.length+1&&!ui.state.placementPending);
    assert.equal(ui.state.undo.length,1);
    assert.deepEqual(ui.state.doc.parts.find(p=>p.id==='elbow').pose.position_mm,[466.7,466.7,500]);
    for(const id of ['left','bottom','right','top',...Array.from({length:4},(_,i)=>'corner-'+i)]){
      assert.deepEqual(ui.state.doc.parts.find(p=>p.id===id),before.parts.find(p=>p.id===id));
    }
    for(const id of ['left-slide','bottom-slide'])assert.equal(ui.state.doc.joints.find(j=>j.id===id).locked,false);
    assert.equal((await post('validate',ui.state.doc)).valid,true);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.joints.length===before.joints.length);
    assert.deepEqual(ui.state.doc,{...before,results:{}});
    ui.document.querySelector('#redo').click();await ui.wait(()=>ui.state.scene.joints.length===before.joints.length+1);
    assert.deepEqual(ui.state.doc.parts.find(p=>p.id==='elbow').pose.position_mm,[466.7,466.7,500]);
  }finally{await ui.close();}
});

test('cancelling coordinated sliding leaves every tee and screw in its original state',async()=>{
  const ui=await editor(await resizeFixture('tests/fixtures/sliding-corner.pipe.yaml'));try{
    const before=structuredClone(ui.state.doc);await openSlidingConnection(ui);modalAction(ui,'Cancel');
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.dirty,false);assert.equal(ui.state.undo.length,0);
    for(const p of before.parts)assert.ok(ui.partObjects.get(p.id).position.distanceTo(new Three.Vector3(...p.pose.position_mm))<1e-7);
  }finally{await ui.close();}
});

test('a sliding connection with insufficient travel keeps Connect disabled and identifies the limit',async()=>{
  const doc=await resizeFixture('tests/fixtures/sliding-corner.pipe.yaml');doc.joints.find(j=>j.id==='bottom-slide').limits={slide_mm:[-50,50]};
  const ui=await editor(doc);try{
    const before=structuredClone(ui.state.doc);ui.select('bottom-arm');ui.document.querySelector('#connect-selected').click();ui.select('elbow');
    await ui.wait(()=>ui.document.querySelector('#snap-force')?.disabled===false,20000);
    assert.match(ui.document.querySelector('#snap-review-status').textContent,/travel/i);
    assert.match(ui.document.querySelector('#snap-review-status').textContent,/bottom-slide/);
    assert.equal(ui.document.querySelector('#modal-actions .primary').disabled,true);assert.deepEqual(ui.state.doc,before);
    assert.equal(ui.state.undo.length,0);modalAction(ui,'Cancel');
  }finally{await ui.close();}
});

test('dragging towards a joint asks for a preview when both sliding arms need to move',async()=>{
  const ui=await editor(await resizeFixture('tests/fixtures/sliding-corner.pipe.yaml'));try{
    const before=structuredClone(ui.state.doc);
    ui.drag(ui.pixel(new Three.Vector3(466.7,450,510)),ui.pixel(new Three.Vector3(600,466.7,510)));
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    assert.equal(ui.document.querySelector('#snap-move').value,'both');assert.deepEqual(ui.state.doc,before);
    assert.equal(ui.state.undo.length,0);modalAction(ui,'Cancel');assert.deepEqual(ui.state.doc,before);
  }finally{await ui.close();}
});

function seatedHuman(){return {...blankDesign('Pose a human'),objects:[{id:'person',template:'human',parameters:{pose:'seated'}}]};}

test('an expanded human regroups, keeps its edited parts, and supports undo and redo',async()=>{
  const expanded=await post('expand',seatedHuman(),{object:'person'});
  expanded.parts[0].body.mass_kg=20;
  const ui=await editor(expanded);try{
    ui.select('person/left_hand');assert.ok(ui.document.querySelector('#regroup-object'));
    const before=structuredClone(ui.state.scene.parts);ui.document.querySelector('#regroup-object').click();
    await ui.wait(()=>ui.state.doc.objects?.length===1&&!ui.state.placementPending);
    assert.equal(ui.state.doc.parts.length,0);assert.equal(ui.state.undo.length,1);
    assert.equal(ui.state.scene.joints.length,18);
    for(const p of before){const q=ui.state.scene.parts.find(v=>v.id===p.id);assert.deepEqual(q.geometry,p.geometry);assert.equal(q.mass_kg,p.mass_kg);}
    assert.equal(ui.document.querySelector('[data-object-mode="whole"]').getAttribute('aria-pressed'),'true');
    assert.equal(ui.document.querySelectorAll('[data-object-select="person"]').length,1);
    assert.ok(ui.document.querySelector('#object-edited-pose'));assert.equal(ui.document.querySelector('#object-pose'),null);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.doc.parts.length===19&&ui.document.querySelector('#regroup-object'));
    assert.ok(ui.document.querySelector('#regroup-object'));
    ui.document.querySelector('#redo').click();await ui.wait(()=>ui.state.doc.parts.length===0&&ui.document.querySelector('#object-edited-pose'));
    assert.equal(ui.state.doc.objects[0].components.parts[0].body.mass_kg,20);
  }finally{await ui.close();}
});

async function triangleDesign(){
  return (await post('parse',blankDesign('Triangle'),{text:await readFile('examples/hinged-triangle.pipe.yaml','utf8')})).document;
}

test('threading the final pipe solves a hinged triangle despite 15 degree rotation snaps',async()=>{
  const doc=await triangleDesign(),ui=await editor(doc,JSON.stringify({...settings.DEFAULT_SNAP_SETTINGS,rotationDeg:15}));
  try{
    const frame=JSON.parse(JSON.stringify(doc.parts.filter(p=>['bottom','right','top','left'].includes(p.id)||p.id.startsWith('corner-'))));
    for(const connector of ['upper-back','upper-left']){
      ui.select('top-bar');ui.document.querySelector('#connect-selected').click();ui.select(connector);
      await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false,20000);
      assert.equal(ui.document.querySelector('#snap-move').value,'fit');assert.ok(ui.document.querySelector('#snap-preview-view canvas[data-rendered]'));
      assert.match(ui.document.querySelector('#snap-review-status').textContent,/joints adjusted continuously/);
      assert.equal(ui.document.querySelector('#snap-force').disabled,false);
      const lock=ui.document.querySelector('#snap-lock');lock.checked=false;lock.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
      await ui.wait(()=>!ui.document.querySelector('#snap-force').disabled&&ui.document.querySelector('#modal-actions .primary').disabled===false,20000);
      ui.document.querySelector('#modal-actions .primary').click();await ui.wait(()=>!ui.state.placementPending&&!ui.document.querySelector('#modal').open);
    }
    assert.equal(ui.state.doc.joints.length,doc.joints.length+2);assert.equal(ui.state.undo.length,2);assert.equal(ui.state.snapSettings.rotationDeg,15);
    for(const part of frame)assert.deepEqual(ui.state.doc.parts.find(p=>p.id===part.id),part);
    assert.ok(ui.state.doc.parts.filter(p=>p.id.startsWith('leg-')).some(p=>p.pose.rotation_deg.some(a=>Math.abs(a/15-Math.round(a/15))>.01)));
    const saved=structuredClone(ui.state.doc);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.joints.length===doc.joints.length+1);
    ui.document.querySelector('#redo').click();await ui.wait(()=>ui.state.scene.joints.length===doc.joints.length+2);
    assert.deepEqual(ui.state.doc.parts,saved.parts);assert.deepEqual(ui.state.doc.joints,saved.joints);
  }finally{await ui.close();}
});

test('Force searches a blocked connection, previews the adjusted station, then connects in one edit',async()=>{
  const base=await triangleDesign();const first=await post('snap-options',base,{member:'top-bar',connector:'upper-back',port:'through',at_mm:800,locked:false});
  const doc=first.options.find(o=>o.move===first.recommended).document;
  for(const j of doc.joints)if(j.type==='socket'&&!j.locked)j.limits={slide_mm:[-.00001,.00001]};
  const requests=[];const ui=await editor(doc,null,{interceptFetch:(path,options)=>{if(path==='/api/snap-options')requests.push(JSON.parse(options.body));return null;}});
  try{
    ui.select('top-bar');ui.document.querySelector('#connect-selected').click();ui.select('upper-left');
    await ui.wait(()=>ui.document.querySelector('#snap-force')?.disabled===false,20000);
    assert.equal(ui.document.querySelector('#modal-actions .primary').disabled,true);
    const original=structuredClone(ui.state.doc);ui.document.querySelector('#snap-force').click();
    assert.equal(ui.document.querySelector('#snap-force').disabled,true);
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary').disabled===false,30000);
    assert.equal(requests.at(-1).force,true);assert.equal(requests[0].force,false);
    assert.deepEqual(ui.state.doc,original);assert.equal(ui.state.undo.length,0);
    assert.notEqual(Number(ui.document.querySelector('#snap-station').value),500);
    assert.match(ui.document.querySelector('#snap-review-status').textContent,/Fitted socket centre/);
    const station=Number(ui.document.querySelector('#snap-station').value);
    ui.document.querySelector('#modal-actions .primary').click();await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    assert.equal(ui.state.doc.joints.length,doc.joints.length+1);
    assert.ok(Math.abs(ui.state.doc.joints.at(-1).b.at_mm-station)<.0001);
  }finally{await ui.close();}
});

test('cancelling Force discards a late search result without a new connection or moved parts',async()=>{
  let started=false,release;const hold=new Promise(resolve=>release=resolve);
  const ui=await editor(await triangleDesign(),null,{beforeResponse:async path=>{if(path==='/api/snap-options'&&started)await hold;}});
  try{
    ui.select('top-bar');ui.document.querySelector('#connect-selected').click();ui.select('upper-back');
    await ui.wait(()=>ui.document.querySelector('#snap-force')?.disabled===false,20000);
    const original=structuredClone(ui.state.doc);started=true;ui.document.querySelector('#snap-force').click();
    ui.document.querySelector('#modal-close').click();release();await pause(3000);
    assert.deepEqual(ui.state.doc,original);assert.equal(ui.state.undo.length,0);assert.equal(ui.state.placementPending,false);assert.equal(ui.document.querySelector('#modal').open,false);
  }finally{release();await ui.close();}
});

function chainDesign(count=25){const doc=blankDesign('Grouped chain');doc.objects=[{id:'chain',template:'chain',parameters:{length_mm:count*20},pose:{position_mm:[0,0,count*20+200]}}];return doc;}

async function adjustmentDesign(){return (await post('parse',blankDesign('Fit allowance'),{text:await readFile('tests/fixtures/assembly-adjustment.pipe.yaml','utf8')})).document;}

test('connection tolerance admits a one millimetre residual and remains editable',async()=>{
  const ui=await editor(await adjustmentDesign());try{
    ui.select('bottom-arm');ui.document.querySelector('#connect-selected').click();ui.select('elbow');
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    assert.equal(ui.document.querySelector('#snap-tolerance').value,'2');
    assert.match(ui.document.querySelector('#snap-review-status').textContent,/Accepted connection gap: 1.00 mm/);
    changeInput(ui,'#snap-tolerance',.03);await ui.wait(()=>!ui.document.querySelector('#snap-force').disabled);
    assert.equal(ui.document.querySelector('#modal-actions .primary').disabled,true);
    changeInput(ui,'#snap-tolerance',2);await ui.wait(()=>!ui.document.querySelector('#modal-actions .primary').disabled);
    modalAction(ui,'Connect');await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    assert.equal(ui.state.doc.joints.at(-1).fit_tolerance_mm,2);
  }finally{await ui.close();}
});

for(const adjustment of ['unlock','resize'])test(`Force previews bounded ${adjustment} changes and applies them with one Undo`,async()=>{
  const doc=await adjustmentDesign(),requests=[];
  const ui=await editor(doc,null,{interceptFetch:(path,options)=>{if(path==='/api/snap-options')requests.push(JSON.parse(options.body));return null;}});
  try{
    ui.select('bottom-arm');ui.document.querySelector('#connect-selected').click();ui.select('elbow');
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    ui.document.querySelector('#snap-force-options summary').click();
    changeInput(ui,adjustment==='unlock'?'#snap-unlock-count':'#snap-resize-count',1);
    await ui.wait(()=>!ui.document.querySelector('#snap-force').disabled);
    changeInput(ui,'#snap-resize-mm',2);await ui.wait(()=>!ui.document.querySelector('#snap-force').disabled);
    const before=structuredClone(ui.state.doc);ui.document.querySelector('#snap-force').click();
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary').disabled===false,30000);
    assert.equal(requests.at(-1).force,true);assert.equal(requests.at(-1).force_options.max_length_change_mm,2);
    const changes=ui.document.querySelector('#snap-adjustments').textContent;
    assert.match(changes,adjustment==='unlock'?/Loosen, fit and retighten/:/401.00 → 400.00 mm/);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);
    modalAction(ui,'Connect');await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    assert.equal(ui.state.doc.joints.length,before.joints.length+1);
    for(const j of before.joints)assert.equal(ui.state.doc.joints.find(k=>k.id===j.id).locked,j.locked);
    if(adjustment==='resize')assert.ok(Math.abs(ui.state.scene.parts.find(p=>p.id==='left-arm').length_mm-400)<.03);
    const accepted=structuredClone(ui.state.doc);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.joints.length===before.joints.length);
    assert.deepEqual({...ui.state.doc,results:{}},{...before,results:{}});
    ui.document.querySelector('#redo').click();await ui.wait(()=>ui.state.scene.joints.length===accepted.joints.length);
    assert.deepEqual({...ui.state.doc,results:{}},{...accepted,results:{}});
  }finally{await ui.close();}
});

test('invalid Force limits block submission and a corrected value permits another search',async()=>{
  const requests=[];const ui=await editor(await adjustmentDesign(),null,{interceptFetch:(path,options)=>{if(path==='/api/snap-options')requests.push(JSON.parse(options.body));return null;}});
  try{
    ui.select('bottom-arm');ui.document.querySelector('#connect-selected').click();ui.select('elbow');
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    const count=requests.length;changeInput(ui,'#snap-resize-count',-1);
    ui.document.querySelector('#snap-force').click();
    assert.equal(requests.length,count);assert.equal(ui.document.querySelector('#modal-actions .primary').disabled,true);
    assert.equal(ui.document.querySelector('#snap-force').disabled,false);
    changeInput(ui,'#snap-resize-count',1);await ui.wait(()=>!ui.document.querySelector('#snap-force').disabled);
    assert.equal(requests.length,count+1);
    modalAction(ui,'Cancel');assert.equal(ui.state.undo.length,0);
  }finally{await ui.close();}
});

test('cancelling a pending Force resize preserves cut lengths, screws and Undo history',async()=>{
  let started=false,release;const hold=new Promise(resolve=>release=resolve);
  const ui=await editor(await adjustmentDesign(),null,{beforeResponse:async path=>{if(path==='/api/snap-options'&&started)await hold;}});
  try{
    ui.select('bottom-arm');ui.document.querySelector('#connect-selected').click();ui.select('elbow');
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    changeInput(ui,'#snap-resize-count',1);await ui.wait(()=>!ui.document.querySelector('#snap-force').disabled);
    const before=structuredClone(ui.state.doc);started=true;ui.document.querySelector('#snap-force').click();
    modalAction(ui,'Cancel');release();await ui.drain();
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);
    assert.equal(ui.document.querySelector('#modal').open,false);
    assert.equal(ui.state.scene.parts.find(p=>p.id==='left-arm').length_mm,401);
  }finally{release();await ui.close();}
});

function changeInput(ui,selector,value){const input=ui.document.querySelector(selector);input.value=String(value);input.dispatchEvent(new ui.window.Event('change',{bubbles:true}));}
function selectChain(ui,end='end'){ui.document.querySelector('[data-object-select="chain"]').click();ui.document.querySelector('#chain-select-'+end).click();}

test('Add chain creates a length-controlled object with a compact tree and one-step undo',async()=>{
  const ui=await editor(blankDesign('New chain'));try{
    ui.document.querySelector('#chain-button').click();assert.equal(ui.document.querySelector('#modal-title').textContent,'Add a chain');
    modalAction(ui,'Cancel');assert.equal(ui.state.undo.length,0);
    ui.document.querySelector('#chain-button').click();changeInput(ui,'#new-chain-length',503);modalAction(ui,'Add chain');
    await ui.wait(()=>ui.state.scene.parts.length===26&&!ui.document.querySelector('#modal').open);
    assert.equal(ui.state.doc.objects.length,1);assert.equal(ui.state.doc.objects[0].template,'chain');assert.equal(ui.state.doc.parts.length,0);assert.equal(ui.state.doc.joints.length,0);
    assert.equal(ui.state.scene.joints.length,25);assert.equal(ui.state.scene.groups.length,1);assert.equal(ui.state.undo.length,1);
    assert.equal(ui.document.querySelectorAll('[data-object-select]').length,1);assert.equal(ui.document.querySelectorAll('#outline-list [data-select]').length,0);
    assert.match(ui.document.querySelector('#chain-length-summary').textContent,/26 links.*520 mm/);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.parts.length===0);
    ui.document.querySelector('#redo').click();await ui.wait(()=>ui.state.scene.parts.length===26);
  }finally{await ui.close();}
});

test('chain length edits grow and trim links while keeping the selected object and undo history',async()=>{
  const ui=await editor(chainDesign());try{
    selectChain(ui);changeInput(ui,'#chain-length',700);
    await ui.wait(()=>ui.state.scene.parts.length===35&&!ui.state.placementPending);
    assert.equal(ui.state.scene.joints.length,34);assert.equal(ui.state.undo.length,1);
    changeInput(ui,'#chain-length',160);await ui.wait(()=>ui.state.scene.parts.length===8&&!ui.state.placementPending);
    assert.equal(ui.state.selected,'chain/link-8');assert.ok(ui.document.querySelector('#chain-length'));
    assert.equal(ui.state.undo.length,2);assert.equal(ui.state.doc.objects[0].parameters.length_mm,160);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.parts.length===35);
    assert.equal(ui.document.querySelector('#chain-length').value,'700');
  }finally{await ui.close();}
});

test('a chain can move whole, pose links, hold that shape and keep hundreds of links out of the tree',async()=>{
  const ui=await editor(chainDesign(200));try{
    selectChain(ui);changeInput(ui,'[data-pose="position_mm"][data-axis="0"]',3000);
    await ui.wait(()=>ui.state.doc.objects[0].pose.position_mm[0]===3000&&!ui.state.placementPending,20000);
    assert.equal(ui.state.doc.objects[0].components,undefined);assert.equal(ui.state.scene.groups.length,1);
    ui.document.querySelector('[data-object-mode="limb"]').click();await ui.wait(()=>ui.state.doc.objects[0].layout_mode==='posable'&&!ui.state.placementPending);
    assert.equal(ui.state.scene.groups.length,200);assert.equal(ui.document.querySelectorAll('#outline-list [data-select]').length,0);
    const pose=ui.state.scene.parts.find(p=>p.id==='chain/link-200').pose;
    changeInput(ui,'[data-part-pose="position_mm"][data-axis="2"]',pose.position_mm[2]+30);
    await ui.wait(()=>!!ui.state.doc.objects[0].components&&!ui.state.placementPending,20000);
    const shaped=structuredClone(ui.state.scene.parts.map(p=>p.pose));
    ui.document.querySelector('[data-object-mode="whole"]').click();await ui.wait(()=>ui.state.doc.objects[0].layout_mode==='rigid'&&!ui.state.placementPending);
    assert.equal(ui.state.scene.groups.length,1);assert.deepEqual(ui.state.scene.parts.map(p=>p.pose),shaped);
    const tree=ui.document.querySelector('[data-object-tree]');tree.open=true;tree.dispatchEvent(new ui.window.Event('toggle'));
    assert.equal(ui.document.querySelectorAll('#outline-list [data-select]').length,200);
    const reopened=ui.document.querySelector('[data-object-tree]');reopened.open=false;reopened.dispatchEvent(new ui.window.Event('toggle'));
    assert.equal(ui.document.querySelectorAll('#outline-list [data-select]').length,0);
    ui.document.querySelector('[data-mode="simulate"]').click();assert.equal(ui.document.querySelectorAll('#inspector .joint-card').length,1);
  }finally{await ui.close();}
});

test('shortening a chain reports the attachment to release without changing it',async()=>{
  const doc=chainDesign();doc.anchors=[{part:'chain/link-25',surface:'ceiling'}];const ui=await editor(doc);try{
    selectChain(ui,'start');const before=structuredClone(ui.state.doc);changeInput(ui,'#chain-length',100);
    await ui.wait(()=>ui.document.querySelector('#toast').textContent.includes('chain/link-25')&&!ui.state.placementPending);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);assert.equal(ui.document.querySelector('#chain-length').value,'500');
  }finally{await ui.close();}
});

test('chain endpoint controls attach real eyes with a cancellable preview',async()=>{
  const doc=chainDesign();doc.parts=[{id:'hook',catalog:'generic.d-link',pose:{position_mm:[200,0,900]}}];
  const ui=await editor(doc);try{
    selectChain(ui);ui.document.querySelector('#chain-attach-start').click();
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary').disabled===false);
    assert.equal(ui.document.querySelector('#attachment-source-port').value,'b');assert.equal(ui.document.querySelector('#attachment-limb').value,'chain/link-1');
    assert.equal(ui.document.querySelector('#attachment-kind').value,'spherical');assert.equal(ui.state.doc.joints.length,0);
    modalAction(ui,'Cancel');assert.equal(ui.state.undo.length,0);
    ui.document.querySelector('#chain-attach-start').click();await ui.wait(()=>ui.document.querySelector('#modal-actions .primary').disabled===false);
    modalAction(ui,'Connect');await ui.wait(()=>ui.state.doc.joints.length===1&&!ui.state.placementPending);
    const joint=ui.state.doc.joints[0];assert.equal(joint.b.part,'chain/link-1');assert.deepEqual(joint.b.frame.position_mm,[0,0,10]);
    assert.equal(ui.state.doc.objects[0].components,undefined);assert.equal(ui.state.undo.length,1);
  }finally{await ui.close();}
});

test('the main Duplicate button copies only the selected human limb',async()=>{
  const doc=blankDesign('Copy one limb');doc.objects=[{id:'person',template:'human',parameters:{hold_joints:['arms']}}];
  doc.state={joints:{'person/right_elbow':{angle_deg:30}}};
  const ui=await editor(doc);try{
    ui.select('person/right_hand');const original=structuredClone(ui.state.doc.objects[0]),pose=structuredClone(ui.state.scene.parts.find(p=>p.id===ui.state.selected).pose);
    ui.document.querySelector('#duplicate-selected').click();await ui.wait(()=>ui.state.selected==='person-right_hand-copy'&&!ui.state.placementPending);
    assert.equal(ui.state.doc.objects.length,1);assert.equal(ui.state.doc.parts.length,1);assert.equal(ui.state.scene.parts.length,20);assert.equal(ui.state.scene.joints.length,18);
    assert.deepEqual(ui.state.doc.objects[0],original);assert.equal(ui.state.undo.length,1);
    const copied=ui.state.scene.parts.find(p=>p.id==='person-right_hand-copy');
    assert.deepEqual(copied.pose.rotation_deg,pose.rotation_deg);assert.deepEqual(copied.pose.position_mm.slice(1),pose.position_mm.slice(1));
    assert.ok(copied.pose.position_mm[0]>pose.position_mm[0]);assert.equal(ui.document.querySelector('#object-pose-x'),null);
  }finally{await ui.close();}
});

test('Duplicate menu supports keyboard navigation, Escape and outside dismissal without making a copy',async()=>{
  const ui=await editor(fixture());try{
    ui.select('pipe');const toggle=ui.document.querySelector('#duplicate-menu-toggle'),menu=ui.document.querySelector('#duplicate-menu');
    const key=(target,key)=>target.dispatchEvent(new ui.window.KeyboardEvent('keydown',{key,bubbles:true,cancelable:true}));
    assert.equal(menu.hidden,true);toggle.click();assert.equal(toggle.getAttribute('aria-expanded'),'true');
    assert.equal(ui.document.activeElement.dataset.duplicate,'count');key(ui.document.activeElement,'ArrowDown');assert.equal(ui.document.activeElement.dataset.duplicate,'touching');
    key(ui.document.activeElement,'End');assert.equal(ui.document.activeElement.dataset.duplicate,'subassembly');
    key(ui.document.activeElement,'Escape');assert.equal(menu.hidden,true);assert.equal(ui.document.activeElement,toggle);assert.equal(ui.state.undo.length,0);
    key(toggle,'ArrowUp');assert.equal(ui.document.activeElement.dataset.duplicate,'subassembly');
    ui.document.querySelector('#viewport').dispatchEvent(new ui.window.MouseEvent('pointerdown',{bubbles:true}));assert.equal(menu.hidden,true);assert.equal(toggle.getAttribute('aria-expanded'),'false');
    toggle.click();ui.document.querySelector('#part-search').focus();assert.equal(menu.hidden,true);
    assert.equal(ui.state.doc.parts.length,2);assert.equal(ui.state.undo.length,0);
  }finally{await ui.close();}
});

function duplicateGraph(){
  const doc=blankDesign('Duplicate graph');
  doc.parts=['a','b','c','d','e'].map((id,i)=>({id,body:{kind:'rigid',mass_kg:1,geometry:[{type:'box',size_mm:[20,20,20]}]},pose:{position_mm:[20*i,0,10]}}));
  doc.joints=['a','b','c','d'].map((id,i)=>({id:id+'-'+doc.parts[i+1].id,type:i===2?'cylindrical':'fixed',a:{part:id,frame:{position_mm:[10,0,0],axis:[1,0,0]}},b:{part:doc.parts[i+1].id,frame:{position_mm:[-10,0,0],axis:[1,0,0]}}}));
  doc.anchors=[{part:'a',surface:'fixture'}];return doc;
}

test('Duplicate touching copies one hop including loose joints and retains only internal connections',async()=>{
  const ui=await editor(duplicateGraph());try{
    ui.select('c');ui.document.querySelector('#duplicate-menu-toggle').click();ui.document.querySelector('[data-duplicate="touching"]').click();
    await ui.wait(()=>ui.state.selected==='c-copy'&&!ui.state.placementPending);
    assert.deepEqual(ui.state.doc.parts.slice(5).map(p=>p.id),['b-copy','c-copy','d-copy']);
    assert.equal(ui.state.doc.joints.length,6);assert.equal(ui.state.doc.joints.find(j=>j.id==='c-d-copy').type,'cylindrical');
    assert.equal(ui.state.scene.anchors.length,1);assert.equal(ui.state.undo.length,1);
    assert.ok(ui.state.doc.joints.slice(4).every(j=>j.a.part.endsWith('-copy')&&j.b.part.endsWith('-copy')));
  }finally{await ui.close();}
});

test('Duplicate subassembly follows locked joints and stops at a loose connection',async()=>{
  const ui=await editor(duplicateGraph());try{
    ui.select('c');ui.document.querySelector('#duplicate-menu-toggle').click();ui.document.querySelector('[data-duplicate="subassembly"]').click();
    await ui.wait(()=>ui.state.selected==='c-copy'&&!ui.state.placementPending);
    assert.deepEqual(ui.state.doc.parts.slice(5).map(p=>p.id),['a-copy','b-copy','c-copy']);
    assert.equal(ui.state.doc.joints.length,6);assert.equal(ui.state.scene.anchors.length,1);
    assert.deepEqual(ui.state.scene.groups.find(g=>g.includes('c-copy')).sort(),['a-copy','b-copy','c-copy']);
    assert.equal(ui.state.undo.length,1);
  }finally{await ui.close();}
});

test('Duplicate N copies validates the count, supports scope selection and undoes the batch in one step',async()=>{
  const ui=await editor(duplicateGraph());try{
    ui.select('c');const original=structuredClone(ui.state.doc);
    ui.document.querySelector('#duplicate-menu-toggle').click();ui.document.querySelector('[data-duplicate="count"]').click();
    assert.equal(ui.document.querySelector('#modal').open,true);assert.equal(ui.document.querySelector('#duplicate-count').value,'2');
    assert.equal(ui.document.querySelector('#duplicate-scope').value,'part');assert.equal(ui.state.doc.parts.length,5);
    const count=ui.document.querySelector('#duplicate-count'),scope=ui.document.querySelector('#duplicate-scope');
    for(const invalid of ['0','1.5','101','']){
      count.value=invalid;count.dispatchEvent(new ui.window.Event('input',{bubbles:true}));ui.document.querySelector('#modal-actions .primary').click();await pause(20);
      assert.equal(ui.state.undo.length,0);assert.equal(count.checkValidity(),false);assert.equal(ui.document.querySelector('#duplicate-summary').textContent,'');
    }
    count.value='3';scope.value='subassembly';scope.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
    assert.match(ui.document.querySelector('#duplicate-summary').textContent,/3 new copies × 3 parts = 9 new parts/);
    ui.document.querySelector('#duplicate-form').dispatchEvent(new ui.window.Event('submit',{bubbles:true,cancelable:true}));
    await ui.wait(()=>ui.state.selected==='c-copy-3'&&!ui.state.placementPending&&!ui.document.querySelector('#modal').open);
    assert.equal(ui.state.doc.parts.length,14);assert.equal(ui.state.doc.joints.length,10);assert.equal(ui.state.undo.length,1);
    const copied=structuredClone(ui.state.doc);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.parts.length===5);assert.deepEqual(ui.state.doc.parts,original.parts);assert.deepEqual(ui.state.doc.joints,original.joints);
    ui.document.querySelector('#redo').click();await ui.wait(()=>ui.state.scene.parts.length===14);assert.deepEqual(ui.state.doc.parts,copied.parts);assert.deepEqual(ui.state.doc.joints,copied.joints);
    ui.select('c');ui.document.querySelector('#duplicate-selected').click();await ui.wait(()=>ui.state.selected==='c-copy-4'&&!ui.state.placementPending);
    assert.equal(ui.state.doc.parts.length,15);assert.equal(ui.state.doc.joints.length,10);
  }finally{await ui.close();}
});

test('cancelling a Duplicate count popup discards a pending result and leaves the original untouched',async()=>{
  let started=false,release;const responseReady=new Promise(resolve=>release=resolve);
  const ui=await editor(fixture(),null,{beforeResponse:async path=>{if(path==='/api/duplicate'){started=true;await responseReady;}}});
  try{
    ui.select('pipe');const original=structuredClone(ui.state.doc);
    ui.document.querySelector('#duplicate-menu-toggle').click();ui.document.querySelector('[data-duplicate="count"]').click();
    ui.document.querySelector('#modal-actions button').click();assert.deepEqual(ui.state.doc,original);assert.equal(ui.state.undo.length,0);
    ui.document.querySelector('#duplicate-menu-toggle').click();ui.document.querySelector('[data-duplicate="count"]').click();
    ui.document.querySelector('#modal-actions .primary').click();await ui.wait(()=>started);
    assert.equal(ui.document.querySelector('#duplicate-selected').disabled,true);assert.equal(ui.document.querySelector('#duplicate-count').disabled,true);
    ui.document.querySelector('#modal-close').click();release();await ui.wait(()=>!ui.state.placementPending);
    assert.deepEqual(ui.state.doc,original);assert.equal(ui.state.selected,'pipe');assert.equal(ui.state.undo.length,0);
    assert.equal(ui.document.querySelector('#duplicate-selected').disabled,false);
  }finally{release();await ui.close();}
});

test('a legacy expanded human exposes regroup and can then be duplicated as one person',async()=>{
  const expanded=await post('expand',seatedHuman());delete expanded.expanded_objects;
  const ui=await editor(expanded);try{
    ui.select('person/left_shin');ui.document.querySelector('#regroup-object').click();
    await ui.wait(()=>ui.state.doc.objects?.length===1&&!ui.state.placementPending);
    ui.document.querySelector('#duplicate-menu-toggle').click();ui.document.querySelector('[data-duplicate="subassembly"]').click();await ui.wait(()=>ui.state.doc.objects.length===2&&!ui.state.placementPending);
    assert.equal(ui.state.scene.parts.length,38);assert.equal(ui.state.scene.joints.length,36);
    const [a,b]=ui.state.doc.objects;assert.deepEqual(a.components,b.components);
    assert.equal(b.id,'person-copy');assert.equal(ui.state.doc.parts.length,0);
  }finally{await ui.close();}
});

test('whole-person gizmos translate metres and flip posture before posing one arm',async()=>{
  const ui=await editor(seatedHuman());try{
    ui.select('person/left_hand');ui.document.querySelector('[data-tool="translate"]').click();
    const original=new Map([...ui.partObjects].map(([id,o])=>[id,o.matrix.clone()]));
    assert.notEqual(ui.gizmo.object,ui.partObjects.get('person/left_hand'));
    ui.gizmo.axis='X';ui.gizmo.dispatchEvent({type:'mouseDown'});ui.gizmo.object.position.x+=3000;
    ui.gizmo.dispatchEvent({type:'objectChange'});ui.gizmo.dispatchEvent({type:'mouseUp'});
    await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    for(const [id,matrix] of original)assert.ok(ui.partObjects.get(id).position.distanceTo(new Three.Vector3().setFromMatrixPosition(matrix).add(new Three.Vector3(3000,0,0)))<1e-5);
    const before=new Map([...ui.partObjects].map(([id,o])=>[id,o.matrix.clone()]));
    ui.document.querySelector('[data-tool="rotate"]').click();const pivot=ui.gizmo.object.position.clone();
    ui.gizmo.axis='X';ui.gizmo.dispatchEvent({type:'mouseDown'});ui.gizmo.object.quaternion.premultiply(new Three.Quaternion().setFromAxisAngle(new Three.Vector3(1,0,0),Math.PI));
    ui.gizmo.dispatchEvent({type:'objectChange'});ui.gizmo.dispatchEvent({type:'mouseUp'});
    await ui.wait(()=>ui.state.undo.length===2&&!ui.state.placementPending);
    const delta=new Three.Matrix4().makeTranslation(...pivot.toArray()).multiply(new Three.Matrix4().makeRotationX(Math.PI)).multiply(new Three.Matrix4().makeTranslation(...pivot.clone().negate().toArray()));
    for(const [id,matrix] of before){const expected=delta.clone().multiply(matrix);assert.ok(ui.partObjects.get(id).matrix.elements.every((v,i)=>Math.abs(v-expected.elements[i])<1e-4));}
    assert.equal(ui.state.doc.objects.length,1);assert.equal(ui.state.doc.parts.length,0);
    ui.document.querySelector('[data-object-mode="limb"]').click();ui.document.querySelector('[data-tool="translate"]').click();
    const stationary=ui.partObjects.get('person/right_hand').position.clone(),hand=ui.partObjects.get('person/left_hand').position.clone();
    ui.gizmo.axis='X';ui.gizmo.dispatchEvent({type:'mouseDown'});ui.partObjects.get('person/left_hand').position.x+=20;
    ui.gizmo.dispatchEvent({type:'objectChange'});ui.gizmo.dispatchEvent({type:'mouseUp'});
    await ui.wait(()=>ui.state.undo.length===3&&!ui.state.placementPending);
    assert.ok(ui.partObjects.get('person/left_hand').position.distanceTo(hand)>5);
    assert.ok(ui.partObjects.get('person/right_hand').position.distanceTo(stationary)<1e-5);
    assert.equal(ui.state.doc.objects.length,1);
  }finally{await ui.close();}
});

test('object attachments detach and reconnect from a preview without editing skeleton joints',async()=>{
  const opened=await post('parse',blankDesign('Grip'),{text:await readFile('examples/human-pull-up.pipe.yaml','utf8')});
  const ui=await editor(opened.document);try{
    ui.select('person/left_hand');const skeleton=ui.state.scene.joints.filter(j=>j.id.startsWith('person/'));
    const buttons=[...ui.document.querySelectorAll('[data-attachment-detach]')];assert.equal(buttons.length,2);
    const jid=buttons[0].dataset.attachmentDetach;buttons[0].click();
    await ui.wait(()=>!ui.state.doc.joints.some(j=>j.id===jid)&&!ui.state.placementPending);
    assert.deepEqual(ui.state.scene.joints.filter(j=>j.id.startsWith('person/')),skeleton);
    ui.document.querySelector(`[data-attachment-reconnect="${jid}"]`).click();
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    assert.ok(!ui.state.doc.joints.some(j=>j.id===jid));assert.equal(ui.state.undo.length,1);
    modalAction(ui,'Connect');await ui.wait(()=>ui.state.doc.joints.some(j=>j.id===jid)&&!ui.state.placementPending);
    assert.equal(ui.state.undo.length,2);assert.equal(ui.state.doc.objects.length,1);
    assert.equal((await post('validate',ui.state.doc)).valid,true);
  }finally{await ui.close();}
});

test('an attached whole person reports the grip to detach and keeps the original pose',async()=>{
  const opened=await post('parse',blankDesign('Grip'),{text:await readFile('examples/human-pull-up.pipe.yaml','utf8')});
  const ui=await editor(opened.document);try{
    ui.select('person/left_hand');const original=structuredClone(ui.state.doc);
    const input=ui.document.querySelector('[data-pose="position_mm"][data-axis="0"]');input.value='5000';input.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
    await ui.wait(()=>!ui.state.placementPending);
    assert.deepEqual(ui.state.doc,original);assert.equal(ui.state.undo.length,0);
    assert.match(ui.document.querySelector('#toast').textContent,/grip.*Detach.*Connections to structure/);
    assert.equal(ui.document.querySelectorAll('[data-attachment-detach]').length,2);
  }finally{await ui.close();}
});

test('a new body attachment selects a bar station and previews without editing until Connect',async()=>{
  const doc=seatedHuman(),resolved=await post('resolve',doc),hand=resolved.parts.find(p=>p.id==='person/left_hand');
  doc.parts=[{id:'bar',catalog:'tubeclamp.tube-C',parameters:{length_mm:800},pose:{position_mm:[hand.pose.position_mm[0]+15,...hand.pose.position_mm.slice(1)],rotation_deg:[0,90,0]}}];
  const ui=await editor(doc);try{
    ui.select('person/left_hand');const before=structuredClone(ui.state.doc),position=ui.partObjects.get('person/left_hand').position.clone();
    ui.document.querySelector('#object-attach-part').click();
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    assert.equal(ui.document.querySelector('#attachment-target').value,'bar');
    const station=ui.document.querySelector('#attachment-station');station.value='400';station.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);
    modalAction(ui,'Cancel');assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.placementPending,false);
    ui.document.querySelector('#object-attach-part').click();
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    ui.document.querySelector('#attachment-station').value='400';ui.document.querySelector('#attachment-station').dispatchEvent(new ui.window.Event('change',{bubbles:true}));
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    modalAction(ui,'Connect');await ui.wait(()=>ui.state.doc.joints.length===1&&!ui.state.placementPending);
    assert.equal(ui.state.doc.objects.length,1);assert.equal(ui.state.scene.joints.length,19);assert.equal(ui.state.undo.length,1);
    assert.equal(ui.state.doc.joints[0].a.part,'bar');assert.equal(ui.state.doc.joints[0].a.at_mm,400);
    assert.ok(ui.partObjects.get('person/left_hand').position.distanceTo(position)>5);
    assert.equal(ui.document.querySelectorAll('[data-attachment-detach]').length,1);
  }finally{await ui.close();}
});

test('TC148 is available in the toolbox, loads its mesh and connects a continuous pipe',async()=>{
  const doc=fixture();doc.parts=doc.parts.slice(0,1);
  const ui=await editor(doc);try{
    const search=ui.document.querySelector('#part-search');search.value='TC148';search.dispatchEvent(new ui.window.Event('input',{bubbles:true}));
    assert.equal(ui.document.querySelectorAll('[data-catalog]').length,1);
    ui.document.querySelector('[data-catalog="tubeclamp.TC148C"]').click();await ui.wait(()=>ui.state.doc.parts.length===2&&ui.state.scene.parts.length===2);
    const tee=ui.state.scene.parts.find(p=>p.catalog==='tubeclamp.TC148C');assert.equal(tee.ports.through.engagement_mm,28.5);
    await ui.wait(()=>ui.partObjects.get(tee.id).children[0].children.some(o=>o.isMesh));
    ui.select('pipe');ui.document.querySelector('#connect-selected').click();ui.select(tee.id);
    await ui.wait(()=>ui.document.querySelector('#modal-actions .primary')?.disabled===false);
    assert.equal(ui.document.querySelector('#snap-station').value,'500');modalAction(ui,'Connect');
    await ui.wait(()=>ui.state.doc.joints.length===1&&!ui.state.placementPending);
    assert.equal(ui.state.doc.joints[0].a.port,'through');assert.equal((await post('validate',ui.state.doc)).valid,true);
  }finally{await ui.close();}
});
function startShinDrag(ui,offset){
  ui.select('person/left_shin');ui.document.querySelector('[data-object-mode="limb"]').click();ui.document.querySelector('[data-tool="translate"]').click();ui.gizmo.axis='Y';
  ui.gizmo.dispatchEvent({type:'mouseDown'});ui.partObjects.get('person/left_shin').position.y+=offset;
  ui.gizmo.dispatchEvent({type:'objectChange'});
}

test('a shin drag previews following joints before release and saves one undoable pose',async()=>{
  const moves=[];
  const ui=await editor(seatedHuman(),null,{interceptFetch:(path,options)=>{if(path==='/api/move')moves.push(JSON.parse(options.body));return null;}});try{
    ui.state.connectionSnap=false;const before=structuredClone(ui.state.doc),foot=ui.partObjects.get('person/left_foot').position.clone();
    const shin=ui.partObjects.get('person/left_shin').position.clone();startShinDrag(ui,30);
    await ui.wait(()=>ui.partObjects.get('person/left_foot').position.distanceTo(foot)>5);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);
    assert.equal(moves.length,1);assert.equal(moves[0].pose_only,true);assert.equal(moves[0].preview,true);
    assert.ok(ui.partObjects.get('person/left_shin').position.distanceTo(shin.clone().add(new Three.Vector3(0,30,0)))<.1);
    ui.gizmo.dispatchEvent({type:'mouseUp'});await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    assert.equal(moves.length,2);assert.equal(typeof moves[1].preview_id,'string');assert.ok(!moves[1].preview);
    assert.equal(ui.state.scene.parts.length,19);assert.equal(ui.state.doc.objects.length,1);assert.ok(ui.state.doc.objects[0].components);
    assert.equal((await post('validate',ui.state.doc)).issues.filter(i=>['JOINT_POSITION','JOINT_AXIS','SOCKET_ALIGNMENT'].includes(i.code)).length,0);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.doc.objects.length===1&&ui.partObjects.get('person/left_foot').position.distanceTo(foot)<1e-6);
    assert.deepEqual(ui.state.doc.objects,before.objects);assert.ok(ui.partObjects.get('person/left_foot').position.distanceTo(foot)<1e-6);
    ui.document.querySelector('#redo').click();await ui.wait(()=>ui.state.doc.objects[0].components&&ui.partObjects.get('person/left_foot').position.distanceTo(foot)>5);
    assert.ok(ui.partObjects.get('person/left_foot').position.distanceTo(foot)>5);
  }finally{await ui.close();}
});

test('cancelling while a constrained preview is pending discards its late response',async()=>{
  let release,started=false;const held=new Promise(resolve=>release=resolve);
  const ui=await editor(seatedHuman(),null,{beforeResponse:async path=>{if(path==='/api/move'){started=true;await held;}}});
  try{
    const before=structuredClone(ui.state.doc),foot=ui.partObjects.get('person/left_foot').position.clone();startShinDrag(ui,30);
    await ui.wait(()=>started);ui.document.dispatchEvent(new ui.window.KeyboardEvent('keydown',{key:'Escape',bubbles:true}));release();await pause(150);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);assert.equal(ui.state.placementPending,false);
    assert.ok(ui.partObjects.get('person/left_foot').position.distanceTo(foot)<1e-6);
  }finally{release();await ui.close();}
});

test('releasing during a slow preview commits the latest drag target once',async()=>{
  let release,started=false;const held=new Promise(resolve=>release=resolve);
  const ui=await editor(seatedHuman(),null,{beforeResponse:async path=>{if(path==='/api/move'&&!started){started=true;await held;}}});
  try{
    ui.state.connectionSnap=false;const before=ui.partObjects.get('person/left_shin').position.clone();startShinDrag(ui,10);
    await ui.wait(()=>started);ui.partObjects.get('person/left_shin').position.copy(before).add(new Three.Vector3(0,40,0));
    ui.gizmo.dispatchEvent({type:'objectChange'});ui.gizmo.dispatchEvent({type:'mouseUp'});release();
    await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    assert.ok(ui.partObjects.get('person/left_shin').position.distanceTo(before.add(new Three.Vector3(0,40,0)))<.1);
  }finally{release();await ui.close();}
});

test('cancelling while the released pose is being saved discards the late commit',async()=>{
  let release,committing=false;const held=new Promise(resolve=>release=resolve);
  const ui=await editor(seatedHuman(),null,{
    interceptFetch:(path,options)=>{if(path==='/api/move'&&!JSON.parse(options.body).preview)committing=true;return null;},
    beforeResponse:async path=>{if(path==='/api/move'&&committing)await held;}
  });
  try{
    ui.state.connectionSnap=false;const before=structuredClone(ui.state.doc),foot=ui.partObjects.get('person/left_foot').position.clone();
    startShinDrag(ui,30);await ui.wait(()=>ui.partObjects.get('person/left_foot').position.distanceTo(foot)>5);
    ui.gizmo.dispatchEvent({type:'mouseUp'});await ui.wait(()=>committing);
    ui.document.dispatchEvent(new ui.window.KeyboardEvent('keydown',{key:'Escape',bubbles:true}));release();await pause(300);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);assert.equal(ui.state.placementPending,false);
    assert.ok(ui.partObjects.get('person/left_foot').position.distanceTo(foot)<1e-6);
  }finally{release();await ui.close();}
});

test('a failed pose commit restores the viewport and does not create an undo edit',async()=>{
  const ui=await editor(seatedHuman(),null,{interceptFetch:(path,options)=>path==='/api/move'&&!JSON.parse(options.body).preview?
    new Response(JSON.stringify({error:'Fixture changed while placing'}),{status:400}):null});
  try{
    ui.state.connectionSnap=false;const before=structuredClone(ui.state.doc),foot=ui.partObjects.get('person/left_foot').position.clone();
    startShinDrag(ui,30);await ui.wait(()=>ui.partObjects.get('person/left_foot').position.distanceTo(foot)>5);
    ui.gizmo.dispatchEvent({type:'mouseUp'});await ui.wait(()=>ui.document.querySelector('#toast').textContent.includes('Fixture changed'));
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);assert.equal(ui.state.placementPending,false);
    assert.ok(ui.partObjects.get('person/left_foot').position.distanceTo(foot)<1e-6);
  }finally{await ui.close();}
});

async function attachedOffsetCross(){
  const doc=fixture('TC161C');
  const result=await post('snap-options',doc,{member:'pipe',connector:'tee',port:'cross',at_mm:500,locked:false});
  return result.options.find(o=>o.move==='connector').document;
}

test('offset cross rotation uses a handle on the pipe and keeps its bore attached',async()=>{
  const ui=await editor(await attachedOffsetCross());try{
    ui.state.connectionSnap=false;ui.select('tee');ui.document.querySelector('[data-tool="rotate"]').click();
    const start=ui.partObjects.get('tee').position.clone();
    assert.notEqual(ui.gizmo.object,ui.partObjects.get('tee'));assert.deepEqual(ui.gizmo.object.position.toArray(),[0,0,500]);
    ui.gizmo.axis='Z';ui.gizmo.dispatchEvent({type:'mouseDown'});
    ui.gizmo.object.quaternion.multiply(new Three.Quaternion().setFromAxisAngle(new Three.Vector3(0,0,1),Math.PI/2));
    ui.gizmo.dispatchEvent({type:'objectChange'});ui.gizmo.dispatchEvent({type:'mouseUp'});
    await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    const object=ui.partObjects.get('tee');assert.ok(object.position.distanceTo(start)>50);
    assert.ok(new Three.Vector3(0,42.4,0).applyMatrix4(object.matrix).distanceTo(new Three.Vector3(0,0,500))<1e-5);
    assert.equal((await post('validate',ui.state.doc)).valid,true);
  }finally{await ui.close();}
});

test('position fields respect a loose socket and stop at its available travel',async()=>{
  const doc=await attachedOffsetCross();doc.joints[0].limits={slide_mm:[-25,25],angle_deg:[-.1,.1]};
  const ui=await editor(doc);try{
    ui.select('tee');const input=ui.document.querySelector('[data-pose="position_mm"][data-axis="2"]');
    input.value='600';input.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
    await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    assert.ok(Math.abs(ui.partObjects.get('tee').position.z-525)<.001);
    assert.match(ui.document.querySelector('#status-text').textContent,/travel/);
    assert.equal((await post('validate',ui.state.doc)).valid,true);
  }finally{await ui.close();}
});

test('a reusable human exposes constrained segment fields without manual expansion',async()=>{
  const ui=await editor(seatedHuman());try{
    ui.select('person/left_shin');ui.document.querySelector('[data-object-mode="limb"]').click();const before=ui.partObjects.get('person/left_foot').position.clone();
    const input=ui.document.querySelector('[data-part-pose="position_mm"][data-axis="1"]');assert.ok(input);
    input.value=String(Number(input.value)+30);input.dispatchEvent(new ui.window.Event('change',{bubbles:true}));
    await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    assert.ok(ui.partObjects.get('person/left_foot').position.distanceTo(before)>5);
    assert.equal(ui.state.doc.objects.length,1);assert.ok(ui.state.doc.objects[0].components);assert.equal(ui.state.scene.joints.length,18);
  }finally{await ui.close();}
});

test('a free human object position field moves the complete articulated object',async()=>{
  const ui=await editor(seatedHuman());try{
    ui.select('person/left_shin');const before=new Map([...ui.partObjects].map(([id,o])=>[id,o.position.clone()]));
    const input=ui.document.querySelector('[data-pose="position_mm"][data-axis="0"]');input.value='100';
    input.dispatchEvent(new ui.window.Event('change',{bubbles:true}));await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    for(const [id,position] of before)assert.ok(ui.partObjects.get(id).position.distanceTo(position.add(new Three.Vector3(100,0,0)))<1e-5);
  }finally{await ui.close();}
});

test('rotating a thigh uses its spherical hip pivot and carries the lower leg',async()=>{
  const ui=await editor(seatedHuman());try{
    ui.state.connectionSnap=false;ui.select('person/left_thigh');ui.document.querySelector('[data-object-mode="limb"]').click();ui.document.querySelector('[data-tool="rotate"]').click();
    ui.document.querySelector('#snap-button').click();
    assert.equal(ui.gizmo.showX,true);assert.equal(ui.gizmo.showY,true);
    const before=ui.partObjects.get('person/left_foot').position.clone(),hip=ui.gizmo.object.position.clone();
    assert.ok(ui.gizmo.object.position.distanceTo(ui.partObjects.get('person/left_thigh').position)>150);
    ui.gizmo.axis='X';ui.gizmo.dispatchEvent({type:'mouseDown'});
    ui.gizmo.object.quaternion.multiply(new Three.Quaternion().setFromAxisAngle(new Three.Vector3(1,0,0),Three.MathUtils.degToRad(15)));
    ui.gizmo.dispatchEvent({type:'objectChange'});ui.gizmo.dispatchEvent({type:'mouseUp'});
    await ui.wait(()=>ui.state.undo.length===1&&!ui.state.placementPending);
    assert.ok(ui.partObjects.get('person/left_foot').position.distanceTo(before)>30);
    assert.ok(ui.gizmo.object.position.distanceTo(hip)<1e-5);
  }finally{await ui.close();}
});


test('Simulation progress, early cancellation and retry preserve the previous recording',async()=>{
  let releaseStart,cancelled=false,optionsSeen,statusPolls=0;
  const startGate=new Promise(resolve=>releaseStart=resolve);
  const ui=await editor(freeTube(),null,{interceptFetch:(path,options)=>{
    const data=options?.body?JSON.parse(options.body):{};
    if(path==='/api/simulate'){
      optionsSeen=data;
      return startGate.then(()=>new Response(JSON.stringify({id:'test-job',status:'running',progress:{phase:'building',message:'Building rigid body geometry'}})));
    }
    if(path==='/api/simulation-cancel'){cancelled=true;return new Response(JSON.stringify({id:'test-job',status:'cancelled'}));}
    if(path==='/api/simulation-status'){statusPolls++;return new Response(JSON.stringify({id:'test-job',status:cancelled?'cancelled':'running',elapsed_s:4,progress:{phase:'simulating',message:'Integrating physics',simulated_s:1,duration_s:3,percent:33.3,eta_s:8}}));}
    return null;
  }});
  try{
    const old={frames:[{time_s:0,parts:{}}],duration_s:1};
    ui.state.recording=old;ui.state.doc.results={simulate:old};
    ui.document.querySelector('[data-mode="simulate"]').click();
    assert.equal(ui.document.querySelector('#sim-chain-links').value,'1');
    ui.document.querySelector('#sim-chain-links').value='10';
    ui.document.querySelector('[data-run="simulate"]').click();
    assert.ok(ui.state.busy);assert.ok(ui.document.querySelector('[data-run="simulate"]').disabled);
    ui.document.querySelector('#cancel-simulation').click();
    assert.match(ui.document.querySelector('#sim-progress').textContent,/Cancelling/);
    releaseStart();await ui.wait(()=>!ui.state.busy);
    assert.equal(optionsSeen.chain_links_per_body,10);assert.ok(cancelled);
    assert.equal(ui.state.recording,old);assert.equal(ui.state.doc.results.simulate,old);
    assert.equal(ui.document.querySelector('#cancel-simulation'),null);
    assert.match(ui.document.querySelector('#status-text').textContent,/cancelled/);
    cancelled=false;
    ui.document.querySelector('[data-run="simulate"]').click();
    await ui.wait(()=>statusPolls>0);
    assert.match(ui.document.querySelector('#sim-progress').textContent,/33.3%/);
    assert.equal(ui.document.querySelector('#sim-progress progress').value,33.3);
    ui.document.querySelector('#cancel-simulation').click();
    await ui.wait(()=>!ui.state.busy);assert.equal(ui.state.recording,old);
  }finally{releaseStart();await ui.close();}
});

test('Failed simulation restores the Run button and does not overwrite results',async()=>{
  const ui=await editor(freeTube(),null,{interceptFetch:path=>{
    if(path==='/api/simulate')return new Response(JSON.stringify({id:'failed-job',status:'failed',error:'Physics solver diverged'}));
    return null;
  }});
  try{
    ui.document.querySelector('[data-mode="simulate"]').click();
    ui.document.querySelector('[data-run="simulate"]').click();await ui.wait(()=>!ui.state.busy);
    assert.match(ui.document.querySelector('#toast').textContent,/solver diverged/);
    assert.equal(ui.state.recording,null);assert.equal(ui.state.doc.results?.simulate,undefined);
    assert.equal(ui.document.querySelector('[data-run="simulate"]').disabled,false);
  }finally{await ui.close();}
});


test('Tree actions delete an attached grouped chain in one Undo and Redo',async()=>{
  const doc=freeTube();doc.objects=[{id:'sling',template:'chain',parameters:{length_mm:100}}];
  doc.joints=[{id:'attachment',type:'spherical',a:{part:'sling/link-5'},b:{part:doc.parts[0].id}}];
  const ui=await editor(doc);try{
    const before=structuredClone(ui.state.doc);ui.select(doc.parts[0].id);
    const toggle=ui.document.querySelector('[aria-label="Actions for sling"]');toggle.click();
    assert.equal(ui.document.querySelector('#tree-actions-menu').getAttribute('role'),'menu');
    assert.match(ui.document.querySelector('#tree-actions-menu').textContent,/Duplicate subassembly/);
    ui.document.querySelector('[data-tree-command="delete"]').click();await ui.wait(()=>!ui.state.placementPending);
    assert.equal(ui.state.doc.objects.length,0);assert.equal(ui.state.doc.parts.length,1);
    assert.equal(ui.state.doc.joints.length,0);assert.equal(ui.state.selected,doc.parts[0].id);
    assert.equal(ui.state.undo.length,1);assert.equal(ui.document.querySelector('[data-object-select="sling"]'),null);
    ui.document.querySelector('#undo').click();await ui.wait(()=>ui.state.scene.parts.length===6);assert.deepEqual(ui.state.doc,{...before,results:{}});
    ui.document.querySelector('#redo').click();await ui.wait(()=>ui.state.scene.parts.length===1);
  }finally{await ui.close();}
});

test('Tree Body actions delete their own parts, support keyboard dismissal, and leave other bodies alone',async()=>{
  const doc=freeTube();doc.parts.push({...structuredClone(doc.parts[0]),id:'other',pose:{position_mm:[1000,0,0]}});
  const ui=await editor(doc);try{
    ui.select('other');const toggle=ui.document.querySelector('[aria-label="Actions for Body 01"]');
    toggle.dispatchEvent(new ui.window.KeyboardEvent('keydown',{key:'ArrowDown',bubbles:true}));
    assert.equal(ui.document.activeElement.dataset.treeCommand,'properties');
    ui.document.activeElement.dispatchEvent(new ui.window.KeyboardEvent('keydown',{key:'End',bubbles:true}));
    assert.equal(ui.document.activeElement.dataset.treeCommand,'delete');
    ui.document.activeElement.dispatchEvent(new ui.window.KeyboardEvent('keydown',{key:'Escape',bubbles:true}));
    assert.equal(ui.document.querySelector('#tree-actions-menu'),null);assert.equal(ui.document.activeElement,toggle);
    assert.equal(ui.state.undo.length,0);
    toggle.click();ui.document.querySelector('[data-tree-command="delete"]').click();await ui.wait(()=>!ui.state.placementPending);
    assert.deepEqual(ui.state.doc.parts.map(p=>p.id),['other']);assert.equal(ui.state.selected,'other');
  }finally{await ui.close();}
});

test('Tree deletion failures preserve the document, selection and Undo history',async()=>{
  const ui=await editor(freeTube(),null,{interceptFetch:path=>path==='/api/delete'?new Response(JSON.stringify({error:'Deletion unavailable'}),{status:400}):null});
  try{
    const before=structuredClone(ui.state.doc);ui.select(before.parts[0].id);
    ui.document.querySelector('[data-tree-actions]').click();ui.document.querySelector('[data-tree-command="delete"]').click();
    await ui.wait(()=>!ui.state.placementPending);assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);
    assert.equal(ui.state.selected,before.parts[0].id);assert.match(ui.document.querySelector('#toast').textContent,/Deletion unavailable/);
  }finally{await ui.close();}
});

test('Tree actions duplicate a grouped object without expanding it',async()=>{
  const doc=freeTube();doc.objects=[{id:'sling',template:'chain',parameters:{length_mm:100}}];
  const ui=await editor(doc);try{
    ui.document.querySelector('[aria-label="Actions for sling"]').click();ui.document.querySelector('[data-tree-command="duplicate"]').click();
    await ui.wait(()=>!ui.state.placementPending);assert.equal(ui.state.doc.objects.length,2);assert.equal(ui.state.undo.length,1);
  }finally{await ui.close();}
});


test('Tree deletion of a human preserves its surroundings and part Delete never removes the whole object',async()=>{
  const doc=freeTube();doc.objects=[{id:'person',template:'human'}];
  const ui=await editor(doc);try{
    const part=ui.state.scene.parts.find(p=>p.id.startsWith('person/')).id;
    ui.select(part);ui.document.querySelector('#delete-part').click();await ui.wait(()=>!ui.state.placementPending);
    assert.equal(ui.state.doc.objects.length,1);assert.equal(ui.state.undo.length,0);assert.match(ui.document.querySelector('#toast').textContent,/Expand person/);
    const toggle=ui.document.querySelector('[aria-label="Actions for person"]');toggle.focus();
    toggle.dispatchEvent(new ui.window.KeyboardEvent('keydown',{key:'Delete',bubbles:true}));await ui.wait(()=>!ui.state.placementPending);
    assert.equal(ui.state.doc.objects.length,0);assert.equal(ui.state.scene.parts.length,1);assert.equal(ui.state.undo.length,1);
  }finally{await ui.close();}
});


test('Rename from tree changes object name and Properties, preserves references, and supports Undo and Redo',async()=>{
  const doc=freeTube();doc.objects=[{id:'chain',template:'chain',parameters:{length_mm:100}}];
  doc.joints=[{id:'attach',type:'spherical',a:{part:doc.parts[0].id},b:{part:'chain/link-1'}}];
  const ui=await editor(doc);try{
    ui.document.querySelector('[data-object-select="chain"]').click();
    ui.document.querySelector('[aria-label="Actions for chain"]').click();ui.document.querySelector('[data-tree-command="rename"]').click();
    ui.document.querySelector('#item-name').value='Main sling';modalAction(ui,'Rename');await ui.wait(()=>!ui.state.placementPending&&!ui.document.querySelector('#modal').open);
    assert.equal(ui.state.doc.objects[0].label,'Main sling');assert.equal(ui.state.doc.objects[0].id,'chain');assert.deepEqual(ui.state.doc.joints,doc.joints);
    assert.match(ui.document.querySelector('[data-object-select="chain"]').textContent,/Main sling/);assert.equal(ui.document.querySelector('#inspector h2').textContent,'Main sling');
    assert.equal(ui.state.undo.length,1);
    ui.document.querySelector('#undo').click();await ui.wait(()=>!ui.state.doc.objects[0].label);
    ui.document.querySelector('#redo').click();await ui.wait(()=>ui.document.querySelector('[aria-label="Actions for Main sling"]'));
    const opened=await post('rename',ui.state.doc,{object:'chain',name:'Main sling'});assert.equal(opened.changed,false);
  }finally{await ui.close();}
});

test('Rename is available for parts and Body entries and displays names as text',async()=>{
  const ui=await editor(freeTube());try{
    const pid=ui.state.doc.parts[0].id;ui.select(pid);
    ui.document.querySelector(`[data-select="${pid}"]`).parentElement.querySelector('[data-tree-actions]').click();ui.document.querySelector('[data-tree-command="rename"]').click();
    ui.document.querySelector('#item-name').value='Rail <b>one</b>';modalAction(ui,'Rename');await ui.wait(()=>!ui.state.placementPending&&!ui.document.querySelector('#modal').open);
    assert.match(ui.document.querySelector(`[data-select="${pid}"]`).textContent,/Rail <b>one<\/b>/);assert.equal(ui.document.querySelector(`[data-select="${pid}"] b`),null);
    assert.equal(ui.document.querySelector('#inspector h2').textContent,'Rail <b>one</b>');assert.equal(ui.state.doc.parts[0].id,pid);
    ui.document.querySelector('[aria-label="Actions for Body 01"]').click();ui.document.querySelector('[data-tree-command="rename"]').click();
    ui.document.querySelector('#item-name').value='Fixed <b>base</b>';modalAction(ui,'Rename');await ui.wait(()=>!ui.state.placementPending&&!ui.document.querySelector('#modal').open);
    assert.ok(ui.document.querySelector('[aria-label="Actions for Fixed <b>base</b>"]'));assert.match(ui.document.querySelector('[data-tree-group]').textContent,/Fixed <b>base<\/b>/);assert.equal(ui.document.querySelector('[data-tree-group] b'),null);assert.equal(ui.state.doc.metadata.body_labels[0].parts[0],pid);
  }finally{await ui.close();}
});

test('Rename validates blank names, avoids no-op Undo, and discards cancelled responses',async()=>{
  let release;const gate=new Promise(resolve=>release=resolve);
  const ui=await editor(freeTube(),null,{beforeResponse:path=>path==='/api/rename'?gate:undefined});try{
    const toggle=()=>ui.document.querySelector('[aria-label="Actions for Body 01"]').click();
    toggle();ui.document.querySelector('[data-tree-command="rename"]').click();modalAction(ui,'Rename');assert.equal(ui.state.undo.length,0);
    toggle();ui.document.querySelector('[data-tree-command="rename"]').click();ui.document.querySelector('#item-name').value='   ';modalAction(ui,'Rename');
    assert.match(ui.document.querySelector('#rename-item-error').textContent,/Enter a name/);assert.equal(ui.state.undo.length,0);
    await ui.wait(()=>!ui.document.querySelector('#modal-actions .primary').disabled);
    const before=structuredClone(ui.state.doc);ui.document.querySelector('#item-name').value='Cancelled name';modalAction(ui,'Rename');
    await ui.wait(()=>ui.state.placementPending);modalAction(ui,'Cancel');release();await ui.wait(()=>!ui.state.placementPending);
    assert.deepEqual(ui.state.doc,before);assert.equal(ui.state.undo.length,0);
  }finally{release();await ui.close();}
});

test('Simulation errors remain visible in the panel and clear on the next run',async()=>{
  let attempts=0;
  const message='Simulation could not start. Unlocked socket closes a loop. Lock structural sockets. Connection: example.';
  const ui=await editor(freeTube(),null,{interceptFetch:path=>{
    if(path==='/api/simulate'){attempts++;return new Response(JSON.stringify({id:'failed-'+attempts,status:'failed',error:message}));}
    return null;
  }});try{
    ui.document.querySelector('[data-mode="simulate"]').click();ui.document.querySelector('[data-run="simulate"]').click();await ui.wait(()=>!ui.state.busy);
    assert.match(ui.document.querySelector('#simulation-error').textContent,/Lock structural sockets/);
    ui.document.querySelector('[data-mode="design"]').click();ui.document.querySelector('[data-mode="simulate"]').click();assert.ok(ui.document.querySelector('#simulation-error'));
    ui.document.querySelector('[data-run="simulate"]').click();assert.equal(ui.document.querySelector('#simulation-error'),null);await ui.wait(()=>!ui.state.busy);
    assert.equal(attempts,2);assert.ok(ui.document.querySelector('#simulation-error'));
  }finally{await ui.close();}
});
