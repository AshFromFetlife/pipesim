import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {connectionCandidates,hingeCandidates,clearConnectionIntent,alignmentDelta,projectedStation,socketOccupied,rotationAlignment} from '../pipesim/web/snapping.js';
import {DEFAULT_SNAP_SETTINGS,loadSnapDefaults,saveSnapDefaults,validateSnapSettings} from '../pipesim/web/snap-settings.js';

function setup(through=true){
  const camera=new THREE.PerspectiveCamera(38,1.5,1,100000);camera.up.set(0,0,1);camera.position.set(0,-2000,1300);camera.lookAt(0,0,500);camera.updateMatrixWorld();
  const socket={type:'socket',profile:'round',diameter_mm:42.4,position_mm:[0,0,0],axis:[0,0,1],through,engagement_mm:60};
  const scene={parts:[{id:'pipe',kind:'member',length_mm:1000,section:{type:'tube',diameter_mm:42.4},ports:{}},
    {id:'tee',kind:'connector',ports:{through:socket}}],joints:[],groups:[['pipe'],['tee']]};
  const matrices=new Map([['pipe',new THREE.Matrix4().makeTranslation(0,0,500)],['tee',new THREE.Matrix4().makeTranslation(0,0,500)]]);
  const candidates=()=>connectionCandidates(scene,matrices,['tee'],camera,900,600);
  return {scene,matrices,camera,candidates};
}

test('snap sees a pipe at another camera depth and uses its midpoint',()=>{
  const s=setup(),target=new THREE.Vector3(0,0,500),behind=target.clone().sub(s.camera.position).normalize().multiplyScalar(600).add(target);
  s.matrices.set('tee',new THREE.Matrix4().setPosition(behind));
  const candidates=s.candidates();assert.equal(candidates.length,1);assert.equal(candidates[0].at_mm,500);assert.ok(clearConnectionIntent(candidates));
  const aligned=alignmentDelta(s.scene,s.matrices,candidates[0],'connector').multiply(s.matrices.get('tee'));
  assert.ok(new THREE.Vector3().setFromMatrixPosition(aligned).distanceTo(target)<1e-7);
});

test('projected positions on a receding pipe use perspective-correct length',()=>{
  const {camera}=setup(),a=new THREE.Vector3(-100,-900,200),b=new THREE.Vector3(100,800,1200),point=a.clone().lerp(b,.3);
  assert.ok(Math.abs(projectedStation(a,b,point,camera,900,600)-.3)<1e-8);
});

test('misaligned terminal socket requires an alignment review',()=>{
  const s=setup(false);s.scene.parts[1].ports.through.axis=[1,0,0];s.matrices.set('tee',new THREE.Matrix4().makeTranslation(0,0,1000));
  const candidates=s.candidates();assert.equal(candidates.length,1);assert.equal(candidates[0].end,'end');assert.equal(candidates[0].angle,90);assert.equal(clearConnectionIntent(candidates),false);
  const aligned=alignmentDelta(s.scene,s.matrices,candidates[0],'connector').multiply(s.matrices.get('tee'));
  const axis=new THREE.Vector3(1,0,0).transformDirection(aligned);assert.ok(axis.distanceTo(new THREE.Vector3(0,0,-1))<1e-7);
});

test('two visually overlapping pipes stay ambiguous',()=>{
  const s=setup();s.scene.parts.push({...s.scene.parts[0],id:'other'});s.scene.groups.push(['other']);
  s.matrices.set('other',s.matrices.get('pipe').clone());
  assert.equal(s.candidates().length,2);assert.equal(clearConnectionIntent(s.candidates()),false);
});

test('occupied bores and incompatible tube sizes are excluded',()=>{
  const s=setup();s.scene.parts[1].ports.through.excludes=['run_end'];s.scene.parts[1].ports.run_end={...s.scene.parts[1].ports.through,excludes:['through']};
  s.scene.joints=[{id:'existing',a:{part:'tee',port:'run_end'},b:{part:'other',end:'start'}}];
  assert.ok(socketOccupied(s.scene,'tee','through'));assert.equal(s.candidates().length,0);
  s.scene.joints=[];s.scene.parts[0].section.diameter_mm=48.3;assert.equal(s.candidates().length,0);
});

test('a fitting cannot snap to a pipe in its own rigid body',()=>{
  const s=setup();s.scene.groups=[['pipe','tee']];assert.equal(s.candidates().length,0);
});

test('seeded rename matrix uses current display names without changing snap identities',()=>{
  let seed=0x92f4a;
  const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed;};
  const labels=['CrossBar','Rail <b>one</b>','左支柱','tee / pipe','Name & "quote"','same name'];
  const s=setup(),baseline=s.candidates()[0];
  for(let i=0;i<64;i++){
    for(const part of s.scene.parts)part.label=i%7?labels[random()%labels.length]+' '+random():undefined;
    const match=s.candidates()[0];
    assert.equal(match.label,`${s.scene.parts[1].label||'tee'} / through → ${s.scene.parts[0].label||'pipe'} · 500.0 mm from start`);
    assert.deepEqual({...match,label:null},{...baseline,label:null});
  }
});

test('a detached through fitting already on its pipe keeps its physical station',()=>{
  const s=setup();
  s.scene.parts[0].length_mm=1600;
  s.matrices.set('pipe',new THREE.Matrix4().makeTranslation(0,0,800));
  s.matrices.set('tee',new THREE.Matrix4().makeTranslation(0,0,848));
  // At this view scale the fitting is within the old 8 px midpoint magnet.
  const match=connectionCandidates(s.scene,s.matrices,['tee'],s.camera,300,200)[0];
  assert.ok(match);
  assert.ok(Math.abs(match.at_mm-848)<1e-8);
});

test('dragging an eye near a clevis captures hinge bolt ports despite a small hole offset',()=>{
  const {camera}=setup();
  const eye={type:'eye',assembly:'bolt',position_mm:[0,0,0],axis:[0,-1,0]};
  const clevis={type:'clevis',assembly:'bolt',position_mm:[0,0,0],axis:[0,-1,0]};
  const scene={parts:[{id:'male',kind:'connector',ports:{hinge:eye}},
    {id:'female',kind:'connector',ports:{hinge:clevis}}],groups:[['male'],['female']],joints:[]};
  const matrices=new Map([['male',new THREE.Matrix4().makeTranslation(0,0,500)],
    ['female',new THREE.Matrix4().makeTranslation(12,0,500)]]);
  const matches=()=>hingeCandidates(scene,matrices,['male'],camera,900,600);
  assert.equal(matches().length,1);assert.equal(matches()[0].kind,'hinge');
  assert.deepEqual(matches()[0].male,{part:'male',port:'hinge'});
  assert.deepEqual(matches()[0].female,{part:'female',port:'hinge'});
  scene.parts[0].label='Renamed eye';scene.parts[1].label='Renamed clevis';
  assert.equal(matches()[0].label,'Renamed eye / hinge ↔ Renamed clevis / hinge · hinge bolt');
  assert.deepEqual(matches()[0].male,{part:'male',port:'hinge'});
  assert.deepEqual(matches()[0].female,{part:'female',port:'hinge'});
  scene.joints.push({id:'used',a:{part:'female',port:'hinge'},b:{part:'other',port:'eye'}});
  assert.equal(matches().length,0);
  scene.joints=[];scene.groups=[['male','female']];assert.equal(matches().length,0);
  scene.groups=[['male'],['female']];clevis.diameter_mm=12;eye.diameter_mm=8;
  assert.equal(matches().length,0,'incompatible bolt bores must not capture');
});

test('D-shackle drag captures chain eyes and bolt holes while respecting its shared pin',()=>{
  const {camera}=setup();
  const shackle={id:'shackle',kind:'connector',ports:{
    eye:{type:'eye',assembly:'hook',snap_role:'shackle',position_mm:[0,0,10],axis:[1,0,0]},
    pin_hook:{type:'eye',assembly:'hook',snap_role:'shackle',position_mm:[0,0,-15],axis:[1,0,0],excludes:['pin_bolt']},
    pin_bolt:{type:'clevis',assembly:'bolt',snap_role:'shackle',diameter_mm:8,position_mm:[0,0,-15],axis:[1,0,0],excludes:['pin_hook']}
  }};
  const chain={id:'chain',kind:'chain',ports:{end:{type:'eye',assembly:'hook',position_mm:[0,0,0],axis:[0,1,0]}}};
  const bolt={id:'bolt',kind:'connector',ports:{eye:{type:'eye',assembly:'bolt',diameter_mm:10.6,position_mm:[0,0,0],axis:[0,1,0]}}};
  const scene={parts:[shackle,chain,bolt],groups:[['shackle'],['chain'],['bolt']],joints:[]};
  const matrices=new Map([['shackle',new THREE.Matrix4().makeTranslation(12,0,500)],
    ['chain',new THREE.Matrix4().makeTranslation(0,0,500)],
    ['bolt',new THREE.Matrix4().makeTranslation(0,0,480)]]);
  const matches=hingeCandidates(scene,matrices,['shackle'],camera,900,600);
  assert.ok(matches.some(c=>c.kind==='hook'&&c.male.port==='eye'&&c.female.part==='chain'));
  assert.ok(matches.some(c=>c.kind==='hinge'&&c.female.port==='pin_bolt'&&c.male.part==='bolt'));
  const hole={id:'hole',kind:'connector',ports:{bolt1:{type:'bolt',assembly:'bolt',diameter_mm:8,position_mm:[0,0,0],axis:[0,0,1]}}};
  scene.parts.push(hole);scene.groups.push(['hole']);matrices.set('hole',new THREE.Matrix4().makeTranslation(0,0,480));
  assert.ok(hingeCandidates(scene,matrices,['shackle'],camera,900,600)
    .some(c=>c.kind==='hinge'&&c.male.part==='hole'&&c.female.port==='pin_bolt'));
  hole.ports.bolt1.diameter_mm=12;
  assert.equal(hingeCandidates(scene,matrices,['shackle'],camera,900,600)
    .some(c=>c.kind==='hinge'&&c.male.part==='hole'),false,'oversized hole must not capture');
  scene.joints=[{id:'pin-used',a:{part:'shackle',port:'pin_hook'},b:{part:'chain',port:'end'}}];
  assert.equal(socketOccupied(scene,'shackle','pin_bolt'),true);
  assert.equal(hingeCandidates(scene,matrices,['shackle'],camera,900,600)
    .some(c=>c.kind==='hinge'&&c.female.port==='pin_bolt'),false);
  scene.joints=[];scene.parts=[chain,{...chain,id:'other'}];scene.groups=[['chain'],['other']];
  matrices.set('other',new THREE.Matrix4().makeTranslation(12,0,500));
  assert.equal(hingeCandidates(scene,matrices,['chain'],camera,900,600).length,0,
    'unmarked chain links do not acquire extra drag magnets');
});

test('through station keeps the full socket on the pipe',()=>{
  const s=setup();s.matrices.set('tee',new THREE.Matrix4().makeTranslation(0,0,1000));
  assert.equal(s.candidates()[0].at_mm,970);
});

test('configured screen reach changes which sockets can be captured',()=>{
  const s=setup();s.matrices.set('tee',new THREE.Matrix4().makeTranslation(45,0,500));
  assert.equal(connectionCandidates(s.scene,s.matrices,['tee'],s.camera,900,600,{connectionPixels:4}).length,0);
  assert.equal(connectionCandidates(s.scene,s.matrices,['tee'],s.camera,900,600,{connectionPixels:100}).length,1);
});

test('rotation aligns with an angled part without leaving the handle plane',()=>{
  const s=setup();s.scene.parts.push({id:'reference',kind:'connector',ports:{}});s.scene.groups.push(['reference']);
  s.matrices.set('tee',new THREE.Matrix4().makeRotationZ(ThreeAngle(16)));
  s.matrices.set('reference',new THREE.Matrix4().makeRotationZ(ThreeAngle(17)));
  const match=rotationAlignment(s.scene,s.matrices,['tee'],'tee',3,new THREE.Vector3(0,0,1));
  assert.ok(match.label.startsWith('reference'));assert.ok(Math.abs(match.angleDeg-1)<1e-7);
  s.scene.parts.at(-1).label='Square frame';
  assert.ok(rotationAlignment(s.scene,s.matrices,['tee'],'tee',3,new THREE.Vector3(0,0,1)).label.startsWith('Square frame'));
  const corrected=new THREE.Matrix4().makeRotationFromQuaternion(match.correction).multiply(s.matrices.get('tee'));
  assert.ok(corrected.elements.every((v,i)=>Math.abs(v-s.matrices.get('reference').elements[i])<1e-7));
});
function ThreeAngle(degrees){return THREE.MathUtils.degToRad(degrees);}

test('an exactly aligned world axis is not pulled towards another nearby part',()=>{
  const s=setup();s.scene.parts.push({id:'reference',kind:'connector',ports:{}});s.scene.groups.push(['reference']);
  s.matrices.set('tee',new THREE.Matrix4().makeRotationZ(ThreeAngle(90)));s.matrices.set('reference',new THREE.Matrix4().makeRotationZ(ThreeAngle(89)));
  const match=rotationAlignment(s.scene,s.matrices,['tee'],'tee',6,new THREE.Vector3(0,0,1));
  assert.ok(match.label.startsWith('world'));assert.ok(match.angleDeg<1e-7);
});

test('invalid and unavailable saved preferences fall back safely',()=>{
  assert.deepEqual(loadSnapDefaults({getItem(){throw new Error('Storage unavailable');}}),DEFAULT_SNAP_SETTINGS);
  assert.deepEqual(loadSnapDefaults({getItem:()=>'{bad json'}),DEFAULT_SNAP_SETTINGS);
  assert.throws(()=>validateSnapSettings({...DEFAULT_SNAP_SETTINGS,rotationDeg:0}));
  assert.throws(()=>validateSnapSettings({...DEFAULT_SNAP_SETTINGS,translationMm:NaN}));
  assert.throws(()=>validateSnapSettings({...DEFAULT_SNAP_SETTINGS,keyboardMoveMm:0}));
  assert.throws(()=>validateSnapSettings({...DEFAULT_SNAP_SETTINGS,keyboardRotateDeg:181}));
  let saved;const storage={setItem:(key,value)=>saved=value,getItem:()=>saved};
  saveSnapDefaults(storage,{...DEFAULT_SNAP_SETTINGS,translationMm:25,rotationDeg:90});
  assert.equal(loadSnapDefaults(storage).translationMm,25);
  saved=JSON.stringify({gridEnabled:true,translationMm:25,rotationDeg:90,connectionsEnabled:true,connectionPixels:30,alignEnabled:true,alignmentDeg:6});
  assert.equal(loadSnapDefaults(storage).keyboardMoveMm,DEFAULT_SNAP_SETTINGS.keyboardMoveMm);
});
