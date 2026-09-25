import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {connectionCandidates,clearConnectionIntent,alignmentDelta,projectedStation,socketOccupied,rotationAlignment} from '../pipesim/web/snapping.js';
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
  let saved;const storage={setItem:(key,value)=>saved=value,getItem:()=>saved};
  saveSnapDefaults(storage,{...DEFAULT_SNAP_SETTINGS,translationMm:25,rotationDeg:90});
  assert.equal(loadSnapDefaults(storage).translationMm,25);
});
