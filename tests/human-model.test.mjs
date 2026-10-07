import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {RetargetedHuman,HumanModelLayer,mapHumanoidBones,suggestBoneMappings,boneDiagramSvg} from '../pipesim/web/human-model.js';
import {makeRig,humanScene,exportRig,SEGMENTS} from './helpers/human-rig-fixtures.mjs';
import {humanPhysicsFixture} from './helpers/human-physics-fixture.mjs';
const near=(a,b,epsilon=1e-6)=>assert.ok(a.distanceTo(b)<epsilon,`${a.toArray()} differs from ${b.toArray()}`);
const finiteSkin=mesh=>{mesh.updateMatrixWorld(true);mesh.skeleton.update();for(let i=0;i<mesh.geometry.attributes.position.count;i++){const p=mesh.getVertexPosition(i,new THREE.Vector3()).applyMatrix4(mesh.matrixWorld);assert.ok(p.toArray().every(Number.isFinite),'finite skinned vertex');assert.ok(p.length()<10000,'bounded skinned vertex');}};
for(const variant of ['mixamo','blender','vrm'])for(const tPose of [true,false])test(`${variant} ${tPose?'T':'neutral'} bind retarget preserves skin, UV and maps all 19 body pivots`,()=>{
  const rig=makeRig({variant,tPose}),{human,parts}=humanScene(),geometry=rig.mesh.geometry,uv=geometry.attributes.uv.array.slice(),inverses=rig.mesh.skeleton.boneInverses.map(m=>m.clone()),material=rig.mesh.material;
  const mapped=mapHumanoidBones(rig);assert.equal(Object.keys(mapped.mapping).length,19);assert.deepEqual(mapped.missing,[]);
  const model=new RetargetedHuman(rig,human,parts);
  for(const segment of SEGMENTS)near(model.mapping[segment].getWorldPosition(new THREE.Vector3()),parts.get('person/'+segment).position);
  assert.equal(rig.mesh.geometry,geometry);assert.equal(rig.mesh.material,material);assert.deepEqual(geometry.attributes.uv.array,uv);assert.deepEqual(rig.mesh.skeleton.boneInverses,inverses);finiteSkin(rig.mesh);
  for(const {mesh} of model.simpleMeshes)assert.equal(mesh.visible,false);
  model.setEnabled(false);for(const {mesh} of model.simpleMeshes)assert.equal(mesh.visible,true);
  model.setEnabled(true);model.update(0);finiteSkin(rig.mesh);model.dispose();
});
test('rotated/scaled rig root yields correct global pivots',()=>{const rig=makeRig({rootRotation:true}),{human,parts}=humanScene();const model=new RetargetedHuman(rig,human,parts);for(const segment of SEGMENTS)near(model.mapping[segment].getWorldPosition(new THREE.Vector3()),parts.get('person/'+segment).position);finiteSkin(rig.mesh);model.dispose();});
test('edited and recorded poses follow current body transforms without bind drift',()=>{
  const rig=makeRig(),{human,parts}=humanScene(),model=new RetargetedHuman(rig,human,parts);
  const delta=new THREE.Matrix4().makeRotationX(.8);delta.setPosition(400,-200,500);
  for(const part of parts.values()){part.updateMatrix();const m=delta.clone().multiply(part.matrix);m.decompose(part.position,part.quaternion,part.scale);part.updateMatrixWorld(true);}
  for(let frame=0;frame<20;frame++){model.update(frame/30);for(const segment of SEGMENTS)near(model.mapping[segment].getWorldPosition(new THREE.Vector3()),parts.get('person/'+segment).position);finiteSkin(rig.mesh);}
  model.dispose();
});
test('actual mannequin joint offsets follow standing, raised arms, sitting and floor poses',()=>{
  const poses=humanPhysicsFixture();
  const {human,parts}=humanScene();human.rest_parts=poses.standing.parts.map(p=>({id:p.id,pose:p.pose}));human.joints=poses.standing.joints;
  const rig=makeRig(),model=new RetargetedHuman(rig,human,parts);
  for(const [pose,data] of Object.entries(poses)){
    for(const p of data.parts){const part=parts.get('person/'+p.id);part.position.fromArray(p.pose.position_mm);part.rotation.set(...p.pose.rotation_deg.map(THREE.MathUtils.degToRad),'ZYX');part.updateMatrixWorld(true);}
    model.update(data.time_s||0);
    for(const segment of SEGMENTS){
      const part=parts.get('person/'+segment),joint=human.joints.find(j=>j.b.part===segment),offset=new THREE.Vector3(...(joint?.b.frame.position_mm||[0,0,0]));
      near(model.mapping[segment].getWorldPosition(new THREE.Vector3()),offset.applyMatrix4(part.matrixWorld));
    }
    finiteSkin(rig.mesh);assert.equal(model.root.visible,true,pose);
  }
  model.dispose();
});
test('recorded fidget motion stays aligned and unchanged by skin/collision toggles',()=>{
  const poses=humanPhysicsFixture(),frames=Object.values(poses).filter(p=>p.simulation),{human,parts}=humanScene();human.rest_parts=poses.standing.parts.map(p=>({id:p.id,pose:p.pose}));human.joints=poses.standing.joints;
  const rig=makeRig(),model=new RetargetedHuman(rig,human,parts);
  assert.equal(frames.length,3);assert.notDeepEqual(frames[0].parts.find(p=>p.id==='left_hand').pose,frames[2].parts.find(p=>p.id==='left_hand').pose,'actual physics moves the arm');
  for(const frame of frames){
    for(const p of frame.parts){const part=parts.get('person/'+p.id);part.position.fromArray(p.pose.position_mm);part.rotation.set(...p.pose.rotation_deg.map(THREE.MathUtils.degToRad),'ZYX');part.updateMatrixWorld(true);}
    model.update(frame.time_s);const before=rig.mesh.getVertexPosition(250,new THREE.Vector3()).applyMatrix4(rig.mesh.matrixWorld),transforms=[...parts.values()].map(p=>p.matrixWorld.clone());
    model.setEnabled(false);model.setEnabled(true);model.update(frame.time_s);
    near(before,rig.mesh.getVertexPosition(250,new THREE.Vector3()).applyMatrix4(rig.mesh.matrixWorld));assert.deepEqual([...parts.values()].map(p=>p.matrixWorld),transforms);finiteSkin(rig.mesh);
  }
  model.dispose();
});
test('bad mappings fail explicitly, including repeated bones and missing required limbs',()=>{
  const rig=makeRig();assert.throws(()=>mapHumanoidBones(rig,{head:'Absent'}),/not found/);assert.throws(()=>mapHumanoidBones(rig,{head:rig.bones.pelvis.name}),/more than one/);
  rig.parser.associations.set(rig.mesh,{nodes:99});assert.throws(()=>mapHumanoidBones(rig,{head:99}),/not a skin bone/);
  const invalidVrm=makeRig({variant:'vrm'});invalidVrm.parser.associations.set(invalidVrm.mesh,{nodes:99});invalidVrm.parser.json.extensions.VRMC_vrm.humanoid.humanBones.head.node=99;assert.throws(()=>mapHumanoidBones(invalidVrm),/not a skin bone/);
  rig.bones.left_forearm.removeFromParent();const {human,parts}=humanScene();assert.throws(()=>new RetargetedHuman(rig,human,parts),/left_forearm/);
});
test('VRM node indexes disambiguate unnamed/duplicate bones',()=>{const rig=makeRig({variant:'vrm'});for(const bone of Object.values(rig.bones))bone.name='Bone';const {mapping,missing}=mapHumanoidBones(rig);assert.deepEqual(missing,[]);assert.equal(mapping.left_hand,rig.bones.left_hand);});
test('bone inventory suggestions match unique authored names and ignore unnamed or ambiguous bones',()=>{
  const inventory=[{name:'mixamorig:Hips',index:2,named:true},{name:'Head',index:8,named:true},{name:'Bone 23',index:23,named:false},{name:'Arm.L',index:30,named:true},{name:'Arm.L',index:31,named:true}];
  assert.deepEqual(suggestBoneMappings(inventory),{pelvis:2,head:8});
  assert.deepEqual(suggestBoneMappings(inventory,{pelvis:23}),{pelvis:23,head:8});
});
test('bone name heuristics use contained anatomy words, L/R markers, and parent chains',()=>{
  const inventory=[
    {name:'Bone 0',index:0,named:false},{name:'Bone 1',index:1,named:false,parent_bone:0},
    {name:'Bone 2',index:2,named:false,parent_bone:1},{name:'Bone 3',index:3,named:false,parent_bone:2},
    {name:'Character_Head_Control',index:4,named:true,parent_bone:3},
    {name:'Rig_L_Shoulder_Joint',index:10,named:true,parent_bone:2},{name:'Bone 11',index:11,named:false,parent_bone:10},
    {name:'Bone 12',index:12,named:false,parent_bone:11},{name:'Bone 13',index:13,named:false,parent_bone:12},
    {name:'RightShoulderControl',index:20,named:true,parent_bone:2},{name:'Bone 21',index:21,named:false,parent_bone:20},
    {name:'Bone 22',index:22,named:false,parent_bone:21},{name:'Bone 23',index:23,named:false,parent_bone:22},
  ];
  const result=suggestBoneMappings(inventory);
  assert.deepEqual([result.pelvis,result.lumbar,result.thorax,result.neck,result.head],[0,1,2,3,4]);
  assert.deepEqual([result.left_clavicle,result.left_upper_arm,result.left_forearm,result.left_hand],[10,11,12,13]);
  assert.deepEqual([result.right_clavicle,result.right_upper_arm,result.right_forearm,result.right_hand],[20,21,22,23]);
});
test('bone diagram labels unnamed nodes, draws hierarchy, and identifies mapped joints',()=>{
  const inventory=[{name:'Hips',named:true,index:2,parent_bone:null,position:[0,0,0]},{name:'Bone 23',named:false,index:23,parent_bone:2,position:[0,1,0]}];
  const svg=boneDiagramSvg(inventory,{pelvis:2});
  assert.match(svg,/Imported bone hierarchy/);assert.match(svg,/Bone 23/);assert.match(svg,/data-bone-index="23"/);assert.match(svg,/→ pelvis/);assert.match(svg,/<line /);
});
test('skin hit picking maps arms, legs and extra hair to their anatomical bodies',()=>{
  const rig=makeRig(),{human,parts}=humanScene(),model=new RetargetedHuman(rig,human,parts);
  const boneNames=Object.keys(rig.bones);
  for(const [bone,expected] of [['left_upper_arm','left_upper_arm'],['right_shin','right_shin'],['HairTip','head'],['IndexFinger','left_hand']]){
    const index=boneNames.indexOf(bone)*36,face={a:index,b:index+1,c:index+2};
    const point=[face.a,face.b,face.c].reduce((v,i)=>v.add(rig.mesh.getVertexPosition(i,new THREE.Vector3())),new THREE.Vector3()).multiplyScalar(1/3).applyMatrix4(rig.mesh.matrixWorld);
    assert.equal(model.partForHit({object:rig.mesh,face,point}),'person/'+expected);
  }
  const accessory=new THREE.Mesh(new THREE.BoxGeometry(),new THREE.MeshBasicMaterial());assert.equal(model.partForHit({object:accessory,point:parts.get('person/right_hand').position.clone()}),'person/right_hand');model.dispose();
});
test('fixed extra bones preserve local bind; configured angle is relative to bind',()=>{
  const rig=makeRig(),{human,parts}=humanScene();human.render_model.extra_bones={Pocket:{mode:'fixed_angle_to_parent',rotation_deg:[30,0,0]}};
  const finger=rig.bones.IndexFinger.quaternion.clone(),model=new RetargetedHuman(rig,human,parts);model.update(1);
  assert.ok(rig.bones.IndexFinger.quaternion.angleTo(finger)<1e-7);assert.ok(Math.abs(rig.bones.Pocket.quaternion.angleTo(new THREE.Quaternion())-Math.PI/6)<1e-7);model.dispose();
});
test('mapped bone offsets apply after retargeting and can correct a shoulder axis',()=>{
  const rig=makeRig(),{human,parts}=humanScene();human.render_model.bone_offsets={left_clavicle:[15,-10,25]};
  const model=new RetargetedHuman(rig,human,parts),binding=model.bindings.find(item=>item.segment==='left_clavicle');model.update(0);
  const expected=parts.get('person/left_clavicle').getWorldQuaternion(new THREE.Quaternion()).multiply(binding.rotationOffset)
    .multiply(new THREE.Quaternion().setFromEuler(new THREE.Euler(...[15,-10,25].map(THREE.MathUtils.degToRad),'XYZ')));
  assert.ok(binding.bone.getWorldQuaternion(new THREE.Quaternion()).angleTo(expected)<1e-7);model.dispose();
});
for(const mode of ['weighted_ball_joint','damped_spring'])test(`${mode} is bounded, paused-stable and resets reproducibly when scrubbing`,()=>{
  const rig=makeRig(),{human,parts}=humanScene();human.render_model.extra_bones={Hair:{mode,limit_deg:18},Pocket:{mode,limit_deg:20}};const model=new RetargetedHuman(rig,human,parts);
  for(let frame=0;frame<120;frame++){parts.get('person/head').position.x=100*Math.sin(frame/15);model.update(frame/60);for(const extra of model.extras){assert.ok(extra.angle.length()<=THREE.MathUtils.degToRad(extra.settings.limit_deg||70)+1e-6);assert.ok(extra.angle.toArray().every(Number.isFinite));}}
  assert.ok(model.extras.find(e=>e.bone.name==='Pocket').angle.length()>0,'gravity drives secondary motion');const paused=rig.bones.Hair.quaternion.clone();model.update(119/60);assert.ok(paused.angleTo(rig.bones.Hair.quaternion)<1e-7);
  model.update(0);assert.equal(model.extras.find(e=>e.bone.name==='Hair').angle.length(),0);model.dispose();
});
test('GLB exporter/loader round trip retains skeleton weights and retargets',async()=>{
  const source=makeRig(),bytes=await exportRig(source),gltf=await new GLTFLoader().parseAsync(bytes,'');const {human,parts}=humanScene();human.render_model.bone_map={head:'mixamorig:Head'};const model=new RetargetedHuman(gltf,human,parts);assert.equal(model.inventory.filter(b=>b.mapped).length,19);gltf.scene.traverse(mesh=>{if(mesh.isSkinnedMesh){assert.ok(mesh.geometry.attributes.uv);finiteSkin(mesh);}});model.dispose();
});
test('extra overrides resolve original GLB names and explicit node indexes after name sanitizing',async()=>{
  const source=makeRig();source.bones.Hair.name='mixamorig:Hair';source.bones.Pocket.name='Pocket.L';
  const gltf=await new GLTFLoader().parseAsync(await exportRig(source),'');const {human,parts}=humanScene();
  const pocketIndex=gltf.parser.json.nodes.findIndex(n=>n.name==='Pocket.L');
  human.render_model.extra_bones={'mixamorig:Hair':{mode:'fixed_angle_to_parent',rotation_deg:[35,0,0]},[pocketIndex]:{mode:'damped_spring'}};
  const model=new RetargetedHuman(gltf,human,parts),hair=model.extras.find(e=>e.bone.name==='mixamorigHair');
  assert.equal(hair.mode,'fixed_angle_to_parent');assert.ok(Math.abs(hair.bone.quaternion.angleTo(hair.bind)-THREE.MathUtils.degToRad(35))<1e-7);assert.equal(model.extras.find(e=>e.bone.name==='PocketL').mode,'damped_spring');model.dispose();
});
test('disabled imports skip loading; load failures leave simple meshes visible',async()=>{
  const {human,parts}=humanScene();human.render_model.url='/test.glb';let loads=0;const errors=[];const layer=new HumanModelLayer({scene:new THREE.Group(),loader:{async loadAsync(){loads++;throw new Error('Bad skin');}},onError:m=>errors.push(m)});
  human.render_model.enabled=false;await layer.rebuild([human],parts);assert.equal(loads,0);human.render_model.enabled=true;const result=await layer.rebuild([human],parts);assert.equal(loads,1);assert.equal(result.errors.length,1);assert.match(errors[0],/Bad skin/);for(const part of parts.values())assert.equal(part.children[0].visible,true);
});
test('missing server assets report a recoverable load error without loading or hiding the mannequin',async()=>{
  const {human,parts}=humanScene();human.render_model.load_error='The imported file is missing';let loads=0;const errors=[];const layer=new HumanModelLayer({scene:new THREE.Group(),loader:{loadAsync(){loads++;throw new Error('Should not load');}},onError:m=>errors.push(m)});
  const result=await layer.rebuild([human],parts);assert.equal(loads,0);assert.equal(result.errors.length,1);assert.match(errors[0],/file is missing/);for(const part of parts.values())assert.equal(part.children[0].visible,true);
  human.render_model.enabled=false;assert.deepEqual((await layer.rebuild([human],parts)).errors,[]);assert.equal(errors.length,1);
});
test('stale asynchronous imports cannot hide the replacement scene',async()=>{
  const {human,parts}=humanScene();human.render_model.url='/test.glb';let finish;const loader={loadAsync:()=>new Promise(resolve=>{finish=resolve;})},layer=new HumanModelLayer({scene:new THREE.Group(),loader});const loading=layer.rebuild([human],parts);layer.dispose();finish(makeRig());await loading;assert.equal(layer.models.size,0);for(const part of parts.values())assert.equal(part.children[0].visible,true);
});
test('interactive preview replaces one human model and keeps the simple mannequin recoverable',async()=>{
  const {human,parts}=humanScene(),scene=new THREE.Group();human.render_model.url='/test.glb';
  const layer=new HumanModelLayer({scene,loader:{loadAsync:async()=>makeRig()}});await layer.rebuild([human],parts);const first=layer.models.get(human.id);
  const config={...human.render_model,bone_offsets:{left_clavicle:[0,0,20]}};const preview=await layer.preview(human,config,parts);
  assert.notEqual(preview,first);assert.deepEqual(preview.config.bone_offsets.left_clavicle,[0,0,20]);for(const part of parts.values())assert.equal(part.children[0].visible,false);
  layer.dispose();for(const part of parts.values())assert.equal(part.children[0].visible,true);
});
