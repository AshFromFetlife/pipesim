// Export real GLBs and the browser renderer's exact posed vertex positions.
// Python regression tests consume these without running a browser or web server.
import {mkdir,writeFile,readFile} from 'node:fs/promises';
import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {makeRig,exportRig,humanScene,SEGMENTS} from './human-rig-fixtures.mjs';
import {humanPhysicsFixture} from './human-physics-fixture.mjs';
import {RetargetedHuman} from '../../pipesim/web/human-model.js';

const directory=process.argv[2];await mkdir(directory,{recursive:true});
const poses=humanPhysicsFixture(),cases=[];
for(const variant of ['mixamo','blender','rotated','vrm',...process.argv.slice(3)]){
  const real=variant.endsWith('.glb'),name=real?variant.split(/[\\/]/).pop().slice(0,-4):variant;
  const source=real?null:makeRig({variant:variant==='rotated'?'mixamo':variant,rootRotation:variant==='rotated'});
  const bytes=real?new Uint8Array(await readFile(variant)).buffer:await exportRig(source);await writeFile(`${directory}/${name}.glb`,Buffer.from(bytes));
  // Pixel/material tests run in Python. CPU reference skinning does not need a
  // DOM image decoder for the optional textured real-world models.
  const gltf=await new GLTFLoader().register(()=>({name:'REFERENCE_NO_TEXTURES',loadTexture:()=>Promise.resolve(null)})).parseAsync(bytes,'');
  const {human,parts}=humanScene();human.rest_parts=poses.standing.parts.map(p=>({id:p.id,pose:p.pose}));human.joints=poses.standing.joints;
  human.render_model={file:name+'.glb',enabled:true,extra_bones:{Hair:{mode:'weighted_ball_joint',mass:.12,limit_deg:35},Pocket:{mode:'damped_spring',rotation_deg:[10,20,30]},IndexFinger:{mode:'fixed_angle_to_parent',rotation_deg:[12,-23,34]}}};
  if(variant==='vrm')human.render_model.bone_map=Object.fromEntries(SEGMENTS.map((s,i)=>[s,gltf.parser.json.nodes.findIndex(n=>n.name==='Joint_'+i)]));
  for(const p of poses.standing.parts){const part=parts.get('person/'+p.id);part.position.fromArray(p.pose.position_mm);part.rotation.set(...p.pose.rotation_deg.map(THREE.MathUtils.degToRad),'ZYX');part.updateMatrixWorld(true);}
  const model=new RetargetedHuman(gltf,human,parts),frames=[];
  const inputs=Object.entries(poses).concat(Array.from({length:8},(_,i)=>['secondary-'+i,{...poses['hands-up'],time_s:i/30}]));
  for(const [name,data] of inputs){
    for(const p of data.parts){const part=parts.get('person/'+p.id);part.position.fromArray(p.pose.position_mm);part.rotation.set(...p.pose.rotation_deg.map(THREE.MathUtils.degToRad),'ZYX');part.updateMatrixWorld(true);}
    model.update(data.time_s||0);
    const meshes=[];gltf.scene.traverse(mesh=>{if(mesh.isMesh)meshes.push({vertices:Array.from({length:mesh.geometry.attributes.position.count},(_,i)=>mesh.getVertexPosition(i,new THREE.Vector3()).applyMatrix4(mesh.matrixWorld).toArray())});});
    frames.push({name,time_s:data.time_s||0,poses:Object.fromEntries(data.parts.map(p=>['person/'+p.id,p.pose])),meshes,
      bones:Object.fromEntries(Object.entries(model.mapping).map(([s,b])=>[s,b.matrixWorld.elements.slice()]))});
  }
  cases.push({variant:name,human,frames});model.dispose();
}
await writeFile(`${directory}/reference.json`,JSON.stringify(cases));
