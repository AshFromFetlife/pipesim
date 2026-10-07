// Synthetic character fixtures with deliberately varied axes, bind poses and names.
import * as THREE from 'three';
import {GLTFExporter} from 'three/addons/exporters/GLTFExporter.js';
import {HUMAN_BONE_SEGMENTS} from '../../pipesim/web/human-model.js';
export const SEGMENTS=HUMAN_BONE_SEGMENTS.map(([id])=>id);
const xyz={pelvis:[0,0,940],lumbar:[0,0,1050],thorax:[0,0,1200],neck:[0,0,1480],head:[0,0,1600],left_clavicle:[-60,0,1410],left_upper_arm:[-230,0,1400],left_forearm:[-230,0,1080],left_hand:[-230,0,820],right_clavicle:[60,0,1410],right_upper_arm:[230,0,1400],right_forearm:[230,0,1080],right_hand:[230,0,820],left_thigh:[-100,0,940],left_shin:[-100,0,500],left_foot:[-100,0,70],right_thigh:[100,0,940],right_shin:[100,0,500],right_foot:[100,0,70]};
const parent={lumbar:'pelvis',thorax:'lumbar',neck:'thorax',head:'neck'};
for(const side of ['left','right'])Object.assign(parent,{[side+'_clavicle']:'thorax',[side+'_upper_arm']:side+'_clavicle',[side+'_forearm']:side+'_upper_arm',[side+'_hand']:side+'_forearm',[side+'_thigh']:'pelvis',[side+'_shin']:side+'_thigh',[side+'_foot']:side+'_shin'});
const vrm={pelvis:'hips',lumbar:'spine',thorax:'chest',neck:'neck',head:'head'},titles={pelvis:'Hips',lumbar:'Spine',thorax:'Spine2',neck:'Neck',head:'Head'};
for(const side of ['left','right'])for(const [id,name,title] of [['clavicle','Shoulder','Shoulder'],['upper_arm','UpperArm','Arm'],['forearm','LowerArm','ForeArm'],['hand','Hand','Hand'],['thigh','UpperLeg','UpLeg'],['shin','LowerLeg','Leg'],['foot','Foot','Foot']]){vrm[side+'_'+id]=side+name;titles[side+'_'+id]=side[0].toUpperCase()+side.slice(1)+title;}
export function humanScene(id='person'){
  const parts=new Map(),rest_parts=[],joints=[];
  for(const segment of SEGMENTS){
    const rotation=/_upper_arm$|_forearm$|_hand$|_thigh$|_shin$/.test(segment)?[180,0,0]:segment.endsWith('_clavicle')?[0,segment.startsWith('left')?-90:90,0]:[0,0,0];
    const pose={position_mm:[...xyz[segment]],rotation_deg:rotation};
    const obj=new THREE.Group();obj.position.fromArray(pose.position_mm);obj.rotation.set(...rotation.map(THREE.MathUtils.degToRad),'ZYX');obj.updateMatrixWorld(true);obj.add(new THREE.Mesh(new THREE.BoxGeometry(40,40,40),new THREE.MeshBasicMaterial()));
    parts.set(id+'/'+segment,obj);rest_parts.push({id:segment,pose});
    if(parent[segment])joints.push({id:segment+'_joint',a:{part:parent[segment],frame:{position_mm:xyz[segment].map((v,i)=>v-xyz[parent[segment]][i])}},b:{part:segment,frame:{position_mm:[0,0,0]}}});
  }
  return {human:{id,stature_mm:1750,rest_parts,joints,render_model:{enabled:true}},parts};
}
export function makeRig({variant='mixamo',tPose=true,extra=true,rootRotation=false}={}){
  const scene=new THREE.Group(),armature=new THREE.Group();scene.add(armature);
  const source=Object.fromEntries(Object.entries(xyz).map(([name,p])=>[name,new THREE.Vector3(-p[0]/1000,p[2]/1000,p[1]/1000)]));
  if(tPose)for(const side of ['left','right']){const sign=side==='left'?1:-1;source[side+'_forearm'].set(sign*.55,1.4,0);source[side+'_hand'].set(sign*.81,1.4,0);}
  const bones={},associations=new Map();
  for(const [index,segment] of SEGMENTS.entries()){
    const bone=new THREE.Bone();bone.name=variant==='vrm'?'Joint_'+index:variant==='blender'?segment.replace('left_','').replace('right_','')+(segment.startsWith('left_')?'.L':segment.startsWith('right_')?'.R':''):'mixamorig:'+titles[segment];
    const owner=parent[segment];bone.position.copy(source[segment]);if(owner)bone.position.sub(source[owner]);(owner?bones[owner]:armature).add(bone);bones[segment]=bone;associations.set(bone,{nodes:index});
  }
  if(extra)for(const [name,owner,position] of [['Hair','head',[0,0,.08]],['HairTip','Hair',[0,-.3,0]],['Pocket','pelvis',[.13,-.08,.08]],['IndexFinger','left_hand',[.09,0,0]],['LeftToe','left_foot',[0,-.02,.13]],['RightToe','right_foot',[0,-.02,.13]]]){const bone=new THREE.Bone();bone.name=name;bone.position.fromArray(position);bones[owner].add(bone);bones[name]=bone;associations.set(bone,{nodes:associations.size});}
  const meshBones=Object.values(bones),positions=[],normals=[],uvs=[],skinIndices=[],skinWeights=[];armature.updateMatrixWorld(true);
  for(const [index,bone] of meshBones.entries()){
    const center=bone.getWorldPosition(new THREE.Vector3()),next=bone.children.find(c=>c.isBone);let end=next?next.getWorldPosition(new THREE.Vector3()):center.clone().add(new THREE.Vector3(0,.1,0));
    if(bone===bones.left_hand||bone===bones.right_hand)end=center.clone().add(new THREE.Vector3(bone===bones.left_hand?.15:-.15,0,0));
    if(bone===bones.head)end=center.clone().add(new THREE.Vector3(0,.14,0));
    const direction=end.clone().sub(center),length=Math.max(.045,direction.length()),box=new THREE.BoxGeometry(index<5?.17:.065,length,index<5?.10:.065).toNonIndexed(),q=new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0,1,0),direction.normalize()),midpoint=center.clone().add(end).multiplyScalar(.5);
    for(let i=0;i<box.attributes.position.count;i++){const p=new THREE.Vector3().fromBufferAttribute(box.attributes.position,i).applyQuaternion(q).add(midpoint),n=new THREE.Vector3().fromBufferAttribute(box.attributes.normal,i).applyQuaternion(q);positions.push(...p.toArray());normals.push(...n.toArray());uvs.push(box.attributes.uv.getX(i),box.attributes.uv.getY(i));skinIndices.push(index,0,0,0);skinWeights.push(1,0,0,0);}box.dispose();
  }
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));geometry.setAttribute('normal',new THREE.Float32BufferAttribute(normals,3));geometry.setAttribute('uv',new THREE.Float32BufferAttribute(uvs,2));geometry.setAttribute('skinIndex',new THREE.Uint16BufferAttribute(skinIndices,4));geometry.setAttribute('skinWeight',new THREE.Float32BufferAttribute(skinWeights,4));
  const mesh=new THREE.SkinnedMesh(geometry,new THREE.MeshStandardMaterial({color:variant==='vrm'?0xc77789:variant==='blender'?0x9daf62:0x438fae,roughness:.7}));mesh.name='WeightedCharacter';scene.add(mesh);mesh.bind(new THREE.Skeleton(meshBones));
  if(rootRotation){scene.rotation.set(.15,.3,.2);scene.position.set(.5,.2,.1);scene.scale.setScalar(1.2);scene.updateMatrixWorld(true);mesh.bind(mesh.skeleton);}
  const json={extensions:variant==='vrm'?{VRMC_vrm:{specVersion:'1.0',humanoid:{humanBones:Object.fromEntries(SEGMENTS.map((s,i)=>[vrm[s],{node:i}]))}}}:{}};
  return {scene,parser:{associations,json},bones,mesh};
}
export async function exportRig(gltf){
  const old=globalThis.FileReader;if(!old)globalThis.FileReader=class{readAsArrayBuffer(blob){blob.arrayBuffer().then(v=>{this.result=v;this.onloadend?.();});}readAsDataURL(blob){blob.arrayBuffer().then(v=>{this.result='data:'+blob.type+';base64,'+Buffer.from(v).toString('base64');this.onloadend?.();});}};
  try{return await new GLTFExporter().parseAsync(gltf.scene,{binary:true});}finally{if(!old)delete globalThis.FileReader;}
}
