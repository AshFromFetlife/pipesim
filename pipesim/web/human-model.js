// Render-only glTF/VRM skins driven by the existing 19 collision bodies.
// Bone inverses, vertex weights, UVs and materials remain owned by GLTFLoader.
import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';

export const HUMAN_BONE_SEGMENTS = [
  ['pelvis','Hips / pelvis'],['lumbar','Spine'],['thorax','Chest'],['neck','Neck'],['head','Head'],
  ...['left','right'].flatMap(side=>[['clavicle','Shoulder'],['upper_arm','Upper arm'],['forearm','Forearm'],['hand','Hand'],['thigh','Thigh'],['shin','Shin'],['foot','Foot']].map(([part,label])=>[side+'_'+part,side[0].toUpperCase()+side.slice(1)+' '+label.toLowerCase()])),
];
export const EXTRA_BONE_MODES = [
  ['fixed_to_parent','Fixed to parent'],['fixed_angle_to_parent','Fixed angle to parent'],
  ['weighted_ball_joint','Weighted ball joints'],['damped_spring','Damped weighted spring'],
];
const VRM_NAMES = {pelvis:'hips',lumbar:'spine',thorax:'chest',neck:'neck',head:'head'};
for(const side of ['left','right'])for(const [segment,bone] of [['clavicle','Shoulder'],['upper_arm','UpperArm'],['forearm','LowerArm'],['hand','Hand'],['thigh','UpperLeg'],['shin','LowerLeg'],['foot','Foot']])VRM_NAMES[side+'_'+segment]=side+bone;
const ALIASES = {
  pelvis:['hips','hip','pelvis','roothips'],lumbar:['spine','spine01','spine1','abdomen','lowerback'],
  thorax:['chest','upperchest','spine2','spine02','spine3','spine03','upperback','chestupper'],
  neck:['neck','neck1','neck01'],head:['head','head1','head01'],
};
for(const side of ['left','right']){
  const short=side[0];
  for(const [segment,names] of Object.entries({clavicle:['shoulder','clavicle','collar'],upper_arm:['upperarm','arm'],forearm:['lowerarm','forearm'],hand:['hand','wrist'],thigh:['upperleg','upleg','thigh'],shin:['lowerleg','leg','calf','shin'],foot:['foot','ankle']}))
    ALIASES[side+'_'+segment]=names.flatMap(name=>[side+name,name+side,short+name,name+short]);
}
for(const [segment] of HUMAN_BONE_SEGMENTS)ALIASES[segment].push(segment.replaceAll('_',''));
const NEXT = {pelvis:'lumbar',lumbar:'thorax',thorax:'neck',neck:'head'};
for(const side of ['left','right'])Object.assign(NEXT,{[side+'_clavicle']:side+'_upper_arm',[side+'_upper_arm']:side+'_forearm',[side+'_forearm']:side+'_hand',[side+'_thigh']:side+'_shin',[side+'_shin']:side+'_foot'});
const REQUIRED = ['pelvis','head',...['left','right'].flatMap(side=>['upper_arm','forearm','hand','thigh','shin','foot'].map(part=>side+'_'+part))];
const Y = new THREE.Vector3(0,1,0), Z = new THREE.Vector3(0,0,1);
const radians=THREE.MathUtils.degToRad;
const canonical=name=>String(name||'').toLowerCase().replace(/^.*[|:]/,'').replace(/^(mixamorig|bip0?1|def|org|j_bip_c|j_bip_l|j_bip_r)[_. -]*/,'').replace(/[^a-z0-9]/g,'');
const xml=value=>String(value??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const KEYWORDS={pelvis:['pelvis','hips','hip'],lumbar:['abdomen','lowerback','spine'],thorax:['upperchest','chest','upperback','spine'],neck:['neck'],head:['head'],clavicle:['shoulder','clavicle','collar'],upper_arm:['upperarm','arm'],forearm:['forearm','lowerarm'],hand:['hand','wrist'],thigh:['thigh','upperleg','upleg'],shin:['shin','calf','lowerleg','leg'],foot:['foot','ankle']};
const CHAINS=[['pelvis','lumbar','thorax','neck','head'],...['left','right'].flatMap(side=>[[side+'_clavicle',side+'_upper_arm',side+'_forearm',side+'_hand'],[side+'_thigh',side+'_shin',side+'_foot']])];
function boneSide(name){
  const raw=String(name||'').replace(/([a-z])([A-Z])/g,'$1 $2').toLowerCase(),tokens=raw.split(/[^a-z0-9]+/).filter(Boolean),compact=canonical(name);
  if(compact.includes('left')||tokens.includes('l'))return 'left';if(compact.includes('right')||tokens.includes('r'))return 'right';
  const terms=new Set(Object.values(KEYWORDS).flat());
  if([...terms].some(term=>compact.startsWith('l'+term)||compact.endsWith(term+'l')))return 'left';
  if([...terms].some(term=>compact.startsWith('r'+term)||compact.endsWith(term+'r')))return 'right';return null;
}
function boneNameScore(name,segment){
  const compact=canonical(name),side=segment.startsWith('left_')?'left':segment.startsWith('right_')?'right':null,foundSide=boneSide(name),part=side?segment.slice(side.length+1):segment;
  if(side&&foundSide!==side||!side&&foundSide)return -1;
  if(part==='upper_arm'&&(compact.includes('forearm')||compact.includes('lowerarm')))return -1;
  if(part==='shin'&&['upperleg','upleg','thigh'].some(value=>compact.includes(value)))return -1;
  const scores=[];for(const alias of ALIASES[segment]){if(compact===alias)scores.push(140+alias.length);else if(compact.includes(alias))scores.push(90+alias.length);}
  for(const keyword of KEYWORDS[part]){if(compact===keyword)scores.push(120+keyword.length);else if(compact.includes(keyword))scores.push(60+keyword.length);}
  return scores.length?Math.max(...scores)+(side?25:0):-1;
}

/** Return explicit node-index choices for unique, recognizable authored names. */
export function suggestBoneMappings(inventory,current={}){
  const result={...current},used=new Set(),findValue=value=>inventory.find(b=>
    typeof value==='number'?b.index===value:b.name===value||canonical(b.name)===canonical(value));
  for(const value of Object.values(result)){const bone=findValue(value);if(bone)used.add(bone);}
  const proposals=[];
  for(const [segment] of HUMAN_BONE_SEGMENTS){
    if(result[segment]!==undefined&&result[segment]!==null&&result[segment]!=='')continue;
    const scored=inventory.filter(b=>b.named!==false&&!used.has(b)).map(b=>[b,boneNameScore(b.name,segment)]).filter(([,score])=>score>=0);
    if(!scored.length)continue;const best=Math.max(...scored.map(([,score])=>score)),matches=scored.filter(([,score])=>score===best);
    if(matches.length===1)proposals.push([best,segment,matches[0][0]]);
  }
  proposals.sort((a,b)=>b[0]-a[0]);for(const [,segment,bone] of proposals)if(!(segment in result)&&!used.has(bone)){result[segment]=bone.index==null?bone.name:bone.index;used.add(bone);}
  const parentOf=bone=>{
    const index=bone.parent_bone??bone.parent_index??(typeof bone.parent==='number'?bone.parent:null);
    if(index!=null)return inventory.find(item=>item.index===index);
    const name=bone.parent_name||bone.parent,matches=inventory.filter(item=>item.name===name);return matches.length===1?matches[0]:null;
  },childrenOf=bone=>inventory.filter(item=>parentOf(item)===bone);
  let changed=true;while(changed){changed=false;for(const chain of CHAINS)for(let i=0;i<chain.length-1;i++){
    const before=chain[i],after=chain[i+1],a=findValue(result[before]),b=findValue(result[after]);
    if(a&&!(after in result)){const candidates=childrenOf(a).filter(item=>!used.has(item));if(candidates.length===1){const child=candidates[0];result[after]=child.index==null?child.name:child.index;used.add(child);changed=true;}}
    if(b&&!(before in result)){const parent=parentOf(b);if(parent&&!used.has(parent)){result[before]=parent.index==null?parent.name:parent.index;used.add(parent);changed=true;}}
  }}
  return result;
}

/** A dependency-free SVG skeleton for the mapping dialog and its DOM tests. */
export function boneDiagramSvg(inventory,mapping={}){
  const bones=inventory.filter(b=>b&&Number.isInteger(b.index)&&b.index>=0),byIndex=new Map(bones.map(b=>[b.index,b]));
  if(!bones.length)return '<p class="bone-diagram-empty">No bone hierarchy is available for this import.</p>';
  const mapped=new Map();
  for(const [segment,value] of Object.entries(mapping)){
    const bone=typeof value==='number'?byIndex.get(value):bones.find(b=>b.name===value||canonical(b.name)===canonical(value));
    if(bone)mapped.set(bone.index,segment);
  }
  const width=720,height=Math.max(340,Math.min(720,140+bones.length*8)),margin=34,points=new Map();
  const positioned=bones.every(b=>Array.isArray(b.position)&&b.position.length===3&&b.position.every(Number.isFinite));
  if(positioned){
    const pairs=[[0,1],[0,2],[2,1]],ranges=axis=>Math.max(...bones.map(b=>b.position[axis]))-Math.min(...bones.map(b=>b.position[axis]));
    const [horizontal,vertical]=pairs.sort((a,b)=>ranges(b[0])*ranges(b[1])-ranges(a[0])*ranges(a[1]))[0];
    const xs=bones.map(b=>b.position[horizontal]),ys=bones.map(b=>b.position[vertical]),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys);
    const scale=Math.min((width-margin*2)/(maxX-minX||1),(height-margin*2)/(maxY-minY||1));
    for(const bone of bones)points.set(bone.index,[width/2+(bone.position[horizontal]-(minX+maxX)/2)*scale,height/2-(bone.position[vertical]-(minY+maxY)/2)*scale]);
  }else{
    const depth=new Map(),levelOf=bone=>{if(depth.has(bone.index))return depth.get(bone.index);const parent=byIndex.get(bone.parent_bone??bone.parent);const value=parent?levelOf(parent)+1:0;depth.set(bone.index,value);return value;};
    const levels=new Map();for(const bone of bones){const level=levelOf(bone);if(!levels.has(level))levels.set(level,[]);levels.get(level).push(bone);}
    const maxDepth=Math.max(...levels.keys(),1);for(const [level,row] of levels)row.forEach((bone,i)=>points.set(bone.index,[width*(i+1)/(row.length+1),margin+(height-margin*2)*level/maxDepth]));
  }
  const parentOf=bone=>byIndex.get(bone.parent_bone??bone.parent);
  const lines=bones.map(bone=>{const parent=parentOf(bone),a=parent&&points.get(parent.index),b=points.get(bone.index);return a?`<line x1="${a[0]}" y1="${a[1]}" x2="${b[0]}" y2="${b[1]}"/>`:'';}).join('');
  const nodes=bones.map((bone,i)=>{const [x,y]=points.get(bone.index),label=bone.named===false||!bone.name?`Bone ${bone.index}`:bone.name,segment=mapped.get(bone.index);return `<g class="bone-node${segment?' mapped':''}" data-bone-index="${bone.index}" transform="translate(${x} ${y})"><circle r="4"/><text x="${x>width*.62?-7:7}" y="${i%2?-6:12}" text-anchor="${x>width*.62?'end':'start'}">${xml(label)}${segment?` <tspan>→ ${xml(segment.replaceAll('_',' '))}</tspan>`:''}</text><title>${xml(label)} · node ${bone.index}${segment?' · '+xml(segment):''}</title></g>`;}).join('');
  return `<svg class="bone-diagram" viewBox="0 0 ${width} ${height}" role="img" aria-label="Imported bone hierarchy with node labels"><g class="bone-links">${lines}</g>${nodes}</svg>`;
}
function matrixOf(pose={}){return new THREE.Matrix4().compose(new THREE.Vector3(...(pose.position_mm||[0,0,0])),new THREE.Quaternion().setFromEuler(new THREE.Euler(...(pose.rotation_deg||[0,0,0]).map(radians),'ZYX')),new THREE.Vector3(1,1,1));}
function safeNumber(value,fallback,min,max){return Number.isFinite(Number(value))?THREE.MathUtils.clamp(Number(value),min,max):fallback;}
function depth(node){let n=0;for(let parent=node.parent;parent;parent=parent.parent)n++;return n;}

/** glTF node indexes are accepted alongside names so duplicate bone names work. */
export function mapHumanoidBones(gltf,manual={}){
  const all=[];gltf.scene.traverse(node=>{if(node.isBone)all.push(node);});
  const associations=gltf.parser?.associations;
  const byIndex=new Map();
  gltf.scene.traverse(node=>{const index=associations?.get(node)?.nodes;if(index!==undefined)byIndex.set(index,node);});
  const vrm=gltf.parser?.json?.extensions||gltf.userData?.gltfExtensions||{};
  const semantic=vrm.VRMC_vrm?.humanoid?.humanBones||Object.fromEntries((vrm.VRM?.humanoid?.humanBones||[]).map(b=>[b.bone,{node:b.node}]));
  const inventory=all.map(b=>{let parent=b.parent;while(parent&&!parent.isBone)parent=parent.parent;return {name:b.name,index:associations?.get(b)?.nodes,named:!!b.name,parent_bone:parent?associations?.get(parent)?.nodes:null,parent_name:parent?.name||''};});
  const semanticSeed={};for(const [segment] of HUMAN_BONE_SEGMENTS){let value=semantic[VRM_NAMES[segment]];if(segment==='thorax')value=semantic.upperChest||value;if(value)semanticSeed[segment]=value.node;}
  const automatic=suggestBoneMappings(inventory,semanticSeed);
  const mapping={},used=new Set();
  for(const [segment] of HUMAN_BONE_SEGMENTS){
    const explicit=manual[segment];let bone;
    if(explicit!==undefined&&explicit!==null&&explicit!==''){
      // GLTFLoader sanitizes ':' and '.' in node names. The import inspector
      // lists original glTF names, so resolve those through node associations.
      const originalIndex=typeof explicit==='string'?gltf.parser?.json?.nodes?.findIndex(node=>node.name===explicit):-1;
      bone=typeof explicit==='number'?byIndex.get(explicit):all.find(b=>b.name===explicit)||(originalIndex>=0?byIndex.get(originalIndex):undefined);
      if(!bone&&typeof explicit==='string'){
        const matches=all.filter(b=>canonical(b.name)===canonical(explicit));if(matches.length===1)bone=matches[0];
      }
      if(!bone)throw new Error(`Mapped ${segment} bone “${explicit}” was not found in the model.`);
      if(!bone.isBone)throw new Error(`Mapped ${segment} node “${explicit}” is not a skin bone.`);
    }else{
      const value=automatic[segment];bone=typeof value==='number'?byIndex.get(value):all.find(b=>b.name===value);
      if(bone&&!bone.isBone)throw new Error(`The VRM ${VRM_NAMES[segment]} node is not a skin bone.`);
    }
    if(bone){if(used.has(bone))throw new Error(`The bone “${bone.name}” is mapped to more than one body segment.`);mapping[segment]=bone;used.add(bone);}
  }
  return {mapping,inventory:inventory.map(item=>({...item,mapped:Object.keys(mapping).find(k=>mapping[k]===byIndex.get(item.index))||null})),missing:REQUIRED.filter(s=>!mapping[s])};
}

function restAnchor(segment,rest,joints){
  const joint=joints.find(j=>j.b?.part===segment);
  return joint?new THREE.Vector3(...joint.b.frame.position_mm).applyMatrix4(rest.get(segment)):new THREE.Vector3().setFromMatrixPosition(rest.get(segment));
}
function worldPose(node,position,quaternion){
  node.parent?.updateWorldMatrix(true,false);
  if(node.parent){
    node.position.copy(node.parent.worldToLocal(position.clone()));
    node.quaternion.copy(node.parent.getWorldQuaternion(new THREE.Quaternion()).invert()).multiply(quaternion);
  }else{node.position.copy(position);node.quaternion.copy(quaternion);}
  node.updateMatrix();node.updateWorldMatrix(false,false);
}
function disposeTree(root){
  const geometries=new Set(),materials=new Set(),textures=new Set(),skeletons=new Set();
  root.traverse(node=>{if(node.geometry)geometries.add(node.geometry);if(node.skeleton)skeletons.add(node.skeleton);for(const material of Array.isArray(node.material)?node.material:node.material?[node.material]:[]){materials.add(material);for(const value of Object.values(material))if(value?.isTexture)textures.add(value);}});
  for(const value of [...geometries,...materials,...textures,...skeletons])value.dispose?.();
  root.removeFromParent();
}

export class RetargetedHuman {
  constructor(gltf,human,partObjects,{deferVisibility=false}={}){
    this.human=human;this.partObjects=partObjects;this.config=human.render_model||{};
    const {mapping,inventory,missing}=mapHumanoidBones(gltf,this.config.bone_map||{});
    if(missing.length)throw new Error('Map these humanoid bones before enabling the imported model: '+missing.join(', '));
    this.mapping=mapping;this.inventory=inventory;this.segmentByBone=new Map(Object.entries(mapping).map(([segment,bone])=>[bone,segment]));this.root=new THREE.Group();this.root.name=human.id+' imported appearance';this.root.add(gltf.scene);
    this.root.userData.humanModel=human.id;
    const meshes=[];gltf.scene.traverse(node=>{if(node.isSkinnedMesh)meshes.push(node);});
    if(!meshes.length)throw new Error('This model has no vertex-weighted skin. Export a rigged glTF / GLB or VRM character.');
    // Restore the actual skin bind pose, not frame zero of an authored animation.
    for(const mesh of meshes)mesh.skeleton.pose();
    this.root.updateMatrixWorld(true);
    const rests=new Map(human.rest_parts.map(p=>[p.id,matrixOf(p.pose)])),joints=human.joints||[];
    for(const [segment] of HUMAN_BONE_SEGMENTS)if(mapping[segment]&&!rests.has(segment))throw new Error('Missing neutral body geometry for '+segment);
    const anchors=new Map([...rests.keys()].map(segment=>[segment,restAnchor(segment,rests,joints)]));
    // Derive axes from anatomy: this covers Y-up, Z-up, T/A poses and rig roots
    // with a baked rotation without relying on an exporter-specific convention.
    const hips=mapping.pelvis.getWorldPosition(new THREE.Vector3());
    const up=mapping.head.getWorldPosition(new THREE.Vector3()).sub(hips).normalize();
    const right=mapping.right_thigh.getWorldPosition(new THREE.Vector3()).sub(mapping.left_thigh.getWorldPosition(new THREE.Vector3()));
    right.addScaledVector(up,-right.dot(up)).normalize();
    if(up.lengthSq()<.9||right.lengthSq()<.9)throw new Error('The bind pose must have distinct hips, head and left/right thighs.');
    const forward=up.clone().cross(right).normalize();
    const sourceBasis=new THREE.Matrix4().makeBasis(right,forward,up);
    const alignment=new THREE.Quaternion().setFromRotationMatrix(sourceBasis).invert();
    const sourceHeadDistance=mapping.head.getWorldPosition(new THREE.Vector3()).distanceTo(hips);
    const targetHeadDistance=anchors.get('head').distanceTo(anchors.get('pelvis'));
    const scale=targetHeadDistance/sourceHeadDistance;
    if(!Number.isFinite(scale)||scale<=0)throw new Error('The model bind pose has invalid dimensions.');
    this.root.quaternion.copy(alignment);this.root.scale.setScalar(scale);
    this.root.position.copy(anchors.get('pelvis')).sub(hips.clone().applyQuaternion(alignment).multiplyScalar(scale));
    this.root.updateMatrixWorld(true);
    this.bindings=[];
    for(const [segment,bone] of Object.entries(mapping)){
      const rest=rests.get(segment),anchor=anchors.get(segment),position=bone.getWorldPosition(new THREE.Vector3());
      const quaternion=bone.getWorldQuaternion(new THREE.Quaternion());
      let next=NEXT[segment];
      // A chest-only or spine-only rig remains usable; helpers keep their binds.
      while(next&&!mapping[next])next=NEXT[next];
      let sourceDirection,targetDirection;
      if(next){sourceDirection=mapping[next].getWorldPosition(new THREE.Vector3()).sub(position);targetDirection=anchors.get(next).clone().sub(anchor);}
      else if(segment.endsWith('_hand')){
        const finger=bone.children.find(b=>b.isBone&&/middle/i.test(b.name))||bone.children.find(b=>b.isBone&&/index/i.test(b.name))||bone.children.find(b=>b.isBone);
        if(finger)sourceDirection=finger.getWorldPosition(new THREE.Vector3()).sub(position);
        else {const parent=mapping[segment.replace('_hand','_forearm')];sourceDirection=position.clone().sub(parent.getWorldPosition(new THREE.Vector3()));}
        targetDirection=new THREE.Vector3(0,0,1).transformDirection(rest);
      }else if(segment.endsWith('_foot')){
        const toe=bone.children.find(b=>b.isBone&&/toe/i.test(b.name))||bone.children.find(b=>b.isBone);
        sourceDirection=toe?toe.getWorldPosition(new THREE.Vector3()).sub(position):new THREE.Vector3(0,1,0);
        // The ankle lies above the toes in a normal bind pose. Use heading only
        // here; aligning the full sloping vector would tip every shoe upward.
        sourceDirection.z=0;
        targetDirection=Y.clone().transformDirection(rest);
      }else{sourceDirection=Z.clone();targetDirection=Z.clone().transformDirection(rest);}
      if(sourceDirection?.lengthSq()>1e-8&&targetDirection?.lengthSq()>1e-8)quaternion.premultiply(new THREE.Quaternion().setFromUnitVectors(sourceDirection.normalize(),targetDirection.normalize()));
      const restQuaternion=new THREE.Quaternion().setFromRotationMatrix(rest);
      this.bindings.push({segment,bone,offset:anchor.clone().applyMatrix4(rest.clone().invert()),rotationOffset:restQuaternion.invert().multiply(quaternion)});
    }
    this.bindings.sort((a,b)=>depth(a.bone)-depth(b.bone));
    const mapped=new Set(Object.values(mapping));this.extras=[];
    gltf.scene.traverse(bone=>{
      if(!bone.isBone||mapped.has(bone))return;
      const nodeIndex=gltf.parser?.associations?.get(bone)?.nodes;
      const originalName=gltf.parser?.json?.nodes?.[nodeIndex]?.name;
      const settings=(nodeIndex!==undefined?this.config.extra_bones?.[String(nodeIndex)]:null)||
        (originalName?this.config.extra_bones?.[originalName]:null)||this.config.extra_bones?.[bone.name]||{};
      const mode=settings.mode||this.config.default_extra_mode||'fixed_to_parent';
      if(!EXTRA_BONE_MODES.some(([id])=>id===mode))throw new Error('Unknown extra bone mode: '+mode);
      // A helper/twist bone on a mapped limb chain may not acquire independent
      // dynamics: doing so detaches or twists a mapped child from its parent.
      let isAncestor=false;bone.traverse(child=>{if(child!==bone&&mapped.has(child))isAncestor=true;});
      const child=bone.children.find(n=>n.isBone);
      const tail=child?child.position.clone():new THREE.Vector3(0,80/scale,0);
      this.extras.push({bone,settings,mode:isAncestor?'fixed_to_parent':mode,bind:bone.quaternion.clone(),tail,angle:new THREE.Vector3(),velocity:new THREE.Vector3(),previousPosition:null,previousVelocity:new THREE.Vector3(),isAncestor});
    });
    this.extras.sort((a,b)=>depth(a.bone)-depth(b.bone));
    this.simpleMeshes=[];
    for(const [segment] of HUMAN_BONE_SEGMENTS){
      const part=partObjects.get(human.id+'/'+segment);
      part?.traverse(mesh=>{if(mesh.isMesh){if(mesh.userData.humanSimpleVisible===undefined)mesh.userData.humanSimpleVisible=mesh.visible;this.simpleMeshes.push({mesh,visible:mesh.userData.humanSimpleVisible});}});
    }
    this.root.traverse(node=>{if(node.isMesh){node.castShadow=true;node.receiveShadow=true;node.frustumCulled=false;node.userData.part=human.id+'/pelvis';node.userData.humanModel=human.id;}});
    this.lastTime=null;this.enabled=false;this.controlsSimpleMeshes=!deferVisibility;this.update(0);this.setEnabled(this.config.enabled!==false);
  }
  setEnabled(enabled){this.enabled=!!enabled;this.root.visible=this.enabled;if(this.controlsSimpleMeshes)for(const {mesh,visible} of this.simpleMeshes)mesh.visible=this.enabled?false:visible;}
  partForHit(hit){
    const mesh=hit.object,scores=new Map();
    const ancestorSegment=bone=>{for(let node=bone;node;node=node.parent){const segment=this.segmentByBone.get(node);if(segment)return segment;}return null;};
    if(mesh.isSkinnedMesh&&hit.face&&mesh.geometry.attributes.skinIndex&&mesh.geometry.attributes.skinWeight){
      const vertices=[hit.face.a,hit.face.b,hit.face.c],points=vertices.map(index=>mesh.getVertexPosition(index,new THREE.Vector3()));
      const point=mesh.worldToLocal(hit.point.clone());
      const barycentric=THREE.Triangle.getBarycoord(point,...points,new THREE.Vector3());
      const blends=barycentric?barycentric.toArray().map(v=>Math.max(0,v)):[1/3,1/3,1/3];
      const indices=mesh.geometry.attributes.skinIndex,weights=mesh.geometry.attributes.skinWeight;
      for(let vertex=0;vertex<3;vertex++){
        const indexVector=new THREE.Vector4().fromBufferAttribute(indices,vertices[vertex]);
        const weightVector=new THREE.Vector4().fromBufferAttribute(weights,vertices[vertex]);
        for(let influence=0;influence<4;influence++){
          const segment=ancestorSegment(mesh.skeleton.bones[indexVector.getComponent(influence)]);
          if(segment)scores.set(segment,(scores.get(segment)||0)+blends[vertex]*weightVector.getComponent(influence));
        }
      }
    }else{const segment=ancestorSegment(mesh);if(segment)scores.set(segment,1);}
    const strongest=[...scores].sort((a,b)=>b[1]-a[1])[0];
    if(strongest?.[1]>0)return this.human.id+'/'+strongest[0];
    // Unskinned accessories may sit outside the skeleton hierarchy. Use the
    // closest posed body without changing the imported mesh or collision data.
    let closest=null,distance=Infinity;
    for(const [segment] of HUMAN_BONE_SEGMENTS){const part=this.partObjects.get(this.human.id+'/'+segment);if(!part)continue;const d=part.getWorldPosition(new THREE.Vector3()).distanceToSquared(hit.point);if(d<distance){closest=this.human.id+'/'+segment;distance=d;}}
    return closest;
  }
  update(timeSeconds=0){
    // Absolute recorded time makes paused frames stable. Seeking backwards or
    // over a gap resets secondary motion rather than integrating a giant step.
    const rawDt=this.lastTime===null?0:timeSeconds-this.lastTime;
    const reset=this.lastTime===null||rawDt<0||rawDt>.25;
    const dt=reset?0:Math.max(0,rawDt);this.lastTime=timeSeconds;
    for(const extra of this.extras){extra.bone.quaternion.copy(extra.bind);extra.bone.updateMatrix();}
    this.root.updateMatrixWorld(true);
    for(const binding of this.bindings){
      const part=this.partObjects.get(this.human.id+'/'+binding.segment);if(!part)continue;
      part.updateWorldMatrix(true,false);
      const position=binding.offset.clone().applyMatrix4(part.matrixWorld);
      const adjustment=this.config.bone_offsets?.[binding.segment]||[0,0,0];
      const correction=new THREE.Quaternion().setFromEuler(new THREE.Euler(...adjustment.map(radians),'XYZ'));
      const quaternion=part.getWorldQuaternion(new THREE.Quaternion()).multiply(binding.rotationOffset).multiply(correction);
      worldPose(binding.bone,position,quaternion);
    }
    for(const extra of this.extras)this.updateExtra(extra,dt,reset);
    this.root.updateMatrixWorld(true);
    // THREE updates skeletons while rendering; also update here so CPU picking,
    // bounds and regression checks see exactly the same posed vertices.
    this.root.traverse(node=>{if(node.isSkinnedMesh)node.skeleton.update();});
  }
  updateExtra(extra,dt,reset){
    const {bone,settings,mode,bind}=extra;
    if(mode==='fixed_to_parent')return;
    const offset=new THREE.Quaternion().setFromEuler(new THREE.Euler(...(settings.rotation_deg||[0,0,0]).map(radians),'XYZ'));
    bone.quaternion.copy(bind).multiply(offset);bone.updateWorldMatrix(true,false);
    if(mode==='fixed_angle_to_parent')return;
    const position=bone.getWorldPosition(new THREE.Vector3()).multiplyScalar(.001);
    if(reset){extra.angle.set(0,0,0);extra.velocity.set(0,0,0);extra.previousPosition=position.clone();extra.previousVelocity.set(0,0,0);}
    if(dt>0){
      const parentQ=bone.parent.getWorldQuaternion(new THREE.Quaternion());
      const velocity=position.clone().sub(extra.previousPosition||position).divideScalar(dt);
      const acceleration=velocity.clone().sub(extra.previousVelocity).divideScalar(dt).clampLength(0,35);
      const gravity=new THREE.Vector3(0,0,-9.81).sub(acceleration).applyQuaternion(parentQ.invert());
      const length=Math.max(.015,extra.tail.length()*bone.getWorldScale(new THREE.Vector3()).length()/Math.sqrt(3)*.001);
      const mass=safeNumber(settings.mass_kg??settings.mass,.05,.001,20);
      const stiffness=safeNumber(settings.stiffness,mode==='weighted_ball_joint'?.08:2,0,200);
      const damping=safeNumber(settings.damping,mode==='weighted_ball_joint'?.12:.5,0,50);
      const limit=radians(safeNumber(settings.limit_deg,mode==='weighted_ball_joint'?70:25,0,170));
      const inertia=Math.max(.001,mass*length*length);
      const steps=Math.max(1,Math.ceil(dt*120)),step=dt/steps;
      for(let i=0;i<steps;i++){
        // Rotation vectors avoid Euler wraparound at the joint cone boundary.
        const amount=extra.angle.length();const rotation=new THREE.Quaternion().setFromAxisAngle(amount?extra.angle.clone().divideScalar(amount):Y,amount);
        const direction=extra.tail.clone().normalize().applyQuaternion(bind.clone().multiply(offset)).applyQuaternion(rotation);
        const torque=direction.cross(gravity).multiplyScalar(mass*length).addScaledVector(extra.angle,-stiffness).addScaledVector(extra.velocity,-damping);
        extra.velocity.addScaledVector(torque,step/inertia).clampLength(0,12);
        extra.angle.addScaledVector(extra.velocity,step);
        if(extra.angle.length()>limit){extra.angle.setLength(limit);const outward=extra.velocity.dot(extra.angle);if(outward>0)extra.velocity.addScaledVector(extra.angle,-outward/Math.max(limit*limit,1e-12));}
      }
      extra.previousPosition=position;extra.previousVelocity=velocity;
    }
    const angle=extra.angle.length();if(angle>0)bone.quaternion.premultiply(new THREE.Quaternion().setFromAxisAngle(extra.angle.clone().divideScalar(angle),angle));
    bone.updateWorldMatrix(false,true);
  }
  dispose(){this.setEnabled(false);disposeTree(this.root);}
}

/** A failed/late import never hides the editable collision mannequin. */
export class HumanModelLayer {
  constructor({scene,loader=new GLTFLoader(),onError=()=>{}}){this.scene=scene;this.loader=loader;this.onError=onError;this.models=new Map();this.generation=0;this.previewTokens=new Map();}
  async rebuild(humans=[],partObjects){
    this.dispose();const generation=this.generation,result={loaded:[],errors:[]};
    for(const human of humans)if(human.render_model?.enabled!==false&&human.render_model?.load_error){const error=human.render_model.load_error;result.errors.push({id:human.id,error});this.onError(`${human.id}: ${error}`);}
    await Promise.all(humans.filter(h=>h.render_model?.url&&!h.render_model.load_error&&h.render_model.enabled!==false).map(async human=>{
      let gltf,model;
      try{
        gltf=await this.loader.loadAsync(human.render_model.url);
        if(this.generation!==generation){disposeTree(gltf.scene);return;}
        model=new RetargetedHuman(gltf,human,partObjects);this.scene.add(model.root);this.models.set(human.id,model);result.loaded.push(human.id);
      }catch(error){if(model)model.dispose();else if(gltf)disposeTree(gltf.scene);if(this.generation===generation){const message=/DRACOLoader|MeshoptDecoder|KTX2Loader/.test(error.message)?'This file uses mesh or texture compression that is not supported. Export uncompressed glTF / GLB with PNG or JPEG textures.':error.message;result.errors.push({id:human.id,error:message});this.onError(`${human.id}: ${message}`);}}
    }));
    return result;
  }
  update(timeSeconds=0){for(const model of this.models.values())if(model.enabled)model.update(timeSeconds);}
  async preview(human,renderModel,partObjects){
    if(!renderModel?.url)return null;
    const id=human.id,token=(this.previewTokens.get(id)||0)+1;this.previewTokens.set(id,token);
    let gltf,model;
    try{
      gltf=await this.loader.loadAsync(renderModel.url);
      if(this.previewTokens.get(id)!==token){disposeTree(gltf.scene);return null;}
      model=new RetargetedHuman(gltf,{...human,render_model:{...renderModel,enabled:true}},partObjects,{deferVisibility:true});
      if(this.previewTokens.get(id)!==token){model.dispose();return null;}
      this.scene.add(model.root);const previous=this.models.get(id);this.models.set(id,model);previous?.dispose();model.controlsSimpleMeshes=true;model.setEnabled(true);model.update(0);return model;
    }catch(error){if(model)model.dispose();else if(gltf)disposeTree(gltf.scene);throw error;}
  }
  partForHit(hit){const id=hit?.object?.userData?.humanModel;return id?this.models.get(id)?.partForHit(hit)||null:null;}
  dispose(){this.generation++;this.previewTokens.clear();for(const model of this.models.values())model.dispose();this.models.clear();}
}
