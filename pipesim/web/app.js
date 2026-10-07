import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {TransformControls} from 'three/addons/controls/TransformControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {OBJLoader} from 'three/addons/loaders/OBJLoader.js';
import {connectionCandidates,hingeCandidates,clearConnectionIntent,alignmentDelta,socketOccupied,rotationAlignment} from './snapping.js';
import {DEFAULT_SNAP_SETTINGS,loadSnapDefaults,saveSnapDefaults,validateSnapSettings} from './snap-settings.js';
import {DEFAULT_PREFERENCES,loadPreferences,savePreferences,validatePreferences,displayValue,storedValue} from './preferences.js';
import {nextFlexibleShortcutLength} from './resize-shortcuts.js';
import {draftRuns,draftRun,draftPreview} from './drafting.js';
import {HUMAN_POSES} from './human-poses.js';

const $=s=>document.querySelector(s), $$=s=>[...document.querySelectorAll(s)];
const clone=x=>structuredClone(x), esc=x=>String(x??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
let snapDefaults={...DEFAULT_SNAP_SETTINGS};try{snapDefaults=loadSnapDefaults(window.localStorage);}catch{}
let preferences={...DEFAULT_PREFERENCES};try{preferences=loadPreferences(window.localStorage);}catch{}
const LIBRARY_HOTKEYS_KEY='pipesim.library-hotkeys.v1';
const libraryHotkeys=Object.create(null);
try{const saved=JSON.parse(window.localStorage.getItem(LIBRARY_HOTKEYS_KEY)||'{}');
  if(saved&&typeof saved==='object'&&!Array.isArray(saved))for(const [catalog,hotkey] of Object.entries(saved))
    if(typeof catalog==='string'&&typeof hotkey==='string'&&/^(?:Shift\+)?[0-9]$|^Alt\+Shift\+[A-CE-Z]$/.test(hotkey)&&
       !Object.values(libraryHotkeys).includes(hotkey))libraryHotkeys[catalog]=hotkey;
}catch{}
const state={doc:null,path:'',library:{},scene:null,selected:null,connectionSource:null,mode:'design',tool:'select',undo:[],redo:[],dirty:false,revision:0,recording:null,playing:false,frame:0,checks:null,analysis:null,plan:null,step:0,busy:false,simulation:null,simulationError:null,mirrorPoseError:null,simulationOptions:null,ports:preferences.portsVisible,snap:snapDefaults.gridEnabled,connectionSnap:snapDefaults.connectionsEnabled,snapSettings:snapDefaults,placementPending:false,moveBody:true};
let token='',toastTimer,renderer,orbit,gizmo,sceneGeneration=0,dragStart=null,pointerDrag=null,resizeDrag=null,localRotationHeld=false;
const viewport=$('#viewport'), scene=new THREE.Scene(), objects=new THREE.Group(), ports=new THREE.Group(), overlays=new THREE.Group();
const snapGhost=new THREE.Group(),mirrorGhost=new THREE.Group(),resizeHandles=new THREE.Group(),resizePreview=new THREE.Group();let reviewCleanup=null,reviewVersion=0;
scene.add(objects,ports,overlays,snapGhost,mirrorGhost,resizeHandles,resizePreview);scene.background=new THREE.Color('#eef3f5');
const camera=new THREE.PerspectiveCamera(preferences.cameraFovDeg,1,1,100000);camera.up.set(0,0,1);camera.position.set(2100,-2600,1800);
const ambient=new THREE.HemisphereLight('#fbfeff','#aabac0',2.2);ambient.position.set(0,0,2000);scene.add(ambient);
const key=new THREE.DirectionalLight('#fff6e4',3.2);key.position.set(-1400,-2500,4000);key.castShadow=true;key.shadow.mapSize.set(2048,2048);key.shadow.camera.left=-3000;key.shadow.camera.right=3000;key.shadow.camera.top=3000;key.shadow.camera.bottom=-3000;key.shadow.camera.near=1;key.shadow.camera.far=12000;key.shadow.bias=-.0002;key.shadow.normalBias=1;scene.add(key);
const fill=new THREE.DirectionalLight('#e0edf7',1.3);fill.position.set(2300,1400,1800);scene.add(fill);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(30000,30000),new THREE.MeshStandardMaterial({color:'#f0f4f5',roughness:1,metalness:0}));floor.position.z=-1;floor.receiveShadow=true;scene.add(floor);
floor.visible=preferences.floorVisible;
const grid=new THREE.GridHelper(12000,120,'#c6d3d8','#dce5e9');grid.rotation.x=Math.PI/2;grid.position.z=.1;grid.material.transparent=true;grid.material.opacity=.65;scene.add(grid);
function applyViewportTheme(){
  const dark=preferences.theme==='dark'||preferences.theme==='system'&&colorPreference.matches;
  document.documentElement.dataset.viewportTheme=dark?'dark':'light';
  const styles=getComputedStyle(document.documentElement);
  const color=name=>styles.getPropertyValue('--viewport-'+name).trim();
  scene.background.set(color('background'));
  floor.material.color.set(color('floor'));
  const themedGrid=new THREE.GridHelper(12000,120,color('grid-major'),color('grid-minor'));
  grid.geometry.dispose();grid.geometry=themedGrid.geometry;themedGrid.material.dispose();
}
const colorPreference=window.matchMedia('(prefers-color-scheme: dark)');
colorPreference.addEventListener('change',applyViewportTheme);
applyViewportTheme();
grid.visible=preferences.gridVisible;
const raycaster=new THREE.Raycaster(), pointer=new THREE.Vector2(), groundPlane=new THREE.Plane(new THREE.Vector3(0,0,1),0);
const partObjects=new Map();
const beamObjects=new Map();
let hoveredDraftConnection=null;
const mirrorCopies=new Map(),mirrorPlanes=new Map();
const rotationHandle=new THREE.Object3D(),wholeObjectHandle=new THREE.Object3D();scene.add(rotationHandle,wholeObjectHandle);
const objectModes=new Map(),openObjects=new Set();
const selectedObject=()=>state.doc?.objects?.find(o=>state.selected?.startsWith(o.id+'/'));
const objectMode=object=>object.template==='chain'?(object.layout_mode==='posable'?'limb':'whole'):objectModes.get(object.id)||'whole';
const wholeObject=()=>{const object=selectedObject();return object&&objectMode(object)!=='limb'?object:null;};
try{
  renderer=new THREE.WebGLRenderer({antialias:true,alpha:false,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.15;viewport.appendChild(renderer.domElement);
  orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=true;orbit.dampingFactor=.08;orbit.target.set(0,0,500);orbit.minDistance=50;orbit.maxDistance=20000;orbit.maxPolarAngle=Math.PI*.495;
  gizmo=new TransformControls(camera,renderer.domElement);gizmo.setSize(.8);scene.add(gizmo.getHelper());updateSnapControls();
  gizmo.addEventListener('dragging-changed',e=>{orbit.enabled=!e.value;});
  gizmo.addEventListener('mouseDown',()=>{
    if(!beginPlacement('gizmo'))gizmo.detach();
  });
  gizmo.addEventListener('objectChange',()=>{
    if(!dragStart)return;const selected=partObjects.get(dragStart.id);selected.updateMatrix();
    if(state.tool==='translate'){
      const handle=gizmo.object,origin=new THREE.Vector3().setFromMatrixPosition(dragStart.gizmoMatrix);
      const delta=handle.position.clone().sub(origin);
      const bounded=boundedPlacementDelta(dragStart,delta);
      handle.position.copy(origin).add(bounded);handle.updateMatrix();
    }
    let delta;
    if(gizmo.object===wholeObjectHandle){
      wholeObjectHandle.updateMatrix();delta=wholeObjectHandle.matrix.clone().multiply(dragStart.handleMatrix.clone().invert());
      if(state.tool==='rotate'){
        for(const [id,matrix] of dragStart.matrices){const object=partObjects.get(id);delta.clone().multiply(matrix).decompose(object.position,object.quaternion,object.scale);object.updateMatrix();}
        snapRotation(selected);delta=selected.matrix.clone().multiply(dragStart.matrices.get(dragStart.id).clone().invert());
        delta.clone().multiply(dragStart.handleMatrix).decompose(wholeObjectHandle.position,wholeObjectHandle.quaternion,wholeObjectHandle.scale);
      }
    }else if(gizmo.object===rotationHandle){
      const axis=({X:new THREE.Vector3(1,0,0),Y:new THREE.Vector3(0,1,0),Z:new THREE.Vector3(0,0,1)})[gizmo.axis];
      if(state.snap&&axis){const original=new THREE.Quaternion().setFromRotationMatrix(dragStart.handleMatrix),relative=original.clone().invert().multiply(rotationHandle.quaternion);
        const step=THREE.MathUtils.degToRad(state.snapSettings.rotationDeg),angle=2*Math.atan2(new THREE.Vector3(relative.x,relative.y,relative.z).dot(axis),relative.w);
        rotationHandle.quaternion.copy(original).multiply(new THREE.Quaternion().setFromAxisAngle(axis,Math.round(angle/step)*step));}
      rotationHandle.updateMatrix();delta=rotationHandle.matrix.clone().multiply(dragStart.handleMatrix.clone().invert());
    }else{if(state.tool==='rotate')snapRotation(selected);delta=selected.matrix.clone().multiply(dragStart.matrices.get(dragStart.id).clone().invert());}
    for(const [id,matrix] of dragStart.matrices){const object=partObjects.get(id);delta.clone().multiply(matrix).decompose(object.position,object.quaternion,object.scale);object.updateMatrix();}
    previewMovement();
  });
  gizmo.addEventListener('mouseUp',async()=>{
    if(dragStart)await finishPlacement();
  });
  const resize=()=>{const rect=viewport.getBoundingClientRect();renderer.setSize(rect.width,rect.height);camera.aspect=rect.width/rect.height;camera.updateProjectionMatrix();};new ResizeObserver(resize).observe(viewport);
}catch(error){$('#canvas-fallback').classList.remove('hidden');console.error(error);}

function setPose(object,pose={}){object.position.fromArray(pose.position_mm||[0,0,0]);const r=pose.rotation_deg||[0,0,0];object.rotation.set(...r.map(THREE.MathUtils.degToRad),'ZYX');object.updateMatrix();}
function getPose(object){const e=new THREE.Euler().setFromQuaternion(object.quaternion,'ZYX');return {position_mm:object.position.toArray().map(v=>+v.toFixed(5)),rotation_deg:[e.x,e.y,e.z].map(v=>+THREE.MathUtils.radToDeg(v).toFixed(5))};}
function status(text){$('#status-text').textContent=text;}
function toast(text,error=false){clearTimeout(toastTimer);const el=$('#toast');el.textContent=text;el.className=error?'error':'';toastTimer=setTimeout(()=>el.classList.add('hidden'),error?9000:4500);}
const EDITOR_API_VERSION=25;
let serverApiVersion=null;
const SERVER_UPDATE_MESSAGE=`The PipeSim server at ${window.location.host} is older than this editor. Save your work, stop the process using that address, start the server again, then reload this tab.`;
const EDITOR_UPDATE_MESSAGE='This PipeSim tab is older than the running server. Save any unsaved work, then reload this tab.';
function compatibilityMessage(version){
  if(version===EDITOR_API_VERSION)return null;
  const detail=` (editor API ${EDITOR_API_VERSION}; server API ${Number.isInteger(version)?version:'unknown'})`;
  return (Number.isInteger(version)&&version>EDITOR_API_VERSION?EDITOR_UPDATE_MESSAGE:SERVER_UPDATE_MESSAGE)+detail;
}
async function api(route,extra={}){
  const body=JSON.stringify({document:state.doc,path:state.path,...extra});
  for(let attempt=0;attempt<2;attempt++){
    const response=await fetch('/api/'+route,{method:'POST',headers:{'Content-Type':'application/json','X-PipeSim-Token':token},body});
    const data=await response.json();
    if(response.ok)return data;
    if(response.status===403&&data.error==='Invalid editor session token'&&attempt===0){
      // Restarting the local server changes its token. Refresh only the session,
      // keeping the authored document, undo history and camera in this page.
      const sessionResponse=await fetch('/api/bootstrap'),session=await sessionResponse.json();
      if(sessionResponse.ok&&session.token){token=session.token;serverApiVersion=session.api_version;continue;}
    }
    if(response.status===404&&data.error==='Unknown operation'){
      try{const probe=await fetch('/api/bootstrap');if(probe.ok)serverApiVersion=(await probe.json()).api_version;}catch{}
      throw new Error(compatibilityMessage(serverApiVersion)||`The running PipeSim server does not recognize ${route}. Restart the server and reload this tab.`);
    }
    throw new Error(data.error||'Operation failed');
  }
}
function checkpoint(){if(!state.doc)return;state.undo.push(clone(state.doc));if(state.undo.length>preferences.undoLimit)state.undo.shift();state.redo=[];}
function autoMirrorModes(scene=state.scene,{placed=true}={}){
  const sceneParts=new Map((scene?.parts||[]).map(part=>[part.id,part]));
  const documentParts=new Map((state.doc?.parts||[]).map(part=>[part.id,part]));
  let added=0;
  for(const group of state.doc?.draft_subassemblies||[])for(const plane of group.mirrors||[]){
    const axis={x:0,y:1,z:2}[plane.axis];
    for(const run of group.runs){
      if(Object.hasOwn(plane.run_modes||{},run.id))continue;
      let start=new THREE.Vector3(...run.start_mm),end=new THREE.Vector3(...run.end_mm);
      const attached=!!run.attachments?.length;
      if(attached){
        const current=run.attachments.every(a=>{
          const spec=documentParts.get(a.connector),part=sceneParts.get(a.connector);
          if(!spec||!part)return false;
          const pose=spec.pose||{},shown=part.pose||{};
          return ['position_mm','rotation_deg'].every(field=>(pose[field]||[0,0,0]).every((value,i)=>Math.abs(value-(shown[field]||[0,0,0])[i])<1e-6));
        });
        const preview=sceneParts.get(run.id);if(!current||!preview)continue;
        const object=new THREE.Object3D();setPose(object,preview.pose);
        start=new THREE.Vector3(0,0,-preview.length_mm/2).applyMatrix4(object.matrix);
        end=new THREE.Vector3(0,0,preview.length_mm/2).applyMatrix4(object.matrix);
      }
      const length=start.distanceTo(end),tolerance=(attached||!placed)?0.05:2;
      if(length<=1e-6)continue;
      const first=start.getComponent(axis)-plane.offset_mm,last=end.getComponent(axis)-plane.offset_mm;
      const mode=Math.abs((first+last)/2)<=tolerance&&Math.abs(Math.abs(last-first)-length)<=tolerance?'centered':
        Math.abs(first)<=tolerance&&Math.abs(last)<=tolerance?'in_plane':null;
      if(mode){plane.run_modes||={};plane.run_modes[run.id]=mode;added++;}
    }
  }
  return added;
}
function adoptLoadedMirrorModes(scene){if(autoMirrorModes(scene,{placed:false})){state.dirty=true;state.revision++;}}
function constrainDraftMirrors(){
  for(const group of state.doc?.draft_subassemblies||[])for(const plane of group.mirrors||[])for(const [id,mode] of Object.entries(plane.run_modes||{})){
    if(mode==='free')continue;
    const run=group.runs.find(item=>item.id===id);if(!run)continue;
    const axis={x:0,y:1,z:2}[plane.axis],start=new THREE.Vector3(...run.start_mm),end=new THREE.Vector3(...run.end_mm);
    const bound=(run.attachments||[]).filter(attachment=>attachment.end);
    if(bound.length){
      const socketEnd=attachment=>{
        const spec=state.doc.parts.find(part=>part.id===attachment.connector);
        const socket=state.scene?.parts.find(part=>part.id===attachment.connector)?.ports?.[attachment.port];
        if(!spec||!socket)return null;
        const pose=spec.pose||{},rotation=new THREE.Quaternion().setFromEuler(new THREE.Euler(...(pose.rotation_deg||[0,0,0]).map(THREE.MathUtils.degToRad),'ZYX'));
        const direction=new THREE.Vector3(...(socket.axis||[0,0,1])).applyQuaternion(rotation).normalize();
        const depth=attachment.insertion_mm??Math.min(30,socket.engagement_mm*.8);
        const point=new THREE.Vector3(...(socket.position_mm||[0,0,0])).applyQuaternion(rotation)
          .add(new THREE.Vector3(...(pose.position_mm||[0,0,0]))).addScaledVector(direction,-depth);
        return {attachment,socket,direction,depth,point};
      };
      if(mode==='centered'&&bound.length===1&&run.locked_length_mm==null){
        const socket=socketEnd(bound[0]);
        if(socket){
          const {attachment,direction,point}=socket;
          const component=direction.getComponent(axis);
          if(Math.abs(component)>1e-6){
            // An attached end defines the effective direction. The saved draft
            // endpoints can point the other way after a socket snap or copy.
            const length=2*(plane.offset_mm-point.getComponent(axis))/component;
            if(length>1e-6&&length*(1-Math.abs(component))<=.05){
              const other=point.clone().addScaledVector(direction,length);
              run.start_mm=(attachment.end==='start'?point:other).toArray();
              run.end_mm=(attachment.end==='start'?other:point).toArray();
            }
          }
        }
      }else if(mode==='centered'&&bound.length===2){
        const sockets=bound.map(socketEnd);
        if(sockets.every(Boolean)){
          const points=Object.fromEntries(sockets.map(socket=>[socket.attachment.end,socket.point]));
          const midpoint=(points.start.getComponent(axis)+points.end.getComponent(axis))/2;
          const candidates=sockets.map(socket=>{
            const component=socket.direction.getComponent(axis);
            if(Math.abs(component)<1e-6)return null;
            const depth=socket.depth+2*(midpoint-plane.offset_mm)/component;
            return depth>=(socket.socket.min_engagement_mm||0)-1e-6&&depth<=socket.socket.engagement_mm+1e-6?
              {socket,depth,travel:Math.abs(depth-socket.depth)}:null;
          }).filter(Boolean).sort((a,b)=>a.travel-b.travel);
          if(candidates.length){
            const {socket,depth}=candidates[0];
            socket.attachment.insertion_mm=depth;
            points[socket.attachment.end].addScaledVector(socket.direction,socket.depth-depth);
            run.start_mm=points.start.toArray();run.end_mm=points.end.toArray();
          }
        }
      }
      continue;
    }
    if(mode==='in_plane'){start.setComponent(axis,plane.offset_mm);end.setComponent(axis,plane.offset_mm);}
    else{const center=start.clone().add(end).multiplyScalar(.5),length=start.distanceTo(end),sign=end.getComponent(axis)>=start.getComponent(axis)?1:-1;
      center.setComponent(axis,plane.offset_mm);start.copy(center).addScaledVector(new THREE.Vector3().setComponent(axis,1),-sign*length/2);
      end.copy(center).addScaledVector(new THREE.Vector3().setComponent(axis,1),sign*length/2);}
    if(start.distanceTo(end)>1e-6){run.start_mm=start.toArray();run.end_mm=end.toArray();}
  }
}
function mirrorConstrainedDelta(ids,delta){
  const result=delta.clone(),members=new Set(ids);
  for(const group of state.doc?.draft_subassemblies||[])for(const plane of group.mirrors||[])
    if(Object.entries(plane.run_modes||{}).some(([id,mode])=>mode!=='free'&&members.has(id)))result.setComponent({x:0,y:1,z:2}[plane.axis],0);
  return result;
}
let pendingAutomaticDraftRepair=null;
function changed({inferMirrors=true}={}){
  if(pendingAutomaticDraftRepair){
    const job=pendingAutomaticDraftRepair;pendingAutomaticDraftRepair=null;
    api('draft-repair-cancel',{job_id:job.id}).catch(()=>{});
  }
  if(inferMirrors)autoMirrorModes();constrainDraftMirrors();state.simulationError=null;state.doc.results={};delete state.doc.build_plan;state.dirty=true;state.revision++;state.recording=null;state.playing=false;state.checks=null;state.analysis=null;state.plan=null;state.frame=0;updateHeader();scheduleAutosave();
}
function refreshDraft(id,{structure=false}={}){
  const run=draftRun(state.doc,id),previous=partObjects.get(id);
  if(previous){objects.remove(previous);dispose(previous);partObjects.delete(id);state.scene.parts=state.scene.parts.filter(p=>p.id!==id);state.scene.groups=state.scene.groups.filter(g=>!g.includes(id));}
  if(run){const part=draftPreview(run,state.scene,state.library),group=new THREE.Group();group.name=id;group.userData.part=id;
    for(const shape of part.geometry)group.add(meshShape(shape,part.color,part.kind));setPose(group,part.pose);objects.add(group);partObjects.set(id,group);
    state.scene.parts.push(part);state.scene.groups.push([id]);}
  state.scene.draft_attachments=draftRuns(state.doc).flatMap(r=>(r.attachments||[]).map(a=>({...a,run:r.id})));
  if(structure){renderOutline();updateHeader();}
  if(structure||state.selected===id)highlightSelection();
  if(structure||state.selected===id){renderInspector();attachGizmo();updateResizeHandles();}
}
function mirrorSourceIds(group,exactIds){
  const ids=new Set([...group.runs.map(run=>run.id),...(group.mirror_parts||[])]);
  for(const run of group.runs)for(const attachment of run.attachments||[])ids.add(attachment.connector);
  for(const rigid of state.scene?.groups||[])if(rigid.some(id=>ids.has(id)))for(const id of rigid)if(exactIds.has(id))ids.add(id);
  return ids;
}
function mirrorMatrix(axis,offset){
  const matrix=new THREE.Matrix4().identity(),index={x:0,y:1,z:2}[axis];
  matrix.elements[index*4+index]=-1;matrix.elements[12+index]=2*offset;return matrix;
}
function removeMirrorCopy(record){
  mirrorGhost.remove(record.clone);
  record.clone.traverse(node=>{if(node.isMesh)for(const material of [node.material].flat())material.dispose();});
}
function syncMirrorPreviews(){
  const groups=(state.doc?.draft_subassemblies||[]).filter(group=>group.mirrors?.length);
  if(!groups.length&&!mirrorCopies.size&&!mirrorPlanes.size)return;
  const wantedCopies=new Set(),wantedPlanes=new Set();
  const sceneParts=new Map((state.scene?.parts||[]).map(part=>[part.id,part]));
  const exactIds=new Set((state.doc?.parts||[]).map(part=>part.id));
  const pointKey=point=>point.toArray().map(value=>Math.round(value*20)).join(',');
  for(const group of groups){
    for(const plane of group.mirrors){
      const key=group.id+'/'+plane.id;wantedPlanes.add(key);
      if(!mirrorPlanes.has(key)){
        const normal=new THREE.Vector3(...({x:[1,0,0],y:[0,1,0],z:[0,0,1]})[plane.axis]);
        const mesh=new THREE.Mesh(new THREE.PlaneGeometry(12000,12000),
          new THREE.MeshBasicMaterial({color:'#48c6a0',transparent:true,opacity:.09,depthWrite:false,side:THREE.DoubleSide}));
        mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,0,1),normal);
        mesh.position.setComponent({x:0,y:1,z:2}[plane.axis],plane.offset_mm);
        mesh.renderOrder=1;mirrorGhost.add(mesh);mirrorPlanes.set(key,mesh);
      }
    }
    const planes=group.mirrors,groupIds=mirrorSourceIds(group,exactIds);
    const ids=planes.some(plane=>plane.scope==='scene')?new Set(sceneParts.keys()):groupIds;
    const sources=new Map(),seen=new Map();
    for(const id of ids){
      const source=partObjects.get(id),part=sceneParts.get(id);if(!source||!part||part.kind==='human')continue;
      source.updateMatrix();
      let signature=0;source.traverse(()=>signature++);
      const a=part.draft?new THREE.Vector3(0,0,-part.length_mm/2).applyMatrix4(source.matrix):null;
      const b=part.draft?new THREE.Vector3(0,0,part.length_mm/2).applyMatrix4(source.matrix):null;
      sources.set(id,{source,part,signature,a,b});
      const original=a?[pointKey(a),pointKey(b)].sort().join('|'):pointKey(new THREE.Vector3().setFromMatrixPosition(source.matrix));
      seen.set(id,new Set([original]));
    }
    for(let mask=1;mask<1<<planes.length;mask++){
      const sceneMask=planes.some((plane,bit)=>(mask&(1<<bit))&&plane.scope==='scene');
      const transform=new THREE.Matrix4().identity();
      for(let bit=0;bit<planes.length;bit++)if(mask&(1<<bit))transform.premultiply(mirrorMatrix(planes[bit].axis,planes[bit].offset_mm));
      for(const [id,{source,part,signature,a,b}] of sources){
        if(!sceneMask&&!groupIds.has(id))continue;
        const reflected=part.draft?[pointKey(a.clone().applyMatrix4(transform)),pointKey(b.clone().applyMatrix4(transform))].sort().join('|'):
          pointKey(new THREE.Vector3().setFromMatrixPosition(source.matrix).applyMatrix4(transform));
        if(seen.get(id).has(reflected))continue;
        seen.get(id).add(reflected);
        const key=group.id+'/'+mask+'/'+id;wantedCopies.add(key);
        let record=mirrorCopies.get(key);
        if(!record||record.source!==source||record.signature!==signature){
          if(record)removeMirrorCopy(record);
          const clone=source.clone(true);clone.matrixAutoUpdate=false;
          clone.traverse(node=>{node.userData={};if(node.isMesh){
            const fade=material=>{const copy=material.clone();copy.transparent=true;copy.opacity=.4;copy.depthWrite=false;return copy;};
            node.material=Array.isArray(node.material)?node.material.map(fade):fade(node.material);
          }});mirrorGhost.add(clone);
          record={clone,source,signature};mirrorCopies.set(key,record);
        }
        record.clone.matrix.copy(transform).multiply(source.matrix);record.clone.matrixWorldNeedsUpdate=true;
      }
    }
  }
  for(const [key,record] of mirrorCopies)if(!wantedCopies.has(key)){removeMirrorCopy(record);mirrorCopies.delete(key);}
  for(const [key,mesh] of mirrorPlanes)if(!wantedPlanes.has(key)){mirrorGhost.remove(mesh);mesh.geometry.dispose();mesh.material.dispose();mirrorPlanes.delete(key);}
}
async function finalizeDrafts(subassembly=null,run_id=null){
  if(state.placementPending)return;
  if(!draftRuns(state.doc).length){toast('There are no draft pipes to finalize.');return;}
  const job_id=Date.now().toString(36)+Math.random().toString(36).slice(2),revision=state.revision;
  let cancelRequested=false;
  state.placementPending=true;gizmo?.detach();status('Checking draft connections and cut lengths…');
  modal('Finalizing draft','<p>Solving connector poses and cut lengths, then checking the physical assembly once.</p><p id="draft-finalize-status" role="status">Working…</p>',[
    {label:'Cancel finalization',action:async()=>{cancelRequested=true;status('Cancelling draft finalization…');$('#draft-finalize-status').textContent='Cancelling…';await api('draft-finalize-cancel',{job_id});}}
  ]);
  try{const result=await api(run_id?'draft-finalize-selected':'draft-finalize',
    {job_id,...(run_id?{run:run_id}:subassembly?{subassembly}:{})});
    if(cancelRequested||revision!==state.revision)return;
    closeModal();
    if(result.status==='conflict'){
      const rows=result.conflicts.map(c=>`<div class="joint-card"><strong>${esc(c.run||c.parts?.join(' / ')||c.code)}</strong><p>${esc(c.message)}${c.residual_mm!=null?' · '+esc(c.residual_mm)+' mm':''}${c.residual_deg!=null?' · '+esc(c.residual_deg)+'°':''}</p></div>`).join('');
      modal('Draft needs adjustment',`<p>The draft is still editable. Resolve these connections or move the fittings, then finalize again.</p>${rows}`,[{label:'Close',action:closeModal}]);status('Draft has unresolved connections');return;
    }
    await acceptPlacement(result,revision);
    const recovered=result.inferred_through_connections||0;
    toast((run_id?'Selected draft structure finalized. Other draft structures remain editable.':'Draft finalized with exact cut lengths.')+
      (recovered?` Connected ${recovered} aligned through ${recovered===1?'socket':'sockets'} found along the draft pipes.`:''));
    status(run_id?'Selected draft structure finalized':'Draft finalized');
  }catch(e){if(!cancelRequested){toast(e.message,true);status('Finalization failed');}else status('Draft finalization cancelled');}
  finally{if($('#modal').open&&$('#modal-title').textContent==='Finalizing draft')closeModal();state.placementPending=false;attachGizmo();}
}
async function repairDrafts(subassembly=null,run_id=null){
  if(state.placementPending)return;
  const job_id=Date.now().toString(36)+Math.random().toString(36).slice(2),revision=state.revision;
  let cancelRequested=false;state.placementPending=true;gizmo?.detach();status('Repairing draft alignment…');
  modal('Repairing draft','<p>Aligning movable connectors while keeping every pipe in draft mode.</p><p id="draft-repair-status" role="status">Working…</p>',[
    {label:'Cancel repair',action:async()=>{cancelRequested=true;status('Cancelling draft repair…');$('#draft-repair-status').textContent='Cancelling…';await api('draft-repair-cancel',{job_id});}}
  ]);
  try{
    const result=await api(run_id?'draft-repair-selected':'draft-repair',
      {job_id,...(run_id?{run:run_id}:subassembly?{subassembly}:{})});
    if(cancelRequested||revision!==state.revision)return;
    closeModal();
    if(result.status==='conflict'){
      const rows=result.conflicts.map(c=>`<div class="joint-card"><strong>${esc(c.run||c.code)}</strong><p>${esc(c.message)}${c.residual_mm!=null?' · '+esc(c.residual_mm)+' mm':''}</p></div>`).join('');
      modal('Draft still needs adjustment',`<p>The draft is unchanged. Move an anchored fitting or detach a conflicting connection, then try again.</p>${rows}`,[{label:'Close',action:closeModal}]);
      status('Draft repair could not close every fit');return;
    }
    if(result.status==='aligned'){status('Draft connections are already aligned');toast('Draft connections are already aligned.');return;}
    await acceptPlacement(result,revision);
    const recovered=result.inferred_through_connections||0;
    toast(`Draft alignment repaired${recovered?`; connected ${recovered} through ${recovered===1?'socket':'sockets'}`:''}. The pipes remain editable drafts.`);
    status('Draft alignment repaired');
  }catch(e){if(!cancelRequested){toast(e.message,true);status('Draft repair failed');}else status('Draft repair cancelled');}
  finally{if($('#modal').open&&$('#modal-title').textContent==='Repairing draft')closeModal();state.placementPending=false;attachGizmo();}
}
async function mutate(fn){checkpoint();try{fn();changed();await resolve();}catch(e){state.doc=state.undo.pop()||state.doc;toast(e.message,true);await resolve();throw e;}}
async function resolve(){const revision=state.revision;try{const data=await api('resolve');if(revision!==state.revision)return;if(data.document){state.doc=data.document;state.dirty=true;state.revision++;scheduleAutosave();}if(data.mirror_pose_error&&data.mirror_pose_error!==state.mirrorPoseError)toast(data.mirror_pose_error,true);state.mirrorPoseError=data.mirror_pose_error||null;buildScene(data);renderInspector();renderOutline();updateHeader();}catch(e){toast(e.message,true);status('Resolve error: '+e.message);throw e;}}
function updateHeader(){
  if(!state.doc)return;
  const drafts=draftRuns(state.doc).length,exact=state.scene?.parts.filter(p=>!p.draft)||[];
  const exactIds=new Set(exact.map(p=>p.id));
  const bodies=(state.scene?.groups||[]).filter(group=>group.some(id=>exactIds.has(id))).length;
  const mass=exact.reduce((sum,p)=>sum+p.mass_kg,0);
  $('#rename').textContent=state.doc.name||'Untitled creation';
  $('#file-path').textContent=state.path;
  $('#dirty').classList.toggle('changed',state.dirty);
  $('#tree-count').textContent=state.scene?.parts.length||0;
  $('#part-stat').textContent=exact.length+' parts'+(drafts?' · '+drafts+' draft':'');
  $('#body-stat').textContent=bodies+(bodies===1?' rigid body':' rigid bodies');
  $('#mass-stat').textContent=(preferences.massUnit==='kg'?mass.toFixed(1):displayValue(mass,'mass',preferences.massUnit))+' '+preferences.massUnit;
  $('.unit-badge').textContent=`${preferences.lengthUnit} · ${preferences.massUnit} · ${preferences.forceUnit}`;
  $('.view-caption').textContent=preferences.lengthUnit==='mm'?'Z up · millimetres':`Z up · ${preferences.lengthUnit}`;
  $('#undo').disabled=!state.undo.length;$('#redo').disabled=!state.redo.length;$('#finalize-draft').disabled=!drafts;
}

function solid(shape){
  const type=shape.type,radius=shape.radius_mm??shape.diameter_mm/2,length=shape.length_mm;
  if(type==='box')return new THREE.BoxGeometry(...shape.size_mm);
  if(type==='sphere')return new THREE.SphereGeometry(radius,24,16);
  if(type==='cylinder')return new THREE.CylinderGeometry(radius,radius,length,32).rotateX(Math.PI/2);
  if(type==='capsule')return new THREE.CapsuleGeometry(radius,length,8,20).rotateX(Math.PI/2);
  if(type==='tube'){
    const profile=new THREE.Shape();profile.absarc(0,0,radius,0,Math.PI*2,false);const hole=new THREE.Path();hole.absarc(0,0,radius-shape.wall_mm,0,Math.PI*2,true);profile.holes.push(hole);
    return new THREE.ExtrudeGeometry(profile,{depth:length,bevelEnabled:false,steps:1,curveSegments:24}).translate(0,0,-length/2);
  }
  return null;
}
function material(color,kind){return new THREE.MeshStandardMaterial({color,metalness:kind==='member'||kind==='connector'?.55:.04,roughness:kind==='member'?.36:.55});}
function meshShape(shape,color,kind){
  const group=new THREE.Group();
  if(shape.type==='extrusion'){
    const [w,d,l]=shape.size_mm;const mat=material(color,kind);group.add(new THREE.Mesh(new THREE.BoxGeometry(w*.46,d*.46,l),mat));
    for(const x of [-1,1])for(const y of [-1,1]){const c=new THREE.Mesh(new THREE.BoxGeometry(w*.27,d*.27,l),mat);c.position.set(x*w*.365,y*d*.365,0);group.add(c);}
    for(const [x,y,sx,sy] of [[0,1,w*.16,d*.35],[0,-1,w*.16,d*.35],[1,0,w*.35,d*.16],[-1,0,w*.35,d*.16]]){const m=new THREE.Mesh(new THREE.BoxGeometry(sx,sy,l),mat);m.position.set(x*w*.24,y*d*.24,0);group.add(m);}
  }else if(shape.type==='mesh'){
    if(shape.url){const generation=sceneGeneration;const extension=shape.file.split('.').pop().toLowerCase();
      const loader=extension==='stl'?new STLLoader():extension==='obj'?new OBJLoader():new GLTFLoader();
      loader.load(shape.url,loaded=>{if(generation!==sceneGeneration)return;const mesh=extension==='stl'?new THREE.Mesh(loaded,material(color,kind)):loaded.scene||loaded;mesh.scale.setScalar(shape.scale||1);mesh.traverse(o=>{if(o.isMesh){o.castShadow=true;o.receiveShadow=true;if(!o.material)o.material=material(color,kind);}});group.add(mesh);},undefined,e=>toast('Mesh could not be loaded: '+shape.file,true));
    }
  }else{const geometry=solid(shape);if(geometry)group.add(new THREE.Mesh(geometry,material(shape.color||color,kind)));}
  group.position.fromArray(shape.position_mm||[0,0,0]);const angles=shape.rotation_deg||[0,0,0];group.rotation.set(...angles.map(THREE.MathUtils.degToRad),'ZYX');
  if(shape.axis){const align=new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0,0,1),new THREE.Vector3(...shape.axis).normalize());group.quaternion.multiply(align);}
  group.traverse(o=>{if(o.isMesh){o.castShadow=true;o.receiveShadow=true;}});return group;
}
function dispose(group){group.traverse(o=>{o.geometry?.dispose();if(o.material){for(const m of Array.isArray(o.material)?o.material:[o.material])m.dispose();}});group.clear();}
function resizeTarget(){
  const object=selectedObject();
  if(object&&object.template!=='chain')return null;
  if(object?.template==='chain'){
    const chain=state.scene?.chains?.find(c=>c.id===object.id);
    if(!chain)return null;
    const point=(part,port)=>{const data=state.scene.parts.find(p=>p.id===part),mesh=partObjects.get(part);
      return data&&mesh&&data.ports[port]?new THREE.Vector3(...(data.ports[port].position_mm||[0,0,0])).applyMatrix4(mesh.matrix):null;};
    const first=point(chain.start_part,chain.start_port),last=point(chain.end_part,chain.end_port);
    return first&&last?{id:object.id,first,last,length:chain.requested_length_mm,pitch:chain.pitch_mm,chain:true}:null;
  }
  const part=state.scene?.parts.find(p=>p.id===state.selected&&p.kind==='member'),mesh=partObjects.get(state.selected);
  if(!part||!mesh||!Number.isFinite(part.length_mm)||part.length_mm<=0)return null;
  const centered=state.doc?.draft_subassemblies?.some(group=>group.runs.some(run=>run.id===part.id)&&
    group.mirrors?.some(plane=>plane.run_modes?.[part.id]==='centered'))||false;
  return {id:part.id,first:new THREE.Vector3(0,0,-part.length_mm/2).applyMatrix4(mesh.matrix),
    last:new THREE.Vector3(0,0,part.length_mm/2).applyMatrix4(mesh.matrix),length:part.length_mm,chain:false,centered};
}
function updateResizeHandles(){
  dispose(resizeHandles);
  $('#resize-label-first').classList.add('hidden');$('#resize-label-second').classList.add('hidden');
  if(state.tool==='connect'||state.mode!=='design'||state.placementPending)return;
  const target=resizeTarget();if(!target)return;
  for(const [side,point] of [['start',target.first],['end',target.last]]){
    const radius=Math.min(14,Math.max(5,target.first.distanceTo(target.last)*.24));
    const handle=new THREE.Mesh(new THREE.SphereGeometry(radius,16,12),new THREE.MeshBasicMaterial({color:'#f4bb65',depthTest:false}));
    handle.position.copy(point);handle.renderOrder=130;handle.userData.resizeSide=side;resizeHandles.add(handle);
  }
  $('#resize-label-first').textContent=`${preferences.resizeHandle1GrowKey.toUpperCase()}/${preferences.resizeHandle1ShrinkKey.toUpperCase()}`;
  $('#resize-label-second').textContent=`${preferences.resizeHandle2GrowKey.toUpperCase()}/${preferences.resizeHandle2ShrinkKey.toUpperCase()}`;
  $('#resize-label-first').classList.remove('hidden');$('#resize-label-second').classList.remove('hidden');
}
function positionResizeLabels(){
  if(resizeHandles.children.length!==2)return;
  const rect=viewport.getBoundingClientRect();
  for(const [index,id] of ['#resize-label-first','#resize-label-second'].entries()){
    const projected=resizeHandles.children[index].position.clone().project(camera),label=$(id);
    label.style.left=((projected.x+1)*rect.width/2+18)+'px';
    label.style.top=((1-projected.y)*rect.height/2+32)+'px';
  }
}
async function requestResize(target,side,length,shift=false){
  if(state.placementPending||state.busy||!target||!Number.isFinite(length))return;
  const revision=state.revision;state.placementPending=true;updateResizeHandles();
  try{const result=await api('resize-drag',{member:target.id,endpoint:side,length_mm:Math.max(1,length),
      behavior:shift?preferences.resizeShiftMode:preferences.resizeConnectorMode,
      capture_mm:preferences.resizeCaptureMm,capture_deg:preferences.resizeCaptureDeg,
      auto_connect:preferences.resizeAutoConnect,locked:preferences.connectionLocked});
    await acceptPlacement(result,revision);status(`Resized ${target.id} to ${result.length_mm.toFixed(1)} mm`);
  }catch(error){restorePlacement();toast(error.message,true);}
  finally{state.placementPending=false;updateResizeHandles();}
}
function buildScene(data){
  state.scene=data;sceneGeneration++;gizmo?.detach();dispose(objects);dispose(ports);dispose(overlays);partObjects.clear();beamObjects.clear();
  if(state.selected&&!data.parts.some(p=>p.id===state.selected)){
    const chain=data.chains?.find(c=>state.selected.startsWith(c.id+'/'));
    if(chain)state.selected=chain.end_part;
  }
  for(const part of data.parts){const group=new THREE.Group();group.name=part.id;group.userData.part=part.id;for(const shape of part.geometry)group.add(meshShape(shape,part.color,part.kind));setPose(group,part.pose);objects.add(group);partObjects.set(part.id,group);}
  for(const drive of state.doc.drives||[]){if(!drive.route_mm?.length)continue;const points=drive.route_mm.map(p=>new THREE.Vector3(...p));const curve=new THREE.CatmullRomCurve3(points,false,'catmullrom',0);const belt=new THREE.Mesh(new THREE.TubeGeometry(curve,80,2,6,false),new THREE.MeshStandardMaterial({color:'#374b52',roughness:.8}));overlays.add(belt);}
  for(const anchor of data.anchors){const part=partObjects.get(anchor.part);if(!part)continue;const loc=part.position.clone();const marker=new THREE.Mesh(new THREE.RingGeometry(70,73,48),new THREE.MeshBasicMaterial({color:'#74a28c',transparent:true,opacity:.55,side:THREE.DoubleSide}));marker.position.copy(loc);marker.position.z+=1;marker.quaternion.copy(part.quaternion);overlays.add(marker);}
  updatePorts();updateConnectionHint();highlightSelection();updateHeader();updateResizeHandles();if(state.tool==='translate'||state.tool==='rotate')attachGizmo();
}
function updatePorts(){dispose(ports);if(!state.scene)return;const show=state.ports||state.tool==='connect';for(const p of state.scene.parts){if(!show&&p.id!==state.selected&&p.id!==hoveredDraftConnection?.connector)continue;for(const [name,port] of Object.entries(p.ports)){const focused=p.id===hoveredDraftConnection?.connector&&name===hoveredDraftConnection?.port;if(!show&&p.id===hoveredDraftConnection?.connector&&!focused)continue;const chain=state.scene.chains?.find(c=>p.id.startsWith(c.id+'/'));if(chain&&!((p.id===chain.start_part&&name===chain.start_port)||(p.id===chain.end_part&&name===chain.end_port)||(chain.layout_mode==='posable'&&p.id===state.selected)))continue;if(port.type!=='socket'&&!show)continue;const radius=focused?12:port.type==='socket'?7:4;const sphere=new THREE.Mesh(new THREE.SphereGeometry(radius,12,8),new THREE.MeshBasicMaterial({color:focused?'#f3ba62':port.type==='socket'?'#3c9c85':'#779eba',depthTest:false,transparent:true,opacity:focused?1:.8}));const local=new THREE.Vector3(...(port.position_mm||[0,0,0]));const obj=partObjects.get(p.id);sphere.position.copy(local.applyMatrix4(obj.matrix));sphere.renderOrder=100;sphere.userData={port:name,part:p.id,portType:port.type};ports.add(sphere);const axis=new THREE.Vector3(...(port.axis||[0,0,1])).transformDirection(obj.matrix);const arrow=new THREE.ArrowHelper(axis,sphere.position,38,focused?'#f3ba62':'#64a294',8,4);arrow.visible=show||focused;ports.add(arrow);}}}
function highlightSelection(){
  const object=wholeObject();
  for(const [id,group] of partObjects){const active=object?id.startsWith(object.id+'/'):id===state.selected,hovered=id===hoveredDraftConnection?.connector;group.traverse(o=>{if(o.isMesh&&o.material?.emissive){o.material.emissive.set(hovered?'#f3ba62':active?'#438f79':'#000000');o.material.emissiveIntensity=hovered?.45:active?.12:0;}});}
  const part=state.scene?.parts.find(p=>p.id===state.selected);$('#selection-summary').textContent=part?part.id+' · '+part.kind:'Nothing selected';$('#selection-label').textContent=part?.id||'';$('#selection-label').classList.toggle('hidden',!part);
}
function select(id){
  if(state.tool==='connect'){
    const part=state.scene?.parts.find(p=>p.id===id);
    if(part?.kind==='member')state.connectionSource=id;
    else{
      // A target click must not replace the tube being connected. This also
      // makes picking connectors in the design tree follow the same workflow.
      if(part&&Object.values(part.ports).some(p=>p.type==='socket'))connectDialog(id);
      return;
    }
  }
  state.selected=id;updateConnectionHint();highlightSelection();updatePorts();renderInspector();renderOutline();attachGizmo();updateResizeHandles();
}
function attachGizmo(){
  if(!gizmo)return;gizmo.detach();gizmo.showX=gizmo.showY=gizmo.showZ=true;gizmo.setSpace?.(state.tool==='rotate'&&localRotationHeld?'local':'world');
  if(state.placementPending||state.mode!=='design'||!['translate','rotate'].includes(state.tool)||!state.selected)return;
  const selected=partObjects.get(state.selected);if(!selected)return;
  const instance=wholeObject();
  if(instance){
    setPose(wholeObjectHandle,instance.pose);
    wholeObjectHandle.position.copy(partObjects.get(instance.id+'/pelvis')?.position||selected.position);wholeObjectHandle.updateMatrix();
    gizmo.attach(wholeObjectHandle);gizmo.setMode(state.tool);return;
  }
  const group=state.scene.groups.find(g=>g.includes(state.selected))||[state.selected];
  const candidates=state.scene.joints.filter(j=>!j.locked&&['socket','cylindrical','revolute','spherical'].includes(j.type)&&group.includes(j.a.part)!==group.includes(j.b.part));
  const joint=candidates.find(j=>group.includes(j.b.part))||candidates[0];
  if(state.tool==='rotate'&&joint){
    const a=state.scene.parts.find(p=>p.id===joint.a.part),object=partObjects.get(a.id),frame=joint.a.port?a.ports[joint.a.port]:joint.a.frame||{};
    const axis=new THREE.Vector3(...(frame.axis||[0,0,1])).applyQuaternion(object.quaternion).normalize();
    const local=frame.position_mm||(joint.a.end||joint.a.at_mm!=null?[0,0,(joint.a.at_mm??(joint.a.end==='end'?a.length_mm:0))-a.length_mm/2]:[0,0,0]);
    rotationHandle.position.fromArray(local).applyMatrix4(object.matrix).addScaledVector(axis,-(joint.insertion_mm||0));
    if(joint.type==='spherical')rotationHandle.quaternion.copy(object.quaternion).multiply(new THREE.Quaternion().setFromEuler(new THREE.Euler(...(frame.rotation_deg||[0,0,0]).map(THREE.MathUtils.degToRad),'ZYX')));
    else rotationHandle.quaternion.setFromUnitVectors(new THREE.Vector3(0,0,1),axis);
    rotationHandle.updateMatrix();gizmo.attach(rotationHandle);gizmo.setSpace?.('local');gizmo.showX=gizmo.showY=joint.type==='spherical';
  }else gizmo.attach(selected);
  gizmo.setMode(state.tool);
}
function updateConnectionHint(){
  const source=state.scene?.parts.find(p=>p.id===state.connectionSource&&p.kind==='member');
  if(!source)state.connectionSource=null;
  $('#placement-hint').classList.toggle('hidden',state.tool!=='connect');
  $('#placement-hint').textContent=source
    ?'Connecting '+source.id+': click a socket marker or a connector. Esc cancels.'
    :'Select a tube, dowel or extrusion, then click a socket marker or a connector. Esc cancels.';
}
function setTool(tool){
  state.tool=tool;
  if(tool!=='resize'&&resizeDrag)clearResizeDrag();
  if(renderer)renderer.domElement.style.cursor='';
  state.connectionSource=tool==='connect'&&state.scene?.parts.some(p=>p.id===state.selected&&p.kind==='member')?state.selected:null;
  if(tool==='connect'){if(state.mode!=='design')setMode('design');ports.visible=true;}
  $$('[data-tool]').forEach(b=>b.classList.toggle('active',b.dataset.tool===tool));
  updateConnectionHint();updatePorts();attachGizmo();updateResizeHandles();
}
function fitView(){if(!objects.children.length||!orbit)return;syncMirrorPreviews();const box=new THREE.Box3().setFromObject(objects);for(const {clone} of mirrorCopies.values())box.expandByObject(clone);const center=box.getCenter(new THREE.Vector3()),size=box.getSize(new THREE.Vector3());const halfAngle=Math.atan(Math.tan(THREE.MathUtils.degToRad(camera.fov)/2)*Math.min(1,camera.aspect));const distance=Math.max(size.length()/2,150)/Math.sin(halfAngle)*1.16;const direction=camera.position.clone().sub(orbit.target).normalize();orbit.target.copy(center);camera.position.copy(center).addScaledVector(direction,distance);orbit.update();}
function icon(part){const base='<svg viewBox="0 0 48 48" fill="none" stroke="#8eaaaF" stroke-width="6" stroke-linecap="round" stroke-linejoin="round">';let p='<path d="M13 34V14h22v20H13Z"/>';
  const name=part.name||'';if(part.kind==='member')p='<path d="M13 37L35 11"/><path d="M18 39L40 13" stroke="#bbccd2" stroke-width="2"/>';
  else if(/Flange|flange/.test(name))p='<ellipse cx="24" cy="33" rx="17" ry="7" fill="#d7e1e5" stroke-width="2"/><path d="M24 31V12" stroke-width="13"/><ellipse cx="24" cy="12" rx="6" ry="3" fill="#718891" stroke-width="1"/>';
  else if(/Corner Middle/.test(name))p='<path d="M24 6V42M24 24L8 33M24 24L40 33"/>';
  else if(/Split Tee/.test(name))p='<path d="M9 13H39M24 18V38"/><path d="M9 21H39" stroke-width="2"/><circle cx="11" cy="21" r="3" stroke-width="2"/><circle cx="37" cy="21" r="3" stroke-width="2"/>';
  else if(/Swivel Short Tee/.test(name))p='<ellipse cx="17" cy="14" rx="7" ry="10" stroke-width="4"/><path d="M20 22L30 29V39" stroke-width="10"/><path d="M14 23L24 30" stroke-width="3"/>';
  else if(/Crossover/.test(name))p='<path d="M15 7V41"/><path d="M7 25H41" stroke="#bbccd2" stroke-width="8"/><path d="M7 25H41"/>';
  else if(/Tee|tee/.test(name))p='<path d="M10 15H38M24 15V37"/>';
  else if(/Elbow|elbow/.test(name))p='<path d="M12 36V14H35"/>';
  else if(/Ring|Eye|Pin/.test(name))p='<circle cx="24" cy="24" r="13" stroke-width="7"/><path d="M34 33L39 38" stroke-width="4"/>';
  else if(/Swivel/.test(name))p='<path d="M12 12L21 21M28 28L36 36" stroke-width="11"/><circle cx="24" cy="24" r="5" stroke-width="3"/>';
  else if(part.kind==='panel'&&part.panel_layers)p='<path d="M5 24L27 13L43 24L21 37Z" fill="#647d89" stroke="#405866" stroke-width="2"/><path d="M5 19L27 8L43 19L21 32Z" fill="#8fa7b1" stroke="#405866" stroke-width="2"/>';
  else if(part.kind==='panel')p='<path d="M5 21L27 10L43 21L21 34Z" fill="#d4b991" stroke="#b59b78" stroke-width="2"/><path d="M5 21V27L21 40L43 27V21" stroke="#b59b78" stroke-width="2"/>';
  return base+p+'</svg>';
}
function libraryHotkeyFromEvent(event){
  const digit=/^Digit([0-9])$/.exec(event.code||'')?.[1]||(/^[0-9]$/.test(event.key)?event.key:null);
  if(digit&&!event.altKey&&!event.ctrlKey&&!event.metaKey)return (event.shiftKey?'Shift+':'')+digit;
  const letter=/^Key([A-Z])$/.exec(event.code||'')?.[1]||(/^[a-z]$/i.test(event.key)?event.key.toUpperCase():null);
  if(letter&&letter!=='D'&&event.altKey&&event.shiftKey&&!event.ctrlKey&&!event.metaKey)return 'Alt+Shift+'+letter;
  return null;
}
function showDraftConnection(connector,port){hoveredDraftConnection=connector?{connector,port}:null;highlightSelection();updatePorts();}
function addLibraryCatalog(catalog){
  if(!state.library[catalog]||state.busy||state.placementPending||dragStart)return;
  if(state.library[catalog].kind==='chain')chainDialog(catalog);
  else if(state.library[catalog].kind==='wheel')wheelDialog(catalog);
  else addPart(catalog);
}
function saveLibraryHotkeys(){try{window.localStorage.setItem(LIBRARY_HOTKEYS_KEY,JSON.stringify(libraryHotkeys));return true;}catch{return false;}}
let libraryMenuOwner=null;
function closeLibraryMenu(focus=false){
  $('#library-actions-menu')?.remove();const owner=libraryMenuOwner;libraryMenuOwner=null;
  owner?.setAttribute('aria-expanded','false');if(focus)owner?.focus();
}
function setLibraryHotkey(catalog){
  const old=libraryHotkeys[catalog];let chosen=old||null;
  modal('Set part hotkey',`<p>Press 0–9, Shift+0–9, or Alt+Shift+A–Z (except D). The shortcut adds this part from anywhere in the editor. It is saved in this browser.</p><div class="single-field"><label for="hotkey-capture">HOTKEY FOR ${esc(catalog)}</label><input id="hotkey-capture" readonly placeholder="Press a shortcut" value="${esc(old||'')}"></div><p id="hotkey-message" role="status"></p>`,[
    {label:'Cancel',action:closeModal},
    {label:'Save hotkey',primary:true,action:()=>{
      if(!chosen){$('#hotkey-message').textContent='Press a supported shortcut first.';return;}
      const previous=Object.keys(libraryHotkeys).find(id=>id!==catalog&&libraryHotkeys[id]===chosen);
      if(previous)delete libraryHotkeys[previous];libraryHotkeys[catalog]=chosen;
      const saved=saveLibraryHotkeys();closeModal();capture.blur();renderLibrary();
      toast(`${chosen} adds ${catalog}.${previous?' Reassigned from '+previous+'.':''}${saved?'':' Browser storage is unavailable; this assignment lasts only for this tab.'}`);
    }}
  ]);
  const capture=$('#hotkey-capture');capture.onkeydown=e=>{
    if(e.key==='Escape'||e.key==='Tab')return;
    e.preventDefault();e.stopPropagation();
    const hotkey=libraryHotkeyFromEvent(e);
    if(!hotkey){$('#hotkey-message').textContent='Use a number key, Shift+number, or Alt+Shift+letter (except D).';return;}
    chosen=hotkey;capture.value=hotkey;
    const previous=Object.keys(libraryHotkeys).find(id=>id!==catalog&&libraryHotkeys[id]===hotkey);
    $('#hotkey-message').textContent=previous?`Currently assigned to ${previous}; saving will reassign it.`:'Ready to save.';
  };capture.focus();
}
function openLibraryMenu(button,catalog){
  if(libraryMenuOwner===button){closeLibraryMenu(true);return;}
  closeLibraryMenu();libraryMenuOwner=button;button.setAttribute('aria-expanded','true');
  const menu=document.createElement('div');menu.id='library-actions-menu';menu.className='library-actions-menu';
  menu.setAttribute('role','menu');menu.setAttribute('aria-label',`Actions for ${catalog}`);
  menu.innerHTML=`<div class="tree-menu-title">${esc(catalog)}</div><button role="menuitem" data-library-command="set">Set hotkey…${libraryHotkeys[catalog]?' · '+esc(libraryHotkeys[catalog]):''}</button>${libraryHotkeys[catalog]?'<button role="menuitem" data-library-command="remove">Remove hotkey</button>':''}`;
  document.body.appendChild(menu);const rect=button.getBoundingClientRect();
  menu.style.left=Math.max(8,Math.min(rect.right-190,window.innerWidth-198))+'px';
  menu.style.top=Math.max(8,Math.min(rect.bottom+4,window.innerHeight-menu.offsetHeight-8))+'px';
  const items=[...menu.querySelectorAll('button')];for(const item of items){item.disabled=state.busy||state.placementPending;
    item.onclick=()=>{closeLibraryMenu();if(item.dataset.libraryCommand==='set')setLibraryHotkey(catalog);
      else{delete libraryHotkeys[catalog];const saved=saveLibraryHotkeys();renderLibrary();toast(saved?'Hotkey removed.':'Hotkey removed for this tab; browser storage is unavailable.');}};
  }
  menu.onkeydown=e=>{if(e.key==='Escape'){e.preventDefault();e.stopPropagation();closeLibraryMenu(true);return;}
    if(['ArrowDown','ArrowUp','Home','End'].includes(e.key)){e.preventDefault();const i=items.indexOf(document.activeElement);
      items[e.key==='Home'?0:e.key==='End'?items.length-1:(i+(e.key==='ArrowDown'?1:-1)+items.length)%items.length].focus();}
    if(e.key!=='Tab')e.stopPropagation();
  };items.find(item=>!item.disabled)?.focus();
}
document.addEventListener('pointerdown',e=>{if(!e.target.closest('#library-actions-menu, [data-library-actions]'))closeLibraryMenu();});
document.addEventListener('focusin',e=>{if(!e.target.closest('#library-actions-menu, [data-library-actions]'))closeLibraryMenu();});
function renderLibrary(){closeLibraryMenu();const query=$('#part-search').value.toLowerCase(),category=$('#category').value,size=$('#size-filter').value;const entries=Object.entries(state.library).filter(([id,p])=>{
  if(!(id+' '+p.name).toLowerCase().includes(query))return false;
  if(category==='motion'){if(['member','panel','load'].includes(p.kind))return false;if(id.startsWith('tubeclamp.')&&!/173|138|140|148|179/.test(id))return false;}else if(category!=='all'&&p.kind!==category)return false;
  if(size!=='all'&&id.startsWith('tubeclamp.')&&!id.endsWith(size))return false;return true;
});$('#library-count').textContent=entries.length+' COMPONENTS';$('#part-list').innerHTML=entries.map(([id,p])=>`<div class="part-card-row"><button class="part-card" draggable="true" data-catalog="${esc(id)}" title="Add ${esc(p.name||id)}"><span class="part-thumb">${icon(p)}</span><span class="part-card-text"><div class="part-code">${esc(id.split('.').slice(1).join('.'))}${libraryHotkeys[id]?` <span class="part-hotkey">${esc(libraryHotkeys[id])}</span>`:''}</div><div class="part-name">${esc((p.name||id).replace(/^\d+[MF]?\s*-\s*/,'').split(' · ')[0].replace(/\s*\([^)]*\)/g,''))}</div><div class="part-vendor">${esc(p.source?.supplier||'CUSTOMISABLE')}</div></span><span class="part-plus">+</span></button><button class="library-actions-toggle" data-library-actions="${esc(id)}" aria-label="Actions for ${esc(p.name||id)}" aria-haspopup="menu" aria-expanded="false" title="Part options">⋯</button></div>`).join('')||'<div class="empty-library">No matching parts.<br>Try another size or category.</div>';
  $$('.part-card').forEach(b=>{b.addEventListener('click',()=>addLibraryCatalog(b.dataset.catalog));b.addEventListener('dragstart',e=>{e.dataTransfer.setData('application/pipesim-part',b.dataset.catalog);e.dataTransfer.effectAllowed='copy';});});
  $$('[data-library-actions]').forEach(b=>{b.onclick=e=>{e.preventDefault();e.stopPropagation();openLibraryMenu(b,b.dataset.libraryActions);};
    b.onkeydown=e=>{if(['ArrowDown','ArrowUp'].includes(e.key)){e.preventDefault();openLibraryMenu(b,b.dataset.libraryActions);}};});
}
let treeMenuOwner=null;
function treeActionsButton(index,label){return `<button class="tree-actions-toggle" data-tree-actions="${index}" aria-label="Actions for ${esc(label)}" aria-haspopup="menu" aria-expanded="false" title="Actions for ${esc(label)}">⋯</button>`;}
function closeTreeMenu(focus=false){
  $('#tree-actions-menu')?.remove();const owner=treeMenuOwner;treeMenuOwner=null;
  owner?.setAttribute('aria-expanded','false');if(focus)owner?.focus();
}
function openTreeMenu(button,target){
  if(treeMenuOwner===button){closeTreeMenu(true);return;}closeTreeMenu();treeMenuOwner=button;button.setAttribute('aria-expanded','true');
  const menu=document.createElement('div');menu.id='tree-actions-menu';menu.className='tree-actions-menu';menu.setAttribute('role','menu');menu.setAttribute('aria-label',`Actions for ${target.label}`);
  menu.innerHTML=`<div class="tree-menu-title">${esc(target.label)} · ${target.members.length} parts</div><button role="menuitem" data-tree-command="properties">Show properties</button><button role="menuitem" data-tree-command="rename">Rename…</button>${target.duplicate?`<button role="menuitem" data-tree-command="duplicate">Duplicate ${target.part?'part':'subassembly'}</button>`:''}${target.reopen?`<button role="menuitem" data-tree-command="reopen">Return ${target.part?'pipe':'pipes'} to draft</button>`:''}<button role="menuitem" class="tree-delete" data-tree-command="delete">Delete ${target.part?'part':'subassembly'}</button>`;
  document.body.appendChild(menu);const rect=button.getBoundingClientRect();menu.style.left=Math.max(8,Math.min(rect.right-210,window.innerWidth-218))+'px';menu.style.top=Math.max(8,Math.min(rect.bottom+4,window.innerHeight-menu.offsetHeight-8))+'px';
  const items=[...menu.querySelectorAll('button')];items.forEach(item=>{item.disabled=state.busy||state.placementPending;item.onclick=()=>{
    closeTreeMenu();if(item.dataset.treeCommand==='rename'){renameTreeTarget(target);return;}if(item.dataset.treeCommand==='delete'){deleteTreeTarget(target);return;}if(item.dataset.treeCommand==='reopen'){reopenTreeTarget(target);return;}
    if(target.object)objectModes.set(target.object,'whole');setMode('design');select(target.members[0]);
    if(item.dataset.treeCommand==='duplicate')duplicateSelected(target.part?'part':'subassembly');
  };});
  menu.onkeydown=e=>{
    if(e.key==='Escape'){e.preventDefault();e.stopPropagation();closeTreeMenu(true);return;}
    if(['ArrowDown','ArrowUp','Home','End'].includes(e.key)){e.preventDefault();const i=items.indexOf(document.activeElement);items[e.key==='Home'?0:e.key==='End'?items.length-1:(i+(e.key==='ArrowDown'?1:-1)+items.length)%items.length].focus();}
    if(e.key!=='Tab')e.stopPropagation();
  };items.find(item=>!item.disabled)?.focus();
}
document.addEventListener('pointerdown',e=>{if(!e.target.closest('#tree-actions-menu, [data-tree-actions]'))closeTreeMenu();});
document.addEventListener('focusin',e=>{if(!e.target.closest('#tree-actions-menu, [data-tree-actions]'))closeTreeMenu();});
function renameTreeTarget(target){
  if(state.busy||state.placementPending||dragStart)return;
  const revision=state.revision;let cancelled=false;
  modal('Rename '+(target.object?'object':target.part?'part':'Body'),`<form id="rename-item-form"><div class="single-field"><label for="item-name">NAME</label><input id="item-name" required maxlength="120" value="${esc(target.label)}"></div><p id="rename-item-error" role="alert"></p></form>`,[
    {label:'Cancel',action:closeModal},{label:'Rename',primary:true,action:async()=>{
      const input=$('#item-name');if(!input.reportValidity())return;
      const name=input.value.trim();if(!name){$('#rename-item-error').textContent='Enter a name.';return;}if(name===target.label){closeModal();return;}
      if(state.placementPending)return;state.placementPending=true;
      try{
        const result=await api('rename',{name,...(target.object?{object:target.object}:target.part?{part:target.part}:{members:target.members})});
        if(cancelled||revision!==state.revision)return;
        if(result.changed)await acceptPlacement(result,revision);
        closeModal();toast('Renamed to '+result.name+'.');status('Name updated');
      }catch(error){if(!cancelled)$('#rename-item-error').textContent=error.message;}
      finally{state.placementPending=false;attachGizmo();}
    }}]);
  reviewCleanup=()=>{cancelled=true;};
  $('#rename-item-form').onsubmit=e=>{e.preventDefault();$('#modal-actions .primary').click();};$('#item-name').focus();$('#item-name').select();
}
async function deleteTreeTarget(target){
  if(state.busy||state.placementPending||dragStart)return;
  const revision=state.revision;state.placementPending=true;gizmo?.detach();status('Deleting '+target.label+'…');
  try{
    const result=await api('delete',target.object?{object:target.object}:{members:target.members});
    if(revision!==state.revision)return;
    if(result.deleted_parts.includes(state.selected))state.selected=null;
    if(result.deleted_parts.includes(state.connectionSource))state.connectionSource=null;
    await acceptPlacement(result,revision);toast(`Deleted ${target.label} · ${result.deleted_parts.length} parts. Undo restores them.`);status('Subassembly deleted');
  }catch(error){toast(error.message,true);status('Deletion failed · design unchanged');}
  finally{state.placementPending=false;attachGizmo();}
}
async function reopenTreeTarget(target){
  if(state.busy||state.placementPending||dragStart)return;
  const revision=state.revision;state.placementPending=true;gizmo?.detach();status('Returning '+target.label+' to draft…');
  try{
    const result=await api('draft-reopen',{members:target.members});
    if(revision!==state.revision)return;
    state.selected=result.converted_parts[0];
    await acceptPlacement(result,revision);
    toast(`${result.converted_parts.length} ${result.converted_parts.length===1?'pipe':'pipes'} returned to draft. Undo restores the exact assembly.`);
    status('Subassembly is editable in draft mode');
  }catch(error){toast(error.message,true);status('Return to draft failed · design unchanged');}
  finally{state.placementPending=false;attachGizmo();}
}
function renderOutline(){
  closeTreeMenu();if(!state.scene)return;
  const targets=[];const actions=target=>{targets.push(target);return treeActionsButton(targets.length-1,target.label);};
  const instances=state.doc.objects||[],direct=new Set(state.doc.parts.map(p=>p.id)),owned=new Set(),reopenable=id=>{const part=state.doc.parts.find(p=>p.id===id),definition=part&&state.library[part.catalog];return !!(part&&definition?.kind==='member'&&JSON.stringify(definition.geometry||[]).includes('$length_mm'));},partButton=id=>{const label=state.scene.parts.find(p=>p.id===id)?.label||id;return `<div class="outline-heading"><button class="outline-part ${state.selected===id?'active':''}" data-select="${esc(id)}" title="${esc(id)}"><span>◇</span>${esc(label)}</button>${actions({label,part:id,members:[id],duplicate:true,reopen:reopenable(id)})}</div>`;};
  const grouped=instances.map(instance=>{
    const members=state.scene.parts.filter(p=>!direct.has(p.id)&&p.id.startsWith(instance.id+'/')).map(p=>p.id);members.forEach(id=>owned.add(id));
    if(instance.template==='chain'){
      const info=state.scene.chains.find(c=>c.id===instance.id),open=openObjects.has(instance.id);
      const action=actions({label:instance.label||instance.id,object:instance.id,members,duplicate:true});
      return `<div class="object-outline"><div class="outline-heading"><button class="outline-part ${selectedObject()?.id===instance.id?'active':''}" data-object-select="${esc(instance.id)}"><span>⛓</span>${esc(instance.label||instance.id)} <span class="subtle">${info.length_mm} mm · ${members.length} links</span></button>${action}</div><details class="outline-group" data-object-tree="${esc(instance.id)}" ${open?'open':''}><summary>Individual links</summary>${open?members.map(partButton).join(''):''}</details></div>`;
    }
    const action=actions({label:instance.label||instance.id,object:instance.id,members,duplicate:true});
      return `<div class="object-outline"><div class="outline-heading"><button class="outline-part ${selectedObject()?.id===instance.id?'active':''}" data-object-select="${esc(instance.id)}"><span>♙</span>${esc(instance.label||instance.id)} <span class="subtle">${members.length} parts</span></button>${action}</div><details class="outline-group" data-object-tree="${esc(instance.id)}" ${openObjects.has(instance.id)?'open':''}><summary>Individual parts</summary>${members.map(partButton).join('')}</details></div>`;
  }).join('');
  const draftIds=new Set(draftRuns(state.doc).map(r=>r.id));
  const drafts=(state.doc.draft_subassemblies||[]).map(group=>`<details class="outline-group" open><summary>Draft · ${esc(group.id)} <span class="subtle">· ${group.runs.length} runs</span></summary>${group.runs.map(run=>`<div class="outline-heading"><button class="outline-part ${state.selected===run.id?'active':''}" data-select="${esc(run.id)}"><span>◇</span>${esc(run.id)}</button></div>`).join('')}</details>`).join('');
  const bodies=state.scene.groups.map(g=>g.filter(id=>!owned.has(id)&&!draftIds.has(id))).filter(g=>g.length);
  $('#outline-list').innerHTML=grouped+drafts+bodies.map((group,i)=>{const label=state.doc.metadata?.body_labels?.find(r=>r.parts.length===group.length&&r.parts.every(p=>group.includes(p)))?.label||'Body '+String(i+1).padStart(2,'0');const target={label,members:group,duplicate:state.scene.groups.find(g=>g.includes(group[0])).length===group.length,reopen:group.some(reopenable)};return `<details class="outline-group" open><summary data-tree-group="${targets.length}">⌄ ${esc(label)} <span class="subtle"> · ${group.length} parts</span>${actions(target)}</summary>${group.map(partButton).join('')}</details>`;}).join('');
  $$('#outline-list [data-tree-actions]').forEach(button=>{
    const target=targets[+button.dataset.treeActions];button.onclick=e=>{e.preventDefault();e.stopPropagation();openTreeMenu(button,target);};
    button.onkeydown=e=>{if(e.key==='Delete'){e.preventDefault();e.stopPropagation();deleteTreeTarget(target);}else if(['ArrowDown','ArrowUp'].includes(e.key)){e.preventDefault();e.stopPropagation();openTreeMenu(button,target);}};
  });
  $$('#outline-list [data-tree-group]').forEach(summary=>summary.onkeydown=e=>{if(e.key==='Delete'&&e.target===summary){e.preventDefault();e.stopPropagation();deleteTreeTarget(targets[+summary.dataset.treeGroup]);}});
  $$('#outline-list [data-select]').forEach(b=>b.onclick=()=>select(b.dataset.select));
  $$('[data-object-tree]').forEach(d=>d.ontoggle=()=>{if(!d.isConnected)return;const wasOpen=openObjects.has(d.dataset.objectTree);if(d.open)openObjects.add(d.dataset.objectTree);else openObjects.delete(d.dataset.objectTree);if(wasOpen!==d.open&&state.doc.objects.find(o=>o.id===d.dataset.objectTree)?.template==='chain')renderOutline();});
  $$('[data-object-select]').forEach(b=>b.onclick=async()=>{
    const id=b.dataset.objectSelect,instance=state.doc.objects.find(o=>o.id===id);objectModes.set(id,'whole');
    if(instance.template==='chain'&&instance.layout_mode==='posable')await objectEdit('object-layout',{object:id,mode:'rigid'});
    select(state.scene.parts.find(p=>p.id===id+'/pelvis')?.id||state.scene.parts.find(p=>p.id.startsWith(id+'/'))?.id);
  });
}
async function addPart(catalog,position=null,{origin=false,quiet=false}={}){const definition=state.library[catalog];if(!definition)return;const short=catalog.split('.').pop().toLowerCase().replace(/[^a-z0-9-]/g,'-');let n=1;while(state.doc.parts.some(p=>p.id===short+'-'+n)||draftRun(state.doc,short+'-'+n))n++;const id=short+'-'+n;const params=clone(definition.parameters||{});const z=definition.kind==='member'?(params.length_mm||1000)/2:definition.kind==='panel'?(params.thickness_mm||30)/2:params.height_mm?params.height_mm/2:50;
  if(definition.kind==='member'&&JSON.stringify(definition.geometry||[]).includes('$length_mm')){
    const start=position?[position[0],position[1],position[2]]:[0,-600,0],length=params.length_mm||1000;
    delete params.length_mm;checkpoint();state.doc.draft_subassemblies||=[{id:'draft-1',runs:[]}];
    state.doc.draft_subassemblies[0].runs.push({id,catalog,parameters:params,start_mm:start,end_mm:[start[0],start[1],start[2]+length],attachments:[]});
    state.selected=id;changed();refreshDraft(id,{structure:true});setMode('design');setTool('translate');if(!quiet)toast('Added '+id+' as a draft run. Connect its ends, then finalize the subassembly.');return id;
  }
  await mutate(()=>{state.doc.parts.push({id,catalog,parameters:params,pose:{position_mm:position?[position[0],position[1],position[2]+(origin?0:z)]:[0,-600,z],rotation_deg:[0,0,0]}});
    const mirrored=(state.doc.draft_subassemblies||[]).filter(group=>group.mirrors?.length);
    if(mirrored.length===1&&(mirrored[0].mirrors||[]).some(plane=>plane.scope!=='scene')){
      const existing=new Set(state.doc.parts.map(part=>part.id));
      mirrored[0].mirror_parts=[...new Set((mirrored[0].mirror_parts||[]).filter(part=>existing.has(part)))];
      if(!mirrored[0].mirror_parts.includes(id))mirrored[0].mirror_parts.push(id);
    }
    state.selected=id;});setMode('design');setTool('translate');if(!quiet)toast('Added '+id+'. Drag it onto a pipe or socket to connect.');return id;}

async function deleteDraft(id){
  if(state.busy||state.placementPending||!draftRun(state.doc,id))return;
  const revision=state.revision;state.placementPending=true;gizmo?.detach();status('Deleting draft run…');
  try{
    const result=await api('delete',{members:[id]});
    if(revision!==state.revision)return;
    state.selected=null;
    await acceptPlacement(result,revision);
    toast('Draft run deleted. Undo restores it.');status('Draft run deleted');
  }catch(error){toast(error.message,true);status('Deletion failed · design unchanged');}
  finally{state.placementPending=false;attachGizmo();}
}

function addDraftMirrorDialog(group,run){
  if((group.mirrors||[]).length>=3){toast('A draft subassembly can use one mirror plane per axis.',true);return;}
  modal('Add draft mirror',`<p>Mirror the scene while editing. Generated parts are previews until you keep the mirror or finalize.</p>
    <div class="single-field"><label for="mirror-axis">PLANE NORMAL</label><select id="mirror-axis">${['x','y','z'].filter(axis=>!(group.mirrors||[]).some(plane=>plane.axis===axis)).map(axis=>`<option value="${axis}">${axis.toUpperCase()} = offset</option>`).join('')}</select></div>
    <div class="single-field"><label for="mirror-offset">OFFSET · mm</label><input id="mirror-offset" type="number" step="any" required value="0"></div>
    <p>Common planes are X = 0, Y = 0, and Z at the height you choose.</p>`,[
    {label:'Cancel',action:closeModal},
    {label:'Add mirror',primary:true,action:async()=>{
      const input=$('#mirror-offset');if(!input.reportValidity())return;
      const axis=$('#mirror-axis').value,offset=Number(input.value);if(!Number.isFinite(offset))return;
      const used=new Set((group.mirrors||[]).map(plane=>plane.id));let index=1;while(used.has('mirror-'+index))index++;
      closeModal();await mutate(()=>{
        group.mirrors||=[];group.mirrors.push({id:'mirror-'+index,axis,offset_mm:offset,scope:'scene',run_modes:{}});
      });
      syncMirrorPreviews();fitView();
    }}]);
  let edited=false;
  $('#mirror-axis').onchange=()=>{if(!edited)$('#mirror-offset').value=$('#mirror-axis').value==='z'?Number(state.scene.parts.find(p=>p.id===run.id).pose.position_mm[2].toFixed(2)):0;};
  $('#mirror-offset').oninput=()=>{edited=true;};
}
function removeDraftMirrorDialog(group,plane){
  modal('Turn off mirror',`<p>Turn off ${esc(plane.axis.toUpperCase())} = ${esc(plane.offset_mm)} mm for ${esc(group.id)}.</p><p>Keep copies makes the reflected pipes and connectors independently editable. Discard copies removes only the generated previews.</p>`,[
    {label:'Cancel',action:closeModal},
    {label:'Discard copies',action:async()=>{closeModal();await mutate(()=>{group.mirrors=group.mirrors.filter(item=>item.id!==plane.id);if(!group.mirrors.length){delete group.mirrors;delete group.mirror_parts;}});syncMirrorPreviews();}},
    {label:'Keep copies',primary:true,action:async()=>{
      if(state.placementPending)return;const revision=state.revision;state.placementPending=true;gizmo?.detach();
      closeModal();status('Keeping mirrored draft geometry…');
      try{const result=await api('draft-mirror-bake',{group:group.id,plane:plane.id});if(revision!==state.revision)return;
        await acceptPlacement(result,revision);syncMirrorPreviews();toast('Mirror copies are now independent draft geometry.');}
      catch(error){toast(error.message,true);}finally{state.placementPending=false;attachGizmo();}
    }}]);
}

const DRAFT_SOCKET_CAPTURE_MM=10,DRAFT_SOCKET_CAPTURE_DEG=5;
function canAlignDraftConnector(id){
  if(!state.doc.parts.some(part=>part.id===id)||state.scene.anchors?.some(anchor=>anchor.part===id))return false;
  if(state.scene.joints?.some(joint=>joint.a.part===id||joint.b.part===id))return false;
  if(draftRuns(state.doc).some(run=>(run.attachments||[]).some(attachment=>attachment.connector===id)))return false;
  if((state.scene.groups||[]).some(group=>group.includes(id)&&group.length>1))return false;
  if((state.doc.draft_subassemblies||[]).some(group=>(group.mirror_parts||[]).includes(id)))return false;
  return true;
}
const REPAIRABLE_DRAFT_CONFLICTS=new Set(['THROUGH_FIT','POSITION_MISMATCH','AXIS_MISMATCH','LOCKED_LENGTH']);
function repairNewDraftConnection(runId){
  const preview=state.scene.parts.find(part=>part.id===runId);
  if(!preview?.conflicts?.some(conflict=>REPAIRABLE_DRAFT_CONFLICTS.has(conflict.code)))return;
  const revision=state.revision,document=state.doc;
  Promise.resolve().then(async()=>{
    if(revision!==state.revision||document!==state.doc||state.placementPending||dragStart)return;
    const job={id:Date.now().toString(36)+Math.random().toString(36).slice(2)};
    pendingAutomaticDraftRepair=job;state.placementPending=true;gizmo?.detach();
    status('Aligning connected draft structure…');
    try{
      const result=await api('draft-repair-selected',{job_id:job.id,run:runId});
      if(pendingAutomaticDraftRepair!==job||revision!==state.revision||document!==state.doc)return;
      if(result.status==='repaired'){
        pendingAutomaticDraftRepair=null;
        await acceptPlacement(result,revision,false);
        toast('Draft connection added and alignment repaired.');status('Draft graph updated');
      }else if(result.status==='conflict')status('Draft connection needs further adjustment');
      else status('Draft connection alignment checked');
    }catch(error){
      if(pendingAutomaticDraftRepair===job&&revision===state.revision&&document===state.doc){
        toast('Automatic draft alignment failed: '+error.message,true);
        status('Draft connection needs adjustment');
      }
    }finally{
      if(pendingAutomaticDraftRepair===job)pendingAutomaticDraftRepair=null;
      state.placementPending=false;attachGizmo();
    }
  });
}
function alignDraftConnector(run,fitting,socket,attachment,preview=null){
  if(!canAlignDraftConnector(fitting.id))return false;
  if(!socket.through&&(state.doc.draft_subassemblies||[]).some(group=>group.runs.includes(run)&&
    (group.mirrors||[]).some(plane=>plane.run_modes?.[run.id]&&plane.run_modes[run.id]!=='free')))return false;
  preview||=draftPreview(run,state.scene,state.library);
  const pipe=new THREE.Object3D();setPose(pipe,preview.pose);
  const start=new THREE.Vector3(0,0,-preview.length_mm/2).applyMatrix4(pipe.matrix);
  const direction=new THREE.Vector3(0,0,1).transformDirection(pipe.matrix);
  const object=partObjects.get(fitting.id);if(!object)return false;
  object.updateMatrix();
  const mouth=new THREE.Vector3(...(socket.position_mm||[0,0,0])).applyMatrix4(object.matrix);
  const axis=new THREE.Vector3(...(socket.axis||[0,0,1])).transformDirection(object.matrix);
  let targetMouth,targetAxis;
  if(socket.through){
    const station=mouth.clone().sub(start).dot(direction);
    const half=(socket.engagement_mm||0)/2;
    if(preview.length_mm<half*2)return false;
    targetMouth=start.clone().addScaledVector(direction,THREE.MathUtils.clamp(station,half,preview.length_mm-half));
    targetAxis=direction.clone().multiplyScalar(axis.dot(direction)<0?-1:1);
  }else{
    const end=attachment.end;
    if(end!=='start'&&end!=='end')return false;
    targetAxis=direction.clone().multiplyScalar(end==='start'?1:-1);
    targetMouth=start.clone().addScaledVector(direction,end==='start'?0:preview.length_mm)
      .addScaledVector(targetAxis,attachment.insertion_mm||0);
  }
  if(mouth.distanceTo(targetMouth)>DRAFT_SOCKET_CAPTURE_MM||
    axis.angleTo(targetAxis)>THREE.MathUtils.degToRad(DRAFT_SOCKET_CAPTURE_DEG))return false;
  const rotation=new THREE.Quaternion().setFromUnitVectors(axis,targetAxis).multiply(object.quaternion);
  const target=new THREE.Object3D();target.quaternion.copy(rotation);
  target.position.copy(targetMouth).sub(new THREE.Vector3(...(socket.position_mm||[0,0,0])).applyQuaternion(rotation));
  const pose=getPose(target),part=state.doc.parts.find(item=>item.id===fitting.id);
  part.pose=pose;fitting.pose=clone(pose);setPose(object,pose);
  return true;
}
function attachAlignedThroughSockets(run){
  const preview=draftPreview(run,state.scene,state.library),object=new THREE.Object3D();setPose(object,preview.pose);
  const start=new THREE.Vector3(0,0,-preview.length_mm/2).applyMatrix4(object.matrix);
  const direction=new THREE.Vector3(0,0,1).transformDirection(object.matrix);
  const section=preview.section,profile=['tube','round','circle'].includes(section.type)?'round':section.profile||section.type;
  let added=0;
  for(const fitting of state.scene.parts){
    if((run.attachments||[]).some(a=>a.connector===fitting.id))continue;
    const matrix=partObjects.get(fitting.id)?.matrix;if(!matrix)continue;
    for(const [name,socket] of Object.entries(fitting.ports||{})){
      if((run.attachments||[]).some(a=>a.connector===fitting.id))break;
      if(socket.type!=='socket'||!socket.through||socketOccupied(state.scene,fitting.id,name))continue;
      if(profile!==(socket.profile||'round')||Math.abs((section.diameter_mm||0)-(socket.diameter_mm||0))>.6)continue;
      const mouth=new THREE.Vector3(...(socket.position_mm||[0,0,0])).applyMatrix4(matrix);
      const axis=new THREE.Vector3(...(socket.axis||[0,0,1])).transformDirection(matrix);
      const station=mouth.clone().sub(start).dot(direction);
      const gap=mouth.distanceTo(start.clone().addScaledVector(direction,station));
      if(gap>Math.min(DRAFT_SOCKET_CAPTURE_MM,(socket.diameter_mm||0)/4)||
        Math.abs(axis.dot(direction))<Math.cos(THREE.MathUtils.degToRad(DRAFT_SOCKET_CAPTURE_DEG))||
        station<0||station>preview.length_mm)continue;
      alignDraftConnector(run,fitting,socket,{connector:fitting.id,port:name},preview);
      run.attachments.push({connector:fitting.id,port:name});added++;
    }
  }
  return added;
}
function draftConnect(match,recordUndo=true){
  const run=draftRun(state.doc,match.member),fitting=state.scene.parts.find(p=>p.id===match.connector),socket=fitting?.ports?.[match.port];
  if(!run||!socket||socket.type!=='socket')throw new Error('Choose a draft run and a socket');
  if((run.attachments||[]).some(a=>a.connector===match.connector))
    throw new Error('This pipe is already attached to this connector; one pipe cannot occupy two sockets of the same connector');
  if(socketOccupied(state.scene,match.connector,match.port))throw new Error('This socket or its shared bore is occupied');
  const section=state.scene.parts.find(p=>p.id===run.id).section,profile=['tube','round','circle'].includes(section.type)?'round':section.profile||section.type;
  if(profile!==(socket.profile||'round')||Math.abs((section.diameter_mm||0)-(socket.diameter_mm||0))>.6)throw new Error('The pipe size or profile does not match this socket');
  const attachment={connector:match.connector,port:match.port};
  if(!socket.through){attachment.end=match.end||(['start','end'].find(e=>!(run.attachments||[]).some(a=>a.end===e)));
    if(!attachment.end)throw new Error('Both pipe ends are already connected');
    if((run.attachments||[]).some(a=>a.end===attachment.end))throw new Error('That pipe end is already connected');
    attachment.insertion_mm=match.insertion_mm??Math.min(30,socket.engagement_mm*.8);
    if(attachment.insertion_mm<(socket.min_engagement_mm||0)||attachment.insertion_mm>socket.engagement_mm)throw new Error('Insertion is outside the socket engagement range');
  }
  if(recordUndo)checkpoint();
  alignDraftConnector(run,fitting,socket,attachment);
  run.attachments||=[];run.attachments.push(attachment);
  const owner=state.doc.draft_subassemblies.find(group=>group.runs.includes(run));
  const ends=Object.fromEntries(run.attachments.filter(item=>item.end).map(item=>[item.end,item]));
  if(attachment.end&&ends.start&&ends.end)for(const plane of owner.mirrors||[]){
    if(plane.run_modes?.[run.id]!=='centered')continue;
    const axis={x:0,y:1,z:2}[plane.axis],frames={};
    for(const [name,item] of Object.entries(ends)){
      const part=state.doc.parts.find(p=>p.id===item.connector),shown=state.scene.parts.find(p=>p.id===item.connector);
      const socket=shown?.ports?.[item.port],pose=part?.pose||{};
      if(!part||!socket)continue;
      const rotation=new THREE.Quaternion().setFromEuler(new THREE.Euler(...(pose.rotation_deg||[0,0,0]).map(THREE.MathUtils.degToRad),'ZYX'));
      const direction=new THREE.Vector3(...(socket.axis||[0,0,1])).applyQuaternion(rotation).normalize();
      const point=new THREE.Vector3(...(socket.position_mm||[0,0,0])).applyQuaternion(rotation)
        .add(new THREE.Vector3(...(pose.position_mm||[0,0,0])))
        .addScaledVector(direction,-item.insertion_mm);
      frames[name]={socket,direction,point};
    }
    if(!frames.start||!frames.end)continue;
    const current=frames[attachment.end];
    const component=current.direction.getComponent(axis);
    if(Math.abs(component)<.99999)continue;
    const midpoint=(frames.start.point.getComponent(axis)+frames.end.point.getComponent(axis))/2;
    const depth=attachment.insertion_mm+2*(midpoint-plane.offset_mm)/component;
    if(depth<(current.socket.min_engagement_mm||0)||depth>current.socket.engagement_mm)continue;
    current.point.addScaledVector(current.direction,-(depth-attachment.insertion_mm));
    attachment.insertion_mm=depth;
    run.start_mm=frames.start.point.toArray();run.end_mm=frames.end.point.toArray();
  }
  const additional=attachAlignedThroughSockets(run);changed();refreshDraft(run.id);updatePorts();repairNewDraftConnection(run.id);
  toast(`${additional+1} draft ${additional?'connections':'connection'} added. Finalize when the subassembly is ready.`);status('Draft graph updated');
}

function positionDraftRun(run,matrix){
  const length=state.scene.parts.find(p=>p.id===run.id).length_mm;
  run.start_mm=new THREE.Vector3(0,0,-length/2).applyMatrix4(matrix).toArray();
  run.end_mm=new THREE.Vector3(0,0,length/2).applyMatrix4(matrix).toArray();
  const direction=new THREE.Vector3().subVectors(new THREE.Vector3(...run.end_mm),new THREE.Vector3(...run.start_mm)).normalize();
  const reference=Math.abs(direction.y)<.95?new THREE.Vector3(0,1,0):new THREE.Vector3(1,0,0);
  const x=reference.clone().cross(direction).normalize(),y=direction.clone().cross(x);
  const basis=new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x,y,direction));
  const actual=new THREE.Quaternion().setFromRotationMatrix(matrix);
  const relative=basis.invert().multiply(actual);
  run.roll_deg=THREE.MathUtils.radToDeg(2*Math.atan2(relative.z,relative.w));
}
function draftMoveMembers(id){
  const members=new Set([id]);let changed=true;
  while(changed){
    changed=false;
    for(const run of draftRuns(state.doc))if(members.has(run.id)||(run.attachments||[]).some(a=>members.has(a.connector))){
      for(const part of [run.id,...(run.attachments||[]).map(a=>a.connector)])if(!members.has(part)){members.add(part);changed=true;}
    }
    for(const joint of state.scene.joints)if(joint.type==='fixed'||joint.locked){
      if(members.has(joint.a.part)||members.has(joint.b.part))for(const part of [joint.a.part,joint.b.part])if(!members.has(part)){members.add(part);changed=true;}
    }
  }
  return [...members];
}
function draftTranslationGroup(id){
  const members=draftMoveMembers(id),inside=new Set(members);
  if(members.some(part=>state.scene.anchors.some(anchor=>anchor.part===part))){toast('This draft structure is fixed to the world. Remove its anchor to move it.');return null;}
  if(members.some(part=>!draftRun(state.doc,part)&&!state.doc.parts.some(item=>item.id===part))){toast('Move the attached object directly to reposition this draft structure.');return null;}
  if(state.scene.joints.some(j=>inside.has(j.a.part)!==inside.has(j.b.part))){toast('A joint connects this draft structure to another part. Move its connector instead.');return null;}
  return members;
}
function translateDraftMembers(members,delta){
  delta=mirrorConstrainedDelta(members,delta);
  if(delta.lengthSq()<1e-12)return false;
  checkpoint();
  for(const id of members){
    const part=state.doc.parts.find(item=>item.id===id);
    if(part){part.pose||={};part.pose.position_mm=new THREE.Vector3(...(part.pose.position_mm||[0,0,0])).add(delta).toArray();
      const scenePart=state.scene.parts.find(item=>item.id===id);scenePart.pose.position_mm=[...part.pose.position_mm];
      setPose(partObjects.get(id),scenePart.pose);}
    else{const run=draftRun(state.doc,id);run.start_mm=new THREE.Vector3(...run.start_mm).add(delta).toArray();run.end_mm=new THREE.Vector3(...run.end_mm).add(delta).toArray();}
  }
  changed();for(const id of members)if(draftRun(state.doc,id))refreshDraft(id);
  updatePorts();renderInspector();attachGizmo();return true;
}
function resizeDraftSpan(run,preview,value,owner){
  if(!Number.isFinite(value)||value<=0||Math.abs(value-preview.length_mm)<1e-6)return;
  const endAttachments=(run.attachments||[]).filter(a=>a.end),centered=(owner.mirrors||[]).find(plane=>plane.run_modes?.[run.id]==='centered');
  let members=[run.id],delta=new THREE.Vector3();
  if(centered&&endAttachments.length===1){
    members=draftTranslationGroup(run.id);if(!members){renderInspector();return;}
    const axis={x:0,y:1,z:2}[centered.axis],object=new THREE.Object3D();setPose(object,preview.pose);
    const start=new THREE.Vector3(0,0,-preview.length_mm/2).applyMatrix4(object.matrix);
    const end=new THREE.Vector3(0,0,preview.length_mm/2).applyMatrix4(object.matrix);
    const sign=Math.sign(end.getComponent(axis)-start.getComponent(axis));
    delta.setComponent(axis,sign*(value-preview.length_mm)/2*(endAttachments[0].end==='start'?-1:1));
    for(const group of state.doc.draft_subassemblies||[])for(const plane of group.mirrors||[])
      for(const [id,mode] of Object.entries(plane.run_modes||{})){
        if(mode==='free'||!members.includes(id)||id===run.id&&plane===centered)continue;
        if(Math.abs(delta.getComponent({x:0,y:1,z:2}[plane.axis]))>1e-6){
          toast('Another mirror constraint holds this connected structure in place. Detach or move its fitting first.',true);renderInspector();return;
        }
      }
  }
  checkpoint();
  if(delta.lengthSq()>0)for(const id of members){
    const part=state.doc.parts.find(item=>item.id===id);
    if(part){part.pose||={};part.pose.position_mm=new THREE.Vector3(...(part.pose.position_mm||[0,0,0])).add(delta).toArray();
      const shown=state.scene.parts.find(item=>item.id===id);shown.pose.position_mm=[...part.pose.position_mm];setPose(partObjects.get(id),shown.pose);}
    else{const member=draftRun(state.doc,id);member.start_mm=new THREE.Vector3(...member.start_mm).add(delta).toArray();member.end_mm=new THREE.Vector3(...member.end_mm).add(delta).toArray();}
  }
  const start=new THREE.Vector3(...run.start_mm),direction=new THREE.Vector3(...run.end_mm).sub(start).normalize();
  run.end_mm=start.addScaledVector(direction,value).toArray();
  if(run.locked_length_mm!=null)run.locked_length_mm=value;
  changed({inferMirrors:!centered});
  for(const id of members)if(draftRun(state.doc,id))refreshDraft(id);
  updatePorts();renderInspector();attachGizmo();
}
function snappedDraftMatrix(match){
  const matrix=partObjects.get(match.member).matrix.clone();
  return alignmentDelta(state.scene,currentMatrices(),match,'member').multiply(matrix);
}

function currentMatrices(){return new Map([...partObjects].map(([id,o])=>{o.updateMatrix();return [id,o.matrix.clone()];}));}
function currentSnapSettings(){return {...state.snapSettings,gridEnabled:state.snap,connectionsEnabled:state.connectionSnap};}
function updateSnapControls(){
  const settings=currentSnapSettings();
  gizmo?.setTranslationSnap(settings.gridEnabled?settings.translationMm:null);
  // Apply reference alignment before quantising the raw rotation ourselves.
  gizmo?.setRotationSnap(settings.gridEnabled&&!settings.alignEnabled?THREE.MathUtils.degToRad(settings.rotationDeg):null);
  $('#snap-button').classList.toggle('active',settings.gridEnabled);$('#snap-button').setAttribute('aria-pressed',String(settings.gridEnabled));
  $('#snap-button').title=`Position ${settings.translationMm} mm · rotation ${settings.rotationDeg}°`;
  $('#snap-summary').textContent=settings.gridEnabled?`${settings.translationMm} mm · ${settings.rotationDeg}°`:settings.alignEnabled?'Align to parts':'Free placement';
  $('#connection-snap-button').classList.toggle('active',settings.connectionsEnabled);$('#connection-snap-button').setAttribute('aria-pressed',String(settings.connectionsEnabled));
}
function snapRotation(selected){
  const settings=currentSnapSettings();dragStart.alignment=null;
  if(!settings.alignEnabled)return;
  const axis=({X:new THREE.Vector3(1,0,0),Y:new THREE.Vector3(0,1,0),Z:new THREE.Vector3(0,0,1)})[gizmo.axis]||gizmo.rotationAxis?.clone();
  if(axis&&dragStart.rotationSpace==='local'&&['X','Y','Z'].includes(gizmo.axis))
    axis.applyQuaternion(new THREE.Quaternion().setFromRotationMatrix(dragStart.gizmoMatrix));
  const match=rotationAlignment(state.scene,currentMatrices(),dragStart.group,dragStart.id,settings.alignmentDeg,axis);
  if(match){selected.quaternion.premultiply(match.correction);dragStart.alignment='Aligned with '+match.label;}
  else if(settings.gridEnabled&&axis?.lengthSq()>0){
    axis.normalize();const original=new THREE.Quaternion().setFromRotationMatrix(dragStart.matrices.get(dragStart.id));
    const delta=selected.quaternion.clone().multiply(original.clone().invert());
    const angle=2*Math.atan2(new THREE.Vector3(delta.x,delta.y,delta.z).dot(axis),delta.w);
    const step=THREE.MathUtils.degToRad(settings.rotationDeg);
    selected.quaternion.copy(original).premultiply(new THREE.Quaternion().setFromAxisAngle(axis,Math.round(angle/step)*step));
    dragStart.alignment=`Angle snap ${settings.rotationDeg}°`;
  }
  selected.updateMatrix();
}
function snapSettingsDialog(){
  if(state.placementPending){toast('Finish the connection preview before changing preferences.');return;}
  const settings=currentSnapSettings();
  modal('Preferences',`<h3>SNAPPING &amp; GRID</h3><label class="check-label"><input id="settings-grid" type="checkbox">Use position and angle increments</label><div class="fields snap-settings-fields"><div class="field"><label for="settings-position">POSITION · mm</label><input id="settings-position" type="number" min="0.1" max="10000" step="any"></div><div class="field"><label for="settings-angle">ROTATION · °</label><input id="settings-angle" type="number" min="0.1" max="180" step="any"></div></div><div class="snap-presets"><button data-snap-preset="90">Frame · 90°</button><button data-snap-preset="45">Brace · 45°</button><button data-snap-preset="15">Fine · 15°</button></div><label class="check-label"><input id="settings-align" type="checkbox">Align rotation with existing parts and world axes</label><div class="single-field"><label for="settings-alignment">ALIGNMENT WINDOW · °</label><input id="settings-alignment" type="number" min="0.1" max="20" step="any"></div><p>Nearby reference axes take priority over the rotation increment. Turn alignment off to use only your chosen increment.</p><label class="check-label"><input id="settings-connections" type="checkbox">Snap and connect on drop</label><div class="single-field"><label for="settings-capture">CONNECTION REACH ON SCREEN · px</label><input id="settings-capture" type="number" min="4" max="100" step="any"></div><p>Apply changes this session. Save as defaults remembers them in this browser.</p>`,[
    {label:'Cancel',action:closeModal},
    {label:'Apply',action:()=>apply(false)},
    {label:'Save as defaults',primary:true,action:()=>apply(true)}
  ]);
  const options=(values,current)=>values.map(([value,label])=>`<option value="${esc(value)}" ${value===current?'selected':''}>${esc(label)}</option>`).join('');
  $('#modal-content').insertAdjacentHTML('beforeend',`<div class="preferences-sections">
    <section><h3>STARTUP</h3><div class="single-field"><label>OPEN ON START</label><select id="pref-startup">${options([['workspace','Last saved design or workspace default'],['autosave','Latest autosave'],['empty','Empty design'],['example','Example design']],preferences.startup)}</select></div>
      <div class="single-field"><label>EXAMPLE</label><select id="pref-example">${options((state.examples||[]).map(path=>[path,path.split('/').pop()]),preferences.startupExample||state.examples?.[0])}</select></div>
      <label class="check-label"><input id="pref-fit" type="checkbox">Fit design in view when opened</label><div class="single-field"><label>DEFAULT TOOL</label><select id="pref-tool">${options([['select','Select'],['translate','Move'],['rotate','Rotate'],['resize','Resize length'],['connect','Connect']],preferences.defaultTool)}</select></div></section>
    <section><h3>AUTOSAVE</h3><label class="check-label"><input id="pref-autosave" type="checkbox">Keep recovery copies in the workspace</label>
      <div class="single-field"><label>FOLDER INSIDE WORKSPACE</label><input id="pref-directory" value="${esc(preferences.autosaveDirectory)}"></div>
      <div class="fields"><div class="field"><label>EVERY · MINUTES</label><input id="pref-minutes" type="number" min="1" max="120" step="1" value="${preferences.autosaveMinutes}"></div>
      <div class="field"><label>COPIES PER DESIGN</label><input id="pref-keep" type="number" min="1" max="100" step="1" value="${preferences.autosaveKeep}"></div></div>
      <p>Autosaves are separate recovery files. Saving your design still uses Save.</p></section>
    <section><h3>DISPLAY UNITS</h3><div class="fields"><div class="field"><label>LENGTH</label><select id="pref-length">${options([['mm','Millimetres'],['cm','Centimetres'],['m','Metres'],['in','Inches'],['ft','Feet']],preferences.lengthUnit)}</select></div>
      <div class="field"><label>MASS</label><select id="pref-mass">${options([['kg','Kilograms'],['g','Grams'],['lb','Pounds']],preferences.massUnit)}</select></div>
      <div class="field"><label>FORCE</label><select id="pref-force">${options([['N','Newtons'],['kN','Kilonewtons'],['lbf','Pounds force']],preferences.forceUnit)}</select></div></div>
      <p>Design files, simulation, and APIs remain in mm, kg, and N.</p></section>
    <section><h3>VIEW &amp; HISTORY</h3><label class="check-label"><input id="pref-grid" type="checkbox">Show ground grid</label>
      <label class="check-label"><input id="pref-floor" type="checkbox">Show floor</label>
      <label class="check-label"><input id="pref-ports" type="checkbox">Show connection ports</label>
      <label class="check-label"><input id="pref-fit-duplicate" type="checkbox">Fit view after duplicate</label>
      <div class="single-field"><label>VIEWPORT THEME</label><select id="pref-theme">${options([['system','Follow system'],['light','Light'],['dark','Dark']],preferences.theme)}</select></div>
      <div class="single-field"><label>CAMERA FIELD OF VIEW · °</label><input id="pref-fov" type="number" min="20" max="90" step="1" value="${preferences.cameraFovDeg}"></div>
      <div class="single-field"><label>UNDO STEPS TO KEEP</label><input id="pref-undo" type="number" min="10" max="500" step="1" value="${preferences.undoLimit}"></div></section>
    <section><h3>KEYBOARD NUDGES</h3><div class="fields"><div class="field"><label for="settings-keyboard-move">KEYBOARD NUDGE DISTANCE · mm</label><input id="settings-keyboard-move" type="number" min="0.1" max="10000" step="any"></div>
      <div class="field"><label for="settings-keyboard-rotate">KEYBOARD ROTATION STEP · °</label><input id="settings-keyboard-rotate" type="number" min="0.1" max="180" step="any"></div></div>
      <p>W/A/S/D move in the XY plane; F/V move up/down. Z/X/C rotate around world X/Y/Z. Shift uses one tenth of the keyboard step; Alt reverses rotation. With the R rotation gizmo, hold Shift to use the part’s local axes instead of world axes. Alt+Shift+D duplicates the selected part; Alt+D also works when the browser does not reserve it. G/R keep the move and rotate gizmos. Home frames the design. Keyboard steps are separate from snap increments.</p></section>
    <section><h3>NEW ITEMS</h3><div class="fields"><div class="field"><label>DUPLICATE COPIES</label><input id="pref-duplicate-count" type="number" min="1" max="100" step="1" value="${preferences.defaultDuplicateCount}"></div>
      <div class="field"><label>HUMAN HEIGHT · mm</label><input id="pref-human-height" type="number" min="500" max="2500" step="1" value="${preferences.defaultHumanHeightMm}"></div>
      <div class="field"><label>HUMAN MASS · kg</label><input id="pref-human-mass" type="number" min="10" max="250" step="any" value="${preferences.defaultHumanMassKg}"></div></div>
      <div class="single-field"><label>FLEXIBLE LINE LENGTH · mm</label><input id="pref-chain-length" type="number" min="1" max="100000" step="1" value="${preferences.defaultChainLengthMm}"></div></section>
    <section><h3>CONNECTIONS</h3><label class="check-label"><input id="pref-lock" type="checkbox">Secure new socket connections by default</label>
      <div class="single-field"><label>DEFAULT FIT TOLERANCE · mm</label><input id="pref-tolerance" type="number" min="0.01" max="20" step="any" value="${preferences.connectionToleranceMm}"></div></section>
    <section><h3>RESIZE LENGTH</h3><p>Select a pipe or flexible line, then drag either gold end handle. Both ends keep the opposite end fixed. Hold Shift to use the alternate connector behavior.</p>
      <div class="fields"><div class="field"><label>CONNECTORS WHEN DRAGGING</label><select id="pref-resize-mode">${options([['follow','Follow the pipe'],['detach','Leave behind and disconnect']],preferences.resizeConnectorMode)}</select></div>
      <div class="field"><label>WITH SHIFT HELD</label><select id="pref-resize-shift">${options([['detach','Leave behind and disconnect'],['follow','Follow the pipe']],preferences.resizeShiftMode)}</select></div></div>
      <label class="check-label"><input id="pref-resize-auto" type="checkbox">Connect newly covered free through sockets</label>
      <div class="fields"><div class="field"><label>CONNECTION REACH · mm</label><input id="pref-resize-capture" type="number" min="0" max="100" step="any" value="${preferences.resizeCaptureMm}"></div>
      <div class="field"><label>AXIS WINDOW · °</label><input id="pref-resize-angle" type="number" min="0" max="45" step="any" value="${preferences.resizeCaptureDeg}"></div></div>
      <div class="single-field"><label>KEYBOARD RESIZE STEP · mm</label><input id="pref-resize-step" type="number" min="0.1" max="1000" step="any" value="${preferences.resizeKeyStepMm}"></div>
      <div class="fields"><div class="field"><label>FIRST HANDLE · GROW</label><input id="pref-resize-key-1-grow" maxlength="1" value="${esc(preferences.resizeHandle1GrowKey)}"></div>
      <div class="field"><label>FIRST HANDLE · SHRINK</label><input id="pref-resize-key-1-shrink" maxlength="1" value="${esc(preferences.resizeHandle1ShrinkKey)}"></div></div>
      <div class="fields"><div class="field"><label>SECOND HANDLE · GROW</label><input id="pref-resize-key-2-grow" maxlength="1" value="${esc(preferences.resizeHandle2GrowKey)}"></div>
      <div class="field"><label>SECOND HANDLE · SHRINK</label><input id="pref-resize-key-2-shrink" maxlength="1" value="${esc(preferences.resizeHandle2ShrinkKey)}"></div></div>
      <p>The first and second handles correspond to the two visible ends. Shift also applies to keyboard resizing. A flexible line changes by at least one full segment per keypress.</p></section>
    <section><h3>RENDER &amp; SIMULATION</h3><div class="fields"><div class="field"><label>IMAGE WIDTH · px</label><input id="pref-width" type="number" min="320" max="4096" step="1" value="${preferences.renderWidth}"></div>
      <div class="field"><label>IMAGE HEIGHT · px</label><input id="pref-height" type="number" min="240" max="4096" step="1" value="${preferences.renderHeight}"></div></div>
      <div class="fields"><div class="field"><label>LIGHTING</label><select id="pref-light">${options([['studio','Studio'],['technical','Technical'],['flat','Flat']],preferences.renderLighting)}</select></div>
      <div class="field"><label>BACKGROUND</label><select id="pref-background">${options([['#edf1f3','Soft grey'],['#ffffff','White'],['transparent','Transparent']],preferences.renderBackground)}</select></div></div>
      <div class="fields"><div class="field"><label>SIMULATION · SECONDS</label><input id="pref-sim-seconds" type="number" min="0.1" max="30" step="any" value="${preferences.simulationSeconds}"></div>
      <div class="field"><label>CHAIN LINKS PER BODY</label><input id="pref-chain-links" type="number" min="1" max="1000" step="1" value="${preferences.simulationChainLinks}"></div></div>
      <div class="single-field"><label>YELLOW DEFLECTION WARNING · mm</label><input id="pref-deflection-warning" type="number" min="0.01" max="100000" step="any" value="${preferences.deflectionWarningMm}"></div>
      <p>In validation and playback, red marks predicted yielding or possible fracture. Yellow marks displacement above this threshold.</p></section>
  </div>`);
  for(const [id,value] of [['fit',preferences.fitOnOpen],['autosave',preferences.autosaveEnabled],['grid',preferences.gridVisible],['floor',preferences.floorVisible],['ports',preferences.portsVisible],['fit-duplicate',preferences.fitAfterDuplicate],['lock',preferences.connectionLocked],['resize-auto',preferences.resizeAutoConnect]])$('#pref-'+id).checked=value;
  $('#settings-grid').checked=settings.gridEnabled;$('#settings-position').value=settings.translationMm;$('#settings-angle').value=settings.rotationDeg;
  $('#settings-align').checked=settings.alignEnabled;$('#settings-alignment').value=settings.alignmentDeg;
  $('#settings-connections').checked=settings.connectionsEnabled;$('#settings-capture').value=settings.connectionPixels;
  $('#settings-keyboard-move').value=settings.keyboardMoveMm;$('#settings-keyboard-rotate').value=settings.keyboardRotateDeg;
  $$('[data-snap-preset]').forEach(button=>button.onclick=()=>{$('#settings-angle').value=button.dataset.snapPreset;$('#settings-grid').checked=true;});
  function apply(persist){
    for(const input of $$('#modal-content input[type="number"]'))if(!input.reportValidity())return;
    let next=validateSnapSettings({gridEnabled:$('#settings-grid').checked,translationMm:$('#settings-position').value,rotationDeg:$('#settings-angle').value,
      alignEnabled:$('#settings-align').checked,alignmentDeg:$('#settings-alignment').value,connectionsEnabled:$('#settings-connections').checked,connectionPixels:$('#settings-capture').value,
      keyboardMoveMm:$('#settings-keyboard-move').value,keyboardRotateDeg:$('#settings-keyboard-rotate').value});
    let chosen;try{chosen=validatePreferences({autosaveEnabled:$('#pref-autosave').checked,autosaveDirectory:$('#pref-directory').value,
      autosaveMinutes:$('#pref-minutes').value,autosaveKeep:$('#pref-keep').value,startup:$('#pref-startup').value,
      startupExample:$('#pref-example').value,lengthUnit:$('#pref-length').value,massUnit:$('#pref-mass').value,
      forceUnit:$('#pref-force').value,gridVisible:$('#pref-grid').checked,floorVisible:$('#pref-floor').checked,portsVisible:$('#pref-ports').checked,
      theme:$('#pref-theme').value,undoLimit:$('#pref-undo').value,defaultTool:$('#pref-tool').value,fitOnOpen:$('#pref-fit').checked,
      fitAfterDuplicate:$('#pref-fit-duplicate').checked,cameraFovDeg:$('#pref-fov').value,
      defaultDuplicateCount:$('#pref-duplicate-count').value,defaultHumanHeightMm:$('#pref-human-height').value,
      defaultHumanMassKg:$('#pref-human-mass').value,defaultChainLengthMm:$('#pref-chain-length').value,
      renderWidth:$('#pref-width').value,renderHeight:$('#pref-height').value,renderLighting:$('#pref-light').value,
      renderBackground:$('#pref-background').value,simulationSeconds:$('#pref-sim-seconds').value,
      simulationChainLinks:$('#pref-chain-links').value,deflectionWarningMm:$('#pref-deflection-warning').value,connectionLocked:$('#pref-lock').checked,
      connectionToleranceMm:$('#pref-tolerance').value,resizeConnectorMode:$('#pref-resize-mode').value,
      resizeShiftMode:$('#pref-resize-shift').value,resizeAutoConnect:$('#pref-resize-auto').checked,
      resizeCaptureMm:$('#pref-resize-capture').value,resizeCaptureDeg:$('#pref-resize-angle').value,
      resizeKeyStepMm:$('#pref-resize-step').value,
      resizeHandle1GrowKey:$('#pref-resize-key-1-grow').value,resizeHandle1ShrinkKey:$('#pref-resize-key-1-shrink').value,
      resizeHandle2GrowKey:$('#pref-resize-key-2-grow').value,resizeHandle2ShrinkKey:$('#pref-resize-key-2-shrink').value});}
    catch(error){toast(error.message,true);return;}
    if(persist)next=saveSnapDefaults(window.localStorage,next);
    state.snapSettings=next;state.snap=next.gridEnabled;state.connectionSnap=next.connectionsEnabled;updateSnapControls();clearSnapPreview();closeModal();
    preferences=persist?savePreferences(window.localStorage,chosen):chosen;
    grid.visible=preferences.gridVisible;$('#grid-button').classList.toggle('active',grid.visible);
    floor.visible=preferences.floorVisible;camera.fov=preferences.cameraFovDeg;camera.updateProjectionMatrix();
    state.ports=preferences.portsVisible;$('#ports-button').classList.toggle('active',state.ports);updatePorts();
    applyViewportTheme();state.undo.splice(0,Math.max(0,state.undo.length-preferences.undoLimit));
    updateHeader();renderInspector();applyStructuralColors();if(state.mode==='simulate'&&state.recording)showFrame(state.frame);updateResizeHandles();scheduleAutosave();
    toast(persist?'Preferences saved in this browser.':'Preferences applied for this session.');
  }
}
function clearSnapPreview(){dispose(snapGhost);$('#snap-hint').classList.add('hidden');}
function previewCopy(id,pose,ghost=false,definition=null){
  let source=partObjects.get(id);if(!source)return null;
  if(definition){source=new THREE.Group();for(const shape of definition.geometry)source.add(meshShape(shape,definition.color,definition.kind));setPose(source,definition.pose);}
  const copy=source.clone(true);if(pose)setPose(copy,pose);
  copy.traverse(o=>{if(!o.isMesh)return;o.geometry=o.geometry.clone();o.material=(Array.isArray(o.material)?o.material:[o.material]).map(m=>{const c=m.clone();if(ghost){c.color.set('#48c6a0');c.transparent=true;c.opacity=.48;c.depthWrite=false;c.emissive?.set('#246750');}else{c.emissive?.set('#000000');}return c;});if(o.material.length===1)o.material=o.material[0];o.castShadow=false;o.receiveShadow=false;});
  if(definition)dispose(source);return copy;
}
function showPosePreview(poses,definitions=[]){clearSnapPreview();for(const id of new Set([...Object.keys(poses),...definitions.map(p=>p.id)])){const definition=definitions.find(p=>p.id===id);const copy=previewCopy(id,poses[id]||definition?.pose,true,definition);if(copy)snapGhost.add(copy);}}
function dragCandidates(group){
  if(!state.connectionSnap)return [];
  const rect=viewport.getBoundingClientRect(),matrices=currentMatrices(),settings=currentSnapSettings();
  return [...connectionCandidates(state.scene,matrices,group,camera,rect.width,rect.height,settings),
    ...hingeCandidates(state.scene,matrices,group,camera,rect.width,rect.height,settings)]
    .sort((a,b)=>a.score-b.score).slice(0,6);
}
let lastSnapTime=0;
function showDragSnap(){
  if(!dragStart||performance.now()-lastSnapTime<65)return;lastSnapTime=performance.now();
  const matches=dragStart.matches?.length?dragStart.matches:dragCandidates(dragStart.group);clearSnapPreview();if(!matches.length){if(dragStart.alignment){$('#snap-hint').textContent=dragStart.alignment;$('#snap-hint').classList.remove('hidden');}return;}
  const best=matches[0];
  if(best.kind==='hinge'){
    const marker=new THREE.Mesh(new THREE.SphereGeometry(10,12,8),new THREE.MeshBasicMaterial({color:'#48c6a0',depthTest:false}));
    marker.position.fromArray(best.point);marker.renderOrder=150;snapGhost.add(marker);
    $('#snap-hint').textContent=best.label+' · Release to join the bolt holes';$('#snap-hint').classList.remove('hidden');
    return;
  }
  const side=dragStart.group.includes(best.connector)?'connector':'member';
  const id=side==='connector'?best.connector:best.member,group=state.scene.groups.find(g=>g.includes(id))||[id];
  const matrices=currentMatrices(),delta=alignmentDelta(state.scene,matrices,best,side),poses={};
  for(const part of group){const object=new THREE.Object3D();delta.clone().multiply(matrices.get(part)).decompose(object.position,object.quaternion,object.scale);poses[part]=getPose(object);}
  showPosePreview(poses);
  const marker=new THREE.Mesh(new THREE.SphereGeometry(10,12,8),new THREE.MeshBasicMaterial({color:'#48c6a0',depthTest:false}));marker.position.fromArray(best.point);marker.renderOrder=150;snapGhost.add(marker);
  $('#snap-hint').textContent=best.label+' · '+(clearConnectionIntent(matches)?'Release to connect':'Release to choose alignment');$('#snap-hint').classList.remove('hidden');
}
function beginPlacement(kind){
  if(state.placementPending||!state.selected)return false;
  const draft=draftRun(state.doc,state.selected);
  if(draft?.attachments?.length&&state.tool==='rotate'){toast('Rotate a connector to change an attached draft run’s direction.');return false;}
  const draftGroup=draft?.attachments?.length?draftTranslationGroup(draft.id):null;
  if(draft?.attachments?.length&&!draftGroup)return false;
  const instance=wholeObject(),group=draftGroup|| (instance?state.scene.parts.filter(p=>p.id.startsWith(instance.id+'/')).map(p=>p.id):state.scene.groups.find(g=>g.includes(state.selected))||[state.selected]);
  if(state.scene.anchors.some(a=>group.includes(a.part))){toast('This body is fixed to the world. Remove its anchor to move it.');return false;}
  const constrained=!draft&&(state.scene.joints.some(j=>group.includes(j.a.part)!==group.includes(j.b.part))||group.some(id=>!state.doc.parts.some(p=>p.id===id))||!!state.doc.state?.joints);
  dragStart={kind,id:state.selected,group,connectedDraft:!!draftGroup,constrained,mode:state.tool==='rotate'?'rotate':'translate',rotationSpace:gizmo.space,revision:state.revision,
    objectId:instance?.id,objectPose:clone(instance?.pose||{}),
    handleMatrix:(instance?wholeObjectHandle:rotationHandle).matrix.clone(),gizmoMatrix:(gizmo.object||partObjects.get(state.selected)).matrix.clone(),
    matrices:new Map(group.map(id=>[id,partObjects.get(id).matrix.clone()]))};
  if(kind==='pointer'||state.tool==='translate')dragStart.placement=placementContext(group);
  return true;
}
function drawMovement(drag){
  for(const part of state.scene.parts){const object=partObjects.get(part.id);if(object)setPose(object,drag.result?.poses[part.id]||part.pose);}
  updatePorts();showDragSnap();
}
function previewMovement(){
  const drag=dragStart;if(!drag)return;
  if(!drag.constrained){updatePorts();if(!drag.connectedDraft)showDragSnap();return;}
  drag.target=getPose(partObjects.get(drag.id));
  if(drag.objectId){
    const root=new THREE.Object3D();setPose(root,drag.objectPose);
    const delta=partObjects.get(drag.id).matrix.clone().multiply(drag.matrices.get(drag.id).clone().invert());
    delta.multiply(root.matrix).decompose(root.position,root.quaternion,root.scale);drag.target=getPose(root);
  }
  drag.pending=clone(drag.target);
  drag.rawPoses=Object.fromEntries(drag.group.map(id=>[id,getPose(partObjects.get(id))]));drag.matches=dragCandidates(drag.group);drawMovement(drag);
  if(drag.job)return;
  drag.job=(async()=>{
    while(drag.pending&&dragStart===drag){
      const target=drag.pending;drag.pending=null;
      try{
        const result=await api(drag.objectId?'move-object':'move',{object:drag.objectId,selected:drag.id,target,mode:drag.mode,seed:drag.seed,preview:true,pose_only:true});
        if(dragStart!==drag||drag.revision!==state.revision)return;
        drag.seed=result.seed;
        if(!drag.pending){drag.result=result;drag.resultTarget=target;drag.error=null;drawMovement(drag);status(result.message);}
      }catch(e){if(dragStart===drag&&!drag.pending){drag.error=e;status(e.message);}}
    }
  })().finally(()=>{drag.job=null;});
}
function restorePlacement(){dragStart=null;for(const part of state.scene.parts){const object=partObjects.get(part.id);if(object)setPose(object,part.pose);}clearSnapPreview();updatePorts();state.placementPending=false;attachGizmo();}
async function acceptPlacement(result,revision,recordUndo=true){
  if(revision!==state.revision){restorePlacement();return;}
  state.placementPending=true;gizmo?.detach();
  if(recordUndo)checkpoint();state.doc=result.document;changed();clearSnapPreview();
  try{if(result.scene){buildScene(result.scene);renderOutline();}else await resolve();}
  finally{state.placementPending=false;renderInspector();attachGizmo();}
}
async function placeWithoutConnection(poses,revision,recordUndo=true){
  try{const result=await api('move',{poses});await acceptPlacement(result,revision,recordUndo);status('Placement updated');}
  catch(e){restorePlacement();toast(e.message,true);}
}
async function finishPlacement(){
  const drag=dragStart;if(!drag)return;
  if(drag.connectedDraft){
    const delta=new THREE.Vector3().setFromMatrixPosition(partObjects.get(drag.id).matrix)
      .sub(new THREE.Vector3().setFromMatrixPosition(drag.matrices.get(drag.id)));
    dragStart=null;clearSnapPreview();
    if(drag.revision!==state.revision){restorePlacement();return;}
    if(!translateDraftMembers(drag.group,delta))restorePlacement();
    else status('Connected draft structure moved');
    return;
  }
  if(draftRun(state.doc,drag.id)){
    const run=draftRun(state.doc,drag.id),matches=dragCandidates([drag.id]);
    const matrix=matches.length?snappedDraftMatrix(matches[0]):partObjects.get(drag.id).matrix;
    dragStart=null;clearSnapPreview();checkpoint();positionDraftRun(run,matrix);changed();refreshDraft(run.id);
    if(matches.length){try{draftConnect(matches[0],false);}catch(e){toast(e.message,true);}}
    else{const added=attachAlignedThroughSockets(run);if(added){changed();refreshDraft(run.id);updatePorts();repairNewDraftConnection(run.id);toast(`${added} draft ${added===1?'connection':'connections'} added.`);}else status('Draft run moved');}
    state.placementPending=false;attachGizmo();return;
  }
  if(drag.constrained){
    state.placementPending=true;gizmo?.detach();await drag.job;
    if(dragStart!==drag)return;
    if(drag.revision!==state.revision){restorePlacement();return;}
    if(drag.error){restorePlacement();toast(drag.error.message,true);return;}
    const matches=drag.matches?.length?drag.matches:dragCandidates(Object.keys(drag.result?.poses||{}));
    if(!matches.length&&(!drag.result||!Object.keys(drag.result.poses).length)){const message=drag.result?.message;restorePlacement();if(message)status(message);return;}
    const poses=drag.matches?.length?drag.rawPoses:drag.result.poses;clearSnapPreview();
    if(matches.length){dragStart=null;await offerConnection(matches,poses,drag.revision);}
    else{
      try{
        const result=await api(drag.objectId?'move-object':'move',{object:drag.objectId,selected:drag.id,target:drag.resultTarget,mode:drag.mode,seed:drag.seed,preview_id:drag.result.preview_id});
        if(dragStart!==drag)return;
        dragStart=null;await acceptPlacement(result,drag.revision);status(result.message);
      }catch(e){if(dragStart===drag){restorePlacement();toast(e.message,true);}}
    }
    return;
  }
  const poses=Object.fromEntries(drag.group.map(id=>[id,getPose(partObjects.get(id))]));
  if(drag.group.every(id=>partObjects.get(id).matrix.elements.every((v,i)=>Math.abs(v-drag.matrices.get(id).elements[i])<1e-6))){restorePlacement();return;}
  const matches=dragCandidates(drag.group);dragStart=null;clearSnapPreview();
  if(drag.revision!==state.revision){restorePlacement();return;}
  state.placementPending=true;gizmo?.detach();
  if(!matches.length){await placeWithoutConnection(poses,drag.revision);return;}
  await offerConnection(matches,poses,drag.revision);
}
async function offerConnection(matches,poses={},revision=state.revision,recordUndo=true,movingId=null){
  if(matches[0]?.kind==='hinge'){
    state.placementPending=true;gizmo?.detach();status('Aligning hinge bolt holes…');
    try{
      const match=matches[0],move=match.moving===match.male.part?'a':'b';
      const result=await api('connect-ports',{a:match.male,b:match.female,type:'revolute',move,poses});
      await acceptPlacement(result,revision,recordUndo);
      toast('Hinge joined at its bolt holes.');status('Hinge joint created');
    }catch(e){restorePlacement();toast(e.message,true);}
    return;
  }
  if(draftRun(state.doc,matches[0]?.member)){
    try{
      if(Object.keys(poses).length){
        const delta=alignmentDelta(state.scene,currentMatrices(),matches[0],'connector');
        poses=Object.fromEntries(Object.entries(poses).map(([id,pose])=>{
          const object=new THREE.Object3D();setPose(object,pose);
          delta.clone().multiply(object.matrix).decompose(object.position,object.quaternion,object.scale);
          return [id,getPose(object)];
        }));
        const moved=await api('move',{poses});await acceptPlacement(moved,revision,recordUndo);recordUndo=false;
      }
      else{
        if(recordUndo)checkpoint();
        if(movingId===matches[0].connector){
          const id=matches[0].connector,object=partObjects.get(id),target=new THREE.Object3D();
          alignmentDelta(state.scene,currentMatrices(),matches[0],'connector').multiply(object.matrix).decompose(target.position,target.quaternion,target.scale);
          const pose=getPose(target);state.doc.parts.find(p=>p.id===id).pose=pose;
          state.scene.parts.find(p=>p.id===id).pose=pose;setPose(object,pose);
        }else positionDraftRun(draftRun(state.doc,matches[0].member),snappedDraftMatrix(matches[0]));
        changed();refreshDraft(matches[0].member);recordUndo=false;
      }
      draftConnect(matches[0],recordUndo);
    }catch(e){toast(e.message,true);}finally{clearSnapPreview();state.placementPending=false;attachGizmo();}
    return;
  }
  state.placementPending=true;gizmo?.detach();status('Checking connection and movement…');
  try{
    const data=await api('snap-options',{...matches[0],poses});
    if(revision!==state.revision){restorePlacement();return;}
    const choice=data.options.find(o=>o.move===data.recommended);
    if(clearConnectionIntent(matches)&&choice?.available&&!choice.requires_preview){await acceptPlacement(choice,revision,recordUndo);toast('Connected '+matches[0].connector+' to '+matches[0].member+'.');status('Connection created');return;}
    connectionReview(matches,poses,revision,data,null,recordUndo);
  }catch(e){restorePlacement();toast(e.message,true);}
}
function connectionReview(matches,poses={},revision=state.revision,initial=null,replaceJoint=null,recordUndo=true){
  state.placementPending=true;gizmo?.detach();let result=initial,chosen=null,requestVersion=0,previewRenderer=null,previewControls=null,previewObjects=null;
  const session=++reviewVersion;
  modal(replaceJoint?'Edit socket connection':'Align and connect',`<div class="single-field"><label for="snap-target">CONNECTION</label><select id="snap-target">${matches.map((m,i)=>`<option value="${i}">${esc(m.label||m.connector+' / '+m.port)}</option>`).join('')}</select></div><div id="snap-settings"></div><div class="single-field"><label for="snap-move">PARTS TO MOVE</label><select id="snap-move" disabled></select></div><label class="check-label"><input id="snap-lock" type="checkbox" ${preferences.connectionLocked?'checked':''}>Secure the screw</label><div id="snap-preview-view" aria-label="Connection alignment preview"></div><p id="snap-review-status" role="status">Checking available movement…</p>`,[
    {label:'Cancel',action:()=>{closeModal();restorePlacement();}},
    ...(Object.keys(poses).length?[{label:'Place only',action:async()=>{closeModal();await placeWithoutConnection(poses,revision,recordUndo);}}]:[]),
    {label:'Force',action:()=>refresh(true)},
    {label:'Connect',primary:true,action:async()=>{if(!chosen?.available||revision!==state.revision)return;const option=chosen;closeModal();await acceptPlacement(option,revision,recordUndo);toast('Connection created.');status('Connection created');}}
  ]);
  $('#modal').classList.add('connection-modal');$('#modal-actions .primary').disabled=true;
  const forceButton=$$('#modal-actions button').find(b=>b.textContent==='Force');forceButton.id='snap-force';
  forceButton.title='Search for a fit using the permitted screw releases and cut-length adjustments. Review the changes before connecting.';
  const forceHelp=document.createElement('p');forceHelp.id='snap-force-help';forceHelp.textContent='Force searches more hinge rotations and slides, and can adjust socket position to find a fit. Review the preview, then Connect.';
  $('#snap-review-status').after(forceHelp);forceButton.setAttribute('aria-describedby','snap-force-help');
  const fitControls=document.createElement('div');fitControls.innerHTML=`<div class="single-field"><label for="snap-tolerance">CONNECTION TOLERANCE · mm</label><input id="snap-tolerance" type="number" min="0.01" max="20" step="any" value="${initial?.joint?.fit_tolerance_mm??preferences.connectionToleranceMm}"></div>
    <details id="snap-force-options"><summary>Force options</summary><p>Allow a bounded adjustment of the build. Released screws are tightened again after fitting. World anchors stay fixed.</p>
    <div class="single-field"><label for="snap-unlock-count">LOOSEN & RETIGHTEN UP TO · connectors</label><input id="snap-unlock-count" type="number" min="0" max="6" step="1" value="0"></div>
    <div class="fields"><div class="field"><label for="snap-resize-count">RESIZE UP TO · pipes</label><input id="snap-resize-count" type="number" min="0" max="4" step="1" value="0"></div>
    <div class="field"><label for="snap-resize-mm">MAX LENGTH CHANGE · mm each</label><input id="snap-resize-mm" type="number" min="0.01" max="1000" step="any" value="20"></div></div>
    <p>Pipe lengths may increase or decrease. The preview lists the proposed cut lengths; Connect applies them as one undoable edit.</p></details><div id="snap-adjustments" aria-live="polite"></div>`;
  $('#snap-preview-view').before(fitControls);
  fitControls.querySelectorAll('input').forEach(field=>{field.required=true;field.onchange=()=>refresh(false);});
  function drawPreview(){
    clearSnapPreview();if(previewObjects)dispose(previewObjects);
    if(chosen?.available)showPosePreview(chosen.poses,chosen.preview_parts);
    if(!renderer)return;
    try{
      const match=matches[+$('#snap-target').value],host=$('#snap-preview-view');
      if(!previewRenderer){previewRenderer=new THREE.WebGLRenderer({antialias:true});previewRenderer.setPixelRatio(Math.min(devicePixelRatio,2));previewRenderer.setSize(320,220);host.appendChild(previewRenderer.domElement);}
      const view=new THREE.Scene();view.background=scene.background.clone();view.add(new THREE.HemisphereLight('#ffffff','#70818a',3));const light=new THREE.DirectionalLight('#ffffff',3);light.position.set(1,-2,3);view.add(light);
      previewObjects=new THREE.Group();view.add(previewObjects);
      for(const id of new Set([match.member,match.connector,...(chosen?.moved||[]),...(chosen?.context_parts||[]),...(chosen?.preview_parts||[]).map(p=>p.id)])){const p=state.scene.parts.find(p=>p.id===id);const copy=previewCopy(id,chosen?.poses[id]||p.pose,false,chosen?.preview_parts?.find(p=>p.id===id));if(copy)previewObjects.add(copy);}
      const box=new THREE.Box3().setFromObject(previewObjects),center=box.getCenter(new THREE.Vector3()),radius=Math.max(box.getSize(new THREE.Vector3()).length()/2,70);
      const cam=new THREE.PerspectiveCamera(38,320/220,1,100000);cam.up.set(0,0,1);cam.position.copy(center).addScaledVector(camera.position.clone().sub(orbit.target).normalize(),radius/Math.sin(THREE.MathUtils.degToRad(19))*1.15);cam.lookAt(center);
      previewControls?.dispose();previewControls=new OrbitControls(cam,previewRenderer.domElement);previewControls.target.copy(center);previewControls.addEventListener('change',()=>previewRenderer?.render(view,cam));previewControls.update();previewRenderer.render(view,cam);
    }catch(e){$('#snap-preview-view').textContent='The green outline in the main view shows this alignment.';}
  }
  function showResult(data){
    result=data;const previous=$('#snap-move').value;
    forceButton.disabled=false;
    $('#snap-move').innerHTML=data.options.map(o=>`<option value="${o.move}" ${o.available?'':'disabled'}>${esc(o.label)}${o.available?'':' — '+esc(o.reason)}</option>`).join('');
    $('#snap-move').value=data.options.some(o=>o.move===previous&&o.available)?previous:data.recommended||'';$('#snap-move').disabled=!data.recommended;
    chooseMovement();
  }
  function chooseMovement(){
    chosen=result?.options.find(o=>o.move===$('#snap-move').value&&o.available);
    $('#modal-actions .primary').disabled=!chosen;
    const changes=chosen?.adjustments;
    $('#snap-adjustments').innerHTML=changes&&(changes.unlocked.length||changes.resized.length)?`<h3>PROPOSED ASSEMBLY CHANGES</h3><ul>${changes.unlocked.map(c=>`<li>Loosen, fit and retighten <strong>${esc(c.connector)}</strong> / ${esc(c.port||c.joint)}</li>`).join('')}${changes.resized.map(c=>`<li><strong>${esc(c.part)}</strong>: ${c.before_mm.toFixed(2)} → ${c.after_mm.toFixed(2)} mm (${c.change_mm>0?'+':''}${c.change_mm.toFixed(2)} mm)</li>`).join('')}</ul>`:'';
    if(chosen?.joint){
      if($('#snap-station'))$('#snap-station').value=Number(chosen.joint.b.at_mm.toFixed(4));
      if($('#snap-insertion'))$('#snap-insertion').value=Number(chosen.joint.insertion_mm.toFixed(4));
    }
    $('#snap-review-status').textContent=chosen?`${chosen.label}. ${chosen.description||'Alignment rotation: '+chosen.rotation_deg.toFixed(1)+'°.'} Drag the preview to inspect.`:(result?.options.find(o=>o.move==='fit')?.reason||result?.options.map(o=>o.reason).filter((v,i,a)=>a.indexOf(v)===i).join(' · ')||'No alignment found yet. Use Force to search more joint motion.');
    drawPreview();
  }
  async function refresh(force=false){
    const request=++requestVersion;chosen=null;$('#modal-actions .primary').disabled=true;forceButton.disabled=true;clearSnapPreview();$('#snap-adjustments').innerHTML='';
    const match=matches[+$('#snap-target').value];const socket=state.scene.parts.find(p=>p.id===match.connector).ports[match.port];
    const field=$('#snap-station')||$('#snap-insertion');
    if(field&&!field.checkValidity()){field.reportValidity();forceButton.disabled=false;$('#snap-review-status').textContent=socket.through?'Keep the full socket on the pipe.':'Insertion is the depth inside an end socket.';return;}
    for(const input of fitControls.querySelectorAll('input'))if(!input.checkValidity()){input.reportValidity();forceButton.disabled=false;$('#snap-review-status').textContent='Choose valid fit limits before searching.';return;}
    $('#snap-review-status').textContent=force?'Searching hinge rotations and slides together… You can cancel while the search runs.':'Checking available movement…';
    try{
      const data=await api('snap-options',{...match,poses,force,replace_joint:replaceJoint,locked:$('#snap-lock').checked,
        tolerance_mm:+$('#snap-tolerance').value,force_options:{unlock_connectors:+$('#snap-unlock-count').value,resize_members:+$('#snap-resize-count').value,max_length_change_mm:+$('#snap-resize-mm').value},
        ...(socket.through?{at_mm:+$('#snap-station').value}:{end:$('#snap-end').value,insertion_mm:+$('#snap-insertion').value})});
      if(request!==requestVersion||session!==reviewVersion||revision!==state.revision)return;
      showResult(data);
    }catch(e){if(request===requestVersion&&session===reviewVersion){$('#snap-review-status').textContent=e.message;$('#snap-move').disabled=true;forceButton.disabled=false;}}
  }
  function settings(){
    const match=matches[+$('#snap-target').value],fitting=state.scene.parts.find(p=>p.id===match.connector),member=state.scene.parts.find(p=>p.id===match.member),socket=fitting.ports[match.port];
    $('#snap-settings').innerHTML=socket.through?`<div class="single-field"><label for="snap-station">SOCKET CENTRE FROM PIPE START · mm</label><input id="snap-station" type="number" min="${socket.engagement_mm/2}" max="${member.length_mm-socket.engagement_mm/2}" step="any" value="${match.at_mm??member.length_mm/2}"></div><p>${Number((member.length_mm/2).toFixed(4))} mm centres this fitting on a ${Number(member.length_mm.toFixed(4))} mm pipe. The pipe keeps its cut length.</p>`:`<div class="single-field"><label for="snap-end">PIPE END</label><select id="snap-end"><option value="start">Start</option><option value="end">End</option></select></div><div class="single-field"><label for="snap-insertion">DEPTH INSIDE SOCKET · mm</label><input id="snap-insertion" type="number" min="${socket.min_engagement_mm}" max="${socket.engagement_mm}" step="any" value="${match.insertion_mm??Math.min(30,socket.engagement_mm*.8)}"></div>`;
    if($('#snap-end'))$('#snap-end').value=match.end||'start';
    $$('#snap-settings input, #snap-settings select').forEach(e=>e.onchange=()=>refresh());
  }
  reviewCleanup=()=>{reviewVersion++;requestVersion++;previewControls?.dispose();if(previewObjects)dispose(previewObjects);previewRenderer?.dispose();previewRenderer=null;$('#modal').classList.remove('connection-modal');restorePlacement();};
  $('#snap-target').onchange=()=>{settings();refresh();};$('#snap-move').onchange=chooseMovement;$('#snap-lock').onchange=()=>refresh();
  const old=state.doc.joints?.find(j=>j.id===replaceJoint);if(old){$('#snap-lock').checked=!!old.locked;$('#snap-tolerance').value=old.fit_tolerance_mm??preferences.connectionToleranceMm;}
  settings();if(initial)showResult(initial);else refresh();
}

function propertyFields(label,values,key){return `<h3>${label}</h3><div class="fields">${['X','Y','Z'].map((axis,i)=>`<div class="field"><label>${axis} ${key==='rotation_deg'?'°':'mm'}</label><input type="number" data-pose="${key}" data-axis="${i}" value="${Number(values[i]||0).toFixed(1)}" step="${key==='rotation_deg'?state.snapSettings.rotationDeg:state.snapSettings.translationMm}" aria-label="${label} ${axis}"></div>`).join('')}</div>${key==='position_mm'?`<button class="inspect-action secondary" data-drop-floor="${label==='OBJECT POSITION'?'object':'part'}" title="Place the lowest point on Z = 0, lifting from below the floor if needed">Drop to floor</button>`:''}`;}
function bindFloorButtons(selected,objectId=null){
  $$('[data-drop-floor]').forEach(button=>{
    button.disabled=state.busy||state.placementPending;
    button.onclick=()=>dropSelectionToFloor(selected,button.dataset.dropFloor==='object'?objectId:null);
  });
}
async function dropSelectionToFloor(selected,objectId){
  if(state.busy||state.placementPending)return;
  const revision=state.revision;state.placementPending=true;gizmo?.detach();
  $$('[data-drop-floor]').forEach(button=>button.disabled=true);
  try{
    const result=await api('drop-to-floor',{selected,object:objectId});
    if(revision!==state.revision)return;
    if(result.moved.length)await acceptPlacement(result,revision);
    status(result.message);toast(result.message);
  }catch(error){if(revision===state.revision)toast(error.message,true);}
  finally{restorePlacement();renderInspector();}
}
async function editPose(part,input){
  const key=input.dataset.partPose||input.dataset.pose,target=clone(part.pose);
  target[key][+input.dataset.axis]=Number(input.value);
  await moveTarget(part.id,target,key==='rotation_deg'?'rotate':'translate');
}
async function moveTarget(selected,target,mode,objectId=null){
  if(state.placementPending)return;const revision=state.revision;state.placementPending=true;gizmo?.detach();
  try{
    const result=await api(objectId?'move-object':'move',{object:objectId,selected,target,mode});
    if(revision!==state.revision){restorePlacement();return;}
    if(result.moved.length)await acceptPlacement(result,revision);else{restorePlacement();renderInspector();}
    status(result.message);
  }catch(e){restorePlacement();renderInspector();toast(e.message,true);}
}
const keyboardAxes={a:[0,-1],d:[0,1],s:[1,-1],w:[1,1],v:[2,-1],f:[2,1]};
const keyboardRotations={z:0,x:1,c:2};
const keyboardQueue=[];let keyboardNudging=false;
async function applyKeyboardNudge(command){
  const part=state.scene.parts.find(p=>p.id===command.selected);if(!part)return;
  const axis=new THREE.Vector3().setComponent(command.axis,1),rotation=command.mode==='rotate';
  const draft=draftRun(state.doc,command.selected);
  if(draft){
    if(draft.attachments?.length){
      if(rotation){toast('Rotate a connector to change an attached draft run’s direction.');return;}
      const group=draftTranslationGroup(draft.id);if(!group)return;
      translateDraftMembers(group,axis.multiplyScalar(command.amount));status('Connected draft structure nudged');return;
    }
    const matrix=partObjects.get(draft.id).matrix.clone(),delta=new THREE.Matrix4();
    if(rotation){const center=new THREE.Vector3(...part.pose.position_mm);
      delta.makeTranslation(...center.toArray()).multiply(new THREE.Matrix4().makeRotationAxis(axis,THREE.MathUtils.degToRad(command.amount)))
        .multiply(new THREE.Matrix4().makeTranslation(...center.clone().negate().toArray()));
    }else delta.makeTranslation(...axis.multiplyScalar(command.amount).toArray());
    checkpoint();positionDraftRun(draft,delta.multiply(matrix));changed();refreshDraft(draft.id);status('Draft run nudged');return;
  }
  const instance=wholeObject(),pose=clone(instance?.pose||part.pose||{});
  pose.position_mm=[...(pose.position_mm||[0,0,0])];pose.rotation_deg=[...(pose.rotation_deg||[0,0,0])];
  if(rotation){const object=new THREE.Object3D();setPose(object,pose);
    object.quaternion.premultiply(new THREE.Quaternion().setFromAxisAngle(axis,THREE.MathUtils.degToRad(command.amount)));
    pose.rotation_deg=getPose(object).rotation_deg;
  }else pose.position_mm[command.axis]=+(pose.position_mm[command.axis]+command.amount).toFixed(5);
  await moveTarget(command.selected,pose,command.mode,instance?.id||null);
}
async function flushKeyboardNudges(){
  if(keyboardNudging)return;keyboardNudging=true;
  try{while(keyboardQueue.length){const command=keyboardQueue.shift();
    if(state.selected!==command.selected||state.mode!=='design'||state.busy||$('#modal').open)continue;
    if(command.action==='duplicate')await duplicateSelected();else await applyKeyboardNudge(command);
  }}finally{keyboardNudging=false;}
}
function queueKeyboardNudge(event){
  if(event.ctrlKey||event.metaKey||!state.selected||state.mode!=='design'||state.busy||dragStart||state.placementPending&&!keyboardNudging)return false;
  const key=event.key.toLowerCase(),move=keyboardAxes[key],rotate=keyboardRotations[key];
  if(!move&&rotate===undefined)return false;
  if(move&&event.altKey)return false;
  const mode=move?'translate':'rotate',axis=move?move[0]:rotate;
  const step=mode==='translate'?state.snapSettings.keyboardMoveMm:state.snapSettings.keyboardRotateDeg;
  const amount=step*(event.shiftKey?0.1:1)*(move?move[1]:event.altKey?-1:1);
  const last=keyboardQueue.at(-1);
  if(last&&last.selected===state.selected&&last.mode===mode&&last.axis===axis&&Math.sign(last.amount)===Math.sign(amount))last.amount+=amount;
  else keyboardQueue.push({selected:state.selected,mode,axis,amount});
  flushKeyboardNudges();return true;
}
async function objectEdit(route,extra){
  if(state.busy||state.placementPending)return;const revision=state.revision;state.placementPending=true;gizmo?.detach();
  try{const result=await api(route,extra);await acceptPlacement(result,revision);}
  catch(error){restorePlacement();renderInspector();toast(error.message,true);}
}
async function regroupObject(id){
  objectModes.set(id,'whole');await objectEdit('regroup',{object:id});
  if(state.doc.objects?.some(o=>o.id===id)){select(state.selected);toast('Regrouped '+id+'. Its parts and joints remain articulated.');}
}
function bindDuplicateButton(){
  const split=document.createElement('div');split.className='duplicate-control';
  split.innerHTML=`<div class="duplicate-buttons">
    <button id="duplicate-selected" class="inspect-action secondary" title="Make one copy of just the selected part · Alt+Shift+D">Duplicate</button>
    <button id="duplicate-menu-toggle" class="inspect-action secondary" aria-label="Duplicate options" aria-haspopup="menu" aria-controls="duplicate-menu" aria-expanded="false" title="More duplicate options">▾</button></div>
    <div id="duplicate-menu" class="duplicate-menu" role="menu" aria-label="Duplicate options" hidden>
      <button role="menuitem" data-duplicate="count" tabindex="-1">Duplicate N copies…<small>Choose a count and what to include</small></button>
      <button role="menuitem" data-duplicate="touching" tabindex="-1">Duplicate directly touching parts<small>Selected part and its immediate connections</small></button>
      <button role="menuitem" data-duplicate="subassembly" tabindex="-1">Duplicate entire subassembly<small>Locked connections, or the whole grouped object</small></button>
    </div>`;
  if(draftRun(state.doc,state.selected)){
    split.querySelector('[data-duplicate="touching"]').remove();
    const whole=split.querySelector('[data-duplicate="subassembly"]');
    whole.innerHTML='Duplicate entire subassembly<small>Connected draft pipes, fittings, and internal joints</small>';
  }
  $('#inspector .inspect-section').appendChild(split);
  split.querySelectorAll('button').forEach(b=>b.disabled=state.busy||state.placementPending);
  $('#duplicate-selected').onclick=()=>duplicateSelected();
  const toggle=$('#duplicate-menu-toggle'),menu=$('#duplicate-menu'),items=[...menu.querySelectorAll('button')];
  const open=(index=0)=>{menu.hidden=false;toggle.setAttribute('aria-expanded','true');items[index].focus();};
  toggle.onclick=()=>menu.hidden?open():closeDuplicateMenu(true);
  toggle.onkeydown=e=>{if(['ArrowDown','ArrowUp'].includes(e.key)){e.preventDefault();e.stopPropagation();open(e.key==='ArrowUp'?items.length-1:0);}};
  menu.onkeydown=e=>{
    if(e.key==='Escape'){e.preventDefault();e.stopPropagation();closeDuplicateMenu(true);return;}
    if(['ArrowDown','ArrowUp','Home','End'].includes(e.key)){
      e.preventDefault();e.stopPropagation();const i=items.indexOf(document.activeElement);
      items[e.key==='Home'?0:e.key==='End'?items.length-1:(i+(e.key==='ArrowDown'?1:-1)+items.length)%items.length].focus();
    }else if(e.key!=='Tab')e.stopPropagation();
  };
  for(const button of items)button.onclick=()=>{closeDuplicateMenu(true);if(button.dataset.duplicate==='count')duplicateCountDialog();else duplicateSelected(button.dataset.duplicate);};
}
function closeDuplicateMenu(focus=false){
  const menu=$('#duplicate-menu');if(!menu||menu.hidden)return;
  menu.hidden=true;$('#duplicate-menu-toggle').setAttribute('aria-expanded','false');if(focus)$('#duplicate-menu-toggle').focus();
}
document.addEventListener('pointerdown',e=>{if(!e.target.closest('.duplicate-control'))closeDuplicateMenu();});
document.addEventListener('focusin',e=>{if(!e.target.closest('.duplicate-control'))closeDuplicateMenu();});
function draftDuplicateMembers(selected){
  const adjacent=new Map(),link=(a,b)=>{if(!adjacent.has(a))adjacent.set(a,new Set());adjacent.get(a).add(b);};
  for(const run of draftRuns(state.doc))for(const attachment of run.attachments||[]){link(run.id,attachment.connector);link(attachment.connector,run.id);}
  for(const joint of state.scene.joints)if(joint.locked??joint.type==='fixed'){
    link(joint.a.part,joint.b.part);link(joint.b.part,joint.a.part);
  }
  const members=new Set([selected]),pending=[selected];
  while(pending.length)for(const neighbor of adjacent.get(pending.pop())||[])if(!members.has(neighbor)){
    members.add(neighbor);pending.push(neighbor);
  }
  return [...members];
}
function duplicateScopeInfo(scope){
  const selected=state.selected;
  if(scope==='part')return {count:1,description:'Just '+selected+'.'};
  if(draftRun(state.doc,selected)&&scope==='subassembly')return {count:draftDuplicateMembers(selected).length,
    description:'Connected draft pipes and fittings, including fixed joints. World fixings and outside connections stay with the original.'};
  if(scope==='touching'){
    const members=new Set([selected]);for(const joint of state.scene.joints)if([joint.a.part,joint.b.part].includes(selected)){members.add(joint.a.part);members.add(joint.b.part);}
    return {count:members.size,description:'The selected part and every part connected directly to it, including loose joints. Neighbours of those parts are excluded.'};
  }
  const instance=!state.doc.parts.some(p=>p.id===selected)&&selectedObject();
  if(instance)return {count:state.scene.parts.filter(p=>p.id.startsWith(instance.id+'/')&&!state.doc.parts.some(s=>s.id===p.id)).length,description:'The whole '+instance.id+' object, including its articulated joints.'};
  return {count:(state.scene.groups.find(g=>g.includes(selected))||[selected]).length,description:'Parts joined by locked connections. Loose and articulated joints mark the boundary.'};
}
function duplicatePreviewMembers(scope){
  const selected=state.selected;
  if(scope==='part')return [selected];
  if(draftRun(state.doc,selected))return draftDuplicateMembers(selected);
  if(scope==='touching'){
    const members=new Set([selected]);
    for(const joint of state.scene.joints)if([joint.a.part,joint.b.part].includes(selected)){
      members.add(joint.a.part);members.add(joint.b.part);
    }
    return [...members];
  }
  const instance=!state.doc.parts.some(p=>p.id===selected)&&selectedObject();
  if(instance)return state.scene.parts.filter(p=>p.id.startsWith(instance.id+'/')&&!state.doc.parts.some(s=>s.id===p.id)).map(p=>p.id);
  return state.scene.groups.find(g=>g.includes(selected))||[selected];
}
function duplicatePreviewBounds(members){
  const bounds=new THREE.Box3();
  for(const id of members){const object=partObjects.get(id);if(object)bounds.union(new THREE.Box3().setFromObject(object));}
  return bounds;
}
function defaultDuplicateStep(scope){
  const bounds=duplicatePreviewBounds(duplicatePreviewMembers(scope));
  const grid=state.snap?state.snapSettings.translationMm:1;
  return Math.ceil(Math.max(100,bounds.max.x-bounds.min.x+50)/grid)*grid;
}
function drawDuplicatePreview(members,count,step){
  clearSnapPreview();
  const total=members.length*count;
  if(total<=60){
    for(let copyIndex=1;copyIndex<=count;copyIndex++)for(const id of members){
      const ghost=previewCopy(id,null,true);if(!ghost)continue;
      ghost.position.addScaledVector(step,copyIndex);ghost.updateMatrix();snapGhost.add(ghost);
    }
  }else{
    const bounds=duplicatePreviewBounds(members);
    for(let copyIndex=1;copyIndex<=count;copyIndex++){
      const shift=step.clone().multiplyScalar(copyIndex);
      snapGhost.add(new THREE.Box3Helper(bounds.clone().translate(shift),0x48c6a0));
    }
  }
  $('#duplicate-preview-note').textContent=total<=60?`Previewing ${count} ${count===1?'copy':'copies'} in the scene.`:
    `Previewing ${count} copy bounds in the scene (${total} parts).`;
}
function duplicateCountDialog(){
  if(state.busy||state.placementPending)return;
  const draft=!!draftRun(state.doc,state.selected);
  let cancelled=false,offsetEdited=false;
  modal('Duplicate N copies',`<form id="duplicate-form">
    <div class="single-field"><label for="duplicate-count">Number of new copies</label><input id="duplicate-count" type="number" min="1" max="100" step="1" value="${preferences.defaultDuplicateCount}" required aria-describedby="duplicate-count-help"></div>
    <p id="duplicate-count-help">Enter a whole number from 1 to 100.</p>
    <div class="single-field"><label for="duplicate-scope">Include in each copy</label><select id="duplicate-scope"><option value="part">Selected part only</option><option value="touching">Part and directly touching parts</option><option value="subassembly">Entire subassembly</option></select></div>
    <div class="fields">${['X','Y','Z'].map((axis,i)=>`<div class="field"><label for="duplicate-offset-${i}">STEP ${axis} · mm</label><input id="duplicate-offset-${i}" type="number" step="any" required value="${i===0?defaultDuplicateStep('part'):0}"></div>`).join('')}</div>
    <p>Each copy moves by this X/Y/Z step from the preceding one. Negative values reverse an axis; zero leaves it unchanged.</p>
    <p id="duplicate-scope-help"></p><p id="duplicate-summary" role="status"></p>
    <p id="duplicate-preview-note" role="status"></p>
    <p>${draft?'Individual draft pipes start without socket connections. Draft subassembly copies keep their internal connections.':'Copies keep their internal connections. World fixings and connections to other parts stay with the original.'}</p>
    </form>`,[{label:'Cancel',action:closeModal},{label:'Duplicate',primary:true,action:async()=>{
      const inputs=['#duplicate-count','#duplicate-offset-0','#duplicate-offset-1','#duplicate-offset-2'].map(selector=>$(selector));
      if(inputs.some(input=>!input.reportValidity()))return;
      const step=[0,1,2].map(axis=>Number($('#duplicate-offset-'+axis).value));
      if(!step.some(value=>value!==0)){toast('Choose a nonzero X, Y or Z offset.',true);return;}
      if(await duplicateSelected($('#duplicate-scope').value,Number($('#duplicate-count').value),()=>cancelled,step))closeModal();
    }}]);
  reviewCleanup=()=>{cancelled=true;clearSnapPreview();};
  if(draft)$('#duplicate-scope option[value="touching"]').remove();
  const update=()=>{const scope=$('#duplicate-scope').value,info=duplicateScopeInfo(scope),input=$('#duplicate-count');
    $('#duplicate-scope-help').textContent=info.description;
    const fields=[0,1,2].map(axis=>$('#duplicate-offset-'+axis)),step=fields.map(field=>Number(field.value));
    const valid=input.checkValidity()&&fields.every(field=>field.checkValidity())&&step.some(value=>value!==0);
    $('#duplicate-summary').textContent=valid?`${input.value} new ${Number(input.value)===1?'copy':'copies'} × ${info.count} ${info.count===1?'part':'parts'} = ${Number(input.value)*info.count} new parts. One Undo removes them all.`:'';
    if(valid)drawDuplicatePreview(duplicatePreviewMembers(scope),Number(input.value),new THREE.Vector3(...step));
    else{clearSnapPreview();$('#duplicate-preview-note').textContent='Enter a count and a nonzero offset to preview the copies.';}
  };
  $('#duplicate-count').oninput=update;
  $('#duplicate-scope').onchange=()=>{if(!offsetEdited)$('#duplicate-offset-0').value=defaultDuplicateStep($('#duplicate-scope').value);update();};
  for(const axis of [0,1,2])$('#duplicate-offset-'+axis).oninput=()=>{offsetEdited=true;update();};
  $('#duplicate-form').onsubmit=e=>{e.preventDefault();$('#modal-actions .primary').click();};update();$('#duplicate-count').focus();$('#duplicate-count').select();
}
async function resizePipe(member,length,releases=[]){
  if(state.busy||state.placementPending)return;
  if(!releases.length&&state.scene.parts.find(p=>p.id===member)?.length_mm===length){renderInspector();return;}
  const revision=state.revision;state.placementPending=true;gizmo?.detach();
  $$('[data-param]').forEach(input=>input.disabled=true);status('Checking connected parts and socket travel…');
  try{
    const result=await api('resize',{member,length_mm:length,releases});
    if(revision!==state.revision)return;
    if(result.status==='resized'){
      await acceptPlacement(result,revision);state.selected=member;select(member);
      status('Length updated · '+result.moved.length+' connected parts moved');
    }else{
      state.placementPending=false;renderInspector();
      modal('Length change blocked',`<p>${esc(result.reason)}</p><p>${esc(member)} remains at its current length. Open a connection or adjust the frame before retrying.</p>
        ${(result.suggestions||[]).map((s,i)=>`<div class="joint-card"><p>${esc(s.message)}</p><p>${s.joints.map(esc).join('<br>')}</p><button class="inspect-action" data-resize-release="${i}">${s.action==='loosen'?'Loosen and resize':'Disconnect and resize'}</button></div>`).join('')}
        ${result.blockers?.length?`<details ${result.suggestions?.length?'':'open'}><summary>View ${result.blockers.length} blocking constraints</summary>
        ${result.blockers.map((b,i)=>`<div class="joint-card"><div class="joint-card-top">${esc(b.joint||'World anchor: '+b.anchor)}</div><p>${esc(b.parts.join(' ↔ '))}${b.port?' · socket '+esc(b.port):''}</p><button class="inspect-action secondary" data-resize-show="${i}">${b.anchor?'Show anchored part':'Show connection'}</button></div>`).join('')}</details>`:''}`,
        [{label:'Keep current length',action:closeModal}]);
      $$('[data-resize-show]').forEach(button=>button.onclick=()=>{
        const blocker=result.blockers[+button.dataset.resizeShow];closeModal();setTool('select');select(blocker.connector||blocker.anchor);
        const edit=$$('[data-joint-edit]').find(b=>b.dataset.jointEdit===blocker.joint);
        edit?.closest('.joint-card')?.scrollIntoView?.({block:'center'});
        if(blocker.joint)toast('Review '+blocker.joint+' in Connections.');
      });
      $$('[data-resize-release]').forEach(button=>button.onclick=()=>{const choice=result.suggestions[+button.dataset.resizeRelease];closeModal();resizePipe(member,length,choice.releases);});
      status('Length unchanged · review the blocking connections');
    }
  }catch(error){toast(error.message,true);status('Length unchanged');}
  finally{state.placementPending=false;if(!$('#modal').open)renderInspector();attachGizmo();}
}
async function duplicateSelected(scope='part',count=1,cancelled=()=>false,offsetMm=null){
  if(state.busy||state.placementPending||!state.selected)return false;
  closeDuplicateMenu();const selected=state.selected,revision=state.revision;
  if(draftRun(state.doc,selected)&&scope==='part')return duplicateDraft(selected,scope,count,cancelled,offsetMm);
  state.placementPending=true;gizmo?.detach();$$('.duplicate-control button, #duplicate-form input, #duplicate-form select').forEach(b=>b.disabled=true);
  try{
    const result=await api('duplicate',{selected,scope,count,grid_mm:state.snap?state.snapSettings.translationMm:1,
      ...(offsetMm?{offset_mm:offsetMm}:{})});
    if(cancelled()||revision!==state.revision)return false;
    state.selected=result.selected;await acceptPlacement(result,revision);
    setTool('translate');if(preferences.fitAfterDuplicate)fitView();toast(`Created ${count} ${count===1?'copy':'copies'} · ${result.parts_per_copy*count} new ${result.parts_per_copy*count===1?'part':'parts'}.`);return true;
  }catch(error){if(!cancelled()&&revision===state.revision){state.selected=selected;toast(error.message,true);}return false;}
  finally{state.placementPending=false;select(state.selected);$$('#duplicate-form input, #duplicate-form select').forEach(b=>b.disabled=false);}
}
function duplicateDraft(selected,scope,count,cancelled,offsetMm=null){
  if(scope!=='part'){toast('Duplicate a draft pipe by itself; socket connections stay with the original.',true);return false;}
  if(!Number.isInteger(count)||count<1||count>100){toast('Number of copies must be a whole number from 1 to 100.',true);return false;}
  if(offsetMm&&(!Array.isArray(offsetMm)||offsetMm.length!==3||offsetMm.some(v=>!Number.isFinite(v))||!offsetMm.some(Boolean))){
    toast('Choose a finite, nonzero X, Y or Z offset.',true);return false;
  }
  if(cancelled())return false;
  const source=draftRun(state.doc,selected),group=state.doc.draft_subassemblies.find(g=>g.runs.includes(source));
  const part=state.scene.parts.find(p=>p.id===selected),matrix=partObjects.get(selected).matrix;
  const start=new THREE.Vector3(0,0,-part.length_mm/2).applyMatrix4(matrix),end=new THREE.Vector3(0,0,part.length_mm/2).applyMatrix4(matrix);
  const width=new THREE.Box3().setFromObject(partObjects.get(selected)).getSize(new THREE.Vector3()).x;
  const grid=state.snap?state.snapSettings.translationMm:1,spacing=Math.ceil(Math.max(100,width+50)/grid)*grid;
  const step=new THREE.Vector3(...(offsetMm||[spacing,0,0]));
  const used=new Set([...state.scene.parts.map(p=>p.id),...(state.doc.objects||[]).map(o=>o.id),...state.doc.draft_subassemblies.map(g=>g.id)]);
  const stem=selected.replace(/-copy(?:-\d+)?$/,'')+'-copy';let next=1;
  const coordinate=v=>{const rounded=+v.toFixed(5);return rounded===0?0:rounded;};
  const copies=[];checkpoint();
  for(let i=0;i<count;i++){
    let id=stem;if(used.has(id))do{id=stem+'-'+(++next);}while(used.has(id));
    used.add(id);const offset=step.clone().multiplyScalar(i+1),copy=clone(source);
    copy.id=id;copy.start_mm=start.clone().add(offset).toArray().map(coordinate);
    copy.end_mm=end.clone().add(offset).toArray().map(coordinate);copy.attachments=[];
    group.runs.push(copy);copies.push(id);
  }
  changed();for(const id of copies)refreshDraft(id);
  setTool('translate');select(copies.at(-1));updateHeader();if(preferences.fitAfterDuplicate)fitView();
  toast(`Created ${count} draft ${count===1?'copy':'copies'}. Socket connections stay with the original.`);
  return true;
}
function decorateInspectorUnits(){
  if(preferences.massUnit!=='kg'){
    const chip=$('#inspector .chip.gray');
    if(chip&&/^[-\d.]+ kg$/.test(chip.textContent))chip.textContent=displayValue(parseFloat(chip.textContent),'mass',preferences.massUnit)+' '+preferences.massUnit;
    for(const row of $$('#inspector .property-row'))if(row.querySelector('span')?.textContent==='Mass'){
      const value=row.querySelector('strong');
      if(value&&/^[-\d.]+ kg$/.test(value.textContent))value.textContent=displayValue(parseFloat(value.textContent),'mass',preferences.massUnit)+' '+preferences.massUnit;
    }
  }
  const unit=preferences.lengthUnit;
  if(unit==='mm')return;
  const inputs=$$('#inspector [data-pose="position_mm"], #inspector [data-part-pose="position_mm"], #inspector [data-param$="_mm"], #inspector [data-object-param$="_mm"], #draft-length');
  for(const input of inputs){
    input.value=displayValue(Number(input.value),'length',unit);
    if(input.step&&input.step!=='any')input.step=Math.max(.0001,displayValue(Number(input.step),'length',unit));
    if(input.min)input.min=displayValue(Number(input.min),'length',unit);
    const label=input.closest('.field, .single-field')?.querySelector('label');
    if(label){label.textContent=label.textContent.replace(/\bmm\b/i,unit);if(!label.textContent.includes(unit))label.textContent+=' · '+unit;}
    const handler=input.onchange;
    if(handler)input.onchange=event=>{input.value=storedValue(input.value,'length',unit);return handler.call(input,event);};
  }
}
function renderInspector(){renderInspectorContents();decorateInspectorUnits();}
function renderInspectorContents(){
  if(!state.doc)return;$('#inspector-title').textContent=({design:'PROPERTIES',check:'DESIGN CHECKS',simulate:'PHYSICS & MOTION',build:'ASSEMBLY PROCESS'})[state.mode];
  if(hoveredDraftConnection)showDraftConnection(null);
  if(state.mode==='check')return renderChecks();if(state.mode==='simulate')return renderSimulation();if(state.mode==='build')return renderBuild();
  const p=state.scene?.parts.find(p=>p.id===state.selected);if(!p){$('#inspector').innerHTML='<div class="empty-state"><div class="empty-symbol">◇</div><h2>Make something useful.</h2><p>Pick a part to inspect its dimensions, position and connections.</p><p>Add components from the library,<br>or start with an example design.</p></div>';return;}
  if(p.draft){
    const run=draftRun(state.doc,p.id),ends=(run.attachments||[]).filter(a=>a.end),fixed=ends.length===2;
    const owner=state.doc.draft_subassemblies.find(group=>group.runs.includes(run));
    const sceneMirrorOwner=state.doc.draft_subassemblies.find(group=>(group.mirrors||[]).some(plane=>plane.scope==='scene'));
    const mirrorAddOwner=sceneMirrorOwner||owner;
    const mirrorEntries=[...(owner.mirrors||[]).map(plane=>({group:owner,plane})),
      ...state.doc.draft_subassemblies.filter(group=>group!==owner).flatMap(group=>(group.mirrors||[])
        .filter(plane=>plane.scope==='scene').map(plane=>({group,plane})))];
    const centeredAttachment=ends.length>0&&(owner.mirrors||[]).some(plane=>plane.run_modes?.[run.id]==='centered');
    const mirrorPanel=`<div class="inspect-section"><h3>SYMMETRY</h3><p>Mirrors preview the scene while it stays in draft.</p>
      ${mirrorEntries.map(({group,plane},index)=>`<div class="joint-card"><div class="joint-card-top"><strong>${esc(plane.axis.toUpperCase())} = ${esc(plane.offset_mm)} mm${plane.scope==='scene'?' · Scene':''}</strong><button class="subtle" data-mirror-remove="${index}">Turn off…</button></div>
        ${group===owner?`<div class="single-field"><label for="mirror-mode-${esc(plane.id)}">THIS PIPE ON PLANE</label><select id="mirror-mode-${esc(plane.id)}" data-mirror-mode="${esc(plane.id)}"><option value="free" ${!plane.run_modes?.[run.id]||plane.run_modes?.[run.id]==='free'?'selected':''}>Free</option><option value="centered" ${plane.run_modes?.[run.id]==='centered'?'selected':''}>Centered, perpendicular</option><option value="in_plane" ${plane.run_modes?.[run.id]==='in_plane'?'selected':''}>Centreline in plane</option></select></div>`:''}</div>`).join('')}
      <button class="inspect-action secondary" id="draft-mirror-add" ${(mirrorAddOwner.mirrors||[]).length>=3?'disabled':''}>Add mirror plane</button></div>`;
    $('#inspector').innerHTML=`<div class="inspect-section"><div class="inspect-id">${esc(p.catalog)}</div><h2>${esc(p.id)}</h2><span class="chip">DRAFT RUN</span><p>Length follows the connector graph. The blue tube is a provisional preview; orange means a connection needs adjustment.</p></div>
      <div class="inspect-section">${propertyFields('POSITION',p.pose.position_mm,'position_mm')}<p>${run.attachments?.length?'Position moves the connected draft structure together.':'Move the tube by dragging it, using the gizmo, or entering its centre position.'}</p></div>
      <div class="inspect-section"><h3>WORKING SPAN</h3><div class="single-field"><label>Preview length · mm</label><input id="draft-length" type="number" min="1" step="any" value="${p.length_mm.toFixed(2)}" ${fixed?'disabled':''}></div><label class="check-label"><input id="draft-lock" type="checkbox" ${run.locked_length_mm!=null?'checked':''}>Lock cut length at ${p.length_mm.toFixed(1)} mm</label><p>${fixed?'Both ends are connected; move a fitting to change the span.':centeredAttachment?'Changing the span moves the connected fitting equally away from or toward the mirror plane.':'Drag the run or enter a rough working length. Exact cut length is calculated when you finalize.'}</p><button class="inspect-action" id="draft-connect">Connect to a socket</button></div>
      ${mirrorPanel}
      <div class="inspect-section"><h3>CONNECTIONS · ${(run.attachments||[]).length}</h3>${(run.attachments||[]).map((a,i)=>`<div class="joint-card"><strong>${esc(a.connector)} / ${esc(a.port)}</strong><p>${esc(a.end||'through station')}</p><button class="subtle" data-draft-detach="${i}">Detach</button></div>`).join('')||'<p>No sockets yet. Choose Connect, then a connector.</p>'}
      ${p.conflicts.map(c=>`<div class="joint-card"><strong>${esc(c.code)}</strong><p>${esc(c.message)}</p></div>`).join('')}<button class="inspect-action secondary" id="draft-repair-selected">Repair alignment · stay in draft</button><button class="inspect-action" id="draft-finalize-selected">Finalize subassembly</button><p>Only draft pipes connected to this run will be finalized; separate structures stay in draft.</p><button class="inspect-action secondary" id="draft-delete">Delete draft run</button></div>`;
    $('#inspector [data-drop-floor]').remove();
    $('#draft-mirror-add').onclick=()=>addDraftMirrorDialog(mirrorAddOwner,run);
    $$('[data-mirror-remove]').forEach(button=>button.onclick=()=>{const entry=mirrorEntries[Number(button.dataset.mirrorRemove)];removeDraftMirrorDialog(entry.group,entry.plane);});
    $$('[data-mirror-mode]').forEach(select=>select.onchange=async()=>{
      const plane=owner.mirrors.find(item=>item.id===select.dataset.mirrorMode),mode=select.value;
      try{await mutate(()=>{plane.run_modes||={};plane.run_modes[run.id]=mode;});}
      catch{/* mutate already reports the actual validation error and restores the document. */}
    });
    $$('[data-pose="position_mm"]').forEach(input=>input.onchange=()=>{
      const target=Number(input.value),axis=+input.dataset.axis;if(!Number.isFinite(target)){renderInspector();return;}
      const group=draftTranslationGroup(run.id);if(!group){renderInspector();return;}
      const delta=new THREE.Vector3().setComponent(axis,target-p.pose.position_mm[axis]);
      if(translateDraftMembers(group,delta))status('Draft structure moved');else renderInspector();
    });
    $('#draft-length').onchange=e=>resizeDraftSpan(run,p,+e.target.value,owner);
    $('#draft-lock').onchange=e=>{checkpoint();if(e.target.checked)run.locked_length_mm=p.length_mm;else delete run.locked_length_mm;changed();refreshDraft(run.id);};
    $$('[data-draft-detach]').forEach(button=>{
      const attachment=run.attachments[+button.dataset.draftDetach],card=button.parentElement;
      card.classList.add('draft-connection-card');card.tabIndex=0;
      card.setAttribute('aria-label',`Highlight ${attachment.connector} ${attachment.port}`);
      const show=()=>showDraftConnection(attachment.connector,attachment.port),clear=()=>showDraftConnection(null);
      card.addEventListener('mouseenter',show);card.addEventListener('mouseleave',clear);
      card.addEventListener('focusin',show);
      card.addEventListener('focusout',event=>{if(!card.contains(event.relatedTarget))clear();});
      button.onclick=()=>{checkpoint();run.attachments.splice(+button.dataset.draftDetach,1);changed();refreshDraft(run.id);};
    });
    $('#draft-connect').onclick=()=>setTool('connect');
    $('#draft-repair-selected').onclick=()=>repairDrafts(null,run.id);
    $('#draft-finalize-selected').onclick=()=>finalizeDrafts(null,run.id);
    $('#draft-delete').onclick=()=>deleteDraft(run.id);bindDuplicateButton();return;
  }
  const instance=(state.doc.objects||[]).find(o=>p.id.startsWith(o.id+'/'));
  if(instance&&!state.doc.parts.some(s=>s.id===p.id))return renderObjectInspector(instance,p);
  const spec=state.doc.parts.find(x=>x.id===p.id),definition=state.doc.definitions?.[p.catalog]||state.library[p.catalog]||spec?.body||{};
  const params={...definition.parameters,...spec?.parameters};const connections=state.scene.joints.filter(j=>[j.a.part,j.b.part].includes(p.id));const group=state.scene.groups.find(g=>g.includes(p.id))||[];const anchor=(state.doc.anchors||[]).find(a=>a.part===p.id);
  const draftConnections=draftRuns(state.doc).flatMap(run=>(run.attachments||[]).map((attachment,index)=>({run,attachment,index}))).filter(({attachment})=>attachment.connector===p.id);
  $('#inspector').innerHTML=`<div class="inspect-section"><div class="inspect-id">${esc(p.catalog||p.id)}</div><h2>${esc((p.label!==p.id&&p.label)||spec?.label||definition.name||p.id)}</h2><span class="chip">${esc(p.kind.toUpperCase())}</span><span class="chip gray">${p.mass_kg.toFixed(2)} kg</span></div><div class="inspect-section">${propertyFields('POSITION',p.pose.position_mm,'position_mm')}<div style="height:17px"></div>${propertyFields('ROTATION',p.pose.rotation_deg,'rotation_deg')}<label class="check-label"><input type="checkbox" id="move-body" ${state.moveBody?'checked':''}>Move the connected rigid body (${group.length})</label>${!spec?'<button class="inspect-action" id="expand-selected">Expand object to edit its parts</button>':''}</div><div class="inspect-section"><h3>DIMENSIONS & MATERIAL</h3>${Object.entries(params).filter(([,v])=>typeof v==='number').map(([k,v])=>`<div class="single-field"><label>${esc(k==='joint_damping_nms_rad'?'PASSIVE JOINT DAMPING · N·m·s/rad':k.replaceAll('_',' ').toUpperCase())}</label><input type="number" data-param="${esc(k)}" value="${v}" step="1" min="0.01"></div>`).join('')}<div class="property-row"><span>Material</span><strong>${esc(definition.material||'Custom body')}</strong></div><div class="property-row"><span>Mass</span><strong>${p.mass_kg.toFixed(3)} kg</strong></div>${p.kind==='member'?'<button class="inspect-action" id="connect-selected">Connect to a socket ⌘</button>':''}</div><div class="inspect-section">${p.kind==='panel'?'<button class="inspect-action" id="fasten-panel">Fasten board at bolt holes</button>':''}${p.kind==='wheel'&&spec&&!connections.some(j=>j.a.part===p.id&&j.a.port==='axle'||j.b.part===p.id&&j.b.port==='axle')?'<button class="inspect-action" id="mount-wheel">Mount wheel axle</button>':''}<button class="inspect-action secondary" id="new-joint">Add joint or attachment</button><h3>CONNECTIONS <span style="float:right">${connections.length}</span></h3>${connections.map(j=>`<div class="joint-card"><div class="joint-card-top"><span>${esc(j.a.part===p.id?j.b.part:j.a.part)}</span><label><input type="checkbox" data-lock="${esc(j.id)}" ${j.locked||j.type==='fixed'?'checked':''}>Locked</label></div><p>${esc(j.a.port||j.type)} → ${esc(j.b.at_mm!=null?Number(j.b.at_mm).toFixed(1)+' mm from pipe start':j.b.end||j.b.port||j.type)} ${j.insertion_mm?' · '+Number(j.insertion_mm).toFixed(1)+' mm insertion':''}</p><button class="subtle" data-joint-edit="${esc(j.id)}" style="padding:3px 9px 3px 0">Edit joint</button><button class="subtle" data-detach="${esc(j.id)}" style="padding:3px 0">Detach</button></div>`).join('')||'<p>No connections. This part moves independently.</p>'}<label class="check-label"><input id="anchor-check" type="checkbox" ${anchor?'checked':''}>Fixed to the world</label>${anchor?`<div class="single-field"><label>MOUNTING SURFACE</label><select id="anchor-surface">${['floor','wall','ceiling','fixture'].map(s=>`<option ${anchor.surface===s?'selected':''}>${s}</option>`).join('')}</select></div>`:''}</div><div class="inspect-section"><h3>PART REFERENCE</h3><p>${esc(p.source?.geometry_status||'User defined geometry')}</p>${p.source?.assumptions?'<p>'+esc(p.source.assumptions.join('. '))+'</p>':''}${/^https?:\/\//.test(p.source?.url||'')?`<a class="results-link" target="_blank" rel="noreferrer" href="${esc(p.source.url)}">Supplier specifications ↗</a>`:''}<button class="inspect-action secondary" id="edit-part-definition">Edit part definition</button></div>`;
  if(draftConnections.length){
    const section=$('#anchor-check').closest('.inspect-section');
    section.querySelector('h3 span').textContent=connections.length+draftConnections.length;
    if(!connections.length)section.querySelector('h3 + p')?.remove();
    for(const {run,attachment,index} of draftConnections){
      const card=document.createElement('div');card.className='joint-card';
      card.innerHTML=`<div class="joint-card-top"><span>${esc(run.id)}</span><span>DRAFT</span></div><p>${esc(attachment.port)} · ${esc(attachment.end||'through station')}</p><button class="subtle" data-draft-connector-detach="${index}">Detach</button>`;
      $('#anchor-check').parentElement.before(card);
      card.querySelector('button').onclick=()=>{checkpoint();run.attachments.splice(index,1);changed();refreshDraft(run.id);renderInspector();};
    }
  }
  if(p.kind==='panel'&&definition.panel_layers)$('#fasten-panel').textContent='Fasten padded panel at bolt holes';
  bindDuplicateButton();bindFloorButtons(p.id);$('#new-joint').onclick=()=>jointDialog();if($('#fasten-panel'))$('#fasten-panel').onclick=()=>fastenPanelDialog(p);
  if($('#mount-wheel'))$('#mount-wheel').onclick=()=>wheelDialog(p.catalog,null,p.id);
  const candidate=state.scene.regroupable_objects?.find(o=>o.parts.includes(p.id));
  if(candidate){
    const section=document.createElement('div');section.className='inspect-section';
    section.innerHTML=`<h3>${esc(candidate.id)}</h3><p>${candidate.parts.length} editable parts. Regroup to move this ${candidate.template==='human'?'person':'object'} as a whole while keeping its joints and pose.</p><button id="regroup-object" class="inspect-action">Regroup as object</button>`;
    $('#inspector').children[1].before(section);$('#regroup-object').onclick=()=>regroupObject(candidate.id);
  }
  $('#move-body').checked=true;$('#move-body').disabled=true;$('#move-body').parentElement.title='Locked connections stay rigid. Loose joints follow within their available travel.';
  $$('[data-pose]').forEach(input=>{input.disabled=!spec;input.onchange=()=>editPose(p,input);});
  $$('[data-param]').forEach(input=>{input.disabled=!spec;input.onchange=()=>p.kind==='member'&&input.dataset.param==='length_mm'?resizePipe(p.id,Number(input.value)):mutate(()=>{spec.parameters={...params,[input.dataset.param]:Number(input.value)};});});
  $$('[data-lock]').forEach(input=>{const j=state.doc.joints?.find(j=>j.id===input.dataset.lock);input.disabled=!j;input.onchange=()=>mutate(()=>{j.locked=input.checked;if(j.type==='fixed'&&!input.checked)j.type='spherical';});});
  $$('[data-joint-edit]').forEach(b=>b.onclick=()=>jointDialog(b.dataset.jointEdit));
  $$('[data-detach]').forEach(button=>button.onclick=()=>mutate(()=>{state.doc.joints=state.doc.joints.filter(j=>j.id!==button.dataset.detach);}));
  $('#anchor-check').onchange=e=>mutate(()=>{state.doc.anchors=(state.doc.anchors||[]).filter(a=>a.part!==p.id);if(e.target.checked)state.doc.anchors.push({part:p.id,surface:'fixture'});});
  if($('#anchor-surface'))$('#anchor-surface').onchange=e=>mutate(()=>{state.doc.anchors.find(a=>a.part===p.id).surface=e.target.value;});
  if($('#connect-selected'))$('#connect-selected').onclick=()=>setTool('connect');if($('#expand-selected'))$('#expand-selected').onclick=expandObjects;
  $('#edit-part-definition').onclick=()=>libraryEditor(p.catalog,p.catalog?definition:spec.body);
}

function humanPoseOptions(selected='standing'){
  let group='';
  return HUMAN_POSES.map(p=>{
    const start=p.group!==group?`${group?'</optgroup>':''}<optgroup label="${esc(p.group)}">`:'';
    group=p.group;
    return `${start}<option value="${esc(p.id)}" ${p.id===selected?'selected':''}>${esc(p.label)}</option>`;
  }).join('')+'</optgroup>';
}
function humanPoseDescription(id){return HUMAN_POSES.find(p=>p.id===id)?.description||'';}
const HUMAN_POSTURES=[['relaxed','Relaxed'],['upper_body','Hold upper body · free legs'],['arms','Hold arms'],['torso','Hold torso'],['legs','Hold legs'],['all','Hold whole body'],['custom','Choose individual joints']];
function humanPosture(parameters){
  if(parameters.hold_pose)return 'all';
  const held=parameters.hold_joints||[];
  return !held.length?'relaxed':held.length===1&&HUMAN_POSTURES.some(([id])=>id===held[0]&&!['relaxed','all','custom'].includes(id))?held[0]:'custom';
}
function setHumanPosture(parameters,mode,joints=[]){
  parameters.hold_pose=mode==='all';
  if(mode==='all'||mode==='relaxed')delete parameters.hold_joints;
  else parameters.hold_joints=mode==='custom'?joints:[mode];
}
function postureOptions(mode){return HUMAN_POSTURES.map(([id,label])=>`<option value="${id}" ${mode===id?'selected':''}>${label}</option>`).join('');}
function renderObjectAttachments(instance,selectedPart){
  const chain=instance.template==='chain';
  const flexibleLines=chain?[]:(state.doc.objects||[]).filter(o=>o.template==='chain');
  const inside=id=>id.startsWith(instance.id+'/'),connections=state.scene.joints.filter(j=>inside(j.a.part)!==inside(j.b.part));
  const detached=(state.doc.metadata?.detached_attachments||[]).filter(r=>(r.object===instance.id||!chain&&[r.joint.a.part,r.joint.b.part].some(id=>inside(id)))&&state.scene.parts.some(p=>p.id===r.joint.a.part)&&state.scene.parts.some(p=>p.id===r.joint.b.part));
  const anchors=state.scene.anchors.filter(a=>inside(a.part));
  const section=document.createElement('div');section.className='inspect-section';
  section.innerHTML=`<h3>CONNECTIONS TO STRUCTURE</h3><p>Attachments stay connected while posing. Detach grips or mounts to reposition the whole ${chain?'chain':'person'}.</p>
    ${connections.map(j=>`<div class="joint-card"><div class="joint-card-top">${esc(j.id)}</div><p>${esc(j.a.part)} ↔ ${esc(j.b.part)} · ${esc(j.type)}</p><button class="subtle" data-attachment-edit="${esc(j.id)}">Edit connection</button> <button class="subtle" data-attachment-detach="${esc(j.id)}">Detach</button></div>`).join('')}
    ${anchors.map(a=>`<div class="joint-card"><p>${esc(a.part)} · fixed to ${esc(a.surface||'world')}</p><button class="subtle" data-object-unanchor="${esc(a.part)}">Release world anchor</button></div>`).join('')}
    ${!connections.length&&!anchors.length?`<p>No external attachments. The whole ${chain?'chain':'person'} can move freely.</p>`:''}
    ${detached.map(r=>`<div class="joint-card"><p>${esc(r.joint.id)} · detached</p><button class="inspect-action secondary" data-attachment-reconnect="${esc(r.joint.id)}">Preview reconnect</button></div>`).join('')}
    <button class="inspect-action secondary" id="object-attach-part">${chain?'Attach flexible link to a part':'Attach body part to structure'}</button>
    ${flexibleLines.length?`<div class="single-field"><label for="human-flexible-line">FLEXIBLE LINE</label><select id="human-flexible-line">${flexibleLines.map(line=>`<option value="${esc(line.id)}">${esc(line.label||line.id)}</option>`).join('')}</select></div><button class="inspect-action secondary" id="human-attach-flexible">Attach line to this body surface</button>`:''}`;
  $('#object-expand').closest('.inspect-section').before(section);
  $$('[data-attachment-edit]').forEach(b=>b.onclick=()=>jointDialog(b.dataset.attachmentEdit));
  $$('[data-attachment-detach]').forEach(b=>b.onclick=()=>{
    const joint=connections.find(j=>j.id===b.dataset.attachmentDetach);
    const line=(state.doc.objects||[]).find(o=>o.template==='chain'&&[joint.a.part,joint.b.part].some(id=>id.startsWith(o.id+'/')));
    objectEdit('detach-attachment',{object:line?.id||instance.id,joint:joint.id});
  });
  $$('[data-attachment-reconnect]').forEach(b=>b.onclick=()=>{
    const saved=detached.find(r=>r.joint.id===b.dataset.attachmentReconnect);
    const owner=(state.doc.objects||[]).find(o=>o.id===saved.object)||instance;
    attachmentDialog(owner,selectedPart,b.dataset.attachmentReconnect);
  });
  $$('[data-object-unanchor]').forEach(b=>b.onclick=()=>objectEdit('release-object-anchor',{object:instance.id,part:b.dataset.objectUnanchor}));
  $('#object-attach-part').onclick=()=>attachmentDialog(instance,selectedPart);
  if(flexibleLines.length)$('#human-attach-flexible').onclick=()=>{
    const line=flexibleLines.find(o=>o.id===$('#human-flexible-line').value);
    const link=state.scene.parts.find(p=>p.id===line.id+'/link-1');
    attachmentDialog(line,link,null,selectedPart.id);
  };
}
function attachmentDialog(instance,selectedPart,reconnect=null,preferredTarget=null){
  const chain=instance.template==='chain';
  if(state.busy||state.placementPending)return;
  const inside=p=>p.id.startsWith(instance.id+'/'),limbs=state.scene.parts.filter(inside),targets=state.scene.parts.filter(p=>!inside(p));
  if(!targets.length){toast('Add a bar or another structure part to attach to.');return;}
  let result=null,request=0;const revision=state.revision;
  state.placementPending=true;gizmo?.detach();
  modal(reconnect?'Reconnect body attachment':chain?'Attach flexible line to a part':'Attach body part to structure',reconnect?`<p>${esc(reconnect)}</p><p>Preview reaching the saved attachment point.</p><p id="attachment-preview-status" role="status"></p>`:`
    <div class="single-field"><label for="attachment-limb">${chain?'CHAIN LINK':'BODY PART'}</label><select id="attachment-limb">${limbs.map(p=>`<option value="${esc(p.id)}" ${p.id===selectedPart.id?'selected':''}>${esc(p.id)}</option>`).join('')}</select></div>
    ${chain?`<div class="single-field"><label for="attachment-source-port">POINT ON LINK</label><select id="attachment-source-port"><option value="b">B eye</option><option value="a">A eye</option></select></div>`:''}
    <div class="single-field"><label for="attachment-target">STRUCTURE PART</label><select id="attachment-target">${targets.map(p=>`<option value="${esc(p.id)}">${esc(p.id)}</option>`).join('')}</select></div>
    <div id="attachment-location"></div><div class="single-field"><label for="attachment-kind">ATTACHMENT</label><select id="attachment-kind"><option value="revolute">Grip · pivots around the bar</option><option value="fixed">Fixed · holds position and orientation</option><option value="spherical">Ball joint · rotates freely</option></select></div>
    <p>The preview poses the connected ${chain?'line':'limb'} to meet the attachment. Other attachments and joint limits remain active.${chain?' Multiple links can attach along a body; each connection is an idealized no-slip point, not frictional wrapping.':''}</p><p id="attachment-preview-status" role="status"></p>`,[
    {label:'Cancel',action:closeModal},
    {label:'Connect',primary:true,action:async()=>{if(!result||revision!==state.revision)return;const accepted=result;closeModal();await acceptPlacement(accepted,revision);toast(chain?'Flexible line attached.':'Body part attached.');}}
  ]);
  $('#modal').classList.add('connection-modal','attachment-modal');
  reviewCleanup=()=>{request++;clearSnapPreview();$('#modal').classList.remove('connection-modal','attachment-modal');state.placementPending=false;attachGizmo();};
  async function preview(){
    const current=++request;result=null;clearSnapPreview();$('#modal-actions .primary').disabled=true;
    $('#attachment-preview-status').textContent='Checking reach and existing connections…';
    let extra={object:instance.id,reconnect,preview:true};
    if(!reconnect){
      const part=state.scene.parts.find(p=>p.id===$('#attachment-target').value),target={part:part.id};
      if($('#attachment-station')){
        if(!$('#attachment-station').reportValidity()){ $('#attachment-preview-status').textContent='Choose a point within the pipe length.';return; }
        target.at_mm=+$('#attachment-station').value;
      }else if($('#attachment-surface-x'))target.surface_hint_mm=['x','y','z'].map(axis=>+$('#attachment-surface-'+axis).value);
      else if($('#attachment-port')?.value)target.port=$('#attachment-port').value;
      else target.frame={position_mm:[0,0,0],axis:[0,0,1]};
      extra={...extra,part:$('#attachment-limb').value,target,type:$('#attachment-kind').value,...(chain?{part_port:$('#attachment-source-port').value}:{})};
    }
    try{
      const data=await api('attach-part',extra);
      if(current!==request||revision!==state.revision)return;
      result=data;showPosePreview(data.poses);
      const bodyEnd=[data.joint?.a,data.joint?.b].find(end=>end?.frame&&state.scene.parts.find(p=>p.id===end.part)?.kind==='human');
      if(chain&&bodyEnd){
        const marker=new THREE.Mesh(new THREE.SphereGeometry(9,12,8),new THREE.MeshBasicMaterial({color:'#48c6a0',depthTest:false}));
        marker.position.fromArray(bodyEnd.frame.position_mm).applyMatrix4(partObjects.get(bodyEnd.part).matrix);
        marker.renderOrder=150;snapGhost.add(marker);
      }
      $('#attachment-preview-status').textContent='Ready to connect. '+(data.moved.length?(chain?'The green preview shows the fitted line.':'The green preview shows the new limb pose.'):'The attachment points already meet.')+(extra.target?.surface_hint_mm?' The joint lands on the body surface.':'');
      $('#modal-actions .primary').disabled=false;
    }catch(e){if(current===request)$('#attachment-preview-status').textContent=e.message;}
  }
  function location(){
    const target=state.scene.parts.find(p=>p.id===$('#attachment-target').value),limb=partObjects.get($('#attachment-limb').value);
    const options=Object.entries(target.ports||{}).filter(([,p])=>p.type!=='socket');
    const local=limb.position.clone().applyMatrix4(partObjects.get(target.id).matrix.clone().invert());
    $('#attachment-location').innerHTML=chain&&target.kind==='human'?`<div class="single-field"><label>BODY SURFACE NEAR LOCAL XYZ · mm</label><div class="fields">${['x','y','z'].map((axis,i)=>`<div class="field"><label for="attachment-surface-${axis}">${axis.toUpperCase()}</label><input id="attachment-surface-${axis}" type="number" step="any" value="${local.getComponent(i).toFixed(1)}"></div>`).join('')}</div></div><p>The chosen point is projected onto this body part's surface. Adjust XYZ to use a different side.</p>`:target.kind==='member'?`<div class="single-field"><label for="attachment-station">POINT FROM PIPE START · mm</label><input id="attachment-station" type="number" min="0" max="${target.length_mm}" step="any" value="${Math.min(target.length_mm,Math.max(0,local.z+target.length_mm/2)).toFixed(2)}"></div>`:options.length?`<div class="single-field"><label for="attachment-port">ATTACHMENT POINT</label><select id="attachment-port">${options.map(([id,p])=>`<option value="${esc(id)}">${esc(p.label||id)}</option>`).join('')}</select></div>`:'<p>Attach at the target part’s origin.</p>';
    if(chain&&target.kind==='human'){
      const shape=target.geometry[0],radius=shape.radius_mm??((shape.diameter_mm||0)/2);
      const half=shape.size_mm?.map(v=>v/2)||[radius,radius,(shape.length_mm||0)/2+radius];
      const row=document.createElement('div');row.className='single-field';
      row.innerHTML=`<label for="attachment-surface-side">BODY SIDE</label><select id="attachment-surface-side"><option value="">Nearest to line</option><option value="y+">Front (+Y)</option><option value="y-">Back (-Y)</option><option value="x-">Left (-X)</option><option value="x+">Right (+X)</option><option value="z+">Top (+Z)</option><option value="z-">Bottom (-Z)</option></select>`;
      $('#attachment-location').prepend(row);
      $('#attachment-surface-side').onchange=e=>{
        if(!e.target.value)return;
        const axis='xyz'.indexOf(e.target.value[0]),sign=e.target.value[1]==='+'?1:-1;
        for(let i=0;i<3;i++)$('#attachment-surface-'+'xyz'[i]).value=(i===axis?sign*(half[i]+50):THREE.MathUtils.clamp(+$('#attachment-surface-'+'xyz'[i]).value,-half[i],half[i])).toFixed(1);
        preview();
      };
    }
    $$('#attachment-location input, #attachment-location select').forEach(el=>{if(el.id!=='attachment-surface-side')el.onchange=preview;});preview();
  }
  if(reconnect)preview();else{
    if(chain){$('#attachment-kind').value='spherical';$('#attachment-source-port').value=selectedPart.id===instance.id+'/link-1'?'b':'a';$('#attachment-source-port').onchange=preview;}
    $('#attachment-target').onchange=location;$('#attachment-limb').onchange=location;$('#attachment-kind').onchange=preview;
    const origin=partObjects.get(selectedPart.id).position;
    $('#attachment-target').value=preferredTarget&&targets.some(p=>p.id===preferredTarget)?preferredTarget:[...targets].sort((a,b)=>partObjects.get(a.id).position.distanceTo(origin)-partObjects.get(b.id).position.distanceTo(origin))[0].id;
    location();
  }
}
function renderChainControls(instance,selectedPart){
  const info=state.scene.chains.find(c=>c.id===instance.id),section=document.createElement('div');section.className='inspect-section';
  const profiles=Object.entries(state.library).filter(([,part])=>part.kind==='chain'&&part.ports?.a&&part.ports?.b);
  section.innerHTML=`<h3>FLEXIBLE LINE</h3>
    <div class="single-field"><label for="chain-length">Length · mm</label><input id="chain-length" type="number" required min="0.001" max="${info.pitch_mm*1000}" step="any" value="${info.requested_length_mm}"></div>
    <p id="chain-length-summary">${info.count} segments × ${info.pitch_mm} mm pitch = ${info.length_mm} mm. Length rounds up to whole segments.</p>
    <div class="single-field"><label for="chain-profile">Material and section</label><select id="chain-profile">${profiles.map(([id,part])=>`<option value="${esc(id)}" ${id===info.link_catalog?'selected':''}>${esc(part.name||id)}</option>`).join('')}</select></div>
    <p>${esc(info.profile)} · ${info.break_force_n?`illustrative break threshold ${displayValue(info.break_force_n,'force',preferences.forceUnit)} ${preferences.forceUnit}${info.break_strain?` at about ${(100*info.break_strain).toFixed(1)}% tensile strain`:''}`:'no break threshold'}. Segment dimensions and mass follow the selected profile. Replace illustrative strengths with verified ratings for a real design.</p>
    <p>Move the line as one object or choose Pose segments. Simulation always lets its joints flex.</p>
    <div class="object-modes"><button id="chain-select-start">Select start</button><button id="chain-select-end">Select end</button></div>
    <div class="single-field"><label for="chain-link-index">Select segment</label><input id="chain-link-index" type="number" min="1" max="${info.count}" step="1" value="${Number(selectedPart.id.split('/link-').pop())||1}"></div>
    <div class="object-modes"><button id="chain-attach-start">Attach start</button><button id="chain-attach-end">Attach end</button></div>`;
  $('#object-expand').closest('.inspect-section').before(section);
  $('#chain-length').onchange=async e=>{if(!e.target.reportValidity())return;await objectEdit('object-parameters',{object:instance.id,parameters:{...instance.parameters,length_mm:+e.target.value}});};
  $('#chain-profile').onchange=e=>objectEdit('object-parameters',{object:instance.id,parameters:{...instance.parameters,link_catalog:e.target.value}});
  const selectEnd=end=>select(end==='start'?info.start_part:info.end_part);
  $('#chain-select-start').onclick=()=>selectEnd('start');$('#chain-select-end').onclick=()=>selectEnd('end');
  $('#chain-link-index').onchange=e=>{if(e.target.reportValidity())select(instance.id+'/link-'+e.target.value);};
  for(const end of ['start','end'])$('#chain-attach-'+end).onclick=()=>attachmentDialog(instance,state.scene.parts.find(p=>p.id===info[end+'_part']));
}

function wheelDialog(catalog='generic.wheel',position=null,wheelId=null){
  const wheels=Object.entries(state.library).filter(([,part])=>part.kind==='wheel'&&part.ports?.axle);
  if(!wheels.length){toast('No wheel with an axle port is available.');return;}
  const existing=wheelId&&state.scene.parts.find(part=>part.id===wheelId);
  const definition=state.library[catalog]||wheels[0][1];
  const parameters=existing?{...definition.parameters,...state.doc.parts.find(part=>part.id===wheelId)?.parameters}:definition.parameters;
  const targets=state.scene.parts.filter(part=>part.id!==wheelId&&!draftRun(state.doc,part.id));
  if(existing&&!targets.length){toast('Add a part to mount this wheel to first.');return;}
  const selectedTarget=targets.some(part=>part.id===state.selected)?state.selected:existing?targets[0]?.id:'';
  const revision=state.revision;
  const wheelOptions=wheels.map(([id,part])=>`<option value="${esc(id)}" ${id===catalog?'selected':''}>${esc(part.name||id)}</option>`).join('');
  const targetOptions=`${existing?'':'<option value="">Leave unattached for now</option>'}${targets.map(part=>`<option value="${esc(part.id)}">${esc(part.label||part.id)} · ${esc(part.id)}</option>`).join('')}`;
  const dimensions=existing?'':`<div class="single-field"><label for="wheel-catalog">WHEEL TYPE</label><select id="wheel-catalog">${wheelOptions}</select></div>
    <div class="fields"><div class="field"><label for="wheel-diameter">DIAMETER · mm</label><input id="wheel-diameter" type="number" min="0.01" step="any" value="${esc(parameters.diameter_mm)}"></div>
    <div class="field"><label for="wheel-width">WIDTH · mm</label><input id="wheel-width" type="number" min="0.01" step="any" value="${esc(parameters.width_mm)}"></div>
    <div class="field"><label for="wheel-mass">MASS · kg</label><input id="wheel-mass" type="number" min="0.001" step="any" value="${esc(parameters.mass_kg)}"></div></div>`;
  modal(existing?'Mount wheel axle':'Add a wheel',`${dimensions}<div class="single-field"><label for="wheel-target">MOUNT TO</label><select id="wheel-target">${targetOptions}</select></div>
    <div id="wheel-mount-fields"><div class="single-field"><label for="wheel-location">AXLE LOCATION</label><select id="wheel-location"></select></div>
    <div id="wheel-station-fields" class="fields"><div class="field"><label for="wheel-station">DISTANCE FROM PIPE START · mm</label><input id="wheel-station" type="number" min="0" step="any"></div>
    <div class="field"><label for="wheel-side">SIDE</label><select id="wheel-side"><option value="y">+Y</option><option value="-y">−Y</option><option value="x">+X</option><option value="-x">−X</option></select></div>
    <div class="field"><label for="wheel-clearance">AXLE CLEARANCE · mm</label><input id="wheel-clearance" type="number" min="0" step="any" value="10"></div></div>
    <div id="wheel-custom-fields"><p>Coordinates are relative to the chosen part. The wheel centre is placed at this axle point.</p>
      <div class="fields">${['x','y','z'].map(axis=>`<div class="field"><label for="wheel-local-${axis}">${axis.toUpperCase()} · mm</label><input id="wheel-local-${axis}" type="number" step="any" value="0"></div>`).join('')}</div>
      <div class="single-field"><label for="wheel-axis">AXLE DIRECTION</label><select id="wheel-axis"><option value="0,1,0">+Y</option><option value="0,-1,0">−Y</option><option value="1,0,0">+X</option><option value="-1,0,0">−X</option><option value="0,0,1">+Z</option><option value="0,0,-1">−Z</option></select></div></div>
    <p id="wheel-mount-summary" role="status"></p></div><p>The wheel spins on a revolute axle joint. Specify the real axle, bearing and retaining hardware before fabrication.</p>`,[
    {label:'Cancel',action:closeModal},
    {label:existing?'Mount wheel':'Add wheel',primary:true,action:async()=>{
      const positive=(id,label)=>{const value=Number($(id).value);if(!Number.isFinite(value)||value<=0)throw new Error(`${label} must be positive.`);return value;};
      const wheelWidth=existing?parameters.width_mm:positive('#wheel-width','Wheel width');
      const targetId=$('#wheel-target').value;
      let target=null;
      if(targetId){
        const part=targets.find(item=>item.id===targetId),mode=$('#wheel-location').value;
        if(mode==='port')target={part:targetId,port:$('#wheel-port').value};
        else if(mode==='station'){
          const station=Number($('#wheel-station').value),clearance=Number($('#wheel-clearance').value);
          if(!Number.isFinite(station)||station<0||station>part.length_mm)throw new Error('Choose a point along the pipe.');
          if(!Number.isFinite(clearance)||clearance<0)throw new Error('Axle clearance cannot be negative.');
          const side=$('#wheel-side').value,sign=side.startsWith('-')?-1:1,index=side.endsWith('x')?0:1;
          const section=part.section||{},radius=(section.diameter_mm||Math.max(section.width_mm||0,section.height_mm||0))/2;
          const offset=radius+wheelWidth/2+clearance,local=[0,0,station-part.length_mm/2],axis=[0,0,0];
          local[index]=sign*offset;axis[index]=sign;
          target={part:targetId,frame:{position_mm:local,axis}};
        }else{
          const coordinates=['x','y','z'].map(axis=>Number($(`#wheel-local-${axis}`).value));
          if(!coordinates.every(Number.isFinite))throw new Error('Enter finite axle coordinates.');
          target={part:targetId,frame:{position_mm:coordinates,axis:$('#wheel-axis').value.split(',').map(Number)}};
        }
      }else if(existing)throw new Error('Choose a part to mount the wheel to.');
      const payload=existing?{wheel:wheelId,target}:{catalog:$('#wheel-catalog').value,
        parameters:{diameter_mm:positive('#wheel-diameter','Wheel diameter'),width_mm:wheelWidth,mass_kg:positive('#wheel-mass','Wheel mass')},target,
        ...(position?{pose:{position_mm:[position[0],position[1],position[2]+positive('#wheel-diameter','Wheel diameter')/2]}}:{})};
      const result=await api(existing?'mount-wheel':'add-wheel',payload);
      closeModal();state.selected=result.wheel;await acceptPlacement(result,revision);
      setMode('design');setTool('translate');toast(target?'Wheel mounted on a free-spinning axle.':'Wheel added. Select it to mount its axle later.');
    }}]);
  $('#wheel-target').value=selectedTarget||'';
  function refreshLocation(){
    const part=targets.find(item=>item.id===$('#wheel-target').value),fields=$('#wheel-mount-fields');
    fields.classList.toggle('hidden',!part);if(!part)return;
    const ports=Object.entries(part.ports||{}).filter(([name,port])=>port.type!=='socket'&&!socketOccupied(state.scene,part.id,name));
    $('#wheel-location').innerHTML=`${part.kind==='member'?'<option value="station">Along pipe or profile</option>':''}${ports.length?'<option value="port">Attachment port</option>':''}<option value="custom">Local point</option>`;
    $('#wheel-station').value=part.kind==='member'?part.length_mm/2:0;
    $('#wheel-station').max=part.length_mm||0;
    $('#wheel-port-options')?.remove();
    const holder=document.createElement('div');holder.id='wheel-port-options';holder.className='single-field';
    holder.innerHTML=`<label for="wheel-port">PORT</label><select id="wheel-port">${ports.map(([name,port])=>`<option value="${esc(name)}">${esc(name)} · ${esc(port.type)}</option>`).join('')}</select>`;
    $('#wheel-location').parentElement.after(holder);
    $('#wheel-location').onchange=refreshMode;refreshMode();
  }
  function refreshMode(){
    const mode=$('#wheel-location').value;
    $('#wheel-station-fields').classList.toggle('hidden',mode!=='station');
    $('#wheel-port-options').classList.toggle('hidden',mode!=='port');
    $('#wheel-custom-fields').classList.toggle('hidden',mode!=='custom');
    $('#wheel-mount-summary').textContent=mode==='station'?'The axle sits beside the chosen pipe station. Adjust the clearance for your mounting hardware.':
      mode==='port'?'The wheel axle will align exactly with the selected port.':'The wheel axle will align with this point and direction on the part.';
  }
  $('#wheel-target').onchange=refreshLocation;
  if($('#wheel-catalog'))$('#wheel-catalog').onchange=()=>{
    const next=state.library[$('#wheel-catalog').value].parameters;
    for(const [field,key] of [['#wheel-diameter','diameter_mm'],['#wheel-width','width_mm'],['#wheel-mass','mass_kg']])$(field).value=next[key];
  };
  refreshLocation();
}

function chainDialog(catalog='generic.chain-link',position=null){
  if(state.busy||state.placementPending)return;
  const links=Object.entries(state.library).filter(([,p])=>p.kind==='chain'&&p.ports?.a&&p.ports?.b);
  modal('Add a flexible line',`<div class="single-field"><label for="chain-catalog">MATERIAL AND SECTION</label><select id="chain-catalog">${links.map(([id,p])=>`<option value="${esc(id)}" ${id===catalog?'selected':''}>${esc(p.name||id)}</option>`).join('')}</select></div><div class="single-field"><label for="new-chain-length">Length · mm</label><input id="new-chain-length" type="number" required min="0.001" step="any" value="${preferences.defaultChainLengthMm}"></div><p id="new-chain-summary" role="status"></p><p>Short segments flex at ball joints. Geometry, mass and collision size follow the selected profile; example break thresholds are illustrative.</p>`,[
    {label:'Cancel',action:closeModal},{label:'Add line',primary:true,action:async()=>{
      const input=$('#new-chain-length');if(!input.reportValidity())return;
      const parameters={length_mm:+input.value,link_catalog:$('#chain-catalog').value};
      let n=1;while(state.scene.parts.some(p=>p.id==='chain-'+n||p.id.startsWith('chain-'+n+'/'))||(state.doc.objects||[]).some(o=>o.id==='chain-'+n))n++;
      const id='chain-'+n,pitch=chainPitch(parameters.link_catalog),length=Math.ceil(parameters.length_mm/pitch)*pitch;
      await mutate(()=>{state.doc.objects||=[];state.doc.objects.push({id,template:'chain',parameters,pose:{position_mm:position?[position[0],position[1],position[2]+length+20]:[0,-600,length+100]}});state.selected=id+'/link-1';});
      closeModal();setMode('design');setTool('translate');fitView();toast('Added '+id+'. Set its length in Properties or choose Pose segments.');
    }}]);
  function refresh(){const profile=state.library[$('#chain-catalog').value],pitch=chainPitch($('#chain-catalog').value),length=+$('#new-chain-length').value,count=Math.max(1,Math.ceil(length/pitch));$('#new-chain-length').max=pitch*1000;$('#new-chain-summary').textContent=`${count} segments × ${pitch} mm pitch = ${count*pitch} mm. ${profile.break_force_n?`Illustrative break threshold: ${displayValue(profile.break_force_n,'force',preferences.forceUnit)} ${preferences.forceUnit}.`:'No break threshold specified.'}`;}
  $('#new-chain-length').value=Math.min(preferences.defaultChainLengthMm,chainPitch($('#chain-catalog').value)*1000);
  $('#new-chain-length').oninput=refresh;$('#chain-catalog').onchange=refresh;refresh();
}
function chainPitch(catalog){const ports=state.library[catalog].ports;return Math.hypot(...ports.a.position_mm.map((v,i)=>v-ports.b.position_mm[i]));}

function renderObjectInspector(instance,selectedPart){
  const pose={position_mm:[0,0,0],rotation_deg:[0,0,0],...instance.pose}, human=instance.template==='human',chain=instance.template==='chain';
  const parameters=human?{stature_mm:1750,mass_kg:75,strength_scale:1,joint_damping_nms_rad:.08,...instance.parameters}:instance.parameters||{};
  const posture=humanPosture(parameters);
  $('#inspector').innerHTML=`<div class="inspect-section"><div class="inspect-id">${esc(instance.template)}</div><h2>${esc(instance.label||instance.id)}</h2><div class="object-modes" role="group" aria-label="Object manipulation"><button data-object-mode="whole" aria-pressed="${objectMode(instance)!=='limb'}">${human?'Move whole person':chain?'Move whole chain':'Move whole object'}</button><button data-object-mode="limb" aria-pressed="${objectMode(instance)==='limb'}">${human?'Pose limbs':chain?'Pose links':'Pose parts'}</button></div><p>${objectMode(instance)==='limb'?'Select a body part, then drag or rotate it. Connected joints follow within their limits.':'Drag or rotate any part to move the entire object and keep its pose.'}</p></div><div class="inspect-section">${propertyFields('OBJECT POSITION',pose.position_mm,'position_mm')}${propertyFields('OBJECT ROTATION',pose.rotation_deg,'rotation_deg')}</div><div class="inspect-section"><h3>${chain?'ADVANCED EDITING':'OBJECT PARAMETERS'}</h3>${Object.entries(parameters).filter(([k,v])=>!chain&&typeof v==='number'&&(!human||k!=='grip_diameter_mm')).map(([k,v])=>`<div class="single-field"><label>${esc(k==='joint_damping_nms_rad'?'PASSIVE JOINT DAMPING · N·m·s/rad':k.replaceAll('_',' ').toUpperCase())}</label><input type="number" data-object-param="${esc(k)}" value="${v}"></div>`).join('')}${human?`
    ${instance.components?'<p id="object-edited-pose">Edited pose · use Pose limbs to adjust it.</p>':`<div class="single-field"><label for="object-pose">INITIAL POSE</label><select id="object-pose">${humanPoseOptions(parameters.pose)}</select><p id="object-pose-description">${esc(humanPoseDescription(parameters.pose||'standing'))}</p></div>`}
    <div class="single-field"><label for="object-hold">POSTURE CONTROL</label><select id="object-hold">${postureOptions(posture)}</select></div>
    <div class="single-field" id="object-held-joints-field" ${posture==='custom'?'':'hidden'}><label for="object-held-joints">JOINTS OR GROUPS · COMMA SEPARATED</label><input id="object-held-joints" value="${esc((parameters.hold_joints||[]).join(', '))}" placeholder="left_elbow, right_elbow, torso"></div>
    <p>Mass and stature estimate torso and limb thickness for rendering and fit checks. Body shape varies between people of the same mass.</p>
    <p>Held joints resist motion with finite torque. Upper body leaves hips, knees and ankles free. Strength scale adjusts available torque. Passive damping slows free motion without holding an angle.</p>
    <div class="single-field"><label for="object-grip">GRIPPED BAR DIAMETER · mm</label><input id="object-grip" type="number" min="8" max="80" step="0.1" placeholder="Open hands" value="${parameters.grip_diameter_mm??''}"></div>
    <p>Curled hands have a grip port at the palm centre. Connect it to a bar with a revolute joint for a grasp that pivots around the bar. Calibrate limb measurements and initial joint angles in Source or expand the model.</p>`:''}<button class="inspect-action" id="object-expand">Expand into editable parts</button></div>`;
  if(human){
    const planes=(state.doc.draft_subassemblies||[]).flatMap(group=>(group.mirrors||[])
      .filter(plane=>plane.axis==='x'||plane.axis==='y').map(plane=>({group,plane})));
    const current=instance.symmetry,section=document.createElement('div');section.className='inspect-section';
    section.innerHTML=`<h3>MIRROR-LINE POSE</h3><div class="single-field"><label for="human-mirror-line">FIX PERSON TO MIRROR</label><select id="human-mirror-line">
      <option value="" ${current?'':'selected'}>Free pose</option>
      ${current?`<option value="current" selected>Current line · ${esc(current.axis.toUpperCase())} = ${esc(current.offset_mm)} mm</option>`:''}
      ${planes.map(({group,plane},index)=>`<option value="${index}">${esc(group.id)} · ${esc(plane.axis.toUpperCase())} = ${esc(plane.offset_mm)} mm</option>`).join('')}
    </select></div><p>${current?`Centerline fixed at ${esc(current.axis.toUpperCase())} = ${esc(current.offset_mm)} mm and ${current.axis==='x'?'Y':'X'} = ${esc(current.line_offset_mm)} mm. The whole person moves vertically and can rotate while its left-right axis stays perpendicular to the mirror; arms and legs pose in mirrored pairs.`:
      'Choose a vertical draft mirror. The current position sets the centerline; mirrored arms and legs will pose together.'}</p>
      ${planes.length?'':'<p>Add an X or Y draft mirror plane to use this constraint.</p>'}
      ${state.mirrorPoseError?.startsWith(instance.id+':')?`<p class="issue error">${esc(state.mirrorPoseError)}</p>`:''}`;
    $('#inspector').appendChild(section);
    $('#human-mirror-line').onchange=e=>{
      if(e.target.value==='current')return;
      const choice=planes[Number(e.target.value)];
      objectEdit('human-symmetry',{object:instance.id,...(e.target.value===''?{}:{group:choice.group.id,plane:choice.plane.id})});
    };
    if(current)$$('[data-pose="position_mm"]').forEach(input=>{if(+input.dataset.axis<2)input.disabled=true;});
  }
  if(chain){
    $('#inspector [data-object-mode="whole"]').textContent='Move whole line';
    $('#inspector [data-object-mode="limb"]').textContent='Pose segments';
    $('#inspector .inspect-id').textContent=state.scene.chains.find(c=>c.id===instance.id)?.profile||'chain';
  }
  if(selectedPart){
    const segment=document.createElement('div');segment.className='inspect-section';segment.hidden=objectMode(instance)!=='limb';
    segment.innerHTML=`<h3>${esc(selectedPart.label||selectedPart.id)}</h3><p>Adjust this part to pose its joints. The object stays grouped; connected parts follow.</p>`+
      (propertyFields('PART POSITION',selectedPart.pose.position_mm,'position_mm')+propertyFields('PART ROTATION',selectedPart.pose.rotation_deg,'rotation_deg')).replaceAll('data-pose=','data-part-pose=');
    $('#inspector').children[1].before(segment);
    $$('[data-part-pose]').forEach(input=>input.onchange=()=>editPose(selectedPart,input));
  }
  $$('[data-object-mode]').forEach(button=>button.onclick=async()=>{
    if(state.placementPending||state.busy)return;
    if(chain){await objectEdit('object-layout',{object:instance.id,mode:button.dataset.objectMode==='limb'?'posable':'rigid'});}else{objectModes.set(instance.id,button.dataset.objectMode);if(button.dataset.objectMode==='limb')openObjects.add(instance.id);}
    select(state.selected);if(!['translate','rotate'].includes(state.tool))setTool('translate');
  });
  $('#inspector [data-pose]').closest('.inspect-section').hidden=objectMode(instance)==='limb';
  $$('[data-pose]').forEach(input=>input.onchange=()=>{
    const next=clone(pose);next[input.dataset.pose][+input.dataset.axis]=+input.value;
    moveTarget(selectedPart.id,next,input.dataset.pose==='rotation_deg'?'rotate':'translate',instance.id);
  });
  const changeParameters=edit=>{const next=clone(instance.parameters||{});edit(next);return objectEdit('object-parameters',{object:instance.id,parameters:next});};
  $$('[data-object-param]').forEach(input=>input.onchange=()=>changeParameters(p=>{p[input.dataset.objectParam]=+input.value;}));
  if(human){
    if($('#object-pose'))$('#object-pose').onchange=e=>{
      $('#object-pose-description').textContent=humanPoseDescription(e.target.value);
      changeParameters(p=>{p.pose=e.target.value;});
    };
    $('#object-hold').onchange=e=>{
      const mode=e.target.value;
      if(mode==='custom'){$('#object-held-joints-field').hidden=false;$('#object-held-joints').focus();return;}
      changeParameters(p=>setHumanPosture(p,mode));
    };
    $('#object-held-joints').onchange=e=>changeParameters(p=>setHumanPosture(p,'custom',e.target.value.split(',').map(s=>s.trim()).filter(Boolean)));
    $('#object-grip').onchange=e=>changeParameters(p=>{if(e.target.value==='')delete p.grip_diameter_mm;else p.grip_diameter_mm=+e.target.value;});
  }
  if(chain)renderChainControls(instance,selectedPart);
  renderObjectAttachments(instance,selectedPart);
  bindDuplicateButton();bindFloorButtons(selectedPart.id,instance.id);$('#object-expand').onclick=()=>expandObjects(instance.id);

}

function renderChecks(){const result=state.checks,analysis=state.analysis;$('#inspector').innerHTML=`<div class="inspect-section"><div class="report-head"><div class="report-symbol ${result&&!result.valid?'error':''}">${result?(result.valid?'✓':'!'):'◇'}</div><div><h2>${result?(result.valid?'Geometry checked':'Review connections'):'Check the design'}</h2><div class="subtle">Sockets · alignment · collisions</div></div></div><button class="inspect-action" data-run="validate">${state.busy?'Working…':'Run validation'}</button>${result?`<div class="metric-cards"><div class="metric"><strong>${result.summary.errors}</strong><span>ERRORS</span></div><div class="metric"><strong>${result.summary.warnings}</strong><span>ADVISORIES</span></div></div>`:''}</div><div class="inspect-section"><h3>STRUCTURAL RESPONSE</h3><p>3D beam analysis of the current load case. Review the material and connector assumptions with the results.</p><button class="inspect-action secondary" data-run="analyse">Calculate stress & deflection</button>${analysis?`<p><b>${esc(analysis.status)}</b></p>${analysis.message?'<p>'+esc(analysis.message)+'</p>':''}${(analysis.members||[]).map(m=>`<button class="issue" data-result-part="${esc(m.part)}"><div class="issue-code">${esc(m.part)}</div><div class="property-row"><span>Peak stress</span><strong>${m.max_von_mises_mpa.toFixed(2)} MPa</strong></div><div class="property-row"><span>Deflection</span><strong>${m.max_displacement_mm.toFixed(3)} mm</strong></div></button>`).join('')}<p>Model results · unverified strength data remains unknown.</p>`:''}</div>${result?'<div class="inspect-section"><h3>FINDINGS</h3>'+result.issues.map((i,index)=>`<button class="issue ${i.severity}" data-result-finding="${index}" data-result-part="${esc(i.parts?.[0]||'')}"><div class="issue-code">${esc(i.code)}</div><p>${esc(i.message)}</p></button>`).join('')+'</div>':''}`;bindOperations();}
function renderSimulation(){const r=state.recording;const humans=(state.doc.objects||[]).filter(o=>o.template==='human');$('#inspector').innerHTML=`<div class="inspect-section"><h2>Let physics explain it.</h2><p>Release the structure under gravity. Loose sockets can slide and turn; motors act through physical joints.</p><div class="single-field"><label>DURATION · SECONDS</label><input id="sim-duration" type="number" min="0.1" max="30" step="1" value="${state.simulationOptions?.duration||Math.min(30,Math.max(.1,Number(state.doc.metadata?.simulation_duration_s)||preferences.simulationSeconds))}"></div><div class="single-field"><label for="sim-chain-links">Chain links per rigid body</label><input id="sim-chain-links" type="number" min="1" max="1000" step="1" value="${state.simulationOptions?.chain_links_per_body||preferences.simulationChainLinks}" ${state.busy?'disabled':''}></div><p>1 keeps full flexibility. Higher values make groups of links rigid for faster simulation. Attachment links stay flexible.</p><button class="inspect-action" data-run="simulate" ${state.busy?'disabled':''}>${state.simulation?'Simulating…':'Run simulation ▶'}</button>${state.simulation?'<div id="sim-progress" role="status" aria-live="polite"></div><button class="inspect-action secondary" id="cancel-simulation">Cancel simulation</button>':''}${state.simulationError?`<div class="issue error" role="alert" id="simulation-error"><strong>Simulation error</strong><p>${esc(state.simulationError)}</p></div>`:''}${r?`<div class="metric-cards"><div class="metric"><strong>${r.frames.length}</strong><span>RECORDED FRAMES</span></div><div class="metric"><strong>${r.settled?'Settled':'Moving'}</strong><span>FINAL STATE</span></div></div><p>${r.final_max_speed_m_s?.toFixed(3)||'0'} m/s maximum final speed</p>`:''}</div><div class="inspect-section"><h3>MOVING CONNECTIONS</h3>${(state.scene.chains||[]).map(c=>`<div class="joint-card"><div class="joint-card-top">${esc(c.id)}</div><p>${c.count} flexible links</p></div>`).join('')}${state.scene.joints.filter(j=>!j.locked&&j.type!=='fixed'&&!(state.scene.chains||[]).some(c=>j.a.part.startsWith(c.id+'/')&&j.b.part.startsWith(c.id+'/'))).map(j=>`<div class="joint-card"><div class="joint-card-top">${esc(j.id)}</div><p>${esc(j.type)}${j.motor?' · motor':''}</p>${Object.entries(j.limits||{}).map(([k,v])=>`<p>${esc(k)}: ${esc(JSON.stringify(v))}</p>`).join('')}</div>`).join('')||((state.scene.chains||[]).length?'':'<p>All connected parts are secured. Loosen a socket in Design mode to give it motion.</p>')}</div><div class="inspect-section"><h3>HUMAN FIT</h3><p>Test joint-limited reach and seated dimensions with a 19-segment human model.</p><button class="inspect-action secondary" id="fit-human">Open fit test</button><button class="inspect-action secondary" data-run="fit">Run saved design tests</button></div>${r?.events?.length?'<div class="inspect-section"><h3>SIMULATION EVENTS</h3>'+r.events.map(e=>`<div class="issue"><div class="issue-code">${esc(e.type)}</div><p>${esc(e.part||e.joint||'')} ${e.note?esc(e.note):''}</p></div>`).join('')+'</div>':''}`;bindOperations();$('#fit-human').onclick=fitDialog;if($('#cancel-simulation'))$('#cancel-simulation').onclick=cancelSimulation;updateSimulationProgress();}
function renderBuild(){const p=state.plan;$('#inspector').innerHTML=`<div class="inspect-section"><h2>A plan you can build.</h2><p>Find an insertion order with clear paths and stable intermediate structures.</p><button class="inspect-action" data-run="plan">${state.busy?'Checking insertion paths…':'Find assembly order'}</button>${p?`<div class="report-head" style="margin-top:17px"><div class="report-symbol ${p.status==='buildable'?'':'warn'}">${p.status==='buildable'?'✓':'?'}</div><div><b>${esc(p.status)}</b><div class="subtle">${p.steps.length} assembly steps</div></div></div>${p.reason?'<p>'+esc(p.reason)+'</p>':''}`:''}</div>${p?.status==='buildable'?`<div class="inspect-section"><h3>STEP ${state.step+1} OF ${p.steps.length}</h3><p>${esc(p.steps[state.step].instruction)}</p>${p.steps[state.step].fasten.map(f=>'<p>↳ '+esc(f.action)+'</p>').join('')}<button class="inspect-action" id="export-build">Export illustrated build book ↗</button></div><div class="inspect-section"><h3>ASSEMBLY ORDER</h3>${p.steps.map((s,i)=>`<button class="build-step ${i===state.step?'active':''}" data-step="${i}"><span>${String(s.number).padStart(2,'0')}</span>${esc(s.part)}</button>`).join('')}</div>`:''}`;bindOperations();$$('[data-step]').forEach(b=>b.onclick=()=>showStep(+b.dataset.step));if($('#export-build'))$('#export-build').onclick=()=>exportBuild();}
function findingReference(button){
  const finding=state.checks?.issues[Number(button.dataset.resultFinding)]||{};
  const joints=[finding.joint,...(finding.joints||[])].filter(Boolean);
  const joint=state.scene.joints.find(j=>joints.includes(j.id));
  const unsupported=finding.code==='UNSUPPORTED'?(state.checks?.support?.components||[]).filter(c=>!c.stable).flatMap(c=>c.parts):[];
  const ids=[...(finding.parts||[]),button.dataset.resultPart,joint?.a.part,joint?.b.part,...unsupported];
  return {finding,joints,part:ids.find(id=>state.scene.parts.some(p=>p.id===id))};
}
function openFindingProperties(button){
  if(state.busy||state.placementPending)return;
  const {finding,joints,part}=findingReference(button);
  // Findings about design-wide references have no individual Properties page.
  if(!part){sourceDialog();return;}
  restore();setTool('select');
  const instance=state.doc.objects?.find(o=>part.startsWith(o.id+'/'));
  if(instance){openObjects.add(instance.id);objectModes.set(instance.id,finding.code==='UNSUPPORTED'?'whole':'limb');}
  state.selected=part;setMode('design');select(part);
  const connection=$$('[data-joint-edit], [data-attachment-edit]').find(b=>joints.includes(b.dataset.jointEdit||b.dataset.attachmentEdit));
  const lock=$$('[data-lock]').find(b=>joints.includes(b.dataset.lock));
  let control=finding.code==='LOOSE_SCREW'&&lock?lock:connection;
  if(!control&&joints.length&&instance)control=$('#object-expand');
  if(!control){
    const parameter={LENGTH:'length_mm',WALL:'wall_mm'}[finding.code];
    if(parameter)control=$$('[data-param]').find(input=>input.dataset.param===parameter);
    if(['GEOMETRY','DIMENSION','MASS','MESH_MASS','APPROXIMATE_GEOMETRY','PROFILE_MISMATCH'].includes(finding.code))control=$('#edit-part-definition')||$('#object-expand');
    const visible=e=>!e.closest('[hidden]');
    control||=finding.code==='UNSUPPORTED'?$$('[data-drop-floor]').find(visible):null;
    control||=$$('[data-part-pose="position_mm"], [data-pose="position_mm"]').find(visible);
  }
  $('#inspector').scrollTop=0;
  control?.scrollIntoView?.({block:'center'});control?.focus({preventScroll:true});
  status('Properties opened for '+part);
}
function bindOperations(){
  $$('[data-run]').forEach(b=>{b.disabled=state.busy;b.onclick=()=>run(b.dataset.run);});
  $$('[data-result-part]').forEach(b=>{
    b.title='Click to highlight. Double-click or press Enter to open Properties.';
    // Keep the finding node intact on the first click so native double-clicks
    // can reach the same button, including when its text was clicked.
    b.onclick=()=>{const {part}=findingReference(b);if(part){state.selected=part;highlightSelection();}};
    b.ondblclick=()=>openFindingProperties(b);
    b.onkeydown=e=>{if(e.key==='Enter'){e.preventDefault();openFindingProperties(b);}};
  });
}
function updateSimulationProgress(){
  const job=state.simulation;if(!job)return;
  const p=job.progress||{};let message=job.cancelRequested?'Cancelling simulation…':p.message||'Preparing simulation';
  if(p.simulated_s!==undefined)message+=` · ${p.simulated_s.toFixed(2)} / ${p.duration_s.toFixed(2)} s (${p.percent.toFixed(1)}%)`;
  if(job.elapsed_s!==undefined)message+=` · ${Math.round(job.elapsed_s)} s elapsed`;
  if(p.eta_s!=null)message+=` · about ${Math.ceil(p.eta_s)} s remaining`;
  status(message);
  const panel=$('#sim-progress');if(panel){panel.replaceChildren();const text=document.createElement('p');text.textContent=message;panel.appendChild(text);const meter=document.createElement('progress');meter.max=100;if(p.percent!==undefined)meter.value=p.percent;meter.setAttribute('aria-label','Simulation progress');meter.style.width='100%';panel.appendChild(meter);}
  const cancel=$('#cancel-simulation');if(cancel)cancel.disabled=!!job.cancelRequested;
}
async function cancelSimulation(){
  const job=state.simulation;if(!job||job.cancelRequested)return;
  job.cancelRequested=true;updateSimulationProgress();
  if(!job.id)return;
  try{await api('simulation-cancel',{document:null,job_id:job.id});}
  catch(error){job.cancelRequested=false;updateSimulationProgress();toast('Could not cancel: '+error.message,true);}
}
async function generateSimulation(options){
  const job=state.simulation;
  try{
    Object.assign(job,await api('simulate',options));updateSimulationProgress();
    while(true){
      if(job.cancelRequested){await api('simulation-cancel',{document:null,job_id:job.id});status('Simulation cancelled');return null;}
      if(job.status==='cancelled'){status('Simulation cancelled');return null;}
      if(job.status==='failed')throw new Error(job.error||'Simulation failed');
      if(job.status==='completed'){
        const completed=await api('simulation-result',{document:null,job_id:job.id});
        if(job.cancelRequested){status('Simulation cancelled');return null;}
        if(!completed.result)throw new Error(completed.error||'Simulation recording is unavailable');
        return completed.result;
      }
      await new Promise(resolve=>setTimeout(resolve,500));
      Object.assign(job,await api('simulation-status',{document:null,job_id:job.id}));updateSimulationProgress();
    }
  }catch(error){
    if(job.id&&job.status==='running'){
      try{await api('simulation-cancel',{document:null,job_id:job.id});}
      catch{error.message+='; unable to reach the worker to cancel it. Restart the server if it is still running.';}
    }
    throw error;
  }
}
async function run(operation){if(state.busy)return;state.busy=true;const revision=state.revision;const extra=operation==='simulate'?{duration:Number($('#sim-duration')?.value||3),chain_links_per_body:Number($('#sim-chain-links')?.value||1),deflection_warning_mm:preferences.deflectionWarningMm}:{};if(operation==='simulate'){state.simulationError=null;state.simulationOptions=extra;state.simulation={status:'starting',progress:{message:'Preparing simulation'}};state.playing=false;}status(({validate:'Checking sockets and collisions…',analyse:'Solving the structural load case…',simulate:'Integrating rigid-body physics…',plan:'Searching stable assembly sequences…',fit:'Checking human fit…'})[operation]);renderInspector();
  try{const result=operation==='simulate'?await generateSimulation(extra):await api(operation,extra);if(!result)return;if(revision!==state.revision){toast('The design changed during the calculation. Run it again for the current design.');return;}state.doc.results||={};state.doc.results[operation]=result;state.dirty=true;
    if(operation==='validate'){state.checks=result;status(result.valid?'Geometry checks passed':'Design has '+result.summary.errors+' errors');}
    if(operation==='analyse'){state.analysis=result;status('Structural analysis: '+result.status);applyStructuralColors();}
    if(operation==='simulate'){state.recording=result;state.frame=0;installBeamPlayback(result);$('#time-slider').max=result.frames.length-1;$('#timeline-mid').textContent=(result.duration_s/2).toFixed(1)+' s';$('#timeline-end').textContent=result.duration_s.toFixed(1)+' s';state.playing=true;showFrame(0);status('Simulation recorded · '+result.frames.length+' frames');}
    if(operation==='plan'){state.plan=result;state.doc.build_plan=result;state.step=0;status('Assembly search: '+result.status);if(result.status==='buildable')showStep(0);}
    if(operation==='fit')jsonDialog('Human fit results',result);updateHeader();
  }catch(e){if(operation==='simulate')state.simulationError=e.message;toast(e.message,true);status('Operation needs attention');}finally{state.busy=false;state.simulation=null;renderInspector();}}
function setMode(mode){state.mode=mode;if(mode!=='design'&&resizeDrag)clearResizeDrag();if(mode!=='design'&&renderer)renderer.domElement.style.cursor='';$$('[data-mode]').forEach(b=>{b.classList.toggle('active',b.dataset.mode===mode);b.setAttribute('aria-selected',b.dataset.mode===mode?'true':'false');});if(mode!=='design'){gizmo?.detach();if(state.tool==='connect')setTool('select');}else attachGizmo();ports.visible=mode==='design';for(const obj of partObjects.values())obj.visible=true;for(const obj of beamObjects.values())obj.visible=false;if(mode==='simulate'&&state.recording)showFrame(state.frame);if(mode==='build'&&state.plan?.status==='buildable')showStep(state.step);applyStructuralColors();renderInspector();updateResizeHandles();}
function showStep(index){if(state.plan?.status!=='buildable')return;state.step=index;const step=state.plan.steps[index];state.selected=step.part;for(const [id,object] of partObjects)object.visible=step.installed_parts.includes(id);highlightSelection();ports.visible=false;renderBuild();}
function statusColor(deflection,yielded){return yielded?'#e45b55':deflection>preferences.deflectionWarningMm?'#efb849':null;}
function tintPart(group,color){if(!group)return;group.traverse(object=>{if(!object.isMesh||!object.material?.color)return;object.userData.baseColor??=object.material.color.clone();object.material.color.copy(color?new THREE.Color(color):object.userData.baseColor);});}
function applyStructuralColors(){for(const p of state.scene?.parts||[])tintPart(partObjects.get(p.id),null);if(state.mode!=='check')return;for(const member of state.analysis?.members||[]){const unsafe=(member.yield_utilisation||0)>=1||(member.buckling_utilisation||0)>=1; tintPart(partObjects.get(member.part),statusColor(member.max_displacement_mm,unsafe));}}
function installBeamPlayback(recording){for(const [id,group] of beamObjects){objects.remove(group);dispose(group);partObjects.delete(id);}beamObjects.clear();const owner=new Map();for(const [source,model] of Object.entries(recording.beam_model||{}))for(const id of model.segments)owner.set(id,source);for(const part of recording.beam_elements||[]){const group=new THREE.Group();group.name=part.id;group.userData.part=owner.get(part.id);for(const shape of part.geometry)group.add(meshShape(shape,part.color,part.kind));setPose(group,part.pose);group.visible=false;objects.add(group);beamObjects.set(part.id,group);partObjects.set(part.id,group);}}
function showFrame(index){const frame=state.recording?.frames[index];if(!frame)return;state.frame=index;for(const [id,pose] of Object.entries(frame.parts)){if(partObjects.has(id))setPose(partObjects.get(id),pose);}for(const [source,model] of Object.entries(state.recording.beam_model||{})){const status=frame.beam_status?.[source],color=statusColor(status?.deflection_mm||0,status?.yielded||status?.possible_fracture);const original=partObjects.get(source);if(original)original.visible=false;for(const id of model.segments){const segment=beamObjects.get(id);if(segment){segment.visible=true;tintPart(segment,color);}}}$('#time-slider').value=index;$('#time-label').textContent=frame.time_s.toFixed(2)+' s';ports.visible=false;}
async function restore(){state.playing=false;$('#play-button').textContent='▶';for(const p of state.scene.parts)setPose(partObjects.get(p.id),p.pose);for(const o of partObjects.values())o.visible=true;for(const o of beamObjects.values())o.visible=false;state.frame=0;$('#time-slider').value=0;$('#time-label').textContent='0.00 s';ports.visible=true;applyStructuralColors();updatePorts();}

function modal(title,content,actions=[]){$('#modal-title').textContent=title;$('#modal-content').innerHTML=content;$('#modal-actions').replaceChildren();for(const a of actions){const b=document.createElement('button');b.textContent=a.label;b.className='button'+(a.primary?' primary':'');b.onclick=async()=>{b.disabled=true;try{await a.action();}catch(e){toast(e.message,true);}finally{b.disabled=false;}};$('#modal-actions').appendChild(b);}$('#modal').showModal();}
function closeModal(){$('#modal').close();const cleanup=reviewCleanup;reviewCleanup=null;cleanup?.();}
function canOpenDesign(){
  if(!token){toast('Wait for the workspace to finish loading.');return false;}
  if(state.busy||state.placementPending||dragStart){toast('Finish the current operation before opening a design.');return false;}
  return true;
}
function activateDesign(data){
  closeModal();
  state.doc=data.document;state.path=data.path;state.library=data.library;
  state.selected=null;state.connectionSource=null;state.undo=[];state.redo=[];objectModes.clear();openObjects.clear();
  state.dirty=!!(data.imported||data.autosaved||data.auto_symmetry);state.mirrorPoseError=data.mirror_pose_error||null;state.revision++;state.playing=false;state.frame=0;state.step=0;
  state.checks=null;state.analysis=null;state.recording=null;state.plan=null;state.simulationOptions=null;state.simulationError=null;
  $('#time-slider').value=0;$('#time-slider').max=100;$('#time-label').textContent='0.00 s';
  $('#timeline-mid').textContent='1.5 s';$('#timeline-end').textContent='3 s';$('#play-button').textContent='▶';
  clearSnapPreview();setTool(preferences.defaultTool);adoptLoadedMirrorModes(data.scene);buildScene(data.scene);renderLibrary();renderOutline();setMode('design');updateHeader();if(preferences.fitOnOpen)fitView();scheduleAutosave();
  status(data.autosaved?'Recovered autosave · Save to keep this version':data.imported?'Opened file · Save to keep it in your workspace':'Loaded '+data.path);
  if(state.mirrorPoseError)toast(state.mirrorPoseError,true);
}
function confirmDesignOpen(data){
  if(!state.dirty){activateDesign(data);return;}
  modal('Save changes before opening?',`<p><strong>${esc(state.doc.name||'Untitled creation')}</strong> has unsaved changes. Save them before opening <strong>${esc(data.document.name||data.path)}</strong>?</p>`,[
    {label:'Cancel',action:closeModal},
    {label:'Open without saving',action:()=>activateDesign(data)},
    {label:'Save and open',primary:true,action:()=>saveDialog(()=>data.imported?activateDesign(data):openWorkspaceDesign(data.path))}
  ]);
}
async function openDesign(readDesign,label){
  if(!canOpenDesign())return;
  closeModal();
  modal('Opening design',`<p id="opening-status" role="status">Reading ${esc(label)}…</p>`,[{label:'Cancel',action:closeModal}]);
  const message=$('#opening-status');
  try{
    const data=await readDesign();
    // A closed or replaced dialog cancels the load, including a late response.
    if(!message.isConnected||!$('#modal').open)return;
    confirmDesignOpen(data);
  }catch(error){
    if(!message.isConnected||!$('#modal').open)return;
    $('#modal-title').textContent='Could not open design';message.className='file-error';
    message.textContent=error.message+'\nYour current design has been kept.';
  }
}
function openWorkspaceDesign(path){
  return openDesign(async()=>{
    const response=await fetch('/api/open?path='+encodeURIComponent(path)),data=await response.json();
    if(!response.ok)throw new Error(data.error||'Could not read the design');return data;
  },path);
}
function openAutosave(path){
  return openDesign(async()=>{
    const response=await fetch('/api/autosave?directory='+encodeURIComponent(preferences.autosaveDirectory)+'&path='+encodeURIComponent(path));
    const data=await response.json();if(!response.ok)throw new Error(data.error||'Could not read the autosave');return data;
  },path);
}
async function loadDialog(){
  if(!canOpenDesign())return;
  closeModal();
  modal('Load design','<button id="open-file-button" class="option-row">Open file…<small>Choose a YAML or JSON design from your computer · Ctrl+O</small></button><p>You can also drop a design file anywhere in the editor.</p><div class="single-field"><label for="design-search">SAVED IN DESIGNS/</label><input id="design-search" type="search" placeholder="Find a saved design" disabled></div><div id="saved-design-list" aria-live="polite"><p>Loading saved designs…</p></div><h3>RECOVERY COPIES</h3><div id="autosave-list" aria-live="polite"><p>Loading autosaves…</p></div>',[{label:'Close',action:closeModal}]);
  $('#open-file-button').onclick=chooseDesignFile;
  const list=$('#saved-design-list'),search=$('#design-search');
  try{
    const response=await fetch('/api/designs'),data=await response.json();
    if(!list.isConnected||!$('#modal').open)return;
    if(!response.ok)throw new Error(data.error||'Could not list saved designs');
    const render=()=>{
      const matches=data.designs.filter(file=>file.path.toLowerCase().includes(search.value.toLowerCase()));
      list.innerHTML=matches.length?matches.map(file=>`<button class="option-row" data-open="${esc(file.path)}">${esc(file.path.slice('designs/'.length))}<small>${esc(file.path)} · ${Math.max(1,Math.ceil(file.size_bytes/1024))} KB</small></button>`).join(''):
        `<p>${data.designs.length?'No designs match your search.':'No saved designs yet. Save a design into designs/ to find it here.'}</p>`;
      list.querySelectorAll('[data-open]').forEach(button=>button.onclick=()=>openWorkspaceDesign(button.dataset.open));
    };
    search.disabled=false;search.oninput=render;render();
  }catch(error){if(list.isConnected){list.innerHTML='<p class="file-error" role="alert"></p>';list.firstChild.textContent=error.message;}}
  const recover=$('#autosave-list');
  try{
    const response=await fetch('/api/autosaves?directory='+encodeURIComponent(preferences.autosaveDirectory));
    if(!recover.isConnected||!$('#modal').open)return;
    if(!response.ok)throw new Error('Could not list autosaves');
    const records=(await response.json()).autosaves;
    recover.innerHTML=records.length?records.map(record=>`<button class="option-row" data-autosave="${esc(record.path)}">${esc(record.source_path)}<small>${esc(new Date(record.saved_at*1000).toLocaleString())} · ${esc(record.path)}</small></button>`).join(''):'<p>No recovery copies in the configured folder.</p>';
    recover.querySelectorAll('[data-autosave]').forEach(button=>button.onclick=()=>openAutosave(button.dataset.autosave));
  }catch(error){if(recover.isConnected)recover.textContent=error.message;}
}
function chooseDesignFile(){if(canOpenDesign())$('#design-file').click();}
function openDesignFiles(files){
  if(files.length!==1){toast('Open one design file at a time.',true);return;}
  const file=files[0];
  if(!/\.(yaml|yml|json)$/i.test(file.name)){toast('Choose a PipeSim .yaml, .yml or .json design. Use Import a 3D part for meshes.',true);return;}
  return openDesign(async()=>{
    if(file.size>32*1024*1024)throw new Error('Design files must be smaller than 32 MiB.');
    const text=await file.text();
    return api('open-file',{document:undefined,filename:file.name,text});
  },file.name);
}
$('#design-file').onchange=event=>{
  const files=Array.from(event.target.files);event.target.value='';
  if(files.length)openDesignFiles(files);
};
// OS files are handled at document capture, before the toolbox's canvas drop.
// File contents are not available during dragover; inspect the advertised type.
let fileDragDepth=0;
const isFileDrag=event=>Array.from(event.dataTransfer?.types||[]).includes('Files')||event.dataTransfer?.files?.length>0;
const hideFileDrop=()=>{fileDragDepth=0;$('#design-drop-overlay').classList.add('hidden');};
document.addEventListener('dragenter',event=>{
  if(!isFileDrag(event))return;event.preventDefault();fileDragDepth++;
  $('#design-drop-overlay').classList.remove('hidden');
},true);
document.addEventListener('dragover',event=>{
  if(!isFileDrag(event))return;event.preventDefault();event.stopPropagation();event.dataTransfer.dropEffect='copy';
},true);
document.addEventListener('dragleave',event=>{
  if(!isFileDrag(event))return;event.preventDefault();
  if(--fileDragDepth<=0)hideFileDrop();
},true);
document.addEventListener('drop',event=>{
  hideFileDrop();if(!isFileDrag(event))return;
  event.preventDefault();event.stopPropagation();openDesignFiles(Array.from(event.dataTransfer.files||[]));
},true);
document.addEventListener('dragend',hideFileDrop,true);
window.addEventListener('blur',hideFileDrop);
function jsonDialog(title,data){modal(title,'<textarea id="json-result" spellcheck="false" readonly></textarea>',[{label:'Close',action:closeModal}]);$('#json-result').value=JSON.stringify(data,null,2);}
function sourceDialog(){modal('Design source · YAML or JSON','<p>The file contains parts, readable poses, socket states, load cases and recorded results. JSON is also valid YAML.</p><textarea id="source-text" spellcheck="false" aria-label="Design source"></textarea>',[{label:'Cancel',action:closeModal},{label:'Apply design',primary:true,action:async()=>{const result=await api('parse',{text:$('#source-text').value});checkpoint();state.doc=result.document;state.library=result.library;renderLibrary();changed();buildScene(result.scene);renderInspector();renderOutline();closeModal();toast('Design source applied.');}}]);$('#source-text').value=JSON.stringify(state.doc,null,2);}
function libraryEditor(id,definition){if(!id&&definition&&state.doc.parts.find(p=>p.id===state.selected)?.body){const spec=state.doc.parts.find(p=>p.id===state.selected);modal('Edit rigid body','<p>Edit geometry, mass, inertia, material and attachment ports.</p><textarea id="body-source" spellcheck="false"></textarea>',[{label:'Cancel',action:closeModal},{label:'Apply body',primary:true,action:async()=>{const edited=JSON.parse($('#body-source').value);await mutate(()=>{spec.body=edited;});closeModal();}}]);$('#body-source').value=JSON.stringify(definition,null,2);return;}id=id||state.scene?.parts.find(p=>p.id===state.selected)?.catalog||Object.keys(state.library)[0];definition=definition||state.doc.definitions?.[id]||state.library[id];if(!definition){toast('Select a catalogue part first.');return;}modal('Edit library part','<p>Edit dimensions, geometry, mass, socket positions and supplier references. This design stores an explicit override of the catalogue entry.</p><div class="single-field"><label>CATALOGUE ID</label><input id="definition-id"></div><textarea id="definition-source" spellcheck="false" style="margin-top:15px"></textarea>',[{label:'Cancel',action:closeModal},{label:'Save part definition',primary:true,action:async()=>{const edited=JSON.parse($('#definition-source').value),key=$('#definition-id').value;await mutate(()=>{state.doc.definitions||={};state.doc.definitions[key]=edited;});state.library[key]=edited;renderLibrary();closeModal();}}]);$('#definition-id').value=id;$('#definition-source').value=JSON.stringify(definition,null,2);}
function saveDialog(afterSave=null){
  modal('Save design','<p>Files stay in your local PipeSim workspace.</p><div class="single-field"><label for="save-path">FILE PATH</label><input id="save-path" aria-label="Save file path"></div><p id="save-error" class="file-error" role="alert"></p>',[
    {label:'Cancel',action:closeModal},
    {label:'Save file',primary:true,action:async()=>{
      const path=$('#save-path').value.trim(),error=$('#save-error'),revision=state.revision,document=state.doc,source=state.path;
      try{
        const saved=await api('save',{path,source_path:state.path});
        if(state.revision!==revision||state.doc!==document||state.path!==source){toast('Saved '+saved.saved+'. Your newer edits are still open.');return;}
        state.doc=saved.document;state.path=saved.saved;state.dirty=false;clearTimeout(autosaveTimer);updateHeader();
        try{window.localStorage.setItem('pipesim.last-saved-path.v1',saved.saved);}catch{}
        toast('Saved '+saved.saved);
        if(error.isConnected&&$('#modal').open){closeModal();await afterSave?.();}
      }catch(e){error.textContent=e.message;}
    }}
  ]);
  $('#save-path').value=state.path;
}
function exportDialog(){modal('Export your creation','<button class="option-row" id="export-book-option">Illustrated build book<small>Printable instructions, bill of materials and a stock cutting plan</small></button><button class="option-row" id="export-image-option">Render an image<small>PNG from the current camera, with configurable lighting</small></button><button class="option-row" id="export-file-option">Download design file<small>Portable JSON with parts, constraints and recorded results</small></button>',[{label:'Close',action:closeModal}]);$('#export-book-option').onclick=()=>{closeModal();exportBuild();};$('#export-image-option').onclick=()=>{closeModal();renderDialog();};$('#export-file-option').onclick=()=>{const blob=new Blob([JSON.stringify(state.doc,null,2)],{type:'application/json'});const url=URL.createObjectURL(blob);const link=document.createElement('a');link.href=url;link.download=(state.doc.name||'design').replace(/[^a-z0-9-]/gi,'-')+'.pipe.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),10000);};}
async function exportBuild(){if(state.busy)return;state.busy=true;status('Generating the illustrated build book…');try{const result=await api('export',{engineering:true});modal('Build book ready',`<p>${result.steps} illustrated assembly steps, ${result.stock_bars} stock lengths, plus the parts list and engineering results.</p><a class="button primary" href="${esc(result.url)}" target="_blank">Open printable instructions ↗</a><p>Saved in ${esc(result.directory)}</p>`,[{label:'Done',action:closeModal}]);status('Build instructions exported');}catch(e){toast(e.message,true);}finally{state.busy=false;}}
function renderDialog(){modal('Render an image',`<div class="fields"><div class="field"><label>WIDTH px</label><input id="render-width" value="${preferences.renderWidth}" type="number"></div><div class="field"><label>HEIGHT px</label><input id="render-height" value="${preferences.renderHeight}" type="number"></div></div><div class="single-field"><label>LIGHTING</label><select id="render-light"><option>studio</option><option>technical</option><option>flat</option></select></div><div class="single-field"><label>BACKGROUND</label><select id="render-bg"><option value="#edf1f3">Soft grey</option><option value="#ffffff">White</option><option value="transparent">Transparent</option></select></div>`,[{label:'Cancel',action:closeModal},{label:'Render PNG',primary:true,action:async()=>{status('Rendering image…');const result=await api('render',{options:{width:+$('#render-width').value,height:+$('#render-height').value,eye:camera.position.toArray(),target:orbit.target.toArray(),lighting:$('#render-light').value,background:$('#render-bg').value}});closeModal();modal('Image ready',`<a href="${esc(result.url)}" target="_blank"><img src="${esc(result.url)}" style="width:100%" alt="Rendered pipe creation"></a><p>Open the image to save it at full resolution.</p>`,[{label:'Done',action:closeModal}]);status('Image exported');}}]);$('#render-light').value=preferences.renderLighting;$('#render-bg').value=preferences.renderBackground;}
function humanDialog(){
  modal('Add a human model',`<p>A configurable 19-part mannequin with articulated spine, neck, shoulders, arms, hands, hips, knees and ankles. Mass and stature estimate body thickness for fit checks.</p>
    <div class="fields"><div class="field"><label for="human-height">STATURE mm</label><input id="human-height" value="${preferences.defaultHumanHeightMm}" type="number"></div><div class="field"><label for="human-mass">MASS kg</label><input id="human-mass" value="${preferences.defaultHumanMassKg}" type="number"></div></div>
    <div class="single-field"><label for="human-pose">INITIAL POSE</label><select id="human-pose">${humanPoseOptions()}</select><p id="human-pose-description">${esc(humanPoseDescription('standing'))}</p></div>
    <div class="single-field"><label for="human-hold">POSTURE CONTROL</label><select id="human-hold">${HUMAN_POSTURES.filter(([id])=>id!=='custom').map(([id,label])=>`<option value="${id}">${label}</option>`).join('')}</select></div>
    <div class="fields"><div class="field"><label for="human-strength">STRENGTH SCALE</label><input id="human-strength" type="number" min="0.1" max="10" step="0.1" value="1"></div><div class="field"><label for="human-grip">GRIPPED BAR Ø mm</label><input id="human-grip" type="number" min="8" max="80" step="0.1" placeholder="Open hands"></div></div>
    <div class="single-field"><label for="human-damping">PASSIVE JOINT DAMPING · N·m·s/rad</label><input id="human-damping" type="number" min="0" step="0.1" value="0.08"></div>
    <p>Posture control applies finite joint torques. Grip geometry needs a joint to a bar to hold on; the pull-up examples include both hand connections.</p>`,[
    {label:'Cancel',action:closeModal},{label:'Add human',primary:true,action:async()=>{
      const parameters={stature_mm:+$('#human-height').value,mass_kg:+$('#human-mass').value,pose:$('#human-pose').value,strength_scale:+$('#human-strength').value,joint_damping_nms_rad:+$('#human-damping').value};
      setHumanPosture(parameters,$('#human-hold').value);if($('#human-grip').value!=='')parameters.grip_diameter_mm=+$('#human-grip').value;
      await mutate(()=>{state.doc.objects||=[];let i=1;while(state.doc.objects.some(o=>o.id==='human-'+i))i++;state.doc.objects.push({id:'human-'+i,template:'human',parameters,pose:{position_mm:[1000,0,0]}});});
      closeModal();fitView();toast('Added a 19-segment human. Use Source to calibrate individual measurements.');
    }}]);
  $('#human-pose').onchange=e=>{$('#human-pose-description').textContent=humanPoseDescription(e.target.value);};
}
let autosaveTimer=null,lastAutosavedRevision=-1;
async function performAutosave(){
  clearTimeout(autosaveTimer);autosaveTimer=null;
  if(!preferences.autosaveEnabled||!state.doc||!state.dirty||state.revision===lastAutosavedRevision)return;
  if(state.busy||state.placementPending||dragStart){scheduleAutosave();return;}
  const revision=state.revision;
  try{
    await api('autosave',{directory:preferences.autosaveDirectory,source_path:state.path,
      keep:preferences.autosaveKeep});
    lastAutosavedRevision=revision;
    status('Autosaved locally · original design unchanged');
  }catch(error){status('Autosave failed: '+error.message);}
  if(state.revision!==revision)scheduleAutosave();
}
function scheduleAutosave(){
  clearTimeout(autosaveTimer);autosaveTimer=null;
  if(!preferences.autosaveEnabled||!state.doc||!state.dirty||state.revision===lastAutosavedRevision)return;
  autosaveTimer=setTimeout(performAutosave,preferences.autosaveMinutes*60000);
}
function fitDialog(){const humans=state.scene.parts.filter(p=>p.kind==='human'&&p.id.endsWith('/pelvis')).map(p=>p.id.slice(0,-7));if(!humans.length){humanDialog();return;}modal('Human fit test',`<div class="single-field"><label>HUMAN</label><select id="fit-id">${humans.map(id=>`<option>${esc(id)}</option>`).join('')}</select></div><div class="single-field"><label>TEST</label><select id="fit-kind"><option value="reach">Right-hand reach</option><option value="seat">Seated dimensions</option></select></div><div class="fields">${['X','Y','Z'].map((v,i)=>`<div class="field"><label>TARGET ${v} mm</label><input id="fit-${i}" type="number" value="${[300,400,1200][i]}"></div>`).join('')}</div><div class="single-field"><label>SEAT PART (FOR SEATED TEST)</label><select id="fit-seat">${state.scene.parts.filter(p=>p.kind==='panel').map(p=>`<option>${esc(p.id)}</option>`).join('')}</select></div>`,[{label:'Cancel',action:closeModal},{label:'Run fit test',primary:true,action:async()=>{const params={human:$('#fit-id').value};if($('#fit-kind').value==='reach')params.target=[0,1,2].map(i=>+$('#fit-'+i).value);else params.seat=$('#fit-seat').value;const r=await api('fit',params);closeModal();if(r.parts)for(const [id,p] of Object.entries(r.parts))if(partObjects.has(id))setPose(partObjects.get(id),p);jsonDialog('Fit test result',r);}}]);}
function connectDialog(connector,port=null,replaceJoint=null){
  const old=state.doc.joints?.find(j=>j.id===replaceJoint);
  const member=state.scene.parts.find(p=>p.id===(old?.b.part||state.connectionSource)&&p.kind==='member');
  if(!member){toast('Select a tube, dowel or extrusion first, then click the target connector.');return;}
  const fitting=state.scene.parts.find(p=>p.id===connector);
  if(member.draft?(draftRun(state.doc,member.id)?.attachments||[]).some(a=>a.connector===connector):
    state.scene.joints.some(j=>j.id!==replaceJoint&&j.type==='socket'&&
      [j.a.part,j.b.part].includes(member.id)&&[j.a.part,j.b.part].includes(connector))){
    toast('This pipe is already attached to this connector.');return;
  }
  const sockets=Object.entries(fitting?.ports||{}).filter(([name,p])=>p.type==='socket'&&!socketOccupied(state.scene,connector,name,replaceJoint));
  if(!sockets.length)return;
  if(port&&socketOccupied(state.scene,connector,port,replaceJoint)){toast('This socket or its shared bore is occupied.');return;}
  if(member.draft){
    const run=draftRun(state.doc,member.id),open=['start','end'].filter(e=>!(run.attachments||[]).some(a=>a.end===e));
    const available=sockets.filter(([,socket])=>socket.through||open.length);
    if(!available.length){toast('Both draft pipe ends are occupied.');return;}
    if(!port)port=available[0][0];
    const attach=()=>{const chosen=$('#modal').open?$('#draft-port')?.value||port:port,socket=fitting.ports[chosen],end=$('#modal').open?$('#draft-end')?.value||open[0]:open[0];
      try{draftConnect({member:member.id,connector,port:chosen,...(socket.through?{}:{end})});closeModal();}catch(e){toast(e.message,true);}};
    if(available.length===1&&open.length<=1){attach();return;}
    modal('Connect draft run',`<p>Add the intended socket relation now. Exact fit and cut length are checked when you finalize.</p><div class="single-field"><label>SOCKET</label><select id="draft-port">${available.map(([name,s])=>`<option value="${esc(name)}" ${name===port?'selected':''}>${esc(s.label||name)}${s.through?' · through':''}</option>`).join('')}</select></div><div class="single-field"><label>PIPE END</label><select id="draft-end">${open.map(end=>`<option>${end}</option>`).join('')}</select></div>`,[{label:'Cancel',action:closeModal},{label:'Connect',primary:true,action:attach}]);
    $('#draft-end').disabled=!!fitting.ports[port].through;$('#draft-port').onchange=()=>{$('#draft-end').disabled=!!fitting.ports[$('#draft-port').value].through;};return;
  }
  const wasStation=old&&!fitting.ports[old.a.port]?.through&&old.insertion_mm>fitting.ports[old.a.port]?.engagement_mm;
  if(wasStation&&sockets.some(([name])=>name==='through'))port='through';
  if(!port)port=sockets.some(([name])=>name==='through')?'through':sockets[0][0];
  const occupiedEnds=new Set(state.scene.joints.filter(j=>j.id!==replaceJoint).flatMap(j=>[j.a,j.b]).filter(e=>e.part===member.id).map(e=>e.end));
  const matches=sockets.map(([name,socket])=>({member:member.id,connector,port:name,
    end:old?.b.end||(occupiedEnds.has('start')?'end':'start'),
    at_mm:old?.b.at_mm??(wasStation?old.insertion_mm:member.length_mm/2),
    insertion_mm:name===old?.a.port?old.insertion_mm:Math.min(30,socket.engagement_mm*.8),
    label:connector+' / '+(socket.label||name)+' · '+(socket.through?'continuous pipe':'pipe end')
  })).sort((a,b)=>(a.port===port?-1:b.port===port?1:0));
  connectionReview(matches,{},state.revision,null,replaceJoint);
}
async function expandObjects(id=null){const doc=await api('expand',typeof id==='string'?{object:id}:{});checkpoint();state.doc=doc;changed();await resolve();toast('Expanded into editable parts. Select any part and use Regroup as object to restore the object.');}

function placementContext(excluded=[]){
  const moving=new Set(excluded),bounds=new THREE.Box3(),references=[];
  for(const part of state.scene?.parts||[]){
    const object=partObjects.get(part.id);if(!object)continue;
    object.updateWorldMatrix(true,true);
    const box=new THREE.Box3().setFromObject(object);if(box.isEmpty())continue;
    bounds.union(box);
    if(moving.has(part.id))continue;
    if(part.kind==='member'&&part.length_mm>0){
      references.push({id:part.id,a:new THREE.Vector3(0,0,-part.length_mm/2).applyMatrix4(object.matrixWorld),
        b:new THREE.Vector3(0,0,part.length_mm/2).applyMatrix4(object.matrixWorld)});
    }else references.push({id:part.id,a:box.getCenter(new THREE.Vector3())});
  }
  if(bounds.isEmpty())bounds.setFromCenterAndSize(orbit.target,new THREE.Vector3(1000,1000,1000));
  // Leave room for intentional construction around the assembly, while
  // excluding ray/plane intersections many scene lengths away.
  const reach=Math.max(3000,bounds.getSize(new THREE.Vector3()).length()*3);
  return {references,bounds:bounds.expandByScalar(reach)};
}
function boundedPlacementDelta(drag,delta){
  if(!delta.toArray().every(Number.isFinite))return new THREE.Vector3();
  const origin=new THREE.Vector3().setFromMatrixPosition(drag.matrices.get(drag.id));
  const target=origin.clone().add(delta).clamp(drag.placement.bounds.min,drag.placement.bounds.max);
  target.z=Math.max(Math.min(0,origin.z),target.z);
  const result=target.sub(origin);
  return draftRun(state.doc,drag.id)?mirrorConstrainedDelta(drag.group,result):result;
}
function nearbyGeometryPoint(context,exact=true){
  const hit=exact&&raycaster.intersectObjects(objects.children,true).find(hit=>{
    let node=hit.object;while(node&&!node.userData.part)node=node.parent;
    return node&&context.references.some(ref=>ref.id===node.userData.part);
  });
  if(hit)return hit.point;
  const rect=renderer.domElement.getBoundingClientRect(),width=rect.width,height=rect.height;
  let best=null;
  for(const ref of context.references){
    const a=ref.a.clone().project(camera),b=(ref.b||ref.a).clone().project(camera);
    if(a.z< -1||a.z>1||b.z< -1||b.z>1)continue;
    const ax=(a.x-pointer.x)*width/2,ay=(a.y-pointer.y)*height/2;
    const dx=(b.x-a.x)*width/2,dy=(b.y-a.y)*height/2;
    const t=THREE.MathUtils.clamp(-(ax*dx+ay*dy)/(dx*dx+dy*dy||1),0,1);
    const distance=Math.hypot(ax+t*dx,ay+t*dy);
    if(distance>80||best&&distance>=best.distance)continue;
    let station=t;
    if(ref.b){
      const near=-ref.a.clone().applyMatrix4(camera.matrixWorldInverse).z;
      const far=-ref.b.clone().applyMatrix4(camera.matrixWorldInverse).z;
      if(near>0&&far>0)station=t*near/((1-t)*far+t*near);
    }
    best={distance,point:ref.b?ref.a.clone().lerp(ref.b,station):ref.a};
  }
  return best?.point||null;
}
function pointAtSceneDepth(reference){
  const plane=new THREE.Plane().setFromNormalAndCoplanarPoint(camera.getWorldDirection(new THREE.Vector3()),reference);
  return raycaster.ray.intersectPlane(plane,new THREE.Vector3());
}
function resolvedDropPoint(context){
  const reference=nearbyGeometryPoint(context);
  if(reference)return pointAtSceneDepth(reference)?.clamp(context.bounds.min,context.bounds.max)||null;
  const ground=raycaster.ray.intersectPlane(groundPlane,new THREE.Vector3());
  if(ground&&context.bounds.containsPoint(ground))return ground;
  const focus=context.bounds.getCenter(new THREE.Vector3());
  const point=pointAtSceneDepth(focus);
  if(!point)return null;
  point.clamp(context.bounds.min,context.bounds.max);point.z=Math.max(0,point.z);
  return point;
}
function ray(event){const rect=renderer.domElement.getBoundingClientRect();pointer.set((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1);camera.updateMatrixWorld();raycaster.setFromCamera(pointer,camera);}
function pickSocket(){
  const markers=ports.children.filter(p=>p.visible&&p.userData.portType==='socket');
  const direct=raycaster.intersectObjects(markers,false)[0];
  if(direct)return direct.object.userData;
  // Keep small socket markers usable when zoomed out: the pick tolerance is
  // measured in screen pixels, independently of the fitting's physical size.
  const rect=renderer.domElement.getBoundingClientRect();
  const nearby=markers.map(marker=>{
    const point=marker.position.clone().project(camera);
    return {marker,point,distance:Math.hypot((point.x-pointer.x)*rect.width/2,(point.y-pointer.y)*rect.height/2)};
  }).filter(p=>p.point.z>=-1&&p.point.z<=1&&p.distance<=10).sort((a,b)=>a.distance-b.distance||a.point.z-b.point.z);
  if(!nearby.length)return null;
  const target=nearby[0].marker.userData;
  // Closely overlapping openings on the same fitting need an explicit choice.
  if(nearby[1]?.marker.userData.part===target.part&&nearby[1].distance-nearby[0].distance<2)return {part:target.part};
  return target;
}
function pickPart(){
  const hit=raycaster.intersectObjects(objects.children,true).find(h=>{let node=h.object;while(node){if(!node.visible)return false;node=node.parent;}return true;});
  if(!hit)return null;let node=hit.object;while(node&&!node.userData.part)node=node.parent;
  return node?{id:node.userData.part,point:hit.point}:null;
}
function cancelPlacement(){
  const pointerId=pointerDrag?.pointerId;pointerDrag=null;
  if(pointerId!=null&&renderer.domElement.hasPointerCapture(pointerId))renderer.domElement.releasePointerCapture(pointerId);
  orbit.enabled=true;restorePlacement();
}
function pickResizeHandle(event){
  const hit=raycaster.intersectObjects(resizeHandles.children,false)[0]?.object;
  if(hit)return hit;
  const rect=renderer.domElement.getBoundingClientRect();
  const nearby=resizeHandles.children.map(handle=>{const point=handle.position.clone().project(camera);
    return {handle,distance:Math.hypot((point.x+1)*rect.width/2+rect.left-event.clientX,
      (1-point.y)*rect.height/2+rect.top-event.clientY)};}).sort((a,b)=>a.distance-b.distance);
  return nearby[0]?.distance<18?nearby[0].handle:null;
}
function clearResizeDrag(){
  const drag=resizeDrag;if(!drag)return;resizeDrag=null;orbit.enabled=true;
  const canvas=renderer.domElement;if(canvas.hasPointerCapture(drag.pointerId))canvas.releasePointerCapture(drag.pointerId);
  canvas.style.cursor='';
  const mesh=partObjects.get(drag.target.id);if(mesh&&state.scene){const part=state.scene.parts.find(p=>p.id===drag.target.id);
    if(part){setPose(mesh,part.pose);mesh.scale.set(1,1,1);}}
  dispose(resizePreview);updateResizeHandles();
}
if(renderer){let down=null;
  renderer.domElement.addEventListener('pointerdown',e=>{
    down=[e.clientX,e.clientY];if(e.button!==0||state.placementPending||state.mode!=='design')return;
    ray(e);
    const handle=resizeHandles.children.length?pickResizeHandle(e):null;
    if(handle){
      const target=resizeTarget(),side=handle.userData.resizeSide;
      if(!target)return;
      const moving=(side==='start'?target.first:target.last).clone(),fixed=(side==='start'?target.last:target.first).clone();
      const plane=new THREE.Plane().setFromNormalAndCoplanarPoint(camera.getWorldDirection(new THREE.Vector3()),moving);
      resizeDrag={pointerId:e.pointerId,target,side,moving,fixed,outward:moving.clone().sub(fixed).normalize(),
        plane,grab:raycaster.ray.intersectPlane(plane,new THREE.Vector3())||moving.clone(),
        length:target.length,shift:e.shiftKey,handle};
      orbit.enabled=false;renderer.domElement.style.cursor='grabbing';renderer.domElement.setPointerCapture(e.pointerId);down=null;e.stopImmediatePropagation();return;
    }
    if(!['select','translate'].includes(state.tool)||gizmo.axis)return;
    const hit=pickPart();if(!hit)return;
    select(hit.id);orbit.enabled=false;
    pointerDrag={pointerId:e.pointerId,x:e.clientX,y:e.clientY,point:hit.point,
      plane:new THREE.Plane().setFromNormalAndCoplanarPoint(camera.getWorldDirection(new THREE.Vector3()),hit.point),started:false};
    renderer.domElement.setPointerCapture(e.pointerId);e.stopImmediatePropagation();
  },true);
  renderer.domElement.addEventListener('pointermove',e=>{
    if(resizeDrag&&resizeDrag.pointerId===e.pointerId){
      e.stopImmediatePropagation();ray(e);
      const point=raycaster.ray.intersectPlane(resizeDrag.plane,new THREE.Vector3());if(!point)return;
      const drag=resizeDrag,delta=point.sub(drag.grab).dot(drag.outward);
      drag.length=Math.max(1,drag.target.length+delta*(drag.target.centered?2:1));drag.shift=e.shiftKey;
      const change=drag.length-drag.target.length;
      const endpoint=drag.moving.clone().addScaledVector(drag.outward,drag.target.centered?change/2:change);
      const opposite=drag.target.centered?drag.fixed.clone().addScaledVector(drag.outward,-change/2):drag.fixed;
      drag.handle.position.copy(endpoint);
      if(drag.target.centered)resizeHandles.children.find(handle=>handle!==drag.handle)?.position.copy(opposite);
      if(!drag.target.chain){const mesh=partObjects.get(drag.target.id);
        mesh.scale.z=drag.length/drag.target.length;mesh.position.copy(opposite).add(endpoint).multiplyScalar(.5);mesh.updateMatrix();
      }else{dispose(resizePreview);const length=endpoint.distanceTo(drag.fixed),line=new THREE.Mesh(
        new THREE.CylinderGeometry(3,3,length,8),new THREE.MeshBasicMaterial({color:'#f4bb65',transparent:true,opacity:.8,depthTest:false}));
        line.position.copy(drag.fixed).add(endpoint).multiplyScalar(.5);line.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),drag.outward);line.renderOrder=125;resizePreview.add(line);}
      const behavior=drag.shift?preferences.resizeShiftMode:preferences.resizeConnectorMode;
      status(`Resize ${drag.target.id}: ${drag.length.toFixed(1)} mm · connectors ${behavior==='follow'?'follow':'stay put'}`);return;
    }
    if(!pointerDrag&&!dragStart&&resizeHandles.children.length){
      ray(e);renderer.domElement.style.cursor=pickResizeHandle(e)?'grab':'';
      if(state.tool==='resize')return;
    }
    if(!pointerDrag||pointerDrag.pointerId!==e.pointerId)return;e.stopImmediatePropagation();
    if(!pointerDrag.started){if(Math.hypot(e.clientX-pointerDrag.x,e.clientY-pointerDrag.y)<4)return;
      if(!beginPlacement('pointer')){cancelPlacement();return;}pointerDrag.started=true;pointerDrag.context=dragStart.placement;gizmo.detach();}
    ray(e);const reference=nearbyGeometryPoint(pointerDrag.context,false);
    const point=reference?pointAtSceneDepth(reference):raycaster.ray.intersectPlane(pointerDrag.plane,new THREE.Vector3());if(!point)return;
    let delta=point.sub(pointerDrag.point);if(state.snap)delta.divideScalar(state.snapSettings.translationMm).round().multiplyScalar(state.snapSettings.translationMm);
    delta=boundedPlacementDelta(dragStart,delta);
    for(const [id,matrix] of dragStart.matrices){const object=partObjects.get(id);matrix.decompose(object.position,object.quaternion,object.scale);object.position.add(delta);object.updateMatrix();}
    previewMovement();
  },true);
  renderer.domElement.addEventListener('pointerup',e=>{
    if(resizeDrag&&resizeDrag.pointerId===e.pointerId){const drag=resizeDrag,changed=Math.abs(drag.length-drag.target.length)>0.01;
      clearResizeDrag();down=null;e.stopImmediatePropagation();
      if(changed)requestResize(drag.target,drag.side,drag.length,e.shiftKey);return;}
    if(pointerDrag&&pointerDrag.pointerId===e.pointerId){const started=pointerDrag.started;pointerDrag=null;down=null;orbit.enabled=true;
      if(renderer.domElement.hasPointerCapture(e.pointerId))renderer.domElement.releasePointerCapture(e.pointerId);
      e.stopImmediatePropagation();if(started)finishPlacement();else attachGizmo();return;}
    if(state.placementPending||gizmo.dragging||!down||e.button!==0||Math.hypot(e.clientX-down[0],e.clientY-down[1])>5){down=null;return;}
    down=null;ray(e);
    if(state.tool==='connect'){const socket=pickSocket();if(socket){connectDialog(socket.part,socket.port);return;}}
    select(pickPart()?.id||null);
  },true);
  renderer.domElement.addEventListener('pointercancel',()=>{down=null;if(resizeDrag)clearResizeDrag();else cancelPlacement();});
  renderer.domElement.addEventListener('lostpointercapture',()=>{if(resizeDrag)clearResizeDrag();if(pointerDrag){down=null;cancelPlacement();}});
  renderer.domElement.addEventListener('dragover',e=>{e.preventDefault();e.dataTransfer.dropEffect='copy';});
  renderer.domElement.addEventListener('drop',async e=>{
    e.preventDefault();const catalog=e.dataTransfer.getData('application/pipesim-part');if(!catalog||state.placementPending)return;
    ray(e);const context=placementContext(),point=resolvedDropPoint(context);if(!point)return;
    if(state.snap)point.divideScalar(state.snapSettings.translationMm).round().multiplyScalar(state.snapSettings.translationMm);
    point.z=Math.max(0,point.z);
    if(state.library[catalog].kind==='chain'){chainDialog(catalog,point.toArray());return;}
    if(state.library[catalog].kind==='wheel'){wheelDialog(catalog,point.toArray());return;}
    try{const id=await addPart(catalog,point.toArray(),{origin:state.library[catalog].kind==='connector',quiet:true});
      const matches=dragCandidates([id]);if(matches.length)await offerConnection(matches,{},state.revision,false,id);else toast('Added '+id+'. Drag it onto a pipe or socket to connect.');
    }catch(error){toast(error.message,true);}
  });
}
$('#part-search').oninput=renderLibrary;$('#category').onchange=renderLibrary;$('#size-filter').onchange=renderLibrary;
$('#wheel-button').onclick=()=>wheelDialog();
$('#finalize-draft').onclick=()=>finalizeDrafts();
$('#connection-snap-button').onclick=()=>{state.connectionSnap=!state.connectionSnap;updateSnapControls();clearSnapPreview();status(state.connectionSnap?'Connection snapping enabled · drag onto a pipe or socket':'Connection snapping disabled');};
$('#snap-settings-button').onclick=snapSettingsDialog;
$('#modal').addEventListener('cancel',e=>{if(reviewCleanup){e.preventDefault();closeModal();}});
$$('[data-left]').forEach(b=>b.onclick=()=>{$$('[data-left]').forEach(x=>x.classList.toggle('active',x===b));$('#library-content').classList.toggle('hidden',b.dataset.left!=='library');$('#outline-content').classList.toggle('hidden',b.dataset.left!=='outline');});
$$('[data-mode]').forEach(b=>b.onclick=()=>setMode(b.dataset.mode));$$('[data-tool]').forEach(b=>b.onclick=()=>setTool(b.dataset.tool));
$('#modal-close').onclick=closeModal;$('#source-button').onclick=sourceDialog;$('#save-button').onclick=()=>saveDialog();$('#load-button').onclick=loadDialog;$('#export-button').onclick=exportDialog;$('#camera-button').onclick=renderDialog;$('#human-button').onclick=humanDialog;$('#chain-button').onclick=()=>chainDialog();$('#library-edit').onclick=()=>libraryEditor();$('#expand-button').onclick=()=>expandObjects().catch(e=>toast(e.message,true));
$('#rename').onclick=()=>{modal('Name your creation','<div class="single-field"><label>DESIGN NAME</label><input id="design-name"></div>',[{label:'Cancel',action:closeModal},{label:'Rename',primary:true,action:async()=>{await mutate(()=>{state.doc.name=$('#design-name').value;});closeModal();}}]);$('#design-name').value=state.doc.name;};
$('#examples-button').onclick=()=>{
  if(!canOpenDesign())return;
  modal('Example designs',(state.examples||[]).map(path=>`<button class="option-row" data-open="${esc(path)}">${esc(path.split('/').pop().replace('.pipe.yaml','').replaceAll('-',' '))}<small>${esc(path)}</small></button>`).join(''),[{label:'Close',action:closeModal}]);
  $$('[data-open]').forEach(b=>b.onclick=()=>openWorkspaceDesign(b.dataset.open));
};
$('#new-button').onclick=async()=>{checkpoint();state.doc={format:'pipesim/1',units:'mm-kg-s-N-deg',name:'Untitled creation',parts:[],joints:[],anchors:[]};state.path='designs/untitled.pipe.yaml';state.selected=null;changed();await resolve();setMode('design');};
$('#undo').onclick=async()=>{if(!state.undo.length)return;state.redo.push(clone(state.doc));state.doc=state.undo.pop();changed();await resolve();};$('#redo').onclick=async()=>{if(!state.redo.length)return;state.undo.push(clone(state.doc));state.doc=state.redo.pop();changed();await resolve();};
$('#delete-part').onclick=()=>{const id=state.selected;if(!id)return;if(draftRun(state.doc,id))deleteDraft(id);else deleteTreeTarget({label:id,members:[id]});};
$('#fit-view').onclick=fitView;$('#grid-button').onclick=()=>{grid.visible=!grid.visible;$('#grid-button').classList.toggle('active',grid.visible);};$('#ports-button').onclick=()=>{state.ports=!state.ports;$('#ports-button').classList.toggle('active',state.ports);ports.visible=true;updatePorts();};$('#snap-button').onclick=()=>{state.snap=!state.snap;updateSnapControls();};
$$('[data-camera]').forEach(b=>b.onclick=()=>{const distance=camera.position.distanceTo(orbit.target);const vector=({top:new THREE.Vector3(.001,-.001,1),front:new THREE.Vector3(0,-1,.001),side:new THREE.Vector3(1,0,.001)})[b.dataset.camera];camera.position.copy(orbit.target).addScaledVector(vector.normalize(),distance);$('#view-title').textContent=b.dataset.camera[0].toUpperCase()+b.dataset.camera.slice(1)+' view';orbit.update();});
$('#play-button').onclick=()=>{if(!state.recording){setMode('simulate');toast('Run a simulation to record a motion timeline.');return;}state.playing=!state.playing;$('#play-button').textContent=state.playing?'Ⅱ':'▶';};$('#reset-button').onclick=restore;$('#time-slider').oninput=e=>{state.playing=false;showFrame(+e.target.value);};
$('#capture-frame').onclick=async()=>{if(!state.recording){toast('Record a simulation first.');return;}if(Object.keys(state.recording.beam_model||{}).length){toast('Bent beams are temporary simulation elements and cannot yet be captured as one straight design part.',true);return;}const frame=clone(state.recording.frames[state.frame]);const expanded=await api('snapshot',{frame});checkpoint();state.doc=expanded;changed();await resolve();toast('Captured this pose and adjusted the remaining joint limits. Compound human rotations may need anatomical review.');};
$('#import-button').onclick=()=>$('#mesh-file').click();$('#mesh-file').onchange=e=>{const file=e.target.files[0];if(!file)return;modal('Import '+file.name,'<p>Provide the physical mass and convert the mesh coordinates to millimetres. Hollow parts can use separate collision geometry in the library editor.</p><div class="fields"><div class="field"><label>MASS kg</label><input id="import-mass" type="number" value="1" min="0.001"></div><div class="field"><label>SCALE TO mm</label><input id="import-scale" type="number" value="1" min="0.001"></div></div>',[{label:'Cancel',action:closeModal},{label:'Import part',primary:true,action:async()=>{const buffer=new Uint8Array(await file.arrayBuffer());let binary='';for(let i=0;i<buffer.length;i+=32768)binary+=String.fromCharCode(...buffer.subarray(i,i+32768));const r=await api('import-mesh',{filename:file.name,data:btoa(binary),mass_kg:+$('#import-mass').value,scale:+$('#import-scale').value});checkpoint();state.doc=r.document;state.library=r.library;changed();renderLibrary();closeModal();await addPart(r.import.part);}}]);e.target.value='';};
document.addEventListener('keydown',e=>{if(e.key==='Shift'&&!localRotationHeld){localRotationHeld=true;if(!dragStart)attachGizmo();}});
document.addEventListener('keyup',e=>{if(e.key==='Shift'){localRotationHeld=false;if(!dragStart)attachGizmo();}});
window.addEventListener('blur',()=>{if(localRotationHeld){localRotationHeld=false;if(!dragStart)attachGizmo();}});
document.addEventListener('keydown',e=>{
  const key=e.key.toLowerCase();
  if((e.ctrlKey||e.metaKey)&&key==='o'){e.preventDefault();if(!$('#modal').open)chooseDesignFile();return;}
  if(e.key==='Escape'&&dragStart){e.preventDefault();cancelPlacement();setTool('select');return;}
  if(e.key==='Escape'&&resizeDrag){e.preventDefault();clearResizeDrag();return;}
  if(['INPUT','TEXTAREA','SELECT'].includes(document.activeElement?.tagName)||document.activeElement?.isContentEditable||$('#modal').open)return;
  if(state.placementPending&&(e.ctrlKey||e.metaKey))return;
  if((e.ctrlKey||e.metaKey)&&key==='z'){e.preventDefault();$('#undo').click();return;}
  if((e.ctrlKey||e.metaKey)&&key==='y'){e.preventDefault();$('#redo').click();return;}
  if((e.ctrlKey||e.metaKey)&&key==='s'){e.preventDefault();saveDialog();return;}
  if(e.altKey&&!e.ctrlKey&&!e.metaKey&&key==='d'){
    e.preventDefault();
    if(e.repeat||!state.selected||state.mode!=='design'||state.busy||dragStart)return;
    if(keyboardNudging)keyboardQueue.push({selected:state.selected,action:'duplicate'});
    else if(!state.placementPending)duplicateSelected();
    return;
  }
  const libraryHotkey=libraryHotkeyFromEvent(e);
  const catalog=libraryHotkey&&Object.keys(libraryHotkeys).find(id=>libraryHotkeys[id]===libraryHotkey&&state.library[id]);
  if(catalog){e.preventDefault();if(!e.repeat&&!state.busy&&!state.placementPending&&!dragStart)addLibraryCatalog(catalog);return;}
  if(!e.ctrlKey&&!e.metaKey&&!e.altKey&&!e.repeat&&state.mode==='design'){
    const shortcuts=[[preferences.resizeHandle1GrowKey,'start',1],[preferences.resizeHandle1ShrinkKey,'start',-1],
      [preferences.resizeHandle2GrowKey,'end',1],[preferences.resizeHandle2ShrinkKey,'end',-1]];
    const match=shortcuts.find(([letter])=>letter===key);
    if(match){const target=resizeTarget();if(target){e.preventDefault();
      const length=target.chain?nextFlexibleShortcutLength({requestedLengthMm:target.length,pitchMm:target.pitch,
        stepMm:preferences.resizeKeyStepMm,direction:match[2]}):target.length+match[2]*preferences.resizeKeyStepMm;
      if(length!==null)requestResize(target,match[1],length,e.shiftKey);return;}}
  }
  if(queueKeyboardNudge(e)){e.preventDefault();return;}
  if(state.placementPending)return;
  if(e.key==='Home'||key==='h')fitView();
  else if(key==='g')setTool('translate');else if(key==='r')setTool('rotate');
  else if(key==='q'||e.key==='Escape')setTool('select');
  else if(e.key==='Delete')$('#delete-part').click();
  else if(e.key==='/'){e.preventDefault();$('#part-search').focus();}
});
window.addEventListener('beforeunload',e=>{if(state.dirty){e.preventDefault();e.returnValue='';}});
let last=performance.now(),accumulator=0;
function tick(now){requestAnimationFrame(tick);const dt=Math.min((now-last)/1000,.1);last=now;if(state.playing&&state.recording){accumulator+=dt;const step=1/(state.recording.fps||30);if(accumulator>=step){showFrame((state.frame+1)%state.recording.frames.length);accumulator%=step;}$('#play-button').textContent='Ⅱ';}orbit?.update();syncMirrorPreviews();if(state.selected&&partObjects.has(state.selected)){const pos=partObjects.get(state.selected).position.clone().project(camera);const rect=viewport.getBoundingClientRect();const label=$('#selection-label');label.style.left=((pos.x+1)*rect.width/2+15)+'px';label.style.top=((1-pos.y)*rect.height/2+48)+'px';}positionResizeLabels();renderer?.render(scene,camera);}requestAnimationFrame(tick);
async function startupDesign(bootstrap){
  if(preferences.startup==='empty'){
    const document={format:'pipesim/1',units:'mm-kg-s-N-deg',name:'Untitled creation',parts:[],joints:[],anchors:[]};
    const path='designs/untitled.pipe.yaml';
    const result=await api('resolve',{document,path});
    return {...bootstrap,document,path,scene:result,auto_symmetry:false,mirror_pose_error:null};
  }
  if(preferences.startup==='autosave'){
    try{
      const list=await fetch('/api/autosaves?directory='+encodeURIComponent(preferences.autosaveDirectory));
      if(list.ok){const records=(await list.json()).autosaves;if(records.length){
        const response=await fetch('/api/autosave?directory='+encodeURIComponent(preferences.autosaveDirectory)+'&path='+encodeURIComponent(records[0].path));
        if(response.ok)return {...bootstrap,...await response.json()};
      }}
    }catch{}
  }
  let path=preferences.startup==='example'?preferences.startupExample||bootstrap.examples?.[0]:null;
  if(preferences.startup==='workspace')try{path=window.localStorage.getItem('pipesim.last-saved-path.v1');}catch{}
  if(path&&path!==bootstrap.path){
    const response=await fetch('/api/open?path='+encodeURIComponent(path));
    if(response.ok)return {...bootstrap,...await response.json()};
  }
  return bootstrap;
}
try{const response=await fetch('/api/bootstrap'),bootstrap=await response.json();if(!response.ok)throw new Error(bootstrap.error);token=bootstrap.token;serverApiVersion=bootstrap.api_version;const data=await startupDesign(bootstrap);state.doc=data.document;state.path=data.path;state.library={...data.library,...state.doc.definitions};state.examples=bootstrap.examples;state.dirty=!!(data.autosaved||data.auto_symmetry);state.mirrorPoseError=data.mirror_pose_error||null;state.selected=data.scene.parts.some(p=>p.id==='leg-1')?'leg-1':null;adoptLoadedMirrorModes(data.scene);buildScene(data.scene);renderLibrary();renderOutline();renderInspector();updateHeader();setTool(preferences.defaultTool);if(preferences.fitOnOpen)fitView();$('#grid-button').classList.toggle('active',grid.visible);$('#ports-button').classList.toggle('active',state.ports);if(state.dirty)scheduleAutosave();const notice=compatibilityMessage(serverApiVersion);if(notice){status(serverApiVersion>EDITOR_API_VERSION?'Editor page update required':'Editor server update required');toast(notice,true);}else status(data.autosaved?'Recovered autosave · Save to keep this version':'Workspace ready · all changes stay local');if(state.mirrorPoseError)toast(state.mirrorPoseError,true);}catch(e){toast(e.message,true);status('Workspace could not be opened');}

function nearbyPanelBoltHoles(panel){
  const box=panel.geometry.find(shape=>shape.type==='box'&&shape.size_mm?.length===3);
  const panelObject=partObjects.get(panel.id);
  if(!box||!panelObject)return [];
  const center=new THREE.Vector3(...(box.position_mm||[0,0,0]));
  const inverse=panelObject.matrix.clone().invert(),size=box.size_mm;
  const normal=new THREE.Vector3(0,0,1).transformDirection(panelObject.matrix);
  const occupied=new Map();
  for(const joint of state.scene.joints)for(const end of [joint.a,joint.b])if(end.port){
    const key=end.part+'/'+end.port;occupied.set(key,(occupied.get(key)||0)+1);
  }
  const holes=[];
  for(const fitting of state.scene.parts){
    if(fitting.id===panel.id||fitting.draft)continue;
    const object=partObjects.get(fitting.id);if(!object)continue;
    for(const [portName,port] of Object.entries(fitting.ports||{})){
      // Swivel fittings expose their bolt-sized through bore as an eye port.
      if(port.type!=='bolt'&&(port.type!=='eye'||!(port.diameter_mm>0)))continue;
      const used=occupied.get(fitting.id+'/'+portName)||0;
      if(used>=(port.capacity||1))continue;
      const world=new THREE.Vector3(...(port.position_mm||[0,0,0])).applyMatrix4(object.matrix);
      const local=world.clone().applyMatrix4(inverse).sub(center);
      if(Math.abs(local.x)>size[0]/2||Math.abs(local.y)>size[1]/2)continue;
      const gap=Math.max(0,Math.abs(local.z)-size[2]/2);
      if(gap>60)continue;
      const axis=new THREE.Vector3(...(port.axis||[0,0,1])).transformDirection(object.matrix);
      if(Math.abs(axis.dot(normal))<.8)continue;
      holes.push({fitting:fitting.id,port:portName,gap,local:world.applyMatrix4(inverse).toArray(),
        axis:axis.transformDirection(inverse).toArray()});
    }
  }
  return holes.sort((a,b)=>a.gap-b.gap||a.fitting.localeCompare(b.fitting)||a.port.localeCompare(b.port));
}

function fastenPanelDialog(panel){
  const padded=!!state.library[panel.catalog]?.panel_layers;
  const label=padded?'padded panel':'board';
  const holes=nearbyPanelBoltHoles(panel);
  if(!holes.length){modal('Fasten '+label,
    `<p>No unused, suitably oriented bolt holes lie under this ${label} within 60 mm of its ${padded?'backing':'mounting'} face. Position it over the flanges, then try again.</p>`,
    [{label:'Close',action:closeModal}]);return;}
  modal('Fasten '+label+' at bolt holes',`<p>Choose the flange holes for screws through ${esc(panel.id)}${padded?' backing':''}. Each selected hole becomes a fixed attachment; the ${label} stays where it is.</p>
    <div style="max-height:300px;overflow:auto">${holes.map((hole,i)=>`<label class="check-label"><input type="checkbox" data-panel-hole="${i}" checked>${esc(hole.fitting)} / ${esc(hole.port)} · board X ${hole.local[0].toFixed(1)}, Y ${hole.local[1].toFixed(1)} mm · gap ${hole.gap.toFixed(1)} mm</label>`).join('')}</div>
    <p>Drilling positions and fasteners are recorded; the ${label} mesh remains solid.</p>`,[
    {label:'Cancel',action:closeModal},
    {label:'Fasten selected holes',primary:true,action:async()=>{
      const selected=$$('[data-panel-hole]:checked').map(input=>holes[+input.dataset.panelHole]);
      if(!selected.length)throw new Error('Choose at least one bolt hole.');
      await mutate(()=>{
        state.doc.joints||=[];
        let number=1;
        for(const hole of selected){
          while(state.doc.joints.some(j=>j.id===`${panel.id}-screw-${number}`))number++;
          state.doc.joints.push({id:`${panel.id}-screw-${number++}`,type:'fixed',
            a:{part:hole.fitting,port:hole.port},
            b:{part:panel.id,frame:{position_mm:hole.local,axis:hole.axis}},
            metadata:{hardware:`Screw through ${panel.id}${padded?' backing':''} at local X ${hole.local[0].toFixed(1)}, Y ${hole.local[1].toFixed(1)} mm into ${hole.fitting}/${hole.port}; specify length for ${padded?'backing':'panel'} thickness and gap`}});
        }
      });
      closeModal();toast(`Fastened ${selected.length} ${selected.length===1?'hole':'holes'} on ${panel.id}.`);
    }}
  ]);
}

function jointCreateDialog(){
  const parts=state.scene.parts,selected=parts.find(part=>part.id===state.selected);
  if(!selected||parts.length<2){toast('Add a second part first.');return;}
  const usable=part=>Object.entries(part.ports||{}).filter(([name,port])=>port.type!=='socket'&&!socketOccupied(state.scene,part.id,name));
  const position=(part,port)=>new THREE.Vector3(...port.position_mm).applyMatrix4(partObjects.get(part.id).matrix);
  const complement=usable(selected).flatMap(([firstName,first])=>{
    if(!['eye','clevis'].includes(first.type)||first.assembly!=='bolt')return [];
    return parts.flatMap(part=>part.id===selected.id?[]:usable(part)
      .filter(([,port])=>['eye','clevis'].includes(port.type)&&port.type!==first.type&&port.assembly==='bolt'&&
        Math.abs((first.diameter_mm||0)-(port.diameter_mm||0))<=1)
      .map(([name,port])=>({part,name,firstName,distance:position(selected,first).distanceTo(position(part,port))})));
  }).sort((a,b)=>a.distance-b.distance)[0];
  const other=complement?.part||parts.find(part=>part.id!==selected.id);
  const options=parts.map(part=>`<option value="${esc(part.id)}">${esc(part.id)} · ${esc(part.label||part.catalog||part.kind)}</option>`).join('');
  const raw={id:'attachment-'+(state.doc.joints?.length||0),type:'fixed',
    a:{part:selected.id,frame:{position_mm:[0,0,0],axis:[0,0,1]}},
    b:{part:other.id,frame:{position_mm:[0,0,0],axis:[0,0,1]}},
    metadata:{hardware:'Specify the actual bolt, bracket or connector and its installation method'}};
  const revision=state.revision;let preview=null,request=0;
  modal('Add joint or attachment',`<p>Choose the parts and their attachment ports. The preview aligns the port centres and axes, including hinge bolt holes.</p>
    <div class="single-field"><label for="joint-part-a">FIRST PART</label><select id="joint-part-a">${options}</select></div>
    <div class="single-field"><label for="joint-port-a">ATTACHMENT PORT</label><select id="joint-port-a"></select></div>
    <div class="single-field"><label for="joint-part-b">SECOND PART</label><select id="joint-part-b">${options}</select></div>
    <div class="single-field"><label for="joint-port-b">ATTACHMENT PORT</label><select id="joint-port-b"></select></div>
    <div class="fields"><div class="field"><label for="joint-create-type">JOINT</label><select id="joint-create-type"><option value="revolute">Hinge · rotates</option><option value="fixed">Fixed</option><option value="spherical">Ball · rotates freely</option></select></div>
    <div class="field"><label for="joint-create-move">PARTS TO MOVE</label><select id="joint-create-move"><option value="auto">Automatic</option><option value="a">First connected body</option><option value="b">Second connected body</option></select></div></div>
    <p id="joint-create-status" role="status">Checking the selected ports…</p>
    <details><summary>Advanced joint JSON</summary><p>Use this for custom attachment frames or other joint types.</p><textarea id="joint-source" spellcheck="false"></textarea><button class="inspect-action secondary" id="joint-apply-json">Apply JSON joint</button></details>`,[
    {label:'Cancel',action:closeModal},
    {label:'Join selected ports',primary:true,action:async()=>{
      if(!preview||revision!==state.revision)throw new Error('Refresh the connection preview first.');
      const payload=values();const result=await api('connect-ports',payload);
      closeModal();await acceptPlacement(result,revision);toast('Joint created at the selected ports.');
    }}]);
  const join=$('#modal-actions .primary');join.disabled=true;
  $('#joint-part-a').value=selected.id;$('#joint-part-b').value=other.id;
  $('#joint-create-type').value=complement?'revolute':'fixed';
  $('#joint-source').value=JSON.stringify(raw,null,2);
  $('#joint-apply-json').onclick=async()=>{
    try{const edited=JSON.parse($('#joint-source').value);
      await mutate(()=>{state.doc.joints||=[];state.doc.joints.push(edited);});closeModal();
    }catch(error){toast(error.message,true);}
  };
  function fillPort(partField,portField,preferred){
    const part=parts.find(item=>item.id===$(partField).value),ports=part?usable(part):[];
    $(portField).innerHTML=ports.length?ports.map(([name,port])=>`<option value="${esc(name)}">${esc(name)} · ${esc(port.type)}</option>`).join(''):'<option value="">No free attachment ports</option>';
    if(ports.some(([name])=>name===preferred))$(portField).value=preferred;
    else if(ports.some(([name])=>name==='hinge'))$(portField).value='hinge';
  }
  function values(){return {a:{part:$('#joint-part-a').value,port:$('#joint-port-a').value},
    b:{part:$('#joint-part-b').value,port:$('#joint-port-b').value},
    type:$('#joint-create-type').value,move:$('#joint-create-move').value};}
  async function refresh(){
    const current=++request;preview=null;join.disabled=true;clearSnapPreview();
    const payload=values();if(!payload.a.port||!payload.b.port||payload.a.part===payload.b.part){
      $('#joint-create-status').textContent='Choose two different parts with free attachment ports.';return;}
    $('#joint-create-status').textContent='Checking fit and existing connections…';
    try{const result=await api('connect-ports',{...payload,preview:true});
      if(current!==request||revision!==state.revision)return;
      preview=result;join.disabled=false;showPosePreview(result.poses);
      $('#joint-create-status').textContent=`Ready: ${result.gap_mm.toFixed(1)} mm between ports. ${result.move?'The '+(result.move==='a'?'first':'second')+' connected body moves into place.':'Both bodies stay fixed.'}`;
    }catch(error){if(current===request)$('#joint-create-status').textContent=error.message;}
  }
  $('#joint-part-a').onchange=()=>{fillPort('#joint-part-a','#joint-port-a');refresh();};
  $('#joint-part-b').onchange=()=>{fillPort('#joint-part-b','#joint-port-b');refresh();};
  for(const id of ['#joint-port-a','#joint-port-b','#joint-create-type','#joint-create-move'])$(id).onchange=refresh;
  reviewCleanup=()=>{request++;clearSnapPreview();};
  fillPort('#joint-part-a','#joint-port-a',complement?.firstName);
  fillPort('#joint-part-b','#joint-port-b',complement?.name);
  refresh();
}
function jointDialog(id=null){
  const old=id?state.doc.joints?.find(j=>j.id===id):null;
  if(old?.type==='socket')return connectDialog(old.a.part,old.a.port,id);
  if(id&&!old){toast('Expand this object before editing its joints.');return;}
  if(!old)return jointCreateDialog();
  const others=state.scene.parts.filter(p=>p.id!==state.selected);
  const selected=state.scene.parts.find(p=>p.id===state.selected), other=others[0];
  const pivot=new THREE.Vector3(...selected.pose.position_mm), local=other?pivot.clone().applyMatrix4(partObjects.get(other.id).matrix.clone().invert()):pivot;
  const value=old||{id:'attachment-'+(state.doc.joints?.length||0),type:'fixed',a:{part:selected.id,frame:{position_mm:[0,0,0],axis:[0,0,1]}},b:{part:other.id,frame:{position_mm:local.toArray(),axis:[0,0,1]}},metadata:{hardware:'Specify the actual bolt, bracket or connector and its installation method'}};
  modal(old?'Edit connection':'Add connection','<p>Set the two part IDs and their local attachment frames or named ports. Choose fixed, revolute, prismatic, cylindrical, spherical or distance. Limits use mm and degrees; motor torque uses N·m.</p><textarea id="joint-source" spellcheck="false"></textarea>',[{label:'Cancel',action:closeModal},{label:'Apply connection',primary:true,action:async()=>{const edited=JSON.parse($('#joint-source').value);await mutate(()=>{state.doc.joints||=[];if(old)state.doc.joints[state.doc.joints.findIndex(j=>j.id===old.id)]=edited;else state.doc.joints.push(edited);});closeModal();}}]);
  $('#joint-source').value=JSON.stringify(value,null,2);
}
