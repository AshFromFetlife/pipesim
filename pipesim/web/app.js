import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {TransformControls} from 'three/addons/controls/TransformControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {OBJLoader} from 'three/addons/loaders/OBJLoader.js';
import {connectionCandidates,clearConnectionIntent,alignmentDelta,socketOccupied,rotationAlignment} from './snapping.js';
import {DEFAULT_SNAP_SETTINGS,loadSnapDefaults,saveSnapDefaults,validateSnapSettings} from './snap-settings.js';

const $=s=>document.querySelector(s), $$=s=>[...document.querySelectorAll(s)];
const clone=x=>structuredClone(x), esc=x=>String(x??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
let snapDefaults={...DEFAULT_SNAP_SETTINGS};try{snapDefaults=loadSnapDefaults(window.localStorage);}catch{}
const state={doc:null,path:'',library:{},scene:null,selected:null,connectionSource:null,mode:'design',tool:'select',undo:[],redo:[],dirty:false,revision:0,recording:null,playing:false,frame:0,checks:null,analysis:null,plan:null,step:0,busy:false,simulation:null,simulationError:null,simulationOptions:null,ports:false,snap:snapDefaults.gridEnabled,connectionSnap:snapDefaults.connectionsEnabled,snapSettings:snapDefaults,placementPending:false,moveBody:true};
let token='',toastTimer,renderer,orbit,gizmo,sceneGeneration=0,dragStart=null,pointerDrag=null;
const viewport=$('#viewport'), scene=new THREE.Scene(), objects=new THREE.Group(), ports=new THREE.Group(), overlays=new THREE.Group();
const snapGhost=new THREE.Group();let reviewCleanup=null,reviewVersion=0;
scene.add(objects,ports,overlays,snapGhost);scene.background=new THREE.Color('#eef3f5');
const camera=new THREE.PerspectiveCamera(38,1,1,100000);camera.up.set(0,0,1);camera.position.set(2100,-2600,1800);
const ambient=new THREE.HemisphereLight('#fbfeff','#aabac0',2.2);ambient.position.set(0,0,2000);scene.add(ambient);
const key=new THREE.DirectionalLight('#fff6e4',3.2);key.position.set(-1400,-2500,4000);key.castShadow=true;key.shadow.mapSize.set(2048,2048);key.shadow.camera.left=-3000;key.shadow.camera.right=3000;key.shadow.camera.top=3000;key.shadow.camera.bottom=-3000;key.shadow.camera.near=1;key.shadow.camera.far=12000;key.shadow.bias=-.0002;key.shadow.normalBias=1;scene.add(key);
const fill=new THREE.DirectionalLight('#e0edf7',1.3);fill.position.set(2300,1400,1800);scene.add(fill);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(30000,30000),new THREE.MeshStandardMaterial({color:'#f0f4f5',roughness:1,metalness:0}));floor.position.z=-1;floor.receiveShadow=true;scene.add(floor);
const grid=new THREE.GridHelper(12000,120,'#c6d3d8','#dce5e9');grid.rotation.x=Math.PI/2;grid.position.z=.1;grid.material.transparent=true;grid.material.opacity=.65;scene.add(grid);
function applyViewportTheme(){
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
const raycaster=new THREE.Raycaster(), pointer=new THREE.Vector2(), groundPlane=new THREE.Plane(new THREE.Vector3(0,0,1),0);
const partObjects=new Map();
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
const EDITOR_API_VERSION=13;
const SERVER_UPDATE_MESSAGE='The running editor server is out of date. Save your work, restart the PipeSim server, then refresh the page.';
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
      if(sessionResponse.ok&&session.token){token=session.token;continue;}
    }
    if(response.status===404&&data.error==='Unknown operation')throw new Error(SERVER_UPDATE_MESSAGE);
    throw new Error(data.error||'Operation failed');
  }
}
function checkpoint(){if(!state.doc)return;state.undo.push(clone(state.doc));if(state.undo.length>80)state.undo.shift();state.redo=[];}
function changed(){state.simulationError=null;state.doc.results={};delete state.doc.build_plan;state.dirty=true;state.revision++;state.recording=null;state.playing=false;state.checks=null;state.analysis=null;state.plan=null;state.frame=0;updateHeader();}
async function mutate(fn){checkpoint();try{fn();changed();await resolve();}catch(e){state.doc=state.undo.pop()||state.doc;toast(e.message,true);await resolve();throw e;}}
async function resolve(){const revision=state.revision;try{const data=await api('resolve');if(revision!==state.revision)return;buildScene(data);renderInspector();renderOutline();updateHeader();}catch(e){toast(e.message,true);status('Resolve error: '+e.message);throw e;}}
function updateHeader(){if(!state.doc)return;$('#rename').textContent=state.doc.name||'Untitled creation';$('#file-path').textContent=state.path;$('#dirty').classList.toggle('changed',state.dirty);$('#tree-count').textContent=state.scene?.parts.length||0;$('#part-stat').textContent=(state.scene?.parts.length||0)+' parts';$('#body-stat').textContent=(state.scene?.groups.length||0)+((state.scene?.groups.length||0)===1?' rigid body':' rigid bodies');$('#mass-stat').textContent=(state.scene?.parts.reduce((sum,p)=>sum+p.mass_kg,0)||0).toFixed(1)+' kg';$('#undo').disabled=!state.undo.length;$('#redo').disabled=!state.redo.length;}

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
function buildScene(data){
  state.scene=data;sceneGeneration++;gizmo?.detach();dispose(objects);dispose(ports);dispose(overlays);partObjects.clear();
  if(state.selected&&!data.parts.some(p=>p.id===state.selected)){
    const chain=data.chains?.find(c=>state.selected.startsWith(c.id+'/'));
    if(chain)state.selected=chain.end_part;
  }
  for(const part of data.parts){const group=new THREE.Group();group.name=part.id;group.userData.part=part.id;for(const shape of part.geometry)group.add(meshShape(shape,part.color,part.kind));setPose(group,part.pose);objects.add(group);partObjects.set(part.id,group);}
  for(const drive of state.doc.drives||[]){if(!drive.route_mm?.length)continue;const points=drive.route_mm.map(p=>new THREE.Vector3(...p));const curve=new THREE.CatmullRomCurve3(points,false,'catmullrom',0);const belt=new THREE.Mesh(new THREE.TubeGeometry(curve,80,2,6,false),new THREE.MeshStandardMaterial({color:'#374b52',roughness:.8}));overlays.add(belt);}
  for(const anchor of data.anchors){const part=partObjects.get(anchor.part);if(!part)continue;const loc=part.position.clone();const marker=new THREE.Mesh(new THREE.RingGeometry(70,73,48),new THREE.MeshBasicMaterial({color:'#74a28c',transparent:true,opacity:.55,side:THREE.DoubleSide}));marker.position.copy(loc);marker.position.z+=1;marker.quaternion.copy(part.quaternion);overlays.add(marker);}
  updatePorts();updateConnectionHint();highlightSelection();updateHeader();if(state.tool==='translate'||state.tool==='rotate')attachGizmo();
}
function updatePorts(){dispose(ports);if(!state.scene)return;const show=state.ports||state.tool==='connect';for(const p of state.scene.parts){if(!show&&p.id!==state.selected)continue;for(const [name,port] of Object.entries(p.ports)){const chain=state.scene.chains?.find(c=>p.id.startsWith(c.id+'/'));if(chain&&!((p.id===chain.start_part&&name===chain.start_port)||(p.id===chain.end_part&&name===chain.end_port)||(chain.layout_mode==='posable'&&p.id===state.selected)))continue;if(port.type!=='socket'&&!show)continue;const radius=port.type==='socket'?7:4;const sphere=new THREE.Mesh(new THREE.SphereGeometry(radius,12,8),new THREE.MeshBasicMaterial({color:port.type==='socket'?'#3c9c85':'#779eba',depthTest:false,transparent:true,opacity:.8}));const local=new THREE.Vector3(...(port.position_mm||[0,0,0]));const obj=partObjects.get(p.id);sphere.position.copy(local.applyMatrix4(obj.matrix));sphere.renderOrder=100;sphere.userData={port:name,part:p.id,portType:port.type};ports.add(sphere);const axis=new THREE.Vector3(...(port.axis||[0,0,1])).transformDirection(obj.matrix);const arrow=new THREE.ArrowHelper(axis,sphere.position,38,'#64a294',8,4);arrow.visible=show;ports.add(arrow);}}}
function highlightSelection(){
  const object=wholeObject();
  for(const [id,group] of partObjects){const active=object?id.startsWith(object.id+'/'):id===state.selected;group.traverse(o=>{if(o.isMesh&&o.material?.emissive){o.material.emissive.set(active?'#438f79':'#000000');o.material.emissiveIntensity=active?.12:0;}});}
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
  state.selected=id;updateConnectionHint();highlightSelection();updatePorts();renderInspector();renderOutline();attachGizmo();
}
function attachGizmo(){
  if(!gizmo)return;gizmo.detach();gizmo.showX=gizmo.showY=gizmo.showZ=true;gizmo.setSpace?.('world');
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
  state.connectionSource=tool==='connect'&&state.scene?.parts.some(p=>p.id===state.selected&&p.kind==='member')?state.selected:null;
  if(tool==='connect'){if(state.mode!=='design')setMode('design');ports.visible=true;}
  $$('[data-tool]').forEach(b=>b.classList.toggle('active',b.dataset.tool===tool));
  updateConnectionHint();updatePorts();attachGizmo();
}
function fitView(){if(!objects.children.length||!orbit)return;const box=new THREE.Box3().setFromObject(objects),center=box.getCenter(new THREE.Vector3()),size=box.getSize(new THREE.Vector3());const halfAngle=Math.atan(Math.tan(THREE.MathUtils.degToRad(camera.fov)/2)*Math.min(1,camera.aspect));const distance=Math.max(size.length()/2,150)/Math.sin(halfAngle)*1.16;const direction=camera.position.clone().sub(orbit.target).normalize();orbit.target.copy(center);camera.position.copy(center).addScaledVector(direction,distance);orbit.update();}
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
  else if(part.kind==='panel')p='<path d="M5 21L27 10L43 21L21 34Z" fill="#d4b991" stroke="#b59b78" stroke-width="2"/><path d="M5 21V27L21 40L43 27V21" stroke="#b59b78" stroke-width="2"/>';
  return base+p+'</svg>';
}
function renderLibrary(){const query=$('#part-search').value.toLowerCase(),category=$('#category').value,size=$('#size-filter').value;const entries=Object.entries(state.library).filter(([id,p])=>{
  if(!(id+' '+p.name).toLowerCase().includes(query))return false;
  if(category==='motion'){if(['member','panel','load'].includes(p.kind))return false;if(id.startsWith('tubeclamp.')&&!/173|138|140|148|179/.test(id))return false;}else if(category!=='all'&&p.kind!==category)return false;
  if(size!=='all'&&id.startsWith('tubeclamp.')&&!id.endsWith(size))return false;return true;
});$('#library-count').textContent=entries.length+' COMPONENTS';$('#part-list').innerHTML=entries.map(([id,p])=>`<button class="part-card" draggable="true" data-catalog="${esc(id)}" title="Add ${esc(p.name)}"><span class="part-thumb">${icon(p)}</span><span class="part-card-text"><div class="part-code">${esc(id.split('.').slice(1).join('.'))}</div><div class="part-name">${esc((p.name||id).replace(/^\d+[MF]?\s*-\s*/,'').split(' · ')[0].replace(/\s*\([^)]*\)/g,''))}</div><div class="part-vendor">${esc(p.source?.supplier||'CUSTOMISABLE')}</div></span><span class="part-plus">+</span></button>`).join('')||'<div class="empty-library">No matching parts.<br>Try another size or category.</div>';
  $$('.part-card').forEach(b=>{b.addEventListener('click',()=>state.library[b.dataset.catalog].kind==='chain'?chainDialog(b.dataset.catalog):addPart(b.dataset.catalog));b.addEventListener('dragstart',e=>{e.dataTransfer.setData('application/pipesim-part',b.dataset.catalog);e.dataTransfer.effectAllowed='copy';});});
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
  menu.innerHTML=`<div class="tree-menu-title">${esc(target.label)} · ${target.members.length} parts</div><button role="menuitem" data-tree-command="properties">Show properties</button><button role="menuitem" data-tree-command="rename">Rename…</button>${target.duplicate?`<button role="menuitem" data-tree-command="duplicate">Duplicate ${target.part?'part':'subassembly'}</button>`:''}<button role="menuitem" class="tree-delete" data-tree-command="delete">Delete ${target.part?'part':'subassembly'}</button>`;
  document.body.appendChild(menu);const rect=button.getBoundingClientRect();menu.style.left=Math.max(8,Math.min(rect.right-210,window.innerWidth-218))+'px';menu.style.top=Math.max(8,Math.min(rect.bottom+4,window.innerHeight-menu.offsetHeight-8))+'px';
  const items=[...menu.querySelectorAll('button')];items.forEach(item=>{item.disabled=state.busy||state.placementPending;item.onclick=()=>{
    closeTreeMenu();if(item.dataset.treeCommand==='rename'){renameTreeTarget(target);return;}if(item.dataset.treeCommand==='delete'){deleteTreeTarget(target);return;}
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
function renderOutline(){
  closeTreeMenu();if(!state.scene)return;
  const targets=[];const actions=target=>{targets.push(target);return treeActionsButton(targets.length-1,target.label);};
  const instances=state.doc.objects||[],direct=new Set(state.doc.parts.map(p=>p.id)),owned=new Set(),partButton=id=>{const label=state.scene.parts.find(p=>p.id===id)?.label||id;return `<div class="outline-heading"><button class="outline-part ${state.selected===id?'active':''}" data-select="${esc(id)}" title="${esc(id)}"><span>◇</span>${esc(label)}</button>${actions({label,part:id,members:[id],duplicate:true})}</div>`;};
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
  const bodies=state.scene.groups.map(g=>g.filter(id=>!owned.has(id))).filter(g=>g.length);
  $('#outline-list').innerHTML=grouped+bodies.map((group,i)=>{const label=state.doc.metadata?.body_labels?.find(r=>r.parts.length===group.length&&r.parts.every(p=>group.includes(p)))?.label||'Body '+String(i+1).padStart(2,'0');const target={label,members:group,duplicate:state.scene.groups.find(g=>g.includes(group[0])).length===group.length};return `<details class="outline-group" open><summary data-tree-group="${targets.length}">⌄ ${esc(label)} <span class="subtle"> · ${group.length} parts</span>${actions(target)}</summary>${group.map(partButton).join('')}</details>`;}).join('');
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
async function addPart(catalog,position=null,{origin=false,quiet=false}={}){const definition=state.library[catalog];if(!definition)return;const short=catalog.split('.').pop().toLowerCase().replace(/[^a-z0-9-]/g,'-');let n=1;while(state.doc.parts.some(p=>p.id===short+'-'+n))n++;const id=short+'-'+n;const params=clone(definition.parameters||{});const z=definition.kind==='member'?(params.length_mm||1000)/2:definition.kind==='panel'?(params.thickness_mm||30)/2:params.height_mm?params.height_mm/2:50;await mutate(()=>{state.doc.parts.push({id,catalog,parameters:params,pose:{position_mm:position?[position[0],position[1],position[2]+(origin?0:z)]:[0,-600,z],rotation_deg:[0,0,0]}});state.selected=id;});setMode('design');setTool('translate');if(!quiet)toast('Added '+id+'. Drag it onto a pipe or socket to connect.');return id;}

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
  if(state.placementPending){toast('Finish the connection preview before changing snap settings.');return;}
  const settings=currentSnapSettings();
  modal('Snap settings',`<label class="check-label"><input id="settings-grid" type="checkbox">Use position and angle increments</label><div class="fields snap-settings-fields"><div class="field"><label for="settings-position">POSITION · mm</label><input id="settings-position" type="number" min="0.1" max="10000" step="any"></div><div class="field"><label for="settings-angle">ROTATION · °</label><input id="settings-angle" type="number" min="0.1" max="180" step="any"></div></div><div class="snap-presets"><button data-snap-preset="90">Frame · 90°</button><button data-snap-preset="45">Brace · 45°</button><button data-snap-preset="15">Fine · 15°</button></div><label class="check-label"><input id="settings-align" type="checkbox">Align rotation with existing parts and world axes</label><div class="single-field"><label for="settings-alignment">ALIGNMENT WINDOW · °</label><input id="settings-alignment" type="number" min="0.1" max="20" step="any"></div><p>Nearby reference axes take priority over the rotation increment. Turn alignment off to use only your chosen increment.</p><label class="check-label"><input id="settings-connections" type="checkbox">Snap and connect on drop</label><div class="single-field"><label for="settings-capture">CONNECTION REACH ON SCREEN · px</label><input id="settings-capture" type="number" min="4" max="100" step="any"></div><p>Apply changes this session. Save as defaults also remembers these settings for future visits in this browser.</p>`,[
    {label:'Cancel',action:closeModal},
    {label:'Apply',action:()=>apply(false)},
    {label:'Save as defaults',primary:true,action:()=>apply(true)}
  ]);
  $('#settings-grid').checked=settings.gridEnabled;$('#settings-position').value=settings.translationMm;$('#settings-angle').value=settings.rotationDeg;
  $('#settings-align').checked=settings.alignEnabled;$('#settings-alignment').value=settings.alignmentDeg;
  $('#settings-connections').checked=settings.connectionsEnabled;$('#settings-capture').value=settings.connectionPixels;
  $$('[data-snap-preset]').forEach(button=>button.onclick=()=>{$('#settings-angle').value=button.dataset.snapPreset;$('#settings-grid').checked=true;});
  function apply(persist){
    for(const input of $$('#modal-content input[type="number"]'))if(!input.reportValidity())return;
    let next=validateSnapSettings({gridEnabled:$('#settings-grid').checked,translationMm:$('#settings-position').value,rotationDeg:$('#settings-angle').value,
      alignEnabled:$('#settings-align').checked,alignmentDeg:$('#settings-alignment').value,connectionsEnabled:$('#settings-connections').checked,connectionPixels:$('#settings-capture').value});
    if(persist)next=saveSnapDefaults(window.localStorage,next);
    state.snapSettings=next;state.snap=next.gridEnabled;state.connectionSnap=next.connectionsEnabled;updateSnapControls();renderInspector();clearSnapPreview();closeModal();
    toast(persist?'Snap defaults saved in this browser.':'Snap settings applied.');
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
function dragCandidates(group){const rect=viewport.getBoundingClientRect();return state.connectionSnap?connectionCandidates(state.scene,currentMatrices(),group,camera,rect.width,rect.height,currentSnapSettings()):[];}
let lastSnapTime=0;
function showDragSnap(){
  if(!dragStart||performance.now()-lastSnapTime<65)return;lastSnapTime=performance.now();
  const matches=dragStart.matches?.length?dragStart.matches:dragCandidates(dragStart.group);clearSnapPreview();if(!matches.length){if(dragStart.alignment){$('#snap-hint').textContent=dragStart.alignment;$('#snap-hint').classList.remove('hidden');}return;}
  const best=matches[0],side=dragStart.group.includes(best.connector)?'connector':'member';
  const id=side==='connector'?best.connector:best.member,group=state.scene.groups.find(g=>g.includes(id))||[id];
  const matrices=currentMatrices(),delta=alignmentDelta(state.scene,matrices,best,side),poses={};
  for(const part of group){const object=new THREE.Object3D();delta.clone().multiply(matrices.get(part)).decompose(object.position,object.quaternion,object.scale);poses[part]=getPose(object);}
  showPosePreview(poses);
  const marker=new THREE.Mesh(new THREE.SphereGeometry(10,12,8),new THREE.MeshBasicMaterial({color:'#48c6a0',depthTest:false}));marker.position.fromArray(best.point);marker.renderOrder=150;snapGhost.add(marker);
  $('#snap-hint').textContent=best.label+' · '+(clearConnectionIntent(matches)?'Release to connect':'Release to choose alignment');$('#snap-hint').classList.remove('hidden');
}
function beginPlacement(kind){
  if(state.placementPending||!state.selected)return false;
  const instance=wholeObject(),group=instance?state.scene.parts.filter(p=>p.id.startsWith(instance.id+'/')).map(p=>p.id):state.scene.groups.find(g=>g.includes(state.selected))||[state.selected];
  if(state.scene.anchors.some(a=>group.includes(a.part))){toast('This body is fixed to the world. Remove its anchor to move it.');return false;}
  const constrained=state.scene.joints.some(j=>group.includes(j.a.part)!==group.includes(j.b.part))||group.some(id=>!state.doc.parts.some(p=>p.id===id))||!!state.doc.state?.joints;
  dragStart={kind,id:state.selected,group,constrained,mode:state.tool==='rotate'?'rotate':'translate',revision:state.revision,
    objectId:instance?.id,objectPose:clone(instance?.pose||{}),
    handleMatrix:(instance?wholeObjectHandle:rotationHandle).matrix.clone(),matrices:new Map(group.map(id=>[id,partObjects.get(id).matrix.clone()]))};return true;
}
function drawMovement(drag){
  for(const part of state.scene.parts){const object=partObjects.get(part.id);if(object)setPose(object,drag.result?.poses[part.id]||part.pose);}
  updatePorts();showDragSnap();
}
function previewMovement(){
  const drag=dragStart;if(!drag)return;
  if(!drag.constrained){updatePorts();showDragSnap();return;}
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
  try{if(result.scene){buildScene(result.scene);renderOutline();renderInspector();}else await resolve();}
  finally{state.placementPending=false;attachGizmo();}
}
async function placeWithoutConnection(poses,revision,recordUndo=true){
  try{const result=await api('move',{poses});await acceptPlacement(result,revision,recordUndo);status('Placement updated');}
  catch(e){restorePlacement();toast(e.message,true);}
}
async function finishPlacement(){
  const drag=dragStart;if(!drag)return;
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
async function offerConnection(matches,poses={},revision=state.revision,recordUndo=true){
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
  modal(replaceJoint?'Edit socket connection':'Align and connect',`<div class="single-field"><label for="snap-target">CONNECTION</label><select id="snap-target">${matches.map((m,i)=>`<option value="${i}">${esc(m.label||m.connector+' / '+m.port)}</option>`).join('')}</select></div><div id="snap-settings"></div><div class="single-field"><label for="snap-move">PARTS TO MOVE</label><select id="snap-move" disabled></select></div><label class="check-label"><input id="snap-lock" type="checkbox" checked>Secure the screw</label><div id="snap-preview-view" aria-label="Connection alignment preview"></div><p id="snap-review-status" role="status">Checking available movement…</p>`,[
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
  const fitControls=document.createElement('div');fitControls.innerHTML=`<div class="single-field"><label for="snap-tolerance">CONNECTION TOLERANCE · mm</label><input id="snap-tolerance" type="number" min="0.01" max="20" step="any" value="${initial?.joint?.fit_tolerance_mm??2}"></div>
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
  const old=state.doc.joints?.find(j=>j.id===replaceJoint);if(old){$('#snap-lock').checked=!!old.locked;$('#snap-tolerance').value=old.fit_tolerance_mm??2;}
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
    <button id="duplicate-selected" class="inspect-action secondary" title="Make one copy of just the selected part">Duplicate</button>
    <button id="duplicate-menu-toggle" class="inspect-action secondary" aria-label="Duplicate options" aria-haspopup="menu" aria-controls="duplicate-menu" aria-expanded="false" title="More duplicate options">▾</button></div>
    <div id="duplicate-menu" class="duplicate-menu" role="menu" aria-label="Duplicate options" hidden>
      <button role="menuitem" data-duplicate="count" tabindex="-1">Duplicate N copies…<small>Choose a count and what to include</small></button>
      <button role="menuitem" data-duplicate="touching" tabindex="-1">Duplicate directly touching parts<small>Selected part and its immediate connections</small></button>
      <button role="menuitem" data-duplicate="subassembly" tabindex="-1">Duplicate entire subassembly<small>Locked connections, or the whole grouped object</small></button>
    </div>`;
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
function duplicateScopeInfo(scope){
  const selected=state.selected;
  if(scope==='part')return {count:1,description:'Just '+selected+'.'};
  if(scope==='touching'){
    const members=new Set([selected]);for(const joint of state.scene.joints)if([joint.a.part,joint.b.part].includes(selected)){members.add(joint.a.part);members.add(joint.b.part);}
    return {count:members.size,description:'The selected part and every part connected directly to it, including loose joints. Neighbours of those parts are excluded.'};
  }
  const instance=!state.doc.parts.some(p=>p.id===selected)&&selectedObject();
  if(instance)return {count:state.scene.parts.filter(p=>p.id.startsWith(instance.id+'/')&&!state.doc.parts.some(s=>s.id===p.id)).length,description:'The whole '+instance.id+' object, including its articulated joints.'};
  return {count:(state.scene.groups.find(g=>g.includes(selected))||[selected]).length,description:'Parts joined by locked connections. Loose and articulated joints mark the boundary.'};
}
function duplicateCountDialog(){
  if(state.busy||state.placementPending)return;
  let cancelled=false;
  modal('Duplicate N copies',`<form id="duplicate-form">
    <div class="single-field"><label for="duplicate-count">Number of new copies</label><input id="duplicate-count" type="number" min="1" max="100" step="1" value="2" required aria-describedby="duplicate-count-help"></div>
    <p id="duplicate-count-help">Enter a whole number from 1 to 100.</p>
    <div class="single-field"><label for="duplicate-scope">Include in each copy</label><select id="duplicate-scope"><option value="part">Selected part only</option><option value="touching">Part and directly touching parts</option><option value="subassembly">Entire subassembly</option></select></div>
    <p id="duplicate-scope-help"></p><p id="duplicate-summary" role="status"></p>
    <p>Copies keep their internal connections and are placed beside the original. World fixings and connections to other parts stay with the original.</p>
    </form>`,[{label:'Cancel',action:closeModal},{label:'Duplicate',primary:true,action:async()=>{
      if(!$('#duplicate-count').reportValidity())return;
      if(await duplicateSelected($('#duplicate-scope').value,Number($('#duplicate-count').value),()=>cancelled))closeModal();
    }}]);
  reviewCleanup=()=>{cancelled=true;};
  const update=()=>{const info=duplicateScopeInfo($('#duplicate-scope').value),input=$('#duplicate-count');
    $('#duplicate-scope-help').textContent=info.description;
    $('#duplicate-summary').textContent=input.checkValidity()?`${input.value} new ${Number(input.value)===1?'copy':'copies'} × ${info.count} ${info.count===1?'part':'parts'} = ${Number(input.value)*info.count} new parts. One Undo removes them all.`:'';
  };
  $('#duplicate-count').oninput=update;$('#duplicate-scope').onchange=update;
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
async function duplicateSelected(scope='part',count=1,cancelled=()=>false){
  if(state.busy||state.placementPending||!state.selected)return false;
  closeDuplicateMenu();const selected=state.selected,revision=state.revision;
  state.placementPending=true;gizmo?.detach();$$('.duplicate-control button, #duplicate-form input, #duplicate-form select').forEach(b=>b.disabled=true);
  try{
    const result=await api('duplicate',{selected,scope,count,grid_mm:state.snap?state.snapSettings.translationMm:1});
    if(cancelled()||revision!==state.revision)return false;
    state.selected=result.selected;await acceptPlacement(result,revision);
    setTool('translate');fitView();toast(`Created ${count} ${count===1?'copy':'copies'} · ${result.parts_per_copy*count} new ${result.parts_per_copy*count===1?'part':'parts'}.`);return true;
  }catch(error){if(!cancelled()&&revision===state.revision){state.selected=selected;toast(error.message,true);}return false;}
  finally{state.placementPending=false;select(state.selected);$$('#duplicate-form input, #duplicate-form select').forEach(b=>b.disabled=false);}
}
function renderInspector(){
  if(!state.doc)return;$('#inspector-title').textContent=({design:'PROPERTIES',check:'DESIGN CHECKS',simulate:'PHYSICS & MOTION',build:'ASSEMBLY PROCESS'})[state.mode];
  if(state.mode==='check')return renderChecks();if(state.mode==='simulate')return renderSimulation();if(state.mode==='build')return renderBuild();
  const p=state.scene?.parts.find(p=>p.id===state.selected);if(!p){$('#inspector').innerHTML='<div class="empty-state"><div class="empty-symbol">◇</div><h2>Make something useful.</h2><p>Pick a part to inspect its dimensions, position and connections.</p><p>Add components from the library,<br>or start with an example design.</p></div>';return;}
  const instance=(state.doc.objects||[]).find(o=>p.id.startsWith(o.id+'/'));
  if(instance&&!state.doc.parts.some(s=>s.id===p.id))return renderObjectInspector(instance,p);
  const spec=state.doc.parts.find(x=>x.id===p.id),definition=state.doc.definitions?.[p.catalog]||state.library[p.catalog]||spec?.body||{};
  const params={...definition.parameters,...spec?.parameters};const connections=state.scene.joints.filter(j=>[j.a.part,j.b.part].includes(p.id));const group=state.scene.groups.find(g=>g.includes(p.id))||[];const anchor=(state.doc.anchors||[]).find(a=>a.part===p.id);
  $('#inspector').innerHTML=`<div class="inspect-section"><div class="inspect-id">${esc(p.catalog||p.id)}</div><h2>${esc((p.label!==p.id&&p.label)||spec?.label||definition.name||p.id)}</h2><span class="chip">${esc(p.kind.toUpperCase())}</span><span class="chip gray">${p.mass_kg.toFixed(2)} kg</span></div><div class="inspect-section">${propertyFields('POSITION',p.pose.position_mm,'position_mm')}<div style="height:17px"></div>${propertyFields('ROTATION',p.pose.rotation_deg,'rotation_deg')}<label class="check-label"><input type="checkbox" id="move-body" ${state.moveBody?'checked':''}>Move the connected rigid body (${group.length})</label>${!spec?'<button class="inspect-action" id="expand-selected">Expand object to edit its parts</button>':''}</div><div class="inspect-section"><h3>DIMENSIONS & MATERIAL</h3>${Object.entries(params).filter(([,v])=>typeof v==='number').map(([k,v])=>`<div class="single-field"><label>${esc(k==='joint_damping_nms_rad'?'PASSIVE JOINT DAMPING · N·m·s/rad':k.replaceAll('_',' ').toUpperCase())}</label><input type="number" data-param="${esc(k)}" value="${v}" step="1" min="0.01"></div>`).join('')}<div class="property-row"><span>Material</span><strong>${esc(definition.material||'Custom body')}</strong></div><div class="property-row"><span>Mass</span><strong>${p.mass_kg.toFixed(3)} kg</strong></div>${p.kind==='member'?'<button class="inspect-action" id="connect-selected">Connect to a socket ⌘</button>':''}</div><div class="inspect-section"><button class="inspect-action secondary" id="new-joint">Add joint or attachment</button><h3>CONNECTIONS <span style="float:right">${connections.length}</span></h3>${connections.map(j=>`<div class="joint-card"><div class="joint-card-top"><span>${esc(j.a.part===p.id?j.b.part:j.a.part)}</span><label><input type="checkbox" data-lock="${esc(j.id)}" ${j.locked||j.type==='fixed'?'checked':''}>Locked</label></div><p>${esc(j.a.port||j.type)} → ${esc(j.b.at_mm!=null?Number(j.b.at_mm).toFixed(1)+' mm from pipe start':j.b.end||j.b.port||j.type)} ${j.insertion_mm?' · '+Number(j.insertion_mm).toFixed(1)+' mm insertion':''}</p><button class="subtle" data-joint-edit="${esc(j.id)}" style="padding:3px 9px 3px 0">Edit joint</button><button class="subtle" data-detach="${esc(j.id)}" style="padding:3px 0">Detach</button></div>`).join('')||'<p>No connections. This part moves independently.</p>'}<label class="check-label"><input id="anchor-check" type="checkbox" ${anchor?'checked':''}>Fixed to the world</label>${anchor?`<div class="single-field"><label>MOUNTING SURFACE</label><select id="anchor-surface">${['floor','wall','ceiling','fixture'].map(s=>`<option ${anchor.surface===s?'selected':''}>${s}</option>`).join('')}</select></div>`:''}</div><div class="inspect-section"><h3>PART REFERENCE</h3><p>${esc(p.source?.geometry_status||'User defined geometry')}</p>${p.source?.assumptions?'<p>'+esc(p.source.assumptions.join('. '))+'</p>':''}${/^https?:\/\//.test(p.source?.url||'')?`<a class="results-link" target="_blank" rel="noreferrer" href="${esc(p.source.url)}">Supplier specifications ↗</a>`:''}<button class="inspect-action secondary" id="edit-part-definition">Edit part definition</button></div>`;
  bindDuplicateButton();bindFloorButtons(p.id);$('#new-joint').onclick=()=>jointDialog();
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

const HUMAN_POSES=['standing','seated','crouching','pull-up'];
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
  const inside=id=>id.startsWith(instance.id+'/'),connections=state.scene.joints.filter(j=>inside(j.a.part)!==inside(j.b.part));
  const detached=(state.doc.metadata?.detached_attachments||[]).filter(r=>r.object===instance.id&&state.scene.parts.some(p=>p.id===r.joint.a.part)&&state.scene.parts.some(p=>p.id===r.joint.b.part));
  const anchors=state.scene.anchors.filter(a=>inside(a.part));
  const section=document.createElement('div');section.className='inspect-section';
  section.innerHTML=`<h3>CONNECTIONS TO STRUCTURE</h3><p>Attachments stay connected while posing. Detach grips or mounts to reposition the whole ${chain?'chain':'person'}.</p>
    ${connections.map(j=>`<div class="joint-card"><div class="joint-card-top">${esc(j.id)}</div><p>${esc(j.a.part)} ↔ ${esc(j.b.part)} · ${esc(j.type)}</p><button class="subtle" data-attachment-edit="${esc(j.id)}">Edit connection</button> <button class="subtle" data-attachment-detach="${esc(j.id)}">Detach</button></div>`).join('')}
    ${anchors.map(a=>`<div class="joint-card"><p>${esc(a.part)} · fixed to ${esc(a.surface||'world')}</p><button class="subtle" data-object-unanchor="${esc(a.part)}">Release world anchor</button></div>`).join('')}
    ${!connections.length&&!anchors.length?`<p>No external attachments. The whole ${chain?'chain':'person'} can move freely.</p>`:''}
    ${detached.map(r=>`<div class="joint-card"><p>${esc(r.joint.id)} · detached</p><button class="inspect-action secondary" data-attachment-reconnect="${esc(r.joint.id)}">Preview reconnect</button></div>`).join('')}
    <button class="inspect-action secondary" id="object-attach-part">${chain?'Attach chain link to structure':'Attach body part to structure'}</button>`;
  $('#object-expand').closest('.inspect-section').before(section);
  $$('[data-attachment-edit]').forEach(b=>b.onclick=()=>jointDialog(b.dataset.attachmentEdit));
  $$('[data-attachment-detach]').forEach(b=>b.onclick=()=>objectEdit('detach-attachment',{object:instance.id,joint:b.dataset.attachmentDetach}));
  $$('[data-attachment-reconnect]').forEach(b=>b.onclick=()=>attachmentDialog(instance,selectedPart,b.dataset.attachmentReconnect));
  $$('[data-object-unanchor]').forEach(b=>b.onclick=()=>objectEdit('release-object-anchor',{object:instance.id,part:b.dataset.objectUnanchor}));
  $('#object-attach-part').onclick=()=>attachmentDialog(instance,selectedPart);
}
function attachmentDialog(instance,selectedPart,reconnect=null){
  const chain=instance.template==='chain';
  if(state.busy||state.placementPending)return;
  const inside=p=>p.id.startsWith(instance.id+'/'),limbs=state.scene.parts.filter(inside),targets=state.scene.parts.filter(p=>!inside(p));
  if(!targets.length){toast('Add a bar or another structure part to attach to.');return;}
  let result=null,request=0;const revision=state.revision;
  state.placementPending=true;gizmo?.detach();
  modal(reconnect?'Reconnect body attachment':chain?'Attach chain to structure':'Attach body part to structure',reconnect?`<p>${esc(reconnect)}</p><p>Preview reaching the saved attachment point.</p><p id="attachment-preview-status" role="status"></p>`:`
    <div class="single-field"><label for="attachment-limb">${chain?'CHAIN LINK':'BODY PART'}</label><select id="attachment-limb">${limbs.map(p=>`<option value="${esc(p.id)}" ${p.id===selectedPart.id?'selected':''}>${esc(p.id)}</option>`).join('')}</select></div>
    ${chain?`<div class="single-field"><label for="attachment-source-port">LINK EYE</label><select id="attachment-source-port"><option value="b">Start eye</option><option value="a">End eye</option></select></div>`:''}
    <div class="single-field"><label for="attachment-target">STRUCTURE PART</label><select id="attachment-target">${targets.map(p=>`<option value="${esc(p.id)}">${esc(p.id)}</option>`).join('')}</select></div>
    <div id="attachment-location"></div><div class="single-field"><label for="attachment-kind">ATTACHMENT</label><select id="attachment-kind"><option value="revolute">Grip · pivots around the bar</option><option value="fixed">Fixed · holds position and orientation</option><option value="spherical">Ball joint · rotates freely</option></select></div>
    <p>The preview poses the connected ${chain?'chain':'limb'} to meet the attachment. Other attachments and joint limits remain active.</p><p id="attachment-preview-status" role="status"></p>`,[
    {label:'Cancel',action:closeModal},
    {label:'Connect',primary:true,action:async()=>{if(!result||revision!==state.revision)return;const accepted=result;closeModal();await acceptPlacement(accepted,revision);toast(chain?'Chain attached.':'Body part attached.');}}
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
      }else if($('#attachment-port')?.value)target.port=$('#attachment-port').value;
      else target.frame={position_mm:[0,0,0],axis:[0,0,1]};
      extra={...extra,part:$('#attachment-limb').value,target,type:$('#attachment-kind').value,...(chain?{part_port:$('#attachment-source-port').value}:{})};
    }
    try{
      const data=await api('attach-part',extra);
      if(current!==request||revision!==state.revision)return;
      result=data;showPosePreview(data.poses);
      $('#attachment-preview-status').textContent='Ready to connect. '+(data.moved.length?(chain?'The green preview shows the fitted chain.':'The green preview shows the new limb pose.'):'The attachment points already meet.');
      $('#modal-actions .primary').disabled=false;
    }catch(e){if(current===request)$('#attachment-preview-status').textContent=e.message;}
  }
  function location(){
    const target=state.scene.parts.find(p=>p.id===$('#attachment-target').value),limb=partObjects.get($('#attachment-limb').value);
    const options=Object.entries(target.ports||{}).filter(([,p])=>p.type!=='socket');
    const local=limb.position.clone().applyMatrix4(partObjects.get(target.id).matrix.clone().invert());
    $('#attachment-location').innerHTML=target.kind==='member'?`<div class="single-field"><label for="attachment-station">POINT FROM PIPE START · mm</label><input id="attachment-station" type="number" min="0" max="${target.length_mm}" step="any" value="${Math.min(target.length_mm,Math.max(0,local.z+target.length_mm/2)).toFixed(2)}"></div>`:options.length?`<div class="single-field"><label for="attachment-port">ATTACHMENT POINT</label><select id="attachment-port">${options.map(([id,p])=>`<option value="${esc(id)}">${esc(p.label||id)}</option>`).join('')}</select></div>`:'<p>Attach at the target part’s origin.</p>';
    $$('#attachment-location input, #attachment-location select').forEach(el=>el.onchange=preview);preview();
  }
  if(reconnect)preview();else{
    if(chain){$('#attachment-kind').value='spherical';$('#attachment-source-port').value=selectedPart.id===instance.id+'/link-1'?'b':'a';$('#attachment-source-port').onchange=preview;}
    $('#attachment-target').onchange=location;$('#attachment-limb').onchange=location;$('#attachment-kind').onchange=preview;
    const origin=partObjects.get(selectedPart.id).position;
    $('#attachment-target').value=[...targets].sort((a,b)=>partObjects.get(a.id).position.distanceTo(origin)-partObjects.get(b.id).position.distanceTo(origin))[0].id;
    location();
  }
}
function renderChainControls(instance,selectedPart){
  const info=state.scene.chains.find(c=>c.id===instance.id),section=document.createElement('div');section.className='inspect-section';
  section.innerHTML=`<h3>CHAIN LENGTH</h3><div class="single-field"><label for="chain-length">Set chain length · mm</label><input id="chain-length" type="number" required min="0.001" max="${info.pitch_mm*1000}" step="any" value="${info.requested_length_mm}"></div><p id="chain-length-summary">${info.count} links × ${info.pitch_mm} mm pitch = ${info.length_mm} mm. Length rounds up to whole links. Add or remove links at the end; the retained links keep their pose.</p><p>Move the chain as one object, or choose Pose links to shape it. Simulation always lets the links flex.</p><div class="object-modes"><button id="chain-select-start">Select start</button><button id="chain-select-end">Select end</button></div><div class="single-field"><label for="chain-link-index">Select link</label><input id="chain-link-index" type="number" min="1" max="${info.count}" step="1" value="${Number(selectedPart.id.split('/link-').pop())||1}"></div><div class="object-modes"><button id="chain-attach-start">Attach start</button><button id="chain-attach-end">Attach end</button></div>`;
  $('#object-expand').closest('.inspect-section').before(section);
  $('#chain-length').onchange=async e=>{if(!e.target.reportValidity())return;await objectEdit('object-parameters',{object:instance.id,parameters:{...instance.parameters,length_mm:+e.target.value}});};
  const selectEnd=end=>select(end==='start'?info.start_part:info.end_part);
  $('#chain-select-start').onclick=()=>selectEnd('start');$('#chain-select-end').onclick=()=>selectEnd('end');
  $('#chain-link-index').onchange=e=>{if(e.target.reportValidity())select(instance.id+'/link-'+e.target.value);};
  for(const end of ['start','end'])$('#chain-attach-'+end).onclick=()=>attachmentDialog(instance,state.scene.parts.find(p=>p.id===info[end+'_part']));
}

function chainDialog(catalog='generic.chain-link',position=null){
  if(state.busy||state.placementPending)return;
  const links=Object.entries(state.library).filter(([,p])=>p.kind==='chain'&&p.ports?.a&&p.ports?.b);
  modal('Add a chain',`<div class="single-field"><label for="chain-catalog">LINK TYPE</label><select id="chain-catalog">${links.map(([id,p])=>`<option value="${esc(id)}" ${id===catalog?'selected':''}>${esc(p.name||id)}</option>`).join('')}</select></div><div class="single-field"><label for="new-chain-length">Chain length · mm</label><input id="new-chain-length" type="number" required min="0.001" step="any" value="1000"></div><p id="new-chain-summary" role="status"></p><p>The links and their joints are created together. Move the whole chain for layout, then choose Pose links to shape it.</p>`,[
    {label:'Cancel',action:closeModal},{label:'Add chain',primary:true,action:async()=>{
      const input=$('#new-chain-length');if(!input.reportValidity())return;
      const parameters={length_mm:+input.value,link_catalog:$('#chain-catalog').value};
      let n=1;while(state.scene.parts.some(p=>p.id==='chain-'+n||p.id.startsWith('chain-'+n+'/'))||(state.doc.objects||[]).some(o=>o.id==='chain-'+n))n++;
      const id='chain-'+n,pitch=chainPitch(parameters.link_catalog),length=Math.ceil(parameters.length_mm/pitch)*pitch;
      await mutate(()=>{state.doc.objects||=[];state.doc.objects.push({id,template:'chain',parameters,pose:{position_mm:position?[position[0],position[1],position[2]+length+20]:[0,-600,length+100]}});state.selected=id+'/link-1';});
      closeModal();setMode('design');setTool('translate');fitView();toast('Added '+id+'. Set its length in Properties or choose Pose links.');
    }}]);
  function refresh(){const pitch=chainPitch($('#chain-catalog').value),length=+$('#new-chain-length').value,count=Math.max(1,Math.ceil(length/pitch));$('#new-chain-length').max=pitch*1000;$('#new-chain-summary').textContent=`${count} links × ${pitch} mm pitch = ${count*pitch} mm. Rounded up to whole links.`;}
  $('#new-chain-length').oninput=refresh;$('#chain-catalog').onchange=refresh;refresh();
}
function chainPitch(catalog){const ports=state.library[catalog].ports;return Math.hypot(...ports.a.position_mm.map((v,i)=>v-ports.b.position_mm[i]));}

function renderObjectInspector(instance,selectedPart){
  const pose={position_mm:[0,0,0],rotation_deg:[0,0,0],...instance.pose}, human=instance.template==='human',chain=instance.template==='chain';
  const parameters=human?{stature_mm:1750,mass_kg:75,strength_scale:1,joint_damping_nms_rad:.08,...instance.parameters}:instance.parameters||{};
  const posture=humanPosture(parameters);
  $('#inspector').innerHTML=`<div class="inspect-section"><div class="inspect-id">${esc(instance.template)}</div><h2>${esc(instance.label||instance.id)}</h2><div class="object-modes" role="group" aria-label="Object manipulation"><button data-object-mode="whole" aria-pressed="${objectMode(instance)!=='limb'}">${human?'Move whole person':chain?'Move whole chain':'Move whole object'}</button><button data-object-mode="limb" aria-pressed="${objectMode(instance)==='limb'}">${human?'Pose limbs':chain?'Pose links':'Pose parts'}</button></div><p>${objectMode(instance)==='limb'?'Select a body part, then drag or rotate it. Connected joints follow within their limits.':'Drag or rotate any part to move the entire object and keep its pose.'}</p></div><div class="inspect-section">${propertyFields('OBJECT POSITION',pose.position_mm,'position_mm')}${propertyFields('OBJECT ROTATION',pose.rotation_deg,'rotation_deg')}</div><div class="inspect-section"><h3>${chain?'ADVANCED EDITING':'OBJECT PARAMETERS'}</h3>${Object.entries(parameters).filter(([k,v])=>!chain&&typeof v==='number'&&(!human||k!=='grip_diameter_mm')).map(([k,v])=>`<div class="single-field"><label>${esc(k==='joint_damping_nms_rad'?'PASSIVE JOINT DAMPING · N·m·s/rad':k.replaceAll('_',' ').toUpperCase())}</label><input type="number" data-object-param="${esc(k)}" value="${v}"></div>`).join('')}${human?`
    ${instance.components?'<p id="object-edited-pose">Edited pose · use Pose limbs to adjust it.</p>':`<div class="single-field"><label for="object-pose">INITIAL POSE</label><select id="object-pose">${HUMAN_POSES.map(p=>`<option ${parameters.pose===p?'selected':''}>${p}</option>`).join('')}</select></div>`}
    <div class="single-field"><label for="object-hold">POSTURE CONTROL</label><select id="object-hold">${postureOptions(posture)}</select></div>
    <div class="single-field" id="object-held-joints-field" ${posture==='custom'?'':'hidden'}><label for="object-held-joints">JOINTS OR GROUPS · COMMA SEPARATED</label><input id="object-held-joints" value="${esc((parameters.hold_joints||[]).join(', '))}" placeholder="left_elbow, right_elbow, torso"></div>
    <p>Held joints resist motion with finite torque. Upper body leaves hips, knees and ankles free. Strength scale adjusts available torque. Passive damping slows free motion without holding an angle.</p>
    <div class="single-field"><label for="object-grip">GRIPPED BAR DIAMETER · mm</label><input id="object-grip" type="number" min="8" max="80" step="0.1" placeholder="Open hands" value="${parameters.grip_diameter_mm??''}"></div>
    <p>Curled hands have a grip port at the palm centre. Connect it to a bar with a revolute joint for a grasp that pivots around the bar. Calibrate limb measurements and initial joint angles in Source or expand the model.</p>`:''}<button class="inspect-action" id="object-expand">Expand into editable parts</button></div>`;
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
    if($('#object-pose'))$('#object-pose').onchange=e=>changeParameters(p=>{p.pose=e.target.value;});
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
function renderSimulation(){const r=state.recording;const humans=(state.doc.objects||[]).filter(o=>o.template==='human');$('#inspector').innerHTML=`<div class="inspect-section"><h2>Let physics explain it.</h2><p>Release the structure under gravity. Loose sockets can slide and turn; motors act through physical joints.</p><div class="single-field"><label>DURATION · SECONDS</label><input id="sim-duration" type="number" min="0.1" max="30" step="1" value="${state.simulationOptions?.duration||Math.min(30,Math.max(.1,Number(state.doc.metadata?.simulation_duration_s)||3))}"></div><div class="single-field"><label for="sim-chain-links">Chain links per rigid body</label><input id="sim-chain-links" type="number" min="1" max="1000" step="1" value="${state.simulationOptions?.chain_links_per_body||1}" ${state.busy?'disabled':''}></div><p>1 keeps full flexibility. Higher values make groups of links rigid for faster simulation. Attachment links stay flexible.</p><button class="inspect-action" data-run="simulate" ${state.busy?'disabled':''}>${state.simulation?'Simulating…':'Run simulation ▶'}</button>${state.simulation?'<div id="sim-progress" role="status" aria-live="polite"></div><button class="inspect-action secondary" id="cancel-simulation">Cancel simulation</button>':''}${state.simulationError?`<div class="issue error" role="alert" id="simulation-error"><strong>Simulation error</strong><p>${esc(state.simulationError)}</p></div>`:''}${r?`<div class="metric-cards"><div class="metric"><strong>${r.frames.length}</strong><span>RECORDED FRAMES</span></div><div class="metric"><strong>${r.settled?'Settled':'Moving'}</strong><span>FINAL STATE</span></div></div><p>${r.final_max_speed_m_s?.toFixed(3)||'0'} m/s maximum final speed</p>`:''}</div><div class="inspect-section"><h3>MOVING CONNECTIONS</h3>${(state.scene.chains||[]).map(c=>`<div class="joint-card"><div class="joint-card-top">${esc(c.id)}</div><p>${c.count} flexible links</p></div>`).join('')}${state.scene.joints.filter(j=>!j.locked&&j.type!=='fixed'&&!(state.scene.chains||[]).some(c=>j.a.part.startsWith(c.id+'/')&&j.b.part.startsWith(c.id+'/'))).map(j=>`<div class="joint-card"><div class="joint-card-top">${esc(j.id)}</div><p>${esc(j.type)}${j.motor?' · motor':''}</p>${Object.entries(j.limits||{}).map(([k,v])=>`<p>${esc(k)}: ${esc(JSON.stringify(v))}</p>`).join('')}</div>`).join('')||((state.scene.chains||[]).length?'':'<p>All connected parts are secured. Loosen a socket in Design mode to give it motion.</p>')}</div><div class="inspect-section"><h3>HUMAN FIT</h3><p>Test joint-limited reach and seated dimensions with a 19-segment human model.</p><button class="inspect-action secondary" id="fit-human">Open fit test</button><button class="inspect-action secondary" data-run="fit">Run saved design tests</button></div>${r?.events?.length?'<div class="inspect-section"><h3>SIMULATION EVENTS</h3>'+r.events.map(e=>`<div class="issue"><div class="issue-code">${esc(e.type)}</div><p>${esc(e.part||e.joint||'')} ${e.note?esc(e.note):''}</p></div>`).join('')+'</div>':''}`;bindOperations();$('#fit-human').onclick=fitDialog;if($('#cancel-simulation'))$('#cancel-simulation').onclick=cancelSimulation;updateSimulationProgress();}
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
async function run(operation){if(state.busy)return;state.busy=true;const revision=state.revision;const extra=operation==='simulate'?{duration:Number($('#sim-duration')?.value||3),chain_links_per_body:Number($('#sim-chain-links')?.value||1)}:{};if(operation==='simulate'){state.simulationError=null;state.simulationOptions=extra;state.simulation={status:'starting',progress:{message:'Preparing simulation'}};state.playing=false;}status(({validate:'Checking sockets and collisions…',analyse:'Solving the structural load case…',simulate:'Integrating rigid-body physics…',plan:'Searching stable assembly sequences…',fit:'Checking human fit…'})[operation]);renderInspector();
  try{const result=operation==='simulate'?await generateSimulation(extra):await api(operation,extra);if(!result)return;if(revision!==state.revision){toast('The design changed during the calculation. Run it again for the current design.');return;}state.doc.results||={};state.doc.results[operation]=result;state.dirty=true;
    if(operation==='validate'){state.checks=result;status(result.valid?'Geometry checks passed':'Design has '+result.summary.errors+' errors');}
    if(operation==='analyse'){state.analysis=result;status('Structural analysis: '+result.status);}
    if(operation==='simulate'){state.recording=result;state.frame=0;$('#time-slider').max=result.frames.length-1;$('#timeline-mid').textContent=(result.duration_s/2).toFixed(1)+' s';$('#timeline-end').textContent=result.duration_s.toFixed(1)+' s';state.playing=true;status('Simulation recorded · '+result.frames.length+' frames');}
    if(operation==='plan'){state.plan=result;state.doc.build_plan=result;state.step=0;status('Assembly search: '+result.status);if(result.status==='buildable')showStep(0);}
    if(operation==='fit')jsonDialog('Human fit results',result);updateHeader();
  }catch(e){if(operation==='simulate')state.simulationError=e.message;toast(e.message,true);status('Operation needs attention');}finally{state.busy=false;state.simulation=null;renderInspector();}}
function setMode(mode){state.mode=mode;$$('[data-mode]').forEach(b=>{b.classList.toggle('active',b.dataset.mode===mode);b.setAttribute('aria-selected',b.dataset.mode===mode?'true':'false');});if(mode!=='design'){gizmo?.detach();if(state.tool==='connect')setTool('select');}else attachGizmo();ports.visible=mode==='design';for(const obj of partObjects.values())obj.visible=true;if(mode==='build'&&state.plan?.status==='buildable')showStep(state.step);renderInspector();}
function showStep(index){if(state.plan?.status!=='buildable')return;state.step=index;const step=state.plan.steps[index];state.selected=step.part;for(const [id,object] of partObjects)object.visible=step.installed_parts.includes(id);highlightSelection();ports.visible=false;renderBuild();}
function showFrame(index){const frame=state.recording?.frames[index];if(!frame)return;state.frame=index;for(const [id,pose] of Object.entries(frame.parts)){if(partObjects.has(id))setPose(partObjects.get(id),pose);}$('#time-slider').value=index;$('#time-label').textContent=frame.time_s.toFixed(2)+' s';ports.visible=false;}
async function restore(){state.playing=false;$('#play-button').textContent='▶';for(const p of state.scene.parts)setPose(partObjects.get(p.id),p.pose);for(const o of partObjects.values())o.visible=true;state.frame=0;$('#time-slider').value=0;$('#time-label').textContent='0.00 s';ports.visible=true;updatePorts();}

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
  state.dirty=!!data.imported;state.revision++;state.playing=false;state.frame=0;state.step=0;
  state.checks=null;state.analysis=null;state.recording=null;state.plan=null;state.simulationOptions=null;state.simulationError=null;
  $('#time-slider').value=0;$('#time-slider').max=100;$('#time-label').textContent='0.00 s';
  $('#timeline-mid').textContent='1.5 s';$('#timeline-end').textContent='3 s';$('#play-button').textContent='▶';
  clearSnapPreview();setTool('select');buildScene(data.scene);renderLibrary();renderOutline();setMode('design');updateHeader();fitView();
  status(data.imported?'Opened file · Save to keep it in your workspace':'Loaded '+data.path);
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
async function loadDialog(){
  if(!canOpenDesign())return;
  closeModal();
  modal('Load design','<button id="open-file-button" class="option-row">Open file…<small>Choose a YAML or JSON design from your computer · Ctrl+O</small></button><p>You can also drop a design file anywhere in the editor.</p><div class="single-field"><label for="design-search">SAVED IN DESIGNS/</label><input id="design-search" type="search" placeholder="Find a saved design" disabled></div><div id="saved-design-list" aria-live="polite"><p>Loading saved designs…</p></div>',[{label:'Close',action:closeModal}]);
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
        state.doc=saved.document;state.path=saved.saved;state.dirty=false;updateHeader();
        toast('Saved '+saved.saved);
        if(error.isConnected&&$('#modal').open){closeModal();await afterSave?.();}
      }catch(e){error.textContent=e.message;}
    }}
  ]);
  $('#save-path').value=state.path;
}
function exportDialog(){modal('Export your creation','<button class="option-row" id="export-book-option">Illustrated build book<small>Printable instructions, bill of materials and a stock cutting plan</small></button><button class="option-row" id="export-image-option">Render an image<small>PNG from the current camera, with configurable lighting</small></button><button class="option-row" id="export-file-option">Download design file<small>Portable JSON with parts, constraints and recorded results</small></button>',[{label:'Close',action:closeModal}]);$('#export-book-option').onclick=()=>{closeModal();exportBuild();};$('#export-image-option').onclick=()=>{closeModal();renderDialog();};$('#export-file-option').onclick=()=>{const blob=new Blob([JSON.stringify(state.doc,null,2)],{type:'application/json'});const url=URL.createObjectURL(blob);const link=document.createElement('a');link.href=url;link.download=(state.doc.name||'design').replace(/[^a-z0-9-]/gi,'-')+'.pipe.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),10000);};}
async function exportBuild(){if(state.busy)return;state.busy=true;status('Generating the illustrated build book…');try{const result=await api('export',{engineering:true});modal('Build book ready',`<p>${result.steps} illustrated assembly steps, ${result.stock_bars} stock lengths, plus the parts list and engineering results.</p><a class="button primary" href="${esc(result.url)}" target="_blank">Open printable instructions ↗</a><p>Saved in ${esc(result.directory)}</p>`,[{label:'Done',action:closeModal}]);status('Build instructions exported');}catch(e){toast(e.message,true);}finally{state.busy=false;}}
function renderDialog(){modal('Render an image','<div class="fields"><div class="field"><label>WIDTH px</label><input id="render-width" value="1600" type="number"></div><div class="field"><label>HEIGHT px</label><input id="render-height" value="1000" type="number"></div></div><div class="single-field"><label>LIGHTING</label><select id="render-light"><option>studio</option><option>technical</option><option>flat</option></select></div><div class="single-field"><label>BACKGROUND</label><select id="render-bg"><option value="#edf1f3">Soft grey</option><option value="#ffffff">White</option><option value="transparent">Transparent</option></select></div>',[{label:'Cancel',action:closeModal},{label:'Render PNG',primary:true,action:async()=>{status('Rendering image…');const result=await api('render',{options:{width:+$('#render-width').value,height:+$('#render-height').value,eye:camera.position.toArray(),target:orbit.target.toArray(),lighting:$('#render-light').value,background:$('#render-bg').value}});closeModal();modal('Image ready',`<a href="${esc(result.url)}" target="_blank"><img src="${esc(result.url)}" style="width:100%" alt="Rendered pipe creation"></a><p>Open the image to save it at full resolution.</p>`,[{label:'Done',action:closeModal}]);status('Image exported');}}]);}
function humanDialog(){
  modal('Add a human model',`<p>A configurable 19-part mannequin with articulated spine, neck, shoulders, arms, hands, hips, knees and ankles.</p>
    <div class="fields"><div class="field"><label for="human-height">STATURE mm</label><input id="human-height" value="1750" type="number"></div><div class="field"><label for="human-mass">MASS kg</label><input id="human-mass" value="75" type="number"></div></div>
    <div class="single-field"><label for="human-pose">INITIAL POSE</label><select id="human-pose">${HUMAN_POSES.map(p=>`<option>${p}</option>`).join('')}</select></div>
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
}
function fitDialog(){const humans=state.scene.parts.filter(p=>p.kind==='human'&&p.id.endsWith('/pelvis')).map(p=>p.id.slice(0,-7));if(!humans.length){humanDialog();return;}modal('Human fit test',`<div class="single-field"><label>HUMAN</label><select id="fit-id">${humans.map(id=>`<option>${esc(id)}</option>`).join('')}</select></div><div class="single-field"><label>TEST</label><select id="fit-kind"><option value="reach">Right-hand reach</option><option value="seat">Seated dimensions</option></select></div><div class="fields">${['X','Y','Z'].map((v,i)=>`<div class="field"><label>TARGET ${v} mm</label><input id="fit-${i}" type="number" value="${[300,400,1200][i]}"></div>`).join('')}</div><div class="single-field"><label>SEAT PART (FOR SEATED TEST)</label><select id="fit-seat">${state.scene.parts.filter(p=>p.kind==='panel').map(p=>`<option>${esc(p.id)}</option>`).join('')}</select></div>`,[{label:'Cancel',action:closeModal},{label:'Run fit test',primary:true,action:async()=>{const params={human:$('#fit-id').value};if($('#fit-kind').value==='reach')params.target=[0,1,2].map(i=>+$('#fit-'+i).value);else params.seat=$('#fit-seat').value;const r=await api('fit',params);closeModal();if(r.parts)for(const [id,p] of Object.entries(r.parts))if(partObjects.has(id))setPose(partObjects.get(id),p);jsonDialog('Fit test result',r);}}]);}
function connectDialog(connector,port=null,replaceJoint=null){
  const old=state.doc.joints?.find(j=>j.id===replaceJoint);
  const member=state.scene.parts.find(p=>p.id===(old?.b.part||state.connectionSource)&&p.kind==='member');
  if(!member){toast('Select a tube, dowel or extrusion first, then click the target connector.');return;}
  const fitting=state.scene.parts.find(p=>p.id===connector);
  const sockets=Object.entries(fitting?.ports||{}).filter(([name,p])=>p.type==='socket'&&!socketOccupied(state.scene,connector,name,replaceJoint));
  if(!sockets.length)return;
  if(port&&socketOccupied(state.scene,connector,port,replaceJoint)){toast('This socket or its shared bore is occupied.');return;}
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

function ray(event){const rect=renderer.domElement.getBoundingClientRect();pointer.set((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1);raycaster.setFromCamera(pointer,camera);}
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
if(renderer){let down=null;
  renderer.domElement.addEventListener('pointerdown',e=>{
    down=[e.clientX,e.clientY];if(e.button!==0||state.placementPending||state.mode!=='design'||!['select','translate'].includes(state.tool)||gizmo.axis)return;
    ray(e);const hit=pickPart();if(!hit)return;
    select(hit.id);orbit.enabled=false;
    pointerDrag={pointerId:e.pointerId,x:e.clientX,y:e.clientY,point:hit.point,
      plane:new THREE.Plane().setFromNormalAndCoplanarPoint(camera.getWorldDirection(new THREE.Vector3()),hit.point),started:false};
    renderer.domElement.setPointerCapture(e.pointerId);e.stopImmediatePropagation();
  },true);
  renderer.domElement.addEventListener('pointermove',e=>{
    if(!pointerDrag||pointerDrag.pointerId!==e.pointerId)return;e.stopImmediatePropagation();
    if(!pointerDrag.started){if(Math.hypot(e.clientX-pointerDrag.x,e.clientY-pointerDrag.y)<4)return;
      if(!beginPlacement('pointer')){cancelPlacement();return;}pointerDrag.started=true;gizmo.detach();}
    ray(e);const point=raycaster.ray.intersectPlane(pointerDrag.plane,new THREE.Vector3());if(!point)return;
    const delta=point.sub(pointerDrag.point);if(state.snap)delta.divideScalar(state.snapSettings.translationMm).round().multiplyScalar(state.snapSettings.translationMm);
    for(const [id,matrix] of dragStart.matrices){const object=partObjects.get(id);matrix.decompose(object.position,object.quaternion,object.scale);object.position.add(delta);object.updateMatrix();}
    previewMovement();
  },true);
  renderer.domElement.addEventListener('pointerup',e=>{
    if(pointerDrag&&pointerDrag.pointerId===e.pointerId){const started=pointerDrag.started;pointerDrag=null;down=null;orbit.enabled=true;
      if(renderer.domElement.hasPointerCapture(e.pointerId))renderer.domElement.releasePointerCapture(e.pointerId);
      e.stopImmediatePropagation();if(started)finishPlacement();else attachGizmo();return;}
    if(state.placementPending||gizmo.dragging||!down||e.button!==0||Math.hypot(e.clientX-down[0],e.clientY-down[1])>5){down=null;return;}
    down=null;ray(e);
    if(state.tool==='connect'){const socket=pickSocket();if(socket){connectDialog(socket.part,socket.port);return;}}
    select(pickPart()?.id||null);
  },true);
  renderer.domElement.addEventListener('pointercancel',()=>{down=null;cancelPlacement();});
  renderer.domElement.addEventListener('lostpointercapture',()=>{if(pointerDrag){down=null;cancelPlacement();}});
  renderer.domElement.addEventListener('dragover',e=>{e.preventDefault();e.dataTransfer.dropEffect='copy';});
  renderer.domElement.addEventListener('drop',async e=>{
    e.preventDefault();const catalog=e.dataTransfer.getData('application/pipesim-part');if(!catalog||state.placementPending)return;
    ray(e);const hit=state.connectionSnap?pickPart():null;
    const point=hit?.point.clone()||raycaster.ray.intersectPlane(groundPlane,new THREE.Vector3());if(!point)return;
    if(state.snap&&!hit)point.divideScalar(state.snapSettings.translationMm).round().multiplyScalar(state.snapSettings.translationMm);
    if(state.library[catalog].kind==='chain'){chainDialog(catalog,point.toArray());return;}
    try{const id=await addPart(catalog,point.toArray(),{origin:state.library[catalog].kind==='connector',quiet:true});
      const matches=dragCandidates([id]);if(matches.length)await offerConnection(matches,{},state.revision,false);else toast('Added '+id+'. Drag it onto a pipe or socket to connect.');
    }catch(error){toast(error.message,true);}
  });
}
$('#part-search').oninput=renderLibrary;$('#category').onchange=renderLibrary;$('#size-filter').onchange=renderLibrary;
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
$('#delete-part').onclick=()=>{const id=state.selected;if(id)deleteTreeTarget({label:id,members:[id]});};
$('#fit-view').onclick=fitView;$('#grid-button').onclick=()=>{grid.visible=!grid.visible;$('#grid-button').classList.toggle('active',grid.visible);};$('#ports-button').onclick=()=>{state.ports=!state.ports;$('#ports-button').classList.toggle('active',state.ports);ports.visible=true;updatePorts();};$('#snap-button').onclick=()=>{state.snap=!state.snap;updateSnapControls();};
$$('[data-camera]').forEach(b=>b.onclick=()=>{const distance=camera.position.distanceTo(orbit.target);const vector=({top:new THREE.Vector3(.001,-.001,1),front:new THREE.Vector3(0,-1,.001),side:new THREE.Vector3(1,0,.001)})[b.dataset.camera];camera.position.copy(orbit.target).addScaledVector(vector.normalize(),distance);$('#view-title').textContent=b.dataset.camera[0].toUpperCase()+b.dataset.camera.slice(1)+' view';orbit.update();});
$('#play-button').onclick=()=>{if(!state.recording){setMode('simulate');toast('Run a simulation to record a motion timeline.');return;}state.playing=!state.playing;$('#play-button').textContent=state.playing?'Ⅱ':'▶';};$('#reset-button').onclick=restore;$('#time-slider').oninput=e=>{state.playing=false;showFrame(+e.target.value);};
$('#capture-frame').onclick=async()=>{if(!state.recording){toast('Record a simulation first.');return;}const frame=clone(state.recording.frames[state.frame]);const expanded=await api('snapshot',{frame});checkpoint();state.doc=expanded;changed();await resolve();toast('Captured this pose and adjusted the remaining joint limits. Compound human rotations may need anatomical review.');};
$('#import-button').onclick=()=>$('#mesh-file').click();$('#mesh-file').onchange=e=>{const file=e.target.files[0];if(!file)return;modal('Import '+file.name,'<p>Provide the physical mass and convert the mesh coordinates to millimetres. Hollow parts can use separate collision geometry in the library editor.</p><div class="fields"><div class="field"><label>MASS kg</label><input id="import-mass" type="number" value="1" min="0.001"></div><div class="field"><label>SCALE TO mm</label><input id="import-scale" type="number" value="1" min="0.001"></div></div>',[{label:'Cancel',action:closeModal},{label:'Import part',primary:true,action:async()=>{const buffer=new Uint8Array(await file.arrayBuffer());let binary='';for(let i=0;i<buffer.length;i+=32768)binary+=String.fromCharCode(...buffer.subarray(i,i+32768));const r=await api('import-mesh',{filename:file.name,data:btoa(binary),mass_kg:+$('#import-mass').value,scale:+$('#import-scale').value});checkpoint();state.doc=r.document;state.library=r.library;changed();renderLibrary();closeModal();await addPart(r.import.part);}}]);e.target.value='';};
document.addEventListener('keydown',e=>{if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='o'){e.preventDefault();if(!$('#modal').open)chooseDesignFile();return;}if(e.key==='Escape'&&dragStart){e.preventDefault();cancelPlacement();setTool('select');return;}if(state.placementPending&&!$('#modal').open)return;if(['INPUT','TEXTAREA','SELECT'].includes(document.activeElement?.tagName)||$('#modal').open)return;if((e.ctrlKey||e.metaKey)&&e.key==='z'){e.preventDefault();$('#undo').click();}else if((e.ctrlKey||e.metaKey)&&e.key==='y'){e.preventDefault();$('#redo').click();}else if((e.ctrlKey||e.metaKey)&&e.key==='s'){e.preventDefault();saveDialog();}else if(e.key.toLowerCase()==='f')fitView();else if(e.key.toLowerCase()==='g')setTool('translate');else if(e.key.toLowerCase()==='r')setTool('rotate');else if(e.key.toLowerCase()==='q'||e.key==='Escape')setTool('select');else if(e.key==='Delete')$('#delete-part').click();else if(e.key==='/'){e.preventDefault();$('#part-search').focus();}});
window.addEventListener('beforeunload',e=>{if(state.dirty){e.preventDefault();e.returnValue='';}});
let last=performance.now(),accumulator=0;
function tick(now){requestAnimationFrame(tick);const dt=Math.min((now-last)/1000,.1);last=now;if(state.playing&&state.recording){accumulator+=dt;const step=1/(state.recording.fps||30);if(accumulator>=step){showFrame((state.frame+1)%state.recording.frames.length);accumulator%=step;}$('#play-button').textContent='Ⅱ';}orbit?.update();if(state.selected&&partObjects.has(state.selected)){const pos=partObjects.get(state.selected).position.clone().project(camera);const rect=viewport.getBoundingClientRect();const label=$('#selection-label');label.style.left=((pos.x+1)*rect.width/2+15)+'px';label.style.top=((1-pos.y)*rect.height/2+48)+'px';}renderer?.render(scene,camera);}requestAnimationFrame(tick);
try{const response=await fetch('/api/bootstrap'),data=await response.json();if(!response.ok)throw new Error(data.error);token=data.token;state.doc=data.document;state.path=data.path;state.library={...data.library,...state.doc.definitions};state.examples=data.examples;state.selected=data.scene.parts.some(p=>p.id==='leg-1')?'leg-1':null;buildScene(data.scene);renderLibrary();renderOutline();renderInspector();updateHeader();fitView();if(data.api_version!==EDITOR_API_VERSION){status('Editor server update required');toast(SERVER_UPDATE_MESSAGE,true);}else status('Workspace ready · all changes stay local');}catch(e){toast(e.message,true);status('Workspace could not be opened');}

function jointDialog(id=null){
  const old=id?state.doc.joints?.find(j=>j.id===id):null;
  if(old?.type==='socket')return connectDialog(old.a.part,old.a.port,id);
  if(id&&!old){toast('Expand this object before editing its joints.');return;}
  const others=state.scene.parts.filter(p=>p.id!==state.selected);
  if(!old&&!others.length){toast('Add a second part first.');return;}
  const selected=state.scene.parts.find(p=>p.id===state.selected), other=others[0];
  const pivot=new THREE.Vector3(...selected.pose.position_mm), local=other?pivot.clone().applyMatrix4(partObjects.get(other.id).matrix.clone().invert()):pivot;
  const value=old||{id:'attachment-'+(state.doc.joints?.length||0),type:'fixed',a:{part:selected.id,frame:{position_mm:[0,0,0],axis:[0,0,1]}},b:{part:other.id,frame:{position_mm:local.toArray(),axis:[0,0,1]}},metadata:{hardware:'Specify the actual bolt, bracket or connector and its installation method'}};
  modal(old?'Edit connection':'Add connection','<p>Set the two part IDs and their local attachment frames or named ports. Choose fixed, revolute, prismatic, cylindrical, spherical or distance. Limits use mm and degrees; motor torque uses N·m.</p><textarea id="joint-source" spellcheck="false"></textarea>',[{label:'Cancel',action:closeModal},{label:'Apply connection',primary:true,action:async()=>{const edited=JSON.parse($('#joint-source').value);await mutate(()=>{state.doc.joints||=[];if(old)state.doc.joints[state.doc.joints.findIndex(j=>j.id===old.id)]=edited;else state.doc.joints.push(edited);});closeModal();}}]);
  $('#joint-source').value=JSON.stringify(value,null,2);
}
