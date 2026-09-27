import * as THREE from 'three';

const THROUGH_AXIS_TOLERANCE_DEG=THREE.MathUtils.radToDeg(Math.acos(.999));

export const draftRuns=doc=>(doc.draft_subassemblies||[]).flatMap(group=>group.runs);
export const draftRun=(doc,id)=>draftRuns(doc).find(run=>run.id===id);

function substitute(value,parameters){
  if(typeof value==='string'&&value.startsWith('$'))return structuredClone(parameters[value.slice(1)]);
  if(Array.isArray(value))return value.map(v=>substitute(v,parameters));
  if(value&&typeof value==='object')return Object.fromEntries(Object.entries(value).map(([k,v])=>[k,substitute(v,parameters)]));
  return value;
}

export function draftPreview(run,scene,library){
  const originalStart=new THREE.Vector3(...run.start_mm),originalEnd=new THREE.Vector3(...run.end_mm);
  const workingLength=originalStart.distanceTo(originalEnd);
  if(!(workingLength>0))throw new Error('Draft span must be positive');
  const displayLength=run.locked_length_mm??workingLength;
  const bound={};const through=[];const conflicts=[];
  function socketFrame(attachment){
    const fitting=scene.parts.find(p=>p.id===attachment.connector),socket=fitting?.ports?.[attachment.port];
    if(!socket||socket.type!=='socket')throw new Error(`Unknown socket ${attachment.connector}/${attachment.port}`);
    const pose=fitting.pose||{},quaternion=new THREE.Quaternion().setFromEuler(new THREE.Euler(...(pose.rotation_deg||[0,0,0]).map(THREE.MathUtils.degToRad),'ZYX'));
    const mouth=new THREE.Vector3(...(socket.position_mm||[0,0,0])).applyQuaternion(quaternion).add(new THREE.Vector3(...(pose.position_mm||[0,0,0])));
    const axis=new THREE.Vector3(...(socket.axis||[0,0,1])).applyQuaternion(quaternion).normalize();
    return {socket,mouth,axis};
  }
  for(const a of run.attachments||[]){const frame=socketFrame(a);if(frame.socket.through){through.push({a,...frame});continue;}
    bound[a.end]={point:frame.mouth.clone().addScaledVector(frame.axis,-a.insertion_mm),axis:frame.axis,a};}
  let start=originalStart,end=originalEnd;
  if(bound.start&&bound.end){start=bound.start.point;end=bound.end.point;}
  else if(bound.start){start=bound.start.point;end=start.clone().addScaledVector(bound.start.axis,displayLength);}
  else if(bound.end){end=bound.end.point;start=end.clone().addScaledVector(bound.end.axis,displayLength);}
  else end=start.clone().add(originalEnd.clone().sub(originalStart).normalize().multiplyScalar(displayLength));
  let length=start.distanceTo(end);if(!(length>1e-6)){
    start=bound.start.point;end=start.clone().add(originalEnd.clone().sub(originalStart).normalize().multiplyScalar(workingLength));length=workingLength;
    conflicts.push({code:'ZERO_SPAN',residual_mm:+workingLength.toFixed(2),message:'Both pipe ends target the same point; move a connector or detach an end'});
  }
  const direction=end.clone().sub(start).normalize();
  if(bound.start&&bound.end){const span=end.clone().sub(start),lateral=span.clone().addScaledVector(bound.start.axis,-span.dot(bound.start.axis)).length();
    if(lateral>1)conflicts.push({code:'POSITION_MISMATCH',residual_mm:+lateral.toFixed(2),message:`End socket lies ${lateral.toFixed(1)} mm off the start socket axis`});}
  for(const [name,expected] of [['start',direction],['end',direction.clone().negate()]])if(bound[name]){
    const angle=THREE.MathUtils.radToDeg(expected.angleTo(bound[name].axis));
    if(angle>2)conflicts.push({code:'AXIS_MISMATCH',connector:bound[name].a.connector,port:bound[name].a.port,residual_deg:+angle.toFixed(2),message:`${name} socket points ${angle.toFixed(1)}° away from the run`});
  }
  if(bound.start&&bound.end&&run.locked_length_mm!=null&&Math.abs(run.locked_length_mm-length)>.05)conflicts.push({code:'LOCKED_LENGTH',residual_mm:+(length-run.locked_length_mm).toFixed(2),message:'Locked cut length disagrees with the connector span'});
  for(const {a,socket,mouth,axis} of through){const station=mouth.clone().sub(start).dot(direction),gap=mouth.distanceTo(start.clone().addScaledVector(direction,station));
    const angle=THREE.MathUtils.radToDeg(Math.acos(Math.min(1,Math.abs(direction.dot(axis))))),half=(socket.engagement_mm||0)/2;
    const short=Math.max(half-station,station-(length-half),0);
    if(gap>1||angle>THROUGH_AXIS_TOLERANCE_DEG||short>0){
      const message=short>0?`Through socket needs ${short.toFixed(3)} mm more pipe engagement`:
        angle>THROUGH_AXIS_TOLERANCE_DEG?`Through socket axis differs by ${angle.toFixed(3)}Â°`:
        `Through socket misses the pipe centreline by ${gap.toFixed(3)} mm`;
      conflicts.push({code:'THROUGH_FIT',connector:a.connector,port:a.port,residual_mm:+Math.max(gap,short).toFixed(3),residual_deg:+angle.toFixed(3),message});
    }
  }
  const reference=Math.abs(direction.y)<.95?new THREE.Vector3(0,1,0):new THREE.Vector3(1,0,0);
  const x=reference.clone().cross(direction).normalize(),y=direction.clone().cross(x);
  const quaternion=new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x,y,direction));
  quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,0,1),THREE.MathUtils.degToRad(run.roll_deg||0)));
  const euler=new THREE.Euler().setFromQuaternion(quaternion,'ZYX');
  const definition=library[run.catalog];if(!definition||definition.kind!=='member')throw new Error(`Unknown member profile ${run.catalog}`);
  const resolved=substitute(definition,{...definition.parameters,...run.parameters,length_mm:length});
  return {id:run.id,label:run.id+' · draft',catalog:run.catalog,kind:'member',draft:true,
    pose:{position_mm:start.clone().add(end).multiplyScalar(.5).toArray(),rotation_deg:[euler.x,euler.y,euler.z].map(THREE.MathUtils.radToDeg)},
    geometry:resolved.geometry,ports:{},section:resolved.section||{},length_mm:length,mass_kg:0,
    color:conflicts.length?'#cf815d':'#5ba9b5',conflicts,attachments:run.attachments||[]};
}
