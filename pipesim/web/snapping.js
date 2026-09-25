import * as THREE from 'three';

export function socketOccupied(scene, id, name, ignore=null) {
  const fitting=scene.parts.find(p=>p.id===id), port=fitting?.ports[name];
  const excluded=new Set([name,...(port?.excludes||[])]);
  return scene.joints.some(j=>j.id!==ignore&&[j.a,j.b].some(e=>e.part===id&&
    (excluded.has(e.port)||fitting.ports[e.port]?.excludes?.includes(name))));
}

function fits(member, socket) {
  const section=member.section||{};
  const profile=['tube','round','circle'].includes(section.type)?'round':section.profile||section.type;
  return socket.type==='socket'&&profile===(socket.profile||'round')&&
    Math.abs((section.diameter_mm||0)-(socket.diameter_mm||0))<=.6;
}

export function projectedStation(start, end, point, camera, width, height) {
  const a=start.clone().project(camera), b=end.clone().project(camera), p=point.clone().project(camera);
  const dx=(b.x-a.x)*width/2, dy=(b.y-a.y)*height/2;
  const t=THREE.MathUtils.clamp((((p.x-a.x)*width/2)*dx+((p.y-a.y)*height/2)*dy)/(dx*dx+dy*dy||1),0,1);
  // Perspective interpolation must account for depth, otherwise a visual
  // midpoint on a receding tube is incorrectly treated as half its length.
  const wa=-start.clone().applyMatrix4(camera.matrixWorldInverse).z;
  const wb=-end.clone().applyMatrix4(camera.matrixWorldInverse).z;
  return camera.isPerspectiveCamera?t*wa/((1-t)*wb+t*wa):t;
}

export function connectionCandidates(scene, matrices, movingIds, camera, width, height, settings={}) {
  // Accept the earlier boolean argument for callers using the default grid.
  if(typeof settings==='boolean')settings={gridEnabled:settings};
  const capture=settings.connectionPixels??30,step=settings.translationMm??10;
  const moving=new Set(movingIds), candidates=[];
  camera.updateMatrixWorld();
  const project=p=>p.clone().project(camera);
  const pixelDistance=(a,b)=>Math.hypot((a.x-b.x)*width/2,(a.y-b.y)*height/2);
  const members=scene.parts.filter(p=>p.kind==='member'&&p.length_mm>0);
  const groups=new Map(scene.groups.flatMap((g,i)=>g.map(id=>[id,i])));
  for(const fitting of scene.parts) for(const [name,socket] of Object.entries(fitting.ports||{})) {
    if(socket.type!=='socket'||socketOccupied(scene,fitting.id,name))continue;
    const fm=matrices.get(fitting.id); if(!fm)continue;
    const mouth=new THREE.Vector3(...(socket.position_mm||[0,0,0])).applyMatrix4(fm);
    const axis=new THREE.Vector3(...(socket.axis||[0,0,1])).transformDirection(fm);
    for(const member of members) {
      if(moving.has(member.id)===moving.has(fitting.id)||groups.get(member.id)===groups.get(fitting.id)||!fits(member,socket))continue;
      const mm=matrices.get(member.id), length=member.length_mm;
      if(!mm)continue;
      const start=new THREE.Vector3(0,0,-length/2).applyMatrix4(mm), end=new THREE.Vector3(0,0,length/2).applyMatrix4(mm);
      const tubeAxis=new THREE.Vector3(0,0,1).transformDirection(mm);
      const endpoints=[];
      if(socket.through) {
        const half=(socket.engagement_mm||0)/2; if(length<half*2)continue;
        let station=projectedStation(start,end,mouth,camera,width,height)*length;
        // The tube midpoint is a useful construction feature, separate from
        // the grid. Snap to it within a small visual tolerance.
        const center=start.clone().lerp(end,.5);
        if(pixelDistance(project(center),project(mouth))<8)station=length/2;
        else if(settings.gridEnabled)station=Math.round(station/step)*step;
        station=THREE.MathUtils.clamp(station,half,length-half);
        endpoints.push({at_mm:station,point:start.clone().lerp(end,station/length),direction:tubeAxis});
      }else for(const [label,point,direction] of [['start',start,tubeAxis],['end',end,tubeAxis.clone().negate()]]) {
        if(scene.joints.some(j=>[j.a,j.b].some(e=>e.part===member.id&&e.end===label)))continue;
        endpoints.push({end:label,point,direction});
      }
      for(const endpoint of endpoints) {
        const a=project(mouth), b=project(endpoint.point);
        if(a.z< -1||a.z>1||b.z< -1||b.z>1)continue;
        const distance=pixelDistance(a,b); if(distance>capture)continue;
        const alignment=axis.dot(endpoint.direction);
        const angle=THREE.MathUtils.radToDeg(Math.acos(THREE.MathUtils.clamp(socket.through?Math.abs(alignment):alignment,-1,1)));
        candidates.push({member:member.id,connector:fitting.id,port:name,
          ...(socket.through?{at_mm:endpoint.at_mm}:{end:endpoint.end}),
          insertion_mm:socket.through?0:Math.min(30,socket.engagement_mm*.8),
          distance,angle,score:distance+angle*.18,mouth:mouth.toArray(),point:endpoint.point.toArray(),
          label:`${fitting.id} / ${socket.label||name} → ${member.id} · ${socket.through?endpoint.at_mm.toFixed(1)+' mm from start':endpoint.end+' end'}`});
      }
    }
  }
  candidates.sort((a,b)=>a.score-b.score||a.connector.localeCompare(b.connector)||a.port.localeCompare(b.port));
  return candidates.slice(0,6);
}

export function clearConnectionIntent(candidates) {
  return !!candidates.length&&candidates[0].angle<=8&&
    (!candidates[1]||candidates[1].score-candidates[0].score>=7);
}

export function alignmentDelta(scene, matrices, candidate, side) {
  const fitting=scene.parts.find(p=>p.id===candidate.connector), member=scene.parts.find(p=>p.id===candidate.member);
  const socket=fitting.ports[candidate.port], fm=matrices.get(fitting.id), mm=matrices.get(member.id);
  const mouth=new THREE.Vector3(...socket.position_mm).applyMatrix4(fm);
  const axis=new THREE.Vector3(...socket.axis).transformDirection(fm);
  const station=socket.through?candidate.at_mm:candidate.end==='end'?member.length_mm:0;
  const point=new THREE.Vector3(0,0,station-member.length_mm/2).applyMatrix4(mm);
  const direction=new THREE.Vector3(0,0,candidate.end==='end'?-1:1).transformDirection(mm);
  if(socket.through&&axis.dot(direction)<0)direction.negate();
  const rotation=new THREE.Quaternion().setFromUnitVectors(side==='connector'?axis:direction,side==='connector'?direction:axis);
  const delta=new THREE.Matrix4().makeRotationFromQuaternion(rotation), insertion=candidate.insertion_mm||0;
  const target=side==='connector'?point.clone().addScaledVector(direction,insertion):mouth.clone().addScaledVector(axis,-insertion);
  delta.setPosition(target.sub((side==='connector'?mouth:point).clone().applyQuaternion(rotation)));
  return delta;
}

export function rotationAlignment(scene, matrices, movingIds, selected, toleranceDeg=6, rotationAxis=null){
  const moving=new Set(movingIds),part=scene.parts.find(p=>p.id===selected),matrix=matrices.get(selected);
  if(!matrix||!part)return null;
  const axes=[new THREE.Vector3(1,0,0),new THREE.Vector3(0,1,0),new THREE.Vector3(0,0,1)];
  const names=['X','Y','Z'];
  const round=part.kind==='member'&&['tube','round','circle'].includes(part.section?.type);
  const sources=(round?[axes[2]]:axes).map(a=>a.clone().transformDirection(matrix));
  const targets=axes.map((axis,i)=>({axis,label:'world '+names[i]}));
  for(const other of scene.parts){
    if(moving.has(other.id)||!matrices.has(other.id))continue;
    const roundOther=other.kind==='member'&&['tube','round','circle'].includes(other.section?.type);
    for(const i of roundOther?[2]:[0,1,2])targets.push({axis:axes[i].clone().transformDirection(matrices.get(other.id)),label:other.id+' / '+names[i]});
  }
  const pivot=rotationAxis?.lengthSq()>1e-12?rotationAxis.clone().normalize():null;let best=null;
  for(const source of sources)for(const target of targets)for(const sign of [1,-1]){
    const direction=target.axis.clone().multiplyScalar(sign);let correction,angle;
    if(pivot){
      // Match only within the chosen rotation plane. A vector parallel to the
      // handle's axis is unchanged by rotation and conveys no alignment intent.
      if(Math.abs(source.dot(pivot)-direction.dot(pivot))>1e-5)continue;
      const a=source.clone().addScaledVector(pivot,-source.dot(pivot));
      const b=direction.clone().addScaledVector(pivot,-direction.dot(pivot));
      if(a.length()<1e-5||b.length()<1e-5)continue;a.normalize();b.normalize();
      angle=Math.atan2(pivot.dot(a.clone().cross(b)),a.dot(b));
      correction=new THREE.Quaternion().setFromAxisAngle(pivot,angle);
    }else{
      angle=Math.acos(THREE.MathUtils.clamp(source.dot(direction),-1,1));
      correction=new THREE.Quaternion().setFromUnitVectors(source,direction);
    }
    const degrees=Math.abs(THREE.MathUtils.radToDeg(angle));
    if(degrees>toleranceDeg||best&&degrees>=best.angleDeg-1e-6)continue;
    best={correction,angleDeg:degrees,label:target.label,direction};
  }
  return best;
}
