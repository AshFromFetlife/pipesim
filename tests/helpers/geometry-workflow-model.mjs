// Constructive oracle: no PipeSim solver, preview, snapping or validation code.
// Catalogue dimensions define a buildable frame; vector arithmetic defines what
// each edit must do. A solver rejection must never filter a generated case out.
import assert from 'node:assert/strict';
import * as THREE from 'three';

export const v = values => new THREE.Vector3(...values);
export const runs = doc => (doc.draft_subassemblies || []).flatMap(g => g.runs);
export function random(seed) {
  let state = seed >>> 0;
  return () => {state = (Math.imul(state, 1664525) + 1013904223) >>> 0; return state / 0x100000000;};
}
export function shuffled(values, rng) {
  const result = [...values];
  for (let i = result.length - 1; i > 0; i--) {const j = Math.floor(rng() * (i + 1)); [result[i], result[j]] = [result[j], result[i]];}
  return result;
}
export function matrix(pose = {}) {
  return new THREE.Matrix4().compose(v(pose.position_mm || [0, 0, 0]),
    new THREE.Quaternion().setFromEuler(new THREE.Euler(...(pose.rotation_deg || [0, 0, 0]).map(THREE.MathUtils.degToRad), 'ZYX')),
    v([1, 1, 1]));
}
export function endpoints(part) {
  const length = part.length_mm ?? part.parameters?.length_mm;
  assert.ok(Number.isFinite(length) && length > 0, `${part.id}: invalid length ${length}`);
  return [-1, 1].map(sign => v([0, 0, sign * length / 2]).applyMatrix4(matrix(part.pose)).toArray());
}
export function closePoint(actual, expected, label, tolerance = .06) {
  const distance = v(actual).distanceTo(v(expected));
  assert.ok(Number.isFinite(distance) && distance <= tolerance,
    `${label}: displaced ${distance} mm; expected ${JSON.stringify(expected)}, got ${JSON.stringify(actual)}`);
}
export function closeSpan(actual, expected, label) {
  const direct = v(actual[0]).distanceTo(v(expected[0])) + v(actual[1]).distanceTo(v(expected[1]));
  const reverse = v(actual[0]).distanceTo(v(expected[1])) + v(actual[1]).distanceTo(v(expected[0]));
  const ordered = reverse < direct ? [...actual].reverse() : actual;
  ordered.forEach((point, i) => closePoint(point, expected[i], `${label}/${i}`));
}
export const edgeKey = edge => [edge.member, edge.connector, edge.port].join('/');
export function connections(doc) {
  return [...runs(doc).flatMap(run => (run.attachments || []).map(a => ({member: run.id, ...a}))),
    ...(doc.joints || []).filter(j => j.type === 'socket').map(j => ({member: j.b.part, connector: j.a.part, port: j.a.port, end: j.b.end}))];
}

export function generate(seed, library, {steps = 18} = {}) {
  const rng = random(seed), choose = values => values[Math.floor(rng() * values.length)];
  const family = ['comb', 'rectangle', 'offset-grid'][seed % 3];
  const angles = seed % 2 ? [rng()*140-70, rng()*140-70, rng()*360-180] : [0, choose([0,90,180,270]), 0];
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(...angles.map(THREE.MathUtils.degToRad), 'ZYX'));
  const origin = v([rng()*1800-900, rng()*1800-900, 2200+rng()*500]);
  const world = point => v(point).applyQuaternion(q).add(origin).toArray();
  const doc = {format:'pipesim/1', units:'mm-kg-s-N-deg', name:`Geometry workflow ${seed}`,
    parts:[], joints:[], anchors:[], draft_subassemblies:[{id:'frame', runs:[]}]};
  const model = {spans:{}, fittings:{}, edges:[], mirror:null};
  const addPart = (id, catalog, point, turn = 0) => {
    const rotation = q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(v([0,1,0]), turn*Math.PI/180));
    const e = new THREE.Euler().setFromQuaternion(rotation, 'ZYX');
    const pose = {position_mm:world(point), rotation_deg:[e.x,e.y,e.z].map(THREE.MathUtils.radToDeg)};
    doc.parts.push({id, catalog, pose}); model.fittings[id] = structuredClone(pose);
    return doc.parts.at(-1);
  };
  const socket = (part, port) => {
    const definition = library[part.catalog].ports[port], frame = matrix(part.pose);
    const axis = v(definition.axis).transformDirection(frame);
    // The editor's documented default engagement, bounded by catalogue depth.
    const insertion = Math.min(30, definition.engagement_mm*.8);
    assert.ok(definition.through || insertion >= definition.min_engagement_mm);
    const mouth = v(definition.position_mm).applyMatrix4(frame);
    return {definition, axis, point:mouth.addScaledVector(axis, definition.through ? 0 : -insertion)};
  };
  const addRun = (id, first, last, edges = []) => {
    const reversed = rng() < .5;
    const span = reversed ? [last, first] : [first, last];
    doc.draft_subassemblies[0].runs.push({id, catalog:'tubeclamp.tube-C', start_mm:span[0], end_mm:span[1], attachments:[]});
    model.spans[id] = structuredClone(span);
    for (const edge of edges) model.edges.push({member:id, ...edge,
      ...(edge.end ? {end:reversed ? (edge.end === 'start' ? 'end' : 'start') : edge.end} : {})});
  };
  if (family === 'comb') {
    const count = 2 + Math.floor(rng()*4), spacing = 230+rng()*170, length = (count+1)*spacing;
    const through = [];
    for (let i=0; i<count; i++) {
      const catalog = choose(['tubeclamp.TC101C','tubeclamp.TC104C','tubeclamp.TC161C']);
      const part = addPart(`tee-${i}`, catalog, [0,0,(i+1)*spacing]);
      through.push({connector:part.id, port:'through'});
      const port = catalog.endsWith('TC161C') ? 'cross' : 'branch', s = socket(part, port);
      const first = s.definition.through ? s.point.clone().addScaledVector(s.axis,-200) : s.point;
      const last = s.point.clone().addScaledVector(s.axis,400+rng()*250);
      addRun(`branch-${i}`, first.toArray(), last.toArray(), [{connector:part.id,port,...(s.definition.through?{}:{end:'start'})}]);
    }
    addRun('rail', world([0,0,0]), world([0,0,length]), through);
  } else if (family === 'rectangle') {
    const width=500+rng()*650, height=500+rng()*650;
    const corners = [[0,0,0,0],[width,0,0,-90],[width,0,height,180],[0,0,height,90]]
      .map(([x,y,z,turn],i)=>addPart(`corner-${i}`,'tubeclamp.TC125C',[x,y,z],turn));
    for (let i=0;i<4;i++) {
      const a=corners[i], b=corners[(i+1)%4];
      const direction=v(b.pose.position_mm).sub(v(a.pose.position_mm)).normalize();
      const ap=['x','z'].find(port=>socket(a,port).axis.dot(direction)>.99);
      const bp=['x','z'].find(port=>socket(b,port).axis.dot(direction)<-.99);
      assert.ok(ap&&bp,'rectangle construction must have opposed sockets');
      addRun(`edge-${i}`,socket(a,ap).point.toArray(),socket(b,bp).point.toArray(),
        [{connector:a.id,port:ap,end:'start'},{connector:b.id,port:bp,end:'end'}]);
    }
  } else {
    // Actual offset cross fittings: crossing rails occupy separate planes.
    const cols=2+Math.floor(rng()*2), rows=2+Math.floor(rng()*2), dx=350+rng()*150, dz=350+rng()*150;
    const parts=[];
    for(let x=0;x<cols;x++)for(let z=0;z<rows;z++)parts.push(addPart(`cross-${x}-${z}`,'tubeclamp.TC161C',[x*dx,0,z*dz]));
    for(let x=0;x<cols;x++)addRun(`upright-${x}`,world([x*dx,0,-200]),world([x*dx,0,(rows-1)*dz+200]),
      Array.from({length:rows},(_,z)=>({connector:`cross-${x}-${z}`,port:'through'})));
    const offset=library['tubeclamp.TC161C'].ports.cross.position_mm[1];
    for(let z=0;z<rows;z++)addRun(`crossbar-${z}`,world([-200,offset,z*dz]),world([(cols-1)*dx+200,offset,z*dz]),
      Array.from({length:cols},(_,x)=>({connector:`cross-${x}-${z}`,port:'cross'})));
  }
  // A disconnected component detects accidental "edit everything" operations.
  addRun('spare',world([-1500,0,0]),world([-1500,0,600]));
  const build = shuffled(model.edges,rng).map(edge=>({kind:'connect',...edge}));
  const editable = family==='rectangle' ? ['spare'] : Object.keys(model.spans);
  const actions = [...build];
  const choices = ['translate','resize','resize-length','build-from-library','detach-reconnect','undo-redo','duplicate-delete','finalize-reopen','finalize-selected','reload','repair','mirror-cycle'];
  // Mandatory action coverage, shuffled among random repeats. No random chance
  // of silently omitting a whole operation class from an ordinary test run.
  for (const kind of shuffled([...choices,...Array.from({length:steps},()=>choose(choices))],rng)) {
    actions.push({kind, member:choose(kind.startsWith('resize')?editable:Object.keys(model.spans).filter(id=>id!=='spare')),
      axis:Math.floor(rng()*3), delta:choose([-1,1])*(10+Math.floor(rng()*60)),
      edge:choose(model.edges), endpoint:choose(['start','end']), mirrorAxis:choose(['x','y','z'])});
  }
  return {version:1,seed,family,historyLimit:seed%2?10:80,doc,model,actions};
}

export function assertGeometry(scene, model, {exact = false} = {}) {
  const expectedSpans=structuredClone(model.spans), expectedFittings=structuredClone(model.fittings);
  if(exact&&model.mirror) {
    const {axis,offset}=model.mirror, index='xyz'.indexOf(axis);
    const reflect=point=>point.map((value,i)=>i===index?2*offset-value:value);
    for(const [id,span] of Object.entries(model.spans))expectedSpans[`${id}-mirror-mirror-1`]=span.map(reflect);
    for(const [id,pose] of Object.entries(model.fittings))expectedFittings[`${id}-mirror-mirror-1`]={position_mm:reflect(pose.position_mm)};
  }
  const ids=scene.parts.map(p=>p.id);
  assert.equal(new Set(ids).size,ids.length,'duplicate scene IDs');
  assert.deepEqual([...ids].sort(),[...Object.keys(expectedSpans),...Object.keys(expectedFittings)].sort(),'parts were lost or invented');
  for(const [id,span] of Object.entries(expectedSpans))closeSpan(endpoints(scene.parts.find(p=>p.id===id)),span,id);
  for(const [id,pose] of Object.entries(expectedFittings)) {
    const actual=scene.parts.find(p=>p.id===id).pose;
    closePoint(actual.position_mm,pose.position_mm,id);
    if(pose.rotation_deg) {
      const expectedMatrix=matrix(pose),actualMatrix=matrix(actual);
      for(const axis of [[1,0,0],[0,1,0],[0,0,1]])closePoint(v(axis).transformDirection(actualMatrix).toArray(),
        v(axis).transformDirection(expectedMatrix).toArray(),`${id} orientation`,1e-4);
    }
  }
  for(const part of scene.parts)assert.deepEqual(part.conflicts||[],[],`${part.id}: unexpected draft conflict`);
}

export function assertSocketFrames(scene, doc) {
  const parts=new Map(scene.parts.map(part=>[part.id,part]));
  for(const joint of doc.joints||[]) {
    if(joint.type!=='socket')continue;
    const fitting=parts.get(joint.a.part),pipe=parts.get(joint.b.part),socket=fitting.ports[joint.a.port];
    const [first,last]=endpoints(pipe).map(v),direction=last.clone().sub(first).normalize();
    const mouth=v(socket.position_mm).applyMatrix4(matrix(fitting.pose));
    const axis=v(socket.axis).transformDirection(matrix(fitting.pose));
    assert.ok(Math.abs(axis.dot(direction))>1-1e-5,`${joint.id}: socket axis changed`);
    if(socket.through) {
      const station=mouth.clone().sub(first).dot(direction),half=socket.engagement_mm/2,length=first.distanceTo(last);
      closePoint(first.clone().addScaledVector(direction,joint.b.at_mm).toArray(),mouth.toArray(),`${joint.id} station`);
      assert.ok(station>=half-.06&&station<=length-half+.06,`${joint.id}: lost through engagement`);
    } else {
      assert.ok(joint.insertion_mm>=socket.min_engagement_mm-.06&&joint.insertion_mm<=socket.engagement_mm+.06,`${joint.id}: lost end engagement`);
      closePoint((joint.b.end==='start'?first:last).toArray(),mouth.addScaledVector(axis,-joint.insertion_mm).toArray(),`${joint.id} endpoint`);
    }
  }
}

export function assertConnections(doc, expected, {mirror = null, exact = false} = {}) {
  let edges=expected.map(edgeKey);
  if(exact&&mirror)edges=edges.concat(expected.map(e=>edgeKey({...e,member:`${e.member}-mirror-mirror-1`,connector:`${e.connector}-mirror-mirror-1`})));
  assert.deepEqual(connections(doc).map(edgeKey).sort(),edges.sort(),'connections were lost, duplicated or invented');
}

export function component(model, member) {
  const selected=new Set([member]);let changed=true;
  while(changed){changed=false;for(const edge of model.edges)if(selected.has(edge.member)||selected.has(edge.connector))
    for(const id of [edge.member,edge.connector])if(!selected.has(id)){selected.add(id);changed=true;}}
  return selected;
}
export function translateModel(model, member, axis, delta) {
  for(const id of component(model,member)) {
    if(model.spans[id])for(const point of model.spans[id])point[axis]+=delta;
    if(model.fittings[id])model.fittings[id].position_mm[axis]+=delta;
  }
}
