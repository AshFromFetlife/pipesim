import assert from 'node:assert/strict';
import {randomBytes} from 'node:crypto';
import {mkdir, readFile, writeFile} from 'node:fs/promises';
import {resolve, join} from 'node:path';
import {DEFAULT_PREFERENCES} from '../../pipesim/web/preferences.js';
import {generate, runs, connections, edgeKey, endpoints, v, closeSpan,
  assertGeometry, assertConnections, assertSocketFrames, translateModel, component, random, matrix} from './geometry-workflow-model.mjs';

const REQUIRED = ['connect','translate','resize','resize-length','build-from-library','detach-reconnect','undo-redo',
  'duplicate-delete','finalize-reopen','finalize-selected','reload','repair','mirror-cycle'];
const integer = (name, fallback, min=1) => {
  const value=Number(process.env[name]??fallback);
  assert.ok(Number.isSafeInteger(value)&&value>=min,`${name} must be an integer >= ${min}`);
  return value;
};
const cleanDoc = doc => {const result=JSON.parse(JSON.stringify(doc));delete result.results;return result;};

// Dependencies are the same DOM harness and real local service used by the
// interaction regressions. This module never patches application state to edit.
export function registerGeometryWorkflows({test, editor, post, library}) {
  async function runCase(spec, {fault=null, retain=true}={}) {
    let ui, previousEdit, stage='open', completed=0, failed=false;
    const model=structuredClone(spec.model), observed=[], requests=[], messages=[], failures=[], coverage={}, trace=[], requested=[];
    const historyLimit=spec.historyLimit??DEFAULT_PREFERENCES.undoLimit;
    const options={
      storedPreferences:JSON.stringify({...DEFAULT_PREFERENCES,undoLimit:historyLimit}),
      requestTimeoutMs:60000,
      onRequest(path,body){requests.push({path,body});},
      onError(error){failures.push({kind:'runtime',message:String(error),stack:error?.stack});},
      onToast(text,error){if(error)failures.push({kind:'toast',text});messages.push({text,error});},
      onResponse(path,status,data){
        if(path.startsWith('/api/'))observed.push({path,status,data});
        if(status>=400||data?.status==='conflict'||data?.mirror_pose_error)
          failures.push({kind:'response',path,status,data});
      },
      interceptFetch(path){
        if(fault==='reject-finalize'&&path==='/api/draft-finalize')return Promise.resolve(new Response(JSON.stringify({status:'conflict',conflicts:[{code:'INJECTED',message:'Injected false-positive rejection'}]}),{status:200}));
        return null;
      },
    };
    const click=selector=>{const element=ui.document.querySelector(selector);assert.ok(element,`missing control ${selector}`);assert.ok(!element.disabled,`disabled control ${selector}`);element.click();};
    const change=(selector,value,event='change')=>{const input=ui.document.querySelector(selector);assert.ok(input,`missing input ${selector}`);assert.ok(!input.disabled,`disabled input ${selector}`);input.value=String(value);input.dispatchEvent(new ui.window.Event(event,{bubbles:true}));};
    async function settled() {
      await ui.drain();await ui.wait(()=>!ui.state.placementPending&&!ui.state.busy,60000);await ui.drain();ui.tick();
      assert.deepEqual(failures,[],'unexpected user-visible or API rejection');
      assert.ok(!ui.document.querySelector('#modal').open,
        `blocking dialog: ${ui.document.querySelector('#modal-title').textContent} / ${ui.document.querySelector('#modal').textContent}`);
    }
    const select=id=>{click('[data-tool="select"]');ui.document.querySelector(`[data-select="${id}"]`)?.focus();ui.select(id);};
    const check=({full=true,exact=false}={})=>{
      assertGeometry(ui.state.scene,model,{exact});
      if(full)assertConnections(ui.state.doc,model.edges,{mirror:model.mirror,exact});
      else {
        const expected=new Set(model.edges.map(edgeKey)),actual=connections(ui.state.doc).map(edgeKey);
        assert.equal(new Set(actual).size,actual.length,'duplicate connection during construction');
        assert.ok(actual.every(e=>expected.has(e)),`unexpected connection during construction: ${actual}`);
        assert.ok(requested.every(e=>actual.includes(edgeKey(e))),'requested connection was lost during construction');
      }
    };
    async function connect(edge) {
      if(connections(ui.state.doc).some(e=>edgeKey(e)===edgeKey(edge)))return false; // explicitly recorded automatic capture
      select(edge.member);click('#draft-connect');ui.select(edge.connector);
      if(ui.document.querySelector('#modal').open) {
        assert.equal(ui.document.querySelector('#modal-title').textContent,'Connect draft run');
        change('#draft-port',edge.port);
        if(edge.end)change('#draft-end',edge.end);
        click('#modal-actions .primary');
      }
      await settled();
      assert.ok(connections(ui.state.doc).some(e=>edgeKey(e)===edgeKey(edge)),`requested connection missing: ${edgeKey(edge)}`);
      return true;
    }
    async function finalize() {
      click('#finalize-draft');await settled();
      assert.equal(runs(ui.state.doc).length,0,'finalization left draft pipes');
      check({exact:true});
      assertSocketFrames(ui.state.scene,ui.state.doc);
      const report=await post('validate',ui.state.doc);
      assert.equal(report.valid,true,`exact validation including collisions: ${JSON.stringify(report.issues)}`);
    }
    async function reopen() {
      for(const id of Object.keys(model.spans)) {
        if(runs(ui.state.doc).some(run=>run.id===id))continue;
        const part=ui.document.querySelector(`[data-select="${id}"]`);assert.ok(part,`missing exact pipe ${id}`);
        const toggle=part.closest('details').querySelector('summary [data-tree-actions]');
        assert.ok(toggle);toggle.click();click('[data-tree-command="reopen"]');await settled();
      }
      check();
    }
    async function undoRedo(before,after) {
      click('#undo');await settled();assert.deepEqual(cleanDoc(ui.state.doc),cleanDoc(before),'Undo did not restore the whole document');
      click('#redo');await settled();assert.deepEqual(cleanDoc(ui.state.doc),cleanDoc(after),'Redo did not restore the whole document');
    }
    async function action(command) {
      const {kind,member}=command;
      if(kind==='connect')return await connect(command);
      if(kind==='translate') {
        select(member);
        const position=ui.state.scene.parts.find(p=>p.id===member).pose.position_mm;
        change(`[data-pose="position_mm"][data-axis="${command.axis}"]`,position[command.axis]+command.delta);
        translateModel(model,member,command.axis,command.delta);
      } else if(kind==='resize'||kind==='resize-length') {
        select(member);
        const span=model.spans[member],direction=v(span[1]).sub(v(span[0])).normalize();
        const attachments=model.edges.filter(e=>e.member===member),bound=attachments.find(e=>e.end)?.end;
        const endpoint=bound?(bound==='start'?'end':'start'):kind==='resize-length'?'end':command.endpoint;
        // Bound repeated cuts using the independent socket frames. Total pipe
        // length alone is insufficient: one end can reach an interior support
        // even while most of the pipe extends past another support.
        const length=v(span[0]).distanceTo(v(span[1]));
        let delta=command.delta>0||length<500?10:-10;
        if(delta<0)for(const edge of attachments) {
          const fitting=spec.doc.parts.find(p=>p.id===edge.connector);
          const socket=library()[fitting.catalog].ports[edge.port];
          if(!socket.through)continue;
          const mouth=v(socket.position_mm).applyMatrix4(matrix(model.fittings[edge.connector]));
          const station=mouth.sub(v(span[0])).dot(direction);
          const available=(endpoint==='start'?station:length-station)-socket.engagement_mm/2;
          if(available<10+.06)delta=10;
        }
        if(kind==='resize-length')change('#draft-length',length+delta);
        else ui.viewport.dispatchEvent(new ui.window.KeyboardEvent('keydown',{key:endpoint==='start'?(delta>0?'y':'h'):(delta>0?'u':'j'),bubbles:true}));
        const index=endpoint==='start'?0:1;
        span[index]=v(span[index]).addScaledVector(direction,(index?1:-1)*delta).toArray();
      } else if(kind==='build-from-library') {
        const number=(model.added||0)+1;model.added=number;
        const position=[5000+number*2500,3000,2500];
        change('#category','member');click('[data-catalog="tubeclamp.tube-C"]');await settled();
        const pipe=ui.state.selected,length=library()['tubeclamp.tube-C'].parameters.length_mm;
        assert.ok(!model.spans[pipe]);
        select(pipe);
        for(let axis=0;axis<3;axis++)change(`[data-pose="position_mm"][data-axis="${axis}"]`,position[axis]);
        model.spans[pipe]=[-1,1].map(sign=>[position[0],position[1],position[2]+sign*length/2]);
        change('#category','connector');click('[data-catalog="tubeclamp.TC101C"]');await settled();
        const connector=ui.state.selected;assert.ok(!model.fittings[connector]);
        select(connector);
        for(let axis=0;axis<3;axis++){change(`[data-pose="position_mm"][data-axis="${axis}"]`,position[axis]);await settled();select(connector);}
        model.fittings[connector]={position_mm:[...position],rotation_deg:[0,0,0]};
        const edge={member:pipe,connector,port:'through'};model.edges.push(edge);await connect(edge);
      } else if(kind==='detach-reconnect') {
        const edge=command.edge;select(edge.member);
        const run=runs(ui.state.doc).find(r=>r.id===edge.member);
        const index=run.attachments.findIndex(a=>a.connector===edge.connector&&a.port===edge.port);
        assert.ok(index>=0);click(`[data-draft-detach="${index}"]`);await settled();
        assert.ok(!connections(ui.state.doc).some(e=>edgeKey(e)===edgeKey(edge)),'detach was ignored');
        await connect(edge);
      } else if(kind==='undo-redo') {
        assert.ok(previousEdit,'no edit available to exercise Undo');
        await undoRedo(previousEdit.before,previousEdit.after);
      } else if(kind==='duplicate-delete') {
        select(member);click('#duplicate-menu-toggle');click('[data-duplicate="count"]');
        change('#duplicate-count',1,'input');change('#duplicate-scope','part');
        for(let axis=0;axis<3;axis++)change(`#duplicate-offset-${axis}`,axis===command.axis?5000:0,'input');
        const before=structuredClone(ui.state.doc);
        click('#modal-actions .primary');await settled();
        const copy=ui.state.selected;
        assert.ok(!model.spans[copy]&&copy!==member,'duplicate did not create a new pipe');
        const expected=model.spans[member].map(p=>p.map((value,i)=>value+(i===command.axis?5000:0)));
        closeSpan(endpoints(ui.state.scene.parts.find(p=>p.id===copy)),expected,'duplicate');
        assert.deepEqual(runs(ui.state.doc).find(r=>r.id===copy).attachments,[]);
        const copied=structuredClone(ui.state.doc);await undoRedo(before,copied);
        select(copy);click('#draft-delete');await settled();
        assert.ok(!runs(ui.state.doc).some(r=>r.id===copy),'delete was ignored');
      } else if(kind==='finalize-reopen') {
        await finalize();await reopen();
      } else if(kind==='finalize-selected') {
        const selected=component(model,member);
        const remaining=Object.keys(model.spans).filter(id=>!selected.has(id)).sort();
        select(member);click('#draft-finalize-selected');await settled();
        assert.deepEqual(runs(ui.state.doc).map(r=>r.id).sort(),remaining,'selected finalization touched another component');
        check();assertSocketFrames(ui.state.scene,ui.state.doc);await reopen();
      } else if(kind==='repair') {
        select(member);click('#draft-repair-selected');await settled();
      } else if(kind==='reload') {
        const document=structuredClone(ui.state.doc);
        // Exercise the parser/serialization boundary as well as fresh caches.
        const parsed=await post('parse',document,{text:JSON.stringify(document)});
        await ui.close();ui=await editor(parsed.document,null,options);await settled();
        assert.deepEqual(cleanDoc(ui.state.doc),cleanDoc(document),'reload changed the authored document');
        previousEdit=null;
      } else if(kind==='mirror-cycle') {
        select(member);click('#draft-mirror-add');
        const axis=command.mirrorAxis,offset=-10000;
        change('#mirror-axis',axis);change('#mirror-offset',offset);click('#modal-actions .primary');
        await settled();model.mirror={axis,offset};check();
        await finalize();await reopen();
        select(member);click('[data-mirror-remove]');
        // Discard previews after proving that they materialize and reopen.
        const discard=[...ui.document.querySelectorAll('#modal-actions button')].find(b=>/discard/i.test(b.textContent));
        assert.ok(discard,'missing Discard mirror action');discard.click();await settled();model.mirror=null;
      } else throw new Error(`Unknown workflow action ${kind}`);
      await settled();return true;
    }
    try {
      ui=await editor(structuredClone(spec.doc),null,options);await settled();check({full:false});
      for(const [index,command] of spec.actions.entries()) {
        stage=`${index}:${command.kind}`;
        // Reload clears Undo by design. Generate a real edit before testing it
        // instead of counting an unavailable Undo as coverage.
        if(command.kind==='undo-redo'&&!previousEdit) {
          const before=structuredClone(ui.state.doc);
          await action({...command,kind:'translate'});check();
          previousEdit={before,after:structuredClone(ui.state.doc)};
        }
        const before=structuredClone(ui.state.doc),undo=ui.state.undo.length,undoTail=ui.state.undo.at(-1);
        trace.push({command,beforeDocument:before,expected:structuredClone(model),undoEntries:undo});
        const performed=await action(command);
        if(command.kind==='connect')requested.push(command);
        if(performed&&['connect','translate','resize','resize-length'].includes(command.kind)) {
          assert.equal(ui.state.undo.length,Math.min(undo+1,historyLimit),'edit must create one undo entry');
          assert.deepEqual(cleanDoc(ui.state.undo.at(-1)),cleanDoc(before),'undo entry must contain the document before the edit');
        }
        if(performed)coverage[command.kind]=(coverage[command.kind]||0)+1;
        else trace.at(-1).automaticCapture=true;
        if(ui.state.undo.length&&ui.state.undo.at(-1)!==undoTail)
          previousEdit={before:structuredClone(ui.state.undo.at(-1)),after:structuredClone(ui.state.doc)};
        if(fault==='lost-connection'&&command.kind==='connect'&&performed) {
          // Fault injection lives only in this test harness. Production output
          // is deliberately corrupted to prove the oracle rejects it.
          runs(ui.state.doc).find(r=>r.attachments?.length)?.attachments.pop();
        }
        if(fault==='wrong-length'&&command.kind==='resize')ui.state.scene.parts.find(p=>p.id===command.member).length_mm+=20;
        check({full:command.kind!=='connect'});completed=index+1;
      }
      stage='final validation';await finalize();
      return {coverage,completed};
    } catch(error) {
      failed=true;
      if(retain) {
        const parent=resolve(process.env.PIPESIM_WORKFLOW_REPRO_DIR||'fuzz-runs/editor-workflows');
        await mkdir(parent,{recursive:true});
        const folder=join(parent,`${spec.seed}-${Date.now()}`);await mkdir(folder);
        const replay={...spec,actions:spec.actions.slice(0,Math.min(spec.actions.length,completed+1))};
        await writeFile(join(folder,'replay.json'),JSON.stringify(replay,null,2));
        await writeFile(join(folder,'failure.json'),JSON.stringify({stage,error:error.stack,completed,coverage,
          currentDocument:ui?.state.doc,expected:model,messages,failures,trace,requests,responses:observed},null,2));
        throw new Error(`Workflow case_seed=${spec.seed} family=${spec.family} stage=${stage}; replay: PIPESIM_WORKFLOW_REPLAY=${join(folder,'replay.json')}\n${error.message}`,{cause:error});
      }
      throw error;
    } finally {
      try{await ui?.close();}catch(error){if(!failed)throw error;console.error(`Workflow ${spec.seed} cleanup also failed: ${error.message}`);}
    }
  }

  test('stateful geometry editor workflows preserve feasible designs through mixed edits',async()=>{
    const replay=process.env.PIPESIM_WORKFLOW_REPLAY;
    if(replay){
      const spec=JSON.parse(await readFile(replay,'utf8'));
      assert.equal(spec.version,1,'unsupported workflow replay version');
      if(!process.env.PIPESIM_WORKFLOW_MINIMIZE){await runCase(spec);return;}
      const signature=error=>{
        while(error.cause)error=error.cause;
        const message=error.message.split('\n')[0].replace(/-?\d+(?:\.\d+)?/g,'#');
        // Many different application bugs reach the same "API rejection"
        // assertion. Keep the failing endpoint, issue code and affected parts
        // so reduction cannot substitute a different rejection for this one.
        const rejections=Array.isArray(error.actual)?error.actual.filter(item=>item?.kind==='response').map(item=>({
          path:item.path,status:item.status,error:item.data?.error,
          conflicts:item.data?.conflicts?.map(issue=>({code:issue.code,parts:issue.parts}))
        })):[];
        return `${error.constructor.name}: ${message} ${JSON.stringify(rejections)}`;
      };
      let failure;
      try{await runCase(spec,{retain:false});}catch(error){failure=signature(error);}
      assert.ok(failure,'the supplied replay no longer fails');
      // Preserve construction and the failing final action. Reduce only the
      // intervening workflow, and accept only the same failure signature.
      let reduced=spec,attempts=0;
      const deadline=Date.now()+integer('PIPESIM_WORKFLOW_MINIMIZE_SECONDS',180)*1000;
      for(let width=Math.ceil(spec.actions.length/2);width>=1;width=Math.floor(width/2)) {
        for(let index=0;index<reduced.actions.length-1&&Date.now()<deadline;index++) {
          const end=Math.min(index+width,reduced.actions.length-1);
          if(reduced.actions.slice(index,end).some(a=>a.kind==='connect'))continue;
          const candidate={...reduced,actions:reduced.actions.filter((_,i)=>i<index||i>=end)};
          attempts++;
          try{await runCase(candidate,{retain:false});}catch(error){if(signature(error)===failure){reduced=candidate;index--;}}
        }
      }
      const target=resolve(replay+'.reduced.json');
      await writeFile(target,JSON.stringify(reduced,null,2));
      assert.fail(`Workflow remains failing (${failure}); reduced ${spec.actions.length} to ${reduced.actions.length} actions in ${attempts} attempts; replay=${target}`);
    }
    const caseSeed=process.env.PIPESIM_WORKFLOW_CASE_SEED;
    const root=integer('PIPESIM_WORKFLOW_SEED',randomBytes(4).readUInt32LE(),0);
    assert.ok(root<=0xffffffff,'PIPESIM_WORKFLOW_SEED must be a uint32');
    const count=integer('PIPESIM_WORKFLOW_CASES',12,0),steps=integer('PIPESIM_WORKFLOW_STEPS',18,0);
    const rng=random(root), seeds=caseSeed?[integer('PIPESIM_WORKFLOW_CASE_SEED',0,0)]:[0,1,2,3,4,5,...Array.from({length:count},()=>Math.floor(rng()*0x100000000))];
    const totals={},families=new Set();
    console.log(`Geometry editor workflow root_seed=${root}, cases=${seeds.length}, random_steps=${steps}`);
    for(const seed of seeds) {
      const spec=generate(seed,library(),{steps});families.add(spec.family);
      const {coverage,completed}=await runCase(spec);
      for(const kind of REQUIRED)assert.ok(coverage[kind]>0,`${seed}: no successful ${kind}; coverage=${JSON.stringify(coverage)}`);
      for(const [key,value] of Object.entries(coverage))totals[key]=(totals[key]||0)+value;
      console.log(`Workflow case_seed=${seed} family=${spec.family} completed_actions=${completed}`);
    }
    if(!caseSeed)assert.equal(families.size,3,'a topology family was omitted');
    console.log(`Workflow successful actions: ${JSON.stringify(totals)}. This covers generated workflows, not a claim of editor stability.`);
  });

  test('geometry workflow oracles catch false rejections, dropped connections and wrong movement',async()=>{
    for(const [fault,kind,pattern] of [['reject-finalize','finalize-reopen',/blocking dialog|unexpected.*rejection/],
      ['lost-connection','connect',/connection/],['wrong-length','resize',/displaced/]]) {
      const spec=generate(0,library(),{steps:0});
      spec.actions=spec.actions.filter(a=>a.kind==='connect'||a.kind===kind);
      await assert.rejects(runCase(spec,{fault,retain:false}),pattern,`the workflow oracle survived ${fault}`);
    }
  });

  test('retained geometry workflow failures cover browser mirror reopening, free grid cuts and socket collisions',async()=>{
    for(const name of ['browser-mirror-roundtrip','grid-free-end-shrink','mirrored-offset-socket']) {
      const spec=JSON.parse(await readFile(`tests/fixtures/workflows/${name}.json`,'utf8'));
      await runCase(spec);
    }
  });
}
