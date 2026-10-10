// Fresh processes bound retained DOM/native geometry memory during a long run.
import {spawn} from 'node:child_process';
import {randomBytes} from 'node:crypto';
import {mkdir, writeFile} from 'node:fs/promises';
import {resolve, join} from 'node:path';
import {random} from '../tests/helpers/geometry-workflow-model.mjs';

const minutes=Number(process.env.PIPESIM_WORKFLOW_MINUTES??process.env.PIPESIM_FUZZ_MINUTES??45);
const rootSeed=Number(process.env.PIPESIM_WORKFLOW_SEED??randomBytes(4).readUInt32LE());
const limit=Number(process.env.PIPESIM_WORKFLOW_CASES??1000000);
const requiredActions=['connect','translate','resize','resize-length','build-from-library','detach-reconnect',
  'undo-redo','duplicate-delete','finalize-reopen','finalize-selected','reload','repair','mirror-cycle'];
for(const name of ['PIPESIM_WORKFLOW_REPLAY','PIPESIM_WORKFLOW_MINIMIZE','PIPESIM_ANCHORED_RESIZE_REPLAY','PIPESIM_FREE_ASSEMBLY_REPLAY','PIPESIM_MATERIAL_REPLAY','PIPESIM_WHOLE_ASSEMBLY_REPLAY','PIPESIM_REINFORCEMENT_REPLAY'])
  if(process.env[name])throw new Error(`${name} selects a replay, not a fresh campaign; unset it or run the individual test directly`);
if(!Number.isFinite(minutes)||minutes<=0)throw new Error('PIPESIM_WORKFLOW_MINUTES must be positive and finite');
if(!Number.isInteger(rootSeed)||rootSeed<0||rootSeed>0xffffffff)throw new Error('PIPESIM_WORKFLOW_SEED must be a uint32');
if(!Number.isSafeInteger(limit)||limit<=0)throw new Error('PIPESIM_WORKFLOW_CASES must be a positive integer');
const directory=resolve(process.env.PIPESIM_WORKFLOW_REPRO_DIR||`fuzz-runs/editor-campaign-${rootSeed}-${Date.now()}`);
await mkdir(directory,{recursive:true});
const rng=random(rootSeed),started=Date.now(),cases=[];
const retainedSeeds=[0,1,2,3,4,5,6,7,8,16,24,1112331263];
console.log(`Editor workflow campaign root_seed=${rootSeed} budget_minutes=${minutes} artifacts=${directory}`);
const save=()=>writeFile(join(directory,'campaign.json'),JSON.stringify({rootSeed,minutes,startedAt:new Date(started).toISOString(),elapsedSeconds:(Date.now()-started)/1000,cases},null,2));
for(let index=0;index<limit&&(index===0||Date.now()-started<minutes*60000);index++) {
  const seed=index<retainedSeeds.length?retainedSeeds[index]:Math.floor(rng()*0x100000000);
  const steps=Number(process.env.PIPESIM_WORKFLOW_STEPS??(18+Math.floor(rng()*43)));
  const entry={seed,steps,status:'running'};cases.push(entry);await save();
  console.log(`Starting case ${index+1}, seed=${seed}, additional_actions=${steps}`);
  let output='';
  const child=spawn(process.execPath,['--test','--test-name-pattern=stateful geometry|seeded anchored resize|seeded free assembly|seeded material|seeded whole assembly|seeded reinforcement','tests/editor-interactions.test.mjs'],{
    cwd:process.cwd(),windowsHide:true,env:{...process.env,PIPESIM_WORKFLOW_CASE_SEED:String(seed),
      PIPESIM_WORKFLOW_SEED:String(rootSeed),PIPESIM_WORKFLOW_STEPS:String(steps),PIPESIM_WORKFLOW_REPRO_DIR:directory,
      PIPESIM_ANCHORED_RESIZE_CASE_SEED:String(seed),PIPESIM_ANCHORED_RESIZE_SEED:String(rootSeed),
      PIPESIM_FREE_ASSEMBLY_CASE_SEED:String(seed),PIPESIM_FREE_ASSEMBLY_SEED:String(rootSeed),
      PIPESIM_MATERIAL_CASE_SEED:String(seed),PIPESIM_MATERIAL_SEED:String(rootSeed),
      PIPESIM_WHOLE_ASSEMBLY_CASE_SEED:String(seed),PIPESIM_WHOLE_ASSEMBLY_SEED:String(rootSeed),
      PIPESIM_REINFORCEMENT_CASE_SEED:String(seed),PIPESIM_REINFORCEMENT_SEED:String(rootSeed)},
    stdio:['ignore','pipe','pipe']});
  for(const stream of [child.stdout,child.stderr])stream.on('data',chunk=>{output+=chunk;process.stdout.write(chunk);});
  const code=await new Promise((resolve,reject)=>{child.on('error',reject);child.on('exit',resolve);});
  entry.status=code===0?'passed':'failed';entry.exitCode=code;
  const completed=output.match(/Workflow case_seed=(\d+) family=(\S+) completed_actions=(\d+)/);
  const actions=output.match(/Workflow successful actions: (\{[^\r\n]*\})\./);
  if(completed){entry.family=completed[2];entry.completedActions=Number(completed[3]);}
  if(actions)entry.successfulActions=JSON.parse(actions[1]);
  const anchored=output.match(/Anchored resize cases_passed=(\d+) keyboard_edits=(\d+)/);
  if(anchored){entry.anchoredResizeCases=Number(anchored[1]);entry.anchoredResizeEdits=Number(anchored[2]);}
  const free=output.match(/Free assembly cases_passed=(\d+) rigid_connections=(\d+)/);
  if(free){entry.freeAssemblyCases=Number(free[1]);entry.rigidConnections=Number(free[2]);}
  const material=output.match(/Material cases_passed=(\d+) profile_changes=(\d+)/);
  if(material){entry.materialCases=Number(material[1]);entry.profileChanges=Number(material[2]);}
  const whole=output.match(/Whole assembly cases_passed=(\d+) rigid_moves=(\d+)/);
  if(whole){entry.wholeAssemblyCases=Number(whole[1]);entry.wholeAssemblyMoves=Number(whole[2]);}
  const reinforcement=output.match(/Reinforcement cases_passed=(\d+) connections=(\d+)/);
  if(reinforcement){entry.reinforcementCases=Number(reinforcement[1]);entry.reinforcementConnections=Number(reinforcement[2]);}
  // A zero exit status alone also describes skipped tests or a stale filter.
  // Require evidence that all requested workflows actually ran this seed.
  if(code===0&&(!completed||Number(completed[1])!==seed||!entry.completedActions||
      !entry.successfulActions||requiredActions.some(action=>!(entry.successfulActions[action]>0))||
      entry.anchoredResizeCases!==1||entry.anchoredResizeEdits!==4||
      entry.freeAssemblyCases!==1||entry.rigidConnections!==1||entry.materialCases!==1||entry.profileChanges!==3||
      entry.wholeAssemblyCases!==1||entry.wholeAssemblyMoves!==2||entry.reinforcementCases!==1||entry.reinforcementConnections!==1)) {
    entry.status='failed';entry.failure='Missing or incomplete workflow coverage report';
    console.error(entry.failure);
  }
  await writeFile(join(directory,`case-${index}-${seed}.log`),output);await save();
  if(entry.status==='failed'){process.exitCode=1;break;}
}
console.log(`Editor workflow campaign completed ${cases.filter(c=>c.status==='passed').length} cases; ${cases.filter(c=>c.status==='failed').length} failed. Scope and replay data: ${join(directory,'campaign.json')}`);
