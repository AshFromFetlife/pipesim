import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp, mkdir, readFile, rm, writeFile} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join, dirname, resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawnSync} from 'node:child_process';

const driver=fileURLToPath(new URL('../scripts/fuzz-editor-workflows.mjs',import.meta.url));
for(const [name,source] of [
  ['all workflow tests were skipped',"import test from 'node:test'; test('unrelated test',()=>{});"],
  ['a workflow claims success without action coverage',`import test from 'node:test';
    test('stateful geometry',()=>{
      console.log('Workflow case_seed=0 family=comb completed_actions=1');
      console.log('Workflow successful actions: {}.');
      console.log('Anchored resize cases_passed=1 keyboard_edits=4');
    });`],
  ['free assembly placement was omitted',`import test from 'node:test';
    test('stateful geometry',()=>{
      console.log('Workflow case_seed=0 family=comb completed_actions=13');
      console.log('Workflow successful actions: '+JSON.stringify(Object.fromEntries(
        ['connect','translate','resize','resize-length','build-from-library','detach-reconnect',
         'undo-redo','duplicate-delete','finalize-reopen','finalize-selected','reload','repair','mirror-cycle'].map(key=>[key,1])))+'.');
      console.log('Anchored resize cases_passed=1 keyboard_edits=4');
    });`],
  ['material changes were omitted',`import test from 'node:test';
    test('stateful geometry',()=>{
      console.log('Workflow case_seed=0 family=comb completed_actions=13');
      console.log('Workflow successful actions: '+JSON.stringify(Object.fromEntries(
        ['connect','translate','resize','resize-length','build-from-library','detach-reconnect',
         'undo-redo','duplicate-delete','finalize-reopen','finalize-selected','reload','repair','mirror-cycle'].map(key=>[key,1])))+'.');
      console.log('Anchored resize cases_passed=1 keyboard_edits=4');
      console.log('Free assembly cases_passed=1 rigid_connections=1');
    });`],
  ['whole assembly movement was omitted',`import test from 'node:test';
    test('stateful geometry',()=>{
      console.log('Workflow case_seed=0 family=comb completed_actions=13');
      console.log('Workflow successful actions: '+JSON.stringify(Object.fromEntries(
        ['connect','translate','resize','resize-length','build-from-library','detach-reconnect',
         'undo-redo','duplicate-delete','finalize-reopen','finalize-selected','reload','repair','mirror-cycle'].map(key=>[key,1])))+'.');
      console.log('Anchored resize cases_passed=1 keyboard_edits=4');
      console.log('Free assembly cases_passed=1 rigid_connections=1');
      console.log('Material cases_passed=1 profile_changes=3');
    });`],
  ['reinforcement connections were omitted',`import test from 'node:test';
    test('stateful geometry',()=>{
      console.log('Workflow case_seed=0 family=comb completed_actions=13');
      console.log('Workflow successful actions: '+JSON.stringify(Object.fromEntries(
        ['connect','translate','resize','resize-length','build-from-library','detach-reconnect',
         'undo-redo','duplicate-delete','finalize-reopen','finalize-selected','reload','repair','mirror-cycle'].map(key=>[key,1])))+'.');
      console.log('Anchored resize cases_passed=1 keyboard_edits=4');
      console.log('Free assembly cases_passed=1 rigid_connections=1');
      console.log('Material cases_passed=1 profile_changes=3');
      console.log('Whole assembly cases_passed=1 rigid_moves=2');
    });`],
])test(`campaign fails when ${name}`,async()=>{
  const directory=await mkdtemp(join(tmpdir(),'pipesim-campaign-oracle-'));
  try {
    await mkdir(join(directory,'tests'));
    await writeFile(join(directory,'tests','editor-interactions.test.mjs'),source);
    const env={...process.env,PIPESIM_WORKFLOW_CASES:'1',PIPESIM_WORKFLOW_STEPS:'0',
      PIPESIM_WORKFLOW_SEED:'42',PIPESIM_WORKFLOW_MINUTES:'0.01',
      PIPESIM_WORKFLOW_REPRO_DIR:join(directory,'report')};
    for(const key of ['PIPESIM_WORKFLOW_REPLAY','PIPESIM_WORKFLOW_MINIMIZE','PIPESIM_ANCHORED_RESIZE_REPLAY','PIPESIM_FREE_ASSEMBLY_REPLAY','PIPESIM_MATERIAL_REPLAY','PIPESIM_WHOLE_ASSEMBLY_REPLAY','PIPESIM_REINFORCEMENT_REPLAY'])delete env[key];
    const result=spawnSync(process.execPath,[driver],{cwd:directory,env,encoding:'utf8',windowsHide:true,timeout:30000});
    assert.equal(result.status,1,`${result.stdout}\n${result.stderr}`);
    const report=JSON.parse(await readFile(join(directory,'report','campaign.json'),'utf8'));
    assert.equal(report.cases[0].exitCode,0,'the child succeeded, so the driver must detect missing coverage');
    assert.equal(report.cases[0].status,'failed');
    assert.match(report.cases[0].failure,/coverage report/);
  } finally {
    assert.equal(dirname(resolve(directory)),resolve(tmpdir()));
    await rm(directory,{recursive:true,force:true});
  }
});
