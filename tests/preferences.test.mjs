import test from 'node:test';
import assert from 'node:assert/strict';
import {DEFAULT_PREFERENCES,validatePreferences,loadPreferences,savePreferences,displayValue,storedValue} from '../pipesim/web/preferences.js';
import {DEFAULT_SNAP_SETTINGS,validateSnapSettings,loadSnapDefaults,saveSnapDefaults} from '../pipesim/web/snap-settings.js';

test('preview preferences persist, migrate older settings, and validate quality ranges',()=>{
  const values=new Map(),storage={getItem:key=>values.get(key)||null,setItem:(key,value)=>values.set(key,value)};
  values.set('pipesim.preferences.v1',JSON.stringify({simulationChainLinks:12,simulationSeconds:5}));
  assert.equal(loadPreferences(storage).simulationChainLinks,12);
  assert.equal(loadPreferences(storage).simulationMode,'preview');
  values.set('pipesim.preferences.v1',JSON.stringify({simulationChainLinks:1,simulationSeconds:5}));
  assert.equal(loadPreferences(storage).simulationChainLinks,8);
  assert.equal(loadPreferences(storage).simulationSeconds,5);
  savePreferences(storage,{...DEFAULT_PREFERENCES,simulationChainLinks:1});
  assert.equal(loadPreferences(storage).simulationChainLinks,1);
  const saved=savePreferences(storage,{...DEFAULT_PREFERENCES,simulationMode:'full',simulationChainLinks:6,
    previewStepsPerSecond:2000,previewSolverIterations:480,previewBeamSegmentMm:1000});
  assert.deepEqual(loadPreferences(storage),saved);
  for(const [key,value] of Object.entries({simulationMode:'fast',previewStepsPerSecond:0,previewSolverIterations:1,previewBeamSegmentMm:Infinity}))
    assert.throws(()=>validatePreferences({...saved,[key]:value}),new RegExp(key));
});

test('preferences preserve current editor defaults and validate recovery folder',()=>{
  assert.equal(DEFAULT_PREFERENCES.autosaveEnabled,true);
  assert.equal(DEFAULT_PREFERENCES.startup,'workspace');
  assert.deepEqual([DEFAULT_PREFERENCES.lengthUnit,DEFAULT_PREFERENCES.massUnit,DEFAULT_PREFERENCES.forceUnit],['mm','kg','N']);
  assert.throws(()=>validatePreferences({...DEFAULT_PREFERENCES,autosaveDirectory:'../outside'}),/workspace/);
  assert.throws(()=>validatePreferences({...DEFAULT_PREFERENCES,autosaveKeep:0}),/autosaveKeep/);
});

test('preferences persist and display conversions leave internal values intact',()=>{
  const values=new Map(),storage={getItem:key=>values.get(key)||null,setItem:(key,value)=>values.set(key,value)};
  const saved=savePreferences(storage,{...DEFAULT_PREFERENCES,lengthUnit:'in',massUnit:'lb',forceUnit:'lbf',autosaveEnabled:true});
  assert.deepEqual(loadPreferences(storage),saved);
  assert.equal(displayValue(25.4,'length','in'),1);
  assert.equal(storedValue(1,'length','in'),25.4);
  assert.equal(displayValue(4.4482216152605,'force','lbf'),1);
  values.set('pipesim.preferences.v1','{bad');
  assert.deepEqual(loadPreferences(storage),DEFAULT_PREFERENCES);
});

test('structural deflection warning is persisted in millimetres and rejects invalid limits',()=>{
  const values=new Map(),storage={getItem:key=>values.get(key)||null,setItem:(key,value)=>values.set(key,value)};
  const saved=savePreferences(storage,{...DEFAULT_PREFERENCES,deflectionWarningMm:2.5,lengthUnit:'in'});
  assert.equal(saved.deflectionWarningMm,2.5);
  assert.equal(loadPreferences(storage).deflectionWarningMm,2.5);
  assert.throws(()=>validatePreferences({...saved,deflectionWarningMm:0}),/deflectionWarningMm/);
  assert.throws(()=>validatePreferences({...saved,deflectionWarningMm:Infinity}),/deflectionWarningMm/);
});

test('resize shortcuts are configurable, distinct, and kept in the browser',()=>{
  const values=new Map(),storage={getItem:key=>values.get(key)||null,setItem:(key,value)=>values.set(key,value)};
  const saved=savePreferences(storage,{...DEFAULT_PREFERENCES,resizeKeyStepMm:25,
    resizeHandle1GrowKey:'b',resizeHandle1ShrinkKey:'n',resizeHandle2GrowKey:'m',resizeHandle2ShrinkKey:'k',
    resizeConnectorMode:'detach',resizeShiftMode:'follow',resizeCaptureMm:35});
  assert.equal(loadPreferences(storage).resizeKeyStepMm,25);
  assert.equal(saved.resizeConnectorMode,'detach');
  assert.throws(()=>validatePreferences({...saved,resizeHandle2ShrinkKey:'b'}),/different letters/);
  assert.throws(()=>validatePreferences({...saved,resizeHandle2ShrinkKey:'w'}),/unused letter/);
});

test('keyboard nudge distance has one persisted millimetre source across unit preferences',()=>{
  const values=new Map(),storage={getItem:key=>values.get(key)||null,setItem:(key,value)=>values.set(key,value)};
  assert.equal(DEFAULT_SNAP_SETTINGS.keyboardMoveMm,100);
  assert.equal(Object.hasOwn(DEFAULT_PREFERENCES,'keyboardMoveMm'),false);
  savePreferences(storage,{...DEFAULT_PREFERENCES,lengthUnit:'in'});
  const saved=saveSnapDefaults(storage,{...DEFAULT_SNAP_SETTINGS,keyboardMoveMm:25.4,keyboardRotateDeg:7.5});
  assert.equal(saved.keyboardMoveMm,25.4);
  assert.equal(JSON.parse(values.get('pipesim.snap-defaults.v1')).keyboardMoveMm,25.4);
  assert.equal(Object.hasOwn(JSON.parse(values.get('pipesim.preferences.v1')),'keyboardMoveMm'),false);
  assert.equal(loadSnapDefaults(storage).keyboardMoveMm,25.4);
  assert.equal(loadPreferences(storage).lengthUnit,'in');
  assert.equal(displayValue(loadSnapDefaults(storage).keyboardMoveMm,'length','in'),1);
  assert.equal(storedValue(1,'length','in'),25.4);
  assert.throws(()=>validateSnapSettings({...DEFAULT_SNAP_SETTINGS,keyboardMoveMm:0}),/keyboardMoveMm/);
  assert.throws(()=>validateSnapSettings({...DEFAULT_SNAP_SETTINGS,keyboardMoveMm:10001}),/keyboardMoveMm/);
});
