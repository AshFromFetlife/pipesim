export const DEFAULT_PREFERENCES=Object.freeze({
  autosaveEnabled:true,autosaveDirectory:'designs/.autosaves',autosaveMinutes:5,autosaveKeep:10,
  startup:'workspace',startupExample:'',lengthUnit:'mm',massUnit:'kg',forceUnit:'N',
  gridVisible:true,floorVisible:true,portsVisible:false,theme:'system',undoLimit:80,defaultTool:'select',fitOnOpen:true,fitAfterDuplicate:true,
  cameraFovDeg:38,defaultDuplicateCount:2,defaultHumanHeightMm:1750,defaultHumanMassKg:75,defaultChainLengthMm:1000,
  renderWidth:1600,renderHeight:1000,renderLighting:'studio',renderBackground:'#edf1f3',
  simulationSeconds:3,simulationChainLinks:1,deflectionWarningMm:10,connectionLocked:true,connectionToleranceMm:2,
  resizeConnectorMode:'follow',resizeShiftMode:'detach',resizeAutoConnect:true,
  resizeCaptureMm:40,resizeCaptureDeg:15,resizeKeyStepMm:10,
  resizeHandle1GrowKey:'y',resizeHandle1ShrinkKey:'h',resizeHandle2GrowKey:'u',resizeHandle2ShrinkKey:'j'
});
const KEY='pipesim.preferences.v1';
const choices={startup:['workspace','autosave','empty','example'],lengthUnit:['mm','cm','m','in','ft'],
  massUnit:['kg','g','lb'],forceUnit:['N','kN','lbf'],theme:['system','light','dark'],
  defaultTool:['select','translate','rotate','resize','connect'],renderLighting:['studio','technical','flat'],
  resizeConnectorMode:['follow','detach'],resizeShiftMode:['follow','detach'],
  renderBackground:['#edf1f3','#ffffff','transparent']};
const ranges={autosaveMinutes:[1,120],autosaveKeep:[1,100],undoLimit:[10,500],cameraFovDeg:[20,90],
  defaultDuplicateCount:[1,100],defaultHumanHeightMm:[500,2500],defaultHumanMassKg:[10,250],defaultChainLengthMm:[1,100000],
  renderWidth:[320,4096],renderHeight:[240,4096],simulationSeconds:[.1,30],
  simulationChainLinks:[1,1000],deflectionWarningMm:[.01,100000],connectionToleranceMm:[.01,20],
  resizeCaptureMm:[0,100],resizeCaptureDeg:[0,45],resizeKeyStepMm:[.1,1000]};
const scales={length:{mm:1,cm:10,m:1000,in:25.4,ft:304.8},
  mass:{kg:1,g:.001,lb:.45359237},force:{N:1,kN:1000,lbf:4.4482216152605}};
export function validatePreferences(input){
  const result={...DEFAULT_PREFERENCES};
  for(const key of ['autosaveEnabled','gridVisible','floorVisible','portsVisible','fitOnOpen','fitAfterDuplicate','connectionLocked','resizeAutoConnect']){
    if(typeof input[key]!=='boolean')throw new Error(`${key} must be on or off`);
    result[key]=input[key];
  }
  for(const [key,values] of Object.entries(choices)){
    if(!values.includes(input[key]))throw new Error(`Choose a valid ${key}`);
    result[key]=input[key];
  }
  for(const [key,[min,max]] of Object.entries(ranges)){
    const value=Number(input[key]);
    if(!Number.isFinite(value)||value<min||value>max||
      (key!=='simulationSeconds'&&key!=='deflectionWarningMm'&&key!=='connectionToleranceMm'&&key!=='defaultHumanMassKg'&&
       key!=='resizeCaptureMm'&&key!=='resizeCaptureDeg'&&key!=='resizeKeyStepMm'&&!Number.isInteger(value)))
      throw new Error(`${key} must be between ${min} and ${max}`);
    result[key]=value;
  }
  const directory=String(input.autosaveDirectory||'').replaceAll('\\','/').replace(/\/+$/,'');
  if(!directory||directory.startsWith('/')||directory.split('/').some(p=>!p||['.','..','.git','.agents','.codex'].includes(p.toLowerCase()))||
    /^[A-Za-z]:/.test(directory))throw new Error('Choose a folder inside the workspace');
  result.autosaveDirectory=directory;
  result.startupExample=typeof input.startupExample==='string'?input.startupExample:'';
  const keys=['resizeHandle1GrowKey','resizeHandle1ShrinkKey','resizeHandle2GrowKey','resizeHandle2ShrinkKey'];
  const reserved=new Set('wasdfvzxcrgq'.split(''));
  for(const key of keys){
    const value=String(input[key]||'').toLowerCase();
    if(!/^[a-z]$/.test(value)||reserved.has(value))throw new Error('Choose an unused letter for each resize shortcut');
    result[key]=value;
  }
  if(new Set(keys.map(key=>result[key])).size!==keys.length)throw new Error('Resize shortcuts must be different letters');
  return result;
}
export function loadPreferences(storage){
  try{return validatePreferences({...DEFAULT_PREFERENCES,...JSON.parse(storage.getItem(KEY)||'{}')});}
  catch{return {...DEFAULT_PREFERENCES};}
}
export function savePreferences(storage,input){
  const result=validatePreferences(input);storage.setItem(KEY,JSON.stringify(result));return result;
}
export function displayValue(value,kind,unit){return Number((value/scales[kind][unit]).toFixed(4));}
export function storedValue(value,kind,unit){return Number(value)*scales[kind][unit];}
