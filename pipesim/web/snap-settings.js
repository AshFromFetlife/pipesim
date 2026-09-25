export const DEFAULT_SNAP_SETTINGS=Object.freeze({
  gridEnabled:true,translationMm:10,rotationDeg:90,
  connectionsEnabled:true,connectionPixels:30,alignEnabled:true,alignmentDeg:6
});
const key='pipesim.snap-defaults.v1';
const ranges={translationMm:[.1,10000],rotationDeg:[.1,180],connectionPixels:[4,100],alignmentDeg:[.1,20]};

export function validateSnapSettings(settings){
  const result={...DEFAULT_SNAP_SETTINGS};
  for(const name of ['gridEnabled','connectionsEnabled','alignEnabled']){
    if(typeof settings[name]!=='boolean')throw new Error('Choose which snapping modes to enable');
    result[name]=settings[name];
  }
  for(const [name,[min,max]] of Object.entries(ranges)){
    const value=Number(settings[name]);
    if(!Number.isFinite(value)||value<min||value>max)throw new Error(`${name} must be between ${min} and ${max}`);
    result[name]=value;
  }
  return result;
}

export function loadSnapDefaults(storage){
  try{return validateSnapSettings({...DEFAULT_SNAP_SETTINGS,...JSON.parse(storage.getItem(key)||'{}')});}
  catch{return {...DEFAULT_SNAP_SETTINGS};}
}

export function saveSnapDefaults(storage,settings){
  const validated=validateSnapSettings(settings);
  storage.setItem(key,JSON.stringify(validated));
  return validated;
}
