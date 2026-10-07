// Flexible lines render whole segments. A keyboard request must cross a segment
// boundary even when the configured resize step is smaller than one pitch.
export function flexibleSegmentCount(requestedLengthMm,pitchMm){
  if(!Number.isFinite(requestedLengthMm)||requestedLengthMm<=0||
     !Number.isFinite(pitchMm)||pitchMm<=0)throw new RangeError('Choose positive finite flexible-line dimensions');
  return Math.max(1,Math.ceil(requestedLengthMm/pitchMm-1e-10));
}

export function nextFlexibleShortcutLength({requestedLengthMm,pitchMm,stepMm,direction,maxSegments=1000}){
  const count=flexibleSegmentCount(requestedLengthMm,pitchMm);
  if(!Number.isFinite(stepMm)||stepMm<=0)throw new RangeError('Choose a positive finite keyboard step');
  if(direction!==1&&direction!==-1)throw new RangeError('Choose a resize direction');
  if(!Number.isInteger(maxSegments)||maxSegments<1)throw new RangeError('Choose a positive segment limit');
  if(count>maxSegments)throw new RangeError('The flexible line exceeds its segment limit');
  if(direction<0&&count===1||direction>0&&count===maxSegments)return null;

  const increment=Math.max(stepMm,pitchMm);
  const limit=maxSegments*pitchMm;
  let requested=direction>0
    ?Math.min(limit,requestedLengthMm+increment)
    :Math.max(pitchMm,requestedLengthMm-increment);
  // Match chain.dimensions' ceil(... - 1e-10) at floating point boundaries.
  if(direction>0&&flexibleSegmentCount(requested,pitchMm)<=count)
    requested=Math.min(limit,(count+1)*pitchMm);
  if(direction<0&&flexibleSegmentCount(requested,pitchMm)>=count)
    requested=Math.max(pitchMm,(count-1)*pitchMm);
  return requested;
}
