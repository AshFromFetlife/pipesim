import test from 'node:test';
import assert from 'node:assert/strict';
import {randomBytes} from 'node:crypto';
import {flexibleSegmentCount,nextFlexibleShortcutLength} from '../pipesim/web/resize-shortcuts.js';

function next(length,pitch,step,direction,maxSegments){
  return nextFlexibleShortcutLength({requestedLengthMm:length,pitchMm:pitch,stepMm:step,direction,maxSegments});
}

test('segment count matches the flexible-line generator at exact and near boundaries',()=>{
  assert.equal(flexibleSegmentCount(0.001,25),1);
  assert.equal(flexibleSegmentCount(25,25),1);
  assert.equal(flexibleSegmentCount(25+1e-9,25),1);
  assert.equal(flexibleSegmentCount(25+1e-7,25),2);
  assert.equal(flexibleSegmentCount(50,25),2);
  assert.equal(flexibleSegmentCount(50.8,25.4),2);
  assert.equal(flexibleSegmentCount(50.8000001,25.4),3);
});

test('a shortcut step smaller than pitch changes one visible segment each time',()=>{
  for(const pitch of [20,25,80]){
    let length=pitch*3;
    for(let count=4;count<=12;count++){
      length=next(length,pitch,10,1);
      assert.equal(flexibleSegmentCount(length,pitch),count,`growing pitch ${pitch}`);
    }
    for(let count=11;count>=1;count--){
      length=next(length,pitch,10,-1);
      assert.equal(flexibleSegmentCount(length,pitch),count,`shrinking pitch ${pitch}`);
    }
    assert.equal(next(length,pitch,10,-1),null);
  }
});

test('larger configured steps are respected and clipped at limits',()=>{
  assert.equal(next(120,20,60,1),180);
  assert.equal(next(120,20,60,-1),60);
  assert.equal(next(35,25,100,-1),25);
  assert.equal(next(75,25,10,1,3),null);
  assert.equal(next(50,25,100,1,3),75);
  assert.equal(next(50,25,100,-1,3),25);
});

test('floating boundaries always move the visible count in the requested direction',()=>{
  for(const pitch of [0.1,20,25.4,80]){
    for(const count of [2,3,19,1000]){
      for(const offset of [-1e-9,0,1e-9,pitch*0.5]){
        const length=(count-1)*pitch+offset;
        const before=flexibleSegmentCount(length,pitch);
        const shrink=next(length,pitch,Math.min(10,pitch/4),-1);
        if(before===1)assert.equal(shrink,null);
        else assert.ok(flexibleSegmentCount(shrink,pitch)<before,`shrink ${JSON.stringify({pitch,count,offset,length,shrink})}`);
        const grow=next(length,pitch,Math.min(10,pitch/4),1);
        if(before===1000)assert.equal(grow,null);
        else assert.ok(flexibleSegmentCount(grow,pitch)>before,`grow ${JSON.stringify({pitch,count,offset,length,grow})}`);
      }
    }
  }
});

test('generated pitches, lengths, and configured steps always change the rendered count',()=>{
  const seed=process.env.PIPESIM_RESIZE_SHORTCUT_SEED===undefined
    ?randomBytes(4).readUInt32LE(0)
    :Number(process.env.PIPESIM_RESIZE_SHORTCUT_SEED)>>>0;
  let state=seed;
  const random=()=>((state=(Math.imul(state,1664525)+1013904223)>>>0)/2**32);
  for(let caseIndex=0;caseIndex<10000;caseIndex++){
    const pitch=0.01+random()*1000;
    const step=0.001+random()*2000;
    const length=pitch*(0.0001+random()*999.999);
    const before=flexibleSegmentCount(length,pitch);
    for(const direction of [-1,1]){
      const result=next(length,pitch,step,direction);
      if(result===null){
        assert.equal(before,direction<0?1:1000,`seed ${seed}, case ${caseIndex}`);
      }else{
        const after=flexibleSegmentCount(result,pitch);
        assert.ok(direction*(after-before)>0,
          `seed ${seed}, case ${caseIndex}: ${JSON.stringify({pitch,step,length,result,before,after,direction})}`);
      }
    }
  }
});

test('invalid shortcut inputs fail explicitly',()=>{
  assert.throws(()=>flexibleSegmentCount(0,25),RangeError);
  assert.throws(()=>flexibleSegmentCount(25,0),RangeError);
  assert.throws(()=>next(25,25,0,1),RangeError);
  assert.throws(()=>next(25,25,10,0),RangeError);
  assert.throws(()=>next(25,25,10,1,0),RangeError);
  assert.throws(()=>next(100,25,10,1,2),RangeError);
});
