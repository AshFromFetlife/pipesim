"""Measure real local-HTTP drag latency after opening a scene.

Includes first frame, changing targets with previous-frame seeds, and release.
--compare also measures the former uncached, full-document preview pipeline.
No design files are modified; the server uses an ephemeral port.
"""
from pathlib import Path
import argparse
import copy
import json
import statistics
import sys
import threading
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from pipesim.document import read
from pipesim.server import EditorServer


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames',type=int,default=8)
    parser.add_argument('--compare',action='store_true')
    parser.add_argument('--output',type=Path,default=ROOT/'output/preview-benchmark.json')
    args=parser.parse_args()
    if args.frames<2: parser.error('--frames must be at least 2')
    server=EditorServer(ROOT,'examples/human-pull-up.pipe.yaml',0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    def post(route,doc,extra=None):
        data={'document':doc,'path':'examples/preview-benchmark.pipe.yaml',**(extra or {})}
        request=urllib.request.Request(f'http://127.0.0.1:{server.server_port}/api/'+route,
            data=json.dumps(data).encode(),headers={'Content-Type':'application/json','X-PipeSim-Token':server.token})
        start=time.perf_counter()
        with urllib.request.urlopen(request,timeout=120) as response: raw=response.read()
        return json.loads(raw),round((time.perf_counter()-start)*1000,3),len(raw)
    cases=[('human-pull-up','person/left_shin',1),('hinged-triangle','upper-back',0),('sliding-collar','slider',2),('chain','chain/link-200',0)]
    report={'frames':args.frames,'timing':'Milliseconds, local HTTP including serialization; scene opened before first drag. Targets change each frame.','cases':{}}
    try:
        for name,selected,axis in cases:
            doc=read(ROOT/'examples'/f'{name}.pipe.yaml')
            if name=='chain':
                doc['objects'][0]['layout_mode']='posable';doc['objects'][0]['parameters']['length_mm']=4000
            scene,_,_=post('resolve',doc)
            original=next(p['pose'] for p in scene['parts'] if p['id']==selected)
            seed=None;rows=[];targets=[]
            for frame in range(args.frames):
                target=copy.deepcopy(original);target['position_mm'][axis]+=30+frame*2
                if name=='chain': target['position_mm'][2]+=30+frame*2
                targets.append(target)
                payload={'selected':selected,'target':target,'mode':'translate','seed':seed}
                result,elapsed,size=post('move',doc,{**payload,'preview':True,'pose_only':True})
                rows.append({'ms':elapsed,'bytes':size});seed=result['seed']
            _,release_ms,_=post('move',doc,{**payload,'seed':seed,'preview_id':result['preview_id']})
            stats={'first_frame_ms':rows[0]['ms'],'following_median_ms':statistics.median(r['ms'] for r in rows[1:]),
                   'release_ms':release_ms,'frames':rows}
            if args.compare:
                # Equivalent to the old /api/move?preview path: load and resolve
                # again, solve, expand/rebase/regroup, serialize the document.
                from pipesim.document import Assembly
                from pipesim.posing import transform_part
                old=[];seed=None
                for target in targets[:3]:
                    start=time.perf_counter();assembly=Assembly.from_doc(doc,ROOT/'examples')
                    result=transform_part(assembly,selected,target,seed=seed);encoded=json.dumps(result);seed=result['seed']
                    old.append({'ms':round((time.perf_counter()-start)*1000,3),'bytes':len(encoded)})
                stats['uncached_full_document_frames']=old
            report['cases'][name]=stats
            print(f'{name}: first {rows[0]["ms"]:.1f} ms; following {stats["following_median_ms"]:.1f} ms; release {release_ms:.1f} ms',flush=True)
    finally:
        server.shutdown();server.server_close();thread.join()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')


if __name__=='__main__': main()
