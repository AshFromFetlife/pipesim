import {execFileSync} from 'node:child_process';

let cached;
/** Real authored poses and physical recording, shared by CPU and render checks. */
export function humanPhysicsFixture(){
  if(cached)return cached;
  const script=`import json
from pathlib import Path
from pipesim.human import humanoid
from pipesim.document import Assembly
from pipesim.physics import simulate
poses={pose:humanoid(pose=pose) for pose in ['standing','arms-spread','hands-up','seated','crouching','supine']}
doc={'format':'pipesim/1','units':'mm-kg-s-N-deg','name':'Render recording regression','parts':[],'joints':[],
     'objects':[{'id':'person','template':'human','parameters':{'pose':'arms-forward','posture_control':'fidget','movement_seed':9}}],
     'anchors':[{'part':'person/pelvis'}],'environment':{'ground':False}}
recording=simulate(Assembly.from_doc(doc,Path.cwd()),duration=.5,fps=10)
for index in (0,2,5):
    frame=recording['frames'][index]
    poses[f"simulation-fidget-{frame['time_s']:.2f}s"]={'time_s':frame['time_s'],'simulation':True,
        'parts':[{**part,'pose':frame['parts']['person/'+part['id']]} for part in poses['standing']['parts']]}
print(json.dumps(poses))`;
  const output=execFileSync(process.env.PYTHON||'python',['-c',script],{encoding:'utf8',windowsHide:true,stdio:['ignore','pipe','pipe']});
  // Bullet may emit native startup diagnostics before the JSON line.
  cached=JSON.parse(output.slice(output.indexOf('{')));return cached;
}
