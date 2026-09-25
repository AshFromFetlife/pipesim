"""Sample authored motion for joint engagement, limit and collision feasibility."""
import copy
from .document import Assembly,DocumentError
from .math3d import transform
from .rendering import animation_frames
from .validation import validate

def check_motion(assembly,joint=None,samples=31,recording=None):
    if not 2<=samples<=1001: raise DocumentError('Motion sampling requires 2–1001 poses')
    duration=assembly.doc.get('animation',{}).get('duration_s',4)
    recording=recording or animation_frames(assembly,duration,(samples-1)/duration,joint)
    checks=[]; intervals=[]; start=None
    for i,frame in enumerate(recording['frames']):
        posed=copy.deepcopy(assembly)
        for pid,pose in frame['parts'].items(): posed.parts[pid].matrix=transform(pose)
        posed.doc['state']={'joints':frame['joints']}
        report=validate(posed)
        faults=[issue for issue in report['issues'] if issue['severity']=='error' or issue['code']=='INTERSECTION']
        checks.append({'time_s':frame['time_s'],'valid':not faults,'issues':faults,'joints':frame['joints']})
        if not faults and start is None: start=i
        if start is not None and (faults or i==len(recording['frames'])-1):
            end=i-1 if faults else i
            intervals.append({'first_frame':start,'last_frame':end,'from_s':recording['frames'][start]['time_s'],'to_s':recording['frames'][end]['time_s']}); start=None
    return {'valid':all(c['valid'] for c in checks),'input_sha256':assembly.input_hash,'samples':checks,'valid_intervals':intervals,'limitations':['Pose sampling is not continuous collision detection or a dynamics proof; narrow invalid intervals between samples can be missed','Uses supplied joint engagement and collision geometry, including documented approximations']}

def longest_valid_clip(recording,report):
    if not report['valid_intervals']: raise DocumentError('No valid sampled poses in this motion')
    interval=max(report['valid_intervals'],key=lambda r:r['last_frame']-r['first_frame'])
    result=copy.deepcopy(recording); result['frames']=result['frames'][interval['first_frame']:interval['last_frame']+1]
    if len(result['frames'])<2: raise DocumentError('A single valid pose is insufficient for a motion clip')
    origin=result['frames'][0]['time_s']
    for frame in result['frames']: frame['time_s']-=origin
    result['duration_s']=result['frames'][-1]['time_s']; result['checked_original_interval']=interval
    return result
