from pipesim.document import Assembly
from pipesim.fea import analyse
from pipesim.validation import validate
from pipesim.physics import simulate
from pipesim.human import humanoid

for name in ('cantilever','workbench','sliding-collar','gt2-stage','human','seated-human'):
    a=Assembly.load(f'examples/{name}.pipe.yaml')
    print(name,len(a.parts),len(a.joints),'groups',len(a.rigid_groups()),flush=True)
    v=validate(a,collisions=False)
    print('validation',v['valid'],[(i['code'],i['parts']) for i in v['issues']],flush=True)
    if name in ('cantilever','workbench'):
        result=analyse(a)
        print('FEA',result['status'],[(r['part'],r['max_displacement_mm']) for r in result.get('members',[])],flush=True)
    else:
        result=simulate(a,duration=.1,fps=10)
        print('physics',len(result['frames']),result['final_max_speed_m_s'],flush=True)
