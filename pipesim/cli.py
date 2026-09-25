"""The entire pipeline is callable from a terminal without a GUI or display server."""
from __future__ import annotations
import argparse
import copy
import json
import sys
from pathlib import Path
from datetime import datetime,timezone
from .document import Assembly, Library, DocumentError, read, write, plain

def parser():
    p=argparse.ArgumentParser(prog='pipesim',description='Create, inspect, simulate and build modular 3D pipe structures.')
    p.add_argument('--version',action='version',version='PipeSim 0.1.0')
    sub=p.add_subparsers(dest='command',required=True)
    serve=sub.add_parser('editor',help='Open the local cross-platform CAD editor')
    serve.add_argument('design',nargs='?',default='examples/workbench.pipe.yaml'); serve.add_argument('--port',type=int,default=8765); serve.add_argument('--root',default='.'); serve.add_argument('--open',action='store_true')
    lib=sub.add_parser('library',help='List or export the bundled catalogue')
    lib.add_argument('--output','-o'); lib.add_argument('--query',default='')
    for name,help in [('validate','Check a design and optionally its assembly order'),('plan','Find a stable build sequence'),('analyse','Solve a 3D frame finite-element load case'),('simulate','Run articulated rigid-body physics'),('stress','Apply increasing multiaxial loads'),('bom','Generate the bill of materials and cutting plan'),('build','Export illustrated, printable assembly instructions'),('render','Render a PNG or JPEG offscreen'),('animate','Render a motion range or recorded simulation'),('mesh','Export assembly geometry'),('fit','Run human reach and seat-fit design tests'),('expand','Expand reusable objects into editable parts'),('snapshot','Turn a recorded frame into an editable design')]:
        command=sub.add_parser(name,help=help); command.add_argument('design'); command.add_argument('--output','-o')
        if name in ('validate','plan','analyse','simulate','stress','fit'): command.add_argument('--record',action='store_true',help='Store the result in the input design, with its input fingerprint')
        if name=='validate': command.add_argument('--build',action='store_true'); command.add_argument('--no-collisions',action='store_true')
        if name=='plan': command.add_argument('--search-limit',type=int,default=3000)
        if name=='analyse': command.add_argument('--no-self-weight',action='store_true')
        if name=='simulate':
            command.add_argument('--duration',type=float,default=3); command.add_argument('--fps',type=float,default=30); command.add_argument('--dt',type=float,default=1/240)
            command.add_argument('--chain-links-per-body',type=int,default=1,help='Opt in to rigid groups of up to N chain links (default: 1, full flexibility)')
            command.add_argument('--quiet',action='store_true',help='Hide simulation progress')
        if name=='stress':
            command.add_argument('--part',required=True); command.add_argument('--max-force',type=float,default=10000); command.add_argument('--directions',type=int,default=24); command.add_argument('--steps',type=int,default=12); command.add_argument('--seed',type=int,default=1)
            command.add_argument('--rigid',action='store_true',help='Also test overturning, sliding and explicit joint break thresholds'); command.add_argument('--dwell',type=float,default=.5)
        if name=='bom': command.add_argument('--kerf',type=float); command.add_argument('--stock',type=float,nargs='+'); command.add_argument('--trim',type=float)
        if name=='build': command.add_argument('--engineering',action='store_true'); command.add_argument('--no-images',action='store_true')
        if name in ('render','animate'):
            command.add_argument('--width',type=int,default=1280); command.add_argument('--height',type=int,default=800)
            command.add_argument('--eye',type=float,nargs=3,metavar=('X','Y','Z')); command.add_argument('--target',type=float,nargs=3,metavar=('X','Y','Z'))
            command.add_argument('--yaw',type=float,default=40); command.add_argument('--pitch',type=float,default=25); command.add_argument('--distance',type=float)
            command.add_argument('--background',default='#edf1f3'); command.add_argument('--lighting',choices=['studio','flat','technical'],default='studio'); command.add_argument('--orthographic',action='store_true')
        if name=='animate': command.add_argument('--simulation',help='Recording JSON, or use the simulation stored in the design'); command.add_argument('--joint'); command.add_argument('--fps',type=float); command.add_argument('--duration',type=float); command.add_argument('--valid-only',action='store_true',help='Render the longest contiguous interval of valid sampled authored poses')
        if name=='fit': command.add_argument('--human'); command.add_argument('--target',nargs=3,type=float); command.add_argument('--hand',choices=['left','right'],default='right'); command.add_argument('--seat')
        if name=='snapshot': command.add_argument('--simulation'); command.add_argument('--frame',type=int,default=-1)
    human=sub.add_parser('human',help='Generate an editable 19-segment humanoid design')
    human.add_argument('--height',type=float,default=1750); human.add_argument('--mass',type=float,default=75); human.add_argument('--pose',choices=['standing','seated','crouching','pull-up'],default='standing'); human.add_argument('--output','-o',required=True)
    human.add_argument('--hold-joints',nargs='+',help='Anatomical joint names or groups: arms, torso, upper_body, legs')
    human.add_argument('--hold-pose',action='store_true',help='Hold all anatomical joints with bounded torque')
    human.add_argument('--strength-scale',type=float,default=1)
    human.add_argument('--grip-diameter',type=float,help='Curled-hand bore for a bar of this diameter in mm')
    human.add_argument('--joint-damping',type=float,default=.08,help='Passive viscous resistance per anatomical axis, N m s/rad')
    imp=sub.add_parser('import-mesh',help='Make a portable part library from an STL, OBJ or GLB mesh')
    imp.add_argument('mesh'); imp.add_argument('--id',required=True); imp.add_argument('--mass',type=float,required=True); imp.add_argument('--scale',type=float,default=1,help='Scale mesh coordinates into mm'); imp.add_argument('--output','-o',required=True)
    contacts=sub.add_parser('contact-loads',help='Create an editable FEA load case from simulation contacts')
    contacts.add_argument('design'); contacts.add_argument('--simulation',required=True); contacts.add_argument('--parts',nargs='+'); contacts.add_argument('--window',type=float,default=.25); contacts.add_argument('-o','--output',required=True)
    pack=sub.add_parser('bundle',help='Copy a portable design with its resolved catalogue and mesh assets')
    pack.add_argument('design'); pack.add_argument('-o','--output',required=True)
    motion=sub.add_parser('motion-check',help='Test authored motion samples for collisions, limits and engagement')
    motion.add_argument('design'); motion.add_argument('--joint'); motion.add_argument('--samples',type=int,default=31); motion.add_argument('--record',action='store_true'); motion.add_argument('-o','--output')
    return p

def main(argv=None):
    args=parser().parse_args(argv)
    try:
        if args.command=='editor':
            from .server import serve
            serve(Path(args.root).resolve(),args.design,args.port,args.open); return 0
        if args.command=='library':
            library=Library.load(); result={'format':'pipesim-library/1','name':'PipeSim catalogue','materials':library.materials,'parts':{k:v for k,v in library.parts.items() if args.query.lower() in (k+' '+v.get('name','')).lower()}}
        elif args.command=='human':
            from .human import humanoid
            data=humanoid(stature_mm=args.height,mass_kg=args.mass,pose=args.pose,hold_pose=args.hold_pose,hold_joints=args.hold_joints,strength_scale=args.strength_scale,grip_diameter_mm=args.grip_diameter,joint_damping_nms_rad=args.joint_damping)
            result={'format':'pipesim/1','units':'mm-kg-s-N-deg','name':'Articulated human','parts':data['parts'],'joints':data['joints'],'metadata':data['metadata']}
        elif args.command=='import-mesh':
            from .importing import import_mesh
            result=import_mesh(args.mesh,args.output,args.id,args.mass,args.scale)
            print(json.dumps(result,indent=2)); return 0
        else:
            assembly=Assembly.load(args.design)
            if args.command=='motion-check':
                from .motion import check_motion
                result=check_motion(assembly,args.joint,args.samples)
            elif args.command=='bundle':
                from .packaging import bundle
                print(bundle(assembly,args.output)); return 0
            elif args.command=='contact-loads':
                from .loadcases import contact_load_case
                result=contact_load_case(assembly,read(args.simulation),args.parts,args.window)
            elif args.command=='validate':
                from .validation import validate
                result=validate(assembly,not args.no_collisions,args.build)
            elif args.command=='plan':
                from .planning import plan_build
                result=plan_build(assembly,args.search_limit)
            elif args.command=='analyse':
                from .fea import analyse
                result=analyse(assembly,not args.no_self_weight)
            elif args.command=='stress':
                from .fea import destruction_test
                result=destruction_test(assembly,args.part,args.max_force,args.directions,args.steps,args.seed,args.dwell if args.rigid else 0)
            elif args.command=='simulate':
                from .simulation_jobs import simulate_in_worker
                from .simulation_control import console_progress
                # Keep stdout machine-readable when the recording is printed there.
                stream=sys.stdout if args.output else sys.stderr
                progress=None if args.quiet else lambda update: console_progress(update,stream)
                result=simulate_in_worker(assembly,duration=args.duration,fps=args.fps,dt=args.dt,
                    chain_links_per_body=args.chain_links_per_body,progress=progress)
            elif args.command=='bom':
                from .exporting import bom,cutting_plan
                result={'bom':bom(assembly),'cutting':cutting_plan(assembly,args.kerf,args.stock,args.trim)}
            elif args.command=='build':
                from .exporting import build_export
                result=build_export(assembly,args.output or 'output/build',args.engineering,not args.no_images)
                print(json.dumps(result,indent=2)); return 0
            elif args.command in ('render','animate'):
                from .rendering import render_image,render_video,animation_frames
                options={k:getattr(args,k) for k in ('width','height','eye','target','yaw','pitch','distance','background','lighting','orthographic')}
                if args.command=='render': result=render_image(assembly,args.output or 'output/render.png',**options)
                else:
                    recording=read(args.simulation) if args.simulation else assembly.doc.get('results',{}).get('simulate')
                    if args.joint or not recording: recording=animation_frames(assembly,args.duration,args.fps,args.joint)
                    if args.valid_only:
                        if 'engine' in recording: raise DocumentError('--valid-only is for authored motion; simulation contacts are physical events')
                        from .motion import check_motion,longest_valid_clip
                        report=check_motion(assembly,recording=recording)
                        recording=longest_valid_clip(recording,report)
                    result=render_video(assembly,args.output or 'output/motion.mp4',recording,**options)
                print(result); return 0
            elif args.command=='mesh':
                from .geometry import export_mesh
                result=export_mesh(assembly,args.output or 'output/assembly.glb'); print(result); return 0
            elif args.command=='fit':
                from .human import reach,seat_fit,run_fit_tests
                if args.target:
                    if not args.human: raise DocumentError('--target needs --human')
                    result=reach(assembly,args.human,args.target,args.hand)
                elif args.seat:
                    if not args.human: raise DocumentError('--seat needs --human')
                    result=seat_fit(assembly,args.human,args.seat)
                else: result=run_fit_tests(assembly)
            elif args.command=='expand':
                from .editing import expand_objects
                result=expand_objects(assembly)
            elif args.command=='snapshot':
                from .editing import snapshot_design
                recording=read(args.simulation) if args.simulation else assembly.doc.get('results',{}).get('simulate')
                if not recording: raise DocumentError('No simulation recording found')
                result=snapshot_design(assembly,recording['frames'][args.frame])
            if getattr(args,'record',False):
                doc=read(args.design); result['recorded_utc']=datetime.now(timezone.utc).isoformat()
                if args.command=='plan': doc['build_plan']=result
                else: doc.setdefault('results',{})[args.command]=result
                write(args.design,doc)
        if getattr(args,'output',None):
            if result.get('format')=='pipesim/1' and hasattr(args,'design'):
                from .editing import relocate_design
                result=relocate_design(result,assembly.base,Path(args.output).resolve().parent)
            write(args.output,result); print(str(args.output))
        else: print(json.dumps(plain(result),indent=2,allow_nan=False))
        if args.command=='validate' and not result['valid']: return 1
        if args.command=='motion-check' and not result['valid']: return 1
        if args.command=='plan' and result['status']!='buildable': return 1
        if args.command=='analyse' and result['status']!='solved': return 1
        if args.command=='fit' and result.get('passed') is False: return 1
        if args.command=='fit' and (result.get('reachable') is False or result.get('fits') is False): return 1
        return 0
    except KeyboardInterrupt:
        print('PipeSim: Simulation cancelled' if args.command=='simulate' else 'PipeSim: Cancelled',file=sys.stderr,flush=True)
        return 130
    except (DocumentError,ValueError,OSError,KeyError,IndexError,RuntimeError) as exc:
        print(f'PipeSim: {exc}',file=sys.stderr); return 2

if __name__=='__main__': raise SystemExit(main())
