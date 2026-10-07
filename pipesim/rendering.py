"""Headless CPU rendering, animation sampling, PNG and FFmpeg video output."""
from __future__ import annotations
import copy
import math
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
import numpy as np
import pybullet as pb
from PIL import Image, ImageColor, ImageDraw
from scipy.spatial.transform import Rotation
from .document import Assembly, DocumentError
from .physics import World, _matrix
from .geometry import assembly_bounds
from .math3d import transform, pose_of
from .human_assets import scene_humans
from .human_rendering import HumanSkin, SEGMENTS


def find_ffmpeg():
    """Find the encoder without relying on an interactive shell's startup file."""
    explicit=os.environ.get('PIPESIM_FFMPEG')
    if explicit:
        executable=shutil.which(explicit)
        if not executable: raise DocumentError('PIPESIM_FFMPEG does not point to an executable FFmpeg file')
        return executable
    executable=shutil.which('ffmpeg')
    if executable: return executable
    if os.name=='nt':
        # Git Bash commonly adds ~/bin in its login profile; desktop processes
        # do not inherit that shell-specific addition to PATH.
        candidate=Path.home()/'bin'/'ffmpeg.exe'
        if candidate.is_file(): return str(candidate)
    return None

class Renderer:
    def __init__(self,assembly,width=1280,height=800,eye=None,target=None,yaw=40,pitch=25,distance=None,background='#edf1f3',lighting='studio',orthographic=False):
        if not 32<=width<=8192 or not 32<=height<=8192: raise ValueError('Image dimensions must be between 32 and 8192 pixels')
        self.assembly=assembly; self.width=width; self.height=height; self.background=background
        self.lighting=lighting; self.orthographic=orthographic
        self.skins=[]
        for human in scene_humans(assembly):
            config=human['render_model']
            if config.get('enabled',True):
                try: self.skins.append(HumanSkin(assembly.base/config['file'],human))
                except (OSError, ValueError, KeyError, IndexError) as exc:
                    raise DocumentError(f"Cannot render imported human {human['id']}: {exc}") from exc
        self.texture_directory=tempfile.TemporaryDirectory(prefix='pipesim-human-render-')
        self.texture_paths={}
        for skin in self.skins:
            for primitive in skin.primitives:
                if primitive['image'] is not None:
                    path=Path(self.texture_directory.name)/f'{len(self.texture_paths)}.png'
                    primitive['image'].save(path); self.texture_paths[id(primitive)]=path
        self.separate=copy.copy(assembly); self.separate.joints=[]; self.separate.anchors=[]
        self.world=World(self.separate,static=True)
        self.rendered=False
        self.box=assembly_bounds(assembly)
        target=np.array(target if target is not None else (self.box[0]+self.box[1])/2,float)
        extent=max(float(max(self.box[1]-self.box[0])),200)
        distance=distance or extent*2.2
        if eye is None:
            az,el=np.deg2rad([yaw,pitch]); eye=target+distance*np.array([math.cos(el)*math.sin(az),-math.cos(el)*math.cos(az),math.sin(el)])
        self.view=pb.computeViewMatrix(np.array(eye)/1000,target/1000,[0,0,1])
        near=.01; far=max(100.,distance/1000*10)
        if orthographic:
            scale=distance/1000*.28
            self.projection=pb.computeProjectionMatrix(-scale*width/height,scale*width/height,-scale,scale,near,far)
            # Bullet's projection helper is perspective off-axis; replace it with a true orthographic matrix.
            right=scale*width/height
            matrix=np.array([[1/right,0,0,0],[0,1/scale,0,0],[0,0,-2/(far-near),-(far+near)/(far-near)],[0,0,0,1]])
            self.projection=matrix.T.flatten().tolist()
        else: self.projection=pb.computeProjectionMatrixFOV(40,width/height,near,far)
        self.initial={pid:p.matrix.copy() for pid,p in assembly.parts.items()}

    def project(self,position_mm):
        vp=np.array(self.projection).reshape(4,4).T@np.array(self.view).reshape(4,4).T
        clip=vp@np.r_[np.array(position_mm)/1000,1]
        clip/=clip[3]
        return [(clip[0]+1)*self.width/2,(1-clip[1])*self.height/2]

    def frame(self,poses=None,visible=None,highlight=None,arrow=None,time_s=0):
        # Bullet cannot deform/release ordinary visual meshes in place. Reclaim
        # its scene between skinned frames so video length cannot grow native
        # mesh memory without bound. Parsed rigs and textures remain cached.
        if self.skins and self.rendered:
            self.world.close(); self.world=World(self.separate,static=True)
        self.rendered=True
        visible=set(self.initial) if visible is None else set(visible)
        matrices={pid:transform(poses[pid]) if poses and pid in poses else initial.copy() for pid,initial in self.initial.items()}
        skinned={skin.human['id']+'/'+segment for skin in self.skins for segment in skin.mapping}
        # Hide all 19 bodies, including segments an imported skeleton omits.
        for skin in self.skins:
            skinned.update(pid for pid in self.initial if pid.startswith(skin.human['id']+'/'))
        for pid,(body,link,local) in self.world.part_map.items():
            matrix=matrices[pid].copy()
            if pid not in visible or pid in skinned: matrix[:3,3]+=1e8
            dyn=pb.getDynamicsInfo(body,-1,physicsClientId=self.world.client)
            center=matrix@_matrix(dyn[3],dyn[4])
            pb.resetBasePositionAndOrientation(body,center[:3,3]/1000,Rotation.from_matrix(center[:3,:3]).as_quat(),physicsClientId=self.world.client)
            color='#eea24c' if pid==highlight else self.assembly.parts[pid].definition.get('color','#8d9ba5')
            if highlight and pid!=highlight: color='#b4c0c7'
            rgb=np.array(ImageColor.getrgb(color))/255
            pb.changeVisualShape(body,-1,rgbaColor=[*rgb,1],physicsClientId=self.world.client)
        for skin in self.skins:
            skin.update(matrices,time_s)
            visible_segments={s for s in SEGMENTS if skin.human['id']+'/'+s in visible}
            if not visible_segments: continue
            for primitive in skin.primitives:
                vertices=skin.vertices(primitive)/1000
                indices=primitive['indices'].reshape(-1,3) if len(visible_segments)==len(SEGMENTS) else skin.visible_triangles(primitive,visible_segments)
                if not len(indices): continue
                # Smooth normals are regenerated from posed triangles; UV seams
                # remain split because original vertex indexing is retained.
                faces=vertices[indices]; normals=np.zeros_like(vertices)
                face_normals=np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0])
                for corner in range(3): np.add.at(normals,indices[:,corner],face_normals)
                normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-30)
                options={'uvs':primitive['uv'].tolist()} if primitive['uv'] is not None else {}
                visual=pb.createVisualShape(pb.GEOM_MESH,vertices=vertices.tolist(),indices=indices.ravel().tolist(),normals=normals.tolist(),rgbaColor=primitive['color'],physicsClientId=self.world.client,**options)
                body=pb.createMultiBody(baseMass=0,baseVisualShapeIndex=visual,physicsClientId=self.world.client)
                if id(primitive) in self.texture_paths:
                    texture=pb.loadTexture(str(self.texture_paths[id(primitive)]),physicsClientId=self.world.client)
                    pb.changeVisualShape(body,-1,textureUniqueId=texture,physicsClientId=self.world.client)
        modes={'studio':(.5,.75,.2,1),'flat':(1,0,0,0),'technical':(.7,.5,0,0)}
        ambient,diffuse,specular,shadow=modes[self.lighting]
        image=pb.getCameraImage(self.width,self.height,self.view,self.projection,renderer=pb.ER_TINY_RENDERER,lightDirection=[-2,-4,6],lightColor=[1,1,1],shadow=shadow,lightAmbientCoeff=ambient,lightDiffuseCoeff=diffuse,lightSpecularCoeff=specular,physicsClientId=self.world.client)
        rgba=np.asarray(image[2],dtype=np.uint8).reshape(self.height,self.width,4).copy()
        mask=np.asarray(image[4]).reshape(self.height,self.width)
        if self.background=='transparent':
            rgba[(mask<0)|(mask==self.world.ground),3]=0
        else:
            bg=ImageColor.getrgb(self.background)
            rgba[mask<0,:3]=bg
            rgba[:,:,3]=255
        result=Image.fromarray(rgba)
        if arrow and highlight:
            p=self.initial[highlight][:3,3]; direction=np.array(arrow,float)
            start=self.project(p-direction*180); end=self.project(p-direction*40)
            draw=ImageDraw.Draw(result)
            draw.line([tuple(start),tuple(end)],fill='#df7928',width=5)
            v=np.array(end)-start
            if np.linalg.norm(v)>1:
                v=v/np.linalg.norm(v); n=np.array([-v[1],v[0]])
                draw.polygon([tuple(end),tuple(np.array(end)-v*18+n*8),tuple(np.array(end)-v*18-n*8)],fill='#df7928')
        return result

    def close(self):
        self.world.close()
        self.texture_directory.cleanup()
    def __enter__(self): return self
    def __exit__(self,*args): self.close()

def render_image(assembly,path,**options):
    assembly.require_finished('rendering')
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    with Renderer(assembly,**options) as renderer:
        pic=renderer.frame()
        if path.suffix.lower() in ('.jpg','.jpeg'): pic=pic.convert('RGB')
        pic.save(path)
    return str(path)

def animation_frames(assembly,duration=None,fps=None,joint=None):
    animation=assembly.doc.get('animation',{})
    duration=duration or animation.get('duration_s',4); fps=fps or animation.get('fps',30)
    tracks=copy.deepcopy(animation.get('tracks',[]))
    if joint:
        j=next((j for j in assembly.joints if j['id']==joint),None)
        if j is None: raise DocumentError(f'Unknown joint {joint}')
        for coordinate,limits in j.get('limits',{}).items():
            if coordinate=='rotation_deg': continue
            tracks.append({'joint':joint,'coordinate':coordinate,'keyframes':[{'time_s':0,'value':limits[0]},{'time_s':duration/2,'value':limits[1]},{'time_s':duration,'value':limits[0]}]})
    if not tracks: raise DocumentError('No animation tracks; supply --joint with finite limits or record a simulation')
    frames=[]
    for t in np.linspace(0,duration,round(duration*fps)+1):
        doc=copy.deepcopy(assembly.doc); coordinates=doc.setdefault('state',{}).setdefault('joints',{})
        for track in tracks:
            keys=track['keyframes']; times=[k['time_s'] for k in keys]
            if times!=sorted(times) or len(set(times))!=len(times): raise DocumentError('Animation keyframe times must be strictly increasing')
            coordinates.setdefault(track['joint'],{})[track['coordinate']]=float(np.interp(t,times,[k['value'] for k in keys]))
        posed=Assembly.from_doc(doc,assembly.base)
        frames.append({'time_s':float(t),'parts':{pid:pose_of(p.matrix) for pid,p in posed.parts.items()},'joints':coordinates})
    return {'frames':frames,'fps':fps,'duration_s':duration,'method':'Authored joint motion; no physical feasibility is implied'}

def render_video(assembly,path,recording=None,**options):
    assembly.require_finished('rendering video')
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    recording=recording or animation_frames(assembly)
    frames=recording['frames']; fps=recording.get('fps',30)
    if not frames: raise ValueError('Recording has no frames')
    if not math.isfinite(fps) or fps<=0: raise ValueError('Recording frame rate must be positive and finite')
    ffmpeg=None
    if path.suffix.lower() not in ('','.gif','.png'):
        ffmpeg=find_ffmpeg()
        if not ffmpeg: raise DocumentError('FFmpeg is not visible to this server. Add it to the server PATH or set PIPESIM_FFMPEG to its full executable path, then restart the server. GIF export does not require it.')
    with Renderer(assembly,**options) as renderer:
        if path.suffix.lower()=='.gif':
            images=[renderer.frame(f['parts'],time_s=f.get('time_s',i/fps)).convert('RGB').quantize(colors=128) for i,f in enumerate(frames)]
            images[0].save(path,save_all=True,append_images=images[1:],duration=1000/fps,loop=0)
        elif path.suffix.lower()=='.png' or not path.suffix:
            directory=path if not path.suffix else path.parent/path.stem
            directory.mkdir(parents=True,exist_ok=True)
            for i,frame in enumerate(frames): renderer.frame(frame['parts'],time_s=frame.get('time_s',i/fps)).save(directory/f'frame-{i:05}.png')
        else:
            if renderer.width%2 or renderer.height%2: raise ValueError('Video width and height must be even for yuv420p')
            command=[ffmpeg,'-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24','-video_size',f'{renderer.width}x{renderer.height}','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p']
            if path.suffix.lower()=='.mp4': command+=['-movflags','+faststart']
            command.append(str(path))
            with subprocess.Popen(command,stdin=subprocess.PIPE,stderr=subprocess.PIPE) as process:
                try:
                    for i,frame in enumerate(frames): process.stdin.write(renderer.frame(frame['parts'],time_s=frame.get('time_s',i/fps)).convert('RGB').tobytes())
                    process.stdin.close(); process.stdin=None
                    _,error=process.communicate(timeout=60)
                except Exception:
                    process.kill(); process.wait(); raise
                if process.returncode: raise RuntimeError('FFmpeg failed: '+error.decode(errors='replace'))
    return str(path)
