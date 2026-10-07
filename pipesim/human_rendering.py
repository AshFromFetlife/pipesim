"""CPU glTF skinning for headless images and recorded animation exports.

Retargeting follows web/human-model.js. Cross-renderer regressions compare bone
matrices and weighted vertices, including bind-pose calibration and extra bones.
All lengths in the posed scene are millimetres; source assets retain their units.
"""
from __future__ import annotations

import base64
import io
import math
import re
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
from .document import DocumentError
from .human_assets import _read_container, _embed, _metadata, _auto_bone_map, VRM_SEGMENTS
from .math3d import transform

SEGMENTS = ['pelvis', 'lumbar', 'thorax', 'neck', 'head'] + [
    side + '_' + part for side in ('left', 'right') for part in
    ('clavicle', 'upper_arm', 'forearm', 'hand', 'thigh', 'shin', 'foot')]
NEXT = dict(zip(SEGMENTS[:4], SEGMENTS[1:5]))
ALIASES = dict(pelvis=['hips', 'hip', 'pelvis', 'roothips'],
               lumbar=['spine', 'spine01', 'spine1', 'abdomen', 'lowerback'],
               thorax=['chest', 'upperchest', 'spine2', 'spine02', 'spine3', 'spine03', 'upperback', 'chestupper'],
               neck=['neck', 'neck1', 'neck01'], head=['head', 'head1', 'head01'])
for side in ('left', 'right'):
    chain = [side + '_' + p for p in ('clavicle', 'upper_arm', 'forearm', 'hand')]
    legs = [side + '_' + p for p in ('thigh', 'shin', 'foot')]
    NEXT.update(zip(chain[:-1], chain[1:])); NEXT.update(zip(legs[:-1], legs[1:]))
    for part, names in dict(clavicle=['shoulder', 'clavicle', 'collar'], upper_arm=['upperarm', 'arm'],
                            forearm=['lowerarm', 'forearm'], hand=['hand', 'wrist'], thigh=['upperleg', 'upleg', 'thigh'],
                            shin=['lowerleg', 'leg', 'calf', 'shin'], foot=['foot', 'ankle']).items():
        ALIASES[side + '_' + part] = [v for n in names for v in (side+n, n+side, side[0]+n, n+side[0])]
for segment in SEGMENTS: ALIASES[segment].append(segment.replace('_', ''))
REQUIRED = ['pelvis', 'head'] + [s+'_'+p for s in ('left', 'right') for p in ('upper_arm', 'forearm', 'hand', 'thigh', 'shin', 'foot')]


def canonical(name):
    name = re.sub(r'^.*[|:]', '', name.lower())
    name = re.sub(r'^(mixamorig|bip0?1|def|org|j_bip_c|j_bip_l|j_bip_r)[_. -]*', '', name)
    return re.sub('[^a-z0-9]', '', name)


def unit(v):
    return v / max(np.linalg.norm(v), 1e-30)


def rotation(matrix):
    return Rotation.from_matrix(matrix[:3, :3] / np.linalg.norm(matrix[:3, :3], axis=0))


def compose(position, quaternion, scale):
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion.as_matrix() * scale
    matrix[:3, 3] = position
    return matrix


def between(a, b):
    a, b = unit(a), unit(b)
    w = 1 + a @ b
    if w < 1e-8:
        axis = np.array([-a[1], a[0], 0]) if abs(a[0]) > abs(a[2]) else np.array([0, -a[2], a[1]])
        return Rotation.from_rotvec(unit(axis) * math.pi)
    return Rotation.from_quat(np.r_[np.cross(a, b), w])


class HumanSkin:
    def __init__(self, path, human):
        path = Path(path)
        self.human, self.config = human, human['render_model']
        self.document, binary = _read_container(path.read_bytes(), path.suffix.lower())
        self.buffers = _embed(self.document, binary, {}, path.name)
        _metadata(self.document, self.buffers)
        self.nodes = self.document['nodes']
        self.parents = {child: i for i, n in enumerate(self.nodes) for child in n.get('children', [])}
        scenes=self.document.get('scenes',[])
        roots=scenes[self.document.get('scene',0)].get('nodes',[]) if scenes else [i for i in range(len(self.nodes)) if i not in self.parents]
        self.active=set(); pending=list(roots)
        while pending:
            i=pending.pop(); self.active.add(i); pending.extend(self.nodes[i].get('children',[]))
        def depth(i):
            value = 0
            while i in self.parents: i = self.parents[i]; value += 1
            return value
        self.order = sorted(range(len(self.nodes)), key=depth)
        active_skins={node['skin'] for i,node in enumerate(self.nodes) if i in self.active and 'skin' in node}
        self.bones = {i for s in active_skins for i in self.document['skins'][s]['joints']} & self.active
        self.local = []
        for node in self.nodes:
            self.local.append(np.array(node['matrix']).reshape(4, 4).T if 'matrix' in node else
                              compose(node.get('translation', [0, 0, 0]), Rotation.from_quat(node.get('rotation', [0, 0, 0, 1])), node.get('scale', [1, 1, 1])))
        self.root = np.eye(4)
        self.world = np.empty((len(self.nodes), 4, 4))
        self.refresh()
        self.inverse_binds = []
        # Skeleton.pose(), as in THREE: restore bone worlds from inverse binds,
        # then recover their local transforms, before fitting the ragdoll.
        for skin_index,skin in enumerate(self.document['skins']):
            inverses = self.accessor(skin['inverseBindMatrices']).reshape(-1, 4, 4).transpose(0, 2, 1) if 'inverseBindMatrices' in skin else np.tile(np.eye(4), (len(skin['joints']), 1, 1))
            self.inverse_binds.append(inverses)
            if skin_index in active_skins:
                for i, inverse in zip(skin['joints'], inverses): self.world[i] = np.linalg.inv(inverse)
        for i in self.bones:
            parent = self.parents.get(i)
            self.local[i] = np.linalg.inv(self.world[parent]) @ self.world[i] if parent in self.bones else self.world[i].copy()
        self.refresh()
        self.mapping = self.map_bones()
        rests = {p['id']: transform(p['pose']) for p in human['rest_parts']}
        anchors = {}
        for segment, rest in rests.items():
            joint = next((j for j in human['joints'] if j['b']['part'] == segment), None)
            anchors[segment] = (rest @ np.r_[joint['b']['frame']['position_mm'], 1])[:3] if joint else rest[:3, 3].copy()
        hips = self.world[self.mapping['pelvis'], :3, 3].copy()
        up = unit(self.world[self.mapping['head'], :3, 3] - hips)
        right = self.world[self.mapping['right_thigh'], :3, 3] - self.world[self.mapping['left_thigh'], :3, 3]
        right = unit(right - up * (right @ up))
        if min(np.linalg.norm(up), np.linalg.norm(right)) < .9: raise DocumentError('Human model: bind pose has indistinct hips, head or thighs')
        alignment = Rotation.from_matrix(np.column_stack([right, np.cross(up, right), up])).inv()
        scale = np.linalg.norm(anchors['head']-anchors['pelvis']) / np.linalg.norm(self.world[self.mapping['head'], :3, 3]-hips)
        self.root = compose(anchors['pelvis'] - alignment.apply(hips)*scale, alignment, [scale]*3)
        self.refresh()
        self.bindings = []
        for segment, i in self.mapping.items():
            rest, anchor, position = rests[segment], anchors[segment], self.world[i, :3, 3]
            q = rotation(self.world[i]); following = NEXT.get(segment)
            while following and following not in self.mapping: following = NEXT.get(following)
            children = [c for c in self.nodes[i].get('children', []) if c in self.bones]
            if following:
                source = self.world[self.mapping[following], :3, 3] - position
                target = anchors[following] - anchor
            elif segment.endswith('_hand'):
                finger = next((c for c in children if 'middle' in self.nodes[c].get('name', '').lower()),
                              next((c for c in children if 'index' in self.nodes[c].get('name', '').lower()), children[0] if children else None))
                source = self.world[finger, :3, 3]-position if finger is not None else position-self.world[self.mapping[segment.replace('_hand', '_forearm')], :3, 3]
                target = rest[:3, 2]
            elif segment.endswith('_foot'):
                toe = next((c for c in children if 'toe' in self.nodes[c].get('name', '').lower()), children[0] if children else None)
                source = self.world[toe, :3, 3]-position if toe is not None else np.array([0., 1, 0])
                source[2] = 0; target = rest[:3, 1]
            else: source = np.array([0., 0, 1]); target = rest[:3, 2]
            if np.linalg.norm(source) > 1e-4 and np.linalg.norm(target) > 1e-4: q = between(source, target) * q
            self.bindings.append((segment, i, (np.linalg.inv(rest) @ np.r_[anchor, 1])[:3], rotation(rest).inv() * q))
        self.bindings.sort(key=lambda b: depth(b[1]))
        ancestors = set()
        for i in self.mapping.values():
            while i in self.parents: i = self.parents[i]; ancestors.add(i)
        self.extras = []
        for i in self.order:
            if i not in self.bones or i in self.mapping.values(): continue
            settings = self.config.get('extra_bones', {}).get(str(i), self.config.get('extra_bones', {}).get(self.nodes[i].get('name'), {}))
            mode = 'fixed_to_parent' if i in ancestors else settings.get('mode', self.config.get('default_extra_mode', 'fixed_to_parent'))
            child = next((c for c in self.nodes[i].get('children', []) if c in self.bones), None)
            self.extras.append(dict(i=i, settings=settings, mode=mode, bind=rotation(self.local[i]),
                                    tail=self.local[child][:3, 3].copy() if child is not None else np.array([0, 80/scale, 0]),
                                    angle=np.zeros(3), velocity=np.zeros(3), previous=None, previous_velocity=np.zeros(3)))
        self.primitives = self.load_primitives()
        self.last_time = None

    def accessor(self, index):
        a = self.document['accessors'][index]
        dtype = np.dtype({5120:'i1', 5121:'u1', 5122:'<i2', 5123:'<u2', 5125:'<u4', 5126:'<f4'}[a['componentType']])
        size = {'SCALAR':1, 'VEC2':2, 'VEC3':3, 'VEC4':4, 'MAT4':16}[a['type']]
        def read(view_id, offset, count, width, dt):
            view = self.document['bufferViews'][view_id]
            return np.ndarray((count, width), dtype=dt, buffer=self.buffers[view['buffer']],
                              offset=view.get('byteOffset', 0)+offset, strides=(view.get('byteStride', width*dt.itemsize), dt.itemsize)).copy()
        result = read(a['bufferView'], a.get('byteOffset', 0), a['count'], size, dtype) if 'bufferView' in a else np.zeros((a['count'], size), dtype=dtype)
        if 'sparse' in a:
            s = a['sparse']; indices = s['indices']; values = s['values']
            dt = np.dtype({5121:'u1', 5123:'<u2', 5125:'<u4'}[indices['componentType']])
            ids = read(indices['bufferView'], indices.get('byteOffset', 0), s['count'], 1, dt).ravel()
            result[ids] = read(values['bufferView'], values.get('byteOffset', 0), s['count'], size, dtype)
        if a.get('normalized') and dtype.kind in 'iu': result = np.maximum(result.astype(float)/np.iinfo(dtype).max, -1)
        return result

    def refresh(self):
        for i in self.order: self.world[i] = self.world[self.parents[i]] @ self.local[i] if i in self.parents else self.root @ self.local[i]

    def refresh_subtree(self, i):
        pending=[i]
        while pending:
            node=pending.pop()
            self.world[node]=self.world[self.parents[node]] @ self.local[node] if node in self.parents else self.root @ self.local[node]
            pending.extend(self.nodes[node].get('children',[]))

    def map_bones(self):
        extensions = self.document.get('extensions', {})
        semantic = extensions.get('VRMC_vrm', {}).get('humanoid', {}).get('humanBones', {}) or {v['bone']:v for v in extensions.get('VRM', {}).get('humanoid', {}).get('humanBones', [])}
        standard = {segment: semantic[name]['node'] for name, segment in VRM_SEGMENTS.items() if name in semantic}
        if 'upperChest' in semantic: standard['thorax'] = semantic['upperChest']['node']
        explicit_mapping = {}
        for segment in SEGMENTS:
            explicit = self.config.get('bone_map', {}).get(segment)
            if explicit is not None and explicit != '':
                i = explicit if type(explicit) is int else next((i for i,n in enumerate(self.nodes) if n.get('name') == explicit), None)
                if i is None:
                    matches = [i for i in self.order if i in self.bones and canonical(self.nodes[i].get('name', '')) == canonical(explicit)]
                    i = matches[0] if len(matches) == 1 else None
                if i not in self.bones: raise DocumentError(f'Human model: mapped {segment} bone {explicit!r} was not found in the skin')
                explicit_mapping[segment] = i
        mapping = _auto_bone_map(self.nodes, sorted(self.bones), {**standard, **explicit_mapping}, self.parents)
        if any(i not in self.bones for i in mapping.values()) or len(set(mapping.values())) != len(mapping):
            raise DocumentError('Human model: invalid or duplicate bone mapping')
        missing = [s for s in REQUIRED if s not in mapping]
        if missing: raise DocumentError('Human model: map these humanoid bones before rendering: '+', '.join(missing))
        return mapping

    def load_primitives(self):
        result = []
        for i, node in enumerate(self.nodes):
            if i not in self.active or 'mesh' not in node: continue
            for p in self.document['meshes'][node['mesh']]['primitives']:
                attributes = p['attributes']; positions = self.accessor(attributes['POSITION']).astype(float)
                # Retain the default morph pose (face/clothes details).
                for weight, target in zip(node.get('weights', self.document['meshes'][node['mesh']].get('weights', [])), p.get('targets', [])):
                    if 'POSITION' in target: positions += weight*self.accessor(target['POSITION'])
                indices = self.accessor(p['indices']).ravel() if 'indices' in p else np.arange(len(positions))
                mode = p.get('mode', 4)
                if mode == 5: indices = np.array([[indices[j], indices[j+1], indices[j+2]] if j%2==0 else [indices[j+1], indices[j], indices[j+2]] for j in range(len(indices)-2)]).ravel()
                elif mode == 6: indices = np.array([[indices[0], indices[j], indices[j+1]] for j in range(1,len(indices)-1)]).ravel()
                elif mode != 4: raise DocumentError('Human model: headless rendering requires triangle mesh primitives')
                material = self.document.get('materials', [])[p['material']] if 'material' in p else {}
                pbr = material.get('pbrMetallicRoughness', {}); texture = pbr.get('baseColorTexture', {})
                uv_key = 'TEXCOORD_'+str(texture.get('extensions', {}).get('KHR_texture_transform', {}).get('texCoord', texture.get('texCoord', 0)))
                uv = self.accessor(attributes[uv_key]).astype(float) if uv_key in attributes else None
                if uv is not None:
                    tr = texture.get('extensions', {}).get('KHR_texture_transform', {})
                    angle = tr.get('rotation', 0); c,s = math.cos(angle),math.sin(angle)
                    uv = (uv*tr.get('scale',[1,1])) @ np.array([[c,s],[-s,c]]) + tr.get('offset',[0,0])
                    uv[:,1] = 1-uv[:,1]  # glTF image origin is top-left; Bullet uses bottom-left.
                image = None
                if 'index' in texture:
                    tex = self.document['textures'][texture['index']]
                    source = tex.get('source', tex.get('extensions', {}).get('EXT_texture_webp', {}).get('source'))
                    data = self.document['images'][source]
                    if 'uri' in data: raw = base64.b64decode(data['uri'].split(',',1)[1])
                    else:
                        view = self.document['bufferViews'][data['bufferView']]; offset = view.get('byteOffset',0)
                        raw = self.buffers[view['buffer']][offset:offset+view['byteLength']]
                    image = Image.open(io.BytesIO(raw)).convert('RGBA')
                weights = self.accessor(attributes['WEIGHTS_0']).astype(float) if 'skin' in node else None
                if weights is not None:
                    lengths = weights.sum(axis=1,keepdims=True)
                    weights = weights / np.where(lengths>0,lengths,1)
                result.append(dict(node=i, positions=positions, indices=indices.astype(int), uv=uv,
                                   skin=node.get('skin'), weights=weights,
                                   joints=self.accessor(attributes['JOINTS_0']).astype(int) if weights is not None else None,
                                   color=pbr.get('baseColorFactor',[1,1,1,1]), image=image))
        return result

    def update(self, poses, time_s=0):
        raw_dt = 0 if self.last_time is None else time_s-self.last_time
        reset = self.last_time is None or raw_dt<0 or raw_dt>.25
        dt = 0 if reset else max(0,raw_dt); self.last_time = time_s
        for e in self.extras:
            i=e['i']; self.local[i][:3,:3] = e['bind'].as_matrix()*np.linalg.norm(self.local[i][:3,:3],axis=0)
        self.refresh()
        for segment,i,offset,q in self.bindings:
            part = poses[self.human['id']+'/'+segment]
            parent = self.world[self.parents[i]] if i in self.parents else self.root
            position = (np.linalg.inv(parent) @ part @ np.r_[offset,1])[:3]
            adjustment = Rotation.from_euler('XYZ', self.config.get('bone_offsets', {}).get(segment, [0,0,0]), degrees=True)
            local_q = rotation(parent).inv()*rotation(part)*q*adjustment
            self.local[i] = compose(position,local_q,np.linalg.norm(self.local[i][:3,:3],axis=0))
            self.refresh_subtree(i)
        for e in self.extras: self.update_extra(e,dt,reset)
        self.refresh()

    def update_extra(self,e,dt,reset):
        if e['mode']=='fixed_to_parent': return
        i=e['i']; settings=e['settings']
        q=e['bind']*Rotation.from_euler('XYZ',settings.get('rotation_deg',[0,0,0]),degrees=True)
        scale=np.linalg.norm(self.local[i][:3,:3],axis=0)
        self.local[i][:3,:3]=q.as_matrix()*scale; self.refresh_subtree(i)
        if e['mode']=='fixed_angle_to_parent': return
        position=self.world[i,:3,3]*.001
        if reset:
            e.update(angle=np.zeros(3),velocity=np.zeros(3),previous=position.copy(),previous_velocity=np.zeros(3))
        if dt>0:
            velocity=(position-e['previous'])/dt; acceleration=(velocity-e['previous_velocity'])/dt
            acceleration*=min(1,35/max(np.linalg.norm(acceleration),1e-30))
            parent=self.world[self.parents[i]] if i in self.parents else self.root
            gravity=rotation(parent).inv().apply(np.array([0,0,-9.81])-acceleration)
            length=max(.015,np.linalg.norm(e['tail'])*np.linalg.norm(self.world[i,:3,:3])/math.sqrt(3)*.001)
            pendulum=e['mode']=='weighted_ball_joint'
            mass=np.clip(settings.get('mass_kg',settings.get('mass',.05)),.001,20)
            stiffness=np.clip(settings.get('stiffness',.08 if pendulum else 2),0,200)
            damping=np.clip(settings.get('damping',.12 if pendulum else .5),0,50)
            limit=math.radians(np.clip(settings.get('limit_deg',70 if pendulum else 25),0,170))
            inertia=max(.001,mass*length*length); steps=max(1,math.ceil(dt*120)); step=dt/steps
            for _ in range(steps):
                direction=Rotation.from_rotvec(e['angle']).apply(q.apply(unit(e['tail'])))
                torque=np.cross(direction,gravity)*mass*length-e['angle']*stiffness-e['velocity']*damping
                e['velocity']+=torque*step/inertia
                e['velocity']*=min(1,12/max(np.linalg.norm(e['velocity']),1e-30))
                e['angle']+=e['velocity']*step
                if np.linalg.norm(e['angle'])>limit:
                    e['angle']=unit(e['angle'])*limit
                    outward=e['velocity']@e['angle']
                    if outward>0: e['velocity']-=e['angle']*outward/max(limit*limit,1e-12)
            e['previous']=position.copy(); e['previous_velocity']=velocity
        self.local[i][:3,:3]=(Rotation.from_rotvec(e['angle'])*q).as_matrix()*scale
        self.refresh_subtree(i)

    def vertices(self, primitive):
        p=primitive; homogeneous=np.column_stack([p['positions'],np.ones(len(p['positions']))])
        if p['skin'] is None: return (homogeneous @ self.world[p['node']].T)[:,:3]
        skin=self.document['skins'][p['skin']]
        matrices=self.world[skin['joints']] @ self.inverse_binds[p['skin']]
        result=np.zeros((len(homogeneous),4))
        # One influence at a time bounds working memory for high-poly models.
        for influence in range(p['weights'].shape[1]):
            result+=np.einsum('nij,nj->ni',matrices[p['joints'][:,influence]],homogeneous)*p['weights'][:,influence,None]
        return result[:,:3]

    def visible_triangles(self, primitive, visible_segments):
        """Keep partial build illustrations from revealing uninstalled limbs."""
        p=primitive; indices=p['indices'].reshape(-1,3)
        if 'face_segments' not in p:
            reverse={i:SEGMENTS.index(segment) for segment,i in self.mapping.items()}
            def owner(i):
                while i not in reverse and i in self.parents: i=self.parents[i]
                return reverse.get(i,SEGMENTS.index('pelvis'))
            if p['skin'] is None: owners=np.full(len(p['positions']),owner(p['node']))
            else:
                joint_owners=np.array([owner(i) for i in self.document['skins'][p['skin']]['joints']])
                strongest=p['joints'][np.arange(len(p['joints'])),np.argmax(p['weights'],axis=1)]
                owners=joint_owners[strongest]
            a,b,c=owners[indices].T
            p['face_segments']=np.where((a==b)|(a==c),a,np.where(b==c,b,a))
        return indices[np.isin(p['face_segments'],[SEGMENTS.index(s) for s in visible_segments])]
