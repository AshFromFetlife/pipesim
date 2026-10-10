"""Length-controlled chain, rope and webbing segments with articulated joints."""
import copy
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .document import DocumentError, substitute
from .math3d import point, pose_of, transform

DEFAULTS = {'length_mm': 1000, 'link_catalog': 'generic.chain-link'}
MAX_LINKS = 1000


def dimensions(parameters, library):
    parameters = {**DEFAULTS, **parameters}
    unknown = set(parameters)-set(DEFAULTS)
    if unknown:
        raise DocumentError('Unknown chain parameter: '+', '.join(sorted(unknown)))
    length = parameters['length_mm']
    if isinstance(length, bool) or not isinstance(length, (int, float)) or not math.isfinite(length) or length <= 0:
        raise DocumentError('Chain length must be a positive number in millimetres')
    catalog = parameters['link_catalog']
    definition = library.parts.get(catalog, {}) if isinstance(catalog, str) else {}
    if definition.get('kind') != 'chain' or not {'a', 'b'} <= definition.get('ports', {}).keys():
        raise DocumentError('Choose a chain link with a and b attachment ports from the library')
    definition = substitute(definition, definition.get('parameters', {}))
    a, b = (np.array(definition['ports'][p]['position_mm'], float) for p in ('a', 'b'))
    pitch = float(np.linalg.norm(b-a))
    if pitch < 1e-6:
        raise DocumentError('Chain link attachment ports must be at different positions')
    count = max(1, math.ceil(length/pitch-1e-10))
    if count > MAX_LINKS:
        raise DocumentError(f'Chain length exceeds {MAX_LINKS} links ({MAX_LINKS*pitch:g} mm for this link)')
    strength = definition.get('break_force_n')
    if strength is not None and (isinstance(strength, bool) or not isinstance(strength, (int, float))
                                 or not math.isfinite(strength) or strength <= 0):
        raise DocumentError('Flexible segment break force must be a positive finite number')
    rigidity = definition.get('axial_rigidity_n')
    if rigidity is not None and (isinstance(rigidity, bool) or not isinstance(rigidity, (int, float))
                                 or not math.isfinite(rigidity) or rigidity <= 0):
        raise DocumentError('Flexible segment axial rigidity must be a positive finite number')
    return {'count': count, 'pitch_mm': pitch, 'length_mm': count*pitch,
            'requested_length_mm': length, 'link_catalog': catalog, 'a': a, 'b': b,
            'profile': definition.get('flexible_profile', 'chain'), 'break_force_n': strength,
            'axial_rigidity_n': rigidity,
            'break_strain': strength/rigidity if strength is not None and rigidity is not None else None}


def _joint(index, strength=None):
    joint = {'id': f'join-{index}', 'type': 'spherical',
            'a': {'part': f'link-{index}', 'port': 'a'},
            'b': {'part': f'link-{index+1}', 'port': 'b'},
            'limits': {'rotation_deg': [[-80, 80], [-80, 80], [-180, 180]]},
            'damping': .001, 'assembly': 'hook',
            'metadata': {'chain_link': True}}
    if strength is not None:
        joint['break_force_n'] = strength
    return joint


def sync_profile_strength(components, parameters, library):
    """Keep posed compact-line joints rated by their selected material profile.

    Saved components retain link poses and geometry. Their generated internal
    joints are material defaults, so a corrected catalog rating must also
    reach an existing posed or attached line when it is opened again.
    """
    strength = dimensions(parameters, library)['break_force_n']
    for joint in components.get('joints', []):
        if not joint.get('metadata', {}).get('chain_link'):
            continue
        if strength is None:
            joint.pop('break_force_n', None)
        else:
            joint['break_force_n'] = strength
    return components


def _fit_span(points, distances, start, end, *, circular=False):
    points = points.copy();total = distances.sum();gap = np.linalg.norm(end-start)
    fallback = np.diff(points,axis=0)/distances[:,None]
    if gap >= total-1e-8:
        direction = (end-start)/max(gap,1e-9)
        return start+np.r_[0,np.cumsum(distances)][:,None]*direction
    old_axis=points[-1]-points[0];old_axis/=max(np.linalg.norm(old_axis),1e-9)
    straight=max(np.linalg.norm(np.cross(p-points[0],old_axis)) for p in points)<1e-5
    if (straight or circular) and len(distances)>1 and np.ptp(distances)<1e-7:
        # An exact circular seed distributes a new bend across a long,
        # straight run. Iterating from a taut line can create a local kink.
        from scipy.optimize import brentq
        count=len(distances);axis=(end-start)/max(gap,1e-9)
        if gap<1e-9: axis=old_axis if np.linalg.norm(old_axis)>1e-7 else np.array([0.,0.,1.])
        sideways=points[len(points)//2]-(start+end)/2;sideways-=axis*(sideways@axis)
        if np.linalg.norm(sideways)<1e-7:
            sideways=np.array([0.,0.,-1.]);sideways-=axis*(sideways@axis)
        if np.linalg.norm(sideways)<1e-7: sideways=np.array([1.,0,0])
        sideways/=np.linalg.norm(sideways)
        step=brentq(lambda v: distances[0]*np.sin(count*v/2)/np.sin(v/2)-gap,1e-10,2*np.pi/count)
        angles=(np.arange(count)-(count-1)/2)*step
        segments=distances[:,None]*(np.cos(angles)[:,None]*axis-np.sin(angles)[:,None]*sideways)
        return start+np.vstack((np.zeros(3),np.cumsum(segments,axis=0)))
    # A shortened, perfectly straight chain needs a bend to leave the
    # collinear singularity. Keep the perturbation deterministic.
    progress=np.linspace(0,1,len(points))[:,None]
    points+=(1-progress)*(start-points[0])+progress*(end-points[-1])
    axis = (end-start)/max(gap,1e-9)
    if len(points)>2 and max(np.linalg.norm(np.cross(p-start,axis)) for p in points)<1e-5:
        sideways = np.cross(axis, [0,1,0] if abs(axis[1])<.9 else [1,0,0])
        if np.linalg.norm(sideways)<1e-9: sideways=np.array([1.,0,0])
        sideways/=np.linalg.norm(sideways)
        points += np.sin(np.linspace(0,np.pi,len(points)))[:,None]*sideways*max(math.sqrt(max(0,total*total-gap*gap))*.6,1.)
    for _ in range(100):
        points[-1] = end
        for i in range(len(distances)-1,-1,-1):
            vector=points[i]-points[i+1];norm=np.linalg.norm(vector)
            points[i]=points[i+1]+(vector/norm if norm>1e-9 else -fallback[i])*distances[i]
        points[0] = start
        for i, distance in enumerate(distances):
            vector=points[i+1]-points[i];norm=np.linalg.norm(vector)
            points[i+1]=points[i]+(vector/norm if norm>1e-9 else fallback[i])*distance
        if np.linalg.norm(points[-1]-end)<1e-6: break
    if np.linalg.norm(points[-1]-end)>1e-6 and len(distances)>1 and np.ptp(distances)<1e-7:
        # Slow convergence near a straight configuration is not an impossible
        # span. Equal segments have an exact circular-arc construction.
        return _fit_span(points,distances,start,end,circular=True)
    return points


def generate(parameters, library, components=None, *, constraints=()):
    """Grow at the free end; retained links keep their edited geometry and pose."""
    from .math3d import align_axis
    info = dimensions(parameters, library)
    count, a, b = info['count'], info['a'], info['b']
    result = copy.deepcopy(components or {'parts': [], 'joints': []})
    saved = {p['id']: p for p in result['parts']}
    if components:
        old = dimensions(components.get('parameter_reference', parameters), library)
        expected = {f'link-{i}' for i in range(1, len(saved)+1)}
        if set(saved) != expected:
            raise DocumentError('This chain has individually removed or renamed links. Restore its link sequence before setting its length.')
        if old['link_catalog'] != info['link_catalog']:
            # Resample the saved centreline before replacing the link geometry.
            # Link counts and port offsets vary between chain, rope and strap.
            old_nodes = [point(transform(saved['link-1'].get('pose')), old['b'])]
            old_nodes += [point(transform(saved[f'link-{i}'].get('pose')), old['a'])
                          for i in range(1, len(saved)+1)]
            stations = np.r_[0, np.cumsum(np.linalg.norm(np.diff(old_nodes, axis=0), axis=1))]
            nodes = [np.asarray(old_nodes[0], float)]
            for i in range(1, count+1):
                distance = i*info['pitch_mm']
                segment = min(max(1, int(np.searchsorted(stations, distance))), len(saved))
                direction = np.asarray(old_nodes[segment])-np.asarray(old_nodes[segment-1])
                direction /= max(np.linalg.norm(direction), 1e-9)
                target = np.asarray(old_nodes[segment-1])+direction*(distance-stations[segment-1])
                step = target-nodes[-1]
                if np.linalg.norm(step) < 1e-9: step = direction
                nodes.append(nodes[-1]+step/np.linalg.norm(step)*info['pitch_mm'])
            # Chords across bends are shorter than their centreline stations.
            # Normalizing each chord independently accumulates endpoint drift;
            # changing material must instead redistribute the available slack.
            end=target
            if (abs(info['requested_length_mm']-old['requested_length_mm'])<1e-7 and
                    np.linalg.norm(old_nodes[-1]-old_nodes[0])<=info['length_mm']+1e-7):
                end=old_nodes[-1]
            pins={0:np.asarray(nodes[0]),count:np.asarray(end)}
            for distance,position,direction in constraints:
                station=np.clip(distance/info['pitch_mm'],0,count)
                index=min(count-1,max(0,int(math.floor(station))))
                fraction=station-index
                if fraction<1e-8:pins[index]=position
                elif fraction>1-1e-8:pins[index+1]=position
                else:
                    # An interior attachment remains on a rigid link, not on
                    # an invented hinge. Hold that link's tangent and fit the
                    # flexible spans on either side to its two endpoints.
                    pins[index]=position-direction*fraction*info['pitch_mm']
                    pins[index+1]=position+direction*(1-fraction)*info['pitch_mm']
            nodes=np.asarray(nodes)
            for left,right in zip(sorted(pins),sorted(pins)[1:]):
                nodes[left:right+1]=_fit_span(nodes[left:right+1],np.full(right-left,info['pitch_mm']),pins[left],pins[right])
            parts = []
            for i in range(1, count+1):
                matrix = np.eye(4)
                source_index = min(len(saved), max(1, int(np.searchsorted(
                    stations, (i-.5)*info['pitch_mm']))))
                old_matrix = transform(saved[f'link-{source_index}'].get('pose'))
                old_axis = old_matrix[:3,:3]@(old['a']-old['b'])
                matrix[:3,:3] = (align_axis(old_axis, nodes[i]-nodes[i-1])@
                                old_matrix[:3,:3]@align_axis(a-b, old['a']-old['b']))
                if info['profile']=='chain' and old['profile']!='chain' and i%2==0:
                    matrix[:3,:3] = matrix[:3,:3]@Rotation.from_rotvec(
                        (a-b)/info['pitch_mm']*np.pi/2).as_matrix()
                matrix[:3, 3] = nodes[i-1]-matrix[:3, :3]@b
                parts.append({'id': f'link-{i}', 'catalog': info['link_catalog'], 'pose': pose_of(matrix)})
            result.update(parts=parts, joints=[_joint(i, info['break_force_n']) for i in range(1,count)],
                          parameter_reference=copy.deepcopy(parameters))
            return result
    parts = []
    # Interleaved metal rings alternate by 90 degrees. Rope and webbing keep
    # their cross-section orientation between segments until posed or twisted.
    roll = (Rotation.from_rotvec((a-b)/info['pitch_mm']*np.pi/2).as_matrix()
            if info['profile'] == 'chain' else np.eye(3))
    for i in range(1, count+1):
        name = f'link-{i}'
        if name in saved:
            part = saved[name]
        else:
            matrix = np.eye(4)
            if parts:
                previous = transform(parts[-1].get('pose'))
                matrix[:3, :3] = previous[:3, :3]@roll
                matrix[:3, 3] = point(previous, a)-matrix[:3, :3]@b
            else:
                matrix[:3, :3] = align_axis(a-b, [0, 0, -1])
                matrix[:3, 3] = -matrix[:3, :3]@b
            part = {'id': name, 'catalog': info['link_catalog'], 'pose': pose_of(matrix)}
        parts.append(part)
    members = {p['id'] for p in parts}
    joints = [j for j in result.get('joints', []) if {j['a']['part'], j['b']['part']} <= members]
    known = {j['id'] for j in joints}
    for i in range(1, count):
        if f'join-{i}' not in known:
            # Do not silently repair an intentionally detached internal joint.
            if components and i < len(saved):
                raise DocumentError(f'Reconnect join-{i} before setting the chain length')
            joints.append(_joint(i, info['break_force_n']))
    result.update(parts=parts, joints=joints, parameter_reference=copy.deepcopy(parameters))
    return result


def summary(instance, library):
    info = dimensions(instance.get('parameters', {}), library)
    return {k: v for k, v in info.items() if k not in ('a', 'b')} | {
        'id': instance['id'], 'layout_mode': instance.get('layout_mode', 'rigid'),
        'start_part': instance['id']+'/link-1', 'start_port': 'b',
        'end_part': instance['id']+f"/link-{info['count']}", 'end_port': 'a'}


def resize(assembly, object_id, parameters):
    """Resize a flexible line while retaining its external connections."""
    from .grouping import restore_objects
    from .posing import _editable
    instance = next(o for o in assembly.doc['objects'] if o['id'] == object_id)
    dimensions(parameters, assembly.library)
    # Bake a saved animation frame before trimming its link/joint references.
    if assembly.doc.get('state', {}).get('joints'):
        editable = _editable(assembly, {object_id+'/link-1'})
        document = restore_objects(assembly.doc, editable.doc, assembly.base, assembly.library)
    else:
        document = copy.deepcopy(assembly.doc)
    instance = next(o for o in document['objects'] if o['id'] == object_id)
    old_info = dimensions(instance.get('parameters', {}), assembly.library)
    new_info = dimensions(parameters, assembly.library)
    constraints=[]
    if old_info['link_catalog']!=new_info['link_catalog']:
        local_from_world=np.linalg.inv(transform(instance.get('pose')))
        for joint in document.get('joints',[]):
            for side in ('a','b'):
                endpoint=joint[side];pid=endpoint['part']
                if not pid.startswith(object_id+'/link-'):continue
                part=assembly.parts[pid];local,_=part.local_frame(endpoint)
                fraction=float((local-old_info['b'])@(old_info['a']-old_info['b'])/old_info['pitch_mm']**2)
                number=int(pid.rsplit('-',1)[1])
                distance=(new_info['length_mm'] if number==old_info['count'] and fraction>1-1e-6 else
                          (number-1+fraction)*old_info['pitch_mm'])
                if distance>new_info['length_mm']+1e-7:continue
                center=old_info['b']+fraction*(old_info['a']-old_info['b'])
                matrix=local_from_world@part.matrix
                direction=matrix[:3,:3]@(old_info['a']-old_info['b'])/old_info['pitch_mm']
                constraints.append((distance,point(matrix,center),direction))
    generated = generate(parameters, assembly.library, instance.get('components'),constraints=constraints)
    members = {object_id+'/'+p['id'] for p in generated['parts']}
    removed = {p for p in assembly.parts if p.startswith(object_id+'/')}-members
    instance['parameters'] = copy.deepcopy(parameters)
    if 'components' in instance:
        instance['components'] = generated
    changed_profile = old_info['link_catalog'] != new_info['link_catalog']
    if old_info['count'] != new_info['count'] or changed_profile:
        # An attachment belongs to a station on the line, not to an incidental
        # link number. Keep its station when the pitch changes and clamp it to
        # the new free end when the line is shortened.
        from .document import Assembly, joint_kind
        from .math3d import align_axis
        geometry = copy.deepcopy(document)
        geometry['joints'] = []
        geometry['anchors'] = []
        geometry['loads'] = []
        after = Assembly.from_doc(geometry, assembly.base, assembly.library)
        def remap(endpoint):
            pid = endpoint.get('part', '')
            if not pid.startswith(object_id+'/link-'):
                return
            old_part = assembly.parts[pid]
            number = int(pid.rsplit('-', 1)[1])
            local, axis = old_part.local_frame(endpoint)
            fraction = float(np.dot(local-old_info['b'], old_info['a']-old_info['b'])
                             / old_info['pitch_mm']**2)
            terminal = number==old_info['count'] and fraction>1-1e-6
            distance = (new_info['length_mm'] if terminal else
                        max(0., min((number-1+fraction)*old_info['pitch_mm'],
                                    new_info['length_mm'])))
            station = distance/new_info['pitch_mm']
            new_number = min(new_info['count'], max(1, math.ceil(station-1e-9)))
            along = station-(new_number-1)
            new_pid = object_id+f'/link-{new_number}'
            new_part = after.parts[new_pid]
            old_center = old_info['b']+fraction*(old_info['a']-old_info['b'])
            radial_world = old_part.matrix[:3, :3]@(local-old_center)
            position = (new_info['b']+along*(new_info['a']-new_info['b'])+
                        new_part.matrix[:3, :3].T@radial_world)
            world_axis = old_part.matrix[:3, :3]@axis
            endpoint.clear()
            endpoint.update(part=new_pid, frame={'position_mm':position.tolist(),
                                                  'axis':(new_part.matrix[:3, :3].T@world_axis).tolist()})
        for joint in document.get('joints', []):
            remap(joint['a']); remap(joint['b'])
        for anchor in document.get('anchors', []):
            reference = {'part':anchor['part']}
            remap(reference)
            anchor['part'] = reference['part']
        for load in document.get('loads', []):
            reference = {'part':load['part']}
            if 'point_mm' in load:
                reference['frame'] = {'position_mm':load['point_mm']}
            remap(reference)
            load['part'] = reference['part']
            if 'point_mm' in load and 'frame' in reference:
                load['point_mm'] = reference['frame']['position_mm']
        for record in document.get('metadata', {}).get('detached_attachments', []):
            remap(record['joint']['a']); remap(record['joint']['b'])
        line_anchors = [(old,new) for old,new in zip(assembly.anchors,document.get('anchors', []))
                        if old['part'].startswith(object_id+'/')]
        if len(line_anchors)==1:
            old_anchor,new_anchor = line_anchors[0]
            after = Assembly.from_doc(document, assembly.base, assembly.library)
            correction = (assembly.parts[old_anchor['part']].matrix @
                          np.linalg.inv(after.parts[new_anchor['part']].matrix))
            instance['pose'] = pose_of(correction@transform(instance.get('pose')))
        # A free part attached to the edited end follows it. Move the whole
        # connected external component, so other joints on that part remain
        # intact. Anchored hosts are handled by the geometry check below.
        after = Assembly.from_doc(document, assembly.base, assembly.library)
        external = [j for j in document.get('joints', [])
                    if (j['a']['part'].startswith(object_id+'/') !=
                        j['b']['part'].startswith(object_id+'/'))]
        direct = {p['id']:p for p in document.get('parts', [])}
        objects = {o['id']:o for o in document.get('objects', []) if o['id'] != object_id}
        outside = {pid for pid in after.parts if not pid.startswith(object_id+'/')}
        neighbors = {pid:set() for pid in outside}
        for edge in after.joints:
            a,b = edge['a']['part'],edge['b']['part']
            if a in outside and b in outside:
                neighbors[a].add(b); neighbors[b].add(a)
        shifted = set()
        for joint in external:
            line_side = 'a' if joint['a']['part'].startswith(object_id+'/') else 'b'
            host_side = 'b' if line_side == 'a' else 'a'
            host_id = joint[host_side]['part']
            if host_id in shifted:
                continue
            component = {host_id}
            queue = [host_id]
            while queue:
                for adjacent in neighbors[queue.pop()]-component:
                    component.add(adjacent); queue.append(adjacent)
            if any(a['part'] in component for a in after.anchors):
                # A fixed target cannot follow the line. Bend the line toward
                # that target while its other already connected end stays put.
                line_point,_ = after.parts[joint[line_side]['part']].frame(joint[line_side])
                host_point,_ = after.parts[host_id].frame(joint[host_side])
                if np.linalg.norm(line_point-host_point) > .05:
                    line_id = joint[line_side]['part']
                    link = after.parts[line_id]
                    local,_ = link.local_frame(joint[line_side])
                    port = next((p for p in ('a','b') if
                                 np.linalg.norm(link.local_frame({'port':p})[0]-local)<1e-5),None)
                    if port:
                        temporarily = copy.deepcopy(document)
                        temporarily['joints'] = [j for j in temporarily['joints'] if j['id']!=joint['id']]
                        try:
                            free = Assembly.from_doc(temporarily, assembly.base, assembly.library)
                            owner = next(o for o in temporarily['objects'] if o['id']==object_id)
                            desired = free.parts[line_id].matrix.copy()
                            desired[:3,3] += host_point-line_point
                            posed = pose_chain(free, owner, line_id, desired, endpoint=port)
                            if posed and posed['poses']:
                                document = posed['document']
                                document.setdefault('joints',[]).append(copy.deepcopy(joint))
                                after = Assembly.from_doc(document, assembly.base, assembly.library)
                                direct = {p['id']:p for p in document.get('parts', [])}
                                objects = {o['id']:o for o in document.get('objects', []) if o['id'] != object_id}
                        except DocumentError:
                            pass
                continue
            line_point,_ = after.parts[joint[line_side]['part']].frame(joint[line_side])
            kind = joint_kind(joint)
            host_part = after.parts[host_id]
            rotation = np.eye(3)
            if kind == 'fixed':
                previous = next((j for j in assembly.doc.get('joints', []) if j['id']==joint['id']),None)
                if previous:
                    old_line = assembly.parts[previous[line_side]['part']]
                    new_line = after.parts[joint[line_side]['part']]
                    rotation = new_line.matrix[:3,:3]@old_line.matrix[:3,:3].T
            elif kind == 'revolute':
                line_axis = after.parts[joint[line_side]['part']].frame(joint[line_side])[1]
                host_axis = host_part.frame(joint[host_side])[1]
                rotation = align_axis(host_axis,line_axis)
            desired_host = host_part.matrix.copy()
            desired_host[:3,:3] = rotation@host_part.matrix[:3,:3]
            local,_ = host_part.local_frame(joint[host_side])
            desired_host[:3,3] = line_point-desired_host[:3,:3]@local
            delta_matrix = desired_host@np.linalg.inv(host_part.matrix)
            if not np.allclose(delta_matrix,np.eye(4),atol=1e-7,rtol=0):
                owners = {pid.split('/',1)[0] for pid in component if pid not in direct}
                if any(owner not in objects for owner in owners):
                    continue
                for pid in component & direct.keys():
                    direct[pid]['pose'] = pose_of(delta_matrix@after.parts[pid].matrix)
                for owner in owners:
                    objects[owner]['pose'] = pose_of(delta_matrix@transform(objects[owner].get('pose')))
                after = Assembly.from_doc(document, assembly.base, assembly.library)
            shifted.update(component)
    internal = {object_id+'/'+j['id'] for j in generated['joints']}
    gone = {j['id'] for j in assembly.joints if j['id'].startswith(object_id+'/')}-internal
    if 'state' in document:
        document['state']['joints'] = {k: v for k, v in document['state'].get('joints', {}).items() if k not in gone}
    if 'animation' in document:
        document['animation']['tracks'] = [t for t in document['animation'].get('tracks', []) if t['joint'] not in gone]
    if 'drives' in document:
        document['drives'] = [d for d in document['drives'] if d['driver'] not in gone and d['follower'] not in gone]
    document.get('metadata', {}).get('detached_attachments', [])[:] = [r for r in document.get('metadata', {}).get('detached_attachments', [])
        if not {r['joint']['a']['part'], r['joint']['b']['part']} & removed]
    document.pop('results', None); document.pop('build_plan', None)
    return document


def pose_chain(assembly, instance, selected, desired, mode='translate', endpoint=None, *, preview=False):
    """Linear-cost link projection (FABRIK), with fixed attachment boundaries.

    Each link retains its length and roll. Independent joint checks enforce the
    original angular limits. Large targets stop at reachable travel; they never
    detach a constrained end. Unusual edited link mechanisms use the general IK.
    """
    from .document import joint_kind
    from .math3d import align_axis
    from .posing import commit_transform
    from .snapping import _movement_coordinates
    names = [p for p in assembly.parts if p.startswith(instance['id']+'/')]
    expected = [instance['id']+f'/link-{i}' for i in range(1, len(names)+1)]
    if set(names) != set(expected): return None
    names = expected; members = set(names)
    internal = [j for j in assembly.joints if {j['a']['part'], j['b']['part']} <= members]
    if len(internal) != len(names)-1 or any(joint_kind(j) != 'spherical' for j in internal): return None
    links = [assembly.parts[p] for p in names];index = names.index(selected)
    nodes = np.array([links[0].frame({'port': 'b'})[0]]+[p.frame({'port': 'a'})[0] for p in links])
    vectors = np.diff(nodes, axis=0);lengths = np.linalg.norm(vectors, axis=1)
    if np.any(lengths < 1e-6): return None
    pinned = {}
    locked_links = set()
    def lock_link(name):
        i = names.index(name)
        pinned[i] = nodes[i].copy();pinned[i+1] = nodes[i+1].copy()
        locked_links.add(name)
    for anchor in assembly.anchors:
        if anchor['part'] in members:
            lock_link(anchor['part'])
    for joint in assembly.joints:
        if {joint['a']['part'],joint['b']['part']} <= members:
            continue
        for end in ('a','b'):
            connection_endpoint = joint[end]
            name = connection_endpoint['part']
            if name not in members:
                continue
            if joint_kind(joint) != 'spherical':
                lock_link(name)
                continue
            part = assembly.parts[name]
            local = part.local_frame(connection_endpoint)[0]
            i = names.index(name)
            for port, node in (('b', i), ('a', i+1)):
                if np.linalg.norm(local-part.local_frame({'port':port})[0]) < 1e-4:
                    pinned[node] = nodes[node].copy()
                    break
            else:
                # An interior attachment cannot be represented by a chain
                # node without adding a new segment at that station.
                lock_link(name)
    if not pinned: pinned[0] = nodes[0].copy()
    aim = index if endpoint == 'b' else index+1
    local = links[index].local_frame({'port': endpoint or 'a'})[0]
    target = point(desired, local)
    if mode == 'rotate':
        pinned.setdefault(index, nodes[index].copy())
        a, b = (links[index].local_frame({'port': p})[0] for p in ('a', 'b'))
        target = nodes[index]+desired[:3,:3]@(a-b)


    actual = assembly;blocked = None
    for fraction in (1., .5, .25, .125, .0625, .03125, .015625):
        wanted = nodes[aim]+fraction*(target-nodes[aim])
        handles={aim:wanted}
        if mode=='translate' and endpoint is None:
            shift=fraction*(desired[:3,3]-links[index].matrix[:3,3])
            handles={index:nodes[index]+shift,index+1:nodes[index+1]+shift}
        # Clamp the drag to the intersection of each fixed endpoint's reachable
        # ball, before projecting the intervening links onto exact lengths.
        for _ in range(20):
            for handle in handles:
                for at, position in pinned.items():
                    radius=lengths[min(at,handle):max(at,handle)].sum();vector=handles[handle]-position;distance=np.linalg.norm(vector)
                    if distance>radius:
                        correction=vector*(radius/distance-1)
                        handles={key:value+correction for key,value in handles.items()}
        constraints = {**handles, **pinned}
        result=nodes.copy();ordered=sorted(constraints)
        for left,right in zip(ordered,ordered[1:]):
            result[left:right+1]=_fit_span(nodes[left:right+1],lengths[left:right],constraints[left],constraints[right])
        first,last=ordered[0],ordered[-1]
        result[first]=constraints[first];result[last]=constraints[last]
        # Unconstrained tails follow the bend without a variable per joint.
        right_delta=align_axis(vectors[last-1],result[last]-result[last-1]) if last else np.eye(3)
        for i in range(last,len(lengths)): result[i+1]=result[i]+right_delta@vectors[i]
        left_delta=align_axis(vectors[first],result[first+1]-result[first]) if first<len(lengths) else np.eye(3)
        for i in range(first-1,-1,-1): result[i]=result[i+1]-left_delta@vectors[i]
        posed=copy.copy(assembly);posed.parts={pid:copy.copy(p) for pid,p in assembly.parts.items()}
        for i, part in enumerate(links):
            if part.id in locked_links: continue
            matrix=part.matrix.copy()
            matrix[:3,:3]=align_axis(vectors[i],result[i+1]-result[i])@matrix[:3,:3]
            if mode=='rotate' and i==index and fraction==1: matrix[:3,:3]=desired[:3,:3]
            matrix[:3,3]=result[i]-matrix[:3,:3]@part.local_frame({'port':'b'})[0]
            posed.parts[part.id].matrix=matrix
        try:
            _movement_coordinates(assembly,posed)
            actual=posed;break
        except DocumentError as exc: blocked=str(exc)
    poses={pid:pose_of(actual.parts[pid].matrix) for pid in names if not np.allclose(actual.parts[pid].matrix,assembly.parts[pid].matrix,atol=1e-7,rtol=0)}
    matrix=actual.parts[selected].matrix
    position_error=float(np.linalg.norm(matrix[:3,3]-desired[:3,3]))
    angle_error=float(np.rad2deg(Rotation.from_matrix(matrix[:3,:3]@desired[:3,:3].T).magnitude()))
    limited=position_error>.5 if mode=='translate' else angle_error>.1
    response={'poses':poses,'moved':list(poses),'seed':[],
            'position_error_mm':position_error,'angle_error_deg':angle_error,'limited':limited,
            'message':('Chain stopped at its available reach or joint limits' if limited else 'Chain links follow the movement') if poses or not blocked else blocked}
    if not preview: response['document']=commit_transform(assembly,poses)
    return response
