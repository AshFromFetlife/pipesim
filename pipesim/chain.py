"""Length-controlled chains with stable link IDs and physical articulated joints."""
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
    return {'count': count, 'pitch_mm': pitch, 'length_mm': count*pitch,
            'requested_length_mm': length, 'link_catalog': catalog, 'a': a, 'b': b}


def _joint(index):
    return {'id': f'join-{index}', 'type': 'spherical',
            'a': {'part': f'link-{index}', 'port': 'a'},
            'b': {'part': f'link-{index+1}', 'port': 'b'},
            'limits': {'rotation_deg': [[-80, 80], [-80, 80], [-180, 180]]},
            'damping': .001, 'assembly': 'hook',
            'metadata': {'chain_link': True}}


def generate(parameters, library, components=None):
    """Grow at the free end; retained links keep their edited geometry and pose."""
    from .math3d import align_axis
    info = dimensions(parameters, library)
    count, a, b = info['count'], info['a'], info['b']
    result = copy.deepcopy(components or {'parts': [], 'joints': []})
    saved = {p['id']: p for p in result['parts']}
    if components:
        old = dimensions(components.get('parameter_reference', parameters), library)
        if old['link_catalog'] != info['link_catalog']:
            raise DocumentError('An edited chain keeps its link type. Add a new chain to use a different link.')
        expected = {f'link-{i}' for i in range(1, len(saved)+1)}
        if set(saved) != expected:
            raise DocumentError('This chain has individually removed or renamed links. Restore its link sequence before setting its length.')
    parts = []
    roll = Rotation.from_rotvec((a-b)/info['pitch_mm']*np.pi/2).as_matrix()
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
            joints.append(_joint(i))
    result.update(parts=parts, joints=joints, parameter_reference=copy.deepcopy(parameters))
    return result


def summary(instance, library):
    info = dimensions(instance.get('parameters', {}), library)
    return {k: v for k, v in info.items() if k not in ('a', 'b')} | {
        'id': instance['id'], 'layout_mode': instance.get('layout_mode', 'rigid'),
        'start_part': instance['id']+'/link-1', 'start_port': 'b',
        'end_part': instance['id']+f"/link-{info['count']}", 'end_port': 'a'}


def resize(assembly, object_id, parameters):
    """Resize atomically, refusing to remove a link still used by the design."""
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
    generated = generate(parameters, assembly.library, instance.get('components'))
    members = {object_id+'/'+p['id'] for p in generated['parts']}
    removed = {p for p in assembly.parts if p.startswith(object_id+'/')}-members
    blockers = [j['id'] for j in document.get('joints', []) if {j['a']['part'], j['b']['part']} & removed]
    blockers += ['world anchor on '+a['part'] for a in assembly.anchors if a['part'] in removed]
    blockers += ['load on '+l['part'] for l in document.get('loads', []) if l['part'] in removed]
    if blockers:
        raise DocumentError('Shortening would remove an attached link. Detach or move '+', '.join(blockers)+' first.')
    instance['parameters'] = copy.deepcopy(parameters)
    if 'components' in instance:
        instance['components'] = generated
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
    pinned_links = {a['part'] for a in assembly.anchors if a['part'] in members}
    pinned_links |= {j[e]['part'] for j in assembly.joints for e in ('a','b')
                     if j[e]['part'] in members and not {j['a']['part'],j['b']['part']} <= members}
    pinned = {}
    for i, name in enumerate(names):
        if name in pinned_links:
            pinned[i] = nodes[i].copy();pinned[i+1] = nodes[i+1].copy()
    if not pinned: pinned[0] = nodes[0].copy()
    aim = index if endpoint == 'b' else index+1
    local = links[index].local_frame({'port': endpoint or 'a'})[0]
    target = point(desired, local)
    if mode == 'rotate':
        pinned.setdefault(index, nodes[index].copy())
        a, b = (links[index].local_frame({'port': p})[0] for p in ('a', 'b'))
        target = nodes[index]+desired[:3,:3]@(a-b)

    def solve_segment(points, distances, start, end):
        points = points.copy();total = distances.sum();gap = np.linalg.norm(end-start)
        fallback = np.diff(points,axis=0)/distances[:,None]
        if gap >= total-1e-8:
            direction = (end-start)/max(gap,1e-9)
            return start+np.r_[0,np.cumsum(distances)][:,None]*direction
        old_axis=points[-1]-points[0];old_axis/=max(np.linalg.norm(old_axis),1e-9)
        straight=max(np.linalg.norm(np.cross(p-points[0],old_axis)) for p in points)<1e-5
        if straight and len(distances)>1 and np.ptp(distances)<1e-7:
            # An exact circular seed distributes a new bend across a long,
            # straight run. Iterating from a taut line can create a local kink.
            from scipy.optimize import brentq
            count=len(distances);axis=(end-start)/max(gap,1e-9)
            if gap<1e-9: axis=old_axis
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
        return points

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
            result[left:right+1]=solve_segment(nodes[left:right+1],lengths[left:right],constraints[left],constraints[right])
        first,last=ordered[0],ordered[-1]
        result[first]=constraints[first];result[last]=constraints[last]
        # Unconstrained tails follow the bend without a variable per joint.
        right_delta=align_axis(vectors[last-1],result[last]-result[last-1]) if last else np.eye(3)
        for i in range(last,len(lengths)): result[i+1]=result[i]+right_delta@vectors[i]
        left_delta=align_axis(vectors[first],result[first+1]-result[first]) if first<len(lengths) else np.eye(3)
        for i in range(first-1,-1,-1): result[i]=result[i+1]-left_delta@vectors[i]
        posed=copy.copy(assembly);posed.parts={pid:copy.copy(p) for pid,p in assembly.parts.items()}
        for i, part in enumerate(links):
            if part.id in pinned_links: continue
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
