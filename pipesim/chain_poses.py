"""Ready-to-use centreline shapes for grouped chain, rope and webbing.

The presets edit link poses, not the chain's physical joint definitions. Fixed
attachments remain fixed; free spans bend around them as far as joint travel
allows. Whole-line movement subsequently transports the saved shape rigidly.
"""

import copy

import numpy as np
from scipy.spatial.transform import Rotation

from .chain import _fit_span, dimensions, generate
from .document import Assembly, DocumentError, joint_kind
from .math3d import align_axis, pose_of, transform
from .snapping import _movement_coordinates


PRESETS = {
    'straight-down': 'Straight down',
    'straight-across': 'Straight across',
    'circle-touch': 'Circle · ends touching',
    'circle-overlap': 'Circle · overlap one segment',
    'square': 'Rounded square',
    'u-deep': 'Deep U',
    'u-wide': 'Wide U',
    'teardrop': 'Teardrop loop',
    'hanging-arc': 'Hanging arc',
    's-curve': 'S curve',
    'spiral-two': 'Flat spiral · two turns',
    'spiral-four': 'Flat spiral · four turns',
    'helix': 'Helix · three turns',
}


def _sample_curve(points, count, pitch, *, close=False):
    """Sample equal arc stations, then give every physical link exact pitch."""
    stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    if stations[-1] < 1e-8:
        return np.arange(count + 1)[:, None] * np.array([0., 0., -pitch])
    distance = np.linspace(0., stations[-1], count + 1)
    nodes = np.column_stack([np.interp(distance, stations, points[:, axis]) for axis in range(3)])
    vectors = np.diff(nodes, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    vectors /= np.maximum(lengths[:, None], 1e-12)
    if close and count >= 3:
        # Independent chord normalization can open an otherwise closed loop.
        for _ in range(24):
            vectors -= vectors.mean(axis=0)
            vectors /= np.maximum(np.linalg.norm(vectors, axis=1)[:, None], 1e-12)
    return np.vstack((np.zeros(3), np.cumsum(vectors * pitch, axis=0)))


def preset_nodes(preset, count, pitch):
    """World-independent segment endpoints for the selected shape."""
    if preset not in PRESETS:
        raise DocumentError('Choose a flexible-line shape from the list')
    if count < 3 and preset not in ('straight-down', 'straight-across'):
        return preset_nodes('straight-down', count, pitch)
    if preset == 'straight-down':
        return np.arange(count + 1)[:, None] * np.array([0., 0., -pitch])
    if preset == 'straight-across':
        return np.arange(count + 1)[:, None] * np.array([pitch, 0., 0.])
    if preset == 'circle-overlap':
        angle = 2 * np.pi * np.arange(count + 1) / (count - 1)
        points = np.column_stack((np.sin(angle), np.zeros(count + 1), np.cos(angle)))
        vectors = np.diff(points, axis=0)
        vectors *= pitch / np.linalg.norm(vectors, axis=1)[:, None]
        return np.vstack((np.zeros(3), np.cumsum(vectors, axis=0)))

    samples = max(1024, min(8192, count * 8))
    u = np.linspace(0., 1., samples + 1)
    angle = 2 * np.pi * u
    close = preset in ('circle-touch', 'square', 'teardrop')
    if preset == 'circle-touch':
        points = np.column_stack((np.sin(angle), np.zeros_like(u), np.cos(angle)))
    elif preset == 'square':
        cosine, sine = np.cos(angle), np.sin(angle)
        radius = (abs(cosine)**8 + abs(sine)**8)**(-1/8)
        points = np.column_stack((cosine * radius, np.zeros_like(u), sine * radius))
    elif preset == 'teardrop':
        width = .22 + .78 * np.sin(angle / 2)**2
        points = np.column_stack((np.sin(angle) * width, np.zeros_like(u), np.cos(angle) - 1))
    elif preset in ('u-deep', 'u-wide'):
        height = 2.0 if preset == 'u-deep' else .35
        leg = np.linspace(0., 1., samples // 3 + 1)
        arc = np.linspace(np.pi, 2 * np.pi, samples // 3 + 1)
        points = np.vstack((
            np.column_stack((-np.ones_like(leg), np.zeros_like(leg), -height * leg)),
            np.column_stack((np.cos(arc[1:]), np.zeros(len(arc) - 1), -height + np.sin(arc[1:]))),
            np.column_stack((np.ones_like(leg[1:]), np.zeros(len(leg) - 1), -height * (1 - leg[1:]))),
        ))
    elif preset == 'hanging-arc':
        x = 2 * u - 1
        points = np.column_stack((x, np.zeros_like(u), -.8 * (1 - x * x)))
    elif preset == 's-curve':
        points = np.column_stack((2 * u - 1, np.zeros_like(u), .45 * np.sin(2 * np.pi * u)))
    elif preset.startswith('spiral-'):
        turns = min(2 if preset == 'spiral-two' else 4, max(.5, count / 8))
        theta = 2 * np.pi * turns * u
        radius = .35 + 1.65 * u
        points = np.column_stack((radius * np.cos(theta), np.zeros_like(u), radius * np.sin(theta)))
    else:  # helix
        turns = min(3, max(.5, count / 8))
        theta = 2 * np.pi * turns * u
        points = np.column_stack((np.cos(theta), np.sin(theta), -2 * u))
    return _sample_curve(points, count, pitch, close=close)


def _constraints(assembly, names, nodes):
    """Pin attachment points; keep links rigid where the connection requires it."""
    index = {name: i for i, name in enumerate(names)}
    pinned = {}
    locked = set()

    def lock(name):
        i = index[name]
        pinned[i] = nodes[i].copy()
        pinned[i + 1] = nodes[i + 1].copy()
        locked.add(name)

    for anchor in assembly.anchors:
        if anchor['part'] in index:
            lock(anchor['part'])
    for joint in assembly.joints:
        inside = [side for side in ('a', 'b') if joint[side]['part'] in index]
        if len(inside) == 2 and not joint.get('metadata', {}).get('chain_link'):
            lock(joint['a']['part'])
            lock(joint['b']['part'])
            continue
        if len(inside) != 1:
            continue
        side = inside[0]
        name = joint[side]['part']
        if joint_kind(joint) != 'spherical' or joint.get('locked'):
            lock(name)
            continue
        part = assembly.parts[name]
        point = part.local_frame(joint[side])[0]
        for port, node in (('b', index[name]), ('a', index[name] + 1)):
            if np.linalg.norm(point - part.local_frame({'port': port})[0]) < 1e-4:
                pinned[node] = nodes[node].copy()
                break
        else:
            lock(name)
    return pinned, locked


def _align_to_pins(seed, pins):
    if not pins:
        return seed
    keys = sorted(pins)
    source = seed[keys]
    target = np.array([pins[i] for i in keys])
    if len(keys) == 1 or np.linalg.norm(source - source.mean(axis=0)) < 1e-7:
        return seed + (target - source).mean(axis=0)
    a, b = source - source.mean(axis=0), target - target.mean(axis=0)
    u, _, vt = np.linalg.svd(a.T @ b)
    rotation = vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T
    return (seed - source.mean(axis=0)) @ rotation.T + target.mean(axis=0)


def _project(seed, pitch, pins):
    """Keep exact link lengths and all pinned nodes while fitting free spans."""
    result = seed.copy()
    if not pins:
        vectors = np.diff(result, axis=0)
        vectors *= pitch / np.maximum(np.linalg.norm(vectors, axis=1)[:, None], 1e-12)
        result = np.vstack((result[0], result[0] + np.cumsum(vectors, axis=0)))
        return result + seed.mean(axis=0) - result.mean(axis=0)
    keys = sorted(pins)
    vectors = np.diff(result, axis=0)
    vectors *= pitch / np.maximum(np.linalg.norm(vectors, axis=1)[:, None], 1e-12)
    first, last = keys[0], keys[-1]
    result[first] = pins[first]
    for i in range(first - 1, -1, -1):
        result[i] = result[i + 1] - vectors[i]
    for left, right in zip(keys, keys[1:]):
        result[left:right + 1] = _fit_pose_span(result[left:right + 1],
                                               pitch, pins[left], pins[right])
    result[last] = pins[last]
    for i in range(last, len(vectors)):
        result[i + 1] = result[i] + vectors[i]
    return result


def _fit_pose_span(seed, pitch, start, end):
    """Project a shaped span onto exact link lengths without erasing its bend.

    The general chain fitter can replace a slow-to-converge seed with a circular
    arc. That is useful for dragging a taut chain, but loses a requested U,
    square or spiral between two fixed points. Alternating disjoint link pairs
    keeps the authored seed while both endpoints remain fixed.
    """
    count = len(seed) - 1
    if count <= 1 or np.linalg.norm(end - start) >= count * pitch - 1e-8:
        return _fit_span(seed, np.full(count, pitch), start, end)
    points = seed.copy()
    progress = np.linspace(0., 1., count + 1)[:, None]
    points += (1 - progress) * (start - points[0]) + progress * (end - points[-1])
    points[0], points[-1] = start, end
    for _ in range(4000):
        for parity in (0, 1):
            index = np.arange(parity, count, 2)
            if not len(index):
                continue
            vector = points[index + 1] - points[index]
            length = np.linalg.norm(vector, axis=1)
            weight_left = (index != 0).astype(float)
            weight_right = (index + 1 != count).astype(float)
            correction = vector * ((length - pitch) /
                                   (np.maximum(length, 1e-9) * (weight_left + weight_right)))[:, None]
            points[index] += correction * weight_left[:, None]
            points[index + 1] -= correction * weight_right[:, None]
        points[0], points[-1] = start, end
        if np.max(np.abs(np.linalg.norm(np.diff(points, axis=0), axis=1) - pitch)) < 1e-4:
            return points
    residual = np.max(np.abs(np.linalg.norm(np.diff(points, axis=0), axis=1) - pitch))
    if residual < .01:
        return points
    # Very close to full extension, the circular construction is the stable
    # exact solution. It also covers degenerate seeds with coincident nodes.
    return _fit_span(seed, np.full(count, pitch), start, end)


def apply_preset(assembly, object_id, preset):
    """Pose a compact flexible line, preserving every existing attachment."""
    from .grouping import restore_objects
    from .posing import _editable

    if preset not in PRESETS:
        raise DocumentError('Choose a flexible-line shape from the list')
    instance = next((o for o in assembly.doc.get('objects', []) if o['id'] == object_id), None)
    if instance is None or instance['template'] != 'chain':
        raise DocumentError('Select a flexible line')
    if assembly.doc.get('state', {}).get('joints'):
        editable = _editable(assembly, {object_id + '/link-1'})
        baked = restore_objects(assembly.doc, editable.doc, assembly.base, assembly.library)
        assembly = Assembly.from_doc(baked, assembly.base, assembly.library)
        instance = next(o for o in assembly.doc['objects'] if o['id'] == object_id)
    info = dimensions(instance.get('parameters', {}), assembly.library)
    count, pitch = info['count'], info['pitch_mm']
    names = [object_id + f'/link-{i}' for i in range(1, count + 1)]
    links = [assembly.parts[name] for name in names]
    old = np.array([links[0].frame({'port': 'b'})[0]] +
                   [link.frame({'port': 'a'})[0] for link in links])
    pinned, locked = _constraints(assembly, names, old)
    parent = transform(instance.get('pose'))
    target = preset_nodes(preset, count, pitch) @ parent[:3, :3].T
    target += old.mean(axis=0) - target.mean(axis=0)
    target = _align_to_pins(target, pinned)
    local_axis = info['a'] - info['b']
    roll = (Rotation.from_rotvec(local_axis / pitch * np.pi / 2).as_matrix()
            if info['profile'] == 'chain' else np.eye(3))
    best = None
    for fraction in (1., .75, .5, .25, .125, .0625, .03125):
        seed = old + fraction * (target - old)
        try:
            nodes = _project(seed, pitch, pinned)
        except (ValueError, FloatingPointError):
            continue
        trial = copy.copy(assembly)
        trial.parts = {name: copy.copy(part) for name, part in assembly.parts.items()}
        previous = None
        for i, link in enumerate(links):
            vector = nodes[i + 1] - nodes[i]
            if link.id in locked:
                matrix = link.matrix.copy()
            else:
                matrix = link.matrix.copy()
                if previous is None:
                    matrix[:3, :3] = align_axis(old[i + 1] - old[i], vector) @ matrix[:3, :3]
                else:
                    matrix[:3, :3] = align_axis(nodes[i] - nodes[i - 1], vector) @ previous @ roll
                matrix[:3, 3] = nodes[i] - matrix[:3, :3] @ info['b']
            trial.parts[link.id].matrix = matrix
            previous = matrix[:3, :3]
        try:
            _movement_coordinates(assembly, trial)
        except DocumentError:
            continue
        best = (trial, fraction)
        break
    if best is None:
        return {'document': copy.deepcopy(assembly.doc), 'changed': False,
                'message': 'The existing connections hold this line in its current shape.'}
    trial, fraction = best
    doc = copy.deepcopy(assembly.doc)
    instance = next(o for o in doc['objects'] if o['id'] == object_id)
    components = copy.deepcopy(instance.get('components') or generate(instance.get('parameters', {}), assembly.library))
    saved = {part['id']: part for part in components['parts']}
    inverse = np.linalg.inv(parent)
    changed = False
    for i, name in enumerate(names, 1):
        matrix = trial.parts[name].matrix
        changed |= not np.allclose(matrix, assembly.parts[name].matrix, atol=1e-7, rtol=0)
        saved[f'link-{i}']['pose'] = pose_of(inverse @ matrix)
    if not changed:
        return {'document': doc, 'changed': False,
                'message': 'The line already has the closest shape its connections allow.'}
    instance['components'] = components
    instance['pose_preset'] = preset
    doc.pop('results', None)
    doc.pop('build_plan', None)
    return {'document': doc, 'changed': True, 'limited': fraction < 1,
            'message': ('Shape fitted around the existing connections.' if fraction == 1 else
                        'Shape applied as far as the current connections and joint travel allow.')}
