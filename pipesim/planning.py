"""Hierarchical assembly planning by reverse, collision checked sliding cuts."""
from __future__ import annotations
import math
import time
import numpy as np
import pybullet as pb
from scipy.spatial import ConvexHull
from .document import DocumentError, joint_kind
from .math3d import unit, pose_of, point, UnionFind, align_axis
from .validation import CollisionWorld
from .geometry import mesh_for_part, collision_primitives, shape_mesh, shape_transform

PLANNING_ALGORITHM_VERSION = 3

class _StopPlanning(Exception):
    def __init__(self, reason, cancelled=False, details=None):
        self.reason, self.cancelled, self.details = reason, cancelled, details


class _PreferredSearchBudget(Exception):
    pass


def plan_build(assembly, search_limit=None, *, progress=None, cancelled=None):
    """Plan parallel sliding cuts with grounded preparation and explicit lifts.

    Each candidate is a component after parallel socket edges are cut. Thus a
    frame cap or a pipe with two same-DOF fittings moves together; no power-set
    enumeration is needed. Every external collision pair is swept independently,
    and its result can be reused by other subsets of the same design.
    """
    assembly.require_finished('build planning')
    options = assembly.doc.get('build', {})
    limit = search_limit if search_limit is not None else options.get('search_limit', 3000)
    started = time.monotonic()
    time_limit = options.get('time_limit_s', 300)
    query_limit = options.get('collision_query_limit', 250000)
    for name, value in (('search_limit', limit), ('time_limit_s', time_limit),
                        ('collision_query_limit', query_limit)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise DocumentError(f'Build {name} must be a finite non-negative number')
    if not isinstance(limit, int) or not isinstance(query_limit, int) or time_limit <= 0 or query_limit <= 0:
        raise DocumentError('Build limits require integer search/collision budgets and a positive time budget')
    stats = {'states_explored': 0, 'paths_checked': 0, 'collision_queries': 0,
             'total_parts': len(assembly.parts), 'planned_parts': 0, 'remaining_parts': len(assembly.parts)}
    deepest_parts = None
    preferred_deadline = None
    assisted_search = False
    last_progress = -1.
    input_hash = assembly.input_hash
    remaining = frozenset(assembly.parts)
    require_stability = options.get('require_stability', True)
    fixtures = {f['part'] for f in options.get('fixtures', [])}
    allow_fixtures = options.get('allow_temporary_supports', False)
    anchored = {a['part'] for a in assembly.anchors}
    lift_limit = options.get('max_lift_mass_kg', 15.)
    if not isinstance(lift_limit, (int, float)) or isinstance(lift_limit, bool) or not math.isfinite(lift_limit) or lift_limit <= 0:
        raise DocumentError('Build max_lift_mass_kg must be a positive finite number')
    part_masses = {pid: part.mass for pid, part in assembly.parts.items()}
    if 'max_lift_mass_kg' not in options:
        # An indivisible purchased panel can already exceed the preferred
        # handling target. Keep it whole and explicitly require assistance.
        lift_limit = max(lift_limit, max(part_masses.values(), default=0.))
    stable_cache, ground_points, mass_centers, blocked, pair_cache = {}, {}, {}, {}, {}
    ground_hulls = {}
    failed, solved = set(), {}
    part_hulls, support_clouds, floor_planes, plane_candidates, full_plane_support = {}, {}, {}, {}, {}
    joint_data = []
    adjacency = {p: [] for p in remaining}
    for j in assembly.joints:
        a, b = j['a']['part'], j['b']['part']
        _, _, axis = assembly.joint_frames(j)
        port = assembly.parts[a].ports[j['a']['port']] if j.get('type') == 'socket' else None
        item = (j, a, b, unit(axis), port)
        joint_data.append(item)
        adjacency[a].append(item)
        adjacency[b].append(item)
    backbone = anchored | {other for pid in anchored for _, a, b, _, _ in adjacency[pid]
                           for other in (b if a == pid else a,)
                           if assembly.parts[other].kind == 'member'}

    def report(message, force=False):
        nonlocal last_progress
        elapsed = time.monotonic()-started
        if cancelled and cancelled():
            raise _StopPlanning('Build planning cancelled', True)
        if elapsed > time_limit:
            raise _StopPlanning('Planning time budget reached; no impossibility is implied')
        if stats['collision_queries'] >= query_limit:
            raise _StopPlanning('Collision work budget reached; no impossibility is implied')
        if preferred_deadline is not None and elapsed >= preferred_deadline:
            raise _PreferredSearchBudget()
        if progress and (force or elapsed-last_progress >= .2):
            progress({**stats, 'message': ('Assisted build search: ' if assisted_search else '')+message,
                      'elapsed_s': round(elapsed, 3)})
            last_progress = elapsed

    def outcome(status, reason, **extra):
        return {'status': status, 'reason': reason, 'steps': [], 'input_sha256': input_hash,
                **stats, 'elapsed_s': round(time.monotonic()-started, 3), **extra}

    def support_cloud(subset):
        if subset not in support_clouds:
            vertices = []
            for pid in sorted(subset):
                if pid not in part_hulls:
                    report('Finding floor support faces for preparation')
                    p = assembly.parts[pid]
                    points = mesh_for_part(p, True).vertices
                    part_hulls[pid] = points[ConvexHull(points).vertices]
                    mass_centers[pid] = (p.mass, point(p.matrix, p.center_of_mass))
                vertices.append(part_hulls[pid])
            points = np.concatenate(vertices)
            hull = ConvexHull(points)
            center = np.average([mass_centers[p][1] for p in sorted(subset)], axis=0,
                                weights=[mass_centers[p][0] for p in sorted(subset)])
            support_clouds[subset] = (points[hull.vertices], center, hull.equations[:, :3])
        return support_clouds[subset]

    def supported_on_plane(points, center, up, level):
        contacts = points[points @ up <= level+1.]
        if not len(contacts):
            return False
        u = unit(np.cross(up, np.eye(3)[np.argmin(np.abs(up))]))
        basis = np.array([u, np.cross(up, u)])
        projected = np.unique(np.round(contacts @ basis.T, 6), axis=0)
        com = center @ basis.T
        if len(projected) == 1:
            return np.linalg.norm(projected[0]-com) <= 1.
        if np.linalg.matrix_rank(projected-projected.mean(axis=0), tol=1e-6) < 2:
            direction = projected[np.argmax(np.linalg.norm(projected-projected[0], axis=1))]-projected[0]
            direction /= np.linalg.norm(direction)
            offset = com-projected[0]
            stations = (projected-projected[0]) @ direction
            return (np.linalg.norm(offset-direction*(offset @ direction)) <= 1. and
                    stations.min()-1. <= offset @ direction <= stations.max()+1.)
        hull = ConvexHull(projected)
        return bool(np.max(hull.equations[:, :2] @ com+hull.equations[:, 2]) <= 1.)

    def floor_plane(subset, base=None, axis=None, support_full=True):
        base = subset if base is None else base
        key = (subset, base, None if axis is None else tuple(axis), support_full)
        if key in floor_planes:
            return floor_planes[key]
        points, center, normals = support_cloud(subset)
        base_points, base_center, _ = support_cloud(base)
        if subset not in plane_candidates:
            _, indices = np.unique(np.round(np.vstack(([0., 0., 1.], -normals)), 7), axis=0, return_index=True)
            directions_ = np.vstack(([0., 0., 1.], -normals))[indices]
            projections = points @ directions_.T
            levels = projections.min(axis=0)
            spans = projections.max(axis=0)-levels
            order = sorted(range(len(directions_)), key=lambda i: (float(spans[i]), -float(directions_[i, 2])))
            plane_candidates[subset] = [(directions_[i], float(levels[i])) for i in order]
        for index, (up, level) in enumerate(plane_candidates[subset]):
            if index % 32 == 0:
                report('Checking stable floor orientations for a connected group')
            if axis is not None and up @ axis < -1e-8:
                continue
            if float(np.min(base_points @ up)) > level+1.:
                continue
            if not supported_on_plane(base_points, base_center, up, level):
                continue
            support_key = (subset, index)
            if support_full and support_key not in full_plane_support:
                full_plane_support[support_key] = supported_on_plane(points, center, up, level)
            if not support_full or full_plane_support[support_key]:
                floor_planes[key] = up
                return up
        floor_planes[key] = None
        return None

    def stable(subset):
        if not require_stability or not subset:
            return True
        if subset in stable_cache:
            return stable_cache[subset]
        groups = UnionFind(subset)
        for j, a, b, axis, port in joint_data:
            if a in subset and b in subset and joint_kind(j) == 'fixed':
                groups.union(a, b)
        ground = assembly.doc.get('environment', {}).get('ground', True)
        floor = assembly.doc.get('environment', {}).get('ground_z_mm', 0)
        for group in groups.groups():
            report('Checking support for the retained assembly')
            if set(group) & (fixtures if allow_fixtures else set()):
                continue
            if not ground:
                if set(group) & anchored:
                    continue
                stable_cache[subset] = False
                return False
            contacts = []
            contacting = []
            for pid in group:
                if pid not in ground_points:
                    p = assembly.parts[pid]
                    vertices = mesh_for_part(p, True).vertices
                    ground_points[pid] = vertices[np.abs(vertices[:, 2]-floor) <= 1., :2]
                    mass_centers[pid] = (p.mass, point(p.matrix, p.center_of_mass))
                if len(ground_points[pid]):
                    contacting.append(pid)
            contact_key = tuple(sorted(contacting))
            if not contact_key:
                stable_cache[subset] = False
                return False
            com = np.average([mass_centers[p][1] for p in group], axis=0,
                             weights=[mass_centers[p][0] for p in group])
            if contact_key not in ground_hulls:
                points = np.unique(np.round(np.concatenate([ground_points[p] for p in contact_key]), 4), axis=0)
                if len(points) > 2 and np.linalg.matrix_rank(points-points.mean(axis=0)) == 2:
                    ground_hulls[contact_key] = ('polygon', ConvexHull(points).equations)
                else:
                    ground_hulls[contact_key] = ('degenerate', points)
            shape, data = ground_hulls[contact_key]
            supported = (np.max(data[:, :2] @ com[:2]+data[:, 2]) <= 1. if shape == 'polygon' else
                         supported_on_plane(np.column_stack((data, np.full(len(data), floor))), com,
                                            np.array([0., 0., 1.]), floor))
            if not supported:
                stable_cache[subset] = False
                return False
        stable_cache[subset] = True
        return True

    def directions(group, subset):
        axes, restrictions = [], []
        for j, a, b, axis, port in joint_data:
            if not {a, b} <= subset or ((a in group) == (b in group)):
                continue
            axes.extend((axis, -axis))
            if port and port.get('assembly') in ('slide', 'glue'):
                restrictions.append((axis, None if port.get('through') else (-1 if a in group else 1)))
            elif port and port.get('assembly') == 'radial':
                # Split clamps open across the bore in their own frame. World
                # X/Y/Z alone lose every radial route after a general rotation.
                # Prefer fitting-local axes so attached branches move away from
                # their pipe, then add a complete orthogonal transverse basis.
                bases = list(assembly.parts[a].matrix[:3, :3].T)
                transverse = unit(np.cross(axis, np.eye(3)[np.argmin(np.abs(axis))]))
                bases.extend((transverse, np.cross(axis, transverse)))
                for basis in bases:
                    radial = basis-axis*(basis @ axis)
                    if np.linalg.norm(radial) > 1e-8:
                        radial = unit(radial)
                        axes.extend((radial, -radial))
        axes.extend(np.eye(3)[i]*s for i in range(3) for s in (-1, 1))
        result = []
        for axis in axes:
            if any(abs(axis @ r) < .999 or (sign is not None and sign*(axis @ r) < .999)
                   for r, sign in restrictions):
                continue
            if not any(axis @ old > 1.-1e-9 for old in result):
                result.append(axis)
        return result

    def candidates(subset, manual=False):
        degree = {p: sum(a in subset and b in subset for _, a, b, _, _ in adjacency[p]) for p in subset}
        singles = sorted(subset, key=lambda p: (degree[p], -assembly.parts[p].matrix[2, 3], p))
        seen = set()
        bundles = []
        for pid in singles:
            if assembly.parts[pid].kind != 'member':
                continue
            components = connected_groups(subset-{pid})
            if len(components) < 2:
                continue
            if manual:
                retained = max(components, key=lambda component: (len(component), tuple(sorted(component))))
                group = subset-retained
            else:
                unsupported = [component for component in components if not stable(component)]
                group = frozenset({pid}.union(*unsupported))
            if len(group) <= 1 or group == subset or group in seen or sum(part_masses[p] for p in group) > lift_limit:
                continue
            seen.add(group)
            bundles.append((len(group & backbone) if lift_limit <= 15. else 0, -len(group), tuple(sorted(group)), group))
        for *_, group in sorted(bundles):
            yield group
        for pid in singles:
            if len(subset) <= 30:
                group = frozenset([pid]); seen.add(group)
                yield group

        axes = []
        for j, a, b, axis, port in joint_data:
            if a in subset and b in subset and port and port.get('assembly') in ('slide', 'glue'):
                if not any(abs(axis @ old) > .999 for old in axes):
                    axes.append(axis)
        grouped = []
        for axis in axes:
            groups = UnionFind(subset)
            for j, a, b, other_axis, port in joint_data:
                if a not in subset or b not in subset:
                    continue
                if not port or port.get('assembly') not in ('slide', 'glue') or abs(axis @ other_axis) < .999:
                    groups.union(a, b)
            for component in groups.groups():
                group = frozenset(component)
                if len(group) == len(subset) or group in seen:
                    continue
                seen.add(group)
                crossing = sum(a in subset and b in subset and ((a in group) != (b in group))
                               for _, a, b, _, _ in joint_data)
                grouped.append((len(group & backbone) if lift_limit <= 15. else 0, -min(len(group), len(subset)-len(group)), -crossing,
                                sum(part_masses[p] for p in sorted(group)), len(group),
                                -max(assembly.parts[p].matrix[2, 3] for p in group), tuple(sorted(group)), group))
        for *_, group in sorted(grouped):
            yield group
        for pid in singles:
            group = frozenset([pid])
            if group not in seen:
                yield group

    def connected(subset):
        if len(subset) < 2:
            return True
        visited = {next(iter(subset))}
        pending = list(visited)
        while pending:
            pid = pending.pop()
            for _, a, b, _, _ in adjacency[pid]:
                other = b if a == pid else a
                if other in subset and other not in visited:
                    visited.add(other)
                    pending.append(other)
        return len(visited) == len(subset)

    def connected_groups(subset):
        groups = UnionFind(subset)
        for _, a, b, _, _ in joint_data:
            if a in subset and b in subset:
                groups.union(a, b)
        return [frozenset(group) for group in groups.groups()]

    def fasten(j):
        if joint_kind(j) == 'fixed':
            torque = j.get('torque_nm')
            action = 'Secure connection'+(f' to the specified {torque:g} N·m' if torque is not None else '; use the torque specified for the purchased hardware')
        else:
            action = 'Retain the moving connection; leave the intended slide/rotation free'
        if j.get('type') == 'socket':
            if j.get('insertion_mm', 0):
                action += f"; insert the tube {j['insertion_mm']} mm into this socket"
            elif 'at_mm' in j['b']:
                action += f"; align the socket centre {j['b']['at_mm']} mm from the tube start"
        return {'joint': j['id'], 'action': action}

    try:
        report('Finding parallel sliding groups', True)
        if fixtures and not allow_fixtures:
            return outcome('blocked', 'Temporary fixtures are declared but allow_temporary_supports is false')
        if not stable(remaining):
            return outcome('blocked', 'Final structure has unsupported components under the gravity support test')
        with CollisionWorld(assembly) as world:
            increment = 100.
            interfaces = {}
            primitive_cache, primitive_meshes = {}, {}
            for j, a, b, axis, port in joint_data:
                interfaces.setdefault(frozenset([a, b]), []).append((j, a, b, axis, port))

            def primitive_interval(shape, matrix, normal, base):
                local = matrix[:3, :3].T @ normal
                center = float(matrix[:3, 3] @ normal)
                radius = shape.get('radius_mm', shape.get('diameter_mm', 0)/2)
                kind = shape['type']
                if kind in ('box', 'extrusion'):
                    extent = float(np.abs(local) @ (np.asarray(shape['size_mm'])/2))
                elif kind in ('tube', 'cylinder'):
                    extent = abs(local[2])*shape['length_mm']/2+radius*np.linalg.norm(local[:2])
                elif kind == 'sphere':
                    extent = radius
                elif kind == 'capsule':
                    extent = abs(local[2])*shape['length_mm']/2+radius
                else:
                    key = (str(base), repr(shape))
                    if key not in primitive_meshes:
                        clean = {k: v for k, v in shape.items() if k not in ('position_mm', 'rotation_deg', 'axis')}
                        primitive_meshes[key] = shape_mesh(clean, base).vertices
                    projected = primitive_meshes[key] @ local+center
                    return float(projected.min()), float(projected.max())
                return center-extent, center+extent

            boxes = {}
            for pid, part in assembly.parts.items():
                report('Preparing collision bounds for sliding paths')
                primitives = [(shape, part.matrix @ local) for shape, local in collision_primitives(part)]
                primitive_cache[pid] = primitives
                intervals = [[primitive_interval(shape, matrix, axis, part.base) for shape, matrix in primitives]
                             for axis in np.eye(3)]
                boxes[pid] = np.array([[min(pair[0] for pair in pairs), max(pair[1] for pair in pairs)]
                                       for pairs in intervals]).T
            all_bounds = np.array(list(boxes.values())) if boxes else np.array([[[0, 0, 0], [1, 1, 1]]])
            escape = float(np.linalg.norm(all_bounds[:, 1].max(axis=0)-all_bounds[:, 0].min(axis=0)))+100

            def interface_clear(pid, other, axis):
                for j, a, b, socket_axis, port in interfaces.get(frozenset([pid, other]), []):
                    if not port:
                        continue
                    member = assembly.parts[b]
                    if member.kind != 'member' or len(member.shapes) != 1 or member.shapes[0]['type'] not in ('tube', 'cylinder'):
                        continue
                    tube = member.shapes[0]
                    tube_frame = shape_transform(tube)
                    diameter = float(tube.get('diameter_mm', 2*tube.get('radius_mm', 0)))
                    if (np.linalg.norm(tube_frame[:3, 3]) > 1e-6 or abs(tube_frame[2, 2]) < 1.-1e-10 or
                            abs(float(tube.get('length_mm', 0))-member.length) > 1e-6 or
                            abs(diameter-member.section.get('diameter_mm', 0)) > 1e-6):
                        continue
                    if port.get('assembly') == 'radial' and pid == a and abs(axis @ socket_axis) < .01:
                        connector = assembly.parts[a]
                        mouth, _ = connector.frame(j['a'])
                        half = float(port.get('engagement_mm', 0))/2
                        origin = float(mouth @ socket_axis)
                        radius = float(port.get('diameter_mm', 0))/2
                        transverse_u = unit(np.cross(socket_axis, np.eye(3)[np.argmin(np.abs(socket_axis))]))
                        transverse_v = np.cross(socket_axis, transverse_u)
                        clear = True
                        for shape, matrix in primitive_cache[a]:
                            lo, hi = primitive_interval(shape, matrix, socket_axis, connector.base)
                            if lo >= origin-half-1e-5 and hi <= origin+half+1e-5:
                                radial = matrix[:3, 3]-mouth
                                radial -= socket_axis*(radial @ socket_axis)
                                if np.linalg.norm(radial) > 1e-8:
                                    normal = unit(radial)
                                    inner, _ = primitive_interval(shape, matrix, normal, connector.base)
                                    inner -= float(mouth @ normal)
                                    radii = []
                                    for n in (transverse_u, transverse_v):
                                        first, last = primitive_interval(shape, matrix, n, connector.base)
                                        center = float(mouth @ n)
                                        radii.append(max(abs(first-center), abs(last-center)))
                                    # Opening applies to local housing pieces
                                    # next to the bore. It must never erase a
                                    # remote plate merely at the same height.
                                    if radius-.5 <= inner <= radius+float(port.get('clearance_mm', 0))+1. and np.linalg.norm(radii) <= 2*radius:
                                        continue
                            separated = False
                            # Use fitting/motion planes, not only world AABBs:
                            # a long tilted rail has a very loose world box.
                            normals = (axis, socket_axis, np.cross(axis, socket_axis), *matrix[:3, :3].T)
                            for normal in normals:
                                if np.linalg.norm(normal) < 1e-8:
                                    continue
                                normal = unit(normal)
                                lo, hi = primitive_interval(shape, matrix, normal, connector.base)
                                other_intervals = [primitive_interval(s, m, normal, member.base)
                                                   for s, m in primitive_cache[b]]
                                other_lo = min(pair[0] for pair in other_intervals)
                                other_hi = max(pair[1] for pair in other_intervals)
                                travel = float(axis @ normal)*escape
                                swept_lo, swept_hi = min(lo, lo+travel), max(hi, hi+travel)
                                if min(swept_hi-other_lo, other_hi-swept_lo) < .49:
                                    separated = True
                                    break
                            if not separated:
                                clear = False
                                break
                        if clear:
                            return True
                        continue
                    if abs(axis @ socket_axis) < .999:
                        continue
                    if abs(member.matrix[:3, 2] @ socket_axis) < .999:
                        continue
                    mouth, _ = assembly.parts[a].frame(j['a'])
                    offset = mouth-member.matrix[:3, 3]
                    if np.linalg.norm(offset-socket_axis*(offset @ socket_axis)) > 1.1:
                        continue
                    if abs(member.section.get('diameter_mm', 0)-port.get('diameter_mm', 0)) > .6:
                        continue
                    if not port.get('through') and (axis @ socket_axis)*(-1 if pid == a else 1) < .999:
                        continue
                    depth = float(port.get('engagement_mm', 0))
                    drift = (np.linalg.norm(offset-socket_axis*(offset @ socket_axis))+
                             np.linalg.norm(np.cross(member.matrix[:3, 2], socket_axis))*member.length/2+
                             np.linalg.norm(np.cross(axis, socket_axis))*(member.length+depth+escape))
                    oversize = max(0., (diameter-float(port.get('diameter_mm', 0)))/2)
                    if drift+oversize > max(0., float(port.get('clearance_mm', 0)))+1.:
                        continue
                    connector = assembly.parts[a]
                    if a not in primitive_cache:
                        primitive_cache[a] = [(shape, connector.matrix @ local) for shape, local in collision_primitives(connector)]
                    origin = float(mouth @ socket_axis)
                    low, high = ((-depth/2, depth/2) if port.get('through') else (-depth, 0.))
                    member_center = float(member.matrix[:3, 3] @ socket_axis)-origin
                    member_half = abs(member.matrix[:3, 2] @ socket_axis)*member.length/2
                    radius = member.section.get('diameter_mm', 0)/2+drift
                    clear = True
                    for shape, matrix in primitive_cache[a]:
                        first, last = primitive_interval(shape, matrix, socket_axis, connector.base)
                        first -= origin; last -= origin
                        if not port.get('through') and last <= member_center-member_half+.5:
                            continue
                        if not port.get('through'):
                            # During withdrawal the pipe only moves towards
                            # the mouth; geometry behind its initial cut end
                            # cannot become a new obstruction.
                            first = max(first, member_center-member_half-.5)
                        if first >= low-1e-5 and last <= high+1e-5:
                            continue
                        # A branch or screw outside the swept tube cylinder is
                        # harmless. A remote plate on the SAME fitting is not:
                        # never erase that obstruction with a bore exception.
                        normals = [matrix[:3, 3]-mouth, *np.eye(3)]
                        separated = False
                        for normal in normals:
                            transverse = normal-socket_axis*(normal @ socket_axis)
                            if np.linalg.norm(transverse) < 1e-8:
                                continue
                            transverse = unit(transverse)
                            lo, hi = primitive_interval(shape, matrix, transverse, connector.base)
                            center = float(mouth @ transverse)
                            if lo-center >= radius-.5 or hi-center <= -radius+.5:
                                separated = True
                                break
                        if not separated:
                            clear = False
                            break
                    if clear:
                        return True
                return False

            def pair_path(pid, other, axis):
                key = (pid, other, tuple(axis))
                if key in pair_cache:
                    return pair_cache[key]
                if interface_clear(pid, other, axis):
                    pair_cache[key] = True
                    return True
                # A persistent separating coordinate plane also proves a
                # tangential slide safe within the contact tolerance. This
                # avoids thousands of tiny advances along touching surfaces.
                swept_low = np.minimum(boxes[pid][0], boxes[pid][0]+axis*escape)
                swept_high = np.maximum(boxes[pid][1], boxes[pid][1]+axis*escape)
                if np.any(np.minimum(swept_high-boxes[other][0], boxes[other][1]-swept_low) < .49):
                    pair_cache[key] = True
                    return True
                lo, hi = 0., escape
                for i in range(3):
                    if abs(axis[i]) < 1e-10:
                        if boxes[pid][1, i] < boxes[other][0, i]-1. or boxes[other][1, i] < boxes[pid][0, i]-1.:
                            pair_cache[key] = True
                            return True
                    else:
                        t1 = (boxes[other][0, i]-boxes[pid][1, i]-1.)/axis[i]
                        t2 = (boxes[other][1, i]-boxes[pid][0, i]+1.)/axis[i]
                        lo, hi = max(lo, min(t1, t2)), min(hi, max(t1, t2))
                if lo > hi:
                    pair_cache[key] = True
                    return True
                distance, clear = max(0., lo), True
                try:
                    while distance <= hi:
                        report('Checking sliding paths for subassemblies')
                        world.move(pid, axis*distance)
                        stats['collision_queries'] += 1
                        contacts = pb.getClosestPoints(world.world.part_map[pid][0], world.world.part_map[other][0],
                                                       distance=increment/1000, physicsClientId=world.world.client)
                        separation = min((c[8]*1000 for c in contacts), default=increment)
                        if separation < -.5:
                            clear = False
                            break
                        distance += max(.05, min(increment, .5*(separation+.5)))
                finally:
                    world.move(pid, [0, 0, 0])
                pair_cache[key] = clear
                return clear

            def escape_path(group, subset, manual=False):
                others = subset-group
                for axis in directions(group, subset):
                    stats['paths_checked'] += 1
                    report(f'Checking a {len(group)}-part sliding group')
                    preparation_up = floor_plane(subset, others, axis) if manual and others else None
                    rest_after = False
                    if manual and others and preparation_up is None:
                        preparation_up = floor_plane(subset, others, axis, support_full=False)
                        rest_after = preparation_up is not None
                    if manual and others and preparation_up is None:
                        continue
                    floor = assembly.doc.get('environment', {}).get('ground_z_mm', 0)
                    if not manual and assembly.doc.get('environment', {}).get('ground', True) and any(
                            boxes[p][0, 2]+axis[2]*escape < floor-1. for p in group):
                        continue
                    hit = None
                    for pid in sorted(group):
                        for other in sorted(others):
                            if not pair_path(pid, other, axis):
                                hit = other
                                blocked.setdefault(pid, set()).add(other)
                                break
                        if hit:
                            break
                    if hit is None:
                        def projection(pid):
                            center = (boxes[pid][0]+boxes[pid][1])/2
                            half = (boxes[pid][1]-boxes[pid][0])/2
                            return float(center @ axis), float(half @ np.abs(axis))
                        distance = (max((projection(p)[0]+projection(p)[1] for p in others), default=0.)-
                                    min(projection(p)[0]-projection(p)[1] for p in group)+100.) if others else 100.
                        return axis, min(escape, max(100., distance)), preparation_up, rest_after
                return None

            def search(subset, manual=False):
                nonlocal deepest_parts
                key = (subset, manual)
                stats['remaining_parts'] = len(subset)
                if not manual and (deepest_parts is None or len(subset) < len(deepest_parts)):
                    deepest_parts = subset
                report(f'Planning {len(subset)} parts'+(' on the preparation floor' if manual else ''))
                if not subset:
                    return None
                if key in solved:
                    return solved[key]
                if key in failed:
                    return False
                if stats['states_explored'] >= limit:
                    raise _StopPlanning('Search budget exhausted; no impossibility is implied')
                stats['states_explored'] += 1
                if len(subset) == 1:
                    result = {'parts': subset, 'manual': manual, 'leaf': next(iter(subset))}
                    if manual:
                        result['floor_up'] = floor_plane(subset)
                        if result['floor_up'] is None:
                            failed.add(key)
                            return False
                    solved[key] = result
                    return result
                for group in candidates(subset, manual):
                    report(f'Considering a {len(group)}-part parallel cut')
                    after = subset-group
                    if not manual and lift_limit <= 15. and subset-backbone and group & backbone:
                        continue
                    if sum(part_masses[p] for p in group) > lift_limit:
                        continue
                    if manual and not connected(after):
                        continue
                    captured = None
                    retained = after
                    if not manual and not stable(after):
                        if len(group) != 1 or assembly.parts[next(iter(group))].kind != 'member':
                            continue
                        unsupported = [component for component in connected_groups(after) if not stable(component)]
                        if len(unsupported) != 1:
                            continue
                        captured = unsupported[0]
                        retained = after-captured
                        if (not retained or not stable(retained) or
                                sum(part_masses[p] for p in captured | group) > lift_limit):
                            continue
                    path = escape_path(group, subset, manual)
                    if path is None:
                        continue
                    capture_path = escape_path(captured, after) if captured else None
                    if captured and capture_path is None:
                        continue
                    base = search(retained, manual)
                    if base is False:
                        continue
                    capture_tree = search(captured, True) if captured else None
                    if capture_tree is False:
                        continue
                    moving = search(group, True) if len(group) > 1 else {'parts': group, 'manual': manual, 'leaf': next(iter(group))}
                    if moving is False:
                        continue
                    result = {'parts': subset, 'manual': manual, 'base': base, 'moving': moving,
                              'axis': path[0], 'distance': path[1], 'floor_up': path[2], 'rest_after': path[3]}
                    if captured:
                        result.update(capture=capture_tree, capture_axis=capture_path[0], capture_distance=capture_path[1])
                    solved[key] = result
                    stats['planned_parts'] = max(stats['planned_parts'], len(subset))
                    return result
                failed.add(key)
                return False

            sequence = options.get('sequence')
            if sequence is not None:
                if len(sequence) != len(remaining) or set(sequence) != remaining:
                    return outcome('blocked', 'Explicit sequence must contain every part exactly once')
                removed, subset = [], remaining
                for pid in reversed(sequence):
                    if part_masses[pid] > lift_limit:
                        return outcome('indeterminate', f'{pid} exceeds the explicitly selected handling mass limit')
                    after = subset-{pid}
                    if not stable(after):
                        return outcome('blocked', f'Explicit sequence is unstable before adding {pid}')
                    path = escape_path(frozenset([pid]), subset)
                    if path is None:
                        return outcome('blocked', f'{pid} has no straight insertion route at its requested step',
                                       blocked_by=sorted(blocked.get(pid, [])))
                    removed.append((pid, *path)); subset = after
                tree = None
                for pid, axis, distance, preparation_up, rest_after in reversed(removed):
                    leaf = {'parts': frozenset([pid]), 'manual': False, 'leaf': pid}
                    tree = leaf if tree is None else {'parts': tree['parts'] | {pid}, 'manual': False,
                                                     'base': tree, 'moving': leaf, 'axis': axis, 'distance': distance}
            else:
                if 'max_lift_mass_kg' not in options:
                    preferred_deadline = min(30., time_limit*.25)
                try:
                    tree = search(remaining)
                except _PreferredSearchBudget:
                    tree = False
                finally:
                    preferred_deadline = None
                if tree is False and 'max_lift_mass_kg' not in options:
                    assisted_search = True
                    lift_limit = max(lift_limit, sum(part_masses.values()))
                    failed.clear()
                    report('The preferred lighter sequence is not yet found; checking explicit team or equipment assisted lifts', True)
                    tree = search(remaining)
            if tree is False:
                return outcome('indeterminate', 'No verified sliding decomposition found within the candidate cuts; no impossibility is implied',
                               blocked_by={p: sorted(v) for p, v in blocked.items()})
    except _StopPlanning as exc:
        return outcome('cancelled' if exc.cancelled else 'indeterminate', exc.reason,
                       blocked_by={p: sorted(v) for p, v in blocked.items()},
                       unresolved_parts=sorted(deepest_parts or []))

    steps, subassemblies, placed, areas = [], [], {}, {}
    original_inverses = {p: np.linalg.inv(part.matrix) for p, part in assembly.parts.items()}
    transfer_bounds_cache = {}
    workspace_bounds = np.array([[np.inf]*3, [-np.inf]*3])
    context_parts = {'main': remaining}
    area_bounds, area_radii = {}, {}
    floor_z = float(assembly.doc.get('environment', {}).get('ground_z_mm', 0))
    if not assembly.doc.get('environment', {}).get('ground', True):
        floor_z = min(floor_z, float(all_bounds[:, 0, 2].min())-escape-100.)

    def pose_map(ids, replacements=None):
        replacements = replacements or {}
        return {pid: pose_of(replacements.get(pid, placed[pid])) for pid in sorted(ids)}

    def matrices_for(ids, delta):
        return {pid: delta @ assembly.parts[pid].matrix for pid in ids}

    def physical_bounds(pid, matrix):
        key = (pid, matrix.tobytes())
        if key in transfer_bounds_cache:
            return transfer_bounds_cache[key]
        support_cloud(frozenset([pid]))
        delta = matrix @ original_inverses[pid]
        vertices = part_hulls[pid] @ delta[:3, :3].T+delta[:3, 3]
        intervals = [[primitive_interval(shape, delta @ original, axis, assembly.parts[pid].base)
                      for shape, original in primitive_cache[pid]] for axis in np.eye(3)]
        collision = np.array([[min(pair[0] for pair in pairs), max(pair[1] for pair in pairs)]
                              for pairs in intervals]).T
        result = np.array([np.minimum(vertices.min(axis=0), collision[0]),
                           np.maximum(vertices.max(axis=0), collision[1])])
        transfer_bounds_cache[key] = result
        return result

    def transfer_projection(pid, matrix, directions_, points):
        delta = matrix @ original_inverses[pid]
        projected = points @ directions_.T
        low, high = projected.min(axis=0), projected.max(axis=0)
        for index, normal in enumerate(directions_):
            for shape, original in primitive_cache[pid]:
                first, last = primitive_interval(shape, delta @ original, normal, assembly.parts[pid].base)
                low[index], high[index] = min(low[index], first), max(high[index], last)
        return low, high

    def record_workspace(matrices):
        for pid, matrix in matrices.items():
            bounds = physical_bounds(pid, matrix)
            workspace_bounds[0] = np.minimum(workspace_bounds[0], bounds[0])
            workspace_bounds[1] = np.maximum(workspace_bounds[1], bounds[1])

    def preparation_transform(subset, up, context, staging=False):
        if context not in areas:
            cloud = support_cloud(context_parts.get(context, subset))[0]
            radius = float(np.linalg.norm(cloud.max(axis=0)-cloud.min(axis=0)))/2
            area_radii[context] = radius
            extent = max(300., 3*radius+300.)
            obstacles = [np.array([all_bounds[:, 0, :2].min(axis=0), all_bounds[:, 1, :2].max(axis=0)])]
            obstacles.extend(area_bounds.values())
            candidates = sorted((np.array([x, y], dtype=float)*500. for x in range(-40, 41) for y in range(-40, 41)),
                                key=lambda p: (float(p @ p), float(p[0]), float(p[1])))
            for center in candidates:
                bounds = np.array([center-extent, center+extent])
                if all(np.any(bounds[1] < other[0]-100.) or np.any(other[1] < bounds[0]-100.) for other in obstacles):
                    areas[context], area_bounds[context] = center, bounds
                    break
            else:
                raise _StopPlanning('The available clear preparation areas need a different layout; no impossibility is implied')
        center = areas[context]+([2*area_radii[context]+100., 0.] if staging else np.zeros(2))
        rotation = align_axis(up, [0., 0., 1.])
        vertices = support_cloud(subset)[0] @ rotation.T
        delta = np.eye(4)
        delta[:3, :3] = rotation
        delta[:2, 3] = center-(vertices.min(axis=0)+vertices.max(axis=0))[:2]/2
        delta[2, 3] = floor_z-float(vertices[:, 2].min())
        return delta

    def lift_group(moving, target, destination, context, prepared=None, grounded=False):
        """Lift vertically, turn above all geometry, then lower in clear space.

        A swept bounding box proves the two vertical legs; a bounding sphere
        above every stationary part proves the turn and overhead translation.
        Only this already connected group is held during the operation.
        """
        report('Checking floor-to-insertion lifting motions')
        visible = moving | target
        pid = sorted(moving)[0]
        if all(np.allclose(placed[p], destination[p], atol=1e-7) for p in moving):
            return
        original_inverse = np.linalg.inv(assembly.parts[pid].matrix)
        first = placed[pid] @ original_inverse
        last = destination[pid] @ original_inverse
        cloud = support_cloud(moving)[0]
        center = (cloud.min(axis=0)+cloud.max(axis=0))/2
        collision_corners = np.array([[box[x, 0], box[y, 1], box[z, 2]] for p in moving for box in [boxes[p]]
                                      for x in (0, 1) for y in (0, 1) for z in (0, 1)])
        radius = float(np.max(np.linalg.norm(np.vstack((cloud, collision_corners))-center, axis=1)))+1.
        start_center, end_center = point(first, center), point(last, center)
        obstacles = {p: physical_bounds(p, matrix) for p, matrix in placed.items() if p not in moving}
        obstacle_points = {}
        obstruction = None
        high = max([floor_z, start_center[2], end_center[2]]+[box[1, 2] for box in obstacles.values()])+radius+100.
        waypoints = []
        for rotation, location in ((first[:3, :3], [*start_center[:2], high]),
                                   (last[:3, :3], [*start_center[:2], high]),
                                   (last[:3, :3], [*end_center[:2], high])):
            delta = np.eye(4); delta[:3, :3] = rotation
            delta[:3, 3] = np.asarray(location)-rotation @ center
            waypoints.append(matrices_for(moving, delta))
        def translation_clear(start, finish):
            nonlocal obstruction
            for p in moving:
                report('Checking clear lifting corridors')
                first_box, last_box = physical_bounds(p, start[p]), physical_bounds(p, finish[p])
                sweep = np.array([np.minimum(first_box[0], last_box[0]), np.maximum(first_box[1], last_box[1])])
                if sweep[0, 2] < floor_z-1.:
                    return False
                for other, obstacle in obstacles.items():
                    if np.all(np.minimum(sweep[1]-obstacle[0], obstacle[1]-sweep[0]) > .49):
                        # Axis-aligned boxes overlap for many perfectly clear
                        # oblique routes. A separating projection of the actual
                        # convex hulls proves the complete translated sweep.
                        delta = start[p] @ np.linalg.inv(assembly.parts[p].matrix)
                        points = part_hulls[p] @ delta[:3, :3].T+delta[:3, 3]
                        travel = finish[p][:3, 3]-start[p][:3, 3]
                        if other not in obstacle_points:
                            other_delta = placed[other] @ np.linalg.inv(assembly.parts[other].matrix)
                            other_cloud, _, normals = support_cloud(frozenset([other]))
                            obstacle_points[other] = (other_cloud @ other_delta[:3, :3].T+other_delta[:3, 3],
                                                      normals @ other_delta[:3, :3].T)
                        other_cloud, other_normals = obstacle_points[other]
                        normals = support_cloud(frozenset([p]))[2] @ delta[:3, :3].T
                        directions_ = np.vstack((normals, other_normals, np.cross(np.eye(3), travel)))
                        directions_ = directions_[np.linalg.norm(directions_, axis=1) > 1e-8]
                        directions_ /= np.linalg.norm(directions_, axis=1)[:, None]
                        _, indices = np.unique(np.round(directions_, 7), axis=0, return_index=True)
                        directions_ = directions_[indices]
                        first_low, first_high = transfer_projection(p, start[p], directions_, points)
                        other_low, other_high = transfer_projection(other, placed[other], directions_, other_cloud)
                        displacement = travel @ directions_.T
                        low = first_low+np.minimum(displacement, 0.)
                        high_ = first_high+np.maximum(displacement, 0.)
                        if np.any(high_ <= other_low+.49) or np.any(other_high <= low+.49):
                            continue
                        obstruction = {'moving_part': p, 'obstacle': other, 'start_pose': pose_of(start[p]),
                                       'end_pose': pose_of(finish[p]), 'obstacle_pose': pose_of(placed[other])}
                        return False
            return True
        if not translation_clear({p: placed[p] for p in moving}, waypoints[0]):
            raise _StopPlanning('A preparation lift needs a clearer pickup route; no impossibility is implied')
        if not translation_clear(waypoints[-1], destination):
            # A geometry-only design can have an insertion start underneath
            # its stationary frame. Descend outside the frame, then approach
            # horizontally through a proved clear corridor.
            bounds = list(obstacles.values())
            low = np.min([box[0] for box in bounds], axis=0) if bounds else end_center
            high_bounds = np.max([box[1] for box in bounds], axis=0) if bounds else end_center
            found = False
            for coordinate in (0, 1):
                for value in (low[coordinate]-radius-100., high_bounds[coordinate]+radius+100.):
                    corridor = end_center.copy(); corridor[coordinate] = value
                    raised = corridor.copy(); raised[2] = high
                    extra = []
                    for location in (raised, corridor):
                        delta = last.copy(); delta[:3, 3] = location-last[:3, :3] @ center
                        extra.append(matrices_for(moving, delta))
                    if translation_clear(extra[0], extra[1]) and translation_clear(extra[1], destination):
                        waypoints = waypoints[:2]+extra
                        found = True
                        break
                if found:
                    break
            if not found:
                raise _StopPlanning('A preparation lift needs a clearer transfer route; no impossibility is implied',
                                    details={'next_step': len(steps)+1, 'moving_parts': sorted(moving),
                                             'context': context, 'prepared_subassembly': prepared,
                                             'obstruction': obstruction, 'destination': {p: pose_of(m) for p, m in destination.items()}})
        start_poses = pose_map(visible)
        waypoint_poses = [pose_map(visible, item) for item in waypoints]
        for item in [destination, *waypoints]:
            record_workspace(item)
        # A bounding sphere also covers every intermediate rotation, not only
        # the sampled waypoint poses used by illustrations.
        turn_centers = np.array([[*start_center[:2], high], [*end_center[:2], high]])
        workspace_bounds[0] = np.minimum(workspace_bounds[0], turn_centers.min(axis=0)-radius)
        workspace_bounds[1] = np.maximum(workspace_bounds[1], turn_centers.max(axis=0)+radius)
        placed.update(destination)
        label = prepared or ('this connected subassembly' if len(moving) > 1 else pid)
        group_mass = sum(part_masses[p] for p in moving)
        instruction = (f'Lift {label} as one connected group, turn it in the clear overhead space, '
                       +('and lower it onto the floor in the shown orientation.' if grounded else
                         'and bring it to the shown insertion start. Hold only this group for the next joining step.'))
        if group_mass > 15.:
            instruction = f'Arrange a team or suitable lifting equipment for this {group_mass:.3f} kg connected group. '+instruction
        steps.append({'number': len(steps)+1, 'kind': 'lift', 'part': pid, 'parts': [],
                      'introduced_parts': [], 'moving_parts': sorted(moving), 'target_parts': sorted(target),
                      'installed_parts': sorted(visible), 'subassembly': context, 'prepared_subassembly': prepared,
                      'instruction': instruction, 'pose': pose_of(placed[pid]), 'poses': pose_map(visible),
                      'start_poses': start_poses, 'motion_waypoints': waypoint_poses,
                      'manual_support': not grounded, 'held_parts': [] if grounded else sorted(moving),
                      'assistance_required': group_mass > 15.,
                      'grounded_preparation': context != 'main', 'grounded_after': grounded,
                      'floor_z_mm': floor_z, 'group_mass_kg': group_mass,
                      'approach_direction': [0., 0., 1.], 'start_offset_mm': [0., 0., 0.],
                      'fasten': [], 'world_anchors': [], 'world_anchor': None, 'fixture': None})

    def append_step(pid, introduced, target, visible, axis, distance, connections, manual, context, moving_parts=None, prepared=None,
                    delta=None, instruction=None, staged=False):
        p = assembly.parts[pid]
        moving_parts = moving_parts or [pid]
        delta = np.eye(4) if delta is None else delta
        placed.update(matrices_for(moving_parts, delta))
        record_workspace({p: placed[p] for p in moving_parts})
        kind = 'position' if not connections else ('merge' if prepared else 'insert')
        if not connections:
            if prepared:
                action = f'Position {prepared} ({len(moving_parts)} parts) at the shown alignment.'
            elif p.kind == 'member':
                action = f'Position {pid}, cut to {p.length} mm, at the shown alignment.'
            else:
                action = f'Position {p.spec.get("label", p.definition.get("name", pid))} ({pid}) at the shown alignment.'
            if target:
                action += ' Support it separately until a later step connects it.'
        elif prepared:
            action = f'Support {prepared} and slide all {len(moving_parts)} parts together onto {context} along the shown direction.'
        elif p.kind == 'member':
            action = f'Insert {pid}, cut to {p.length} mm, along the shown direction.' if target else f'Position {pid}, cut to {p.length} mm.'
        elif any(port.get('through') and port.get('assembly') == 'slide' for port in p.ports.values()):
            action = f'Slide {pid} onto the accessible tube end before fitting the next stop.' if target else f'Position {pid} for this subassembly.'
        elif any(port.get('assembly') == 'radial' for port in p.ports.values()):
            action = f'Open the split clamp {pid}, fit it around the tube, and close its halves.'
        else:
            action = f'Position {p.spec.get("label", p.definition.get("name", pid))} ({pid}).'
        if manual:
            action = 'Prepare this subassembly on the floor in the shown orientation. '+action
        if instruction:
            action = instruction
        group_mass = sum(part_masses[p] for p in moving_parts)
        if group_mass > 15.:
            action = f'Arrange a team or suitable lifting equipment for this {group_mass:.3f} kg group. '+action
        mounts = [a for a in assembly.anchors if context == 'main' and a['part'] in moving_parts]
        steps.append({'number': len(steps)+1, 'kind': kind, 'part': pid, 'parts': introduced,
                      'introduced_parts': introduced, 'moving_parts': moving_parts, 'target_parts': target,
                      'subassembly': context, 'prepared_subassembly': prepared, 'manual_support': False,
                      'instruction': action, 'pose': pose_of(placed[pid]), 'poses': pose_map(visible),
                      'approach_direction': (-delta[:3, :3] @ axis).tolist(),
                      'start_offset_mm': (delta[:3, :3] @ (axis*distance)).tolist(),
                      'design_approach_direction': (-axis).tolist(), 'design_start_offset_mm': (axis*distance).tolist(),
                      'floor_z_mm': floor_z, 'grounded_preparation': manual, 'grounded_after': manual,
                      'staged_parts': list(moving_parts) if staged else [],
                      'group_mass_kg': group_mass, 'assistance_required': group_mass > 15.,
                      'fasten': [fasten(j) for j in connections],
                      'installed_parts': visible, 'world_anchors': mounts, 'world_anchor': next(iter(mounts), None),
                      'fixture': next((f for f in options.get('fixtures', []) if f['part'] == pid), None)})

    def emit(node, context='main'):
        if node is None:
            return
        report('Placing connected subassemblies on the floor')
        manual = node['manual']
        if 'leaf' in node:
            pid = node['leaf']
            delta = preparation_transform(node['parts'], floor_plane(node['parts']), context) if manual else np.eye(4)
            append_step(pid, [pid], [], [pid], np.array([0., 0., -1.]), 0., [], manual, context, delta=delta)
            return
        emit(node['base'], context)
        moving, base = node['moving']['parts'], node['base']['parts']
        capture_name, captured = None, frozenset()
        if node.get('capture'):
            captured = node['capture']['parts']
            capture_name = f'subassembly-{len(subassemblies)+1}'
            subassemblies.append({'id': capture_name, 'parts': sorted(captured)})
            context_parts[capture_name] = captured
            emit(node['capture'], capture_name)
            offset = np.eye(4)
            offset[:3, 3] = node['capture_axis']*node['capture_distance']
            lift_group(captured, base, matrices_for(captured, offset), context, prepared=capture_name)
            combined_mass = sum(part_masses[p] for p in captured | moving)
            representative = sorted(captured)[0]
            append_step(representative, [], sorted(base), sorted(base | captured), node['capture_axis'], node['capture_distance'], [],
                        False, context, moving_parts=sorted(captured), prepared=capture_name,
                        instruction=f'With a helper supporting the one connected {capture_name}, slide it to the shown alignment. '
                                    f'The helper must keep supporting this group while the next step inserts its joining pipe; '
                                    f'the combined group and pipe mass is {combined_mass:.3f} kg.')
            steps[-1].update(capture_pending=True, held_parts=sorted(captured), manual_support=True,
                             helper_required=True, assistance_required=True, combined_handling_mass_kg=combined_mass,
                             joining_pipe=sorted(moving)[0])
            base = base | captured
        connections = [j for j, a, b, _, _ in joint_data if (a in moving and b in base) or (b in moving and a in base)]
        pid = sorted(moving)[0]
        delta = preparation_transform(node['parts'], node['floor_up'], context) if manual else np.eye(4)
        rest_after = node.get('rest_after', False)
        if manual:
            # Keep a suitable existing floor orientation. Reorienting the
            # growing frame for every new small fitting adds needless lifts.
            reference = sorted(base)[0]
            current = placed[reference] @ np.linalg.inv(assembly.parts[reference].matrix)
            up, level = current[2, :3], floor_z-current[2, 3]
            cloud, center, _ = support_cloud(node['parts'])
            if (up @ node['axis'] >= -1e-8 and float(np.min(cloud @ up)) >= level-1. and
                    supported_on_plane(cloud, center, up, level)):
                delta, rest_after = current, False
            lift_group(base, frozenset(), matrices_for(base, delta), context, grounded=True)
        prepared = None
        introduced = [pid]
        singleton_start = None
        if len(moving) > 1:
            prepared = f'subassembly-{len(subassemblies)+1}'
            subassemblies.append({'id': prepared, 'parts': sorted(moving)})
            context_parts[prepared] = moving
            emit(node['moving'], prepared)
            introduced = []
        elif manual:
            singleton_start = len(steps)
            rest = preparation_transform(moving, floor_plane(moving), context, staging=True)
            append_step(pid, [pid], [], sorted(base | moving), np.array([0., 0., -1.]), 0., [], True, context,
                        delta=rest, staged=True, instruction=f'Place {pid} on the floor beside the prepared base, in the shown resting orientation.')
            introduced = []
        if prepared or manual:
            start = delta.copy()
            start[:3, 3] += delta[:3, :3] @ (node['axis']*node['distance'])
            lift_group(moving, base, matrices_for(moving, start), context, prepared=prepared)
        append_step(pid, introduced, sorted(base), sorted(node['parts']), node['axis'], node['distance'], connections, manual, context,
                    moving_parts=sorted(moving), prepared=prepared, delta=delta)
        if captured:
            steps[-1]['instruction'] = (f'Keep the helper supporting {capture_name}. '+steps[-1]['instruction']+
                                       ' Secure every listed connection before the helper releases the subassembly.')
            steps[-1].update(capture_complete=True, held_parts=sorted(captured), helper_required=True,
                             assistance_required=True, combined_handling_mass_kg=sum(part_masses[p] for p in captured | moving))
            areas.pop(capture_name, None)
            area_bounds.pop(capture_name, None)
            area_radii.pop(capture_name, None)
        if singleton_start is not None:
            staged, joining = steps[singleton_start], steps[-1]
            lifts = [step for step in steps[singleton_start+1:-1] if step['kind'] == 'lift']
            joining['parts'] = joining['introduced_parts'] = [pid]
            joining['pickup_before_insert'] = True
            joining['start_poses'] = staged['poses']
            joining['motion_waypoints'] = [pose for lift in lifts for pose in [*lift['motion_waypoints'], lift['poses']]]
            joining['instruction'] = f'Pick up {pid} from the shown floor position and bring it along the clear lifting route. '+joining['instruction']
            del steps[singleton_start:-1]
            joining['number'] = len(steps)
        if prepared:
            areas.pop(prepared, None)
            area_bounds.pop(prepared, None)
            area_radii.pop(prepared, None)
        if rest_after:
            steps[-1].update(needs_grounding=True, manual_support=True, held_parts=sorted(node['parts']),
                             helper_required=True, assistance_required=True, grounded_after=False,
                             combined_handling_mass_kg=sum(part_masses[p] for p in node['parts']))
            steps[-1]['instruction'] += (' Have the helper steady the now connected group while fastening; '
                                        'keep supporting it until the next step places it on its stable floor face.')
            resting_up = floor_plane(node['parts'])
            if resting_up is None:
                raise _StopPlanning('A connected group needs a different resting face; no impossibility is implied')
            resting = preparation_transform(node['parts'], resting_up, context)
            lift_group(node['parts'], frozenset(), matrices_for(node['parts'], resting), context, grounded=True)

    try:
        emit(tree)
    except _StopPlanning as exc:
        return outcome('cancelled' if exc.cancelled else 'indeterminate', exc.reason,
                       **({'motion_diagnostic': exc.details} if exc.details else {}))
    stats['planned_parts'] = len(remaining)
    stats['remaining_parts'] = 0
    return {'status': 'buildable', 'input_sha256': input_hash, **stats,
            'elapsed_s': round(time.monotonic()-started, 3), 'steps': steps, 'subassemblies': subassemblies,
            'certificate': {'method': 'hierarchical parallel socket cuts, collision checked straight translations and gravity support polygons',
                            'algorithm_version': PLANNING_ALGORITHM_VERSION, 'maximum_sweep_step_mm': increment, 'collision_tolerance_mm': 1.,
                            'stability_checked': require_stability, 'temporary_fixtures': options.get('fixtures', []),
                            'manual_subassembly_support': False, 'grounded_preparation': True,
                            'max_lift_mass_kg': lift_limit,
                            'preferred_lift_mass_kg': 15.,
                            'lift_limit_is_explicit': 'max_lift_mass_kg' in options,
                            'fallback_used': assisted_search,
                            'maximum_handled_mass_kg': max((max(step.get('group_mass_kg', 0), step.get('combined_handling_mass_kg', 0))
                                                            for step in steps), default=0.),
                            'required_workspace_bounds_mm': workspace_bounds.tolist() if steps else None,
                            'floor_work_area_mm': (workspace_bounds[1, :2]-workspace_bounds[0, :2]).tolist() if steps else [0., 0.],
                            'maximum_overhead_height_mm': float(workspace_bounds[1, 2]-floor_z) if steps else 0.,
                            'handling': 'One connected group is held during each explicit lift; retained bases and completed preparation groups rest on verified floor support faces',
                            'assisted_capture_joins': [step['number'] for step in steps if step.get('capture_complete')],
                            'assisted_resting_joins': [step['number'] for step in steps if step.get('needs_grounding')],
                            'assisted_lifts_required': [step['number'] for step in steps if step.get('assistance_required') and step.get('group_mass_kg', 0) > 15.],
                            'lift_clearance': 'Swept vertical bounds and overhead bounding-sphere clearance in separate clear floor preparation areas',
                            'socket_bores': 'Declared axial socket bores override filled convex mesh proxies',
                            'radial_clamps': 'Declared split sockets open their local bore housing; other fitting geometry remains in the swept checks'},
            'limitations': ['Clearance uses supplied collision geometry and declared socket bores',
                            'The shown clear floor preparation areas and overhead lifting space must be available',
                            'No tool access, fastener thread motion, human handling capacity or manufacturing tolerance stack-up',
                            'No claim of general assembly impossibility when the search is indeterminate']}
