"""Fit a new socket by solving the connected hinges and slides together.

Editor snap increments describe the user's starting pose, not the mechanism's
allowed angles. Forward kinematics keeps tree joints exact; loop closures, world
anchors and the new socket are solved continuously and checked independently.
"""
import copy
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .document import DocumentError
from .math3d import point, pose_of, transform
from .posing import Mechanism
from .snapping import _movement_coordinates
from .connection_jobs import check_cancelled


def _connection_variables(mechanism, targets):
    """Keep the paths between the connection and world fixings, plus their loops.

    A closed payload attached at one articulation point travels rigidly with that
    point. Its bones/links cannot help close a socket elsewhere in the structure.
    Include cycles only when they share an edge with an already-required path.
    """
    parents={child:(parent,joint['id']) for parent,child,joint,_ in mechanism.tree}
    def path(group):
        result=set()
        while group in parents:
            group,jid=parents[group];result.add(jid)
        return result
    required=set().union(*(path(group) for group in
        mechanism.anchored|{mechanism.group_of[pid] for pid in targets if pid in mechanism.parts}))
    cycles=[(j['id'],path(mechanism.group_of[j['a']['part']]) ^
             path(mechanism.group_of[j['b']['part']])) for j in mechanism.closures]
    while True:
        previous=len(required)
        for jid,edges in cycles:
            if edges&required: required.update(edges);required.add(jid)
        if len(required)==previous: break
    return [index for jid,entries in mechanism.variables.items() if jid in required
            for _,index,_,_,_ in entries if index is not None]


def fit_connection(assembly, joint, placed, force=False, *, tolerance_mm=2., deadline=None, check_collisions=True, cancelled=None):
    check_cancelled(cancelled)
    # Interactive callers supply a real cancellation token. They finish the
    # finite seed/contact/evaluation matrix instead of failing on elapsed time.
    # Keep the legacy guard for callers with no way to cancel running work.
    deadline=(float('inf') if cancelled else time.monotonic()+(20 if force else 4)) if deadline is None else deadline
    aid, bid = joint['a']['part'], joint['b']['part']
    mechanisms = [Mechanism(assembly, aid)]
    if bid not in mechanisms[0].parts:
        mechanisms.append(Mechanism(assembly, bid))
    reference = max(range(len(mechanisms)), key=lambda i: (
        bool(mechanisms[i].anchored), len(mechanisms[i].groups[mechanisms[i].root]),
        sum(assembly.parts[p].length for p in mechanisms[i].parts)))
    lower, upper, layouts = [], [], []
    for i, mechanism in enumerate(mechanisms):
        start = len(lower)
        lower.extend(mechanism.lower); upper.extend(mechanism.upper)
        coordinates = slice(start, len(lower))
        root = None
        if len(mechanisms) > 1 and i != reference and not mechanism.anchored:
            root = slice(len(lower), len(lower)+6)
            lower.extend([-np.inf]*3+[-np.pi]*3)
            upper.extend([np.inf]*3+[np.pi]*3)
        layouts.append((mechanism, coordinates, root))
    socket = assembly.parts[aid].ports[joint['a']['port']]
    adjustable = None
    initial = [0.]*len(lower)
    if force:
        # A nominal station/depth is useful intent. A wider search can adjust it
        # within the real bore engagement, and reports the fitted value explicitly.
        adjustable = len(lower)
        if socket.get('through'):
            half = socket.get('engagement_mm', 0)/2
            lower.append(half/100); upper.append((assembly.parts[bid].length-half)/100)
            initial.append(joint['b']['at_mm']/100)
        else:
            lower.append(socket.get('min_engagement_mm', 0)/100)
            upper.append(socket['engagement_mm']/100)
            initial.append(joint['insertion_mm']/100)
    lower, upper, initial = np.array(lower), np.array(upper), np.array(initial)
    active=[];rotational=set()
    for mechanism,coordinates,root in layouts:
        active.extend(coordinates.start+index for index in _connection_variables(mechanism,{aid,bid}))
        rotational.update(coordinates.start+index for entries in mechanism.variables.values()
                          for name,index,_,_,_ in entries if index is not None and name!='slide_mm')
        if root is not None:
            active.extend(range(root.start,root.stop))
            rotational.update(range(root.start+3,root.stop))
    if adjustable is not None: active.append(adjustable)
    variable = np.array([i for i in active if upper[i]-lower[i]>1e-12],dtype=int)
    if not len(variable):
        raise DocumentError('No movable hinges or slides can adjust these parts. Loosen a connection to give the frame movement.')
    initial = np.clip(initial, lower, upper)
    moving = set().union(*(m.parts for m in mechanisms))
    ordered_moving = sorted(moving)
    matrices = {pid: p.matrix for pid, p in assembly.parts.items()}
    intent = [pid for pid in ordered_moving if not np.allclose(placed.parts[pid].matrix,matrices[pid],atol=1e-7,rtol=0)]
    a_local, a_axis = assembly.parts[aid].local_frame(joint['a'])
    b_local, b_axis = assembly.parts[bid].local_frame(joint['b'])
    direction = -1 if socket.get('through') and (matrices[aid][:3, :3]@a_axis)@(matrices[bid][:3, :3]@b_axis) < 0 else 1
    contacts = []

    def forward(q):
        deltas = {}; local = []
        for m, coordinates, root_slice in layouts:
            root = None
            if root_slice is not None:
                values = q[root_slice]; pivot = assembly.parts[m.groups[m.root][0]].matrix[:3, 3]
                root = np.eye(4); root[:3, :3] = Rotation.from_rotvec(values[3:]).as_matrix()
                root[:3, 3] = pivot-root[:3, :3]@pivot+values[:3]*100
            delta = m.forward(q[coordinates], root)
            local.append((m, q[coordinates], delta))
            deltas.update({pid: delta[m.group_of[pid]] for pid in m.parts})
        actual = {pid: deltas[pid]@matrices[pid] for pid in ordered_moving}
        return actual, local

    def difference(a, b):
        return np.r_[a[:3, 3]-b[:3, 3], 100*Rotation.from_matrix(a[:3, :3]@b[:3, :3].T).as_rotvec()]

    def socket_error(q, actual):
        a, b = actual[aid], actual[bid]
        mouth = point(a, a_local); axis = a[:3, :3]@a_axis
        local = b_local.copy(); insertion = joint['insertion_mm']
        if adjustable is not None:
            if socket.get('through'): local[2] = q[adjustable]*100-assembly.parts[bid].length/2
            else: insertion = q[adjustable]*100
        gap = point(b, local)-(mouth-axis*insertion)
        alignment = axis-direction*(b[:3, :3]@b_axis)
        return gap, alignment

    def residual(q):
        check_cancelled(cancelled)
        if time.monotonic()>deadline: raise TimeoutError('Fit search time limit')
        actual, local = forward(q)
        gap, alignment = socket_error(q, actual)
        # A nearly aligned existing frame may not be mathematically square.
        # Do not collapse parallel reinforcement arms to remove harmless angular
        # error. Keep an already-valid socket angle within the same allowance
        # used by the independent validator, while prioritizing socket closure.
        angular = alignment*max(0., 1-angular_allowance/max(np.linalg.norm(alignment),1e-15))
        errors = [10*gap, 1000*angular, .001*(q-initial)]
        if angular_allowance:
            errors.append(.01*np.concatenate([actual[pid][:3,3]-placed.parts[pid].matrix[:3,3] for pid in ordered_moving]))
        for m, values, delta in local:
            for closure in m.closures:
                a, b = closure['a']['part'], closure['b']['part']
                expected = delta[m.group_of[a]]@m.delta(closure, values)@matrices[b]
                errors.append(10*difference(expected, actual[b]))
            for gid in m.anchored-{m.root}:
                pid = m.groups[gid][0]
                errors.append(10*difference(actual[pid], matrices[pid]))
        for pid in intent:
            errors.append(.0001*difference(actual[pid], placed.parts[pid].matrix))
        for a, b, pa, pb, normal in contacts:
            aa, bb = actual.get(a, matrices[a]), actual.get(b, matrices[b])
            gap = normal@(point(aa, pa)-point(bb, pb))
            errors.append(np.array([10*max(0., .25-gap)]))
        return np.concatenate(errors)

    def full(values, indices):
        q = initial.copy(); q[indices] = values
        return q

    def preferred_coordinates(q):
        # Round mechanism coordinates, then independently check the complete
        # pose. Rounding saved XYZ poses independently would shear rigid groups
        # or break loop closures. Invalid lattice candidates simply fall back.
        seen=[]
        for angle_step in (15.,1.,None):
            candidate=q.copy()
            if angle_step is not None:
                for index in variable:
                    if index in rotational:
                        candidate[index]=np.deg2rad(np.round(np.rad2deg(q[index])/angle_step)*angle_step)
                    elif index!=adjustable or abs(initial[index]*100-round(initial[index]*100))<1e-6:
                        candidate[index]=np.round(q[index]*100)/100
            if np.any(candidate<lower-1e-10) or np.any(candidate>upper+1e-10): continue
            if any(np.array_equal(candidate,old) for old in seen): continue
            seen.append(candidate)
            yield candidate

    from .validation import SOCKET_AXIS_MIN_DOT
    allowed_alignment = np.sqrt(2*(1-SOCKET_AXIS_MIN_DOT))
    _, initial_alignment = socket_error(initial, forward(initial)[0])
    angular_allowance = allowed_alignment*.99 if np.linalg.norm(initial_alignment)<allowed_alignment else 0.
    def fitted_at(q):
        result = copy.deepcopy(joint)
        if adjustable is not None:
            if socket.get('through'): result['b']['at_mm'] = float(q[adjustable]*100)
            else: result['insertion_mm'] = float(q[adjustable]*100)
        return result

    # A detached through fitting can already surround its pipe. The station in
    # the dialog may be a rounded midpoint, while the physical bore is a little
    # farther along. Use that actual station before changing any joint pose.
    unchanged_intent=all(
        np.allclose(placed.parts[pid].matrix[:3, 3], matrices[pid][:3, 3], atol=.002, rtol=0) and
        np.allclose(placed.parts[pid].matrix[:3, :3], matrices[pid][:3, :3], atol=2e-6, rtol=0)
        for pid in moving)
    if force and socket.get('through') and unchanged_intent:
        mouth = point(matrices[aid], a_local)
        pipe_axis = matrices[bid][:3, :3]@b_axis
        station = (mouth-matrices[bid][:3, 3])@pipe_axis + assembly.parts[bid].length/2
        q = initial.copy()
        q[adjustable] = np.clip(station/100, lower[adjustable], upper[adjustable])
        actual, _ = forward(q)
        gap, alignment = socket_error(q, actual)
        if np.linalg.norm(gap) <= tolerance_mm+1e-7 and np.linalg.norm(alignment) <= max(1e-4, angular_allowance+1e-5):
            candidate = copy.copy(assembly)
            candidate.joints = [*assembly.joints, fitted_at(q)]
            from .snapping import connection_collisions
            if not check_collisions or not connection_collisions(assembly, candidate, {aid, bid}, {aid, bid}, cancelled=cancelled):
                return {}, fitted_at(q), [], sorted({aid, bid}), 0.
    rng = np.random.default_rng(48291)
    best = None; best_coordinates = None; failure = None; attempts = 0
    seeds = [initial.copy()]
    for n in range(11 if force else 1):
        seed = initial+rng.normal(0, .3 if n < 3 else 1., len(initial))
        if adjustable is not None: seed[adjustable] = initial[adjustable]
        seeds.append(np.clip(seed, lower, upper))
    translations=np.array([i for i in variable if i not in rotational],dtype=int)
    attempts_to_solve=([(initial.copy(),translations)] if len(translations) and len(translations)<len(variable) else [])
    attempts_to_solve.extend((seed,variable) for seed in seeds)
    for seed,indices in attempts_to_solve:
        check_cancelled(cancelled)
        contacts.clear()
        for contact_pass in range(5 if force else 2):
            if attempts and time.monotonic() > deadline: break
            attempts += 1
            try:
                solved = least_squares(lambda values: residual(full(values,indices)), seed[indices],
                                       bounds=(lower[indices], upper[indices]), max_nfev=180 if force else 80,
                                       ftol=1e-10, xtol=1e-10, gtol=1e-9)
            except TimeoutError:
                if not failure: failure='The search time budget expired before a fit could be verified.'
                break
            q = full(solved.x,indices); actual, _ = forward(q)
            gap, alignment = socket_error(q, actual)
            metric = float(np.linalg.norm(gap)+100*np.linalg.norm(alignment))
            if best is None or metric < best[0]:
                best = (metric, float(np.linalg.norm(gap)), float(np.rad2deg(2*np.arcsin(np.clip(np.linalg.norm(alignment)/2, 0, 1)))))
                best_coordinates=q.copy()
            rejected_contacts=[]
            for candidate_q in preferred_coordinates(q):
                actual, _ = forward(candidate_q)
                gap, alignment = socket_error(candidate_q, actual)
                if np.linalg.norm(gap) > tolerance_mm+1e-7 or np.linalg.norm(alignment) > max(1e-4,angular_allowance+1e-5):
                    continue
                posed = copy.copy(assembly); posed.parts = {pid: copy.copy(p) for pid, p in assembly.parts.items()}
                for pid, matrix in actual.items(): posed.parts[pid].matrix = matrix
                fitted_joint = fitted_at(candidate_q)
                posed.joints = [*assembly.joints, fitted_joint]
                try:
                    coordinates = _movement_coordinates(assembly, posed)
                except DocumentError as exc:
                    failure = str(exc); continue
                # A rounded station/depth must still engage the real socket.
                # Check before accepting it, so a failed preference can fall
                # through to the continuous solution instead of ending search.
                from .validation import validate
                socket_errors=[issue for issue in validate(posed,collisions=False,geometry=False,support_checks=False)['issues']
                               if issue['severity']=='error' and issue.get('joint')==fitted_joint['id']]
                if socket_errors:
                    failure=socket_errors[0]['message'];continue
                from .snapping import connection_collisions
                check_cancelled(cancelled)
                collisions = connection_collisions(assembly, posed, moving, {aid, bid},cancelled=cancelled) if check_collisions else []
                if collisions:
                    failure = 'Parts still intersect: '+', '.join(dict.fromkeys(f'{a} / {b}' for a, b, _ in collisions))
                    rejected_contacts=[(a, b, point(np.linalg.inv(posed.parts[a].matrix), np.array(contact[5])*1000),
                                        point(np.linalg.inv(posed.parts[b].matrix), np.array(contact[6])*1000), np.array(contact[7]))
                                       for a,b,contact in collisions]
                    continue
                check_cancelled(cancelled)
                poses = {pid: pose_of(matrix) for pid, matrix in actual.items() if not np.allclose(matrix, matrices[pid], atol=1e-7, rtol=0)}
                changes = [{'joint': jid, **values} for jid, values in coordinates.items()]
                angles = [float(np.rad2deg(Rotation.from_matrix(actual[pid][:3, :3]@matrices[pid][:3, :3].T).magnitude())) for pid in poses]
                context = sorted(moving-set(poses))
                return poses, fitted_joint, changes, context, max(angles, default=0)
            if not rejected_contacts: break
            contacts.extend(rejected_contacts)
            seed = q
        if time.monotonic() > deadline: break
    detail = failure or (f'Closest fit leaves {best[1]:.2f} mm and {best[2]:.3f}° of misalignment.' if best else '')
    constraints=[]
    if best_coordinates is not None:
        for mechanism,coordinates,_ in layouts:
            for jid,entries in mechanism.variables.items():
                indices=[coordinates.start+index for _,index,_,_,_ in entries if index is not None and coordinates.start+index in variable]
                if any(min(abs(best_coordinates[i]-lower[i]),abs(best_coordinates[i]-upper[i]))<1e-4 for i in indices):
                    constraints.append(jid)
    if constraints: detail += ' Travel limits reached: '+', '.join(constraints[:6])+(' (and more)' if len(constraints)>6 else '')+'.'
    raise DocumentError('No fit found in this search. '+(detail or 'The search did not establish whether this connection is feasible.'))
