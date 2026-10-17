"""Portable planning cases and checks independent of the planner's search."""
from __future__ import annotations

import copy
import json
import random
from collections import Counter
from pathlib import Path

import numpy as np
import pybullet as pb
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation, Slerp

from pipesim.document import Assembly, joint_kind, read
from pipesim.geometry import collision_primitives, mesh_for_part, shape_transform
from pipesim.math3d import pose_of, transform
from pipesim.validation import CollisionWorld


FIXTURES = Path(__file__).parent / 'fixtures' / 'planning'


def fixture(name):
    return read(FIXTURES / f'{name}.json')


def heavy_workbench(seed=0):
    """An indivisible purchased worktop above the preferred handling mass."""
    doc = read(Path(__file__).resolve().parents[1] / 'examples' / 'workbench.pipe.yaml')
    panel = next(part for part in doc['parts'] if part['id'] == 'worktop')
    panel['mass_kg'] = random.Random(seed).uniform(16, 40)
    return transformed(doc, seed)


def cube(seed=0):
    """Closed rectangular frame, with unequal spans and arbitrary orientation."""
    doc = fixture('cube')
    rng = random.Random(seed)
    reaches = np.array([rng.uniform(180, 530) for _ in range(3)])
    for part in doc['parts']:
        original = np.asarray(part['pose']['position_mm']) - [0, 0, 1000]
        part['pose']['position_mm'] = (original * reaches / 300).tolist()
        if part['catalog'] == 'tubeclamp.tube-C':
            direction = transform(part['pose'])[:3, 2]
            axis = int(np.argmax(np.abs(direction)))
            part['parameters']['length_mm'] += 2 * (reaches[axis] - 300)
    return transformed(doc, seed, ground=False)


def ladder(seed=0, levels=4):
    """Several capped short pipes, each ending in a pair of through tees.

    Every three-part rung has the same sliding freedom on the two long rails.
    Unequal rung spacing and shuffled source order prevent accidental ordering
    by identifiers or a regular lattice.
    """
    rng = random.Random(seed)
    span = rng.uniform(300, 750)
    length = rng.uniform(1100, 1900)
    stations = sorted(rng.sample(range(120, int(length)-120, 100), levels))
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': 'Parallel rail tuples', 'parts': [], 'joints': [],
           'environment': {'ground': False},
           'build': {'require_stability': False}}
    for side, x in (('left', -span/2), ('right', span/2)):
        doc['parts'].append({'id': f'rail-{side}',
            'catalog': 'tubeclamp.tube-C', 'parameters': {'length_mm': length},
            'pose': {'position_mm': [x, 0, length/2]}})
    for index, station in enumerate(stations):
        pipe = f'rung-{index}'
        doc['parts'].append({'id': pipe, 'catalog': 'tubeclamp.tube-C',
            'parameters': {'length_mm': span-69},
            'pose': {'position_mm': [0, 0, station],
                     'rotation_deg': [0, 90, 0]}})
        for side, x, angle, end in (('left', -span/2, 0, 'start'),
                                    ('right', span/2, 180, 'end')):
            tee = f'tee-{side}-{index}'
            doc['parts'].append({'id': tee, 'catalog': 'tubeclamp.TC101C',
                'pose': {'position_mm': [x, 0, station],
                         'rotation_deg': [0, 0, angle]}})
            doc['joints'].extend([
                {'id': f'{tee}-branch', 'type': 'socket',
                 'a': {'part': tee, 'port': 'branch'},
                 'b': {'part': pipe, 'end': end},
                 'insertion_mm': 25, 'locked': True},
                {'id': f'{tee}-through', 'type': 'socket',
                 'a': {'part': tee, 'port': 'through'},
                 'b': {'part': f'rail-{side}', 'at_mm': station},
                 'locked': True}])
    return transformed(doc, seed, ground=False)


def rotated_split_clamp(seed=417):
    """A genuine split clamp goes on between two stops at any world rotation."""
    rng = random.Random(seed)
    length = rng.uniform(900, 1400)
    stations = [('bottom', 40), ('top', length-50),
                ('middle', rng.uniform(.3, .7)*length)]
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': 'Rotated radial collar between stops',
           'parts': [{'id': 'rail', 'catalog': 'tubeclamp.tube-C',
                      'parameters': {'length_mm': length},
                      'pose': {'position_mm': [0, 0, length/2]}}],
           'joints': [], 'anchors': [{'part': 'rail', 'surface': 'fixture'}],
           'environment': {'ground': False},
           'build': {'sequence': ['rail', 'bottom', 'top', 'middle']}}
    for pid, station in stations:
        doc['parts'].append({'id': pid,
            'catalog': 'tubeclamp.TC136C' if pid == 'middle' else 'tubeclamp.TC179C',
            'pose': {'position_mm': [0, 0, station]}})
        doc['joints'].append({'id': f'{pid}-screw', 'type': 'socket',
            'a': {'part': pid, 'port': 'through'},
            'b': {'part': 'rail', 'at_mm': station}, 'locked': True})
    return transformed(doc, seed, ground=False)


def transformed(source, seed, *, ground=True):
    doc = copy.deepcopy(source)
    rng = random.Random(seed)
    # Tilted frames catch implementations that only consider world X/Y/Z cuts.
    angles = [0, 0, rng.uniform(-180, 180)] if ground else [
        rng.uniform(-175, 175) for _ in range(3)]
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
    matrix[:3, 3] = [rng.uniform(-1800, 1800), rng.uniform(-1800, 1800),
                       0 if ground else rng.uniform(-1800, 1800)]
    for part in doc['parts']:
        part['pose'] = pose_of(matrix @ transform(part.get('pose')))
    rng.shuffle(doc['parts'])
    rng.shuffle(doc['joints'])
    return doc


def remote_bore_obstruction(seed=0, *, obstructed=True):
    """A clear declared bore cannot excuse another solid on the same fitting."""
    rng = random.Random(seed)
    distance = rng.uniform(140, 260)
    thickness = rng.uniform(2, 24)
    geometry = [{'type': 'tube', 'diameter_mm': 60, 'wall_mm': 7,
                 'length_mm': 20}]
    if seed & 8:
        # Imported convex proxies may fill a real, explicitly declared bore.
        geometry = [{'type': 'box', 'size_mm': [60, 60, 20]}]
    if obstructed:
        geometry += [
            {'type': 'box', 'size_mm': [100, 100, thickness],
             'position_mm': [0, 0, sign*distance]} for sign in (-1, 1)]
        geometry.append({'type': 'box', 'size_mm': [10, 10, 2*distance],
                         'position_mm': [45, 0, 0]})
        if seed & 16:
            # Opening the local clamp cannot open a separate cage on its body.
            for axis in (0, 1):
                for sign in (-1, 1):
                    position = [0, 0, 0]
                    position[axis] = sign*150
                    size = [340, 340, 20]
                    size[axis] = 20
                    geometry.append({'type': 'box', 'size_mm': size,
                                     'position_mm': position})
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
        'name': 'Bore with remote barriers on the same fitting',
        'parts': [
            {'id': 'pipe', 'catalog': 'tubeclamp.tube-C',
             'parameters': {'length_mm': 200}},
            {'id': 'collar', 'body': {'kind': 'connector', 'mass_kg': 1,
             'geometry': geometry, 'ports': {'through': {'type': 'socket',
                'profile': 'round', 'diameter_mm': 42.4, 'engagement_mm': 20,
                'min_engagement_mm': 10, 'axis': [0, 0, 1],
                'through': True, 'assembly': 'radial' if seed & 16 else 'slide'}}}}],
        'joints': [{'id': 'screw', 'type': 'socket',
                    'a': {'part': 'collar', 'port': 'through'},
                    'b': {'part': 'pipe', 'at_mm': 100}, 'locked': True}],
        'anchors': [{'part': 'pipe', 'surface': 'fixture'}],
        'environment': {'ground': False},
        'build': {'sequence': ['pipe', 'collar']}}
    # Preserve a world-aligned socket axis in retained radial cases so they
    # exercise actual split-opening candidates, not only axial fallback paths.
    return transformed(doc, seed, ground=bool(seed & 16 and seed & 1))


def anchored_cantilever(seed=0):
    """A floor fixing cannot supply unlimited anti-tipping support during a build."""
    rng = random.Random(seed)
    span = rng.uniform(300, 600)
    height = rng.uniform(80, 200)
    center = span/2-20
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
        'name': 'Overhanging load on one floor foot',
        'parts': [
            {'id': 'foot', 'catalog': 'generic.box',
             'parameters': {'width_mm': 40, 'depth_mm': 80,
                            'height_mm': height, 'mass_kg': rng.uniform(.5, 2)},
             'pose': {'position_mm': [0, 0, height/2]}},
            {'id': 'beam', 'catalog': 'generic.box',
             'parameters': {'width_mm': span, 'depth_mm': 50,
                            'height_mm': 20, 'mass_kg': rng.uniform(40, 90)},
             'pose': {'position_mm': [center, 0, height+10]}}],
        'joints': [{'id': 'beam-foot', 'type': 'fixed',
                    'a': {'part': 'foot', 'frame': {'position_mm': [0, 0, height/2]}},
                    'b': {'part': 'beam', 'frame': {'position_mm': [-center, 0, -10]}}}],
        'anchors': [{'part': 'foot', 'surface': 'floor'}]}
    return transformed(doc, seed, ground=True)


def capture_frame(seed=0):
    """Two feet and tall posts joined by a capped rung near their tops.

    With one post removed, the remaining connected frame would tip about its
    single foot. A bounded assisted capture can hold that group while the
    joining post passes through the top tee and seats in the other foot.
    """
    rng = random.Random(seed)
    span, length = rng.uniform(350, 750), rng.uniform(1000, 1600)
    height = rng.uniform(.55, .8)*length+23.3
    doc = {'format': 'pipesim/1', 'units': 'mm-kg-s-N-deg',
           'name': 'Assisted capture between two floor feet',
           'parts': [{'id': 'rung', 'catalog': 'tubeclamp.tube-C',
                      'parameters': {'length_mm': span-69},
                      'pose': {'position_mm': [0, 0, height], 'rotation_deg': [0, 90, 0]}}],
           'joints': [], 'anchors': []}
    for side, x, angle, end in (('left', -span/2, 0, 'start'),
                                ('right', span/2, 180, 'end')):
        foot, rail, tee = f'foot-{side}', f'rail-{side}', f'tee-{side}'
        doc['parts'].extend([
            {'id': foot, 'catalog': 'tubeclamp.TC131C',
             'pose': {'position_mm': [x, 0, 0]}},
            {'id': rail, 'catalog': 'tubeclamp.tube-C',
             'parameters': {'length_mm': length},
             'pose': {'position_mm': [x, 0, length/2+23.3]}},
            {'id': tee, 'catalog': 'tubeclamp.TC101C',
             'pose': {'position_mm': [x, 0, height], 'rotation_deg': [0, 0, angle]}}])
        doc['joints'].extend([
            {'id': f'{foot}-post', 'type': 'socket', 'a': {'part': foot, 'port': 'socket'},
             'b': {'part': rail, 'end': 'start'}, 'insertion_mm': 30, 'locked': True},
            {'id': f'{tee}-post', 'type': 'socket', 'a': {'part': tee, 'port': 'through'},
             'b': {'part': rail, 'at_mm': height-23.3}, 'locked': True},
            {'id': f'{tee}-rung', 'type': 'socket', 'a': {'part': tee, 'port': 'branch'},
             'b': {'part': 'rung', 'end': end}, 'insertion_mm': 25, 'locked': True}])
        doc['anchors'].append({'part': foot, 'surface': 'floor'})
    return transformed(doc, seed, ground=True)


def assert_complete_plan(assembly, result, *, grouped=False):
    assert result['status'] == 'buildable', result
    steps = result['steps']
    assert steps
    introduced = Counter()
    fastened = Counter()
    anchored = Counter()
    seen = set()
    staged = set()
    locations = {}
    joints = {joint['id']: joint for joint in assembly.joints}
    for number, step in enumerate(steps, 1):
        assert step['number'] == number
        added = step.get('introduced_parts', step.get('parts', [step['part']]))
        introduced.update(added)
        assert not (set(added) & seen), step
        seen.update(added)
        moving = set(step.get('moving_parts', added))
        target = set(step.get('target_parts', ()))
        assert moving <= seen and target <= seen, step
        assert not moving & target, step
        lift = step.get('kind') == 'lift'
        context = step.get('subassembly', 'main')
        available = {pid for pid, location in locations.items() if location == context}
        staging = set(step.get('staged_parts', ()))
        if staging:
            assert staging == moving == set(added) and not target, step
        elif not lift:
            assert target == available-moving, (step, available)
        prepared = step.get('prepared_subassembly')
        if lift:
            assert not added and moving, step
            assert not step.get('fasten') and not step.get('world_anchors'), step
        elif prepared:
            assert not added, step
            assert moving == {pid for pid, location in locations.items()
                              if location == prepared}, step
        elif not added and moving <= staged:
            assert moving, step
        else:
            assert moving == set(added), step
        if len(moving) > 1:
            reached = {next(iter(moving))}
            while True:
                before = len(reached)
                for jid in fastened:
                    ends = {joints[jid]['a']['part'], joints[jid]['b']['part']}
                    if ends <= moving and ends & reached:
                        reached.update(ends)
                if len(reached) == before:
                    break
            assert reached == moving, 'A transferred group is not yet connected'
        if lift:
            # The logical assembly context changes when the subsequent merge
            # fastens it; lifting only changes physical poses in the meantime.
            continue
        staged.update(staging)
        if not staging:
            staged.difference_update(moving)
        for pid in moving:
            locations[pid] = context
        crossing = {joint['id'] for joint in assembly.joints if
            ((joint['a']['part'] in moving and joint['b']['part'] in target) or
             (joint['b']['part'] in moving and joint['a']['part'] in target))}
        assert Counter(f['joint'] for f in step.get('fasten', [])) == Counter(
            {jid: 1 for jid in crossing}), step
        for jid in crossing:
            assert {locations[joints[jid][end]['part']] for end in ('a', 'b')} == {context}
        if not crossing:
            assert step.get('kind') == 'position', step
            wording = step['instruction'].lower()
            assert not ('slide' in wording and 'onto' in wording), step
            if target:
                if step.get('capture_pending'):
                    assert 'helper' in wording and 'hold' in wording, step
                else:
                    assert 'separat' in wording and ('support' in wording or 'hold' in wording), step
        offset = np.asarray(step.get('design_start_offset_mm',
                                     step['start_offset_mm']), dtype=float)
        approach = np.asarray(step.get('design_approach_direction',
                                       step['approach_direction']), dtype=float)
        assert np.isfinite(offset).all() and np.isfinite(approach).all(), step
        assert np.linalg.norm(approach) == pytest_approx_one(), step
        if np.linalg.norm(offset) > 1e-8:
            direction = offset / np.linalg.norm(offset)
            assert np.allclose(direction, -approach, atol=1e-7), step
            for joint in assembly.joints:
                a, b = joint['a']['part'], joint['b']['part']
                if not ((a in moving and b in target) or
                        (b in moving and a in target)):
                    continue
                if joint.get('type') != 'socket':
                    continue
                port = assembly.parts[a].ports[joint['a']['port']]
                if port.get('assembly') == 'radial':
                    continue
                _, _, axis = assembly.joint_frames(joint)
                alignment = float(direction @ axis)
                assert abs(alignment) > .998, (step, joint)
                # A closed end admits only withdrawal through its mouth.
                if not port.get('through'):
                    assert alignment * (1 if b in moving else -1) > .998, (
                        step, joint)
        fastened.update(f['joint'] for f in step.get('fasten', []))
        mounts = step.get('world_anchors', [step['world_anchor']]
                          if step.get('world_anchor') else [])
        anchored.update(anchor['part'] for anchor in mounts)
        if step.get('subassembly', 'main') != 'main':
            assert not mounts, 'A detached group cannot be world anchored before its slide'
            assert step.get('manual_support') or step.get('grounded_preparation'), step
    assert introduced == Counter({pid: 1 for pid in assembly.parts}), introduced
    assert set(steps[-1]['installed_parts']) == set(assembly.parts)
    assert fastened == Counter({joint['id']: 1 for joint in assembly.joints}), fastened
    assert anchored == Counter(anchor['part'] for anchor in assembly.anchors), anchored
    assert set(locations.values()) == {'main'}, locations
    assert not staged, 'The plan leaves loose staged parts unassembled'
    if grouped:
        assert any(len(step.get('moving_parts', ())) > 1 for step in steps), result
    # A certificate must identify the geometry/tolerance assumptions used.
    assert result['certificate']['collision_tolerance_mm'] <= 1
    assert result['input_sha256'] == assembly.input_hash


def assert_sweep_samples(assembly, result):
    """Replay intermediate group poses without consulting planner caches.

    Contact allowances are limited to the finite declared bore. In particular,
    another obstruction on the same fitting is still tested. The thin-obstacle
    regression separately covers collisions between these sampled positions.
    """
    sockets = {}
    for joint in assembly.joints:
        if joint.get('type') == 'socket':
            sockets.setdefault(frozenset((joint['a']['part'], joint['b']['part'])),
                               []).append(joint)
    with CollisionWorld(assembly) as world:
        def bore_contact(pid, other, contact, moving, delta, direction):
            for joint in sockets.get(frozenset((pid, other)), ()):
                connector = assembly.parts[joint['a']['part']]
                port = connector.ports[joint['a']['port']]
                mouth, axis = connector.frame(joint['a'])
                if connector.id in moving:
                    mouth = mouth + delta
                low = (-float(port.get('engagement_mm', 0))/2 if
                       port.get('through') else -float(port.get('engagement_mm', 0)))
                high = -low if port.get('through') else 0
                relative = np.asarray([contact[5], contact[6]])*1000 - mouth
                axial = relative @ axis
                radial = np.linalg.norm(relative - np.outer(axial, axis), axis=1)
                if (min(axial) >= low-2 and max(axial) <= high+2 and
                        min(radial) <= float(port['diameter_mm'])/2+2):
                    return True
                if (port.get('assembly') == 'radial' and
                        abs(direction @ axis) < .01 and
                        min(axial) >= low-2 and max(axial) <= high+2):
                    # A declared split clamp opens its finite coaxial housing,
                    # including the wall outside the bore. Infer that housing
                    # independently from actual annular geometry; attached
                    # branches, remote plates, and other body solids stay solid.
                    for shape in connector.shapes:
                        if shape['type'] != 'tube':
                            continue
                        matrix = connector.matrix @ shape_transform(shape)
                        center = matrix[:3, 3] + (delta if connector.id in moving else 0)
                        along = float((center-mouth) @ axis)
                        across = center-mouth-axis*along
                        half = float(shape['length_mm'])/2
                        radius = float(shape.get('radius_mm', shape.get('diameter_mm', 0)/2))
                        if (abs(matrix[:3, 2] @ axis) > .999999 and
                                np.linalg.norm(across) < 1e-4 and
                                along-half >= low-2 and along+half <= high+2 and
                                min(axial) >= along-half-2 and max(axial) <= along+half+2 and
                                max(radial) <= radius+2):
                            return True
                    # Authored mesh fittings often describe their split ring
                    # as tangent boxes. Check the actual contact against one
                    # finite wall box adjacent to the bore, rather than treating
                    # all material on the same fitting as an opened clamp.
                    corner_signs = np.array([[x, y, z] for x in (-1, 1)
                                             for y in (-1, 1) for z in (-1, 1)])
                    bore_radius = float(port['diameter_mm'])/2
                    for shape, local in collision_primitives(connector):
                        if shape['type'] != 'box':
                            continue
                        matrix = connector.matrix @ local
                        if connector.id in moving:
                            matrix[:3, 3] += delta
                        half_size = np.asarray(shape['size_mm'])/2
                        corners = (corner_signs*half_size) @ matrix[:3, :3].T+matrix[:3, 3]-mouth
                        along = corners @ axis
                        radii = np.linalg.norm(corners-np.outer(along, axis), axis=1)
                        if (along.min() < low-.01 or along.max() > high+.01 or
                                np.ptp(along) < .8*(high-low) or
                                radii.min() < bore_radius-.5 or
                                radii.min() > bore_radius+port.get('clearance_mm', 0)+2 or
                                radii.max() > 2*bore_radius):
                            continue
                        contact_point = np.asarray(contact[5 if pid == connector.id else 6])*1000
                        local_contact = matrix[:3, :3].T @ (contact_point-matrix[:3, 3])
                        if np.all(np.abs(local_contact) <= half_size+2):
                            return True
            return False

        for step in result['steps']:
            if step.get('kind') == 'lift':
                continue
            offset = np.asarray(step.get('design_start_offset_mm',
                                         step['start_offset_mm']), dtype=float)
            if not np.linalg.norm(offset) or not step.get('target_parts'):
                continue
            direction = offset / np.linalg.norm(offset)
            moving = set(step.get('moving_parts', [step['part']]))
            targets = step['target_parts']
            bounds = np.array([world.boxes[pid] for pid in targets])
            for fraction in (0, .137, .5, .863, 1):
                delta = offset*fraction
                for pid in moving:
                    moved_box = world.boxes[pid] + delta
                    candidates = np.flatnonzero(
                        np.all(moved_box[1] >= bounds[:, 0]-1, axis=1) &
                        np.all(bounds[:, 1]+1 >= moved_box[0], axis=1))
                    if not len(candidates):
                        continue
                    world.move(pid, delta)
                    try:
                        for index in candidates:
                            other = targets[index]
                            contacts = pb.getClosestPoints(
                                world.world.part_map[pid][0],
                                world.world.part_map[other][0], distance=0,
                                physicsClientId=world.world.client)
                            for contact in contacts:
                                assert contact[8]*1000 >= -1 or bore_contact(
                                    pid, other, contact, moving, delta, direction), (
                                    f'Step {step["number"]}, fraction {fraction}, '
                                    f'{pid} crosses {other}: {contact[8]*1000:g} mm')
                    finally:
                        world.move(pid, [0, 0, 0])

        # Replay the transfer motions in their actual floor-preparation poses.
        # Other completed groups remain obstacles even when the current panel
        # only displays one named subassembly. Nothing consults planner caches.
        body_from_part = {}
        for pid, (position, quaternion) in world.positions.items():
            body = np.eye(4)
            body[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
            body[:3, 3] = np.asarray(position)*1000
            body_from_part[pid] = np.linalg.inv(assembly.parts[pid].matrix) @ body

        def move_pose(pid, matrix):
            body = matrix @ body_from_part[pid]
            pb.resetBasePositionAndOrientation(world.world.part_map[pid][0],
                body[:3, 3]/1000, Rotation.from_matrix(body[:3, :3]).as_quat(),
                physicsClientId=world.world.client)

        known_poses, local_vertices = {}, {}
        for step in result['steps']:
            if 'poses' not in step:
                continue
            if step.get('kind') != 'lift' and not step.get('pickup_before_insert'):
                known_poses.update({pid: transform(pose)
                                   for pid, pose in step['poses'].items()})
                continue
            moving = set(step['moving_parts'])
            start = {pid: transform(pose) for pid, pose in step['start_poses'].items()}
            known_poses.update(start)
            stationary = sorted(set(known_poses)-moving)
            for pid in stationary:
                move_pose(pid, known_poses[pid])
            boxes = np.asarray([pb.getAABB(world.world.part_map[pid][0],
                physicsClientId=world.world.client) for pid in stationary])*1000
            bounds = []
            for pid in moving:
                if pid not in local_vertices:
                    local_vertices[pid] = np.asarray(mesh_for_part(assembly.parts[pid]).vertices)
                original = assembly.parts[pid].matrix
                points = local_vertices[pid] @ original[:3, :3].T+original[:3, 3]
                bounds.append([points.min(axis=0), points.max(axis=0)])
            bounds = np.asarray(bounds)
            center = (bounds[:, 0].min(axis=0)+bounds[:, 1].max(axis=0))/2
            representative = next(iter(moving))
            original_inverse = np.linalg.inv(assembly.parts[representative].matrix)
            path = [step['start_poses'], *step['motion_waypoints']]
            if not step.get('pickup_before_insert'):
                path.append(step['poses'])
            for first, last in zip(path, path[1:]):
                first = transform(first[representative]) @ original_inverse
                last = transform(last[representative]) @ original_inverse
                first_center = first[:3, :3] @ center+first[:3, 3]
                last_center = last[:3, :3] @ center+last[:3, 3]
                rotations = Slerp([0, 1], Rotation.from_matrix([first[:3, :3], last[:3, :3]]))
                for fraction in (0, .173, .5, .827, 1):
                    delta = np.eye(4)
                    delta[:3, :3] = rotations(fraction).as_matrix()
                    delta[:3, 3] = ((1-fraction)*first_center+fraction*last_center
                                       -delta[:3, :3] @ center)
                    for pid in moving:
                        pose = delta @ assembly.parts[pid].matrix
                        points = local_vertices[pid] @ pose[:3, :3].T+pose[:3, 3]
                        assert points[:, 2].min() >= step['floor_z_mm']-1, (
                            f'Lift step {step["number"]} crosses its floor at {fraction:g}')
                        move_pose(pid, pose)
                        if not stationary:
                            continue
                        box = np.asarray(pb.getAABB(world.world.part_map[pid][0],
                            physicsClientId=world.world.client))*1000
                        candidates = np.flatnonzero(
                            np.all(box[1] >= boxes[:, 0]-1, axis=1) &
                            np.all(boxes[:, 1]+1 >= box[0], axis=1))
                        for index in candidates:
                            other = stationary[index]
                            contacts = pb.getClosestPoints(world.world.part_map[pid][0],
                                world.world.part_map[other][0], distance=0,
                                physicsClientId=world.world.client)
                            assert all(contact[8]*1000 >= -1 for contact in contacts), (
                                f'Lift step {step["number"]}: {pid} crosses {other}')
            known_poses.update({pid: transform(pose)
                               for pid, pose in step['poses'].items()})


def assert_gravity_support(assembly, result):
    """Check displayed preparation geometry without trusting support flags.

    Completed components must rest in equilibrium on the preparation floor.
    Only one already-connected component may be held during an explicit lift
    or assisted capture; the next joining step must attach it to the rested
    base. Legacy plans without pose maps are checked at their displayed design
    poses, so the original many-floating-parts failure is also detected.
    """
    joints = {joint['id']: joint for joint in assembly.joints}
    vertices, local_centers, geometry_cache, support_cache = {}, {}, {}, {}
    fastened, anchors, previous = set(), set(), {}
    pending_lift = set()
    pending_capture, capture_base = set(), set()
    capture_declared_mass = None
    pending_grounding = set()
    explicit_limit = assembly.doc.get('build', {}).get('max_lift_mass_kg')
    initial_limit = max(15., max((part.mass for part in assembly.parts.values()), default=0.))
    certificate = result.get('certificate', {})
    handling_limit = explicit_limit if explicit_limit is not None else initial_limit
    if explicit_limit is None and certificate.get('fallback_used'):
        handling_limit = certificate['max_lift_mass_kg']
        assert np.isfinite(handling_limit) and initial_limit <= handling_limit
        assert handling_limit <= sum(part.mass for part in assembly.parts.values())+1e-6
    if 'max_lift_mass_kg' in certificate:
        assert abs(certificate['max_lift_mass_kg']-handling_limit) <= 1e-6
    if 'lift_limit_is_explicit' in certificate:
        assert certificate['lift_limit_is_explicit'] == (explicit_limit is not None)
        assert certificate['preferred_lift_mass_kg'] == 15
        assert explicit_limit is None or not certificate['fallback_used']
    maximum_handled_mass = 0.

    def same_relative_pose(actual, expected):
        # Exported Euler angles near a gimbal singularity lose a few 1e-7 in
        # rotation. Check rotation and physical displacement separately, with
        # one-micron translation tolerance rather than world-coordinate rtol.
        return (np.allclose(actual[:3, :3], expected[:3, :3], atol=1e-6, rtol=0)
                and np.allclose(actual[:3, 3], expected[:3, 3], atol=1e-3, rtol=0))

    def components(parts, connections):
        pending, groups = set(parts), []
        while pending:
            group = {pending.pop()}
            while True:
                reached = set(group)
                for jid in connections:
                    joint = joints[jid]
                    ends = {joint['a']['part'], joint['b']['part']}
                    if joint_kind(joint) == 'fixed' and ends <= parts and ends & group:
                        reached.update(ends)
                if reached == group:
                    break
                group = reached
            pending.difference_update(group)
            groups.append(group)
        return groups

    def world_geometry(pid, matrix):
        key = pid, tuple(np.round(matrix.ravel(), 7))
        if key not in geometry_cache:
            part = assembly.parts[pid]
            if pid not in vertices:
                vertices[pid] = np.asarray(mesh_for_part(part).vertices)
                # The local center is constant. Its property reconstructs the
                # imported mesh, so evaluate it once rather than at every pose.
                local_centers[pid] = np.asarray(part.center_of_mass)
            points = vertices[pid] @ matrix[:3, :3].T + matrix[:3, 3]
            com = matrix[:3, :3] @ local_centers[pid] + matrix[:3, 3]
            geometry_cache[key] = points, com
        return geometry_cache[key]

    def supported(group, poses, floor, step):
        key = floor, tuple((pid, tuple(np.round(poses[pid].ravel(), 7)))
                           for pid in sorted(group))
        if key in support_cache:
            return
        contact_points, centers, masses = [], [], []
        for pid in group:
            points, com = world_geometry(pid, poses[pid])
            assert points[:, 2].min() >= floor-1, (
                f'Step {step["number"]}: {pid} penetrates its preparation floor')
            contact_points.extend(points[points[:, 2] <= floor+1, :2])
            centers.append(com)
            masses.append(assembly.parts[pid].mass)
        assert contact_points, (
            f'Step {step["number"]}: component {sorted(group)} floats above its floor')
        contacts = np.unique(np.round(contact_points, 6), axis=0)
        com = np.average(centers, axis=0, weights=masses)[:2]
        centered = contacts-contacts.mean(axis=0)
        rank = np.linalg.matrix_rank(centered, tol=1e-6)
        if rank == 0:
            margin = np.linalg.norm(com-contacts[0])
        elif rank == 1:
            axis = np.linalg.svd(centered, full_matrices=False)[2][0]
            stations = contacts @ axis
            foot = contacts[0] + axis*np.clip(
                (com-contacts[0]) @ axis,
                stations.min()-stations[0], stations.max()-stations[0])
            margin = np.linalg.norm(com-foot)
        else:
            hull = ConvexHull(contacts)
            margin = np.max(hull.equations[:, :2] @ com+hull.equations[:, 2])
        assert margin <= 1, (
            f'Step {step["number"]}: component {sorted(group)} tips over its '
            f'floor contacts by {margin:g} mm')
        support_cache[key] = True

    for step in result['steps']:
        visible = set(step['installed_parts'])
        shown = step.get('poses', {pid: pose_of(assembly.parts[pid].matrix)
                                  for pid in visible})
        assert set(shown) == visible, step
        poses = {pid: transform(pose) for pid, pose in shown.items()}
        moving = set(step.get('moving_parts', ()))
        actual_mass = sum(assembly.parts[pid].mass for pid in moving)
        maximum_handled_mass = max(maximum_handled_mass, actual_mass)
        assert actual_mass <= handling_limit+1e-6, (
            f'Step {step["number"]} handles {actual_mass:g} kg above the '
            f'{handling_limit:g} kg selected limit')
        if 'group_mass_kg' in step:
            assert abs(step['group_mass_kg']-actual_mass) <= 1e-6, step
        if actual_mass > 15:
            assert step.get('assistance_required'), step
            assert any(word in step.get('instruction', '').lower()
                       for word in ('team', 'lifting equipment')), step
        lift = step.get('kind') == 'lift'
        pickup = step.get('pickup_before_insert', False)
        context = step.get('subassembly', 'main')
        floor = float(step.get('floor_z_mm',
            assembly.doc.get('environment', {}).get('ground_z_mm', 0)))
        if pending_grounding:
            assert lift and moving == pending_grounding and step.get('grounded_after'), (
                'A helper-held completed group must immediately be put on its stable floor face', step)
            pending_grounding = set()
        if not lift:
            for pid in (visible-moving) & set(previous):
                assert np.allclose(poses[pid], previous[pid], atol=1e-5), (
                    'A retained base moves without an explicit transfer', step, pid)
            if (not pickup and (step.get('fasten') or step.get('capture_pending')) and
                    not step.get('introduced_parts') and moving <= set(previous)):
                offset = np.asarray(step['start_offset_mm'], dtype=float)
                for pid in moving:
                    insertion_start = poses[pid].copy()
                    insertion_start[:3, 3] += offset
                    assert np.allclose(previous[pid], insertion_start, atol=1e-5), (
                        'A prepared group jumps to its insertion start', step, pid)
        if pending_lift:
            assert (not lift and moving == pending_lift and
                    (step.get('fasten') or step.get('capture_pending'))), (
                'An airborne group must next be joined to its floor-rested base', step)
            offset = np.asarray(step['start_offset_mm'], dtype=float)
            for pid in moving:
                insertion_start = poses[pid].copy()
                insertion_start[:3, 3] += offset
                assert np.allclose(previous[pid], insertion_start, atol=1e-5), (
                    'Lift destination does not match the insertion start', step, pid)
            pending_lift.clear()
        completing_capture = set(pending_capture)
        if pending_capture:
            assert step.get('capture_complete') and not lift, (
                'A held capture group must be joined in the next step', step)
            assert set(step.get('held_parts', ())) == pending_capture, step
            assert len(moving) == 1 and not moving & pending_capture, step
            assert step.get('helper_required') and step.get('assistance_required'), step
            combined_mass = sum(assembly.parts[pid].mass for pid in moving | pending_capture)
            maximum_handled_mass = max(maximum_handled_mass, combined_mass)
            assert combined_mass <= handling_limit+1e-6, step
            assert abs(step['combined_handling_mass_kg']-combined_mass) <= 1e-6, step
            if capture_declared_mass is not None:
                assert abs(capture_declared_mass-combined_mass) <= 1e-6, step
            pending_capture = set()
        else:
            assert not step.get('capture_complete'), 'Capture completes without a held group'
        if lift or pickup:
            if pickup:
                assert len(moving) == 1 and moving == set(step['introduced_parts']), step
                assert moving <= set(step['start_poses']), step
                supported(moving, {pid: transform(pose)
                    for pid, pose in step['start_poses'].items()}, floor, step)
            else:
                assert moving and len(components(moving, fastened)) == 1, (
                    'Only a previously fastened connected component may be lifted', step)
                assert not step.get('introduced_parts') and not step.get('fasten'), step
            assert abs(step['group_mass_kg']-actual_mass) <= 1e-6, step
            start = step['start_poses']
            assert moving <= set(start), step
            for pid in visible & set(previous):
                assert np.allclose(transform(start[pid]), previous[pid], atol=1e-5), (
                    'A transfer starts from an unshown part pose', step, pid)
            if not pickup:
                assert moving <= set(previous), step
            waypoints = [start, *step['motion_waypoints']]
            if pickup:
                assert len(waypoints) >= 2, step
                offset = np.asarray(step['start_offset_mm'], dtype=float)
                for pid in moving:
                    insertion_start = poses[pid].copy()
                    insertion_start[:3, 3] += offset
                    assert np.allclose(transform(waypoints[-1][pid]), insertion_start,
                                       atol=1e-5), ('Pickup does not reach insertion start', step)
            else:
                waypoints.append(shown)
            for waypoint in waypoints:
                assert set(waypoint) == visible, step
                reference = next(iter(moving))
                reference_actual = np.linalg.inv(transform(waypoint[reference]))
                reference_design = np.linalg.inv(assembly.parts[reference].matrix)
                assert all(same_relative_pose(reference_actual @ transform(waypoint[pid]),
                    reference_design @ assembly.parts[pid].matrix) for pid in moving), (
                    'A lift deforms its rigid group', step['number'])
                for pid in visible-moving:
                    assert np.allclose(transform(waypoint[pid]), poses[pid],
                                       atol=1e-5), 'A lift moves its retained base'
                for pid in moving:
                    points, _ = world_geometry(pid, transform(waypoint[pid]))
                    assert points[:, 2].min() >= floor-1, (
                        'A lift waypoint passes through the preparation floor', step, pid)
        new_connections = {item['joint'] for item in step.get('fasten', ())}
        for jid in fastened | new_connections:
            joint = joints[jid]
            a, b = joint['a']['part'], joint['b']['part']
            if {a, b} <= visible and joint_kind(joint) == 'fixed':
                assert same_relative_pose(np.linalg.inv(poses[a]) @ poses[b],
                    np.linalg.inv(assembly.parts[a].matrix) @ assembly.parts[b].matrix,
                    ), ('Fastened parts have inconsistent rigid poses', step, jid)
        fastened.update(new_connections)
        if step.get('capture_pending'):
            assert not completing_capture and not new_connections, step
            assert moving == set(step.get('held_parts', ())), step
            assert len(components(moving, fastened)) == 1, step
            assert step.get('helper_required') and step.get('assistance_required'), step
            assert visible-moving, 'A capture needs a separately supported base'
            capture_declared_mass = step.get('combined_handling_mass_kg')
        if step.get('needs_grounding'):
            held = set(step.get('held_parts', ()))
            assert held and len(components(held, fastened)) == 1, step
            assert step.get('helper_required') and step.get('assistance_required'), step
            assert not step.get('capture_pending') and not step.get('capture_complete'), step
            mass = sum(assembly.parts[pid].mass for pid in held)
            maximum_handled_mass = max(maximum_handled_mass, mass)
            assert mass <= handling_limit+1e-6, ('The supported handoff exceeds the handling bound', step)
            pending_grounding = held
        if completing_capture:
            assert any(completing_capture | moving <= group and group & capture_base
                       for group in components(visible, fastened)), (
                'The joining pipe does not fasten the held group to its supported base', step)
        anchors.update(anchor['part'] for anchor in step.get('world_anchors', ()))
        check_floor = (context != 'main' or step.get('grounded_preparation') or
            (assembly.doc.get('build', {}).get('require_stability', True) and
             assembly.doc.get('environment', {}).get('ground', True)))
        if not lift and not step.get('staged_parts') and moving:
            pid = next(iter(moving))
            preparation = poses[pid] @ np.linalg.inv(assembly.parts[pid].matrix)
            if 'design_start_offset_mm' in step:
                assert np.allclose(step['start_offset_mm'],
                    preparation[:3, :3] @ step['design_start_offset_mm'], atol=1e-3, rtol=0), step
                assert np.allclose(step['approach_direction'],
                    preparation[:3, :3] @ step['design_approach_direction'], atol=1e-6, rtol=0), step
            if step.get('grounded_preparation'):
                offset = np.asarray(step['start_offset_mm'], dtype=float)
                for pid in moving:
                    points, _ = world_geometry(pid, poses[pid])
                    assert points[:, 2].min()+min(0, offset[2]) >= floor-1, (
                        'An insertion crosses the preparation floor', step, pid)
        for group in components(visible, fastened):
            if step.get('needs_grounding') and group == pending_grounding:
                continue
            if step.get('capture_pending') and group == moving:
                assert group == set(step.get('held_parts', ())), step
                assert step.get('helper_required') and step.get('assistance_required'), step
                assert not new_connections, step
                assert len(components(group, fastened)) == 1, step
                pending_capture, capture_base = set(group), visible-group
                continue
            if group & anchors and context == 'main':
                for pid in group & anchors:
                    assert np.allclose(poses[pid], assembly.parts[pid].matrix,
                                       atol=1e-5), 'A world-anchored part moved'
                if not assembly.doc.get('environment', {}).get('ground', True):
                    continue
            if lift and group <= moving and step.get('manual_support'):
                assert group == moving, step
                try:
                    supported(group, poses, floor, step)
                except AssertionError:
                    assert min(world_geometry(pid, poses[pid])[0][:, 2].min()
                               for pid in group) >= floor-1, step
                    pending_lift = group
                    continue
            if check_floor:
                supported(group, poses, floor, step)
        previous.update(poses)
    assert not pending_lift, 'The plan ends with an airborne component'
    assert not pending_capture, 'The plan leaves a helper holding an unconnected component'
    assert not pending_grounding, 'The plan ends before grounding the helper-held group'
    if 'maximum_handled_mass_kg' in certificate:
        assert abs(certificate['maximum_handled_mass_kg']-maximum_handled_mass) <= 1e-6
    for pid, part in assembly.parts.items():
        assert np.allclose(previous[pid], part.matrix, atol=1e-5), (
            'The completed part did not reach its design pose', pid)


def pytest_approx_one():
    # Keep the generators importable by the standalone fuzz worker.
    import pytest
    return pytest.approx(1, abs=1e-7)
