"""Length-agnostic pipe runs. Drafts are editor graph data, not physical parts."""
from __future__ import annotations

import copy
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation

from .document import Assembly, DocumentError, joint_kind, substitute
from .math3d import UnionFind, axis_frame, pose_of, transform, unit, vec

THROUGH_REPAIR_MARGIN_MM = 0.1
THROUGH_ENGAGEMENT_ROUNDOFF_MM = 0.001
THROUGH_INFER_ALLOWANCE_MM = 1.0
THROUGH_AXIS_TOLERANCE_DEG = float(np.degrees(np.arccos(.999)))


class DraftCancelled(RuntimeError):
    pass


def _check_cancelled(cancelled):
    if cancelled and cancelled():
        raise DraftCancelled('Draft operation cancelled; the draft was not changed')


def runs(doc):
    return [run for group in doc.get('draft_subassemblies', []) for run in group['runs']]


def _mirror_reopen_digest(document):
    """Identify an unchanged exact design while ignoring generated analysis."""
    clean = copy.deepcopy(document)
    clean.pop('results', None)
    clean.pop('build_plan', None)
    clean.get('metadata', {}).pop('draft_mirror_reopen', None)
    if not clean.get('metadata'):
        clean.pop('metadata', None)
    return hashlib.sha256(json.dumps(clean, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def remember_mirrored_draft(source_document, finalized_result):
    """Keep enough round-trip information to reopen baked mirror copies as previews."""
    if finalized_result['status'] != 'finalized':
        return finalized_result
    source = copy.deepcopy(source_document)
    source.get('metadata', {}).pop('draft_mirror_reopen', None)
    finished = copy.deepcopy(finalized_result['document'])
    finished_member_ids = set(finalized_result['lengths_mm'])
    mirror_groups = [group for group in source.get('draft_subassemblies', [])
                     if group.get('mirrors') and any(
                         run['id'] in finished_member_ids for run in group['runs'])]
    if not mirror_groups:
        return finalized_result
    source_part_ids = {part['id'] for part in source.get('parts', [])}
    required_parts = ({part['id'] for part in finished['parts']} - source_part_ids) | finished_member_ids
    for group in mirror_groups:
        required_parts.update(attachment['connector'] for run in group['runs']
                              for attachment in run.get('attachments', []))
    record = {'version': 1, 'source_document': source, 'finished_members': sorted(finished_member_ids),
              'mirror_group_ids': [group['id'] for group in mirror_groups],
              'required_parts': sorted(required_parts),
              'final_digest': _mirror_reopen_digest(finished)}
    finished.setdefault('metadata', {})['draft_mirror_reopen'] = record
    finalized_result['document'] = finished
    return finalized_result


def _reopen_mirrored_assembly(assembly, members):
    record = assembly.doc.get('metadata', {}).get('draft_mirror_reopen')
    if (not isinstance(record, dict) or record.get('version') != 1 or
            not isinstance(record.get('source_document'), dict) or
            not isinstance(record.get('finished_members'), list) or
            not isinstance(record.get('mirror_group_ids'), list) or
            not isinstance(record.get('required_parts'), list) or
            record.get('final_digest') != _mirror_reopen_digest(assembly.doc)):
        return None
    chosen = set(members)
    related = set(record['required_parts'])
    if not chosen <= related or not chosen.intersection(record['finished_members']):
        return None
    document = copy.deepcopy(record['source_document'])
    Assembly.from_doc(document, assembly.base, assembly.library,
                      validate_mirror_geometry=False)
    source_joint_ids = {joint['id'] for joint in document.get('joints', [])}
    return {'status': 'reopened', 'document': document,
            'subassembly': record['mirror_group_ids'][0],
            'restored_mirrors': sum(len(group.get('mirrors', []))
                                    for group in document['draft_subassemblies']
                                    if group['id'] in record['mirror_group_ids']),
            'converted_parts': [run['id'] for group in document['draft_subassemblies']
                                for run in group['runs'] if run['id'] in record['finished_members']],
            'removed_joints': sorted(joint['id'] for joint in assembly.doc.get('joints', [])
                                     if joint['id'] not in source_joint_ids)}


def _coalesce_overlapping_mirror_runs(assembly):
    """Join already baked mirror copies that occupy the same pipe centreline.

    Older editor documents can contain both halves after baking a nearly
    centered crossing. Preserve the original run and all distinct sockets.
    """
    document = copy.deepcopy(assembly.doc)
    aliases = {}
    changed = False
    for group in document.get('draft_subassemblies', []):
        if group.get('mirrors'):
            continue
        retained = []
        for candidate in group['runs']:
            source = next((run for run in retained
                           if candidate['id'].startswith(run['id'] + '-mirror-')), None)
            if (source is None or source['catalog'] != candidate['catalog'] or
                    source.get('parameters', {}) != candidate.get('parameters', {}) or
                    source.get('locked_length_mm') is not None or
                    candidate.get('locked_length_mm') is not None):
                retained.append(candidate)
                continue
            start, end = np.asarray(source['start_mm'], dtype=float), np.asarray(source['end_mm'], dtype=float)
            other_start = np.asarray(candidate['start_mm'], dtype=float)
            other_end = np.asarray(candidate['end_mm'], dtype=float)
            length = np.linalg.norm(end-start)
            if length < 1:
                retained.append(candidate)
                continue
            direction = (end-start)/length
            def station(point):
                delta = point-start
                along = float(delta @ direction)
                return along, float(np.linalg.norm(delta-direction*along))
            low, low_error = station(other_start)
            high, high_error = station(other_end)
            overlap = min(length, max(low, high))-max(0, min(low, high))
            if max(low_error, high_error) > .1 or overlap <= 1:
                retained.append(candidate)
                continue
            minimum, maximum = min(0, low, high), max(length, low, high)
            source['start_mm'] = (start+direction*minimum).tolist()
            source['end_mm'] = (start+direction*maximum).tolist()
            for attachment in candidate.get('attachments', []):
                attachment = copy.deepcopy(attachment)
                if attachment.get('end'):
                    endpoint = low if attachment['end'] == 'start' else high
                    attachment['end'] = ('start' if abs(endpoint-minimum) <= abs(endpoint-maximum)
                                         else 'end')
                if attachment not in source.setdefault('attachments', []):
                    source['attachments'].append(attachment)
            aliases[candidate['id']] = source['id']
            changed = True
        group['runs'] = retained
    if not changed:
        return assembly, aliases
    document.pop('results', None)
    document.pop('build_plan', None)
    return Assembly.from_doc(document, assembly.base, assembly.library,
                             validate_mirror_geometry=False), aliases


def _scope(assembly, subassembly=None, run_id=None, inferred=(), include_run_ids=()):
    groups = assembly.doc.get('draft_subassemblies', [])
    if run_id is None:
        selected = [group for group in groups if subassembly is None or group['id'] == subassembly]
    else:
        owner = next((group for group in groups if any(run['id'] == run_id for run in group['runs'])), None)
        if owner is None or subassembly is not None and owner['id'] != subassembly:
            raise DocumentError('Choose an existing draft run')
        graph = UnionFind([*assembly.parts, *(run['id'] for run in runs(assembly.doc))])
        for joint in assembly.joints:
            if joint_kind(joint) == 'fixed':
                graph.union(joint['a']['part'], joint['b']['part'])
        for run in runs(assembly.doc):
            for attachment in run.get('attachments', []):
                graph.union(run['id'], attachment['connector'])
        for candidate_run, attachment in inferred:
            graph.union(candidate_run, attachment['connector'])
        roots = {graph.find(run_id), *(graph.find(extra) for extra in include_run_ids)}
        selected = [{**group, 'runs': [run for run in group['runs'] if graph.find(run['id']) in roots]}
                    for group in groups]
        selected = [group for group in selected if group['runs']]
    if not selected:
        raise DocumentError('Choose an existing draft subassembly')
    return selected


def _definition(assembly, run, length):
    catalog = assembly.library.parts.get(run['catalog'])
    if catalog is None or catalog.get('kind') != 'member':
        raise DocumentError(f"{run['id']}: choose a member catalogue profile")
    if '$length_mm' not in str(catalog.get('geometry', [])):
        raise DocumentError(f"{run['id']}: draft members need length-parametric geometry")
    params = {**catalog.get('parameters', {}), **run.get('parameters', {}), 'length_mm': length}
    return substitute(catalog, params)


def _socket(assembly, attachment):
    connector = assembly.parts.get(attachment['connector'])
    if connector is None:
        raise DocumentError(f"Unknown draft connector {attachment['connector']}")
    socket = connector.ports.get(attachment['port'])
    if socket is None or socket.get('type') != 'socket':
        raise DocumentError(f"{connector.id}/{attachment['port']} is not a socket")
    mouth, axis = connector.frame({'port': attachment['port']})
    return connector, socket, mouth, unit(axis)


def _check_attachment(assembly, run, attachment, other_runs, *, allow_existing_pair=False):
    connector, socket, _, _ = _socket(assembly, attachment)
    section = _definition(assembly, run, max(1., float(np.linalg.norm(vec(run['end_mm'])-vec(run['start_mm']))))).get('section', {})
    profile = 'round' if section.get('type') in ('tube', 'round', 'circle') else section.get('profile', section.get('type'))
    if profile != socket.get('profile', 'round') or abs(section.get('diameter_mm', 0)-socket.get('diameter_mm', 0)) > .6:
        raise DocumentError('The pipe size or profile does not match this socket')
    if socket.get('through'):
        if 'end' in attachment or 'insertion_mm' in attachment:
            raise DocumentError('A through socket uses a station, not a pipe end')
    else:
        if attachment.get('end') not in ('start', 'end'):
            raise DocumentError('Choose the start or end of the pipe')
        depth = attachment.get('insertion_mm', min(30., socket['engagement_mm']*.8))
        if not socket.get('min_engagement_mm', 0) <= depth <= socket['engagement_mm']:
            raise DocumentError('Insertion is outside the socket engagement range')
        if any(a.get('end') == attachment['end'] for a in run.get('attachments', []) if a is not attachment):
            raise DocumentError(f"{run['id']} {attachment['end']} end is already connected")
    from .connections import socket_blockers
    if socket_blockers(assembly, connector.id, attachment['port']):
        raise DocumentError('This socket or its shared bore is occupied')
    for other in other_runs:
        for used in other.get('attachments', []):
            if used is attachment or used['connector'] != connector.id:
                continue
            if other is run and not allow_existing_pair:
                raise DocumentError(f"{run['id']} is already attached to {connector.id}; one pipe cannot occupy two sockets of the same connector")
            port = connector.ports[used['port']]
            if used['port'] == attachment['port'] or used['port'] in socket.get('excludes', []) or attachment['port'] in port.get('excludes', []):
                raise DocumentError('This socket or its shared bore is occupied by a draft run')


def connect(assembly, run_id, connector, port, end='start', insertion_mm=None, at_mm=None):
    """Record an intent without solving poses, collisions, or cut lengths."""
    doc = copy.deepcopy(assembly.doc)
    all_runs = runs(doc)
    run = next((r for r in all_runs if r['id'] == run_id), None)
    if run is None:
        raise DocumentError(f'Unknown draft run {run_id}')
    item = {'connector': connector, 'port': port}
    socket = _socket(assembly, item)[1]
    if not socket.get('through'):
        item['end'] = end
        item['insertion_mm'] = float(min(30., socket['engagement_mm']*.8) if insertion_mm is None else insertion_mm)
    _check_attachment(assembly, run, item, all_runs)
    run.setdefault('attachments', []).append(item)
    from .symmetry import fit_new_centered_end
    owner = next(group for group in doc['draft_subassemblies'] if run in group['runs'])
    fit_new_centered_end(assembly, owner, run, item)
    doc.pop('results', None); doc.pop('build_plan', None)
    return doc


def _layout(assembly, run):
    start, end = vec(run['start_mm']), vec(run['end_mm'])
    reference_length = float(np.linalg.norm(end-start))
    if reference_length <= 1e-6:
        raise DocumentError(f"{run['id']}: preview span must be positive")
    working_length = float(run.get('locked_length_mm', reference_length))
    attachments = run.get('attachments', [])
    bound = {}
    for attachment in attachments:
        connector, socket, mouth, axis = _socket(assembly, attachment)
        if not socket.get('through'):
            depth = attachment.get('insertion_mm', min(30., socket['engagement_mm']*.8))
            bound[attachment['end']] = (mouth-axis*depth, axis, attachment)
    if 'start' in bound and 'end' in bound:
        start, end = bound['start'][0], bound['end'][0]
    elif 'start' in bound:
        start = bound['start'][0]
        end = start + bound['start'][1]*working_length
    elif 'end' in bound:
        end = bound['end'][0]
        start = end + bound['end'][1]*working_length
    else:
        end = start + unit(end-start)*working_length
    conflicts = []
    seen_connectors = set()
    for attachment in attachments:
        connector = attachment['connector']
        if connector in seen_connectors:
            conflicts.append({'run': run['id'], 'connector': connector,
                              'code': 'DUPLICATE_CONNECTOR',
                              'message': f'{run["id"]} occupies two sockets of {connector}; detach one connection'})
        seen_connectors.add(connector)
    length = float(np.linalg.norm(end-start))
    if length <= 1e-6:
        start = bound['start'][0]
        end = start + unit(vec(run['end_mm'])-vec(run['start_mm']))*reference_length
        length = reference_length
        conflicts.append({'run': run['id'], 'code': 'ZERO_SPAN', 'residual_mm': round(reference_length, 3),
                          'message': 'Both pipe ends target the same point; move a connector or detach an end'})
    direction = unit(end-start)
    if 'start' in bound and 'end' in bound:
        span = end-start
        axis = bound['start'][1]
        lateral = float(np.linalg.norm(span-axis*(span@axis)))
        if lateral > 1.:
            conflicts.append({'run': run['id'], 'code': 'POSITION_MISMATCH', 'residual_mm': round(lateral, 3),
                              'message': f"End socket lies {lateral:.1f} mm off the start socket axis"})
    if 'start' in bound:
        angle = float(np.degrees(np.arccos(np.clip(direction @ bound['start'][1], -1, 1))))
        if angle > 2:
            conflicts.append({'run': run['id'], 'connector': bound['start'][2]['connector'], 'port': bound['start'][2]['port'], 'code': 'AXIS_MISMATCH', 'residual_deg': round(angle, 3), 'message': f"Start socket points {angle:.1f}° away from the run"})
    if 'end' in bound:
        angle = float(np.degrees(np.arccos(np.clip(-direction @ bound['end'][1], -1, 1))))
        if angle > 2:
            conflicts.append({'run': run['id'], 'connector': bound['end'][2]['connector'], 'port': bound['end'][2]['port'], 'code': 'AXIS_MISMATCH', 'residual_deg': round(angle, 3), 'message': f"End socket points {angle:.1f}° away from the run"})
    if 'start' in bound and 'end' in bound and run.get('locked_length_mm') is not None and abs(length-run['locked_length_mm']) > .05:
        conflicts.append({'run': run['id'], 'code': 'LOCKED_LENGTH', 'residual_mm': round(length-run['locked_length_mm'], 3), 'message': 'Locked cut length disagrees with the connector span'})
    through = []
    for attachment in attachments:
        connector, socket, mouth, axis = _socket(assembly, attachment)
        if not socket.get('through'):
            continue
        # A draft through-fit follows the fitting's current position. Its exact
        # station is only fixed when the run is finalized into a socket joint.
        station = float((mouth-start) @ direction)
        gap = float(np.linalg.norm(mouth-(start+direction*station)))
        angular = float(np.degrees(np.arccos(np.clip(abs(direction @ axis), -1, 1))))
        half = socket.get('engagement_mm', 0)/2
        short = max(half-station, station-(length-half), 0.)
        if gap > 1 or angular > THROUGH_AXIS_TOLERANCE_DEG or short > THROUGH_ENGAGEMENT_ROUNDOFF_MM:
            reason = (f'Through socket needs {short:.3f} mm more pipe engagement' if short > THROUGH_ENGAGEMENT_ROUNDOFF_MM else
                      f'Through socket axis differs by {angular:.3f}Â°' if angular > THROUGH_AXIS_TOLERANCE_DEG else
                      f'Through socket misses the pipe centreline by {gap:.3f} mm')
            conflicts.append({'run': run['id'], 'connector': connector.id, 'port': attachment['port'], 'code': 'THROUGH_FIT',
                              'residual_mm': round(max(gap, short), 3),
                              'radial_gap_mm': round(gap, 3),
                              'radial_gap_exact_mm': gap,
                              'engagement_short_mm': round(short, 3),
                              'residual_deg': round(angular, 3),
                              'axis_error_exact_deg': angular, 'message': reason})
        through.append((attachment, station))
    matrix = np.eye(4)
    matrix[:3, :3] = axis_frame(direction) @ Rotation.from_euler('z', run.get('roll_deg', 0), degrees=True).as_matrix()
    matrix[:3, 3] = (start+end)/2
    return {'start': start, 'end': end, 'length': length, 'pose': pose_of(matrix), 'through': through, 'conflicts': conflicts}


def _inferred_through(assembly, allow_near=False):
    """Find unused through sockets occupied, or nearly occupied, by draft runs.

    Repair may claim a unique near fit and then solve its pose. Finalization only
    infers already aligned fits, so it never silently changes an unfinished draft.
    """
    all_runs = runs(assembly.doc)
    by_socket = {}
    near_by_socket = {}
    near = []
    for run in all_runs:
        layout = _layout(assembly, run)
        start, end = layout['start'], layout['end']
        direction = unit(end-start)
        section = _definition(assembly, run, layout['length']).get('section', {})
        profile = 'round' if section.get('type') in ('tube', 'round', 'circle') else section.get('profile', section.get('type'))
        for fitting in assembly.parts.values():
            for port, socket in fitting.ports.items():
                if not socket.get('through') or socket.get('type') != 'socket':
                    continue
                if profile != socket.get('profile', 'round') or abs(section.get('diameter_mm', 0)-socket.get('diameter_mm', 0)) > .6:
                    continue
                attachment = {'connector': fitting.id, 'port': port}
                try:
                    _check_attachment(assembly, run, attachment, all_runs)
                except DocumentError:
                    continue
                _, _, mouth, axis = _socket(assembly, attachment)
                station = float((mouth-start) @ direction)
                half = socket.get('engagement_mm', 0)/2
                gap = float(np.linalg.norm(mouth-(start+direction*station)))
                angle = float(np.degrees(np.arccos(np.clip(abs(direction@axis), -1, 1))))
                if not 0 <= station <= layout['length']:
                    continue
                if (gap <= 1 and angle <= THROUGH_AXIS_TOLERANCE_DEG and
                        half-THROUGH_INFER_ALLOWANCE_MM <= station <= layout['length']-half+THROUGH_INFER_ALLOWANCE_MM):
                    by_socket.setdefault((fitting.id, port), []).append((run['id'], attachment))
                elif gap <= min(10., socket.get('diameter_mm', 0)/4) and angle <= 5:
                    near_by_socket.setdefault((fitting.id, port), []).append((run['id'], attachment))
                    near.append({'run': run['id'], 'connector': fitting.id, 'port': port,
                                 'code': 'UNATTACHED_THROUGH', 'residual_mm': round(gap, 3),
                                 'residual_deg': round(angle, 3),
                                 'message': f'{run["id"]} reaches {fitting.id}/{port} without a full draft through-fit; connect and align it or move it clear'})
    if allow_near:
        for socket, candidates in near_by_socket.items():
            if socket not in by_socket:
                by_socket[socket] = candidates
    unique = [candidates[0] for candidates in by_socket.values() if len(candidates) == 1]
    ambiguous = [{'code': 'AMBIGUOUS_THROUGH', 'connector': connector, 'port': port,
                  'runs': [run_id for run_id, _ in candidates],
                  'message': f'{connector}/{port} aligns with more than one draft pipe; choose its intended connection'}
                 for (connector, port), candidates in by_socket.items() if len(candidates) > 1]
    by_pair = {}
    for run_id, attachment in unique:
        by_pair.setdefault((run_id, attachment['connector']), []).append(attachment['port'])
    for (run_id, connector), ports in by_pair.items():
        if len(ports) > 1:
            ambiguous.append({'code': 'AMBIGUOUS_CONNECTOR', 'connector': connector,
                              'runs': [run_id], 'ports': ports,
                              'message': f'{run_id} aligns with multiple sockets of {connector}; choose one socket'})
    unique = [(run_id, attachment) for run_id, attachment in unique
              if len(by_pair[run_id, attachment['connector']]) == 1]
    return unique, ambiguous, near


def preview(assembly):
    """Small visual proxies; the resolved Assembly remains exact-only."""
    from .symmetry import mirror_run_conflict
    result = []
    planes = {run['id']: group.get('mirrors', []) for group in assembly.doc.get('draft_subassemblies', [])
              for run in group['runs']}
    for run in runs(assembly.doc):
        layout = _layout(assembly, run)
        conflicts = list(layout['conflicts'])
        for plane in planes.get(run['id'], []):
            conflict = mirror_run_conflict(layout, plane, plane.get('run_modes', {}).get(run['id'], 'free'))
            if conflict:
                conflicts.append({'run': run['id'], 'plane': plane['id'], **conflict})
        definition = _definition(assembly, run, layout['length'])
        result.append({'id': run['id'], 'label': run['id']+' · draft', 'catalog': run['catalog'], 'kind': 'member',
                       'draft': True, 'pose': layout['pose'], 'geometry': definition['geometry'],
                       'ports': {}, 'section': definition.get('section', {}), 'length_mm': layout['length'],
                       'mass_kg': 0., 'color': '#5ba9b5' if not conflicts else '#cf815d',
                       'conflicts': conflicts, 'attachments': run.get('attachments', [])})
    return result


def _mirror_conflicts(assembly, selected):
    from .symmetry import mirror_run_conflict

    conflicts = []
    for group in selected:
        for run in group['runs']:
            layout = _layout(assembly, run)
            for plane in group.get('mirrors', []):
                conflict = mirror_run_conflict(layout, plane,
                    plane.get('run_modes', {}).get(run['id'], 'free'))
                if conflict:
                    conflicts.append({'run': run['id'], 'plane': plane['id'], **conflict})
    return conflicts


def _align_free_mirror_runs(assembly, selected):
    """Project unconstrained draft spans onto their requested mirror geometry.

    The raw endpoints are the whole pose for an unattached run. A one-ended,
    unlocked centered run can instead change its working length around the
    socket without moving that connector.
    """
    from .symmetry import AXES, mirror_run_conflict

    document = copy.deepcopy(assembly.doc)
    selected_ids = {run['id'] for group in selected for run in group['runs']}
    moved = set()
    for group in document.get('draft_subassemblies', []):
        for run in group['runs']:
            if run['id'] not in selected_ids:
                continue
            planes = [(plane, plane.get('run_modes', {}).get(run['id'], 'free'))
                      for plane in group.get('mirrors', [])]
            planes = [(plane, mode) for plane, mode in planes if mode != 'free' and
                      mirror_run_conflict(_layout(assembly, run), plane, mode)]
            if not planes:
                continue
            attachments = run.get('attachments', [])
            bound = [item for item in attachments if item.get('end') in ('start', 'end')]
            if not attachments:
                centered = [plane for plane, mode in planes if mode == 'centered']
                all_centered = [plane for plane in group.get('mirrors', [])
                                if plane.get('run_modes', {}).get(run['id']) == 'centered']
                if len(all_centered) > 1:
                    continue
                in_plane = [plane for plane in group.get('mirrors', [])
                            if plane.get('run_modes', {}).get(run['id']) == 'in_plane']
                start, end = vec(run['start_mm']), vec(run['end_mm'])
                direction = end-start
                length = float(run.get('locked_length_mm', np.linalg.norm(direction)))
                midpoint = (start+end)/2
                for plane in in_plane:
                    axis = AXES[plane['axis']]
                    midpoint[axis] = plane['offset_mm']
                    direction[axis] = 0
                if all_centered:
                    plane = all_centered[0]
                    axis = AXES[plane['axis']]
                    sign = 1 if end[axis] >= start[axis] else -1
                    midpoint[axis] = plane['offset_mm']
                    direction = np.zeros(3)
                    direction[axis] = sign
                elif np.linalg.norm(direction) <= 1e-9:
                    available = next((axis for axis in range(3)
                                      if all(AXES[plane['axis']] != axis for plane in in_plane)), None)
                    if available is None:
                        continue
                    direction[available] = 1
                direction = unit(direction)
                run['start_mm'] = (midpoint-direction*length/2).tolist()
                run['end_mm'] = (midpoint+direction*length/2).tolist()
                moved.add(run['id'])
            elif len(bound) == 1 and run.get('locked_length_mm') is None:
                attachment = bound[0]
                _, socket, mouth, axis_direction = _socket(assembly, attachment)
                depth = attachment.get('insertion_mm', min(30., socket['engagement_mm']*.8))
                point = mouth-axis_direction*depth
                for plane, mode in planes:
                    if mode != 'centered':
                        continue
                    axis = AXES[plane['axis']]
                    component = axis_direction[axis]
                    if abs(component) < .99999:
                        continue
                    length = 2*(plane['offset_mm']-point[axis])/component
                    if length <= 1e-6:
                        continue
                    other = point+axis_direction*length
                    start, end = ((point, other) if attachment['end'] == 'start'
                                  else (other, point))
                    run['start_mm'] = start.tolist()
                    run['end_mm'] = end.tolist()
                    moved.add(run['id'])
    if not moved:
        return assembly, set()
    return Assembly.from_doc(document, assembly.base, assembly.library,
                             validate_mirror_geometry=False), moved


def validate_drafts(assembly, *, validate_mirror_geometry=True):
    """Reject broken graph references while allowing geometric closure residuals."""
    groups = assembly.doc.get('draft_subassemblies', [])
    group_ids = [group['id'] for group in groups]
    if len(group_ids) != len(set(group_ids)):
        raise DocumentError('Duplicate draft subassembly id')
    all_runs = runs(assembly.doc)
    ids = [run['id'] for run in all_runs]
    if len(ids) != len(set(ids)) or set(ids).intersection(assembly.parts):
        raise DocumentError('Draft run ids must be unique and distinct from finished parts')
    for run in all_runs:
        span = float(np.linalg.norm(vec(run['end_mm'])-vec(run['start_mm'])))
        if span <= 1e-6:
            raise DocumentError(f"{run['id']}: preview span must be positive")
        _definition(assembly, run, span)
        for attachment in run.get('attachments', []):
            _check_attachment(assembly, run, attachment, all_runs, allow_existing_pair=True)
    if any(group.get('mirrors') for group in groups):
        from .symmetry import validate_mirrors
        validate_mirrors(assembly,geometry=validate_mirror_geometry)


def _relax(assembly, selected, cancelled=None):
    """Solve free exact components and through-only draft runs together.

    Existing joints and anchors are hard boundaries. Each free component moves
    together, so its finished joints retain their original geometry. Through-only
    runs have no end socket to derive their centreline; their preview pose is
    therefore a solve variable, while their working length stays fixed.
    """
    from scipy.optimize import least_squares
    from scipy.sparse import lil_matrix
    from scipy.spatial.transform import Rotation
    from .symmetry import AXES

    if (not any(_layout(assembly, run)['conflicts'] for group in selected for run in group['runs'])
            and not _mirror_conflicts(assembly, selected)):
        return assembly, {}, {}, set()
    _check_cancelled(cancelled)

    graph = UnionFind(assembly.parts)
    for joint in assembly.joints:
        graph.union(joint['a']['part'], joint['b']['part'])
    requested = {a['connector'] for group in selected for run in group['runs'] for a in run.get('attachments', [])}
    selected_ids = {run['id'] for group in selected for run in group['runs']}
    held = {a['connector'] for run in runs(assembly.doc) if run['id'] not in selected_ids
            for a in run.get('attachments', [])}
    anchored = {a['part'] for a in assembly.anchors}
    direct = {p['id'] for p in assembly.doc.get('parts', [])}
    movable = [group for group in graph.groups() if requested.intersection(group) and
               not anchored.intersection(group) and not held.intersection(group) and set(group) <= direct]
    index = {pid: i for i, group in enumerate(movable) for pid in group}
    connector_usage = {}
    through_only_ids = set()
    for group in selected:
        for run in group['runs']:
            if run.get('attachments') and all(a.get('end') not in ('start', 'end')
                                              for a in run['attachments']):
                through_only_ids.add(run['id'])
            for attachment in run.get('attachments', []):
                connector_usage.setdefault(attachment['connector'], set()).add(run['id'])
    mirror_runs = {conflict['run'] for conflict in _mirror_conflicts(assembly, selected)}
    free_runs = [run for group in selected for run in group['runs']
                 if run.get('attachments') and
                 (run['id'] in mirror_runs or
                  any(c['code'] == 'THROUGH_FIT' and
                      (c['radial_gap_exact_mm'] > 1 or
                       c['axis_error_exact_deg'] > THROUGH_AXIS_TOLERANCE_DEG)
                      for c in _layout(assembly, run)['conflicts'])) and
                 not any(a.get('end') in ('start', 'end') for a in run['attachments']) and
                 (len(run['attachments']) > 1 or
                  any(len(connector_usage[a['connector']] & through_only_ids) > 1
                      for a in run['attachments']) or
                  all(a['connector'] not in index for a in run['attachments']))]
    if not movable and not free_runs:
        return assembly, {}, {}, set()
    run_index = {run['id']: len(movable)+i for i, run in enumerate(free_runs)}
    solved_runs = {run['id']: copy.deepcopy(run) for group in selected for run in group['runs']}
    run_endpoints = {run['id']: (vec(run['start_mm']), vec(run['end_mm'])) for run in free_runs}
    working = copy.copy(assembly)
    working.parts = {pid: copy.copy(part) for pid, part in assembly.parts.items()}
    original = {pid: part.matrix.copy() for pid, part in assembly.parts.items()}
    pivots = [np.mean([original[pid][:3, 3] for pid in group], axis=0) for group in movable]
    specs = []
    for group in selected:
        for original_run in group['runs']:
            run = solved_runs[original_run['id']]
            attachments = run.get('attachments', [])
            ends = {a['end']: a for a in attachments if a.get('end') in ('start', 'end')}
            dependencies = {index[a['connector']] for a in attachments if a['connector'] in index}
            if run['id'] in run_index:
                dependencies.add(run_index[run['id']])
            if len(ends) == 1:
                attachment = next(iter(ends.values()))
                for plane in group.get('mirrors', []):
                    if plane.get('run_modes', {}).get(run['id']) != 'centered':
                        continue
                    axis = AXES[plane['axis']]
                    sign = 1 if run['end_mm'][axis] >= run['start_mm'][axis] else -1
                    expected = sign if attachment['end'] == 'start' else -sign
                    if attachment['connector'] in index:
                        specs.append((run, 'mirror_axis', (attachment, axis, expected),
                                      {index[attachment['connector']]}, 3))
            for plane in group.get('mirrors', []):
                mode = plane.get('run_modes', {}).get(run['id'])
                if mode == 'centered' and run['id'] in run_index:
                    specs.append((run, 'mirror_center', (AXES[plane['axis']], plane['offset_mm'],
                                                        1 if run['end_mm'][AXES[plane['axis']]] >=
                                                        run['start_mm'][AXES[plane['axis']]] else -1),
                                  dependencies, 4))
                if mode == 'in_plane' and dependencies:
                    specs.append((run, 'mirror_plane', (AXES[plane['axis']], plane['offset_mm']),
                                  dependencies, 2))
            if 'start' in ends and 'end' in ends:
                specs.append((run, 'ends', None, dependencies, 9))
            for attachment in attachments:
                if 'end' not in attachment:
                    specs.append((run, 'through', attachment, dependencies, 6))
            if run.get('locked_length_mm') is not None and attachments:
                specs.append((run, 'length', None, dependencies, 1))
    if not specs:
        return assembly, {}, {}, set()
    count = 6*(len(movable)+len(free_runs))
    rows = sum(spec[4] for spec in specs)+count
    pattern = lil_matrix((rows, count), dtype=int)
    row = 0
    for _, _, _, dependencies, size in specs:
        for dependency in dependencies:
            pattern[row:row+size, 6*dependency:6*dependency+6] = 1
        row += size
    pattern[row:, :] = np.eye(count, dtype=int)

    def residual(x):
        _check_cancelled(cancelled)
        for i, group in enumerate(movable):
            delta = x[6*i:6*i+3]
            rotation = Rotation.from_rotvec(x[6*i+3:6*i+6]).as_matrix()
            pivot = pivots[i]
            for pid in group:
                matrix = original[pid].copy()
                matrix[:3, :3] = rotation @ matrix[:3, :3]
                matrix[:3, 3] = pivot + rotation @ (matrix[:3, 3]-pivot) + delta
                working.parts[pid].matrix = matrix
        for run in free_runs:
            variable = run_index[run['id']]
            delta = x[6*variable:6*variable+3]
            rotation = Rotation.from_rotvec(x[6*variable+3:6*variable+6]).as_matrix()
            start, end = run_endpoints[run['id']]
            pivot = (start+end)/2
            solved_runs[run['id']]['start_mm'] = (pivot+rotation@(start-pivot)+delta).tolist()
            solved_runs[run['id']]['end_mm'] = (pivot+rotation@(end-pivot)+delta).tolist()
        errors = []
        for run, kind, attachment, _, size in specs:
            try:
                layout = _layout(working, run)
                start, end = layout['start'], layout['end']
                direction = unit(end-start)
                if kind == 'mirror_axis':
                    item, axis_index, expected = attachment
                    socket_axis = _socket(working, item)[3]
                    target = np.zeros(3); target[axis_index] = expected
                    errors.extend((socket_axis-target)*500)
                elif kind == 'mirror_plane':
                    axis_index, offset = attachment
                    errors.extend([(start[axis_index]-offset)*10, (end[axis_index]-offset)*10])
                elif kind == 'mirror_center':
                    axis_index, offset, sign = attachment
                    target = np.zeros(3); target[axis_index] = sign
                    errors.append(((start[axis_index]+end[axis_index])/2-offset)*10)
                    errors.extend((direction-target)*100)
                elif kind == 'ends':
                    first = next(a for a in run['attachments'] if a.get('end') == 'start')
                    last = next(a for a in run['attachments'] if a.get('end') == 'end')
                    axis = _socket(working, first)[3]
                    end_axis = _socket(working, last)[3]
                    span = end-start
                    errors.extend(span-axis*(span@axis))
                    errors.extend((axis-direction)*100)
                    errors.extend((end_axis+direction)*100)
                elif kind == 'through':
                    _, socket, mouth, axis = _socket(working, attachment)
                    projected = float((mouth-start)@direction)
                    half = socket.get('engagement_mm', 0)/2
                    margin = min(THROUGH_REPAIR_MARGIN_MM, max(0., (layout['length']-2*half)/4))
                    station = float(np.clip(projected, half+margin, layout['length']-half-margin))
                    errors.extend(mouth-(start+direction*station))
                    sign = 1 if axis@direction >= 0 else -1
                    errors.extend((axis-sign*direction)*100)
                else:
                    errors.append(layout['length']-run['locked_length_mm'])
            except (DocumentError, ValueError):
                errors.extend([1e6]*size)
        for i in range(len(movable)+len(free_runs)):
            errors.extend(x[6*i:6*i+3]*1e-3)
            errors.extend(x[6*i+3:6*i+6]*.3)
        return np.asarray(errors, dtype=float)

    solution = least_squares(residual, np.zeros(count), jac_sparsity=pattern.tocsr(),
                             tr_solver='lsmr', max_nfev=80, ftol=1e-8, xtol=1e-8, gtol=1e-8)
    residual(solution.x)
    changed = {pid for i, group in enumerate(movable) for pid in group if
               np.linalg.norm(solution.x[6*i:6*i+6]) > 1e-5}
    changed_runs = {run['id'] for run in free_runs if
                    np.linalg.norm(solution.x[6*run_index[run['id']]:6*run_index[run['id']]+6]) > 1e-5}
    if not changed and not changed_runs:
        return assembly, {}, {}, set()
    movement = {pid: float(np.linalg.norm(working.parts[pid].matrix[:3, 3]-original[pid][:3, 3]))
                for pid in changed}
    rotations = {pid: float(np.degrees(np.linalg.norm(solution.x[6*index[pid]+3:6*index[pid]+6])))
                 for pid in changed}
    document = copy.deepcopy(assembly.doc)
    for part in document.get('parts', []):
        if part['id'] in changed:
            part['pose'] = pose_of(working.parts[part['id']].matrix)
    for run in runs(document):
        if run['id'] in changed_runs:
            run['start_mm'] = solved_runs[run['id']]['start_mm']
            run['end_mm'] = solved_runs[run['id']]['end_mm']
    if document.get('draft_subassemblies'):
        from .symmetry import fit_moved_centered_runs
        fit_moved_centered_runs(working, document, changed)
    return Assembly.from_doc(document, assembly.base, assembly.library,
                             validate_mirror_geometry=False), movement, rotations, changed_runs


def _extend_unlocked_through_spans(assembly, selected):
    """Extend free draft ends until every aligned through fitting has full support."""
    selected_ids = {run['id'] for group in selected for run in group['runs']}
    document = copy.deepcopy(assembly.doc)
    resized = {}
    for group in document.get('draft_subassemblies', []):
        for run in group['runs']:
            if run['id'] not in selected_ids or run.get('locked_length_mm') is not None:
                continue
            attachments = run.get('attachments', [])
            bound_ends = {a['end'] for a in attachments if a.get('end') in ('start', 'end')}
            if not attachments or len(bound_ends) == 2:
                continue
            layout = _layout(assembly, run)
            start, end, length = layout['start'], layout['end'], layout['length']
            direction = unit(end-start)
            before, after = 0., 0.
            for attachment in attachments:
                _, socket, mouth, axis = _socket(assembly, attachment)
                if not socket.get('through'):
                    continue
                station = float((mouth-start)@direction)
                gap = float(np.linalg.norm(mouth-(start+direction*station)))
                angular = float(np.degrees(np.arccos(np.clip(abs(direction@axis), -1, 1))))
                if gap > 1 or angular > THROUGH_AXIS_TOLERANCE_DEG:
                    continue
                half = socket.get('engagement_mm', 0)/2
                if station < half+THROUGH_REPAIR_MARGIN_MM:
                    before = max(before, half+THROUGH_REPAIR_MARGIN_MM-station)
                if station > length-half-THROUGH_REPAIR_MARGIN_MM:
                    after = max(after, station+half+THROUGH_REPAIR_MARGIN_MM-length)
            if before <= 0 and after <= 0:
                continue
            centered = any(plane.get('run_modes', {}).get(run['id']) == 'centered'
                           for plane in group.get('mirrors', []))
            if centered and not bound_ends:
                before = after = max(before, after)
            if 'start' in bound_ends:
                before = 0.
            if 'end' in bound_ends:
                after = 0.
            if before <= 0 and after <= 0:
                continue
            run['start_mm'] = (start-direction*before).tolist()
            run['end_mm'] = (end+direction*after).tolist()
            resized[run['id']] = before+after
    if not resized:
        return assembly, {}
    return Assembly.from_doc(document, assembly.base, assembly.library,
                             validate_mirror_geometry=False), resized


def _close_dangling_chain_gaps(assembly):
    """Rejoin a free chain component displaced by an older connector edit.

    A spherical joint constrains its frame origins. Removing that one joint
    must separate the chain and its free payload from the host; translating the
    whole separated component then closes the gap without changing any other
    joint or draft run. Anchors, cycles, and partial object moves stay untouched.
    """
    prefixes=tuple(o['id']+'/' for o in assembly.doc.get('objects',[])
                   if o.get('template')=='chain')
    if not prefixes:return assembly,{},[]
    moved={};closed=[]
    joint_ids=[j['id'] for j in assembly.joints]
    for joint_id in joint_ids:
        joint=next(j for j in assembly.joints if j['id']==joint_id)
        if joint_kind(joint)!='spherical':continue
        ends=[joint[e]['part'] for e in ('a','b')]
        chain=[pid for pid in ends if pid.startswith(prefixes)]
        if len(chain)!=1:continue
        chain_part=chain[0];host=ends[1] if ends[0]==chain_part else ends[0]
        pa,pb,_=assembly.joint_frames(joint)
        gap=float(np.linalg.norm(pb-pa))
        if gap<=1.1:continue
        adjacency={pid:set() for pid in assembly.parts}
        for other in assembly.joints:
            if other['id']==joint_id:continue
            a,b=(other[e]['part'] for e in ('a','b'))
            adjacency[a].add(b);adjacency[b].add(a)
        component={chain_part};queue=[chain_part]
        for pid in queue:
            for neighbor in adjacency[pid]-component:
                component.add(neighbor);queue.append(neighbor)
        if host in component or any(a['part'] in component for a in assembly.anchors):continue
        if any(a['connector'] in component for run in runs(assembly.doc)
               for a in run.get('attachments',[])):continue
        direct={p['id'] for p in assembly.doc.get('parts',[])}
        object_members={o['id']:{pid for pid in assembly.parts
                                  if pid.startswith(o['id']+'/') and pid not in direct}
                        for o in assembly.doc.get('objects',[])}
        if any(members&component and not members<=component
               for members in object_members.values()):continue
        delta=(pa-pb) if chain_part==ends[1] else (pb-pa)
        document=copy.deepcopy(assembly.doc)
        for part in document.get('parts',[]):
            if part['id'] in component:
                matrix=assembly.parts[part['id']].matrix.copy()
                matrix[:3,3]+=delta
                part['pose']=pose_of(matrix)
        for obj in document.get('objects',[]):
            if object_members[obj['id']] and object_members[obj['id']]<=component:
                matrix=transform(obj.get('pose'))
                matrix[:3,3]+=delta
                obj['pose']=pose_of(matrix)
        try:
            candidate=Assembly.from_doc(document,assembly.base,assembly.library,
                                        validate_mirror_geometry=False)
        except DocumentError:
            continue
        def joint_gap(model,item):
            a,b,_=model.joint_frames(item)
            return float(np.linalg.norm(b-a))
        previous={j['id']:joint_gap(assembly,j) for j in assembly.joints}
        actual={j['id']:joint_gap(candidate,j) for j in candidate.joints}
        if (actual[joint_id]>1.1 or
                any(value>max(1.1,previous[jid])+1e-3 for jid,value in actual.items())):
            continue
        moved.update({pid:float(np.linalg.norm(delta)) for pid in component})
        closed.append(joint_id)
        assembly=candidate
    return assembly,moved,closed


def repair(assembly, subassembly=None, cancelled=None, run_id=None):
    """Close draft fit residuals without materializing runs or joints."""
    inferred, ambiguous, _ = _inferred_through(assembly, allow_near=True)
    possible = [(candidate_run, {'connector': conflict['connector'], 'port': conflict['port']})
                for conflict in ambiguous if 'port' in conflict for candidate_run in conflict['runs']]
    groups = _scope(assembly, subassembly, run_id, [*inferred, *possible])
    selected = {run['id'] for group in groups for run in group['runs']}
    conflicts = [conflict for conflict in ambiguous if selected.intersection(conflict['runs'])]
    conflicts.extend(conflict for group in groups for run in group['runs']
                     for conflict in _layout(assembly, run)['conflicts']
                     if conflict['code'] == 'DUPLICATE_CONNECTOR')
    if conflicts:
        return {'status': 'conflict', 'conflicts': conflicts}
    inferred = [(candidate_run, attachment) for candidate_run, attachment in inferred if candidate_run in selected]
    original_lengths = {run['id']: _layout(assembly, run)['length']
                        for group in groups for run in group['runs']}
    _check_cancelled(cancelled)
    if inferred:
        document = copy.deepcopy(assembly.doc)
        for candidate_run, attachment in inferred:
            run = next(run for run in runs(document) if run['id'] == candidate_run)
            run.setdefault('attachments', []).append(attachment)
        assembly = Assembly.from_doc(document, assembly.base, assembly.library,
                                     validate_mirror_geometry=False)
        groups = _scope(assembly, subassembly, run_id)
    assembly, _ = _extend_unlocked_through_spans(assembly, groups)
    groups = _scope(assembly, subassembly, run_id)
    assembly, aligned_runs = _align_free_mirror_runs(assembly, groups)
    groups = _scope(assembly, subassembly, run_id)
    repaired, moved, rotated, moved_runs = _relax(assembly, groups, cancelled)
    moved_runs |= aligned_runs
    _check_cancelled(cancelled)
    repaired, _ = _extend_unlocked_through_spans(repaired, groups)
    repaired, chain_moved, closed_chain_joints = _close_dangling_chain_gaps(repaired)
    groups = _scope(repaired, subassembly, run_id)
    conflicts = [conflict for group in groups for run in group['runs']
                 for conflict in _layout(repaired, run)['conflicts']]
    conflicts.extend(_mirror_conflicts(repaired, groups))
    if conflicts:
        return {'status': 'conflict', 'conflicts': conflicts}
    resized = {}
    for group in groups:
        for run in group['runs']:
            difference = _layout(repaired, run)['length']-original_lengths[run['id']]
            if abs(difference) > 1e-5:
                resized[run['id']] = difference
    if not inferred and not moved and not rotated and not moved_runs and not resized and not closed_chain_joints:
        return {'status': 'aligned'}
    document = copy.deepcopy(repaired.doc)
    document.pop('results', None); document.pop('build_plan', None)
    _check_cancelled(cancelled)
    return {'status': 'repaired', 'document': document,
            'moved_parts_mm': {**moved,**chain_moved}, 'rotated_parts_deg': rotated,
            'moved_runs': sorted(moved_runs),
            'resized_runs_mm': resized, 'inferred_through_connections': len(inferred),
            'rejoined_chain_attachments': closed_chain_joints}


def finalize(assembly, subassembly=None, check_collisions=True, cancelled=None, run_id=None, include_run_ids=()):
    """Materialize a draft group atomically or return all closure conflicts."""
    assembly, aliases = _coalesce_overlapping_mirror_runs(assembly)
    run_id = aliases.get(run_id, run_id)
    include_run_ids = tuple(aliases.get(identifier, identifier) for identifier in include_run_ids)
    inferred, ambiguous, near = _inferred_through(assembly)
    possible = [(candidate_run, {'connector': conflict['connector'], 'port': conflict['port']})
                for conflict in ambiguous if 'port' in conflict for candidate_run in conflict['runs']]
    groups = _scope(assembly, subassembly, run_id, [*inferred, *possible], include_run_ids)
    selected = {run['id'] for group in groups for run in group['runs']}
    conflicts = [conflict for conflict in ambiguous if selected.intersection(conflict['runs'])]
    conflicts.extend(conflict for conflict in near if conflict['run'] in selected)
    conflicts.extend(conflict for group in groups for run in group['runs']
                     for conflict in _layout(assembly, run)['conflicts']
                     if conflict['code'] == 'DUPLICATE_CONNECTOR')
    if conflicts:
        return {'status': 'conflict', 'conflicts': conflicts}
    inferred = [(candidate_run, attachment) for candidate_run, attachment in inferred if candidate_run in selected]
    if inferred:
        document = copy.deepcopy(assembly.doc)
        for candidate_run, attachment in inferred:
            run = next(run for run in runs(document) if run['id'] == candidate_run)
            try:
                _check_attachment(assembly, run, attachment, runs(document))
            except DocumentError as exc:
                return {'status': 'conflict', 'conflicts': [{'run': candidate_run, 'code': 'THROUGH_FIT', 'message': str(exc)}]}
            run.setdefault('attachments', []).append(attachment)
        assembly = Assembly.from_doc(document, assembly.base, assembly.library,
                                     validate_mirror_geometry=False)
        groups = _scope(assembly, subassembly, run_id, include_run_ids=include_run_ids)
    _check_cancelled(cancelled)
    assembly, _ = _extend_unlocked_through_spans(assembly, groups)
    groups = _scope(assembly, subassembly, run_id, include_run_ids=include_run_ids)
    assembly, _ = _align_free_mirror_runs(assembly, groups)
    groups = _scope(assembly, subassembly, run_id, include_run_ids=include_run_ids)
    assembly, moved, rotated, moved_runs = _relax(assembly, groups, cancelled)
    assembly, _ = _extend_unlocked_through_spans(assembly, groups)
    groups = _scope(assembly, subassembly, run_id, include_run_ids=include_run_ids)
    document = copy.deepcopy(assembly.doc)
    conflicts = _mirror_conflicts(assembly, groups)
    for group in groups:
        for run in group['runs']:
            _check_cancelled(cancelled)
            try:
                for attachment in run.get('attachments', []):
                    _check_attachment(assembly, run, attachment, runs(assembly.doc))
                layout = _layout(assembly, run)
                conflicts.extend(layout['conflicts'])
                if layout['conflicts']:
                    continue
                length = layout['length']
                if run.get('locked_length_mm') is not None:
                    length = float(run['locked_length_mm'])
                params = {**run.get('parameters', {}), 'length_mm': round(length, 6)}
                document.setdefault('parts', []).append({'id': run['id'], 'catalog': run['catalog'], 'parameters': params, 'pose': layout['pose']})
                for attachment in run.get('attachments', []):
                    socket = _socket(assembly, attachment)[1]
                    through = bool(socket.get('through'))
                    station = next((s for a, s in layout['through'] if a is attachment), None)
                    endpoint = {'part': run['id'], **({'at_mm': round(station, 6)} if through else {'end': attachment['end']})}
                    jid = f"{attachment['connector']}-{attachment['port']}-{run['id']}"
                    document.setdefault('joints', []).append({'id': jid, 'type': 'socket', 'a': {'part': attachment['connector'], 'port': attachment['port']},
                                                               'b': endpoint, 'insertion_mm': 0. if through else attachment['insertion_mm'], 'locked': True})
            except (DocumentError, ValueError) as exc:
                conflicts.append({'run': run['id'], 'code': 'DRAFT_INVALID', 'message': str(exc)})
    if conflicts:
        return {'status': 'conflict', 'conflicts': conflicts}
    selected = {run['id'] for group in groups for run in group['runs']}
    for group in document['draft_subassemblies']:
        group['runs'] = [run for run in group['runs'] if run['id'] not in selected]
        remaining = {run['id'] for run in group['runs']}
        for plane in group.get('mirrors', []):
            plane['run_modes'] = {run_id: mode for run_id, mode in plane.get('run_modes', {}).items()
                                  if run_id in remaining}
    document['draft_subassemblies'] = [group for group in document['draft_subassemblies'] if group['runs']]
    if not document['draft_subassemblies']:
        del document['draft_subassemblies']
    document.pop('results', None); document.pop('build_plan', None)
    _check_cancelled(cancelled)
    finished = Assembly.from_doc(document, assembly.base, assembly.library,
                                 validate_mirror_geometry=False)
    finished, chain_moved, closed_chain_joints = _close_dangling_chain_gaps(finished)
    document=finished.doc
    from .validation import validate
    validation_doc = copy.deepcopy(document)
    validation_doc.pop('draft_subassemblies', None)
    report = validate(Assembly.from_doc(validation_doc, assembly.base, assembly.library),
                      collisions=check_collisions, cancelled=cancelled)
    _check_cancelled(cancelled)
    added = {r['id'] for g in groups for r in g['runs']}
    conflicts = [i for i in report['issues'] if i['severity'] == 'error']
    if conflicts:
        return {'status': 'conflict', 'conflicts': conflicts}
    return {'status': 'finalized', 'document': document, 'lengths_mm': {p: finished.parts[p].length for p in added},
            'moved_parts_mm': {**moved,**chain_moved}, 'rotated_parts_deg': rotated,
            'inferred_through_connections': len(inferred),
            'rejoined_chain_attachments': closed_chain_joints}


def reopen(assembly, members):
    """Turn selected finished, socket-connected catalogue pipes into draft runs."""
    if not isinstance(members, list) or not members or not all(isinstance(pid, str) for pid in members):
        raise DocumentError('Choose a finished pipe or Body to return to draft')
    if len(members) != len(set(members)) or not set(members) <= assembly.parts.keys():
        raise DocumentError('A selected part no longer exists')
    restored = _reopen_mirrored_assembly(assembly, members)
    if restored is not None:
        return restored
    direct = {part['id']: part for part in assembly.doc.get('parts', [])}
    selected = [pid for pid in members if assembly.parts[pid].kind == 'member']
    if not selected:
        raise DocumentError('The selected Body has no finished catalogue pipes')
    selected_set = set(selected)
    for pid in selected:
        part, spec = assembly.parts[pid], direct.get(pid)
        catalog = assembly.library.parts.get(spec.get('catalog')) if spec else None
        if not spec or not catalog or catalog.get('kind') != 'member' or '$length_mm' not in str(catalog.get('geometry', [])):
            raise DocumentError(f'{pid}: only directly editable, length-parametric catalogue pipes can return to draft')
        if set(spec) - {'id', 'catalog', 'parameters', 'pose'}:
            raise DocumentError(f'{pid}: this pipe has custom body or material settings that draft mode cannot preserve')
        if part.length <= 0:
            raise DocumentError(f'{pid}: pipe length must be positive')
    doc = copy.deepcopy(assembly.doc)
    doc.get('metadata', {}).pop('draft_mirror_reopen', None)
    removed_joints = set()
    attachments = {pid: [] for pid in selected}
    for joint in assembly.joints:
        ends = (joint['a'], joint['b'])
        touching = [end for end in ends if end['part'] in selected_set]
        if not touching:
            continue
        if len(touching) != 1 or joint['type'] != 'socket' or not joint.get('locked'):
            raise DocumentError(f"{joint['id']}: draft mode cannot represent this pipe joint")
        member_end = touching[0]
        connector_end = ends[1] if ends[0] is member_end else ends[0]
        connector = assembly.parts[connector_end['part']]
        socket = connector.ports.get(connector_end.get('port'))
        if socket is None or socket.get('type') != 'socket':
            raise DocumentError(f"{joint['id']}: draft mode needs a connector socket")
        attachment = {'connector': connector.id, 'port': connector_end['port']}
        if socket.get('through'):
            if 'at_mm' not in member_end:
                raise DocumentError(f"{joint['id']}: through socket needs a pipe station")
            # The exact joint's station was needed to validate the source,
            # but draft stations are inferred from the fitting pose.
        else:
            if member_end.get('end') not in ('start', 'end'):
                raise DocumentError(f"{joint['id']}: socket needs a pipe end")
            attachment.update(end=member_end['end'], insertion_mm=joint.get('insertion_mm', 0))
        attachments[member_end['part']].append(attachment)
        removed_joints.add(joint['id'])
    for key in ('anchors', 'loads'):
        if any(item['part'] in selected_set for item in doc.get(key, [])):
            raise DocumentError(f'Attached {key} must be removed before returning these pipes to draft')
    if set(doc.get('build', {}).get('sequence', [])) & selected_set or any(
            item['part'] in selected_set for item in doc.get('build', {}).get('fixtures', [])):
        raise DocumentError('Build references must be removed before returning these pipes to draft')
    if any(item.get('seat') in selected_set for item in doc.get('tests', [])) or any(
            item.get('reference_part') in selected_set for item in doc.get('expanded_objects', [])):
        raise DocumentError('Object or test references must be removed before returning these pipes to draft')
    if set(doc.get('state', {}).get('joints', {})) & removed_joints or any(
            track['joint'] in removed_joints for track in doc.get('animation', {}).get('tracks', [])) or any(
            drive['driver'] in removed_joints or drive['follower'] in removed_joints for drive in doc.get('drives', [])):
        raise DocumentError('Joint state, animation, or drives must be removed before returning these pipes to draft')
    if any({item['joint']['a']['part'], item['joint']['b']['part']} & selected_set
           for item in doc.get('metadata', {}).get('detached_attachments', [])):
        raise DocumentError('Detached attachment references must be removed before returning these pipes to draft')
    draft_runs = []
    for pid in selected:
        part = assembly.parts[pid]
        direction = part.matrix[:3, 2]
        center = part.matrix[:3, 3]
        basis = axis_frame(direction)
        relative = basis.T @ part.matrix[:3, :3]
        roll = float(np.degrees(np.arctan2(relative[1, 0], relative[0, 0])))
        params = copy.deepcopy(direct[pid].get('parameters', {}))
        params.pop('length_mm', None)
        draft_runs.append({'id': pid, 'catalog': direct[pid]['catalog'], 'parameters': params,
                           'start_mm': np.round(center-direction*part.length/2, 8).tolist(),
                           'end_mm': np.round(center+direction*part.length/2, 8).tolist(),
                           'roll_deg': round(roll, 8), 'attachments': attachments[pid]})
    doc['parts'] = [part for part in doc['parts'] if part['id'] not in selected_set]
    doc['joints'] = [joint for joint in doc.get('joints', []) if joint['id'] not in removed_joints]
    existing = {group['id'] for group in doc.get('draft_subassemblies', [])}
    index = 1
    while f'draft-{index}' in existing:
        index += 1
    group_id = f'draft-{index}'
    doc.setdefault('draft_subassemblies', []).append({'id': group_id, 'runs': draft_runs})
    doc.pop('results', None); doc.pop('build_plan', None)
    preview(Assembly.from_doc(doc, assembly.base, assembly.library))
    return {'status': 'reopened', 'document': doc, 'subassembly': group_id,
            'converted_parts': selected, 'removed_joints': sorted(removed_joints)}
