"""Detach a selected part's external links and reconnect nearby free ports."""

import copy

import numpy as np

from .connections import socket_attachment, socket_blockers
from .document import Assembly, DocumentError
from .drafting import preview as draft_preview, runs as draft_runs
from .math3d import transform
from .port_connections import shackle_pin_hole


CAPTURE_MM = 12.0
AXIS_DOT = float(np.cos(np.deg2rad(12)))
EXACT_MM = 1.0
EXACT_AXIS_DOT = float(np.cos(np.deg2rad(1)))


def _owner(assembly, selected):
    if selected not in assembly.parts and selected not in {r['id'] for r in draft_runs(assembly.doc)}:
        raise DocumentError('Select a connector, length or flexible line')
    chain = next((o for o in assembly.doc.get('objects', [])
                  if o.get('template') == 'chain' and selected.startswith(o['id'] + '/')), None)
    return chain['id'] if chain else selected


def _inside(owner, part):
    return part == owner or part.startswith(owner + '/')


def _records(doc, owner):
    records = doc.setdefault('metadata', {}).setdefault('quick_detached', [])
    record = next((item for item in records if item['owner'] == owner), None)
    if record is None:
        record = {'owner': owner, 'joints': [], 'draft_attachments': []}
        records.append(record)
    return record


def detach_all(assembly, selected):
    """Release external links as one edit, retaining a route back to them."""
    owner = _owner(assembly, selected)
    doc = copy.deepcopy(assembly.doc)
    record = _records(doc, owner)
    old_joints = {joint['id'] for joint in record['joints']}
    retained = []
    for joint in doc.get('joints', []):
        a, b = joint['a']['part'], joint['b']['part']
        if _inside(owner, a) != _inside(owner, b):
            if joint['id'] not in old_joints:
                record['joints'].append(copy.deepcopy(joint))
        else:
            retained.append(joint)
    removed_ids = {joint['id'] for joint in doc.get('joints', [])} - {joint['id'] for joint in retained}
    doc['joints'] = retained
    for jid in removed_ids:
        doc.get('state', {}).get('joints', {}).pop(jid, None)
    if 'animation' in doc:
        doc['animation']['tracks'] = [track for track in doc['animation'].get('tracks', [])
                                      if track['joint'] not in removed_ids]
    if 'drives' in doc:
        doc['drives'] = [drive for drive in doc['drives']
                         if drive['driver'] not in removed_ids and drive['follower'] not in removed_ids]
    removed_draft = 0
    for run in draft_runs(doc):
        kept = []
        for attachment in run.get('attachments', []):
            if _inside(owner, run['id']) != _inside(owner, attachment['connector']):
                item = {'run': run['id'], 'attachment': copy.deepcopy(attachment)}
                if item not in record['draft_attachments']:
                    record['draft_attachments'].append(item)
                removed_draft += 1
            else:
                kept.append(attachment)
        run['attachments'] = kept
    count = len(removed_ids) + removed_draft
    if not count:
        return {'document': copy.deepcopy(assembly.doc), 'changed': False, 'count': 0}
    doc.pop('results', None)
    doc.pop('build_plan', None)
    Assembly.from_doc(doc, assembly.base, assembly.library)
    return {'document': doc, 'changed': True, 'count': count}


def _members(assembly):
    result = []
    for part in assembly.parts.values():
        if part.kind == 'member' and part.length > 0:
            result.append({'id': part.id, 'length': part.length,
                           'section': part.section, 'matrix': part.matrix, 'draft': False})
    for part in draft_preview(assembly):
        result.append({'id': part['id'], 'length': part['length_mm'],
                       'section': part['section'], 'matrix': transform(part['pose']), 'draft': True})
    return result


def _socket_fit(member, fitting, name, socket):
    section = member['section']
    profile = 'round' if section.get('type') in ('tube', 'round', 'circle') else section.get('profile', section.get('type'))
    if profile != socket.get('profile', 'round') or abs(section.get('diameter_mm', 0) - socket.get('diameter_mm', 0)) > .6:
        return None
    mouth, axis = fitting.frame({'port': name})
    matrix, length = member['matrix'], member['length']
    tolerance = CAPTURE_MM if member['draft'] else EXACT_MM
    axis_dot = AXIS_DOT if member['draft'] else EXACT_AXIS_DOT
    start = matrix[:3, 3] - matrix[:3, 2] * length / 2
    direction = matrix[:3, 2] / np.linalg.norm(matrix[:3, 2])
    if socket.get('through'):
        station = float((mouth - start) @ direction)
        half = socket.get('engagement_mm', 0) / 2
        if not half <= station <= length - half or abs(axis @ direction) < axis_dot:
            return None
        if np.linalg.norm(mouth - start - direction * station) > tolerance:
            return None
        return {'at_mm': round(station, 6)}
    for end, position, end_axis in (('start', start, direction), ('end', start + direction * length, -direction)):
        if axis @ end_axis < axis_dot:
            continue
        depth = float((mouth - position) @ axis)
        minimum, maximum = socket.get('min_engagement_mm', 0), socket.get('engagement_mm', 0)
        if member['draft'] and depth < minimum and abs(depth) <= maximum + CAPTURE_MM:
            lateral = np.linalg.norm(position - mouth + axis * depth)
            if lateral <= CAPTURE_MM:
                return {'end': end, 'insertion_mm': round(min(30, maximum * .8), 6)}
        if not minimum - tolerance <= depth <= maximum + tolerance:
            continue
        depth = float(np.clip(depth, minimum, maximum))
        if np.linalg.norm(position - (mouth - axis * depth)) <= tolerance:
            return {'end': end, 'insertion_mm': round(depth, 6)}
    return None


def _can_use_socket(assembly, doc, member, connector, port, fit):
    if socket_blockers(assembly, connector, port):
        return False
    for joint in assembly.joints:
        if joint.get('type') == 'socket' and {joint['a']['part'], joint['b']['part']} == {member, connector}:
            return False
        if 'end' in fit and any(end['part'] == member and end.get('end') == fit['end']
                                for end in (joint['a'], joint['b'])):
            return False
    for run in draft_runs(doc):
        for attachment in run.get('attachments', []):
            if run['id'] == member and (attachment['connector'] == connector or
                                        'end' in fit and attachment.get('end') == fit['end']):
                return False
            if attachment['connector'] != connector:
                continue
            other = assembly.parts[connector].ports[attachment['port']]
            socket = assembly.parts[connector].ports[port]
            if attachment['port'] == port or attachment['port'] in socket.get('excludes', []) or port in other.get('excludes', []):
                return False
    return True


def _append_socket(assembly, doc, member, connector, port, fit):
    if not _can_use_socket(assembly, doc, member['id'], connector, port, fit):
        return False
    if member['draft']:
        run = next(run for run in draft_runs(doc) if run['id'] == member['id'])
        run.setdefault('attachments', []).append({'connector': connector, 'port': port,
                                                   **({} if 'at_mm' in fit else fit)})
    else:
        endpoint, depth = socket_attachment(assembly, member['id'], connector, port,
                                            fit.get('end', 'start'), fit.get('insertion_mm'), fit.get('at_mm'))
        ids = {joint['id'] for joint in assembly.joints}
        number = 1
        while f'quick-joint-{number}' in ids:
            number += 1
        doc.setdefault('joints', []).append({'id': f'quick-joint-{number}', 'type': 'socket',
            'a': {'part': connector, 'port': port}, 'b': endpoint,
            'insertion_mm': depth, 'locked': True})
    return True


def _commit_candidate(doc, assembly):
    try:
        return Assembly.from_doc(doc, assembly.base, assembly.library)
    except (DocumentError, ValueError):
        return None


def _port_used(assembly, part, name):
    ports = assembly.parts[part].ports
    excluded = {name, *ports[name].get('excludes', [])}
    return any(end.get('part') == part and
               (end.get('port') in excluded or
                name in ports.get(end.get('port'), {}).get('excludes', []))
               for joint in assembly.joints for end in (joint['a'], joint['b']))


def _selected_end_used(assembly, owner, joint):
    for end in (joint['a'], joint['b']):
        if not _inside(owner, end['part']):
            continue
        if end.get('port') and _port_used(assembly, end['part'], end['port']):
            return True
        if end.get('end') and any(other.get('part') == end['part'] and other.get('end') == end['end']
                                    for current in assembly.joints for other in (current['a'], current['b'])):
            return True
    return False


def _port_joint(first, name, second, other_name, tolerance=EXACT_MM):
    a, b = first.ports[name], second.ports[other_name]
    if a.get('type') == 'socket' or b.get('type') == 'socket':
        return None
    types = {a.get('type'), b.get('type')}
    pin_hole = shackle_pin_hole(a, b)
    ordinary_hinge = types == {'eye', 'clevis'} and not any(
        port.get('type') == 'clevis' and port.get('snap_role') == 'shackle' for port in (a, b))
    if ((ordinary_hinge or pin_hole)
            and a.get('assembly') == b.get('assembly') == 'bolt'):
        if not pin_hole and abs(a.get('diameter_mm', 0) - b.get('diameter_mm', 0)) > 1:
            return None
        kind = 'revolute'
    # Flexible links use matched hook eyes; bolt hardware instead needs an eye and clevis.
    elif (a.get('type') == b.get('type') and a.get('assembly') == b.get('assembly') and
          (not types.intersection({'eye', 'clevis'}) or a.get('assembly') == 'hook')):
        kind = 'spherical' if a.get('assembly') == 'hook' else 'fixed'
    else:
        return None
    pa, aa = first.frame({'port': name})
    pb, ab = second.frame({'port': other_name})
    if np.linalg.norm(pa - pb) > tolerance or abs(aa @ ab) < EXACT_AXIS_DOT:
        return None
    return kind


def attach_nearby(assembly, selected, only_port=None):
    """Connect every compatible, unoccupied port already near the selection."""
    owner = _owner(assembly, selected)
    doc = copy.deepcopy(assembly.doc)
    count = 0
    members = _members(assembly)
    # Preserve the exact joint kind and hardware for detached non-socket links.
    record = None if only_port else next((r for r in doc.get('metadata', {}).get('quick_detached', []) if r['owner'] == owner), None)
    chain_object = next((o for o in doc.get('objects', [])
                         if o['id'] == owner and o.get('template') == 'chain'), None)
    if record:
        pending = []
        for joint in record['joints']:
            if any(existing['id'] == joint['id'] for existing in assembly.joints):
                continue
            if joint['a']['part'] not in assembly.parts or joint['b']['part'] not in assembly.parts:
                pending.append(joint)
                continue
            if any(end.get('port') and _port_used(assembly, end['part'], end['port'])
                   for end in (joint['a'], joint['b'])):
                pending.append(joint)
                continue
            if joint.get('type') == 'socket':
                socket_end = next((end for end in (joint['a'], joint['b'])
                                   if end.get('port') and assembly.parts[end['part']].ports.get(end['port'], {}).get('type') == 'socket'), None)
                member_end = joint['b'] if socket_end is joint['a'] else joint['a']
                fit = {key: member_end[key] for key in ('end', 'at_mm') if key in member_end}
                if socket_end is None or not _can_use_socket(assembly, doc, member_end['part'],
                                                              socket_end['part'], socket_end['port'], fit):
                    pending.append(joint)
                    continue
            try:
                pa, pb, _ = assembly.joint_frames(joint)
                _, axis_a = assembly.parts[joint['a']['part']].frame(joint['a'])
                _, axis_b = assembly.parts[joint['b']['part']].frame(joint['b'])
            except (DocumentError, KeyError):
                pending.append(joint)
                continue
            gap = np.linalg.norm(pa - pb)
            if gap > CAPTURE_MM or abs(axis_a @ axis_b) < EXACT_AXIS_DOT:
                pending.append(joint)
                continue
            candidate_base, candidate_assembly = doc, assembly
            if gap > EXACT_MM:
                if chain_object is None or count:
                    pending.append(joint)
                    continue
                from .grouping import move_object
                delta = pb - pa if _inside(owner, joint['a']['part']) else pa - pb
                target = copy.deepcopy(chain_object.get('pose', {}))
                target['position_mm'] = (np.asarray(target.get('position_mm', [0, 0, 0]), float) + delta).tolist()
                target.setdefault('rotation_deg', [0, 0, 0])
                try:
                    candidate_base = move_object(assembly, owner, target)['document']
                    candidate_assembly = Assembly.from_doc(candidate_base, assembly.base, assembly.library)
                except DocumentError:
                    pending.append(joint)
                    continue
                aligned_a, aligned_b, _ = candidate_assembly.joint_frames(joint)
                if np.linalg.norm(aligned_a - aligned_b) > EXACT_MM:
                    pending.append(joint)
                    continue
            candidate = copy.deepcopy(candidate_base)
            candidate.setdefault('joints', []).append(joint)
            updated = _commit_candidate(candidate, candidate_assembly)
            if updated is None:
                pending.append(joint)
            else:
                doc, assembly = candidate, updated
                count += 1
        record = next(r for r in doc['metadata']['quick_detached'] if r['owner'] == owner)
        record['joints'] = pending
        pending_draft = []
        for item in record['draft_attachments']:
            member = next((part for part in members if part['id'] == item['run']), None)
            attachment = item['attachment']
            fitting = assembly.parts.get(attachment['connector'])
            socket = fitting.ports.get(attachment['port']) if fitting else None
            if member is None or socket is None or _socket_fit(member, fitting, attachment['port'], socket) is None:
                pending_draft.append(item)
                continue
            candidate = copy.deepcopy(doc)
            if not _can_use_socket(assembly, candidate, member['id'], fitting.id,
                                   attachment['port'], attachment):
                pending_draft.append(item)
                continue
            run = next(run for run in draft_runs(candidate) if run['id'] == member['id'])
            run.setdefault('attachments', []).append(copy.deepcopy(attachment))
            updated = _commit_candidate(candidate, assembly)
            if updated is None:
                pending_draft.append(item)
            else:
                doc, assembly = candidate, updated
                count += 1
        record = next(r for r in doc['metadata']['quick_detached'] if r['owner'] == owner)
        record['draft_attachments'] = pending_draft
    for member in members:
        for fitting in list(assembly.parts.values()):
            if _inside(owner, member['id']) == _inside(owner, fitting.id):
                continue
            for name, socket in fitting.ports.items():
                if only_port and (fitting.id != owner or name != only_port):
                    continue
                if socket.get('type') != 'socket':
                    continue
                fit = _socket_fit(member, fitting, name, socket)
                if fit is None:
                    continue
                candidate = copy.deepcopy(doc)
                if not _append_socket(assembly, candidate, member, fitting.id, name, fit):
                    continue
                updated = _commit_candidate(candidate, assembly)
                if updated is not None:
                    doc, assembly = candidate, updated
                    count += 1
    for first in list(assembly.parts.values()):
        if not _inside(owner, first.id):
            continue
        for name in first.ports:
            if only_port and name != only_port:
                continue
            if _port_used(assembly, first.id, name):
                continue
            for second in list(assembly.parts.values()):
                if _inside(owner, second.id):
                    continue
                for other_name in second.ports:
                    if _port_used(assembly, second.id, other_name):
                        continue
                    current_first, current_second = assembly.parts[first.id], assembly.parts[second.id]
                    tolerance = CAPTURE_MM if chain_object and not count else EXACT_MM
                    kind = _port_joint(current_first, name, current_second, other_name, tolerance)
                    if kind is None:
                        continue
                    candidate_base, candidate_assembly = doc, assembly
                    pa, _ = current_first.frame({'port': name})
                    pb, _ = current_second.frame({'port': other_name})
                    if np.linalg.norm(pa - pb) > EXACT_MM:
                        from .grouping import move_object
                        target = copy.deepcopy(chain_object.get('pose', {}))
                        target['position_mm'] = (np.asarray(target.get('position_mm', [0, 0, 0]), float) + pb - pa).tolist()
                        target.setdefault('rotation_deg', [0, 0, 0])
                        try:
                            candidate_base = move_object(assembly, owner, target)['document']
                            candidate_assembly = Assembly.from_doc(candidate_base, assembly.base, assembly.library)
                        except DocumentError:
                            continue
                        if _port_joint(candidate_assembly.parts[first.id], name,
                                       candidate_assembly.parts[second.id], other_name) is None:
                            continue
                    candidate = copy.deepcopy(candidate_base)
                    ids = {joint['id'] for joint in assembly.joints}
                    number = 1
                    while f'quick-joint-{number}' in ids:
                        number += 1
                    candidate.setdefault('joints', []).append({'id': f'quick-joint-{number}', 'type': kind,
                        'a': {'part': first.id, 'port': name},
                        'b': {'part': second.id, 'port': other_name}})
                    updated = _commit_candidate(candidate, candidate_assembly)
                    if updated is not None:
                        doc, assembly = candidate, updated
                        count += 1
                        break
                if _port_used(assembly, first.id, name):
                    break
    if record:
        record = next(r for r in doc['metadata']['quick_detached'] if r['owner'] == owner)
        record['joints'] = [joint for joint in record['joints']
                            if not _selected_end_used(assembly, owner, joint)]
        if not record['joints'] and not record['draft_attachments']:
            doc['metadata']['quick_detached'].remove(record)
    if not count:
        return {'document': copy.deepcopy(assembly.doc), 'changed': False, 'count': 0}
    doc.pop('results', None)
    doc.pop('build_plan', None)
    return {'document': doc, 'changed': True, 'count': count}


def attached_connector_ports(before, after, selected):
    """Ports used by a successful shortcut, with the selected fitting first."""
    added = []
    old_joints = {joint['id'] for joint in before.get('joints', [])}
    for joint in after.get('joints', []):
        if joint['id'] not in old_joints:
            added.extend((end['part'], end['port']) for end in (joint['a'], joint['b'])
                         if end.get('port'))
    old_draft = {(run['id'], item['connector'], item['port'])
                 for run in draft_runs(before) for item in run.get('attachments', [])}
    for run in draft_runs(after):
        for item in run.get('attachments', []):
            if (run['id'], item['connector'], item['port']) not in old_draft:
                added.append((item['connector'], item['port']))
    parts = {part['id']: part for part in after.get('parts', [])}
    result = list(dict.fromkeys((part, port) for part, port in added if part in parts))
    return sorted(result, key=lambda item: item[0] != selected)


def attach_similar(assembly, exemplar, port, apply=False):
    """Attach the same open port on other copies of a fitting near a mate."""
    parts = {part['id']: part for part in assembly.doc.get('parts', [])}
    source = parts.get(exemplar)
    if (source is None or exemplar not in assembly.parts or
            port not in assembly.parts[exemplar].ports or
            not (source.get('catalog') or source.get('body'))):
        return {'changed': False, 'count': 0, 'candidates': []}
    identity = (source.get('catalog'), source.get('body'))
    doc = assembly.doc
    candidates = []
    total = 0
    occupied = {(end['part'], end['port'])
                for joint in assembly.joints for end in (joint['a'], joint['b'])
                if end.get('port')}
    occupied.update((item['connector'], item['port'])
                    for run in draft_runs(assembly.doc) for item in run.get('attachments', []))
    for part in assembly.doc.get('parts', []):
        part_id = part['id']
        if (part_id == exemplar or (part_id, port) in occupied or
                (part.get('catalog'), part.get('body')) != identity):
            continue
        current = Assembly.from_doc(doc, assembly.base, assembly.library)
        if part_id not in current.parts or port not in current.parts[part_id].ports:
            continue
        result = attach_nearby(current, part_id, only_port=port)
        if result['count']:
            candidates.append(part_id)
            total += result['count']
            doc = result['document']
    result = {'changed': bool(total) and apply, 'count': total, 'candidates': candidates}
    if apply:
        result['document'] = doc
    return result
