"""Temporary beam elements for contact-aware dynamic bending.

The design remains a single cut member. Only the simulation assembly is split;
section stiffness and yield curvature come from the same data as frame FEA.
"""
from __future__ import annotations

import copy
import math

import numpy as np

from .document import Part
from .fea import analyse, section_properties
from .math3d import pose_of


def _axial_shape(shape, length):
    shape = copy.deepcopy(shape)
    if shape.get('type') in ('tube', 'cylinder', 'capsule'):
        if abs(shape.get('length_mm', -1) - length) > 1e-5:
            return None
        shape['length_mm'] = length
    elif shape.get('type') in ('box', 'extrusion'):
        size = shape.get('size_mm', [])
        if len(size) != 3 or abs(size[2] - length) > 1e-5:
            return None
    else:
        return None
    if any(abs(v) > 1e-6 for v in shape.get('position_mm', [0, 0, 0])):
        return None
    return shape


def _segment_shape(shape, segment_length):
    shape = copy.deepcopy(shape)
    if 'length_mm' in shape:
        shape['length_mm'] = segment_length
    else:
        shape['size_mm'][2] = segment_length
    return shape


def _transverse_response(part, nodes):
    axis=part.matrix[:3,2]
    center=part.matrix[:3,3]
    strongest=np.zeros(3)
    for node in nodes:
        relative=np.array(node['position_mm'],float)-center
        station=float(relative@axis+part.length/2)
        if not -1e-4<=station<=part.length+1e-4:
            continue
        if np.linalg.norm(relative-axis*(station-part.length/2))>1e-3:
            continue
        motion=np.array(node['translation_mm'],float)
        transverse=motion-axis*(motion@axis)
        if np.linalg.norm(transverse)>np.linalg.norm(strongest):
            strongest=transverse
    return strongest


def prepare_beams(assembly, *, maximum_segment_mm=400, minimum_deflection_mm=1, minimum_segments=3):
    """Return an ephemeral assembly and source/element metadata.

    FEA decides which members need bending. The FEA displacement is only a
    selection signal: it is never used as an imposed pose, since its linear
    small-deflection assumption is invalid near the floor or another body.
    """
    potential=[p for p in assembly.parts.values() if p.kind=='member'
               and p.length>=2*maximum_segment_mm and p.shapes
               and all(_axial_shape(shape,p.length) is not None for shape in p.shapes)
               and p.definition.get('material_data',{}).get('youngs_modulus_pa')]
    if not potential:
        return assembly, {}, {'status':'no_flexible_members','members':[]}
    estimated_nodes=sum(max(1,math.ceil(p.length/1000))+1 for p in assembly.parts.values()
                        if p.kind=='member')+sum(p.kind!='member' for p in assembly.parts.values())
    if estimated_nodes>240:
        structural = {'status':'beam_model_too_large','members':[],
                      'message':'Static frame analysis exceeds its dense-matrix budget; dynamic beam elements remain enabled'}
    else:
        try:
            structural = analyse(assembly)
        except (ValueError, ArithmeticError):
            structural = {'status': 'unavailable', 'members': []}
    selection=structural
    if structural['status']=='mechanism':
        # Moving limbs or a passive pivot prevent static equilibrium even when
        # their supporting frame barely deflects. Hold the current joint pose
        # only for this selection estimate; the simulation keeps every joint's
        # original freedom. Keep the actual static-analysis status in the report.
        reference=copy.copy(assembly)
        reference.joints=copy.deepcopy(assembly.joints)
        for joint in reference.joints: joint['locked']=True
        try:
            estimate=analyse(reference)
        except (ValueError,ArithmeticError):
            estimate={}
        if estimate.get('status')=='solved':
            selection=estimate
            structural['beam_selection_reference']='temporarily_locked_pose'
    fallback = selection['status'] != 'solved'
    # A through collar travels along the whole guide. Attaching it to one
    # temporary element would shorten its track and introduce false end stops.
    # Keep that guide continuous until sliding across beam elements is modelled.
    guides={j['b']['part'] for j in assembly.joints
            if j.get('type')=='socket' and not j.get('locked',True)
            and assembly.parts[j['a']['part']].ports.get(j['a'].get('port'),{}).get('through')}
    if guides:
        structural['rigid_sliding_guides']=sorted(guides)
    potential_ids={p.id for p in potential}
    values = {row['part']: row for row in selection['members']}
    selected = {}
    for pid, part in assembly.parts.items():
        if pid in guides: continue
        row = values.get(pid)
        if fallback:
            if pid not in potential_ids: continue
            # A moving mechanism has no static equilibrium, and rope/human
            # discretization can exhaust a dense FEA budget. Neither condition
            # makes its structural members infinitely stiff or unbreakable.
            gravity=np.array(assembly.doc.get('environment',{}).get('gravity_m_s2',[0,0,-9.81]),float)
            axis=part.matrix[:3,2]
            direction=gravity-axis*(gravity@axis)
            if np.linalg.norm(direction)<1e-8: direction=part.matrix[:3,0]
            row={'max_von_mises_mpa':None,'yield_utilisation':None}
        else:
            if not row: continue
            direction=_transverse_response(part,selection['nodes'])
            if np.linalg.norm(direction)<minimum_deflection_mm: continue
        if part.length < 2*maximum_segment_mm or not part.shapes:
            continue
        if any(_axial_shape(shape, part.length) is None for shape in part.shapes):
            continue
        material = part.definition.get('material_data', {})
        youngs = material.get('youngs_modulus_pa')
        if not youngs or not math.isfinite(youngs):
            continue
        section = section_properties(part)
        count = min(32, max(minimum_segments, math.ceil(part.length / maximum_segment_mm)))
        selected[pid] = (part, row, section, count, direction)
    if not selected:
        return assembly, {}, structural

    dynamic = copy.copy(assembly)
    dynamic.doc = copy.deepcopy(assembly.doc)
    dynamic.parts = dict(assembly.parts)
    dynamic.joints = copy.deepcopy(assembly.joints)
    dynamic.anchors = copy.deepcopy(assembly.anchors)
    model = {}
    for pid, (part, row, section, count, direction) in selected.items():
        length = part.length / count
        ids = []
        for index in range(count):
            sid = f'{pid}~beam-{index + 1}'
            if sid in dynamic.parts:
                raise ValueError(f'{sid}: simulation element id collides with a design part')
            definition = copy.deepcopy(part.definition)
            definition['geometry'] = [_segment_shape(shape, length) for shape in part.shapes]
            definition['length_mm'] = length
            definition['mass_kg'] = part.mass / count
            definition.pop('center_of_mass_mm', None)
            definition.pop('inertia_kg_m2', None)
            matrix = part.matrix.copy()
            matrix[:3, 3] += part.matrix[:3, 2] * (-part.length / 2 + (index + .5) * length)
            dynamic.parts[sid] = Part(sid, {'id': sid}, definition, matrix, part.base)
            ids.append(sid)

        def remap(endpoint):
            local, axis = part.local_frame(endpoint)
            station = float(np.clip(local[2] + part.length / 2, 0, part.length))
            index = min(count - 1, int(station / length))
            frame = copy.deepcopy(endpoint.get('frame', {}))
            frame['position_mm'] = [float(local[0]), float(local[1]), station - (index + .5) * length]
            frame['axis'] = axis.tolist()
            endpoint.clear()
            endpoint.update(part=ids[index], frame=frame)

        for joint in dynamic.joints:
            for end in ('a', 'b'):
                if joint[end]['part'] == pid:
                    remap(joint[end])
        for anchor in dynamic.anchors:
            if anchor['part'] != pid:
                continue
            location = np.array(anchor.get('position_mm', part.matrix[:3, 3]), float)
            station = float(np.clip((location - part.matrix[:3, 3]) @ part.matrix[:3, 2] + part.length / 2, 0, part.length))
            anchor['part'] = ids[min(count - 1, int(station / length))]
        for load in dynamic.doc.get('loads', []):
            if load['part'] != pid:
                continue
            station = float(load.get('at_mm', load.get('point_mm', [0, 0, 0])[2] + part.length / 2))
            index = min(count - 1, max(0, int(station / length)))
            local = np.array(load.get('point_mm', [0, 0, station - part.length / 2]), float)
            load['part'] = ids[index]
            load['at_mm'] = station - index * length
            load['point_mm'] = [float(local[0]), float(local[1]), float(local[2] + part.length / 2 - (index + .5) * length)]

        E = part.definition['material_data']['youngs_modulus_pa']
        span = length / 1000
        member_axis = part.matrix[:3,2]
        hinge_world = np.cross(member_axis,direction)
        if np.linalg.norm(hinge_world)<1e-9:
            hinge_world = part.matrix[:3,0]
        hinge_local = part.matrix[:3,:3].T @ (hinge_world/np.linalg.norm(hinge_world))
        bending_inertia = section['iy']*hinge_local[0]**2 + section['iz']*hinge_local[1]**2
        stiffness = E*bending_inertia/span
        yield_strength = part.definition['material_data'].get('yield_strength_pa')
        # M_y = sigma_y I / c; theta_y = M_y L / EI.
        outer_fibre = abs(hinge_local[0]) * section['cz'] + abs(hinge_local[1]) * section['cy']
        yield_angle = (yield_strength * span / (E * outer_fibre)
                       if yield_strength else None)
        model[pid] = {'segments': ids, 'length_mm': part.length,
                      'selection':'dynamic_fallback' if fallback else structural.get('beam_selection_reference','static_deflection'),
                      'rest_pose': pose_of(part.matrix), 'segment_length_mm': length,
                      'initial_stress_mpa': row['max_von_mises_mpa'],
                      'initial_yield_utilisation': row['yield_utilisation'],
                      'yield_angle_rad': yield_angle, 'hinge_axis':hinge_local.tolist()}
        for index, (left, right) in enumerate(zip(ids, ids[1:])):
            remaining=count-index-1
            effective_inertia=part.mass*remaining/count*(remaining*span/2)**2
            damping=math.sqrt(stiffness*max(effective_inertia,1e-5))
            dynamic.joints.append({'id': f'{pid}~bend-{index + 1}', 'type': 'revolute',
                'a': {'part': left, 'frame': {'position_mm': [0, 0, length / 2], 'axis':hinge_local.tolist()}},
                'b': {'part': right, 'frame': {'position_mm': [0, 0, -length / 2], 'axis':hinge_local.tolist()}},
                'metadata': {'beam_hinge': True, 'source_part': pid,
                             'stiffness_nm_rad': stiffness, 'damping_nm_s_rad':damping,
                             'yield_angle_rad': yield_angle}})
        del dynamic.parts[pid]
    return dynamic, model, structural
