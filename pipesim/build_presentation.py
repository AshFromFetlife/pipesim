"""Practical instruction pages derived from an unchanged verified build plan."""
from __future__ import annotations

import copy

from .symmetry import canonical_catalog


def _initial_part_reorientation(assembly, initial, lifting):
    """Recognize a loose light part being put into its first working pose."""
    moving = lifting.get('moving_parts', [])
    if (initial.get('kind') != 'position' or lifting.get('kind') != 'lift'
            or len(moving) != 1 or initial.get('introduced_parts') != moving
            or initial.get('moving_parts') != moving
            or initial.get('subassembly') != lifting.get('subassembly')
            or not lifting.get('grounded_after')
            or initial.get('floor_z_mm') != lifting.get('floor_z_mm')):
        return False
    part = assembly.parts.get(moving[0])
    if part is None or part.mass > 15:
        return False
    for step in (initial, lifting):
        if (step.get('group_mass_kg', part.mass) > 15
                or step.get('helper_required') or step.get('assistance_required')
                or step.get('fixture') or step.get('world_anchor') or step.get('world_anchors')
                or step.get('fasten') or step.get('target_parts')
                or step.get('manual_support') or step.get('held_parts')
                or step.get('prepared_subassembly')):
            return False
    return True


def instruction_steps(assembly, plan):
    """Return display steps, keeping raw motion evidence and numbering intact.

    Only an initial loose singleton placement followed immediately by an
    unassisted floor-to-floor reorientation is combined. Its final working pose
    is the useful illustration; heavy handling and assembled-group lifts retain
    their own pages. ``source_steps`` maps each displayed instruction back to
    the unchanged verified sequence, and ``image_number`` selects its raw image.
    """
    raw = plan.get('steps', [])
    displayed = []
    index = 0
    while index < len(raw):
        source = raw[index]
        following = raw[index+1] if index+1 < len(raw) else None
        if following is not None and _initial_part_reorientation(assembly, source, following):
            step = copy.deepcopy(following)
            pid = source['introduced_parts'][0]
            part = assembly.parts[pid]
            step.update(kind='position', parts=copy.deepcopy(source.get('parts', [pid])),
                        introduced_parts=copy.deepcopy(source['introduced_parts']),
                        source_steps=[source['number'], following['number']],
                        image_number=following['number'])
            if part.kind == 'member':
                step['display_title'] = 'Position pipe'
                step['instruction'] = (f'Place {pid}, cut to {part.length} mm, '
                                       'on the floor in the shown orientation.')
            else:
                catalog, definition = canonical_catalog(part, assembly)
                label = catalog.rsplit('.', 1)[-1] if catalog else definition.get('name', pid)
                if catalog and catalog.startswith('tubeclamp.'):
                    label = label.upper()
                step['display_title'] = 'Position '+label
                step['instruction'] = f'Place {label} ({pid}) on the floor in the shown orientation.'
            index += 2
        else:
            step = copy.deepcopy(source)
            step['source_steps'] = [source['number']]
            step['image_number'] = source['number']
            index += 1
        step['number'] = len(displayed)+1
        displayed.append(step)
    return displayed
