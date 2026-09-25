# File format, version 1

Designs use `format: pipesim/1`; libraries use `format: pipesim-library/1`. Both accept YAML or JSON. Schemas are in [`pipesim/data/schemas`](../pipesim/data/schemas). Unknown top-level fields and unknown instance/joint properties are rejected; application-specific data belongs in `metadata`. Library definitions are intentionally extensible.

## Display names

Parts and object instances accept an optional `label` for their display name.
Renaming in the editor preserves `id` and all references to it. Names for generated
members are stored in `metadata.part_labels`, keyed by the full member ID.
`metadata.body_labels` stores records with a `label` and a `parts` list; each name
applies to that exact set of members, independent of the tree's Body numbering.

## Coordinates and units

- Length: **mm**, mass: **kg**, time: **s**, force: **N**, moment: **N·m**, angle: **degrees**.
- Right-handed world coordinates, **Z up**. Local member axis is Z, from `-length/2` to `+length/2`.
- `pose.rotation_deg: [x,y,z]` is extrinsic XYZ: `Rz(z) Ry(y) Rx(x)`. Translation is applied afterward. These are ordinary editable numbers, not serialized matrices.
- `state.joints.rotation_deg` uses intrinsic XYZ rotations about the joint's anatomical basis. Single-axis joint angles use the axis on endpoint A. Positive coordinates move B relative to A; if B is anchored, A moves inversely.
- The physics and FEA backends convert lengths to metres internally. Mesh coordinate units must be declared during import; GLB/STL files do not override PipeSim's mm convention.

```yaml
format: pipesim/1
units: mm-kg-s-N-deg
name: Loose collar
parts:
  - id: guide
    catalog: tubeclamp.tube-C
    parameters: {length_mm: 2000, wall_mm: 3.2}
    pose: {position_mm: [0, 0, 1000]}
  - id: collar
    catalog: tubeclamp.TC101C
    pose: {position_mm: [0, 0, 1300]}
joints:
  - id: slide
    type: socket
    a: {part: collar, port: through}
    b: {part: guide, at_mm: 1300}
    insertion_mm: 0
    locked: false
    limits: {slide_mm: [-600, 1250], angle_deg: [-180, 180]}
anchors:
  - {part: guide, surface: fixture}
state:
  joints:
    slide: {slide_mm: 250, angle_deg: 20}
```

Use the actual port names from `pipesim library --query TC101C`: the checked-in catalogue is the authority. The example above is explanatory; the complete runnable [sliding collar](../examples/sliding-collar.pipe.yaml) also includes its stop.

## Length-controlled chain objects

```yaml
objects:
  - id: chain
    template: chain
    parameters:
      length_mm: 1000
      link_catalog: generic.chain-link
    layout_mode: rigid
    pose: {position_mm: [0, 0, 1300], rotation_deg: [0, 30, 0]}
anchors:
  - {part: chain/link-1, surface: ceiling}
```

The origin is the first link's start eye, and an unposed chain extends along local negative Z. Links have stable IDs `chain/link-1` through `chain/link-N`; internal spherical joints are `chain/join-1` through `chain/join-(N-1)`. The first eye is port `b` on the first link, and the last eye is port `a` on the last link. Consecutive links alternate by 90 degrees about the chain axis.

`length_mm` is the requested length along the link attachment pitches. The generator rounds up to whole links, with a limit of 1000 links per object. `link_catalog` defaults to `generic.chain-link` and must identify a `kind: chain` part with distinct `a` and `b` ports. Pitch is the distance between those ports; the bundled proxy has a 20 mm pitch. Geometry, material, mass and provenance come from the selected catalogue part.

`layout_mode` defaults to `rigid`, which groups the current chain shape for editor movement and fitting. `posable` exposes link motion to the editor. Neither setting welds physical joints: rendering, validation and simulation resolve the individual links. Posing and editor frame capture save readable local poses and joints under `components`, as for other regrouped objects. Length changes retain those components, grow at the end or trim trailing links, and reject removal of links still referenced by external attachments, world anchors or loads.

## Parts and libraries

`libraries` lists additional files relative to the design file. Bundled libraries load automatically. A library contains `materials`, `parts` and optionally reusable `objects`, keyed by stable IDs. Duplicate library IDs are errors. Design-level `definitions` are explicit part overrides, including overrides of bundled parts.

```yaml
format: pipesim-library/1
name: My workshop parts
materials:
  my.measured-steel:
    density_kg_m3: 7850
    youngs_modulus_pa: 200000000000
    poisson_ratio: 0.3
    status: measured-coupon
parts:
  my.tube:
    name: Tube from my supplier
    kind: member
    material: my.measured-steel
    parameters: {length_mm: 1000}
    geometry:
      - {type: tube, diameter_mm: 42.4, wall_mm: 3.2, length_mm: $length_mm}
    section: {type: tube, diameter_mm: 42.4, wall_mm: 3.2}
    stock_lengths_mm: [6000]
    source: {supplier: My supplier, geometry_status: measured}
```

`$parameter` substitutes a complete scalar or value. It does not evaluate expressions or execute code. An instance can supply `parameters`, `pose`, `color`, `mass_kg`, friction/restitution, COM and principal inertia. An instance `body` can define or override geometry directly. Changing the material or section in a library invalidates recorded evidence fingerprints.

Supported geometry: `box` (`size_mm`), `cylinder`, `tube` (outside diameter and wall), `sphere`, `capsule` (straight section length plus end caps), simplified `extrusion`, and `mesh`. Shapes have optional local `position_mm`, `rotation_deg`, `axis` and color. Multiple shapes form a rigid body. Use `member` with a section for beam FEA; use `panel`, `rigid`, `load`, `wheel`, `chain`, `belt` or `human` for other bodies.

Sections are `tube`, `solid_round`, `rectangular`, or `explicit` with area, bending inertias, torsional constant and extreme-fibre distances. Section fields use mm, mm² and mm⁴. Elastic properties use Pa. `effective_length_factor` overrides the default conservative Euler factor 2. Timber is a longitudinal beam approximation.

Mass comes from `mass_kg`, then `mass_per_m_kg × length`, then density and primitive volume. Meshes require explicit mass. `center_of_mass_mm` is relative to the part frame; `inertia_kg_m2` contains three principal moments in local axes and is checked for physical inequalities. Otherwise COM/inertia are estimated from the mesh, with the parallel-axis theorem for compounds. Supply measured COM/inertia for asymmetric equipment.

Mesh import:

```powershell
python -m pipesim import-mesh pump.stl --id custom.pump --mass 12.5 --scale 1 -o parts/pump.yaml
```

This bakes geometry into a local GLB, removing external OBJ material dependencies. Add `parts/pump.yaml` to the design's libraries and annotate its ports in the library editor. Default dynamic collision is a convex hull. For hollow bodies, supply `collision_geometry` primitives on the mesh shape; the visual mesh remains detailed. Concave `static_mesh` is restricted to static collision inspection; use convex decomposition for dynamics.

## Ports and connections

A port has a local `position_mm`, `axis`, interface `type`, and installation method (`slide`, `radial`, `bolt`, `glue`, `hook`). Round sockets also specify nominal `diameter_mm`, `through`, `engagement_mm`, and `min_engagement_mm`.

For a terminal socket the port position is its outward mouth; the target member endpoint is `mouth - axis × insertion_mm`. Member `end: start` points inward along +Z; `end: end` points inward along -Z. For a through socket the origin is its centre, insertion is zero, and `at_mm` measures the member station from its start.

A socket joint may declare `fit_tolerance_mm` (0.01–20). New editor connections default to 2 mm; legacy joints without the field keep the validator's 1.1 mm distance threshold. This is the accepted residual between attachment frames, not an extra slide limit or angular freedom. The validator reports `ASSEMBLY_FIT_ALLOWANCE` when a gap exceeds 1.1 mm but remains within the explicit allowance. Actual tip engagement and collision checks still apply, including on locked sockets. A loosened socket retains its declared radial allowance when moved, without accumulating additional clearance on successive edits.

Accepted Force edits save the resulting ordinary poses, lengths and retightened screws. The new joint's `metadata.assembly_adjustments` records `unlocked` socket operations (`connector`, `joint`, `port`, `action: loosen_then_retighten`) and `resized` members (`part`, `before_mm`, `after_mm`, `change_mm`). Search permissions are dialog inputs, not persistent loosened screws or simulation state. The saved parts are sufficient for rendering, simulation and cutting plans.

Endpoint forms:

```yaml
{part: connector, port: socket}
{part: tube, end: start}
{part: tube, at_mm: 250}
{part: bracket, frame: {position_mm: [0, 20, 0], axis: [0, 1, 0]}}
```

Joint types:

| Type | Free coordinates |
|---|---|
| `fixed`, or any `locked: true` joint | None; joins one inferred rigid group |
| `socket`, unlocked | Axial translation and twist |
| `prismatic` | `slide_mm` |
| `revolute` | `angle_deg` |
| `cylindrical` | `slide_mm`, `angle_deg` |
| `spherical` | `rotation_deg: [rx,ry,rz]` |
| `distance` | Spring-enforced distance, optionally tension only |

`limits` contains `[min,max]` pairs, or three pairs for spherical rotation. Bounds are relative to the authored pose. `damping` and `friction` are joint resistance coefficients; angular values act on SI radians. `break_force_n` / `break_torque_nm` are explicit user-supplied failure thresholds. No fracture capacity is inferred from a blank field. `torque_nm` records the installed fastener torque for conditional capacity checks.

Distance links use `rest_length_mm`, `tension_only`, and damping. Their current spring stiffness is 20,000 N/m. Use separate rigid chain links and pin/spherical joints when chain contact geometry matters.

## World mounts, loads and motors

An `anchor` fixes a part to the world at its authored pose. `surface` labels floor, ceiling, wall or fixture; it does not move the part. `position_mm` is the reaction/reporting point. Selective `dofs` are supported by FEA; dynamics requires a fully fixed anchor. For a moving world support, anchor a fixture body and connect it with a joint.

Loads specify `part`, world-space `force_n` / `moment_nm`, and local `point_mm` or member `at_mm`. Optional `start_s` / `end_s` control application in dynamics. Static FEA includes the declared loads as one simultaneous case.

```yaml
motor:
  mode: position
  max_torque_nm: 1.2
  kp: 20
  kd: 1
  schedule:
    - {time_s: 0, target: 0}
    - {time_s: 3, target: 1800}
```

Motor targets are mm/degrees; velocity targets are mm/s or degrees/s. Spherical posture motors use `rotation_deg: [0,0,0]`. Position motors use Bullet constraint servos with position gain `min(1,kp × timestep)` and velocity gain `min(1,kd)`; these are not physical spring constants. Effort limits remain N or N·m.

`drives` connect driver/follower joint IDs. GT2 translation uses `pitch_mm × teeth / (2π)` per radian; gears use `ratio`. Finite stiffness/damping and maximum force transfer reactions back to the driver. Optional sign, efficiency and offsets are explicit. `route_mm` is a world-space visual path; tooth contact, changing belt paths, pretension and tooth failure are not solved.

## Objects, human models and saved poses

Reusable library `objects` contain ordinary parts/joints/anchors. An instance gives its own ID, parameters and pose. Expanded IDs use `instance/part`; external joints can connect to those IDs. [The caster library](../examples/libraries/mechanisms.yaml) is a complete small example.

An edited instance may store authoritative local definitions in `components: {parts: [...], joints: [...]}`. The component IDs and joint endpoints are local to the instance; its `pose` transforms them into the design. This preserves edited geometry, masses, joint frames, limits and motors instead of regenerating a template. Grouping does not create rigid joints. External joints, world anchors, state, animation, loads and tests keep the same full `instance/part` and `instance/joint` references.

`components.parameter_reference` records the human parameters at which those saved components were authored. Later mass/stature changes scale the saved components, and muscle strength, posture selection, damping and grip controls adjust the saved human without resetting its pose. Preset pose and individual measurement changes require posing or editing its parts. Expanding an object records `expanded_objects` provenance with its original instance, reference part/pose and ordering; regrouping consumes that record. Complete older expanded humans can also be recovered from their anatomical part names. Detached external attachments are retained in `metadata.detached_attachments` for the editor's reconnect preview.

`template: human` expands a 19-part mannequin. Parameters: `stature_mm`, `mass_kg`, `pose`, `measurements`, `joint_angles_deg`, `hold_pose`, `hold_joints`, `strength_scale`, `grip_diameter_mm`, `joint_damping_nms_rad`. Poses are `standing`, `seated`, `crouching` and `pull-up`. Measurement keys cover shoulder/hip width and thigh, shin, upper-arm, forearm, hand and foot lengths. Anatomical joint angles are checked before the pose is constructed. `expand` creates editable ordinary parts and joints without losing initial state.

`hold_pose: true` holds all 18 anatomical joints with finite torque. For selective control, leave it false and supply `hold_joints`: a list of anatomical joint names (without the instance prefix) or groups `arms`, `torso`, `upper_body`, `legs`. `upper_body` holds the spine, neck, clavicles, shoulders, elbows and wrists, leaving both hips, knees and ankles passive. An empty list is relaxed. Unknown names are errors. `strength_scale` scales the assumed torque limits, and may be greater than 0 and at most 10; it is not a calibrated measure of a person's strength. `joint_damping_nms_rad` sets nonnegative passive viscous resistance per axis (default 0.08 N·m·s/rad); it slows motion without targeting an angle. The pull-up examples use 1 N·m·s/rad.

```yaml
objects:
  - id: person
    template: human
    parameters:
      stature_mm: 1750
      mass_kg: 75
      pose: pull-up
      grip_diameter_mm: 42.4
      hold_joints: [upper_body]
      strength_scale: 6
      joint_damping_nms_rad: 1
      joint_angles_deg:
        right_hip: [6, 0, 0]
```

Each hand has a `grip` port at its palm centre, with local X along the grasped bar. `grip_diameter_mm` (8–80 mm) generates rigid curled fingers with a clear bore 2 mm larger than that diameter. Connect `person/left_hand` and `person/right_hand`, port `grip`, to a member's `at_mm` station using passive `revolute` joints. These joints prevent sliding/release and allow rotation about the bar. The parameter supplies geometry; it does not create an attachment automatically. The complete [pull-up example](../examples/human-pull-up.pipe.yaml) includes the positioned human, matching stations and both grips. Its hands are attached to the frame, and only the four frame flanges are anchored to the world.

`animation.tracks` interpolate joint coordinates over named times. `state.joints` records a pose without matrices. Closed-loop state edits that cannot be propagated consistently are rejected. Full simulation frames retain every part's position and Euler angles, joint coordinates, reactions, contacts and broken joints. `motor_efforts` records each driven coordinate's applied `torque_nm` (rotation) or `force_n` (translation); passive joints are absent. Recording headers include `dt_s`, `substeps` and `solver_iterations`. Results saved with `simulate --record` or the editor's Save action live under `results.simulate`.

`snapshot` makes a recorded frame the new zero pose, adjusts single-axis limits, motors and drive offsets, removes broken joints, and restarts at rest. Spherical Euler bounds rebase approximately; the snapshot metadata flags anatomical review after compound rotations. Recordings preserve motion history; snapshots are editable pose designs, not exact solver checkpoints.

## Build and test data

`build` can specify kerf, stock lengths, end trim, required stability, a complete sequence and temporary fixtures. Fixtures must have a named part and a physical support description; they require `allow_temporary_supports: true`. `build_plan` stores the discovered sequence and fingerprint.

Saved design `tests` support reach, seat and clearance, with `expect: false` for an intentional negative case. `results` stores validation, simulation, fit, stress or FEA output. It is excluded from input hashing, while resolved catalogue and mesh content are included. A changed source requires recomputation.

`bundle` produces `design.pipe.yaml`, a resolved material library and local meshes. It drops stale evidence while retaining the original input hash in metadata. The bundle can be moved to another machine without its original library paths.
