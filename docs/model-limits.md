# Simulation and validation boundaries

## Three distinct questions

Geometric validation asks whether the authored interfaces and solids agree. Build planning asks whether the chosen parts have a supported insertion order. Dynamics asks how the current constraint system moves under load. A pass in one is not a pass in the others.

## Geometry

Tubeclamp casting envelopes are assembled from drawing-derived primitives. Bores are preserved in collision geometry using 24 tangent segments. Default 1 mm clearance and unreported engagement details are assumptions, not measured tolerances. Generic wheels, shackles and print concepts need actual hardware dimensions.

Connected parts within one rigid group cannot penetrate by more than the validator's 1 mm tolerance. Moving-group intersections are warnings in a static design and failures in an authored-motion check. Adjacent human caps and explicitly local mounting interfaces have limited exclusions. These exclusions do not suppress collisions throughout an entire structure.

Tube members use a solid exterior collision cylinder, so another object cannot move inside the tube's hollow core. Connector bores are hollow. Imported moving meshes use convex collision unless explicit collision geometry is supplied. Visual meshes and physics geometry can therefore differ.

Length editing solves translations at the current joint angles. It preserves anchors, other member lengths, end insertion and locked stations measured from the pipe's start, and permits bounded axial sliding at loose joints. It does not automatically rotate a linkage, change another cut length or solve a person's posture; reusable objects translate as a whole. A saved motion state must be captured as the new editing frame first. Release suggestions search a bounded set of nearby editable socket combinations, so a missing suggestion does not prove that every possible frame adjustment is impossible. Collision checks cover the final pose, not the physical cutting or insertion path.

Coordinated connection placement solves simultaneous sliding across existing rigid bodies. It keeps their current angles; hinges and ball joints can translate with their attached bodies but are not automatically rotated by this option. When there is no world anchor, the largest intermediate body on the connection path is held as an editing reference, typically the surrounding frame. This adds no physical anchor to the design. It checks the final connected pose and permitted joint travel, not a collision-free insertion trajectory. Existing single-body alignment alternatives remain available for rotations allowed by the current joints.

Interactive posing uses bounded inverse kinematics with exact tree connections and checked loop closures. A target is a preference, so a handle can stop short when its joint travel is exhausted. Anatomical limb edits use the pelvis or thorax as a posture reference; posing does not infer floor contact or active balance. Distance links are carried at their existing relative pose, and belt compliance and contact forces remain simulation tasks. Difficult closed loops can stop at the original pose if the local solve cannot preserve all constraints; this is not proof that no other posture is possible. Posing does not check swept collisions. Spherical limits retain the existing snapshot convention of rebased Euler ranges, which is approximate after large compound rotations.

## Assembly planning

The planner searches the reverse removal problem, then reverses that route into insertion instructions. It tries one part at a time, translating along permitted socket and world axes without rotation. Closed slip fittings constrain insertion to their pipe axis; a declared radial route requires an actual split-fitting port. Split halves are approximated during their radial stage, while nearby obstacles remain checked.

Conservative clearance advancement uses the closest geometric separation to choose each movement step. The internal 0.5 mm penetration threshold and 0.05 mm minimum step remain inside a stated 1 mm contact envelope. This improves thin-obstacle handling over a fixed-distance sweep; it still depends on correct collision meshes.

After each removal, every remaining rigid component needs a ground support polygon, real anchor or explicitly allowed fixture. Support assumes gravity along -Z, adequate anchors and no external disturbance. It does not prove frictional stability, tightening access, lifting ergonomics or the need for a second person's hands. Articulated bearing assemblies can need explicitly declared fixtures even when a human could hold them.

`buildable` means a sequence was found under the recorded assumptions. `blocked` identifies an invalid explicit sequence or unsupported final state. `indeterminate` means the bounded path/search class could not find a sequence; a coordinated multi-part insertion or curved path may still exist. The default search budget is 3,000 states.

## Rigid-body simulation

PyBullet runs at 1/240 s by default with 120 solver iterations. Closed loops use 1,000 iterations and substeps of at most 1/480 s to resolve muscle constraints and passive joint stops. These settings are recorded with results. Locked joints merge into compounds; joints with explicit break limits remain instrumented constraints. Moving trees use native prismatic/revolute joints, with two coordinates for cylindrical joints and three bounded serial rotations for spherical joints. Reverse traversal inverts their order as well as their axes. This permits readable joint coordinates and per-axis limits, but Euler singularities and transient constraint-limit error remain possible. Reduce the time step and compare results for difficult mechanisms.

Closed revolute/spherical loops use point constraints. Tree selection prioritizes joints with motors or limits, leaving a passive revolute/spherical edge to close each loop. Two hand grips therefore preserve all anatomical limits and posture motors. Sliding loops, including two-rail carriages and coaxial socket bearings, retain their native prismatic/cylindrical joints. The simulator splits a rigid body's inertia between two representations and welds them together to close the loop, keeping its collision geometry only once. Slide/twist limits, motors and socket disengagement remain active. Rotational cycles without an available passive closure remain unsupported. Closed-loop constraints are solved numerically: small drift and transient limit errors are possible, and forces in redundant supports depend on solver load sharing. Fully fixed world anchors are supported anywhere; selective anchor DOFs are an FEA feature.

The solver resolves gravity, contact, Coulomb friction, damping, torques and external forces. An unlocked socket detaches when its pipe no longer overlaps the bore. Explicit break limits trigger a topology rebuild that preserves poses, root momentum and remaining joint velocities. Spherical axes rebase at that event. Elastic fracture debris and plastic deformation are not generated.

Contact persistence uses a 0.1 mm breaking threshold, below the standard socket's
0.5 mm radial clearance. This prevents stale axle/bore contacts from producing
artificial friction that stops a swinging carriage. Physical collision shapes,
friction, stops and ground contact remain active. Regression tests exercise the
same two-bearing carriage from three starting angles and require continued
oscillation, alongside tests of actual collar stops and ground impacts.

`settled` means final linear speeds are below 0.02 m/s and angular speeds below 0.05 rad/s. It is an endpoint diagnostic, not proof that motion cannot recur. `tipped_or_rotated` records orientation changes above 20°; intended hinges can trigger it too.

GT2/gear drives are finite-compliance kinematic force couplings with reaction torque. Belt route geometry is illustrative. Slack, pulley wrap, tooth contact, tooth stripping and automatically changing routes are not solved. Distance links are penalty springs and can require smaller timesteps at low mass or high stiffness.

## Human model

The mannequin includes pelvis, lumbar and thoracic trunk, neck, head, paired clavicles, upper arms, forearms, hands, thighs, shins and feet. Segment masses sum to the requested total and inertias come from their actual geometric shapes. Every anatomical pivot is stored in local part coordinates, and paired pivots are regression-tested in all supplied poses.

Defaults are normalized engineering fractions and approximate joint ranges; they are not a verbatim implementation of a validated anthropometric dataset. Changing stature alone is not sufficient to represent a particular person, especially children or unusual proportions. Individual measurements and mobility ranges are explicit inputs. Optional curled fingers are rigid geometry within the hand segment, not articulated fingers. The model omits scapular motion, soft tissue, muscle force-length behaviour and active balance. Hand attachments impose an assumed grasp; frictional grip capacity, fatigue and release are not simulated.

Reach is bounded static inverse kinematics using several initial guesses; it can miss another collision-free solution or feasible path. Seated fit is dimensional screening. Optional posture servos have finite assumed strength and hold relative joint angles, not world-space balance. `hold_joints` selects muscle groups or individual joints; unselected joints remain passive with anatomical limits and damping. Per-axis motor efforts are recorded, but do not represent measured physiological muscle forces. None of these results predicts human injury or certifies gym equipment.

## Frame finite elements

Each Euler–Bernoulli element has 12 DOFs: three translations and three rotations at each node. Members split at connection and load stations. Rigid-offset maps transfer forces and moments through connectors and panels; joint constraints remove only the restrained coordinates. Constraint-nullspace analysis reports mechanisms rather than hiding them with artificial springs.

The solver includes axial stretch, bending in two directions, torsion, point loads/moments and consistent distributed self-weight. It calculates a conservative equivalent stress, elastic deflection and Euler buckling ratio. Effective-length factor defaults to 2 for the full cut member. Replace it with justified end-restraint data when appropriate.

The model is linear, small-deflection and isotropic. It excludes shear deformation in short/deep beams, local tube buckling, weld failure, fatigue, plasticity, creep, panel/shell deformation and casting fracture. Timber uses a longitudinal beam approximation, with no grain-dependent joint splitting. Large displacements or utilisation above one invalidate a purely elastic interpretation; they are flagged, not simulated as post-failure bending.

The Tubeclamp family axial statement is only used when the connection is locked and the declared installation torque meets its recorded condition. Per-size applicability still needs verification. Bending, torsion and casting fracture remain unknown. MiniTec's static connection statement is retained as provenance, not automatically converted into a universal directional strength.

Contact load conversion preserves the average resultant force and moment on each contacted structural part. It excludes removed payload mass to avoid double counting. It cannot establish detailed load distribution inside a rigid panel or solve dynamic beam response.

## Numerical verification and scale

Tests compare stiffness symmetry and six rigid-body modes, axial stretch, cantilever bending, end moments, torsion and distributed gravity against closed-form solutions. Dynamics tests check free fall, physical collar stops, release after breakage, belt travel, human energy and final joint limits. See [development](development.md) for the complete test command.

The FEA matrices and constraint nullspace are dense; modest assemblies are the target. Large frames need a sparse formulation. The planner is combinatorial and bounded. GUI calculations run in server threads, but there is no persistent job queue or multi-user collaboration. This version does not contain native STEP/B-rep modelling, a general contact-aware assembly robot planner or experimentally calibrated connector destruction data.
