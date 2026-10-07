# Architecture and verification

## Runtime

```
YAML / JSON + libraries + local meshes
                 │
          document.Assembly
      schema → expand → resolve → state
          ┌──────┼──────────┐
      geometry  constraints  material / section
          │       │             │
     Three.js  PyBullet      frame FEA
     renderer  simulation    stress / deflection
          │       │             │
       collision / motion / contact-load cases
                    │
             assembly planner
                    │
            BOM / cuts / build book
```

`document.py` defines the format boundary. `Part` and `Assembly` are shared by all tools. Joint locking determines rigid groups; users do not maintain a second subassembly tree. Rendering and physics use the same world/part coordinate convention.

`grouping.py` preserves ownership of edited articulated objects independently of those physical rigid groups. `objects[].components` contains local part/joint definitions, while expansion provenance supports regrouping without regenerating geometry. `posing.py` projects limb targets onto available joint motion; whole-object transforms apply one common transform and check external attachments. The editor exposes these as separate manipulation modes. Human parameter changes act relative to the saved component reference.

`geometry.py` contains visual and collision shape construction. `physics.py` writes temporary URDF articulation trees, runs isolated Bullet DIRECT clients and records readable states. `validation.py` evaluates geometry/support; `planning.py` searches safe insertion/removal sequences. `fea.py` contains the complete 12-DOF beam element and constraint solution. `human.py` constructs the mannequin and fit solvers. `loadcases.py` transfers simulation contacts. `rendering.py`, `engineering_plot.py`, `exporting.py` and `packaging.py` produce standalone artifacts.

`beam_dynamics.py` turns sufficiently deflecting, straight catalogue members into temporary short rigid elements for simulation. It selects members with frame FEA, chooses the dominant transverse load plane, and gives adjacent elements revolute bending joints with stiffness `EI/L`, damping, and a yield curvature derived from the same material and section data as FEA. Bullet handles gravity and contact with the floor and other geometry. The result keeps the authored cut member intact and includes temporary element geometry and poses for editor playback. Linear FEA now inserts interior nodes on long members so a restrained span's self weight produces a measurable midspan displacement. Dynamic bending is a one-plane approximation; multiaxial bending, local shell buckling, and fracture without measured ultimate strength remain outside this model. Dense frame systems above 240 estimated nodes currently keep rigid simulation behavior rather than risking an unbounded FEA allocation. `tests/test_beam_dynamics.py` covers free, ground-contact, obstacle-contact, supported-span, and overload cases.

The generic 6 mm jute profile uses a representative 1.8 kN dry, straight-rope
break load and an estimated 3% elongation at break. Its catalog source records
the supplier datasheet and the limits of that assumption. Flexible lines report
their applied threshold, and a continuous overloaded line breaks at its first
failed link rather than removing every segment in one simulation step.
Compact ropes keep saved link poses, while their generated internal joint
strength follows the selected catalog profile when an older design is opened.
`tests/test_rope_drop.py` checks a 10 kg mass falling through 100 mm of slack,
a 24 kg hanging mass and pendulum swing, and the first-link behavior under an intentionally low
rating. These are simulation regressions, not a rating for a specific rope,
knot, or load arrangement.

The local HTTP service serves a vendored Three.js frontend and calls the same Python functions as the CLI. It has no database, cloud storage or remote execution endpoint. Writes require a per-session token, a trusted loopback Host and same-origin requests. Paths are resolved within the editor root, excluding repository control directories. This is a local single-user service, not an internet-facing deployment.

## Commands and exit codes

`editor`, `library`, `validate`, `plan`, `analyse`, `simulate`, `stress`, `bom`, `build`, `render`, `animate`, `motion-check`, `mesh`, `fit`, `human`, `import-mesh`, `expand`, `snapshot`, `contact-loads`, `bundle`.

- **0:** command completed / requested check passed.
- **1:** validation, planning, fit or motion check failed; an FEA case was not solvable.
- **2:** invalid input, missing file, unsupported operation or runtime error.

A solved FEA result can still have `failures`; inspect those fields. Load trials return their event reports rather than treating destruction as a process error. `-o` writes machine-readable results and prints the destination. `--record` on supported calculations writes the result into the input file atomically.

## Verification

```powershell
python -m pytest -q
npm ci
npm run check
python -m pip wheel --no-deps --wheel-dir output/wheels .
```

The tests cover:

- Safe YAML, duplicate keys, nonfinite values, recursive aliases, schema errors, transforms and source dimensions.
- Valid examples plus negative socket occupancy, diameter, alignment, length, wall, engagement, support and load-reference cases.
- Trapped closed fittings versus actual split fittings, successful alternate build order, and bounded-search outcomes.
- Analytical cantilever bending, axial displacement, torsion, end moment and self-weight; stiffness symmetry/rigid modes; mechanisms, yielding, buckling and unknown connector strengths.
- Free fall, locked versus loose collars, physical stop contact, socket disengagement, breakable fixed joints, finite motor travel, pose consistency, human energy and final anatomical limits.
- Human segment count/mass, pivots, custom dimensions, reachable/unreachable/obstructed targets, narrow seats, clearance and bounded posture control.
- Simulated payload contact transferred to the correct structural component, force balance and stale-evidence rejection.
- Kerf/trim conservation, full-stock cuts, panel sizes in BOMs, self-contained mesh bundles, headless images/animations and printable export contents.
- Local service bootstrap, vendored resources, token/origin checks, path boundaries, save/reload, asset URLs and nonfinite JSON rejection.
- Continuous and terminated TC104 runs, shared-bore exclusions, midpoint stations, placement previews, anchored bodies, hinge/slider limits and movement of connected bodies.
- Editor pointer dragging, clear versus ambiguous snapping, connection previews, cancellation, undo, saved snap defaults, exact 90° rotation and alignment with existing angled parts. DOM interaction tests call the real Python service and Three.js geometry, with display widgets substituted; they do not verify WebGL pixels or native transform-handle picking.
- Saved-design browsing and search, OS file input and drop events, YAML/JSON imports, unsaved-change choices, failed/cancelled loads, library replacement and relative assets. Interaction tests run in an isolated temporary workspace and do not write to the user's designs folder.
- Tube rotation through the movement endpoint, explicit diagnostics for older servers, and session-token recovery after a server restart without replacing unsaved edits or undo history.
- Two-hand closed-loop human support, reverse spherical rotation coordinates, selective upper-body control, bounded recorded motor efforts, passive leg swings, relaxed-arm comparison, and UI posture editing/simulation playback.
- Property-panel duplication of catalogue parts, custom bodies and articulated objects, including unique IDs, internal state/tracks/drives, independent edits, undo/redo and failed-copy rollback.
- Connected member resizing: complete branch movement, floor/ceiling anchors, rotated through stations, rigid square rejection, verified screw-release alternatives, travel/engagement limits, obstruction collisions, reusable people, and atomic Properties-panel edits with undo and failure recovery.
- Coordinated connection placement on perpendicular sliding tees, including loose intervening branch sockets, stationary support frames, joint/motor/animation/drive rebasing, travel and collision rejection, rotated frames, drag projection, confirmation, cancellation and undo/redo.
- Articulated transform previews: shin posing with both hand grips retained, offset-bore rotation pivots, physical socket travel, reusable-object expansion, numeric properties, latest-request handling, cancellation and a single undo entry.

### Draft geometry fuzzing

`tests/test_solver_fuzz.py` runs deterministic regression seeds plus fresh random
draft cases on every ordinary test run. It generates one to six through fittings,
two-ended pipes, shared perpendicular tees, four-pipe closed grids, anchored
fittings, fixed or hinged companion parts, and scene mirrors with free, centered,
or in-plane pipe modes. It uses arbitrary world orientations for free pipes and
axis-aligned orientations where a mirror mode constrains pipe geometry. It
perturbs movable connector positions by 2–12 mm and rotations by 2–8°; one in
eight seeds uses wider drags of up to 40 mm and 20°. Some cases also move and
rotate a through pipe's draft centreline by those amounts,
including cases where the pipe alone moves. It checks that repair closes the
draft, then finalizes and validates the exact assembly. Straight through-run
and end-socket cases also check collisions in the finished geometry. The
synthetic tee graphs deliberately cross pipes and remain alignment-only cases.
A failure reports its
individual case seed and writes the baseline and disturbed documents into the
pytest temporary directory. Replay that exact model with:

```powershell
$env:PIPESIM_FUZZ_CASE_SEEDS = '123456789' # use the failing case_seed
python -m pytest -q tests/test_solver_fuzz.py -k draft_repair_varied
Remove-Item Env:PIPESIM_FUZZ_CASE_SEEDS
```

For a long randomized campaign, set a wall-time budget and a maximum number of
cases. The default long-run budgets are two hours and one million generated
models; the first limit reached stops the run. The random root seed and each
failing case seed are printed so a failure can be reproduced. Supplying a root
seed repeats the entire sequence. For example:

```powershell
$env:PIPESIM_FUZZ_LONG = '1'
$env:PIPESIM_FUZZ_HOURS = '4'
$env:PIPESIM_FUZZ_CASES = '1000000'
python -m pytest -s -q tests/test_solver_fuzz.py -k draft_repair_long_randomized
Remove-Item Env:PIPESIM_FUZZ_LONG, Env:PIPESIM_FUZZ_HOURS, Env:PIPESIM_FUZZ_CASES
```

The long mode runs 25 models per child process because geometry libraries can
retain native memory across models. Each batch reports its completed count and
elapsed time; a failing batch retains its generated designs under pytest's
temporary directory. The long mode is opt-in because a full campaign can take
hours. It uses new
randomness by default; fixed seeds are retained for reliable regression replay.
The separate `Long geometry fuzz` GitHub Actions workflow runs fresh two-hour
draft-repair, connected-resize, mirrored socket-capture, and three-mirror cube campaigns each day and can also be started
manually. On failure it keeps the generated designs as artifacts; the log
records the root and failing case seeds. The million-case setting is an upper
bound, not an expected two-hour throughput.

Browser verification additionally exercises the visible GUI: library loading, selection, lock/unlock body inference, edit dialogs, validation, structural response, simulation playback, save and printable instruction export. A browser screenshot is a layout check, not a numerical solver test.

`web/snapping.js` ranks projected connection candidates and aligns rotation axes. `web/snap-settings.js` validates browser defaults. `snapping.py` checks proposed body movements and returns connector/pipe alternatives without changing input; final poses are confirmed before becoming one undoable edit. It preserves anchors and joint freedoms rather than solving arbitrary inverse kinematics.

`sliding.py` adds an alternative that solves translations across the connected mechanism when a socket cannot be aligned by moving one rigid body alone. Existing joints, world anchors, slide limits and socket engagement constrain the solution. Without an anchor, a support body along the path between the connection sides provides the editing reference; the largest body on that path is preferred. Existing rotations and cut lengths are retained. The result passes through the same movement, joint rebasing, validation and collision checks as individual-body placement.

`posing.py` handles transform targets on `/api/move`. A rigid-body graph supplies forward kinematics for bounded hinge, slider, cylindrical and spherical coordinates. Additional anchors and loop closures enter the solve and every final relation is independently checked. Anatomical limbs first use the path from their pelvis or thorax. The GUI sends at most one preview request at a time, coalesces later pointer targets, and discards cancelled or stale responses. A separate rotation handle locates the actual hinge or pipe axis. Preview responses do not mutate the document; release rebases the accepted pose as one edit.

`preview.py` keeps bounded caches within each editor server: four parsed library sets, eight resolved documents, and up to sixteen prepared joint graphs and eight preview results per document. Scene resolution primes the document cache before the first drag. Keys include the complete document and its directory. Library lists, resolved asset paths, modification/creation timestamps, file size and file identity invalidate edited or replaced libraries/meshes; deleted files pass back through the normal asset checks. Source libraries are copied before document overrides are applied. Cached snapshots and mechanisms are read-only, and cache updates are synchronized across request threads.

API version 8 accepts `preview: true, pose_only: true` on movement requests. It returns checked poses and a solver seed without expanding objects, rebasing joints, hashing simulation inputs or generating a scene. Subsequent drag frames reuse the joint setup and seed. Part previews also return `preview_id`; release binds it to the original document, part, target and mode, rechecks the movement and materializes that exact pose. An expired or invalidated result falls back to solving the current inputs. Whole-object previews defer authoring work in the same way. Existing clients that only send `preview: true` still receive a document. Cancellation, failed commits and late responses leave the document and Undo history alone.

Run `python scripts/benchmark_preview.py --compare` for first-frame, changing-target and release timings over local HTTP, plus the former uncached/full-document pipeline. Results go to `output/preview-benchmark.json`; no designs are saved. `tests/test_preview.py` checks cache reuse, edit/file invalidation, directory and tab isolation, concurrent targets, saved joint state, result eviction, exact commits and legacy clients. Caching does not relax the numerical constraints; complex closed loops can still require more solving time than simple hinges or limb chains.

API version 9 adds `tolerance_mm` (default 2) and `force_options: {unlock_connectors: 0, resize_members: 0, max_length_change_mm: 20}` to `/api/snap-options`. `fit_adjustments.py` searches the nearest eligible connectors and members, preferring fewer edits. Only `force: true` enables the edit permissions. Each plan releases socket screws and represents each resizable member by start/end frames joined by a bounded virtual prismatic joint. The existing hinge/slide IK therefore solves poses and several cut lengths concurrently, including closed loops. Materialization converts these temporary frames into real parameterized geometry, rebases stations/travel/motors/tracks/drives, restores reusable objects and retightens screws. Virtual parts/joints never enter the saved design. Independently checked actual geometry supplies both collisions and `preview_parts` for the UI.

The bounded Force search considers up to 12 nearby connectors and 12 eligible members, at most 64 plans, with a roughly 30-second overall search budget. World-anchored members, fixed meshes and members with custom centre-frame attachments are excluded from resizing. Existing travel and engagement bounds remain active while screws are released. A fit within the requested allowance is retained as a fallback while permitted edits are searched for an exact fit. `tests/test_fit_adjustments.py` covers the 1 mm frame, varied small loop length errors, simultaneous length changes, rotated ceiling mounting, limits, grouped objects, collision geometry, retained clearance and immutable previews. Editor tests cover input validation, proposed edits, cancellation, and atomic Undo/Redo.

`resizing.py` implements `/api/resize` as a pure document edit. It solves connected-part translations from joint frames and anchors, with bounded axial freedom for loose sockets/sliders. The solver keeps orientations, other cut lengths and reusable-object poses intact. Inconsistent equality constraints identify the blocking graph; bounded candidate searches verify individual or paired screw releases, then disconnections. A failed request returns diagnostics without a document mutation. Successful edits rebase sliding stations, travel, motor/animation targets and drive offsets, clear stale calculations, and return the resolved scene for one GUI undo entry.

`resize_drag.py` implements `/api/resize-drag` for the two viewport end handles. It uses the rendered endpoints of draft pipes, which can differ from their stored hints when an end socket controls the pose. Extending a free end changes the selected pipe while leaving its existing fittings and connected structure seated. Shortening normally draws through fittings toward the fixed end; when that would disturb another connected or mirrored run, the editor shortens the selected free end locally. The cut retains full engagement in through fittings it already seats, while an unresolved draft fit can remain visible in orange. Finalized members can use the translation solver for angled branches. Every edit verifies the resulting physical endpoints and exact connection constraints. Exact members and flexible lines can be resized while unrelated pipes remain in draft. Shift behavior can leave connectors behind; extending into an unused through or end socket captures it. Draft capture can record a partly entered through fitting so later extension clears the orange engagement warning. Flexible lines resize through object parameters, then restore the fixed endpoint. `tests/test_resize_drag.py` covers both ends, drafts and finalized members, closed frames, connector follow/detach/capture, chains, and randomly reflected and rotated models. `tests/test_resize_capture_mirror.py` covers real connected mirror graphs, socket capture, and partially engaged fittings. Its fresh case seeds can be replayed with `PIPESIM_RESIZE_CAPTURE_CASE_SEED`. The editor interaction suite exercises mouse handles, actual API capture, keyboard shortcuts, preferences, and Undo.

`tests/test_resize_realworld.py` keeps a reduced, self-contained connected
assembly from a failing saved design. Regression cases extend the same pipes
from their free ends and check the rendered endpoints, all original connections,
mirror constraints, draft conflicts, and exact finalized validation. Random
cases generate connected fitting graphs with branches and optional scene mirrors,
then perform sequences of known-solvable end extensions. Each expected edit is
constructed and validated independently before the resize tool is called, so
a rejected operation fails the test. Ordinary runs combine retained seeds with
fresh random seeds; failures keep the individual seed and input document for
replay. The long run uses 25-case child batches to limit native geometry memory
growth and reports each completed batch. To run it locally:

```powershell
$env:PIPESIM_RESIZE_REALWORLD_LONG = '1'
$env:PIPESIM_RESIZE_REALWORLD_HOURS = '2'
$env:PIPESIM_RESIZE_REALWORLD_CASES = '1000000'
python -m pytest -s -q --basetemp=resize-fuzz-repro tests/test_resize_realworld.py -k resize_realworld_long
Remove-Item Env:PIPESIM_RESIZE_REALWORLD_LONG, Env:PIPESIM_RESIZE_REALWORLD_HOURS, Env:PIPESIM_RESIZE_REALWORLD_CASES
```

For socket capture followed by free-end shrink across one or two scene mirrors,
run the separate long campaign. It varies both pipe ends, through and end
sockets, rigid scene orientation, and a fitting shared with another draft run.
Each case independently builds a valid target before checking the editor edit.
The long run uses ten-case child batches and preserves each case's sequence index.

```powershell
$env:PIPESIM_RESIZE_CAPTURE_LONG = '1'
$env:PIPESIM_RESIZE_CAPTURE_SECONDS = '7200'
$env:PIPESIM_RESIZE_CAPTURE_CASES = '1000000'
python -m pytest -s -q --basetemp=capture-fuzz-repro tests/test_resize_capture_mirror.py -k long_generated_mirror_capture_and_shrink_sequences
Remove-Item Env:PIPESIM_RESIZE_CAPTURE_LONG, Env:PIPESIM_RESIZE_CAPTURE_SECONDS, Env:PIPESIM_RESIZE_CAPTURE_CASES
```

The deterministic cube regression starts with one TC128C corner, three pipes,
and three orthogonal scene mirrors. It requires materialization to produce eight
connectors, twelve pipes, and twenty-four distinct socket joints, followed by
exact validation with collisions enabled. The ordinary fuzz test varies all
three spans independently to make rectangular boxes, plus world position,
right-angle orientation, and mirror order. For a longer fresh-seed campaign
with replayable failure documents, the long run uses ten-case child batches
to limit native geometry memory growth:

```powershell
$env:PIPESIM_CUBE_FUZZ_LONG = '1'
$env:PIPESIM_CUBE_FUZZ_SECONDS = '7200'
$env:PIPESIM_CUBE_FUZZ_CASES = '1000000'
python -m pytest -s -q --basetemp=cube-fuzz-repro tests/test_symmetry_cube_fuzz.py -k cube_long_randomized
Remove-Item Env:PIPESIM_CUBE_FUZZ_LONG, Env:PIPESIM_CUBE_FUZZ_SECONDS, Env:PIPESIM_CUBE_FUZZ_CASES
```

## Rebuilding source data

The editable YAML libraries are runtime source files. `scripts/make_library.py` reproduces the initial catalogue from the transcribed supplier tables and stored source metadata. It overwrites those generated YAML files; preserve local catalogue edits elsewhere. `scripts/make_schemas.py` reproduces the JSON Schemas. The example generators are similarly explicit maintenance operations.

`python scripts/make_human_interaction.py` regenerates only the two pull-up examples from catalogue socket frames, cut lengths and a readable human pose. It does not overwrite saved user designs.

The TC136 and TC161 models also regenerate STL files under `pipesim/data/libraries/meshes`. Install the build-only boolean engine with `python -m pip install --target .pipesim/build-tools --no-deps manifold3d==3.5.3` before running the library generator. Runtime installations use the packaged meshes and need no boolean dependency. Editable socket annotations and convex collision shapes stay in YAML; `source.model_sha256` records the mesh checksum. The editor serves packaged STL files separately when they live outside the user's design workspace.

`scripts/fetch_sources.py` downloads public product JSON and drawings, retaining their URLs and hashes. It is not part of runtime and is not required for offline use. Source/reference assets retain supplier rights.

`npm install` and `npm run vendor` update the checked-in Three.js subset from the pinned dependency. The served application does not depend on `node_modules`.

## Extension points

New supplier parts generally require data, not Python changes. Add measured geometry, ports, mass and source assumptions to a library. New articulated equipment can be a reusable object containing ordinary parts/joints. For new solver behaviour, extend the JSON Schema and the relevant shared backend, then add a physical or geometric regression case.

The substantial next engineering extensions are a sparse frame solver, exact CAD/B-rep import, calibrated connector failure surfaces, deformable panels/contact, richer human kinematics and a planner supporting coordinated placement and tool access. Current outputs explicitly retain these model boundaries.
