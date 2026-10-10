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

`grouping.py` preserves ownership of edited articulated objects independently of those physical rigid groups. `objects[].components` contains local part/joint definitions, while expansion provenance supports regrouping without regenerating geometry. `posing.py` projects limb targets onto available joint motion; whole-object transforms carry the complete free attachment component, including other grouped objects, through one common rigid transform. Anchored components retain their attachment constraints. The editor exposes these as separate manipulation modes. Human parameter changes act relative to the saved component reference.

`geometry.py` contains visual and collision shape construction. `physics.py` writes temporary URDF articulation trees, runs isolated Bullet DIRECT clients and records readable states. `validation.py` evaluates geometry/support; `planning.py` searches safe insertion/removal sequences. `fea.py` contains the complete 12-DOF beam element and constraint solution. `human.py` constructs the mannequin and fit solvers. `loadcases.py` transfers simulation contacts. `rendering.py`, `engineering_plot.py`, `exporting.py` and `packaging.py` produce standalone artifacts.

`beam_dynamics.py` turns sufficiently deflecting, straight catalogue members into temporary short rigid elements for simulation. It selects members with frame FEA, chooses the dominant transverse load plane, and gives adjacent elements revolute bending joints with stiffness `EI/L`, damping, and a yield curvature derived from the same material and section data as FEA. Bullet handles gravity and contact with the floor and other geometry. The result keeps the authored cut member intact and includes temporary element geometry and poses for editor playback. Linear FEA inserts interior nodes on long members so a restrained span's self weight produces a measurable midspan displacement. Dynamic bending is a one-plane approximation; multiaxial bending, local shell buckling, and fracture without measured ultimate strength remain outside this model. Dense frame systems above 240 estimated nodes skip the dense static solve but retain dynamic beam elements. An unavailable static equilibrium no longer silently makes a moving arm infinitely stiff. Free-sliding through-collar guides stay continuous because a collar cannot yet transfer between temporary beam elements. `tests/test_beam_dynamics.py` covers free, ground-contact, obstacle-contact, supported-span, and overload cases.

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

Light flexible segments coupled to heavy bodies need small internal integration
steps and gentle contact/joint position correction. Refining the internal timestep
also scales the error-reduction fraction to preserve its correction rate per second.
Remaining rope constraints
keep their own local endpoint pivots when a break rebuilds the world. Closed-loop
beam hinges receive the same elastic and plastic response as tree hinges.
`tests/test_swing_dynamics.py` retains the actual four-way swing, checks surviving
rope joint closure, and verifies that the overloaded arm yields. Its configured
counterweight is 2000 kg and its mannequin is 50 kg.

For smaller mechanisms, beam selection can use a temporary estimate with joints
held at their current pose. This avoids enabling expensive beam elements merely
because a mannequin's legs can move. The actual static-analysis status remains
`mechanism`, and simulated joints retain their original freedom.

`tests/test_dynamics_fuzz.py` adds a 45-minute default physics campaign to the
existing editing campaigns. It varies rope material, body mass, counterweight,
world orientation, and ground clearance, and includes severed free fragments.
Its oracles check joint closure, gravity-driven fragment flight and landing,
retained beam response, and preservation of the authored document. Seeds 0–2
retain the reported swing and fragment cases, and seed 3032479272 retains a
chain-in-hand contact failure found by this campaign. Seed 3198599736 retains a
joint-drift failure with jute and a light counterweight, before fresh cases run.
The exact swing runs through the production recording path for all three seconds
and must retain its nylon connections; deleting constraints cannot satisfy the
continuity check by removing every joint it would inspect. Injected false breaks
and finite scattered geometry verify that these oracles reject bad recordings;
selected random beam cases continue through 1.2 seconds to include later impacts.
Material variations include assemblies mixing rope, chain, and webbing. Four fresh
cases spanning the simulation families always run after the retained cases, even
when a slow full replay uses the nominal time budget.
Fresh seeds exclude retained and already-run seeds, including when an original
random root is replayed after one of its failures becomes a regression. Preview
cases vary grouping, force-update rate, solver iterations, and beam resolution.
The completed swing recording is retained alongside failure artifacts. A retained
regression also checks joint closure through the late 1.5-second impact; checking
only the first quarter-second allowed an insufficient solver budget to pass.
Use `PIPESIM_DYNAMICS_FUZZ_MINUTES` to change the budget,
`PIPESIM_DYNAMICS_FUZZ_SEED` to repeat a campaign, or
`PIPESIM_DYNAMICS_FUZZ_CASE_SEED` to replay one case. Failures save the exact
input and seed under `fuzz-runs/dynamics/`. Set `PIPESIM_DYNAMICS_FUZZ_REPLAY` to
one of those failure directories to run the saved input unchanged, including
after the generator evolves. Finite output and the absence of
reported break events are not sufficient evidence of physical stability.

The simulation panel offers **Preview** and **Full physics**. Preferences stores
the default mode and Preview's maximum links per rigid body, beam segment length,
physics steps per second, and solver iterations. Full ignores these Preview
settings and retains individual links. Recordings store their mode and effective
quality settings; playback identifies approximate Preview results. The CLI exposes
the same controls with `simulate --mode preview` and `--preview-*` options.
Older browser preferences migrate the prototype's one-link Full default to
Preview's eight-link maximum. Custom larger groups are retained, and explicitly
selecting one link in the new Preview preferences remains supported.

Preview groups uniform rated rope and chain sections while preserving mass,
authored poses, attachment links, and the complete centerline load-transfer path.
Remaining flexible boundaries retain the original tensile rating. Weaker sections,
motors, and drives remain flexible; breaks occur at retained boundaries, so their
location is approximate. Grouping preserves existing slack bends and targets
about ten groups along sufficiently long lines. End links may join a compound;
their external attachments retain their original joints and frames, avoiding
isolated tiny bodies beside heavy payloads. Saved joint coordinates are applied
before choosing groups. The legacy explicit `chain_links_per_body` option also
works in Full, but the editor always uses one link per body for Full physics.

Preview keeps dynamic bending enabled on members at least twice the selected
beam segment length; shorter members remain rigid. Eligible members use at least
two elements. Full retains its finer element selection. Internal contact steps
remain finer than the outer force-update step because gram-scale links coupled
to heavy bodies still need stable contacts.
Preview increases contact substeps and solver iterations when measured joint drift
exceeds 2 mm, retaining the extra effort after a topology rebuild and reducing it
again after twenty steps below 1 mm drift. The recording
reports requested and effective solver settings and number of contact refinements.
If a recorded frame still has an unbroken link gap over 8 mm, Preview discards
that attempt and replays the authored model with Full physics. This avoids
returning a successful-looking disconnected recording. Progress and playback
identify the automatic refinement; cancellation remains available. The default
saved-swing Preview must complete without this fallback, while retained seed 7
checks the high-step contact case that needs it. Such cases prioritize closure
over Preview speed. Full flexible-only models update forces at least 2,000 times
per second; internal Bullet contact steps remain finer.
Regression tests compare loaded pendulum trajectories with Full physics, check
mass and pose preservation, and force a grouped line to break and lose its support
spring. The elastic cantilever check compares the same physical tip against Full
and the beam calculation, rather than comparing centres from different meshes.
The dynamics campaign retains a complete three-second Preview replay
(seed 3) and varies link grouping and materials in fresh cases.

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
- Editor pointer dragging, clear versus ambiguous snapping, connection previews, cancellation, undo, saved snap defaults, exact 90° rotation and alignment with existing angled parts. DOM interaction tests call the real Python service and Three.js geometry, with display widgets substituted. A whole-person mirror regression uses the actual Three.js transform controls to ray-pick the rendered arrowheads and drag both allowed axes; WebGL pixels remain outside the harness.
- Saved-design browsing and search, OS file input and drop events, YAML/JSON imports, unsaved-change choices, failed/cancelled loads, library replacement and relative assets. Interaction tests run in an isolated temporary workspace and do not write to the user's designs folder.
- Tube rotation through the movement endpoint, explicit diagnostics for older servers, and session-token recovery after a server restart without replacing unsaved edits or undo history.
- Two-hand closed-loop human support, reverse spherical rotation coordinates, selective upper-body control, bounded recorded motor efforts, passive leg swings, relaxed-arm comparison, and UI posture editing/simulation playback.
- Property-panel duplication of catalogue parts, custom bodies and articulated objects, including unique IDs, internal state/tracks/drives, independent edits, undo/redo and failed-copy rollback.
- Connected member resizing: complete branch movement, floor/ceiling anchors, rotated through stations, rigid square rejection, verified screw-release alternatives, travel/engagement limits, obstruction collisions, reusable people, and atomic Properties-panel edits with undo and failure recovery.
- Coordinated connection placement on perpendicular sliding tees, including loose intervening branch sockets, stationary support frames, joint/motor/animation/drive rebasing, travel and collision rejection, rotated frames, drag projection, confirmation, cancellation and undo/redo.
- Articulated transform previews: shin posing with both hand grips retained, offset-bore rotation pivots, physical socket travel, reusable-object expansion, numeric properties, latest-request handling, cancellation and a single undo entry.

### Stateful geometry editor fuzzing

`npm run check` includes six fixed geometry workflow seeds and twelve fresh
ones. Each constructs a catalogue comb, closed elbow rectangle, or offset
TC161 grid with varying size, world orientation, pipe direction, and connection
order. It starts with unconnected parts, connects them through editor controls,
then shuffles twelve required editing operations among eighteen additional
random operations. These include library construction and placement, both
endpoint hotkeys, the length inspector, connected translation, detach/reconnect,
duplicate/delete, undo/redo, repair, JSON reload, finalization/reopening, and
live mirror finalization/reopening. Selected finalization also checks mixed
draft/exact scenes and preservation of disconnected components. A separate
component detects operations
accidentally applied outside the selection. Actions operate on the state left
by previous actions, rather than resetting to a fixture between edits. Cases
use both 10-entry and 80-entry undo histories, including full-stack rollover.

The independent reference model in `tests/helpers/geometry-workflow-model.mjs`
uses catalogue dimensions and vector arithmetic. It requires the requested
endpoints, unchanged other spans and fitting frames, and the complete expected
connection graph. Finalization additionally checks socket frames and engagement
independently, then calls real validation **with collisions enabled**. The DOM
harness runs actual editor handlers and the Python HTTP service. WebGL rendering
and transform widgets are substituted; this is not visual or native picking
coverage. Error toasts, failed HTTP requests, conflict responses, blocking
dialogs, missing effects, and unexpected geometry changes fail the workflow.
Legitimate impossible-geometry rejection belongs in separate negative tests.

The older long geometry campaigns cover narrower solver properties. In
particular, some synthetic graphs disable collisions and do not establish that
an entire editor workflow works. The lattice sequence generator previously
caught oracle assertion/validation errors and selected a different edit; it
now reports them and uses constructive extend/restore pairs. The extended
resize campaign runs those lattice sequences alongside each generated graph
seed; `PIPESIM_RESIZE_LATTICE_SEEDS` replays individual lattice seeds. No generator in
the new editor suite calls the solver to discard rejected candidates. More
time alone does not fill a missing operation or assertion.

The new workflow exposed three regressions during its development: browser JSON
number normalization invalidating mirror reopening, and shortening one free
grid end silently deforming other pipes, and mirroring a hollow TC161 fitting
replacing its collision solids with a hull that fills the bore. Saved replay fixtures retain all three
workflows independently of generator changes; focused Python regressions
accompany them. Mirror reopening also retains compatibility with older saved
fingerprints. The long mirror campaign now also varies offset socket reflections
and checks that real obstructions remain detectable on both sides. An ordinary editor test also injects
false finalization rejection, connection loss, and incorrect length to verify
the workflow oracles actually fail. Every ordinary case requires successful
coverage of connections and all twelve subsequent operations, and the ordinary
seed set must cover all three topology families. Automatic incidental socket capture is recorded
separately from an explicitly performed connection action.

Run a sustained campaign with `npm run fuzz:editor`. It uses fresh processes,
a default forty-five-minute budget, and longer varying sequences (18–60 additional
actions). It prints the actual completed case/action counts and writes
`campaign.json`; elapsed time or a case limit is not evidence of stability
outside the generated scenarios. A zero process exit without the expected case
and operation coverage report fails the campaign; tests verify that skipped
workflows and empty coverage cannot count as passes. The four Python campaigns now default to
fifteen minutes each. The manual Geometry fuzz workflow runs the editor campaign
and fault checks; normal checks run the bounded editor suite on all three OSes.
Both workflows retain failure artifacts.

Anchored endpoint resizing has its own mandatory workflow matrix. The retained
fixture is the reported design's occupied pipe ends, offset through cross, and
crossbar fixed by two remote flanges. Six fixed cases cover draft, exact, and
mixed assemblies with both endpoint orders. Twenty-four fresh cases additionally
vary rotation, translation, cut length, through-support station, one/two anchors,
and keyboard step. Every case presses U/J and Y/H through the editor, requires
success, checks the requested and opposite ends, preserves world anchors and
connections, and checks undo/redo, reopening, and final validation. Fault injection
verifies that false rejections, anchor movement, dropped connections, and wrong
endpoint movement fail the oracle. The original implementation failed both the
retained keyboard regression and this fuzz matrix with the reported anchor error.

`npm run check` includes these workflows, and every case of `npm run fuzz:editor`
also runs an anchored resize workflow. `PIPESIM_ANCHORED_RESIZE_CASES` controls fresh
cases (default 24); `PIPESIM_ANCHORED_RESIZE_SEED` chooses their root seed, and
`PIPESIM_ANCHORED_RESIZE_CASE_SEED` selects one case. Failures save the complete
starting document, key sequence, and API/toast failures under
`fuzz-runs/anchored-resize`. Set `PIPESIM_ANCHORED_RESIZE_REPLAY` to its `replay.json`
and run `node --test --test-name-pattern='seeded anchored resize' tests/editor-interactions.test.mjs`
to replay the failure.

Free assembly placement retains the complete `4WaySwing` reproduction, including
its human, four chains, attached load, and mirrored collision assets. Nine fixed
cases and eight fresh cases drag either subassembly into the socket, varying
the free pipe end, world orientation, gap, and chain layout mode. The fixed matrix
crosses both sides, both ends, and both chain modes independently, and retains
the browser-rounding failure discovered by this generator. The oracle
requires every attached part to receive exactly one common rigid transform,
with all internal joints and object definitions preserved. Rejection, a Force
dialog, missing payloads, incorrect poses, and broken undo/redo are failures.
Preview latency is bounded, and backend regressions forbid entering joint search
for these operations. Fault injection independently checks rejection, a detached
payload, and a lost joint.

Every sustained editor campaign case now also runs this placement workflow and
requires its completion report. `PIPESIM_FREE_ASSEMBLY_CASES` controls fresh cases
(default 8); `PIPESIM_FREE_ASSEMBLY_SEED` chooses the root seed and
`PIPESIM_FREE_ASSEMBLY_CASE_SEED` selects one case. Failures save the document,
seed, requests, and responses under `fuzz-runs/free-assembly`. Set
`PIPESIM_FREE_ASSEMBLY_REPLAY` to the saved `replay.json` and run
`node --test --test-name-pattern='seeded free assembly' tests/editor-interactions.test.mjs`.

Material changes have a separate mandatory editor matrix. It changes the saved
swing's four bent 1000 mm nylon ropes to chain, another material, and back to
nylon. Attachment stations can land inside the new links (the body attachments
are at 975 mm), so successful API responses alone are insufficient: the oracle
checks every link's length, internal joint closure, preserved external attachment
points, stationary hosts, requested length, and undo/redo. Eleven fixed cases
cross the four lines and both layout modes and cover 20, 25, and 80 mm link
pitches; eight fresh cases additionally vary world poses. Fault injection checks
false rejection, lost attachments, moved hosts, and stretched links.

Every campaign round requires three successful profile changes as well as the
other workflow reports. `PIPESIM_MATERIAL_CASES` and `PIPESIM_MATERIAL_SEED` control
fresh cases; `PIPESIM_MATERIAL_CASE_SEED` selects one case. Failures retain the
document, profile sequence, requests, responses, and errors in `fuzz-runs/material`.
Set `PIPESIM_MATERIAL_REPLAY` to its `replay.json` and run
`node --test --test-name-pattern='seeded material' tests/editor-interactions.test.mjs`
to replay it. The original algorithm fails both the saved-design regression and
the editor matrix; the attachment mismatch must never count as an expected rejection.

Whole-object movement has a mandatory matrix of free chains joined at interior
links. Each case poses a tail link in the editor, switches to Move whole line,
then translates and rotates using the actual gizmo handlers and API. Six retained
cases cross both selected chains with spherical, revolute, and fixed joins;
eight fresh cases vary lengths, attachment stations, world orientations, and
movement targets. Independent rigid-transform checks cover every preview and
committed part, including an unrelated chain that must stay still. Object shapes,
attachments, and undo/redo must survive unchanged, and preview latency is bounded.
The original whole-object implementation fails this workflow as well as the
backend regression; rejection is never an accepted outcome for these free graphs.
Fault injection checks false rejection, a stationary attached chain, and a lost
joint. Every campaign round requires two completed whole-object movements.
`PIPESIM_WHOLE_ASSEMBLY_CASES`, `PIPESIM_WHOLE_ASSEMBLY_SEED`, and
`PIPESIM_WHOLE_ASSEMBLY_CASE_SEED` select fresh count, root, and individual seed.
Failures save replayable inputs under `fuzz-runs/whole-assembly`; set
`PIPESIM_WHOLE_ASSEMBLY_REPLAY` to the saved `replay.json` and run
`node --test --test-name-pattern='seeded whole assembly' tests/editor-interactions.test.mjs`.

Reinforcement closure is another mandatory matrix. It replays the saved
`4WaySwing` intermediate state: a duplicated arm already secured to the shared
top rail, with its bottom socket still open. Both pipe ends, world orientations,
reinforcement spacing, loose-collar starting stations, drag/drop pose intent,
and flexible-line layout modes vary. The real editor connection handler must
complete without Force, keep the arms separate, preserve world anchors, retain
every attachment and cut length, transport the hanging payload rigidly, and
support Undo/Redo. Fault checks reject errors, merged arms, missing joints, and
detached payloads. A campaign cannot pass without a completed reinforcement
connection in every round. `PIPESIM_REINFORCEMENT_CASES` (default 8),
`PIPESIM_REINFORCEMENT_SEED`, and `PIPESIM_REINFORCEMENT_CASE_SEED` control the
matrix. Failures save under `fuzz-runs/reinforcement`; replay with
`PIPESIM_REINFORCEMENT_REPLAY` and the test filter `seeded reinforcement`.

Failures are saved under `fuzz-runs/editor-workflows` (or
`PIPESIM_WORKFLOW_REPRO_DIR`). `replay.json` contains the starting document,
reference model, seed, and action prefix ending at the failure. `failure.json`
contains each preceding document, expected state, final document, error,
toasts, API request bodies, and responses. Replay files are self-contained; generation changes
do not change their actions. The campaign records the running seed before
launching each case, including if it is interrupted.

```powershell
# Replay a seed, or use its saved action sequence after changing the generator.
$env:PIPESIM_WORKFLOW_CASE_SEED = '2'
node --test --test-name-pattern='stateful geometry' tests/editor-interactions.test.mjs
Remove-Item Env:PIPESIM_WORKFLOW_CASE_SEED
$env:PIPESIM_WORKFLOW_REPLAY = 'fuzz-runs/editor-workflows/<failure>/replay.json'
node --test --test-name-pattern='stateful geometry' tests/editor-interactions.test.mjs
# Optional bounded reduction keeps construction and the same failure signature.
$env:PIPESIM_WORKFLOW_MINIMIZE = '1'
node --test --test-name-pattern='stateful geometry' tests/editor-interactions.test.mjs
Remove-Item Env:PIPESIM_WORKFLOW_REPLAY, Env:PIPESIM_WORKFLOW_MINIMIZE
```

`PIPESIM_WORKFLOW_SEED`, `PIPESIM_WORKFLOW_CASES`, and
`PIPESIM_WORKFLOW_STEPS` set the root seed, fresh-case count (bounded suite) or
total case limit (campaign), and additional actions per case. The mandatory
operations still run with zero additional actions. `PIPESIM_WORKFLOW_MINUTES`
sets campaign duration; it falls back to `PIPESIM_FUZZ_MINUTES` when present.
`PIPESIM_WORKFLOW_MINIMIZE_SECONDS` sets the reducer's default 180-second budget;
an in-progress case can finish after the deadline. Reduction never turns a
failure into a passing result or silently accepts a different failure.

### Abstract load attachments

The joint dialog attaches abstract loads without named ports. Choose a host part's
origin or any named attachment point; both parts retain their authored poses.
`port_connections.py` expresses the host pivot and axis in the load's local frame
and creates the selected fixed, hinge, or ball joint. Named host points locate
the pivot without occupying the corresponding physical port. The load's mass
and position continue to determine its gravity force and moment. Validation
permits its display envelope to overlap that declared host; other part pairs
retain their normal collision checks.

The editor suite includes four retained load workflows and twelve fresh cases.
They vary connector types (including connectors with only sockets), endpoint
order, mass, dimensions, position, rotation, and joint type, then check preview,
commit, coincident frames, unchanged poses, validation with overlapping display
envelopes, cancellation, undo/redo, reopening,
and detachment. Failures retain seed and action documents under
`pipesim-load-fuzz-repro-*`. Extend or replay with `PIPESIM_LOAD_FUZZ_CASES` and
`PIPESIM_LOAD_FUZZ_SEED`; `PIPESIM_LOAD_FUZZ_REPRO_DIR` sets the artifact parent.
The manual workflow runs 128 fresh load cases.

### Human editor workflow fuzzing

Human editor fuzzing runs with `npm run check`. Four fixed cases and twelve fresh
cases exercise whole-person translation under X and Y mirrors, explicit mirror
selection and automatic scene mirrors, upright and lying rotations, varied body
dimensions, and standing, seated and raised-arm poses. Each case uses repeated
translation-handle drags, cancellation, inspector edits, keyboard movement,
undo/redo and reopening. It checks the expected displacement and unchanged
rotation of every body segment, stationary surroundings, one undo entry per drag,
and rejected movement across the mirror. Cases also run physics, attach recordings
of varied lengths, and switch human activity through the inspector, checking
unchanged authored poses, updated joint behavior, stale-result removal after
success, and undo/redo. A retained case includes over one million ordinary recorded
values; the document guard budgets shared YAML alias expansion separately from
ordinary document size. These are assertions about the intended
result, beyond successful requests or document validation. Display widgets remain
substituted in the fuzz cases; the separate native-arrow regression covers picking
and drag calculations, while WebGL rendering remains outside this harness.

Failures print root/case seeds and retain the initial and current documents with
the action sequence in a temporary `pipesim-human-fuzz-repro-*` directory. Replay
or extend a campaign with:

```powershell
$env:PIPESIM_HUMAN_FUZZ_SEED = '123456789'
$env:PIPESIM_HUMAN_FUZZ_CASES = '128'
node --test --test-name-pattern='seeded human mirror' tests/editor-interactions.test.mjs
Remove-Item Env:PIPESIM_HUMAN_FUZZ_SEED, Env:PIPESIM_HUMAN_FUZZ_CASES
```

The manual Geometry fuzz workflow also runs 128 fresh human editor cases and
uploads failing workflow documents. `PIPESIM_HUMAN_FUZZ_REPRO_DIR` optionally sets
an existing parent directory for these retained reproductions.

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

The four extended fuzz campaigns run in the normal Python suite. Each runs for
fifteen minutes by default, or until its one-million-model limit is reached. Set
`PIPESIM_FUZZ_MINUTES` to change the time in minutes for each campaign. The
random root seed and each failing case seed are printed so a failure can be
reproduced. Supplying a root
seed repeats the entire sequence. For example:

```powershell
$env:PIPESIM_FUZZ_MINUTES = '20'
$env:PIPESIM_FUZZ_CASES = '1000000'
python -m pytest -s -q tests/test_solver_fuzz.py -k draft_repair_long_randomized
Remove-Item Env:PIPESIM_FUZZ_MINUTES, Env:PIPESIM_FUZZ_CASES
```

The draft repair campaign runs at most 25 models per child process. Connected
resize batches run at most ten seeds, each testing both a generated graph and
a six-edit lattice workflow; the mirror campaigns run at most ten seeds. Short budgets use smaller batches,
and completed-case speed sizes later batches near the deadline. A running case
can still take the campaign past its deadline. Geometry libraries can retain
native memory across models. Each batch reports its completed count and
elapsed time; a failing batch retains its generated designs under pytest's
temporary directory. The campaigns use new randomness by default; fixed seeds
are retained for reliable regression replay. The separate `Geometry fuzz`
GitHub Actions workflow can be started manually and has no daily schedule. On
failure it keeps the generated designs as artifacts; the log records the root
and failing case seeds. The million-case setting is an upper bound, not an
expected fifteen-minute throughput.

Browser verification additionally exercises the visible GUI: library loading, selection, lock/unlock body inference, edit dialogs, validation, structural response, simulation playback, save and printable instruction export. A browser screenshot is a layout check, not a numerical solver test.

`web/snapping.js` ranks projected connection candidates and aligns rotation axes. `web/snap-settings.js` validates browser defaults. `snapping.py` checks proposed body movements and returns connector/pipe alternatives without changing input; final poses are confirmed before becoming one undoable edit. It preserves anchors and joint freedoms rather than solving arbitrary inverse kinematics.

`sliding.py` adds an alternative that solves translations across the connected mechanism when a socket cannot be aligned by moving one rigid body alone. Existing joints, world anchors, slide limits and socket engagement constrain the solution. Without an anchor, a support body along the path between the connection sides provides the editing reference; the largest body on that path is preferred. Existing rotations and cut lengths are retained. The result passes through the same movement, joint rebasing, validation and collision checks as individual-body placement.

`posing.py` handles transform targets on `/api/move`. A rigid-body graph supplies forward kinematics for bounded hinge, slider, cylindrical and spherical coordinates. Additional anchors and loop closures enter the solve and every final relation is independently checked. Anatomical limbs first use the path from their pelvis or thorax. The GUI sends at most one preview request at a time, coalesces later pointer targets, and discards cancelled or stale responses. A separate rotation handle locates the actual hinge or pipe axis. Preview responses do not mutate the document; release rebases the accepted pose as one edit.

`preview.py` keeps bounded caches within each editor server: four parsed library sets, eight resolved documents, and up to sixteen prepared joint graphs and eight preview results per document. Scene resolution primes the document cache before the first drag. Keys include the complete document and its directory. Library lists, resolved asset paths, modification/creation timestamps, file size and file identity invalidate edited or replaced libraries/meshes; deleted files pass back through the normal asset checks. Source libraries are copied before document overrides are applied. Cached snapshots and mechanisms are read-only, and cache updates are synchronized across request threads.

API version 8 accepts `preview: true, pose_only: true` on movement requests. It returns checked poses and a solver seed without expanding objects, rebasing joints, hashing simulation inputs or generating a scene. Subsequent drag frames reuse the joint setup and seed. Part previews also return `preview_id`; release binds it to the original document, part, target and mode, rechecks the movement and materializes that exact pose. An expired or invalidated result falls back to solving the current inputs. Whole-object previews defer authoring work in the same way. Existing clients that only send `preview: true` still receive a document. Cancellation, failed commits and late responses leave the document and Undo history alone.

Run `python scripts/benchmark_preview.py --compare` for first-frame, changing-target and release timings over local HTTP, plus the former uncached/full-document pipeline. Results go to `output/preview-benchmark.json`; no designs are saved. `tests/test_preview.py` checks cache reuse, edit/file invalidation, directory and tab isolation, concurrent targets, saved joint state, result eviction, exact commits and legacy clients. Caching does not relax the numerical constraints; complex closed loops can still require more solving time than simple hinges or limb chains.

API version 9 adds `tolerance_mm` (default 2) and `force_options: {unlock_connectors: 0, resize_members: 0, max_length_change_mm: 20}` to `/api/snap-options`. `fit_adjustments.py` searches the nearest eligible connectors and members, preferring fewer edits. Only `force: true` enables the edit permissions. Each plan releases socket screws and represents each resizable member by start/end frames joined by a bounded virtual prismatic joint. The existing hinge/slide IK therefore solves poses and several cut lengths concurrently, including closed loops. Materialization converts these temporary frames into real parameterized geometry, rebases stations/travel/motors/tracks/drives, restores reusable objects and retightens screws. Virtual parts/joints never enter the saved design. Independently checked actual geometry supplies both collisions and `preview_parts` for the UI.

The bounded Force search considers up to 12 nearby connectors and 12 eligible members and at most 64 plans. World-anchored members, fixed meshes and members with custom centre-frame attachments are excluded from resizing. Existing travel and engagement bounds remain active while screws are released. A fit within the requested allowance is retained as a fallback while permitted edits are searched for an exact fit. `tests/test_fit_adjustments.py` covers the 1 mm frame, varied small loop length errors, simultaneous length changes, rotated ceiling mounting, limits, grouped objects, collision geometry, retained clearance and immutable previews. Editor tests cover input validation, proposed edits, cancellation, and atomic Undo/Redo.

API version 26 adds a per-request `job_id` to `/api/snap-options` and
`/api/connection-cancel`. Interactive searches have no wall-clock failure: their
finite plan, seed, contact-pass, and evaluation limits remain, and a cooperative
token stops numerical work. Early cancellation is retained until the matching
request starts; late results cannot commit. Cancel, modal close, Escape, and
superseding settings reach the server. Legacy Python callers without a token
retain bounded deadlines. Fitting restricts coordinates to paths between the
requested endpoints and world fixings, including overlapping loop closures;
payload loops hanging from a single articulation travel rigidly. Already-aligned
sockets use the validator's angular allowance and prefer small actual part
movements, preventing a near-square reinforcement from collapsing onto its
original arm merely to achieve a mathematically exact angle.

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
replay. The timed run uses up to 25 cases per child batch to limit native geometry memory
growth and reports each completed batch. To run it locally:

```powershell
$env:PIPESIM_FUZZ_MINUTES = '20'
$env:PIPESIM_RESIZE_REALWORLD_CASES = '1000000'
python -m pytest -s -q --basetemp=resize-fuzz-repro tests/test_resize_realworld.py -k resize_realworld_long
Remove-Item Env:PIPESIM_FUZZ_MINUTES, Env:PIPESIM_RESIZE_REALWORLD_CASES
```

For socket capture followed by free-end shrink across one or two scene mirrors,
run the separate long campaign. It varies both pipe ends, through and end
sockets, rigid scene orientation, and a fitting shared with another draft run.
Each case independently builds a valid target before checking the editor edit.
The timed run uses up to ten cases per child batch and preserves each case's sequence index.

```powershell
$env:PIPESIM_FUZZ_MINUTES = '20'
$env:PIPESIM_RESIZE_CAPTURE_CASES = '1000000'
python -m pytest -s -q --basetemp=capture-fuzz-repro tests/test_resize_capture_mirror.py -k long_generated_mirror_capture_and_shrink_sequences
Remove-Item Env:PIPESIM_FUZZ_MINUTES, Env:PIPESIM_RESIZE_CAPTURE_CASES
```

The deterministic cube regression starts with one TC128C corner, three pipes,
and three orthogonal scene mirrors. It requires materialization to produce eight
connectors, twelve pipes, and twenty-four distinct socket joints, followed by
exact validation with collisions enabled. The ordinary fuzz test varies all
three spans independently to make rectangular boxes, plus world position,
right-angle orientation, and mirror order. For a longer fresh-seed campaign
with replayable failure documents, the timed run uses up to ten seeds per child
batch to limit native geometry memory growth. Each seed also checks a saved
TC104C/TC128C mirror frame with a near-centered pipe and a mirrored TC173MC
through fitting under a random rigid orientation. These cases require exact
finalization with collision checking, rather than treating a reported collision
as an expected outcome. Each cube case also returns the finalized mirror
assembly to draft, edits a span, repairs it, and finalizes again:

```powershell
$env:PIPESIM_FUZZ_MINUTES = '20'
$env:PIPESIM_CUBE_FUZZ_CASES = '1000000'
python -m pytest -s -q --basetemp=cube-fuzz-repro tests/test_symmetry_cube_fuzz.py -k cube_long_randomized
Remove-Item Env:PIPESIM_FUZZ_MINUTES, Env:PIPESIM_CUBE_FUZZ_CASES
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
