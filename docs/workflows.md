# Editor and worked workflows

## Build a frame in the editor

Each grouped object, part and **Body** entry in the design tree has a **⋯** actions
button. Use **Show properties**, **Rename…**, **Duplicate** where available, or
**Delete**. Delete removes the parts listed under that entry and
their attachments, loads, and dependent references. Parts outside the entry
stay in place. **Undo** restores the deletion in one step; **Redo** reapplies it.
Use the arrow keys to navigate the menu and **Esc** to dismiss it. **Delete**
while the actions button or a Body heading has focus deletes that tree entry.
Deleting an individual part of a reusable object still requires expanding it.

**Rename…** gives the selected entry a display name. Names are saved with the
design and support **Undo** and **Redo**. Internal IDs stay unchanged, preserving
connections and other references. You can rename a member inside a grouped object
without expanding it. A named Body follows its exact set of member parts; changing
which parts belong to the Body may give it a new default name.

**Drop to floor**, beneath **Position** in Properties, places the lowest point of the selected part or moving assembly on **Z = 0**. It lowers floating parts and lifts parts that intersect or sit below the plane. Rotated geometry, local mesh offsets and connected rigid parts are included. In whole-object mode it places the entire person or chain while preserving its pose. In limb/part mode it follows the same available joint motion as Position edits; a fixing or travel limit that prevents reaching the floor leaves the design unchanged. Each successful placement is one Undo step, and clicking again on an already resting part adds no edit. This positions the model on the plane; use simulation to check whether it balances there.

In **Validate → Findings**, single-click a finding to highlight its part. **Double-click**, or focus the finding and press **Enter**, to open its **Design → Properties**. The relevant connection or setting is brought into view: loose screws focus their Locked checkbox, and unsupported components lead to Position and Drop to floor. Grouped-part findings open the part's controls while keeping the object grouped. Design-wide findings without an editable part open Source instead. Navigation itself does not alter the design or add an Undo step.

Select a part and click **Duplicate** at the top of Properties to make one independent copy of that individual part, including when a human limb is selected. The arrow beside it opens three options:

- **Duplicate N copies…** opens a popup for 1–100 new copies, with a choice of what to include in each copy.
- **Duplicate directly touching parts** copies the selected part and its immediate socket/joint neighbours, including loose connections. It stops after one connection; incidental mesh overlaps do not count as attachments.
- **Duplicate entire subassembly** follows locked connections and stops at loose or articulated joints. For a part inside a grouped object, such as a human, it copies that complete object with its internal articulation and saved posture. Regroup an expanded human first to copy it as one object.

Copies keep their dimensions, material, orientation and internal joints, including motor settings, joint state, animation tracks and internal drives. World fixings and external attachments stay with the original. New copies receive unique IDs and are spaced beside the original; the last copy is selected for repositioning. One Undo removes all copies from the action, and Redo restores them.

Connecting pipes also solves available hinge rotations and slides together when moving one rigid body or using translation alone cannot align the socket. Rotation snap increments apply to your drag target; the connection solver uses continuous angles to find the actual fit. The preview shows every moved part, and **Connect** commits the fitting and the associated movement as one undoable edit.

**Connection tolerance** in **Align and connect** defaults to **2 mm**, with a range of 0.01–20 mm. The solver still seeks an exact fit; the allowance permits a small remaining distance error and is saved on the connection. The preview reports any accepted gap above 0.03 mm. Actual pipe engagement, tube size, socket occupancy, angular alignment and collisions are still checked. A larger allowance does not make a pipe intersecting the fitting wall a valid connection.

**Force** searches more starting poses, including simultaneous movement around closed loops. It may adjust the new socket's position along the pipe, or its insertion within the available engagement; the fitted value appears in the field and preview description. Expand **Force options** to allow loosening and retightening up to **N connectors** (0–6), or resizing up to **N pipes** (0–4) by at most **M mm each**. Both counts start at zero. These permissions apply when you press Force; with edits enabled, it prefers an exact fit over accepting the permitted gap. It can solve several cut lengths and joint motions together. New cut lengths may be longer or shorter; the preview lists the original and proposed lengths, and every socket that will be loosened and retightened. World anchors, joint travel, socket engagement and clearances remain constraints.

**Connect** applies the reviewed changes as one Undo step. **Cancel** discards them, including a search still running. A bounded search that finds no fit leaves the design intact and reports the remaining mismatch or limiting connections; this is not a proof that no assembly is possible. Run build planning again after accepting assembly adjustments.

Try **Examples → Assembly adjustments - one millimetre gap** (`examples/assembly-adjustments.pipe.yaml`). Connect `bottom-arm` to `elbow / x`, using its **End** with 30 mm insertion. The locked frame accepts its 1 mm residual at the default tolerance. For an exact fit, enable one connector release and press Force, or enable one pipe resize with a 2 mm maximum. The latter shortens `left-arm` from 401 to 400 mm. You can also reduce the tolerance to 0.03 mm to reproduce the blocked unadjusted fit.

Try **Examples → Hinged triangle** (`examples/hinged-triangle.pipe.yaml`): a square carries three 1000 mm pipes with six tees. Connect `top-bar` to `upper-back / through`, leave its screw loose, then connect `upper-left / through`. The differing tee offsets require fractional-degree rotations and concurrent slides. The last connection adjusts the earlier joints while the square stays in place.

Changing **Length mm** in Properties moves attached connectors and their connected branches. End sockets keep their insertion depth; locked through sockets keep their distance from the pipe's start. An unanchored member keeps its centre, with its ends moving equally. Floor, wall and ceiling anchors remain fixed, so the member can shift to keep an anchored end in place. Other cut lengths and current angles are preserved. Loose sockets can absorb axial movement within their travel and engagement limits.

If a square or another constrained frame cannot follow the change, **Length change blocked** keeps the original design and identifies the blocking connections and anchors. **View blocking constraints → Show connection** selects the fitting in Properties. Where a checked solution exists, **Loosen and resize** or **Disconnect and resize** applies the named release and length change together as one undoable edit. A larger shortening may require disconnecting a socket because simply loosening its screw would leave too little pipe engagement. You can also keep the current length and adjust the frame yourself. Resizing checks final fit and new collisions; run build planning again for the assembly sequence.

1. Open **Examples → workbench** to inspect a complete design, or **New design** for an empty one.
2. Filter the library by type and tube diameter. Click a component to add it, or drag it onto the floor or an existing part in the viewport.
3. Drag a part's body directly to move it in the camera plane. Use **G** for axis handles, **R** for rotation, **Q** for selection, and **F** to fit the view. Numeric position/rotation fields and dimension parameters provide precise edits. **Grid** toggles position and rotation increments, initially 10 mm and 90°. **Connections** separately toggles connection snapping; it starts enabled. The gear beside these buttons opens **Snap settings**.
4. Drag a connector onto a pipe, or a pipe onto a socket. Nearby openings are found in screen space, including targets at a different camera depth. A green preview marks the proposed alignment. One clearly aligned match connects on release. Misaligned or ambiguous matches open **Align and connect**, with socket selection, a rotatable preview, and a choice of which connected body to move. The connector is preferred; world anchors and existing joints can rule out a movement. **Cancel** restores the original placement; **Place only** keeps an allowed drag without adding a connection. Occupied sockets, shared bores and incompatible tube sizes are excluded. You can also select a member, choose **Connect to a socket**, and click a socket marker or connector to open the same preview. **Esc** exits the connection tool or cancels a drag. Snapping preserves cut lengths unless you explicitly enable and accept a Force resize.
5. The **Locked** checkbox changes the actual constraint graph. A locked socket becomes part of a rigid compound. An unlocked round socket can slide and twist. The Design tree is inferred from these connections.
6. **Edit joint** on a socket opens the guided placement preview, including the pipe station or insertion depth. Other joints expose axes, frames, limits, torque, motor and failure thresholds as editable JSON. **Add joint or attachment** supports arbitrary rigid-body connections. Advanced socket limits and hardware notes remain editable in **Source**.
7. **Fixed to the world** declares a real wall, floor, ceiling or fixture mount. Select a suitable anchor system outside PipeSim; the label does not determine its physical strength.
8. Run **Validate**, then calculate structural response. The GUI distinguishes malformed connections, overlaps, unsupported bodies and assumed geometry.
9. Run **Simulate** to release the structure under gravity. Scrub or play the recording; **Restore authored pose** returns to editing geometry. **Capture frame** makes the current pose a new editable reference and rebases limits.
10. In **Build instructions**, find an assembly order and inspect individual steps. Export the illustrated book. **Save** writes YAML/JSON to a named path inside the workspace.

**Load** lists YAML and JSON files saved under the workspace's `designs/` folder, including subfolders. Search by filename or path, then click a design to open it. **Load → Open file…**, **Ctrl+O** (Command+O on macOS), or dropping a design file anywhere in the editor opens a file from your computer. The file is checked before replacing the current design. Unsaved changes offer **Cancel**, **Open without saving**, or **Save and open**; a failed save keeps the current design open.

Files opened from the OS stay in memory until you **Save** them into the workspace. Their suggested path is `designs/<filename>`; an imported suffix avoids an existing filename. Browsers supply one file's contents, so companion libraries and meshes must already be available in the workspace. File-open references resolve relative to `designs/`. For a design with companion assets, copy its whole folder into `designs/` and use **Load** to preserve its relative paths. A missing asset produces an error without replacing the current design.

Undo/redo applies to edits within the current design and starts afresh when another file opens. Saving is explicit; closing a page with edits prompts through the browser's unsaved-change mechanism. The server binds to loopback and rejects cross-origin mutation requests and paths outside its configured root. Use a dedicated workspace root when sharing a machine.

When updating PipeSim's code, save your work, restart the editor server, then refresh the page. The Python server keeps its loaded code until restarted, while the page's JavaScript is read from disk. The editor detects an older server and explains the required restart. If the server restarts while a page remains open, its next operation refreshes the session token without replacing the open design, camera or undo history.

The editor follows the system/browser light or dark appearance, including dialogs, native controls and the 3D background, floor and grid. Changing the system preference updates an open editor immediately.

For a tee in the middle of a pipe, choose its **through** socket or **Main run · continuous pipe**. **Socket centre from pipe start** is measured along the pipe: 500 mm places the socket centre at the midpoint of a 1000 mm pipe, independent of pipe rotation. **Depth inside socket** applies only to a terminated pipe end and is limited to the available engagement. TC104 also has two end sockets for joining separate tubes; those end sockets and the continuous run share one bore and cannot be occupied simultaneously. See **Examples → tee midpoint** for the three-pipe I frame. Editing an older TC104 connection with an excessive insertion offers the continuous-run mode with that value as its station; applying the preview repairs it.

**Snap settings** provides 90° frame, 45° brace and 15° fine presets, plus custom position/angle increments, connection reach in screen pixels, and an angular window for alignment. Rotation can match axes of existing parts or the world while respecting the active rotation handle. A nearby reference takes priority over the increment; disable reference alignment to use increments alone. **Apply** changes this session. **Save as defaults** also stores the settings in this browser for future visits to this editor address. Preferences do not change the design file or its undo history.

Dragging and numeric position/rotation edits solve the connected mechanism. Locked parts move together; loose sockets slide and turn on their pipe, and articulated joints bring neighbouring parts along. For a hinge or cylindrical connection, the rotation handle sits on the joint axis, including the offset bore of a crossover. Joint limits and socket engagement stop a handle at its available travel. The viewport previews these changes during dragging; release creates one undo entry, and **Esc** restores the original pose even if a preview request is still running.

The editor reuses the loaded design and joint setup during a drag. Preview frames only calculate movement; saving the edited object and joint settings happens on release. This preparation is automatic and refreshes when the design or its library/mesh files change. **Esc** also cancels a released placement while it is still being processed.

Select a human and choose **Move whole person** to translate or rotate all its parts together, including large moves and flips. Its posture remains intact. Choose **Pose limbs**, select a shin, thigh, hand or other segment, then drag it or use **G**/**R**. Limb movement starts from the pelvis or thorax, so moving a shin adjusts the hip and knee and carries its foot along. Existing hand grips and world anchors stay connected. A posed person stays one articulated object in the outline; expand its **Individual parts** entry to select segments.

**Connections to structure** lists the person's grips and mounts. **Detach** releases a connection without separating the skeleton; **Preview reconnect** reaches the saved point and reconnects after review. **Attach body part to structure** offers body/target selectors, a pipe station or named port, and fixed, pivoting or ball-joint attachments. Its preview respects the other grips and available joint travel. If the point cannot be reached, pose the person closer first. Remove external attachments before translating the entire person away from the structure.

**Expand into editable parts** exposes individual geometry and joint definitions. Select any expanded segment and choose **Regroup as object** to restore the wrapper while retaining the edited pose, dimensions, mass, muscle settings and attachments. Older expanded humans with their original anatomical names also support regrouping. Undo and Redo include the entire operation. Regrouping does not lock any physical joint.

A saved joint-state pose is captured as the editing reference when needed. Position motor targets, remaining travel and authored motion tracks rebase with the edit. Posing is kinematic: use **Validate** for intersections and **Simulate** for contact, weight and forces. **Build instructions → Find assembly order** checks a physically possible insertion sequence.

When a connection needs motion on both sides, **Align and connect** also offers **Slide connected bodies together**. For example, two tees on perpendicular rails can slide their inner arms into an elbow together. This follows existing sliding joints, including loose sockets between a tee and its arm, while keeping the shared support frame in place. The preview shows the moving parts and their rails; **Connect** applies their new positions and the new socket connection as one undoable edit. Existing screws keep their lock settings, and slide limits, engagement and collisions are checked. A drag that needs this coordinated movement opens the preview before connecting.

The editor intentionally keeps advanced mechanisms in a structured joint editor and the complete Source dialog. It does not yet provide a sketcher, parametric history tree, surface modeller or native desktop packaging.

## Generate and cancel a simulation

In **Simulate**, set the duration and **Chain links per rigid body**, then choose
**Run simulation**. The default of **1** retains each flexible link. Larger values
(for example **10**) freeze runs of up to that many links at their starting pose.
All visible links, their masses and collision shapes remain in the recording.
Anchored links and links with external attachments remain separate; motors,
breakable connections and driven joints remain active. This option changes only
the simulation, not the editable design. Grouped chains are stiffer; internal
joint reactions are unavailable and contacts are attributed to a whole group.
Use full flexibility for detailed chain behaviour and force checks.

Progress shows geometry preparation, physics loading, simulated time, percentage,
elapsed wall time and an estimated remaining integration time. Setup has no time
estimate. The server also prints flushed progress lines to its console. **Cancel
simulation** stops the worker even during a native solver call; the previous
recording remains available. A failed or cancelled run is not saved as a result.
One simulation can run per editor server. Closing the server stops its worker.

Simulation errors remain visible in the Simulate panel until the next run or
design edit. Sliding connections can close a physical loop: for example, a
carriage can slide on two parallel rails, and coaxial loose sockets can support
a swinging assembly. Both connections remain active, including their travel
limits. A closed connection loop is a path through the mechanism, not an
infinite program loop. Fixing a rail to the world does not lock its fittings:
lock structural sockets in **Design → Properties → Connections** and leave the
intended moving bearings unlocked. Some rotational loops with motors or limits
still require a passive hinge or ball-joint closure; an error identifies those
unsupported connections before simulation starts.

For longer runs from the command line:

```powershell
python -m pipesim simulate designs/test1.yaml --duration 3 --chain-links-per-body 10 -o output/simulation.json
```

Press **Ctrl+C** to cancel. The CLI exits with code 130 and does not publish a
partial recording. Progress goes to stdout when `-o` is supplied, and stderr when
stdout contains the recording JSON. `--quiet` hides progress.

Python callers can supply a progress callback and a cancellation predicate:

```python
from threading import Event
from pipesim.document import Assembly
from pipesim.physics import simulate
from pipesim.simulation_control import console_progress, SimulationCancelled

stop = Event()  # Another thread can call stop.set().
try:
    recording = simulate(Assembly.load("examples/chain.pipe.yaml"), duration=3,
                         chain_links_per_body=10, progress=console_progress,
                         cancelled=stop.is_set)
except SimulationCancelled:
    print("Cancelled; previous recording retained")
```

Direct Python cancellation is checked between setup operations and integration
steps. The editor and CLI use a separate process to allow cancellation during
native calls too. Custom progress callbacks receive a dictionary with `phase`,
`message`, `elapsed_s` and, during integration, `simulated_s`, `duration_s`,
`percent`, `completed_steps`, `total_steps`, and `eta_s` once a step has completed.

## Import a new connector

Use **Import a 3D part**, supply its mass and unit scale, then add the imported catalogue entry. Edit its library definition to specify local ports, source URL, measured dimensions, COM/inertia, collision primitives and capacities. Overrides are stored in the design's `definitions`; a source library can also be edited directly in a text editor.

One detailed mesh can have a simpler compound collision model. A hollow fitting needs bore-preserving collision primitives: a convex hull would plug its socket. The bundled tube fittings use annular segments for this reason.

For a shareable directory:

```powershell
python -m pipesim bundle my-design.pipe.yaml -o output/my-design-bundle
```

Move the entire output folder. Its catalogue and meshes no longer depend on the original paths. A standalone JSON download from the editor preserves library references; use a bundle when those assets must travel too.

## Lay out a chain

Choose **Add a chain** in the library, or click or drop a chain-link catalogue entry. Set the required length in millimetres; the editor creates the links and their spherical joints together. **Set chain length** in Properties grows or trims the end of the chain. The displayed link count and effective length round up to complete links using the selected library part's attachment pitch.

**Move whole chain** preserves the current shape during translation and rotation. The default chain contributes one group to layout solvers and one collapsed tree entry. **Pose links** enables shaping by dragging or rotating a link; use **Select start**, **Select end**, or the numbered **Select link** field without expanding the tree. **Individual links** opens the list on demand. Switch back to **Move whole chain** to hold the edited shape for layout. Long standard chains use a dedicated link projection solver; custom edited mechanisms can fall back to general joint solving.

**Attach start** and **Attach end** open a preview for connecting a link eye to a hook, shackle, bar station or other part. The second attachment can pose the chain while retaining the first attachment. Cancel leaves the design alone. Detach and reconnect controls work as they do for grouped humans. Shortening preserves retained links, their poses and connections; if it would remove an attached or loaded link, the error names the attachment, anchor or load to release first. Links added at the end follow its existing direction.

Simulation always uses the individual articulated links, including when the editor holds the chain's shape. **Capture frame** keeps the chain grouped and retains its length controls. Expand/regroup and duplication of the entire subassembly also preserve the chain object. Each length change or pose is one undoable edit.

Try **Examples → chain** for a tilted chain with its first link fixed to the ceiling. Release that anchor to move the whole chain. The bundled link remains a generic ring proxy with assumed joint limits; the chain generator does not assign a supplier load rating.

## A motor-driven carriage

```powershell
python -m pipesim simulate examples/gt2-stage.pipe.yaml --duration 3.5 --fps 30 -o output/gt2.json
python -m pipesim animate examples/gt2-stage.pipe.yaml --simulation output/gt2.json --eye 1100 -1500 900 --target 0 0 140 --lighting flat -o output/gt2.mp4
```

The 20-tooth, 2 mm-pitch driver advances 40 mm per revolution. Five turns command 200 mm of carriage travel; finite motor torque, belt compliance, damping and applied loads determine the actual response. Change the motor schedule or add a carriage load in Source to experiment.

## Check and render a movement range

```powershell
python -m pipesim motion-check examples/sliding-collar.pipe.yaml --joint loose-screw --samples 61 -o output/range.json
python -m pipesim animate examples/sliding-collar.pipe.yaml --joint loose-screw --valid-only --fps 10 --lighting technical -o output/valid-collar.gif
```

The check reports valid sampled intervals and the colliding or disengaged poses. `--valid-only` selects the longest contiguous valid sampled interval. It does not splice disconnected intervals together. To prove passage through a narrow gap, increase samples and independently inspect the physical clearances. This check is kinematic; use simulation for forces, impacts and gravity.

## Human fit and posture

```powershell
python -m pipesim human --height 1800 --mass 85 --pose seated -o output/person.pipe.yaml
python -m pipesim fit examples/human.pipe.yaml --human person --hand right --target 231 350 1100 -o output/reach.json
python -m pipesim fit examples/seated-human.pipe.yaml -o output/fit-tests.json
```

The last command runs design tests, including an intentionally unreachable target with `expect: false`. A test passes when the observed result matches the expectation.

Start with the person's measured shoulder/hip widths and limb lengths. The reach solver moves the shoulder, elbow and wrist within bounds and checks the resulting arm against the environment and other body segments. It reports the actual palm position, error, angles and colliding parts. A reported pose does not imply a collision-free trajectory to it.

Seat checks report available width/depth, a thigh-support allowance, seat height, estimated popliteal height, current foot gaps and pelvis gap. `fits` is dimensional screening, not a comfort or stability verdict. Place the seated model, inspect contacts and run dynamics. For an active seated pose, set `hold_pose: true`; the finite posture torques are assumptions and can be scaled. Passive ragdoll mode remains the default.

### Two-hand pull-up with free legs

Open **Examples → human pull up**, choose **Simulate**, then **Run simulation**. The example requests four seconds; use the timeline to play or scrub the result. Its 59 parts form a Tubeclamp C42 cage with overhead ladder rungs and a 19-segment person inspired by [this reference](https://in.pinterest.com/pin/808466570608639102/). Dimensions were chosen for the example, and its four floor flanges are explicitly anchored.

Both hands have curled grip geometry and revolute attachments to the front bar. Upper-body posture controllers hold the bent-arm pose with finite torque. Hips, knees and ankles have limits and passive damping, with no motors. Passive joint damping is an explicit assumption of 1 N·m·s/rad per axis. Small initial leg bends produce swinging under gravity; there are no animation tracks. Select the person in Design mode to change **Posture control**, **Strength scale**, or individual held joints. The **human pull up relaxed** companion uses the same starting pose and grips with every posture motor disabled, letting the person drop towards a hanging pose.

```powershell
python -m pipesim simulate examples/human-pull-up.pipe.yaml --duration 4 --fps 30 -o output/pull-up.json
python -m pipesim animate examples/human-pull-up.pipe.yaml --simulation output/pull-up.json --width 960 --height 900 --eye 3000 -4300 2750 --target 0 0 1250 -o output/pull-up.mp4
python -m pipesim simulate examples/human-pull-up-relaxed.pipe.yaml --duration 4 -o output/relaxed.json
python -m pipesim human --pose pull-up --hold-joints upper_body --strength-scale 6 --grip-diameter 42.4 --joint-damping 1 -o output/gripping-person.pipe.yaml
```

The last command creates an editable standalone person; it does not position a frame or attach the hands. Add `--record` to a simulation command to embed its results in that design, or save after simulating in the UI. The grips assume no slip, fatigue or release. Posture torque capacities are adjustable assumptions, and the model does not estimate muscle physiology, injury or equipment safety.

## Turn body contact into a structural load case

```powershell
python -m pipesim simulate examples/contact-load.pipe.yaml --duration 1.5 -o output/contact.json
python -m pipesim contact-loads examples/contact-load.pipe.yaml --simulation output/contact.json --window 0.25 -o output/contact-case.pipe.yaml
python -m pipesim analyse output/contact-case.pipe.yaml -o output/contact-analysis.json
```

The 20 kg load settles onto the bench. Contact force is averaged over the last 0.25 s; the frame case removes the free payload and replaces its contact by a force/moment resultant on the contacted structural component. Its own mass is therefore not counted twice. `--parts` can select the structural seed parts explicitly; fixed attachments are included automatically.

Use the same process for a human on a beam-supported seat. The conversion retains real anchors and reports each transferred wrench. It does not invent ground clamps, and it does not replace distributed local loads with a promise of accurate local panel bending.

## Multiaxial load trials

```powershell
python -m pipesim stress examples/cantilever.pipe.yaml --part beam --max-force 10000 --directions 12 --steps 8 --seed 4 -o output/beam-trials.json
python -m pipesim stress my-design.pipe.yaml --part handle --max-force 1000 --directions 12 --steps 8 --rigid --dwell 1 --record
```

The directions include ±X, ±Y and ±Z plus seeded random unit vectors. Static trials bracket the first predicted yield, buckling or conditional axial-slip event. `--rigid` also runs independent increasing load cases for the specified dwell and records tipping, movement and explicitly configured joint failure. “No event” describes that duration and model; it does not establish an ultimate capacity.

## Rendering and assembly output

```powershell
python -m pipesim render examples/workbench.pipe.yaml --width 1600 --height 1000 --eye 2000 -2500 1800 --target 0 0 500 --background transparent --orthographic --lighting technical -o output/bench.png
python -m pipesim mesh examples/workbench.pipe.yaml -o output/bench.glb
python -m pipesim bom examples/workbench.pipe.yaml --kerf 3 --trim 5 --stock 6000 -o output/cuts.json
python -m pipesim build examples/workbench.pipe.yaml --engineering -o output/build
```

Camera options also include yaw, pitch and distance. Lighting modes are `studio`, `flat` and `technical`. Video uses the same options. GIF needs no external encoder; MP4 uses FFmpeg/libx264 with even image dimensions. A suffix-free animation destination produces numbered PNG frames.

Build export refuses an invalid design or an unproven assembly sequence. A failed search writes `planning-report.json`; inspect blocking parts, introduce a real split connector, change the sequence or declare a physical temporary support. The planner never silently undoes a previously completed joint.
