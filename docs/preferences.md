# Editor preferences

Preferences are browser local. **Apply** changes the current editor session; **Save as defaults** also stores the choices for the next visit. The design file remains independent of these choices. PipeSim continues to store and calculate lengths in mm, masses in kg, and forces in N.

The panel exposes assumptions previously embedded in the editor:

| Area | Previous assumption | Preference |
| --- | --- | --- |
| Startup | Open the server's workspace design | Last saved design or workspace default, latest autosave, empty design, or an example |
| Startup view | Fit the design, start with Select | Fit on open and initial tool |
| Recovery | No automatic recovery copies | Autosave enabled by default, workspace folder, minutes between edits, copies per design |
| Units | Show mm, kg, N | Length (mm/cm/m/in/ft), mass (kg/g/lb), force (N/kN/lbf) display selection |
| View | Grid and floor on, ports off, system viewport theme, 38° field of view | Grid, floor, ports, theme, field of view, fit after duplicate |
| History | 80 undo snapshots | Undo limit |
| New items | 2 duplicate copies, 1750 mm/75 kg person, 1000 mm flexible line | Initial count, stature, mass, and line length |
| Connections | Lock new sockets, 2 mm fit tolerance | Initial lock and tolerance in the connection dialog |
| Render | 1600 × 1000, studio light, soft grey | Image size, lighting, background |
| Simulation | 3 seconds, one chain link per rigid body, 10 mm deflection warning | Duration, chain grouping, and yellow displacement threshold |
| Interaction | 10 mm grid, 90° rotation, 30 px connection reach | Snapping and connection controls remain in the same panel |
| Keyboard nudges | 100 mm movement and 15° rotation | Dedicated distance and angle controls in Preferences, saved with snapping defaults |
| Length editing | Drag the two ends, carry connected fittings, and connect newly covered through sockets | Default and Shift connector behavior, socket capture window, keyboard step, and four shortcuts |

Autosaves are JSON recovery records kept separately from normal designs. The Load dialog lists them, and the startup choice can restore the most recent one. Opening a recovery copy marks the design as unsaved so Save is still needed to keep that version. Retention applies per source design within the selected folder.

Display unit conversion covers the status bar, part mass, editable inspector lengths, and flexible-line force thresholds. Technical dialogs with an explicit `mm`, `kg`, or `N` label continue to accept the labelled unit. Source editing, exported files, and server APIs always use canonical units.

Physics constants, catalogue dimensions, connector tolerances declared by a part, and document-specific settings are model data rather than editor preferences. They remain in the design or library where other tools can read them consistently.

The displacement warning is a display threshold in millimetres; it does not change stiffness or strength. Structural analysis tints a member yellow above that displacement and red when the static load case predicts yielding or buckling. Simulation playback measures bending relative to the beam's overall motion, then tints red after dynamic yielding or possible fracture. Contact with the floor can prevent damage predicted by a contact-free linear analysis. Red takes priority over yellow.

In Design mode, select a pipe or flexible line and drag either gold end handle, including while the Select, Move, or Rotate tool is active. A background drag still controls the camera. The opposite end stays in place; a pipe centered on a mirror plane adjusts both ends equally to remain centered. Extending a free end keeps existing connectors seated. Shortening carries connectors as their constraints allow, while retaining body clearance and socket engagement. Holding Shift leaves connectors behind and disconnects those no longer on the shortened pipe. Extending a pipe into a compatible free through socket connects it within the configured distance and angle window. The default keyboard controls are **Y/H** to grow/shrink one end and **U/J** to grow/shrink the other; all four letters and the step are editable in Preferences. For chain, rope, and strap objects, each key press changes at least one visible segment when possible. Undo restores each completed resize as one action.
