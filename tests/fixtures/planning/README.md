`complicated-metal.json` preserves all 218 parts, 284 joints, anchors, socket
definitions and collision primitives from the reported `complicatedMetalRaw.json`.
The ten mirrored visual meshes are replaced by their authored collision primitives
and deduplicated into local definitions, so this fixture has no private asset paths.
Editor history and cached simulation results are omitted. The original design is
not modified.

`cube.json` is the 20-part, 24-joint closed cube produced by the existing draft
mirror materializer and finalizer from one TC128C corner. Its connector collision
primitives are stored locally, again avoiding generated STL dependencies. Tests
vary all three spans, rotate the complete cube, and shuffle part and joint order.

`tests/planning_cases.py` also generates shared-rail three-part rungs and finite
socket bores with remote obstacles. The latter includes solid convex bore proxies:
the declared bore permits only its own local contact, and must not hide another
plate on the same fitting. The fuzz suite preserves known failures and cycles
these topology families with fresh dimensions, ordering, names and orientations.

Additional generated cases cover rotated split clamps between closed stops,
floor-anchored cantilevers, connected groups that need an assisted turn onto a
stable floor face, and indivisible worktops above the preferred handling mass.
The independent replay checks already-fastened connectivity, each component's
actual ground-contact polygon and center of mass, declared handling weights,
and rigid pickup/transfer paths against other prepared groups. Explicit handling
limits remain hard; heavier default plans must declare assistance.
