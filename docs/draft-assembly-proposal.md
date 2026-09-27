# Draft assembly: graph-first pipe authoring

The core draft authoring and batch finalization path is implemented. See
[draft-assemblies.md](draft-assemblies.md) for the saved format, editor workflow,
and current pose-solver boundaries. The sequence below records the original
design goals; articulated motion across existing loose joints remains a
future extension.

## Finding

The current editor treats every pipe as a finished physical member from the
moment it is added. A catalogue pipe starts at 1000 mm. Its geometry, mass,
socket stations, collision shape and downstream analyses all depend on that
number. A new connection is therefore checked against the entire resolved
assembly. `Force` searches joint motion and, when allowed, combinations of
screw releases and bounded cut-length changes. It rebuilds physical geometry
and collision worlds for candidate solutions. On the 67-part SwingingPerch
design, rechecking an already aligned socket with `Force` took 5.3 seconds in
one local run. The profiled run spent 4.4 of 7.6 seconds in two collision checks,
mostly building static worlds. The search can have a 30-second budget and up to
64 adjustment plans. Repeating it for many edges is the wrong authoring loop.

## Proposed model

New pipes in the GUI begin as **draft runs**. A run records its catalogue
profile, two optional termination points, ordered through-socket attachments,
and any user-specified insertion, overhang or station constraints. Connector
nodes have editable poses; a run references their socket ports. The editor
derives a provisional centreline and displayed span from those poses. The
provisional span is a rendering value, not a locked cut length or a promise that
the assembly is physically buildable. A one-ended run can keep a free tip.

Draft subassemblies should be stored separately from finished `parts` and
`joints`, for example in a new `draft_subassemblies` document field. They can
reference finished connectors as boundary nodes. The resolved `Assembly` model
continues to represent only concrete parts with positive lengths. This avoids
making every geometry, mass, validation, physics, FEA and export path understand
an absent `length_mm`. Existing saved designs remain finished designs. The GUI
can create a draft run by default and offer an explicit **Lock cut length** for
users who know it early.

For interactive editing, connecting a run adds a graph relation immediately.
The browser updates the affected draft component's simple tube proxy and
connector markers without building a PyBullet collision world or invoking
`Assembly.from_doc` for every edge. A debounced local relaxation can improve
the displayed poses, seeded from the previous result. If two sockets are
non-collinear, point in incompatible directions, or are held by conflicting
anchors, changing pipe length alone cannot satisfy them. The draft accepts
the intended relation but marks its positional/angular residual visibly; it
must not display the link as an exact physical fit.

## Finalize subassembly

Finalization is one cancellable batch operation. It solves all unlocked
connector poses, pipe spans, through-socket stations and insertion depths in
the selected draft subassembly together, with fixed boundaries and user-locked
dimensions held exactly. It then materializes standard member `parts` and
socket `joints`, runs exact engagement, travel, collision and validation checks,
and commits the result as one undoable edit. A successful finalization yields
ordinary cut lengths for physics, FEA, cutting plans and export.

If the graph is overconstrained, finalization leaves the draft untouched and
reports the conflicting closures and their residuals in one review. It should
suggest specific releases or moved connectors, rather than sending the user
through a separate Force dialog for each edge. Ordinary connections with
consistent endpoint frames should finalize without Force. The existing Force
solver remains useful for exceptional local repairs and for old exact designs.

Through fittings are stations on one continuous run, not separate short pipe
edges. Their order, minimum engagement and bore-sharing rules must survive
draft editing and be checked at finalization. Mixed draft/finished assemblies
need clear analysis gates: a calculation using an unfinished component must
state that it has no exact cut lengths or physical geometry yet.

## Implementation sequence

1. Add a draft document schema and lightweight renderer for one- and two-ended
   runs, including through fittings and boundary references. Keep existing
   `parts`/`joints` semantics unchanged.
2. Add a graph-edit API and local visual update. Measure connection latency on
   a folded, 200-plus-part fixture; target a responsive edit (for example,
   under 150 ms at the 95th percentile on the development machine) without
   exact collision checks per click.
3. Add a batch materializer with analytic cut lengths for aligned runs, then a
   sparse, warm-started solve for movable connectors and closed paths. Report
   unresolved constraints per connection.
4. Add atomic finalization, Undo/Redo, persistence and analysis gates. Test
   that 100 draft connections can be made without Force, that a buildable loop
   finalizes to valid exact joints, and that an impossible anchored loop stays
   editable with a useful conflict report.

Caching collision worlds could shorten the existing exact path, but it would
not remove the per-connection global solve or the need to decide cut lengths
before the shape is settled.
