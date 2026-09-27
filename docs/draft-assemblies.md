# Draft pipe assemblies

New member profiles in the editor begin as draft runs. Their displayed span is a
working value. Connecting an end to a socket or passing a run through a fitting
records a graph relation and updates only that run's preview. The editor does
not run the exact connection search, collision world, or cut-length solver for
each draft click. Connected end sockets determine the displayed span; with one
end free, the preview length field changes it directly. **Lock cut length** makes
that value a constraint for finalization.
Through-fittings follow their current position along a draft pipe. Moving one
along the pipe does not leave a stale fit error, and Repair preserves that
position while correcting sideways or angular misalignment. Finalization
records the current station as an exact joint.

Draft runs live in `draft_subassemblies`, separate from physical `parts` and
`joints`. A draft catalogue member must have geometry driven by `$length_mm`.
Existing designs remain exact designs. For example:

```yaml
draft_subassemblies:
  - id: frame
    runs:
      - id: rail-1
        catalog: tubeclamp.tube-C
        parameters: {wall_mm: 3.2}
        start_mm: [0, 0, 100]
        end_mm: [1000, 0, 100]
        attachments:
          - {connector: tee-left, port: branch, end: start, insertion_mm: 20}
          - {connector: tee-middle, port: through}
          - {connector: tee-right, port: branch, end: end, insertion_mm: 20}
```

Select a draft pipe and use **Symmetry** in Properties to add up to three mirror
planes, one each for X, Y, and Z. The offset is measured from the scene origin;
X = 0, Y = 0, and Z at a chosen height are common. Reflected pipes and their
attached connectors update as the source is edited. They are translucent,
unselectable previews while the plane is active. A new connector added to a
design with one mirrored draft group also joins that group's previews.

For a pipe that crosses a plane, **Centered, perpendicular** pins its midpoint
to the plane. Changing its working span extends both sides equally; only one
physical pipe is made. **Centreline in plane** pins both ends of a pipe to the
plane at any angle. The editor selects these modes automatically when a pipe
is placed on a mirror plane or a mirror is added around an existing pipe.
Select **Free** to deliberately leave a pipe unconstrained; that choice is
remembered. Connections must already place an attached pipe on the chosen
plane before it can be pinned. Resizing a centered pipe with one attached end
moves its connected draft structure by half the span change, provided no
anchor or other mirror constraint holds that structure in place.
The file format stores planes on the draft group:

```yaml
draft_subassemblies:
  - id: frame
    mirrors:
      - id: left-right
        axis: y
        offset_mm: 0
        run_modes: {crossbar: centered, spine: in_plane}
    runs: [...]
```

A grouped reference human can use a vertical X or Y draft mirror as a pose
guide. Select the human, then choose the plane under **Mirror-line pose** in
Properties. The current horizontal position fixes a vertical centerline in
that plane. The whole person can slide up and down that line; moving an arm or
leg poses its opposite partner as a reflected copy. The back and head can
flex within the plane, while torso twists and head turns are held at zero.
Choose **Free pose** to remove the constraint. The saved line remains available
after the draft mirror is turned off or the draft is finalized.

**Turn off** offers **Discard copies**, which removes the reflected previews, or
**Keep copies**, which turns them into independent draft pipes and exact
connectors for asymmetric refinement. Finalization also keeps the reflected
geometry and creates its socket joints. A selected finalization includes the
reflected copies of the selected structure; other draft structures stay in
draft. Reflected fittings use their actual mirrored solid and socket locations,
including screw locations, rather than reusing an unmirrored catalogue shape.
An explicitly declared `mirror_catalog` counterpart may be used when its
socket frames match. Otherwise the editor writes mirrored mesh assets under
the current design's `assets/mirrored/` folder. Save As preserves references
to those files, so retain them when moving a design to another computer.

The editor previews residuals in orange. **Repair alignment · stay in draft**
uses the graph solver to move eligible free connectors and clear those residuals
without creating physical pipe parts or joints. It is one Undo step; an
unresolved or cancelled repair leaves the draft unchanged. A recorded relation
does not become an exact joint until the author finalizes it. The **Finalize
subassembly** button in a run's Properties processes only that run and draft
runs sharing fittings with it, even if unrelated runs are stored in the same
draft group. The top-level **Finalize draft** button processes all remaining
drafts. Finalization solves unlocked finished components as
rigid poses across the selected draft graph, preserving their existing joints
and holding world-anchored components fixed. It calculates cut lengths and
through stations, materializes standard members and socket joints, then runs
the exact validation and collision check once. A conflict leaves the draft
unchanged and lists the offending sockets and residuals. Finalization is one
Undo step and can be cancelled without committing a partial edit. Validation,
physics, structural analysis, build planning, and build
book export require draft subassemblies to be finalized first.
Dragging a draft pipe through several aligned, unused through sockets records
each fit. Finalization also recovers an aligned through fit missing from an
older draft, including when it joins runs stored in different draft groups.
A close but misaligned unused socket is reported as a conflict rather than
silently finalizing only part of the visible structure.

To edit a finished pipe quickly again, open its part menu or its Body menu in
the tree and choose **Return pipe(s) to draft**. A Body action converts its
catalogue pipe members together while leaving fittings and other Bodies in
place. Existing cut spans, pipe rotations, and socket attachments become draft
graph data; cut lengths become adjustable again. The conversion is one Undo
step. Pipes with anchors, loads, non-socket joints, or other references that
draft mode cannot preserve must have those constraints removed first.

The pose solver currently moves or rotates whole unanchored components of
directly editable finished parts. It does not articulate their existing loose
joints or rotate a world-anchored component. Those boundaries remain fixed, so
an impossible closure is reported instead of being silently forced. Image
rendering requires finalization; the editor viewport shows the draft proxies.
