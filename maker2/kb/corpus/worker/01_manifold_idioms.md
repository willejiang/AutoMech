# Worker: manifold-safe CadQuery / OpenSCAD idioms for mechanical parts

The worker builds ONE part alone in its local frame, with the part's attach point at
the local origin (0,0,0), and exports a single watertight (manifold) solid. The
validator rejects non-manifold meshes, so favor patterns that stay single-solid.

## General manifold rules

- Emit ONE connected solid per part. Never leave two disjoint lumps in one export
  (they read as non-manifold / multiple shells). If a part is conceptually two
  pieces, union them into one or make them separate parts.
- Do booleans BEFORE fillets/chamfers. Fillet edges that survive the boolean, not
  edges that a later cut removes — filleting then cutting through the fillet leaves
  slivers.
- Avoid zero-thickness walls and coincident faces. Two solids that share a full face
  (not overlapping) can produce a non-manifold seam; overlap them slightly, then
  union.
- Keep the part's natural axis on +Z and its origin where origin_note says. The
  assembler positions parts by the manager's poses; a wrong local origin misplaces
  the whole part.

## CadQuery idioms

- A gear blank: a cylinder of pitch-ish radius, then cut tooth profiles; keep it one
  solid. For contact sims a simple involute-approximation or trapezoidal tooth is
  enough — what matters is that outside diameter and tooth pitch match the module.
- A bore: cut a through-hole (`.hole(d)` or a cut cylinder) centered on the axis.
  Make the bore match the shaft radius plus a small clearance so the shaft turns.
- A shaft/axle: a single cylinder on +Z, origin at the bottom face.
- Fillets: select edges by a robust filter (by z-level or tag), not by index, so a
  small geometry change does not shift which edge gets filleted.

## OpenSCAD idioms (legacy fallback backend)

- Prefer `hull()` / `minkowski()` sparingly (they are slow and can bloat the mesh).
- Union overlapping primitives; difference the bore last; render at a modest `$fn`
  (e.g. 48-64) so the STL is manifold without exploding the triangle count.

## When a part won't build

If a build fails or is non-manifold, simplify the geometry (coarser teeth, drop a
decorative feature) rather than adding more booleans — the sim needs correct
bounding size and mating faces, not cosmetic detail.
