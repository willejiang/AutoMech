# Compiler reference — mate → rigid transform (the placement geometry mate_solver implements)

This is the placement-geometry spec the `mate_solver.py` resolvers are built and tested
against. For each mate family it states which port frames coincide, the closed-form
transform, and the residual DOF. A port has a LOCAL frame `L` (a 4×4) whose +Z is the port
axis and whose origin is the port point (in meters). With the base part already at world
`T_base` and its port frame `L_b = T_base · L(base_port)`, the incoming part's world
transform is:

    T_incoming = L_b · R_flip · R_align(offset, angle) · L_c⁻¹

so that the incoming port frame lands on the base port frame (modified by flip/offset/angle).
This is the same closed-form resolve as `assembler._bridge_pose` (the subassembly-level mate
compiler) — byte-identical across runs, no iterative solver. Poses use the `sxyz` Euler
convention (`assembler._mat`/`_decompose`) that `mjcf_builder` and `urdf_builder` share, so a
solved pose round-trips through `build_mjcf` unchanged.

## Coaxial family (coaxial, revolute, cylindrical, press_fit, pin, key, *_bearing)

Align the incoming port axis onto the base port axis, coincide the origins, slide `offset_mm`
along the shared axis, roll `angle_rad`. `R_flip = I` (a shaft entering a bore points the
same way as the bore). In the port frame the shared axis is +Z, so the slide is a +Z
translation of `offset_mm/1000`. Residual DOF is carried by the part's `dof` (a `spin` shaft
keeps 1 rotational DOF), NOT by the mate.

## Face family (face_to_face, snap_fit, bolted, flanged, welded, planar)

Coincide the two face origins and ANTI-align their normals (`R_flip` = 180° about the
in-plane x axis) so the faces sit front-to-front. `offset_mm` opens a gap along the normal;
`angle_rad` rolls in the face plane. Fixed (0 DOF). Bolt-circle / hole-pattern parameters
(BCD, N bolts) fix the residual in-plane rotation via the 3-2-1 locating principle (a face
removes 3 DOF, the pattern removes the other 3).

## Coaxial-face / thread (coaxial_face, threaded, thread_engage)

Resolve orientation as coaxial (axes aligned), then set `offset_mm` so the incoming
face/shoulder seats on the base face plane. A thread additionally couples rotation and
advance by its lead if modeled.

## Gear family (the one special resolver)

A gear mate does NOT coincide ports — it places the incoming gear so the pitch circles are
tangent. Let `r_base`, `r_incoming` be the pitch radii (from the port's `pitch_radius_mm` or
`module·teeth/2`); `base_axis_w` and `base_center_w` come from the base gear's `teeth` port
frame; `sep` is `separation_axis` projected perpendicular to `base_axis_w`.

- **Parallel (`axis_angle_deg == 0`)** — spur / helical / internal:
  - incoming axis = base axis (parallel).
  - center distance `C = r_base + r_incoming` (external) or `r_base − r_incoming` (internal),
    in meters.
  - incoming center = `base_center_w + C · sep`.
  - roll by `angle_rad` (tooth phase; best-effort, pure-contact re-seats teeth).
  - `separation_axis` is REQUIRED when a gear meshes >1 peer, so each neighbor's direction is
    unambiguous. Backward graph traversal negates `separation_axis` (and `offset`), so a mesh
    resolves identically from either side.

- **Perpendicular (`axis_angle_deg == 90`)** — bevel / worm:
  - incoming axis = `base_axis × sep` (perpendicular to the base axis, in the base-axis/sep
    plane), so the two pitch cones share the apex at the axis intersection.
  - incoming center placed `C = r_base + r_incoming` along `sep`, plus `offset_e_mm` along the
    base axis for a worm's offset.

- Other angles → `MateSolveError` (only 0 and 90 are solved; helical/hypoid/crossed are
  future work). Adding a new gear geometry means adding its cone/offset math here AND to
  `precheck.gear_center_distance` (which today assumes parallel axes).

A gear / contact mate auto-registers `(base, incoming)` in `mesh_pairs` so the transmission
detector and the optional gear-ratio escape hatch (`mjcf_builder._add_gear_constraints`) see
it.

## The forest solver

1. Build LinkSpecs, infer + merge ports, parse mates. `root_part` → world identity; the root
   carries no pose.
2. BFS the undirected mate graph from the root. Each mate reaching an UNPLACED part resolves
   that part's world transform (base = the already-placed side).
3. A mate reaching an ALREADY-placed part is a CLOSING edge: recompute through it; if it
   disagrees with the existing transform by more than the position tolerance (2 mm) it is an
   OVER-CONSTRAINT (`MateSolveError`); if it agrees it is a redundant constraint (dropped).
4. Any part never reached is UNDER-CONSTRAINED (would float) → `MateSolveError` naming it.
5. Convert world transforms to a parent-relative PoseSpec forest
   (`T_rel = inv(T_parent) · T_child`, `_decompose` → xyz/rpy), so downstream
   `assembler._root_to_link` reconstructs exactly the solved world transforms and every gate +
   `build_mjcf` agrees with the solver.

## Invariants

- The mate graph must span all parts as ONE connected graph rooted at `root_part`.
- A gear meshing >1 peer must give each mesh a `separation_axis`.
- Solving is fully deterministic (matrix multiply + one `_decompose` per part); re-running on
  the same IR yields identical poses.
