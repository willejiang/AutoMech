# Boss: decomposing a machine into subassemblies

The boss splits a large machine into SUBASSEMBLIES (each one manager's job) and
authors the interface/frame contract that joins them. Aim for subassemblies that are
each buildable by one manager without overload.

## Sizing rule (the biggest quality lever)

A manager overloads past ~12 links per subassembly. When a subassembly's estimated
link budget exceeds that, carve it into two along a natural seam rather than handing
one manager a 20-link blob. Prefer more, smaller subassemblies over few large ones.

## Seam conventions

- A SEAM is where two subassemblies meet. Every seam is either a WELD (rigid join)
  or a gear-mesh / power coupling (motion crosses the seam by tooth contact).
- Author interface frames in GLOBAL coordinates. Each frame is a hard-point both
  sides agree on: the manager on each side must realize geometry so its frame lands
  exactly on that global coordinate (see the manager's <site> contract).
- Put the seam at a real mechanical interface (a mounting face, a bearing centre, a
  gear-mesh line) — not in the middle of a solid part.

## One-driver rule

The whole machine has ONE driven input (the crank/rotor/input gear). Assign the
driver to exactly one part in one subassembly; every other moving part turns only
because motion is transmitted across seams by contact. Do not give two
subassemblies independent drivers.

## Weld tree vs instances

- If several subassemblies are rigidly fixed to a common frame, that frame is the
  weld-tree root; children weld to it at their interface frames.
- If a subassembly repeats (four identical legs, N identical planet gears), define
  it once and instance it at different global poses rather than authoring N copies.

## Fixed hard-points prevent drift

Because the boss owns the global frame coordinates, echo those exact coordinates to
the manager as the sites it must hit. Managers that re-derive hard-points drift; the
boss's job is to remove that freedom by nailing the seam coordinates.
