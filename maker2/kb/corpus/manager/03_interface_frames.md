# Interface frames and frames_realized (the <site> contract)

In hierarchy mode a BOSS splits a machine into subassemblies and hands each manager
an interface/frame contract: named mount frames in GLOBAL coordinates plus the seams
that join subassemblies. The manager must REALIZE each named frame — place real
geometry so the frame's world position matches the boss coordinate.

## Declaring a realized frame

Each interface frame is declared as a `<site name="frame_<name>" pos rpy>` INSIDE
the body that owns it. This is parseable and positional (cleaner than a comment
convention). The parser derives `frames_realized` deterministically:

- the site's parent body -> the `link` the frame is on,
- the site `pos` -> `local_xyz_m` (meters, relative to the body origin),
- the site `rpy` -> `local_rpy_rad`.

So `frames_realized` is a list of {frame_name, link, local_xyz_m, local_rpy_rad}.
The manager does not hand-write this list — it falls out of the sites.

## Self-consistency: a site's WORLD position must match the boss contract

After placement, the site's world position is (body world pose) composed with (site
local pose). It must equal the boss's declared GLOBAL frame coordinate within
tolerance. If a body is moved, the sites on it move too — re-check that the frames
still land on their contracted coordinates. A drifted frame is the single most
common subassembly failure; anchor geometry to the fixed hard-points, do not
re-derive them.

## Every named frame needs a site

If the contract names N interface frames, the skeleton has N `<site>` elements with
matching names. A missing site means the frame is unrealized — the assembler has no
hard-point to stitch that seam to.
