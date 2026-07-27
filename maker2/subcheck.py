"""Per-subassembly rigid-conflict detection.

Runs RIGHT AFTER a subassembly's worker finishes (in orchestrator_boss.
_finish_subassembly), on that ONE sub's own model.urdf — long before the whole-machine
precheck. It reuses precheck's world-mesh primitives, but on the sub's UN-namespaced link
names (precheck._part_overlaps works on the assembled, namespaced robot; here every link
is just its own `l.name`).

Why this exists: the manager authors every part's placement (PoseSpec.xyz_m/rpy_rad) in
one blind LLM call with no geometric feedback, while the worker decides the actual solid
geometry at the part's local origin. When the two disagree, parts the manager thought
were clear interpenetrate. This detector gives the debugger the exact offending pair so
it can fix the pose or the geometry before the sub is accepted.

Overlap is measured on the REAL mesh solids (precheck._solid_intersection_frac, a
manifold boolean), NOT bounding boxes: a hollow part's bore/keyway/cut is empty, so a
shaft in a bearing bore, a key in a keyway, a bearing pressed into a housing bore, a plug
through a hole all read ~0 overlap — only genuinely interpenetrating metal scores high.
This is what lets the gate be strict (fail a sub on a real conflict) without mis-flagging
the many intended nestings that fill a machine.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .precheck import _OVERLAP_FRAC, _link_world_mesh, _solid_intersection_frac
from .viz import load_robot


@dataclass
class Conflict:
    """Two rigid parts in one subassembly that grossly interpenetrate."""

    part_a: str
    part_b: str
    frac: float                                     # shared/smaller-part SOLID volume, [0,1]

    def describe(self) -> str:
        return (f"parts '{self.part_a}' and '{self.part_b}' interpenetrate "
                f"({self.frac:.0%} of the smaller part's solid is inside the other)")


@dataclass
class Floating:
    """A fixed structure part with no physical support under it — held up only by its
    weld, so in the real machine it would have nothing to rest on."""

    part: str
    bottom_mm: float                                # world Z of the part's lowest point
    gap_mm: float                                   # clear gap below it to the next solid

    def describe(self) -> str:
        return (f"fixed part '{self.part}' floats: its underside sits {self.bottom_mm:.1f}mm "
                f"above the base with a {self.gap_mm:.1f}mm empty gap below it — nothing "
                f"physically supports it, so it is held up only by its weld. Rest it on the "
                f"part beneath it or lower its Z onto real support.")


# A part whose underside clears everything below it by more than this (and does not sit on
# the ground) is unsupported. 2mm tolerates a modeling seam / small clearance fit.
_SUPPORT_GAP_MM = 2.0
_GROUND_TOL_MM = 2.0


def floating_parts(model, meshes_dir: str, log_fn=print) -> list[Floating]:
    """Flag fixed STRUCTURE parts that are geometrically unsupported: their underside has
    an empty gap below it down to the next solid AND they do not rest on the ground. This
    catches a part welded floating in mid-air (a bridge/post/plate with nothing under it)
    without running physics.

    Geometry is read from the run's own STLs positioned by the model's WORLD transforms
    (the URDF loader returns degenerate meshes on the single-agent path, so we go straight
    to the STL + pose). All coordinates here are millimetres.

    Exemptions (NOT floating):
    - the base / any part resting on the ground plane (bottom ~ lowest of the assembly);
    - a part a spin shaft physically passes THROUGH (a bearing/bridge whose bore rings an
      arbor) — held by the radial fit, detected as a spin part overlapping its bore volume;
    - spin/free parts (only fixed STRUCTURE is expected to rest on support).
    Support is tested by casting rays straight DOWN from a grid over the part's underside
    and taking the largest gap to whatever solid (any other part) they hit."""
    import os

    import trimesh

    from .mjcf_builder import _world_transforms

    links = list(model.links)
    dof_of = {l.name: getattr(l, "dof", "fixed") for l in links}
    W = _world_transforms(model)                    # meters

    meshes = {}
    for l in links:
        stl = os.path.join(meshes_dir, f"{l.name}.stl")
        T = W.get(l.name)
        if not os.path.exists(stl) or T is None:
            continue
        try:
            mm = trimesh.load(stl, force="mesh")     # local geometry, mm
        except Exception:
            continue
        if not isinstance(mm, trimesh.Trimesh) or len(mm.faces) == 0:
            continue
        Tmm = T.copy()
        Tmm[:3, 3] *= 1000.0                          # m -> mm to match the STL units
        mm.apply_transform(Tmm)
        meshes[l.name] = mm
    if not meshes:
        return []

    # Ground = the lowest underside across the whole assembly (the plane it rests on).
    world_bottom = min(float(m.bounds[0][2]) for m in meshes.values())

    # A fixed part is held by a shaft (not by sitting on support) when a SPIN part's solid
    # actually passes through it — i.e. their solids share volume (the arbor fills the bore).
    spin_meshes = [(n, m) for n, m in meshes.items() if dof_of.get(n) == "spin"]

    def _shaft_passes_through(name, m) -> bool:
        for sn, sm in spin_meshes:
            try:
                if _solid_intersection_frac(m, sm, log_fn=None) > 0.02:
                    return True
            except Exception:
                continue
        return False

    # A part is held by its pose parent ONLY if it geometrically TOUCHES that parent (an
    # upper bearing bolted under a bridge, a hand pressed on a wheel really contact it). A
    # `mount=` label alone does NOT hold anything: a part declared on a bridge but sitting
    # 13mm away from it in space is still floating. So exempt a part only when its solid
    # overlaps or nearly touches its parent's solid; otherwise it must earn support from
    # below or from a shaft through its bore.
    parent_of = {p.child: p.parent for p in model.poses if p.parent}
    present = set(meshes)

    def _touches_parent(name) -> bool:
        par = parent_of.get(name)
        pm = meshes.get(par) if par else None
        if pm is None:
            return False
        m = meshes[name]
        try:
            if _solid_intersection_frac(m, pm, log_fn=None) > 0.01:
                return True
            gap = float(m.nearest.on_surface(pm.vertices)[1].min())
            return gap < 0.5
        except Exception:
            return False

    found: list[Floating] = []
    for name, m in meshes.items():
        if dof_of.get(name) != "fixed":
            continue
        if _touches_parent(name):
            continue                               # really contacts its mount -> held by it
        bottom = float(m.bounds[0][2])
        if bottom - world_bottom <= _GROUND_TOL_MM:
            continue                               # rests on the ground plane -> supported
        if _shaft_passes_through(name, m):
            continue                               # a shaft fills its bore -> held by the fit

        # Cast rays straight down from a grid over the part's XY footprint, starting just
        # below its underside; the nearest hit on ANY other part is the support beneath it.
        lo, hi = m.bounds
        gx = np.linspace(lo[0], hi[0], 4)
        gy = np.linspace(lo[1], hi[1], 4)
        origins, dirs = [], []
        for x in gx:
            for y in gy:
                origins.append([x, y, bottom - 0.1])
                dirs.append([0.0, 0.0, -1.0])
        min_gap = float("inf")
        for onm, om in meshes.items():
            if onm == name:
                continue
            try:
                locs, _, _ = om.ray.intersects_location(
                    ray_origins=np.array(origins), ray_directions=np.array(dirs))
            except Exception:
                continue
            for loc in locs:
                gap = (bottom - 0.1) - float(loc[2])
                if gap >= 0:
                    min_gap = min(min_gap, gap)
        # No hit below at all -> falls all the way to the ground: gap = height above ground.
        if min_gap == float("inf"):
            min_gap = bottom - world_bottom
        if min_gap > _SUPPORT_GAP_MM:
            found.append(Floating(part=name, bottom_mm=bottom - world_bottom,
                                  gap_mm=min_gap))

    found.sort(key=lambda f: f.gap_mm, reverse=True)
    if found:
        log_fn(f"[floating] {len(found)} unsupported fixed part(s): "
               + ", ".join(f"{f.part}({f.gap_mm:.0f}mm gap)" for f in found[:6]))
    return found


def sub_conflicts(model, urdf_path: str, log_fn=print) -> list[Conflict]:
    """Load the sub's own URDF and flag non-adjacent link pairs whose SOLIDS grossly
    interpenetrate. Returns worst-first; [] if the URDF can't be loaded.

    Skips pairs directly joined by a pose edge (parent<->child) as intended nesting. Every
    other pair is scored by REAL mesh-solid overlap (_solid_intersection_frac): a pair is
    a conflict only when its solid intersection is >= _OVERLAP_FRAC of the smaller part.
    Because the measure sees hollow interiors, coaxial shaft-in-bore / key-in-keyway /
    bearing-in-housing nestings read ~0 and need no special-case exemption."""
    try:
        robot = load_robot(urdf_path)
    except Exception as e:
        log_fn(f"[conflict] could not load sub URDF ({type(e).__name__}: {e}); skipping check")
        return []

    # Pose-adjacent link pairs -> intended nesting, never flagged. Read model.poses
    # DIRECTLY (not the lossy model.joints view, which DROPS forest-root poses with an
    # empty parent — so a part placed as a root would never be exempted).
    adj = {frozenset((p.parent, p.child))
           for p in model.poses if p.parent and p.child}

    # COAXIAL SLIDING FIT exemption: a spin tube/arbor running through a fixed bearing's
    # bore (same XY axis line) is an intended nesting even when they are NOT pose-adjacent
    # (both may hang off the base). Their solids overlap only because the bearing STL's bore
    # reads as filled; that is a fit, not a clash. Exempt any spin<->fixed pair whose world
    # centres share an axis line.
    from .mjcf_builder import _world_transforms
    W = _world_transforms(model)
    dof_of = {l.name: getattr(l, "dof", "fixed") for l in model.links}

    def _coaxial(a, b) -> bool:
        Ta, Tb = W.get(a), W.get(b)
        if Ta is None or Tb is None:
            return False
        import numpy as _np
        dxy = _np.array([Ta[0, 3] - Tb[0, 3], Ta[1, 3] - Tb[1, 3]]) * 1000.0
        return float(_np.linalg.norm(dxy)) < 1.5     # centres within 1.5mm of one axis

    names = [l.name for l in model.links]
    meshes = {}
    for n in names:
        m = _link_world_mesh(robot, n)
        if m is not None:
            meshes[n] = m
    present = list(meshes)

    found: list[Conflict] = []
    for i in range(len(present)):
        for k in range(i + 1, len(present)):
            a, b = present[i], present[k]
            if frozenset((a, b)) in adj:
                continue
            dofs = {dof_of.get(a), dof_of.get(b)}
            if dofs == {"spin", "fixed"} and _coaxial(a, b):
                continue                             # tube/arbor through a bearing bore -> fit
            frac = _solid_intersection_frac(meshes[a], meshes[b], log_fn=log_fn)
            if frac >= _OVERLAP_FRAC:
                found.append(Conflict(part_a=a, part_b=b, frac=frac))
            elif frac > 0.05:
                log_fn(f"[conflict] drop small overlap {a}~{b} "
                       f"({frac:.0%} < {_OVERLAP_FRAC:.0%})")

    found.sort(key=lambda c: c.frac, reverse=True)
    return found
