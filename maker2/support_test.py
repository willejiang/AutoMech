"""GRAVITY SUPPORT TEST: which parts are actually held up by real geometry?

This replaces the geometric ray-cast "floating" detector (subcheck.floating_parts).
That detector asked "is there any solid below this part?", which two real failures
proved is the wrong question:

  * a stack of floating parts propping each other up all passed — a watch's minute
    hand rested on the hour hand, which itself rested on nothing;
  * a 1.75 mm air gap under a hand read as contact, because the tolerance that keeps
    legitimate clearance fits from false-flagging is larger than the gaps that matter.

Rather than sharpen the heuristic, ask gravity. Every `mount=` weld is dissolved (each
part becomes an independent free body) except the one already resting on the bench,
which stays welded as the world anchor. Then the assembly simply sits there. Whatever
falls was never supported by anything real — the `mount=` label was the only thing
holding it up.

Friction is pinned near-infinite in the support MJCF so the verdict is about SUPPORT,
not grip: a hand pressed onto its pipe, or a shaft through a bearing bore, is held by
the radial fit and must not creep into a false fault.
"""
from __future__ import annotations

import os

# A part that sinks more than this during the settle is not supported. Generous enough
# to absorb solver micro-settling of a correctly seated part (contact softness lets a
# resting body sink a fraction of a millimetre), tight enough that a genuinely
# unsupported part — which free-falls millimetres to tens of millimetres — is caught.
_FALL_MM = 1.5
_SETTLE_S = 1.0
# Two solids nearer than this are touching (a press/sliding fit); beyond it there is air
# between them and no fit exists, however well the centres line up on the axis.
_FIT_GAP_MM = 0.3


def _really_fits(model, meshes_dir: str, a: str, b: str) -> bool:
    """Do parts `a` and `b` actually meet? A shaft only supports a ring it truly passes
    through: sharing an axis LINE is not enough, since a hand parked past the end of its
    arbor is still perfectly on-axis.

    Measured as a RADIAL fit in the z-band the two share, not as a 3D vertex distance. A
    smooth cylinder carries vertices only on its end rings, so asking "how far is the
    nearest tube vertex from the hand" on a hand that sits mid-tube measures a DIAGONAL to
    the rim, not the gap. On one watch that read 0.354mm -- exactly hypot(0.350 axial to
    the rim, 0.050 real radial gap) -- for a hand correctly fitted with 0.05mm clearance.
    The fit was rejected, the hand fell in the settle test, and the agent spent six
    iterations shoving it along Z (breaking transmission each time) chasing a gap that was
    never axial.
    """
    import os

    import numpy as np
    import trimesh

    from .mjcf_builder import _world_transforms

    W = _world_transforms(model)
    solids = []
    for n in (a, b):
        stl = os.path.join(meshes_dir, f"{n}.stl")
        T = W.get(n)
        if not os.path.exists(stl) or T is None:
            return False
        try:
            mm = trimesh.load(stl, force="mesh")
        except Exception:
            return False
        if not isinstance(mm, trimesh.Trimesh) or len(mm.faces) == 0:
            return False
        Tmm = T.copy()
        Tmm[:3, 3] *= 1000.0
        mm.apply_transform(Tmm)
        solids.append(mm)
    ma, mb = solids
    # Axial overlap first (cheap): no shared span along Z means the shaft ends before the
    # ring begins, so there is nothing in the bore no matter how close the axes are.
    lo = max(float(ma.bounds[0][2]), float(mb.bounds[0][2]))
    hi = min(float(ma.bounds[1][2]), float(mb.bounds[1][2]))
    if hi - lo < _FIT_GAP_MM:
        return False
    # Radial fit, measured only in the shared z-band. One of the two is the bore and the
    # other the shaft; which is which follows from the geometry, so test both directions
    # and accept the one that describes a ring around a post.
    try:
        band = 0.5 * (lo + hi)
        gaps = []
        for inner, outer in ((ma, mb), (mb, ma)):
            iv = inner.vertices[(inner.vertices[:, 2] >= lo) & (inner.vertices[:, 2] <= hi)]
            ov = outer.vertices[(outer.vertices[:, 2] >= lo) & (outer.vertices[:, 2] <= hi)]
            # A smooth cylinder may carry no vertices inside the band; fall back to its
            # full extent, which is the same radius for a part of constant section.
            if len(iv) == 0:
                iv = inner.vertices
            if len(ov) == 0:
                ov = outer.vertices
            if len(iv) == 0 or len(ov) == 0:
                continue
            shaft_r = float(np.linalg.norm(ov[:, :2], axis=1).max())
            bore_r = float(np.linalg.norm(iv[:, :2], axis=1).min())
            gaps.append(bore_r - shaft_r)
        if not gaps:
            return False
        # Negative = interference (still a fit, and a tight one); positive = clearance.
        return min(abs(g) for g in gaps) <= _FIT_GAP_MM
    except Exception:
        return False


def _radial_gap(model, meshes_dir: str, part: str, carrier: str):
    """Bore-minus-shaft radius in mm when `part` rings `carrier` concentrically, else None.

    Only meaningful for two parts that share an axis AND overlap along it: that is the
    case where "it fell" means the bore is too loose, not that the part sits above a gap.
    Returns None otherwise so the caller keeps the axial wording.
    """
    import os

    import numpy as np
    import trimesh

    from .mjcf_builder import _world_transforms

    try:
        W = _world_transforms(model)
        got = []
        for n in (part, carrier):
            stl = os.path.join(meshes_dir, f"{n}.stl")
            T = W.get(n)
            if not os.path.exists(stl) or T is None:
                return None
            mm = trimesh.load(stl, force="mesh")
            if not isinstance(mm, trimesh.Trimesh) or len(mm.faces) == 0:
                return None
            Tmm = T.copy()
            Tmm[:3, 3] *= 1000.0
            mm.apply_transform(Tmm)
            got.append(mm)
        ring, shaft = got
        lo = max(float(ring.bounds[0][2]), float(shaft.bounds[0][2]))
        hi = min(float(ring.bounds[1][2]), float(shaft.bounds[1][2]))
        if hi - lo < _FIT_GAP_MM:
            return None                                  # no axial overlap -> not a ring
        rv = ring.vertices[(ring.vertices[:, 2] >= lo) & (ring.vertices[:, 2] <= hi)]
        sv = shaft.vertices[(shaft.vertices[:, 2] >= lo) & (shaft.vertices[:, 2] <= hi)]
        if len(rv) == 0:
            rv = ring.vertices
        if len(sv) == 0:
            sv = shaft.vertices
        if len(rv) == 0 or len(sv) == 0:
            return None
        gap = (float(np.linalg.norm(rv[:, :2], axis=1).min())
               - float(np.linalg.norm(sv[:, :2], axis=1).max()))
        return gap if gap > 0 else None                  # interference is a different fault
    except Exception:
        return None


def _declared_carriers(link) -> list:
    """Every part `link` says carries it: the primary mount plus any extra mounts."""
    out = []
    for c in ([getattr(link, "mount", "")]
              + list(getattr(link, "extra_mounts", None) or [])):
        c = (str(c) or "").strip()
        if c and c not in out:
            out.append(c)
    return out


def _touches(model, meshes_dir: str, a: str, b: str, tol_mm: float) -> bool:
    """Do the two solids actually MEET, measured on the STLs in world pose?

    Surface distance, not an axis test: this catches a part sitting flat on a bridge or
    a plate, which `_really_fits` (a ring-on-shaft radial test) cannot see.
    """
    import os

    import trimesh

    from .mjcf_builder import _world_transforms

    try:
        W = _world_transforms(model)
        got = []
        for n in (a, b):
            stl = os.path.join(meshes_dir, f"{n}.stl")
            T = W.get(n)
            if not os.path.exists(stl) or T is None:
                return False
            mm = trimesh.load(stl, force="mesh")
            if not isinstance(mm, trimesh.Trimesh) or len(mm.faces) == 0:
                return False
            Tmm = T.copy()
            Tmm[:3, 3] *= 1000.0
            mm.apply_transform(Tmm)
            got.append(mm)
        ma, mb = got
        # BOTH directions, and keep the smaller. Sampling only one solid's vertices
        # measures a distance that depends on where that solid happens to carry
        # vertices: a long plain arbor has them only at its two end rings, so a pinion
        # pressed onto its middle reads 2.2mm from the arbor's vertices while the
        # pinion's own vertices sit 0.005mm from the arbor's surface — the real
        # interference. One direction called that correct press fit unsupported.
        best = float("inf")
        for src, tgt in ((ma, mb), (mb, ma)):
            pts = src.vertices
            if len(pts) == 0:
                continue
            if len(pts) > 400:
                pts = pts[:: max(1, len(pts) // 400)]
            best = min(best, float(trimesh.proximity.closest_point(tgt, pts)[1].min()))
        return best <= tol_mm
    except Exception:
        return False


class Fell:
    """One part that dropped when its mount weld was dissolved: nothing real held it."""

    def __init__(self, part: str, drop_mm: float, parent: str = "", radial_mm=None):
        self.part = part
        self.drop_mm = drop_mm
        self.parent = parent
        # Radial clearance to the declared carrier when the two ARE concentric, else None.
        self.radial_mm = radial_mm

    def describe(self) -> str:
        base = (f"part '{self.part}' FELL {self.drop_mm:.1f}mm when the assembly was "
                f"settled under gravity: nothing physically supports it.")
        if self.parent and self.radial_mm is not None:
            # Concentric but too loose. Telling this part to move DOWN is useless advice:
            # it is a ring around a post, the gap is RADIAL, and sliding along the axis
            # never closes it. One watch burned six iterations shoving the hand along Z,
            # breaking the gear train each time, because the message only offered that.
            return (base + f" It is concentric with '{self.parent}' and overlaps it along "
                    f"the axis, but its bore is {self.radial_mm:.3f}mm LARGER in radius, "
                    f"so it hangs in air around it. In this simulation a bore within "
                    f"0.10mm of the shaft is a PRESS fit (it is carried, and turns 1:1 "
                    f"with it); anything looser is a running fit and is carried by "
                    f"nothing. FIX by tightening the bore of '{self.part}' to within "
                    f"0.10mm of '{self.parent}'. Do NOT move it along the axis — the gap "
                    f"is radial, not vertical — and do NOT add a new support part.")
        where = (f" It is declared on '{self.parent}', but that is only a label — "
                 f"the two never touch.") if self.parent else ""
        return (base + where + f" FIX by "
                f"making it actually meet the part that carries it — move '{self.part}' "
                f"down until its solid touches that part, or lengthen the shaft/arbor/"
                f"tube so it reaches '{self.part}'. Do NOT add a new support part.")


def support_faults(model, ctx, *, settings=None, log_fn=print) -> list[Fell]:
    """Build the support MJCF, settle it under gravity, and return the parts that fell.

    Returns [] when MuJoCo is unavailable or the model cannot be built — an absent
    check must never block a run (the physics stage still judges the machine)."""
    try:
        import mujoco as mj
        import numpy as np
    except Exception as e:
        log_fn(f"[support] mujoco unavailable ({e}); skipping the support test")
        return []

    from .mjcf_builder import build_support_mjcf, coaxial_pairs

    support_metrics: dict = {}
    try:
        path, ground = build_support_mjcf(
            model, ctx, settings=settings, metrics=support_metrics, log_fn=log_fn)
        m = mj.MjModel.from_xml_path(path)
    except Exception as e:
        log_fn(f"[support] could not build/load the support model ({e}); skipping")
        return []

    d = mj.MjData(m)
    # mj_forward FIRST: MjData starts with xpos all-zero, so a baseline captured before
    # kinematics are computed measures every part against the origin instead of its design
    # pose — every drop then reads as a huge negative "rise" and nothing is ever flagged.
    mj.mj_forward(m, d)
    z0 = d.xpos[:, 2].copy()
    for _ in range(int(_SETTLE_S / m.opt.timestep)):
        mj.mj_step(m, d)
    drop = (z0 - d.xpos[:, 2]) * 1000.0                 # meters -> mm, positive = sank

    parent_of = {p.child: p.parent for p in model.poses if p.parent}
    fell: dict = {}
    for b in range(1, m.nbody):
        name = m.body(b).name
        if not name or name == ground:
            continue
        if float(drop[b]) > _FALL_MM:
            fell[name] = float(drop[b])

    # RADIAL FIT = SUPPORT. A part clamped around a shaft/pipe is held by the fit, and in
    # the real machine has nothing under it by design (a watch hand is pressed onto its
    # pipe). Its contact is excluded in the support MJCF — otherwise the bore reads as
    # interpenetration and the solver ejects it — so it falls here as an artifact of the
    # exclusion, not as a real fault. Credit the fit as support instead. Support flows
    # both ways: the ring rides the shaft, and the shaft is journalled in the ring.
    #
    # But the fit must be REAL. coaxial_pairs() only tests distance to the infinite axis
    # LINE, so a hand hovering 10mm past the end of its arbor is still "on the axis" — that
    # is the exact false pass this whole rewrite exists to kill. Require the two solids to
    # actually meet (the shaft passes through the bore), measured on the STLs in world pose.
    links_by_name = {l.name: l for l in model.links}
    coaxial: dict = {}
    for s, f in coaxial_pairs(model, ctx.meshes_dir, include_spin_spin=True):
        if not _really_fits(model, ctx.meshes_dir, s, f):
            continue
        coaxial.setdefault(s, set()).add(f)
        coaxial.setdefault(f, set()).add(s)
    for name in list(fell):
        held = [o for o in coaxial.get(name, ()) if o not in fell]
        if held:
            del fell[name]

    # A valid pin closure is support topology even when its soft equality constraint allows
    # a couple millimetres of solver settling. Credit a fallen body only when it is connected
    # by at least two validated pin/revolute relations to bodies that did not fall: a single
    # pin still allows a pendulum to drop and must not mask a genuinely unheld part.
    closure_neighbors: dict = {}
    for entry in support_metrics.get("support_relation_acceptances", []):
        parts = list(entry.get("parts") or [])
        if len(parts) != 2:
            continue
        a, b = parts
        closure_neighbors.setdefault(a, set()).add(b)
        closure_neighbors.setdefault(b, set()).add(a)
    for name in list(fell):
        held = [o for o in closure_neighbors.get(name, ()) if o not in fell]
        if len(held) >= 2:
            del fell[name]

    # SAME CREDIT FOR A PART THAT SIMPLY TOUCHES ITS CARRIER. The exclusions above are
    # not the only ones in the support MJCF, and a part whose declared carrier is right
    # there against it did not fall for want of support — the contact that held it was
    # removed, or the solver dropped it. Measured on run 1_12_20260803_195154 iter_1:
    # intermediate_upper_bearing sits flat on skeleton_bearing_bridge (surface distance
    # 0.0000mm, 2.00mm of axial overlap) and was still reported unsupported, as was
    # intermediate_pinion_11t, pressed onto its arbor with a correct 0.005mm
    # interference. Neither is coaxial-ring-on-shaft, so the credit above never saw them.
    #
    # The test still means what it says: a part is credited only when its solid actually
    # MEETS a carrier that itself stayed up. A part declared onto something it does not
    # touch — the fault this whole module exists to catch — has no such contact and
    # still falls.
    _TOUCH_MM = 0.05
    for name in list(fell):
        l = links_by_name.get(name)
        if l is None:
            continue
        for car in _declared_carriers(l):
            if car in fell or car not in links_by_name:
                continue
            if _touches(model, ctx.meshes_dir, name, car, _TOUCH_MM):
                del fell[name]
                break

    # ROOT CAUSE ONLY. When a bearing floats 0.5mm off the baseplate, everything stacked
    # above it falls too — reporting all 14 parts buries the 3 real faults and invites the
    # agent to "fix" parts that were never wrong. A part whose own support also fell is a
    # CONSEQUENCE; keep only the parts whose declared parent stayed up (or that declared
    # none), i.e. the top of each broken chain.
    found: list[Fell] = []
    for name, dz in fell.items():
        par = parent_of.get(name, "")
        if par and par in fell:
            continue                                    # its support fell -> downstream
        found.append(Fell(part=name, drop_mm=dz, parent=par,
                          radial_mm=_radial_gap(model, ctx.meshes_dir, name, par)
                          if par else None))
    found.sort(key=lambda f: f.drop_mm, reverse=True)
    if found:
        log_fn(f"[support] {len(found)} unsupported part(s) (of {len(fell)} that fell; "
               f"the rest were carried down by these): "
               + ", ".join(f"{f.part}({f.drop_mm:.0f}mm)" for f in found[:6]))
    else:
        log_fn(f"[support] every part held up under gravity (anchor '{ground}')")
    return found
