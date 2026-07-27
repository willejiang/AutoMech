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
    arbor is still perfectly on-axis."""
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
    try:
        return float(ma.nearest.on_surface(mb.vertices)[1].min()) <= _FIT_GAP_MM
    except Exception:
        return False


class Fell:
    """One part that dropped when its mount weld was dissolved: nothing real held it."""

    def __init__(self, part: str, drop_mm: float, parent: str = ""):
        self.part = part
        self.drop_mm = drop_mm
        self.parent = parent

    def describe(self) -> str:
        where = (f" It is declared on '{self.parent}', but that is only a label — "
                 f"the two never touch.") if self.parent else ""
        return (f"part '{self.part}' FELL {self.drop_mm:.1f}mm when the assembly was "
                f"settled under gravity: nothing physically supports it.{where} FIX by "
                f"making it actually meet the part that carries it — move '{self.part}' "
                f"down until its solid touches that part, or lengthen the shaft/arbor/"
                f"tube so it reaches '{self.part}'. Do NOT add a new support part.")


def support_faults(model, ctx, *, log_fn=print) -> list[Fell]:
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

    try:
        path, ground = build_support_mjcf(model, ctx, log_fn=log_fn)
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
        found.append(Fell(part=name, drop_mm=dz, parent=par))
    found.sort(key=lambda f: f.drop_mm, reverse=True)
    if found:
        log_fn(f"[support] {len(found)} unsupported part(s) (of {len(fell)} that fell; "
               f"the rest were carried down by these): "
               + ", ".join(f"{f.part}({f.drop_mm:.0f}mm)" for f in found[:6]))
    else:
        log_fn(f"[support] every part held up under gravity (anchor '{ground}')")
    return found
