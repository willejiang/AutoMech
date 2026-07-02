"""Geometric pre-check (Stage D): validate the assembled machine BEFORE physics.

The assembler places subassemblies from the boss's global frame contract, but
whether the pieces actually fit depends on where the managers really put their
parts. This module loads the assembled URDF and checks — cheaply, without a sim —
that the seams line up, so an obviously-broken assembly routes back to the boss or
the blamed manager instead of wasting a physics run.

Checks:
  frame_misalign      a WELD seam's two frames don't coincide (position/axis).
  gear_center_distance a gear-MESH seam's two gears aren't one mesh center-distance
                       apart (they won't mesh) — the exact tourbillon failure.
  aabb_overlap        two subs that share NO seam interpenetrate (needs real meshes;
                       skipped when meshes are absent).
  load_error          the URDF won't load at all.

Severity routes the boss loop (Stage F): "interface" -> boss re-plan; "sub" ->
re-run only the blamed manager. See .claude/plans/precious-humming-wand.md.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

import numpy as np

from .viz import load_robot


# A frame counts as coincident within this position / axis tolerance.
_POS_TOL_M = 0.002          # 2 mm
_AXIS_DOT_MIN = 0.99        # ~8 degrees
# A gear-mesh center distance may deviate this fraction from the summed pitch radii.
_MESH_TOL_FRAC = 0.15
_GEAR_RE = re.compile(r"gear|pinion|cog|wheel", re.I)


@dataclass
class Violation:
    kind: str                       # frame_misalign|gear_center_distance|aabb_overlap|load_error
    severity: str                   # "interface" (boss re-plan) | "sub" (re-run a manager)
    detail: str = ""
    sub_id: str = ""
    value: float = 0.0


@dataclass
class PrecheckReport:
    ok: bool
    violations: list = field(default_factory=list)

    def summary(self) -> str:
        if self.ok:
            return "geometry pre-check OK"
        return "; ".join(f"{v.kind}({v.severity})"
                         + (f" {v.sub_id}" if v.sub_id else "") for v in self.violations)


def _ns(sub_id: str, name: str) -> str:
    return f"{sub_id}_{name}"


def _world(robot, link: str) -> np.ndarray:
    """4x4 world transform of a link (base_link -> link) in the assembled scene."""
    return robot.get_transform(frame_to=link, frame_from=robot.base_link)


def _realized_link(sub, frame_name: str) -> str | None:
    for e in (sub.sub_frames or []):
        if e.get("frame") == frame_name:
            return e.get("link")
    return None


def _mesh_pitch_radius(link_spec) -> float | None:
    """Best-effort pitch radius (meters) for a gear link from its size_mm."""
    if link_spec is None:
        return None
    sz = link_spec.size_mm or {}
    for key in ("pitch_radius", "pitch_radius_mm", "radius", "pitch_dia", "pitch_diameter"):
        if key in sz:
            v = float(sz[key])
            if "dia" in key:
                v /= 2.0
            return v / 1000.0
    return None


def precheck(plan, subs: dict, assembled_urdf: str, *, log_fn=print) -> PrecheckReport:
    """Geometrically validate the assembled machine. Returns a PrecheckReport.

    `subs` is {sub_id: SubResult} (for realized-frame lookup + gear sizes); the URDF
    is the assembler's output. Runs before physics; on !ok the boss loop routes by
    each violation's severity.
    """
    def log(m):
        log_fn(f"[precheck] {m}")

    violations: list = []
    try:
        robot = load_robot(assembled_urdf)
    except Exception as e:
        log(f"URDF failed to load: {e}")
        return PrecheckReport(ok=False, violations=[
            Violation(kind="load_error", severity="interface", detail=str(e)[:200])])

    link_names = {l.name for l in robot.robot.links}

    # 1. WELD seams: each seam's frames must be REALIZED (the manager put a real link
    #    at each). Weld frames are REFERENCE points on each sub, not a mating pair —
    #    the assembler positions the child by its GLOBAL frame pose, so the two frames
    #    need NOT coincide (e.g. two housings held a fixed distance apart). We only
    #    fault a frame the manager never placed (its sub must be re-run).
    for seam in plan.seams:
        if seam.kind != "weld":
            continue
        p_sub, c_sub = subs.get(seam.parent_sub), subs.get(seam.child_sub)
        pl = _realized_link(p_sub, seam.parent_frame) if p_sub else None
        cl = _realized_link(c_sub, seam.child_frame) if c_sub else None
        if not pl or not cl:
            miss = seam.parent_sub if not pl else seam.child_sub
            violations.append(Violation(
                kind="frame_misalign", severity="sub", sub_id=miss,
                detail=f"seam '{seam.id}': frame not realized on '{miss}'"))

    # 2. Gear-MESH power seams: the two gear centers must be ~one mesh center-distance
    #    (sum of pitch radii) apart, else the teeth can't engage.
    for seam in plan.seams:
        if seam.kind != "power" or not seam.mesh_pair or len(seam.mesh_pair) != 2:
            continue
        drive_link, driven_link = seam.mesh_pair
        dn = _ns(seam.parent_sub, drive_link)
        vn = _ns(seam.child_sub, driven_link)
        if dn not in link_names or vn not in link_names:
            # mesh_pair may name links by realized frame instead; skip if unresolved.
            continue
        Td, Tv = _world(robot, dn), _world(robot, vn)
        center_d = float(np.linalg.norm(Td[:3, 3] - Tv[:3, 3]))
        p_sub, c_sub = subs.get(seam.parent_sub), subs.get(seam.child_sub)
        rd = _mesh_pitch_radius(p_sub.model.link_by_name(drive_link)) if p_sub and p_sub.model else None
        rv = _mesh_pitch_radius(c_sub.model.link_by_name(driven_link)) if c_sub and c_sub.model else None
        if rd is not None and rv is not None:
            want = rd + rv
            if want > 0 and abs(center_d - want) > _MESH_TOL_FRAC * want:
                violations.append(Violation(
                    kind="gear_center_distance", severity="sub",
                    sub_id=seam.owner_sub or seam.parent_sub, value=center_d,
                    detail=f"mesh '{seam.id}': gear centers {center_d*1000:.1f} mm apart "
                           f"but pitch radii sum to {want*1000:.1f} mm "
                           f"(>{_MESH_TOL_FRAC:.0%} off -> gears won't mesh)"))
        else:
            # No pitch radii to check against: at least flag a zero/degenerate gap.
            if center_d < _POS_TOL_M:
                violations.append(Violation(
                    kind="gear_center_distance", severity="sub",
                    sub_id=seam.owner_sub or seam.parent_sub, value=center_d,
                    detail=f"mesh '{seam.id}': gear centers coincide "
                           f"({center_d*1000:.2f} mm) — they cannot both occupy one spot"))

    # 3. AABB overlap between subs that share NO seam (needs real geometry). Skipped
    #    when meshes are absent (0-byte STLs -> no usable bounds), to avoid false
    #    positives; Stage F only relies on 1-2 for the milestone.
    seamed = set()
    for seam in plan.seams:
        seamed.add(frozenset((seam.parent_sub, seam.child_sub)))
    sub_bounds = _sub_bounds(robot, plan)
    ids = [s.id for s in plan.subassemblies]
    for i in range(len(ids)):
        for k in range(i + 1, len(ids)):
            a, b = ids[i], ids[k]
            if frozenset((a, b)) in seamed:
                continue
            ba, bb = sub_bounds.get(a), sub_bounds.get(b)
            if ba is None or bb is None:
                continue
            if _aabb_overlap(ba, bb):
                violations.append(Violation(
                    kind="aabb_overlap", severity="interface",
                    detail=f"subs '{a}' and '{b}' interpenetrate but share no seam"))

    ok = len(violations) == 0
    if ok:
        log("OK: all seams aligned")
    else:
        for v in violations:
            log(f"VIOLATION {v.kind} [{v.severity}] {v.sub_id}: {v.detail}")
    return PrecheckReport(ok=ok, violations=violations)


def _sub_bounds(robot, plan):
    """AABB (min,max in world) of each subassembly's geometry, or None if a sub has
    no usable mesh bounds (empty STLs)."""
    out: dict = {}
    # Map each scene geometry to its owning link's world AABB.
    for s in plan.subassemblies:
        lo = np.array([np.inf] * 3)
        hi = np.array([-np.inf] * 3)
        found = False
        for l in robot.robot.links:
            if not l.name.startswith(f"{s.id}_"):
                continue
            try:
                T = _world(robot, l.name)
            except Exception:
                continue
            # Pull this link's mesh bounds if present.
            geom = robot.scene.geometry.get(l.name) if hasattr(robot, "scene") else None
            if geom is None or not hasattr(geom, "bounds") or geom.bounds is None:
                continue
            corners = _transform_aabb(geom.bounds, T)
            if corners is None:
                continue
            lo = np.minimum(lo, corners.min(axis=0))
            hi = np.maximum(hi, corners.max(axis=0))
            found = True
        out[s.id] = (lo, hi) if found and np.all(np.isfinite(lo)) else None
    return out


def _transform_aabb(bounds, T):
    """World AABB corners of a local [min,max] box under transform T."""
    try:
        lo, hi = np.asarray(bounds[0]), np.asarray(bounds[1])
    except Exception:
        return None
    corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                        for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    h = np.hstack([corners, np.ones((8, 1))])
    return (h @ T.T)[:, :3]


def _aabb_overlap(ba, bb, margin=1e-4) -> bool:
    (lo_a, hi_a), (lo_b, hi_b) = ba, bb
    return bool(np.all(lo_a <= hi_b - margin) and np.all(lo_b <= hi_a - margin))


# --------------------------------------------------------------------------- #
# CLI (Stage D verification): plan + assembled URDF -> report.
# --------------------------------------------------------------------------- #

def main() -> int:
    import argparse
    import os
    import sys
    from .boss import load_plan
    from .orchestrator_boss import _load_sub_from_disk

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except Exception:
            pass

    ap = argparse.ArgumentParser(description="maker2 geometric pre-check")
    ap.add_argument("--plan", required=True)
    ap.add_argument("--urdf", default=None, help="assembled URDF (default <session>/assembly/model.urdf)")
    a = ap.parse_args()

    session_root = os.path.dirname(os.path.abspath(a.plan))
    plan = load_plan(a.plan)
    subs = {s.id: _load_sub_from_disk(s.id, session_root, log_fn=lambda m: None)
            for s in plan.subassemblies}
    urdf = a.urdf or os.path.join(session_root, "assembly", "model.urdf")
    rep = precheck(plan, subs, urdf, log_fn=print)
    print("-" * 56)
    print(f"RESULT: {'OK' if rep.ok else 'FAIL'} — {rep.summary()}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
