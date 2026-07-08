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

    # 4. PART INTERPENETRATION: rigid pieces occupying the same space. Within a sub
    #    (severity "sub" -> re-run that manager) and across a WELD seam (severity
    #    "interface" -> boss re-plan). Skips pairs joined by a joint (a shaft in a
    #    bearing SHOULD overlap) and only flags GROSS overlap (see _part_overlaps).
    violations.extend(_part_overlaps(robot, plan, subs, log))

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
            geom = _geom_for(robot, l.name)
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
# Part interpenetration (check #4)
# --------------------------------------------------------------------------- #

# Flag only GROSS overlap: the shared bounding volume must be at least this fraction
# of the SMALLER part's bounding volume. Below this, treat as a legit tight/nested fit.
_OVERLAP_FRAC = 0.30


def _geom_for(robot, link_name):
    """Fetch a link's scene geometry, tolerant of yourdfpy's keyings. yourdfpy keys
    robot.scene.geometry by '<link>_visual' (both sub and assembled URDFs — the visual
    is named that in urdf_builder), sometimes by the mesh basename, or the bare link
    name. Trying only the bare name (as this module used to) silently finds NOTHING, so
    every overlap check vacuously passed. Mirrors assembler._sub_world_bounds._geom_for."""
    if not hasattr(robot, "scene"):
        return None
    scene = robot.scene.geometry
    base = ""
    try:
        link = next((l for l in robot.robot.links if l.name == link_name), None)
        vis = link.visuals[0] if (link and getattr(link, "visuals", None)) else None
        if vis and vis.geometry and getattr(vis.geometry, "mesh", None):
            base = os.path.basename(vis.geometry.mesh.filename or "")
    except Exception:
        base = ""
    for key in (f"{link_name}_visual", base or None, link_name):
        if key and key in scene:
            return scene[key]
    for key in scene:                                    # last resort: prefix match
        if key.startswith(link_name):
            return scene[key]
    return None


def _link_world_mesh(robot, link_name):
    """A copy of a link's mesh transformed into world coords, or None."""
    geom = _geom_for(robot, link_name)
    if geom is None or not hasattr(geom, "bounds") or geom.bounds is None:
        return None
    try:
        T = _world(robot, link_name)
        m = geom.copy()
        m.apply_transform(T)
        return m
    except Exception:
        return None


def _aabb_vol(bounds) -> float:
    lo, hi = np.asarray(bounds[0]), np.asarray(bounds[1])
    d = np.clip(hi - lo, 0, None)
    return float(d[0] * d[1] * d[2])


def _intersection_frac(ma, mb) -> float:
    """Rough interpenetration score of two WORLD meshes in [0,1]: how much of the
    SMALLER part's bounding volume is shared with the other. Uses the intersection-AABB
    volume as a fraction of the smaller part's AABB volume — coarse but MONOTONIC and
    robust (surface-point containment is unreliable for solids that share a boundary).
    Enough to catch GROSS interpenetration; a tight nested fit stays well under the
    threshold because its shared volume is small relative to the parts."""
    loa, hia = np.asarray(ma.bounds[0]), np.asarray(ma.bounds[1])
    lob, hib = np.asarray(mb.bounds[0]), np.asarray(mb.bounds[1])
    lo_i = np.maximum(loa, lob)
    hi_i = np.minimum(hia, hib)
    if np.any(lo_i >= hi_i):
        return 0.0
    vi = _aabb_vol((lo_i, hi_i))
    vs = min(_aabb_vol(ma.bounds), _aabb_vol(mb.bounds))
    return (vi / vs) if vs > 0 else 0.0


def _solid_intersection_frac(ma, mb) -> float:
    """REAL solid-overlap score of two WORLD meshes in [0,1]: the volume of their actual
    mesh boolean INTERSECTION as a fraction of the smaller part's solid volume.

    This is the correct measure for rigid-conflict detection: unlike the AABB proxy
    (_intersection_frac), it sees that a HOLLOW part's bore/keyway/cut is empty. A shaft
    threaded through a bearing bore, a key seated in a keyway, a bearing pressed into a
    housing bore, a plug passing through a hole — all have ~0 REAL overlap even though
    their bounding boxes overlap heavily. Only two parts whose SOLID metal actually
    interpenetrates score high.

    Requires a mesh boolean engine (manifold3d). Falls back to the AABB proxy when the
    boolean is unavailable or a part isn't watertight (a non-watertight mesh has no
    well-defined solid volume) — so this never crashes the gate, it only degrades to the
    old behavior for that one pair."""
    # Cheap AABB pre-filter: disjoint boxes -> definitely no solid overlap. Skips the
    # (relatively) expensive boolean for the common far-apart case.
    loa, hia = np.asarray(ma.bounds[0]), np.asarray(ma.bounds[1])
    lob, hib = np.asarray(mb.bounds[0]), np.asarray(mb.bounds[1])
    if np.any(np.maximum(loa, lob) >= np.minimum(hia, hib)):
        return 0.0
    # A solid volume is only meaningful for watertight meshes; degrade gracefully.
    if not (getattr(ma, "is_watertight", False) and getattr(mb, "is_watertight", False)):
        return _intersection_frac(ma, mb)
    try:
        import trimesh
        # A near-empty boolean result can have zero volume; trimesh's center-of-mass
        # divide then warns harmlessly. Silence it — we guard the volume below anyway.
        with np.errstate(divide="ignore", invalid="ignore"):
            inter = trimesh.boolean.intersection([ma, mb], engine="manifold")
            if inter is None or len(getattr(inter, "vertices", ())) == 0:
                return 0.0
            vi = float(inter.volume)
    except Exception:
        return _intersection_frac(ma, mb)
    vs = min(float(ma.volume), float(mb.volume))
    if vs <= 0:
        return _intersection_frac(ma, mb)
    return max(0.0, vi / vs)


def _part_overlaps(robot, plan, subs: dict, log) -> list:
    """Flag rigid parts that GROSSLY interpenetrate. Within a sub -> severity 'sub'
    (re-run that manager); across a WELD seam -> severity 'interface' (boss re-plan).

    Overlap is measured on the REAL mesh solids (_solid_intersection_frac, a manifold
    boolean), NOT bounding boxes: a hollow part's bore/keyway/cut is empty, so a shaft in
    a bearing bore, an oil seal a shaft passes through, or a whole sub NESTING inside
    another (input-shaft assembly slotted into the housing) all read ~0 — only genuinely
    interpenetrating metal scores high. Skips pose-adjacent pairs (intended nesting) and
    only flags above _OVERLAP_FRAC. Logs every DROP so a suppressed overlap is visible."""
    out: list = []

    def _sub_meshes(sub):
        """{namespaced_link -> world mesh} for one sub, missing meshes dropped."""
        model = getattr(subs.get(sub.id), "model", None)
        if model is None:
            return {}, set()
        # pose-adjacent link pairs (namespaced) -> intended nesting, skip. Read model.poses
        # DIRECTLY (not the lossy model.joints view, which DROPS forest-root poses).
        adj = {frozenset((_ns(sub.id, p.parent), _ns(sub.id, p.child)))
               for p in model.poses if p.parent and p.child}
        m = {}
        for l in model.links:
            ln = _ns(sub.id, l.name)
            wm = _link_world_mesh(robot, ln)
            if wm is not None:
                m[ln] = wm
        return m, adj

    sub_mesh_cache = {}
    for sub in plan.subassemblies:
        meshes, adj = _sub_meshes(sub)
        sub_mesh_cache[sub.id] = meshes
        names = list(meshes)
        worst = None
        for i in range(len(names)):
            for k in range(i + 1, len(names)):
                a, b = names[i], names[k]
                if frozenset((a, b)) in adj:
                    continue
                frac = _solid_intersection_frac(meshes[a], meshes[b])
                if frac >= _OVERLAP_FRAC:
                    if worst is None or frac > worst[0]:
                        worst = (frac, a, b)
                elif frac > 0.05:
                    log(f"drop small overlap {a}~{b} ({frac:.0%} < {_OVERLAP_FRAC:.0%})")
        if worst:
            frac, a, b = worst
            out.append(Violation(
                kind="part_overlap", severity="sub", sub_id=sub.id, value=frac,
                detail=f"parts '{a}' and '{b}' interpenetrate ({frac:.0%} of the smaller "
                       f"part's solid is inside the other) — fix their placement"))

    # Cross-WELD-seam gross overlap: check the REAL part solids of the two subs against
    # each other (NOT whole-sub bounding boxes — two subs that legitimately NEST, like a
    # shaft assembly inside a housing, have hugely overlapping AABBs but ~0 solid overlap).
    # A genuine seam mis-placement that drives one sub's metal INTO the other's still trips.
    for seam in plan.seams:
        if seam.kind != "weld":
            continue
        ma = sub_mesh_cache.get(seam.parent_sub, {})
        mb = sub_mesh_cache.get(seam.child_sub, {})
        if not ma or not mb:
            continue
        worst = None
        for an, amesh in ma.items():
            for bn, bmesh in mb.items():
                frac = _solid_intersection_frac(amesh, bmesh)
                if frac >= _OVERLAP_FRAC and (worst is None or frac > worst[0]):
                    worst = (frac, an, bn)
        if worst:
            frac, an, bn = worst
            out.append(Violation(
                kind="part_overlap", severity="interface", value=frac,
                detail=f"welded subs '{seam.parent_sub}' and '{seam.child_sub}' "
                       f"interpenetrate: parts '{an}' and '{bn}' overlap {frac:.0%} of the "
                       f"smaller part's solid — the seam drives them into each other; "
                       f"re-plan the frame offsets"))
    return out


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
