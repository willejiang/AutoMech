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

import hashlib
import json
import os
import re
from dataclasses import asdict, dataclass, field

import numpy as np

from .viz import load_robot


# A frame counts as coincident within this position / axis tolerance.
_POS_TOL_M = 0.002          # 2 mm
_AXIS_DOT_MIN = 0.99        # ~8 degrees
# A gear-mesh center distance may deviate this fraction from the summed pitch radii.
_MESH_TOL_FRAC = 0.15
_GEAR_RE = re.compile(r"gear|pinion|cog|wheel", re.I)


def _stable_id(prefix: str, value: dict) -> str:
    raw = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return prefix + hashlib.sha256(raw).hexdigest()[:16]


@dataclass
class Violation:
    kind: str                       # frame_misalign|gear_center_distance|aabb_overlap|load_error
    severity: str                   # "interface" (boss re-plan) | "sub" (re-run a manager)
    detail: str = ""
    sub_id: str = ""
    value: float = 0.0
    violation_id: str = ""
    seam_id: str = ""
    involved_sub_ids: list = field(default_factory=list)
    parent_link: str = ""
    child_link: str = ""
    parent_local_link: str = ""
    child_local_link: str = ""
    shaft_role: str = ""
    overlap_fraction: float = 0.0
    threshold: float = 0.0
    observations: dict = field(default_factory=dict)

    def __post_init__(self):
        if not self.violation_id:
            core = {"kind": self.kind, "severity": self.severity, "seam_id": self.seam_id,
                    "sub_id": self.sub_id, "involved_sub_ids": sorted(self.involved_sub_ids),
                    "links": sorted(x for x in (self.parent_link, self.child_link) if x)}
            self.violation_id = _stable_id("violation_", core)

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class PrecheckReport:
    ok: bool
    violations: list = field(default_factory=list)
    failure_id: str = ""

    def __post_init__(self):
        if not self.failure_id:
            core = {"ok": self.ok, "violations": [v.to_dict() for v in self.violations]}
            self.failure_id = _stable_id("precheck_", core)

    def summary(self) -> str:
        if self.ok:
            return "geometry pre-check OK"
        return "; ".join(f"{v.kind}({v.severity})"
                         + (f" {v.sub_id}" if v.sub_id else "") for v in self.violations)

    def to_dict(self) -> dict:
        return {"ok": self.ok, "failure_id": self.failure_id,
                "aggregate_overlap": sum(float(v.overlap_fraction) for v in self.violations),
                "violations": [v.to_dict() for v in self.violations]}


def _ns(sub_id: str, name: str) -> str:
    return f"{sub_id}_{name}"


def _local_name(sub_id: str, namespaced: str) -> str:
    prefix = f"{sub_id}_"
    return namespaced[len(prefix):] if namespaced.startswith(prefix) else namespaced


def _shaft_role(*names: str) -> str:
    text = " ".join(names).upper()
    if "INPUT" in text:
        return "input"
    if "INTER" in text or "MIDDLE" in text:
        return "inter"
    if "OUTPUT" in text:
        return "output"
    return ""


def _world(robot, link: str) -> np.ndarray:
    """4x4 world transform of a link (base_link -> link) in the assembled scene."""
    return robot.get_transform(frame_to=link, frame_from=robot.base_link)


def _realized_frame_in_root(sub,frame_name):
    entry=next((e for e in (sub.sub_frames or []) if e.get('frame')==frame_name),None)
    if entry is None or sub.model is None:return None
    from .assembler import _mat,_root_to_link
    T=_root_to_link(sub.model).get(entry.get('link'))
    return None if T is None else T@_mat(entry.get('local_xyz_m',(0,0,0)),entry.get('local_rpy_rad',(0,0,0)))


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

    # 方案B-v3: when the managers authored coordinates parametrically, `plan.params_text` IS
    # the machine's single source of truth — every interface frame has a same-named zero-arg
    # function returning its GLOBAL mm coordinate, and every manager placed its parts by
    # calling those functions. So precheck asks params DIRECTLY for a frame's world point
    # instead of reconstructing it from the collapsed sub_frames (which, in manager_py mode,
    # degrade every frame to a part's local origin -> phantom weld gaps). This is deterministic
    # code reading a config file, not an agent — no reason to launder the truth through model.
    _params_ns: dict = {}
    if (getattr(plan, "params_text", "") or "").strip() and "def " in plan.params_text:
        try:
            exec(compile(plan.params_text, "<params>", "exec"), _params_ns)
        except Exception as e:
            log(f"params module did not exec ({type(e).__name__}: {e}); "
                "falling back to realized-frame reconstruction")
            _params_ns = {}

    def _frame_world_m(frame_name: str):
        """GLOBAL position (meters) of an interface frame straight from the params module, or
        None when params can't supply it (no module / no such function / bad return)."""
        fn = _params_ns.get(frame_name)
        if not callable(fn):
            return None
        try:
            xyz_mm = fn()
            v = np.array([float(xyz_mm[0]), float(xyz_mm[1]), float(xyz_mm[2])]) / 1000.0
            return v
        except Exception:
            return None


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

    # 1b. Optional through-shaft rear datum: the solved rear shaft point must lie on the
    # rear housing bore plane. The front pair remains the only placement weld.
    for seam in plan.seams:
        rp=getattr(seam,'rear_parent_frame','');rc=getattr(seam,'rear_child_frame','')
        if seam.kind!='weld' or not (rp and rc):continue
        ps,cs=subs.get(seam.parent_sub),subs.get(seam.child_sub)
        if not ps or not cs:continue
        Pw_m=_frame_world_m(rp);Cw_m=_frame_world_m(rc)
        try:
            if Pw_m is not None and Cw_m is not None:
                Pw3,Cw3=Pw_m,Cw_m
            else:
                Pl=_realized_frame_in_root(ps,rp);Cl=_realized_frame_in_root(cs,rc)
                if Pl is None or Cl is None:continue
                Pw3=(_world(robot,_ns(seam.parent_sub,ps.model.root_link))@Pl)[:3,3]
                Cw3=(_world(robot,_ns(seam.child_sub,cs.model.root_link))@Cl)[:3,3]
            pfr=next(f for f in plan.sub_by_id(seam.parent_sub).frames if f.name==rp)
            axis=np.asarray(pfr.axis,float);axis=axis/np.linalg.norm(axis)
            signed=float(np.dot(Cw3-Pw3,axis));err=abs(signed)
        except Exception:continue
        if err>_POS_TOL_M:
            violations.append(Violation(kind='rear_mount_plane',severity='interface',sub_id=seam.parent_sub,
              seam_id=seam.id,involved_sub_ids=[seam.parent_sub,seam.child_sub],value=err,
              threshold=_POS_TOL_M,observations={'rear_parent_frame':rp,'rear_child_frame':rc,
              'signed_residual_mm':signed*1000.0},detail=f"seam '{seam.id}' rear shaft datum misses "
              f"rear housing plane by {err*1000:.1f} mm"))

    # 1c. GENERAL WELD-COINCIDENCE (machine-agnostic): a weld whose mate is meant to make
    #     its two frames MEET — an `insert` (shaft end into a bore), a `seat` (face on face),
    #     or any weld naming paired ports — must, on the assembled geometry, have its parent
    #     and child frames actually coincide. The assembler places each sub by its declared
    #     global pose; when two seams pull one sub in incompatible directions (an
    #     over-constrained contract), a weld silently ends up open. This is the general form
    #     of the reducer's authoritative-solve failure, but it needs no gear cluster and no
    #     recognized topology — it runs for ANY machine from the realized frames alone.
    for seam in plan.seams:
        if seam.kind != "weld":
            continue
        mate = getattr(seam, "mate_type", "") or ""
        has_ports = bool(getattr(seam, "parent_port", "") and getattr(seam, "child_port", ""))
        if mate not in ("insert", "seat") and not has_ports:
            continue  # a bare reference weld need not coincide (subs held a fixed gap apart)
        ps, cs = subs.get(seam.parent_sub), subs.get(seam.child_sub)
        if not ps or not cs:
            continue
        # 方案B-v3: prefer the params truth. Both weld frames are named exactly like their
        # params functions, so their world points come straight from the source of truth —
        # no root@local reconstruction, no collapsed sub_frames.
        Pw_m = _frame_world_m(seam.parent_frame)
        Cw_m = _frame_world_m(seam.child_frame)
        if Pw_m is not None and Cw_m is not None:
            gap = float(np.linalg.norm(Cw_m - Pw_m))
        else:
            Pl = _realized_frame_in_root(ps, seam.parent_frame)
            Cl = _realized_frame_in_root(cs, seam.child_frame)
            if Pl is None or Cl is None:
                continue  # realization already faulted in step 1
            try:
                Pw = _world(robot, _ns(seam.parent_sub, ps.model.root_link)) @ Pl
                Cw = _world(robot, _ns(seam.child_sub, cs.model.root_link)) @ Cl
                gap = float(np.linalg.norm(Cw[:3, 3] - Pw[:3, 3]))
            except Exception:
                continue
        if gap > _POS_TOL_M:
            violations.append(Violation(
                kind="weld_frame_coincidence", severity="interface",
                sub_id=seam.parent_sub, seam_id=seam.id,
                involved_sub_ids=[seam.parent_sub, seam.child_sub], value=gap,
                threshold=_POS_TOL_M,
                observations={"parent_frame": seam.parent_frame,
                              "child_frame": seam.child_frame, "mate_type": mate,
                              "gap_mm": gap * 1000.0},
                detail=f"weld '{seam.id}' ({mate or 'ported'}): frames "
                       f"'{seam.parent_frame}' and '{seam.child_frame}' are {gap*1000:.1f} mm "
                       "apart in the assembly but this mate requires them to coincide — the "
                       "plan's placement is over-constrained/contradictory; fix the seam "
                       "frames or the conflicting mate."))

    # 2. Gear-MESH power seams: the two gear centers must be ~one mesh center-distance
    #    (sum of pitch radii) apart, else the teeth can't engage.
    for seam in plan.seams:
        if seam.kind != "power" or not seam.mesh_pair or len(seam.mesh_pair) != 2:
            continue
        drive_link, driven_link = seam.mesh_pair
        dn = _ns(seam.parent_sub, drive_link)
        vn = _ns(seam.child_sub, driven_link)
        # 方案B-v3: a mesh seam names its two gear CENTERS as frames (pinion1_center /
        # gear2_center) == params function names, so read the true world centers from params.
        # This avoids the URDF-chain _world() degenerating to 0 for flat global poses that
        # aren't in one kinematic tree.
        Pc = _frame_world_m(seam.parent_frame)
        Cc = _frame_world_m(seam.child_frame)
        if Pc is not None and Cc is not None:
            center_d = float(np.linalg.norm(Pc - Cc))
        else:
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
_MAX_SEAM_HITS = 4   # max collision pairs reported per sub / per weld seam (worst-first), so all
                    # real clashes surface in one iteration without a bad sub flooding the report


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


def _solid_intersection_frac(ma, mb, log_fn=None) -> float:
    """REAL solid-overlap score of two WORLD meshes in [0,1]: the volume of their actual
    mesh boolean INTERSECTION as a fraction of the smaller part's solid volume.

    This is the correct measure for rigid-conflict detection: unlike the AABB proxy
    (_intersection_frac), it sees that a HOLLOW part's bore/keyway/cut is empty. A shaft
    threaded through a bearing bore, a key seated in a keyway, a bearing pressed into a
    housing bore, a plug passing through a hole — all have ~0 REAL overlap even though
    their bounding boxes overlap heavily. Only two parts whose SOLID metal actually
    interpenetrates score high.

    Manifold booleans require volume meshes. CadQuery can export a valid annulus as two
    consistently-wound shells that trimesh does not label watertight; use its watertight
    convex hull for the boolean operand in that case. This closes tessellation seams while
    preserving holes in the other operand, unlike the old AABB fallback which made every
    slender part inside a hollow housing look 100% embedded. AABB remains the last resort
    when no usable solid can be formed."""
    # Cheap AABB pre-filter: disjoint boxes -> definitely no solid overlap. Skips the
    # (relatively) expensive boolean for the common far-apart case.
    loa, hia = np.asarray(ma.bounds[0]), np.asarray(ma.bounds[1])
    lob, hib = np.asarray(mb.bounds[0]), np.asarray(mb.bounds[1])
    if np.any(np.maximum(loa, lob) >= np.minimum(hia, hib)):
        return 0.0
    operands=[]
    repaired=[]
    for mesh in (ma,mb):
        if getattr(mesh,"is_watertight",False) and float(getattr(mesh,"volume",0.0))>0:
            operands.append(mesh)
            continue
        try:
            solid=mesh.convex_hull
            if not solid.is_watertight or float(solid.volume)<=0:
                raise ValueError("convex hull is not a usable volume")
            operands.append(solid);repaired.append(True)
        except Exception:
            if log_fn:log_fn("[conflict] AABB fallback: mesh has no usable solid volume")
            return _intersection_frac(ma,mb)
    try:
        import trimesh
        # A near-empty boolean result can have zero volume; trimesh's center-of-mass
        # divide then warns harmlessly. Silence it — we guard the volume below anyway.
        with np.errstate(divide="ignore", invalid="ignore"):
            inter=trimesh.boolean.intersection(operands,engine="manifold")
            if inter is None or len(getattr(inter,"vertices",()))==0:
                return 0.0
            vi=float(inter.volume)
    except Exception as e:
        if log_fn:log_fn(f"[conflict] AABB fallback: solid boolean failed ({type(e).__name__})")
        return _intersection_frac(ma,mb)
    vs=min(float(x.volume) for x in operands)
    if vs<=0:
        if log_fn:log_fn("[conflict] AABB fallback: repaired solid has zero volume")
        return _intersection_frac(ma,mb)
    if repaired and log_fn:log_fn("[conflict] repaired non-watertight mesh with convex hull")
    return max(0.0,vi/vs)


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
        hits = []
        for i in range(len(names)):
            for k in range(i + 1, len(names)):
                a, b = names[i], names[k]
                if frozenset((a, b)) in adj:
                    continue
                frac = _solid_intersection_frac(meshes[a], meshes[b], log_fn=log)
                if frac >= _OVERLAP_FRAC:
                    hits.append((frac, a, b))
                elif frac > 0.05:
                    log(f"drop small overlap {a}~{b} ({frac:.0%} < {_OVERLAP_FRAC:.0%})")
        # Report EVERY interpenetrating pair (worst-first, capped), not just the single worst — so
        # all collisions in a sub surface in ONE iteration instead of one-per-round.
        hits.sort(key=lambda h: h[0], reverse=True)
        for frac, a, b in hits[:_MAX_SEAM_HITS]:
            out.append(Violation(
                kind="part_overlap", severity="sub", sub_id=sub.id, value=frac,
                involved_sub_ids=[sub.id], parent_link=a, child_link=b,
                parent_local_link=_local_name(sub.id, a), child_local_link=_local_name(sub.id, b),
                overlap_fraction=frac, threshold=_OVERLAP_FRAC,
                observations={"measure": "real_solid_intersection_fraction"},
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
        # An INSERT fit (a bearing pressed into a housing seat, a shaft end into a bore) is a
        # coaxial nesting where ONLY the two MATING parts (the seat and the bearing the seam
        # declares) touch by design — the bearing OD equals the seat bore, so a manifold boolean
        # sees a thin shared shell. That tolerance must apply to the seam's OWN mating pair ONLY,
        # NOT to every other part pair across the two subs: a big gear that eats into the top cover
        # is a real collision even though its sub happens to be insert-welded to the housing. So we
        # resolve the seam's declared mating links and give THEM the loose insert floor, while every
        # other cross-sub pair keeps the strict collision bar (and a lower report floor so an 8%
        # gear-into-cover bite is not silently dropped).
        mate = getattr(seam, "mate_type", "") or ""
        p_sub_obj, c_sub_obj = subs.get(seam.parent_sub), subs.get(seam.child_sub)
        mate_pn = _realized_link(p_sub_obj, seam.parent_frame) if p_sub_obj else None
        mate_cn = _realized_link(c_sub_obj, seam.child_frame) if c_sub_obj else None
        mate_pair = frozenset((_ns(seam.parent_sub, mate_pn) if mate_pn else "",
                               _ns(seam.child_sub, mate_cn) if mate_cn else ""))
        # a non-mating cross-sub collision (gear vs cover) should report well below 0.30
        _NONMATE_FLOOR = 0.05
        hits = []
        for an, amesh in ma.items():
            for bn, bmesh in mb.items():
                is_mate = frozenset((an, bn)) == mate_pair
                # the seam's own insert pair tolerates a press-fit shell; everything else must not
                # interpenetrate at all
                report_floor = (0.60 if (mate == "insert" and is_mate) else _NONMATE_FLOOR)
                frac = _solid_intersection_frac(amesh, bmesh, log_fn=log)
                if frac >= report_floor:
                    hits.append((frac, an, bn, is_mate))
                elif 0.0 < frac < report_floor:
                    tag = "insert-fit mating" if is_mate else "cross-sub"
                    log(f"drop {tag} overlap {an}~{bn} ({frac:.0%} < {report_floor:.0%})")
        # Report EVERY colliding pair over its floor, worst-first (capped) — NOT just the single
        # worst. A seam often has several independent clashes at once (a rear bearing at 100% AND a
        # gear eating the housing wall at 8%); reporting only the worst hid the gear clash until the
        # bearing was fixed, so each iteration surfaced one more and never converged. Cap per seam so
        # a badly-placed sub does not flood the report.
        hits.sort(key=lambda h: h[0], reverse=True)
        for frac, an, bn, is_mate in hits[:_MAX_SEAM_HITS]:
            role = _shaft_role(seam.id, seam.parent_frame, seam.child_frame, an, bn)
            out.append(Violation(
                kind="part_overlap", severity="interface", sub_id=seam.parent_sub, value=frac,
                seam_id=seam.id, involved_sub_ids=[seam.parent_sub, seam.child_sub],
                parent_link=an, child_link=bn,
                parent_local_link=_local_name(seam.parent_sub, an),
                child_local_link=_local_name(seam.child_sub, bn), shaft_role=role,
                overlap_fraction=frac, threshold=_OVERLAP_FRAC,
                observations={"measure": "real_solid_intersection_fraction",
                              "parent_frame": seam.parent_frame,
                              "child_frame": seam.child_frame,
                              "mate_type": getattr(seam, "mate_type", ""),
                              "is_mating_pair": bool(is_mate)},
                detail=(
                    f"welded subs '{seam.parent_sub}' and '{seam.child_sub}' "
                    f"interpenetrate: parts '{an}' and '{bn}' overlap {frac:.0%} of the "
                    f"smaller part's solid — the seam drives them into each other; "
                    f"re-plan the frame offsets"
                    if is_mate else
                    f"parts '{an}' and '{bn}' (subs '{seam.parent_sub}'/'{seam.child_sub}') "
                    f"collide {frac:.0%} of the smaller part's solid — these are NOT the seam's "
                    f"mating faces, so this is a real clash. If one part is a functional part "
                    f"whose size is fixed by the spec (a gear sized by the gear ratio), do NOT "
                    f"shrink it; enlarge the CONTAINING part (housing wall/cover/cavity) to clear "
                    f"it while keeping the interface frames fixed.")))
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
    ap.add_argument("--out", default=None, help="write the structured report JSON here")
    a = ap.parse_args()

    session_root = os.path.dirname(os.path.abspath(a.plan))
    plan = load_plan(a.plan)
    subs = {s.id: _load_sub_from_disk(s.id, session_root, log_fn=lambda m: None)
            for s in plan.subassemblies}
    urdf = a.urdf or os.path.join(session_root, "assembly", "model.urdf")
    rep = precheck(plan, subs, urdf, log_fn=print)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump(rep.to_dict(), f, indent=2)
    print("-" * 56)
    print(f"RESULT: {'OK' if rep.ok else 'FAIL'} — {rep.summary()}")
    return 0 if rep.ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
