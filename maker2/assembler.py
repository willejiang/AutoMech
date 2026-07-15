"""Deterministic assembler (Stage C): stitch per-subassembly KinematicModels into
ONE final KinematicModel using the boss's SubassemblyPlan.

The boss (Stage A) is the only LLM that reasons about seams; this module is PURE
PYTHON so surgical re-runs re-assemble identically, the biggest artifact never
hits the token cap, and the single-tree invariant is enforced by reusing
manager._validate_model. It:

  1. Namespaces every sub's links/poses -> "<sub_id>_<name>" (remapping endpoints,
     root_link, mesh_filename).
  2. Consolidates each sub's meshes into one meshes/ dir (copy, not symlink -> Windows).
  3. Concatenates all links + poses; the global root is the root of plan.root_sub.
  4. Adds BRIDGE poses from the plan's seams:
       weld  -> a fixed placement pose from the parent sub's realized frame link to the
                CHILD sub's ROOT, with an origin computed so the two frames coincide.
       power (gear MESH, the milestone) -> NO cross-seam pose; the housing weld holds
                the gear centers one mesh-distance apart and the gears couple by tooth
                contact at sim time (the pair is recorded in the model's mesh_pairs).
  5. Re-validates via manager._validate_model (name normalization + weak forest check /
     mesh_filename) and writes model.urdf (build_urdf, render/appearance) AND model.mjcf
     (build_mjcf, the simulation compiler) from the same merged KinematicModel.

See .claude/plans/precious-humming-wand.md.
"""

from __future__ import annotations

import json
import os
import shutil

import numpy as np
import trimesh.transformations as tf

from .manager import ManagerError, _validate_model, save_model
from .model import KinematicModel, LinkSpec, PoseSpec
from .urdf_builder import build_urdf, validate_urdf


def _write_assembled_mjcf(final, ctx, settings, log) -> None:
    """Write the assembled machine's MJCF (model.mjcf) next to model.urdf, from the
    SAME merged KinematicModel. build_mjcf is the sole simulation compiler (CoACD,
    mm->m, mass, solver tuning) and physics rebuilds it authoritatively at run time;
    this just persists a matching on-disk artifact so the assembled machine has an
    MJCF, not only a (render-only) URDF. Best-effort — a failure here never breaks the
    assembly, since physics does not depend on this file existing."""
    try:
        from .mjcf_builder import build_mjcf
        path = build_mjcf(final, ctx, settings=settings, log_fn=log)
        log(f"wrote assembled MJCF {path}")
    except Exception as e:
        log(f"WARNING: could not write assembled model.mjcf: {e}")


class AssemblerError(RuntimeError):
    """The subassemblies could not be stitched into one valid machine."""


# --------------------------------------------------------------------------- #
# Transforms
# --------------------------------------------------------------------------- #

def _mat(xyz, rpy) -> np.ndarray:
    """4x4 from xyz (m) + rpy (rad, fixed-axis XYZ / sxyz) — same convention as
    urdf_builder._origin_matrix."""
    m = tf.euler_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]), axes="sxyz")
    m[:3, 3] = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
    return m


def _decompose(m: np.ndarray):
    """4x4 -> (xyz tuple, rpy tuple) in the sxyz convention build_urdf expects."""
    rx, ry, rz = tf.euler_from_matrix(m, axes="sxyz")
    x, y, z = m[:3, 3]
    return (float(x), float(y), float(z)), (float(rx), float(ry), float(rz))


def _root_to_link(model: KinematicModel) -> dict:
    """Accumulate the transform from the sub's ROOT link to every link, by walking
    the POSE forest (root at identity). Uses each pose's own xyz_m/rpy_rad. Links
    unreachable from root_link (separate forest components) are seeded at identity so
    they still get a transform."""
    children: dict[str, list] = {}
    for p in model.poses:
        if p.parent:
            children.setdefault(p.parent, []).append(p)
    T: dict[str, np.ndarray] = {}
    # Seed every forest root (a link that is never a parented pose's child).
    parented = {p.child for p in model.poses if p.parent}
    roots = [l.name for l in model.links if l.name not in parented]
    if model.root_link and model.root_link not in roots:
        roots.append(model.root_link)
    stack = []
    for r in roots:
        T[r] = np.eye(4)
        stack.append(r)
    while stack:
        node = stack.pop()
        for p in children.get(node, []):
            T[p.child] = T[node] @ _mat(p.xyz_m, p.rpy_rad)
            stack.append(p.child)
    return T


# --------------------------------------------------------------------------- #
# Realized-frame lookup
# --------------------------------------------------------------------------- #

def _frame_realized(sub, frame_name: str):
    """Find a sub's realized frame entry (from sub_frames.json / model.frames_realized).
    Returns (link_name, local_matrix) or raises AssemblerError."""
    entries = sub.sub_frames or []
    for e in entries:
        if e.get("frame") == frame_name:
            return e["link"], _mat(e.get("local_xyz_m", (0, 0, 0)),
                                   e.get("local_rpy_rad", (0, 0, 0)))
    raise AssemblerError(
        f"subassembly '{sub.id}' did not realize interface frame '{frame_name}' "
        f"(have: {[e.get('frame') for e in entries]})")


def _frame_in_root(sub, frame_name: str) -> tuple[str, np.ndarray]:
    """Pose of a sub's interface frame in that sub's OWN root frame, plus the link
    it lives on. = (root->link) @ (link-local frame offset)."""
    link, local = _frame_realized(sub, frame_name)
    r2l = _root_to_link(sub.model)
    if link not in r2l:
        raise AssemblerError(f"sub '{sub.id}' frame '{frame_name}' is on link "
                             f"'{link}' which is not reachable from its root")
    return link, (r2l[link] @ local)


# --------------------------------------------------------------------------- #
# Namespacing
# --------------------------------------------------------------------------- #

def _ns(sub_id: str, name: str) -> str:
    return f"{sub_id}_{name}"


def _namespaced_links(sub, meshes_dir: str, ns_id: str | None = None) -> list:
    """Clone a sub's links with namespaced names + consolidated mesh paths.

    ``ns_id`` is the namespace for the link names AND their STL files (defaults to
    sub.id; an instance passes ``f"{sub.id}_{k}"``). Each instance gets its OWN copy of
    the STL under its namespaced name — manager._validate_model reassigns
    mesh_filename to "meshes/<link_name>.stl", so the file must exist under that exact
    name. (The identical geometry is re-copied per instance; the real dedup win is
    building the sub ONCE, not the STL bytes.)"""
    ns_id = ns_id or sub.id
    src_meshes = os.path.join(sub.ctx.run_dir, "meshes") if sub.ctx else ""
    out = []
    for l in sub.model.links:
        new_name = _ns(ns_id, l.name)
        rel = f"meshes/{new_name}.stl"
        if src_meshes:
            src = os.path.join(src_meshes, f"{l.name}.stl")
            dst = os.path.join(meshes_dir, f"{new_name}.stl")
            try:
                if os.path.exists(src) and not os.path.exists(dst):
                    shutil.copy2(src, dst)
            except Exception:
                pass
        out.append(LinkSpec(
            name=new_name, description=l.description, shape_hint=l.shape_hint,
            size_mm=dict(l.size_mm), origin_note=l.origin_note, color=l.color,
            mesh_filename=rel, dof=l.dof, spin_axis=tuple(l.spin_axis),
            driver=l.driver))
    return out


def _namespaced_poses(sub, ns_id: str | None = None) -> list:
    """Namespace a sub's internal poses. A sub's own root-placement pose (parent "",
    child = sub root) is DROPPED — the assembler owns where each sub root goes (via a
    weld seam for non-root subs, or the origin for the root sub), so keeping the sub's
    internal root pose would double-place its root."""
    ns_id = ns_id or sub.id
    root = sub.model.root_link
    out = []
    for p in sub.model.poses:
        if not p.parent and p.child == root:
            continue                      # assembler places the sub root itself
        out.append(PoseSpec(
            name=_ns(ns_id, p.name),
            parent=_ns(ns_id, p.parent) if p.parent else "",
            child=_ns(ns_id, p.child),
            xyz_m=tuple(p.xyz_m), rpy_rad=tuple(p.rpy_rad)))
    return out


def _sub_instance_ids(spec, base_id: str) -> list[str]:
    """The namespace id(s) for a spec: [base_id] for a normal sub, or
    [base_id_0, base_id_1, ...] for a sub with N instances (identical repeated copies)."""
    insts = getattr(spec, "instances", None) or []
    if len(insts) <= 1:
        return [base_id]
    return [f"{base_id}_{k}" for k in range(len(insts))]


# --------------------------------------------------------------------------- #
# Bridge joints from seams
# --------------------------------------------------------------------------- #

def _bridge_pose_instance(seam, subs: dict, placed_root: dict, *,
                          child_ns: str, child_root_global) -> tuple:
    """A fixed WELD pose that places ONE INSTANCE copy of the child sub at its declared
    per-copy GLOBAL root pose (``child_root_global`` from instances[k]). N identical copies
    live at N distinct absolute poses, which a single port mate cannot express — so instances
    are the one remaining place an absolute coordinate is authored. The pose is expressed
    relative to the PARENT's realized frame link (the pose parent) so the forest is consistent
    with the port-based welds. Returns (PoseSpec, child_root_global)."""
    parent = subs[seam.parent_sub]
    child = subs[seam.child_sub]
    pL, _T_pRoot_pf = _frame_in_root(parent, seam.parent_frame)     # parent frame's link
    parent_root_global = placed_root[seam.parent_sub]
    r2l_parent = _root_to_link(parent.model)
    T_global_pL = parent_root_global @ r2l_parent[pL]
    T_origin = tf.inverse_matrix(T_global_pL) @ child_root_global
    xyz, rpy = _decompose(T_origin)
    p = PoseSpec(
        name=f"seam_{seam.id}_{child_ns}",
        parent=_ns(parent.id, pL), child=_ns(child_ns, child.model.root_link),
        xyz_m=xyz, rpy_rad=rpy)
    return p, child_root_global


def _bridge_pose_from_ports(seam, subs: dict, placed_root: dict, *,
                            child_ns: str | None = None) -> tuple:
    """NUMBER-FREE weld: place the child sub by welding its realized ``child_port`` onto the
    parent's realized ``parent_port`` IN WORLD — no boss coordinate. This is the boss-level
    analogue of ``mate_solver._resolve_coaxial``: the parent is already placed, so its port's
    world pose is known; the child hangs so its own port lands exactly there. Because the
    child mates to where the parent's frame is ACTUALLY realized (not where the boss said in
    absolute coords), a boss/realization mismatch cannot fling the child away — the seam is
    rigid by construction.

    Ports default to the seam's frames (``parent_frame``/``child_frame``) when the port
    fields are unset, so a seam only needs ``mate_type`` set to opt in. ``offset_mm`` slides
    the child along the shared port axis (e.g. an insert seat depth). Returns (PoseSpec,
    child_root_global) with the same shape as ``_bridge_pose_instance``."""
    parent = subs[seam.parent_sub]
    child = subs[seam.child_sub]
    child_ns = child_ns or child.id
    p_port = getattr(seam, "parent_port", "") or seam.parent_frame
    c_port = getattr(seam, "child_port", "") or seam.child_frame

    pL, T_pRoot_pf = _frame_in_root(parent, p_port)          # parent port in parent-root
    cL, T_cRoot_cf = _frame_in_root(child, c_port)           # child port in child-root
    # Parent port in WORLD (parent already placed at placed_root[parent_sub]).
    parent_root_global = placed_root[seam.parent_sub]
    T_world_pf = parent_root_global @ T_pRoot_pf
    # Optional axial seat: slide the child along the shared (parent-port +Z) axis.
    off = np.eye(4)
    off[2, 3] = float(getattr(seam, "offset_mm", 0.0)) / 1000.0
    # Child root so its port lands on the parent port (+ optional seat offset).
    child_root_global = T_world_pf @ off @ tf.inverse_matrix(T_cRoot_cf)

    # Express the weld origin relative to the PARENT's realized port LINK (the pose parent),
    # matching the instance placer's parenting so the pose forest is identical in shape.
    r2l_parent = _root_to_link(parent.model)
    T_global_pL = parent_root_global @ r2l_parent[pL]
    T_origin = tf.inverse_matrix(T_global_pL) @ child_root_global
    xyz, rpy = _decompose(T_origin)
    p = PoseSpec(
        name=f"seam_{seam.id}" if child_ns == child.id else f"seam_{seam.id}_{child_ns}",
        parent=_ns(parent.id, pL), child=_ns(child_ns, child.model.root_link),
        xyz_m=xyz, rpy_rad=rpy)
    return p, child_root_global


# --------------------------------------------------------------------------- #
# Cross-subassembly gear-mesh placement (solve-then-build)
# --------------------------------------------------------------------------- #
# The boss authors NO base coordinates. For a machine whose subs are coupled by gear
# meshes, the SOLVER places the meshing sub cluster at true center-distance (read from the
# BUILT gears' module x teeth), and the passive base is then placed as a follower of that
# solved cluster. See .claude/plans/breezy-giggling-deer.md.

def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def _gear_pitch_r_mm(link) -> float | None:
    """Pitch radius (mm) of a built gear LinkSpec from size_mm: pitch_radius, or
    module*teeth/2, or a pitch/outer diameter halved. None if not resolvable. Mirrors
    mate_solver._gear_pitch_radius_mm but on the assembler's LinkSpec."""
    sz = getattr(link, "size_mm", {}) or {}

    def num(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return None
    if num(sz.get("pitch_radius")):
        return num(sz.get("pitch_radius"))
    if num(sz.get("pitch_radius_mm")):
        return num(sz.get("pitch_radius_mm"))
    mod, teeth = num(sz.get("module")), sz.get("teeth")
    try:
        if mod and teeth and int(teeth) > 0:
            return mod * int(teeth) / 2.0
    except (TypeError, ValueError):
        pass
    for k in ("pitch_dia", "pitch_diameter"):
        if num(sz.get(k)):
            return num(sz.get(k)) / 2.0
    return None


def _link_in_root(sub, link_name: str):
    """(axis_world_in_root, center_in_root) of a link's spin axis + origin, expressed in
    the sub's OWN root frame. axis from the link's spin_axis, origin from root->link."""
    r2l = _root_to_link(sub.model)
    T = r2l.get(link_name)
    if T is None:
        raise AssemblerError(f"sub '{sub.id}' link '{link_name}' not reachable from root")
    lk = next((l for l in sub.model.links if l.name == link_name), None)
    axis_local = _unit(getattr(lk, "spin_axis", (0, 0, 1)) or (0, 0, 1)) if lk else np.array([0, 0, 1.0])
    axis_root = _unit(T[:3, :3] @ axis_local)
    center_root = T[:3, 3]
    return axis_root, center_root


def _gear_link(sub, seam, which: int):
    """The gear LinkSpec that a power seam's mesh FRAME is realized on (role-based identity),
    with the `mesh_pair` part NAME as a legacy fallback. `which` is 0 for the parent gear, 1 for
    the child gear.

    Role-based first: the boss names the mesh by FRAME (parent_frame/child_frame, e.g.
    gear1_center), and the manager realizes that frame ON its actual gear link (whatever it named
    it — large_gear, stage1_pinion, ...). Reading the gear FROM the realized frame means the boss
    never has to guess the manager's gear part name (the same class of bug fixed for seat bores).
    Falls back to `mesh_pair[which]` by name only when the frame isn't realized on a link."""
    frame_name = seam.parent_frame if which == 0 else seam.child_frame
    if frame_name:
        try:
            link_name, _ = _frame_realized(sub, frame_name)
            lk = next((l for l in sub.model.links if l.name == link_name), None)
            # A manager may realize a mesh frame on the shaft/bearing that carries the
            # gear. Accept role-based identity only when the built link is actually a gear;
            # otherwise continue to the explicit mesh_pair fallback.
            if lk is not None and _gear_pitch_r_mm(lk):
                return lk
        except AssemblerError:
            pass                              # frame not realized -> fall back to mesh_pair name
    mp = getattr(seam, "mesh_pair", ()) or ()
    if len(mp) == 2:
        return next((l for l in sub.model.links if l.name == mp[which]), None)
    return None


def _classify_subs(plan, subs: dict):
    """Return (gear_sub_ids: set, base_sub_id or None). A gear stage is an endpoint of a
    power seam with a 2-tuple mesh_pair; a passive base has no mesh endpoint and parents at
    least one weld 'insert' seam. Returns (set(), None) when there is no gear cluster."""
    gear_ids: set = set()
    for seam in plan.seams:
        if seam.kind == "power" and len(getattr(seam, "mesh_pair", ()) or ()) == 2:
            gear_ids.add(seam.parent_sub)
            gear_ids.add(seam.child_sub)
    if not gear_ids:
        return set(), None
    base_id = None
    for s in plan.subassemblies:
        if s.id in gear_ids:
            continue
        parents_insert = any(
            seam.kind == "weld" and seam.parent_sub == s.id
            and getattr(seam, "mate_type", "") == "insert"
            for seam in plan.seams)
        if parents_insert:
            base_id = s.id
            break
    return gear_ids, base_id


def _base_bore_dir(plan, subs, base_id, parent_gear_sub, child_gear_sub):
    """World-ish direction from the parent gear stage's base bore to the child stage's base
    bore, read from the base's insert-weld mount frames (`xyz_m`). This is the layout the
    base was built for; using it as the mesh separation keeps the solved gear cluster ALIGNED
    with the bores that hold it (instead of an arbitrary perpendicular). Returns a 3-vector or
    None when the base / its bores can't be resolved."""
    if base_id is None or base_id not in subs:
        return None
    base = subs[base_id]

    def bore_xyz(gear_sub):
        seam = next((s for s in plan.seams
                     if s.kind == "weld" and getattr(s, "mate_type", "") == "insert"
                     and s.parent_sub == base_id and s.child_sub == gear_sub), None)
        if seam is None:
            return None
        spec = next((s for s in plan.subassemblies if s.id == base_id), None)
        if spec is None:
            return None
        f = next((f for f in (spec.frames or []) if f.name == seam.parent_frame), None)
        return np.asarray(f.xyz_m, float) if f is not None else None

    a = bore_xyz(parent_gear_sub)
    b = bore_xyz(child_gear_sub)
    if a is None or b is None:
        return None
    d = b - a
    # Guard: if the two bore declarations are collapsed / near-coincident (a common boss/manager
    # lapse), this direction is meaningless — return None so the seam separation_axis / boss-axis
    # fallback governs the mesh separation instead of a degenerate hint.
    return d if float(np.linalg.norm(d)) > 1e-3 else None


def _rot_a_to_b(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """4x4 rotation taking unit vector a onto unit vector b (identity if already aligned;
    180° about any perpendicular if antiparallel)."""
    a = _unit(np.asarray(a, float)); b = _unit(np.asarray(b, float))
    c = float(np.dot(a, b))
    if c > 1 - 1e-9:
        return np.eye(4)
    if c < -1 + 1e-9:
        perp = np.cross(a, [1.0, 0, 0])
        if float(np.linalg.norm(perp)) < 1e-6:
            perp = np.cross(a, [0, 1.0, 0])
        return tf.rotation_matrix(np.pi, _unit(perp))
    v = np.cross(a, b)
    return tf.rotation_matrix(float(np.arccos(max(-1.0, min(1.0, c)))), _unit(v))


def _cluster_orient_to_seat_axis(plan, subs: dict, driver_gear: str, base_id, log) -> np.ndarray:
    """World pose to pin the driver gear stage so its spin axis aligns with the BOSS-declared
    seat axis of the bore it inserts into. Returns identity if that can't be resolved (falls
    back to the gear's build-local orientation)."""
    if base_id is None:
        return np.eye(4)
    seam = next((s for s in plan.seams
                 if s.kind == "weld" and getattr(s, "mate_type", "") == "insert"
                 and s.parent_sub == base_id and s.child_sub == driver_gear), None)
    if seam is None:
        return np.eye(4)
    spec = next((s for s in plan.subassemblies if s.id == base_id), None)
    fr = next((f for f in (spec.frames or []) if f.name == seam.parent_frame), None) if spec else None
    if fr is None:
        return np.eye(4)
    seat_axis_w = _unit(np.asarray(fr.axis, float))
    if float(np.linalg.norm(seat_axis_w)) < 1e-9:
        return np.eye(4)
    # Driver gear's own spin axis in its child-root frame (its gear link's axis).
    dgear = subs.get(driver_gear)
    gm = _gear_link(dgear, seam, 1) if dgear is not None else None
    if gm is None:
        # fall back to the mesh gear resolution via the power seam owner
        pseam = next((s for s in plan.seams if s.kind == "power"
                      and (s.parent_sub == driver_gear or s.child_sub == driver_gear)), None)
        if pseam is not None and dgear is not None:
            gm = _gear_link(dgear, pseam, 0 if pseam.parent_sub == driver_gear else 1)
    if gm is None:
        return np.eye(4)
    axis_local, _c = _link_in_root(dgear, gm.name)
    R = _rot_a_to_b(axis_local, seat_axis_w)
    log(f"[mesh] cluster oriented: driver '{driver_gear}' spin axis -> boss seat axis "
        f"{tuple(round(x,3) for x in seat_axis_w)} (from '{base_id}.{seam.parent_frame}')")
    return R


def _place_mesh_cluster(plan, subs: dict, gear_ids: set, base_id, log) -> dict:
    """Solve world root poses for the gear-stage cluster so every meshing pair sits at true
    center-distance C=(r_a+r_b) (radii from the BUILT gears), axes parallel, separation
    perpendicular to the gear axis. Returns {sub_id: 4x4 world root pose} for gear subs.

    The driver (or first) gear stage is pinned at the origin; each downstream stage is
    placed off its already-placed mesh parent via a BFS over power seams."""
    power = [s for s in plan.seams
             if s.kind == "power" and len(getattr(s, "mesh_pair", ()) or ()) == 2]
    if not power:
        return {}
    by_parent: dict = {}
    for seam in power:
        by_parent.setdefault(seam.parent_sub, []).append(seam)

    driver_seam = next((s for s in power if getattr(s, "driver", False)), None)
    root_gear = (driver_seam.parent_sub if driver_seam else power[0].parent_sub)
    # Orient the WHOLE cluster so the driver gear's spin axis points along the BOSS-DECLARED
    # seat axis (the one authoritative direction: the axis a shaft runs through its bore).
    # Otherwise the driver is pinned at identity and the cluster inherits the gear's build-local
    # +Z spin axis, which need not match how the housing bores were declared -> shafts weld at a
    # right angle to the housing and float off. We read seat.axis from the driver's insert weld.
    T_root = _cluster_orient_to_seat_axis(plan, subs, root_gear, base_id, log)
    placed: dict = {root_gear: T_root}
    # DIAGNOSTIC: driver's spin axis in world after orientation (should equal the seat axis).
    try:
        _dg = subs.get(root_gear)
        _pseam0 = next((s for s in plan.seams if s.kind == "power"
                        and (s.parent_sub == root_gear or s.child_sub == root_gear)), None)
        _gm0 = _gear_link(_dg, _pseam0, 0 if _pseam0 and _pseam0.parent_sub == root_gear else 1) if (_dg and _pseam0) else None
        if _gm0 is not None:
            _axl, _ = _link_in_root(_dg, _gm0.name)
            _axw = T_root[:3, :3] @ _axl
            log(f"[mesh] DIAG driver '{root_gear}' gear '{_gm0.name}' local-axis {np.round(_axl,3)} "
                f"-> world {np.round(_axw,3)} (want seat axis)")
    except Exception as _e:
        log(f"[mesh] DIAG driver axis check failed ({type(_e).__name__})")
    queue = [root_gear]
    seen = {root_gear}
    while queue:
        cur = queue.pop(0)
        for seam in by_parent.get(cur, []):
            if seam.child_sub in seen:
                continue
            parent = subs[seam.parent_sub]
            child = subs[seam.child_sub]
            gp = _gear_link(parent, seam, 0)
            gc = _gear_link(child, seam, 1)
            if gp is None or gc is None:
                raise AssemblerError(
                    f"power seam '{seam.id}': could not resolve the meshing gears — the mesh "
                    f"frames '{seam.parent_frame}'/'{seam.child_frame}' were not realized on a "
                    f"gear link and mesh_pair {seam.mesh_pair} names no built gear. Realize each "
                    f"mesh frame on its gear.")
            r_p = _gear_pitch_r_mm(gp)
            r_c = _gear_pitch_r_mm(gc)
            if not r_p or not r_c:
                raise AssemblerError(
                    f"power seam '{seam.id}': gear '{gp.name}'/'{gc.name}' missing a pitch "
                    f"radius (need module+teeth or pitch_dia)")
            C = (r_p + r_c) / 1000.0

            # Parent gear axis + center in WORLD (parent already placed).
            axis_p_root, center_p_root = _link_in_root(parent, gp.name)
            Tp = placed[seam.parent_sub]
            axis_w = _unit(Tp[:3, :3] @ axis_p_root)
            center_p_w = (Tp @ np.append(center_p_root, 1.0))[:3]

            # Separation direction: prefer the BASE's bore-to-bore layout (the direction the
            # housing was built for) so the solved gear cluster ALIGNS with its bores; then an
            # explicit seam separation_axis; else a deterministic perpendicular of the axis.
            sep = _base_bore_dir(plan, subs, base_id, seam.parent_sub, seam.child_sub)
            if sep is None:
                sa = getattr(seam, "separation_axis", None) or getattr(seam, "axis", None)
                sep = (np.asarray(sa, float)
                       if (isinstance(sa, (list, tuple)) and len(sa) == 3)
                       else (np.array([1.0, 0, 0]) if abs(axis_w[0]) < 0.9
                             else np.array([0, 1.0, 0])))
            sep = _unit(sep - np.dot(sep, axis_w) * axis_w)
            if float(np.linalg.norm(sep)) < 1e-9:
                sep = _unit(np.cross(axis_w, [0, 0, 1.0]) if abs(axis_w[2]) < 0.9
                            else np.cross(axis_w, [1.0, 0, 0]))
            center_c_w = center_p_w + C * sep

            # Child root pose: ROTATE the child so its gear spin axis aligns with the parent
            # gear's WORLD axis (the whole cluster is rigid — same orientation as the driver,
            # which we may have re-oriented to the boss seat axis). Then translate so the child
            # gear center lands at center_c_w. Without the rotation, downstream stages keep their
            # build-local +Z axis while the driver points along +X → the gears sit at 90° and
            # never mesh.
            axis_c_root, center_c_root = _link_in_root(child, gc.name)
            R = _rot_a_to_b(axis_c_root, axis_w)
            Tc = R.copy()
            center_c_root_rot = (R[:3, :3] @ center_c_root)
            Tc[:3, 3] = center_c_w - center_c_root_rot
            placed[seam.child_sub] = Tc
            log(f"[mesh] placed '{seam.child_sub}' at C={C*1000:.1f}mm from "
                f"'{seam.parent_sub}' (r={r_p:.1f}+{r_c:.1f}) so '{gp.name}'~'{gc.name}' mesh")
            log(f"[mesh] DIAG '{seam.child_sub}' world center={np.round(center_c_w,4)} "
                f"sep={np.round(sep,3)} (parent center={np.round(center_p_w,4)})")
            seen.add(seam.child_sub)
            queue.append(seam.child_sub)
    return {k: v for k, v in placed.items() if k in gear_ids}


def _derive_base_pose(plan, subs: dict, base_id: str, placed_root: dict, log):
    """Place the passive base as a FOLLOWER of the solved gear cluster: land the base's first
    insert-bore frame onto the already-placed child stage's realized shaft frame. Sets
    placed_root[base_id]. Best-effort — if the bore/shaft frames aren't realized, the base
    keeps its BFS placement (which for the root base is the origin)."""
    inserts = [s for s in plan.seams
               if s.kind == "weld" and s.parent_sub == base_id
               and getattr(s, "mate_type", "") == "insert"
               and s.child_sub in placed_root]
    if not inserts:
        return
    seam = inserts[0]
    base = subs[base_id]
    child = subs[seam.child_sub]
    try:
        _bL, T_bRoot_bf = _frame_in_root(base, seam.parent_frame)     # bore in base-root
        _cL, T_cRoot_cf = _frame_in_root(child, seam.child_frame)     # shaft in child-root
    except AssemblerError:
        return
    T_world_shaft = placed_root[seam.child_sub] @ T_cRoot_cf
    # Base root so its bore frame lands on the child's shaft frame in world.
    placed_root[base_id] = T_world_shaft @ tf.inverse_matrix(T_bRoot_bf)
    log(f"[mesh] derived base '{base_id}' pose from solved shaft of '{seam.child_sub}' "
        f"(bore '{seam.parent_frame}' <- shaft '{seam.child_frame}')")


def _override_base_bores(plan, subs: dict, base_id: str, placed_root: dict, log) -> None:
    """DETERMINISTIC bore placement: the assembler OWNS where each housing bore sits, derived from
    the shaft it mates — NOT from where the manager built/realized its bearing.

    The weld path (`_bridge_pose_from_ports`) reads only the REALIZED seat pose from
    ``base.sub_frames``, never the boss's declared axis. The housing manager routinely realizes all
    seats collapsed on the body root at origin (built its bearings on the wrong axis), so shafts
    weld at the origin and float outside the housing. Here, for each insert seam, we REWRITE the
    base's realized seat frame so it lands at the mated shaft's already-solved WORLD pose. The
    weld-BFS then places each shaft at its own bore. This mirrors `_place_mesh_cluster`: derive from
    real solved geometry, ignore the manager's coordinates.

    Must run AFTER the shaft cluster is in ``placed_root`` and the base root is pinned (either at
    origin when the base IS root_sub, or by `_derive_base_pose`), and BEFORE the weld-BFS so the
    corrected frames are consumed. Mutates ``base.sub_frames`` in place (read live by
    `_frame_realized`); no re-persist needed."""
    base = subs.get(base_id)
    if base is None or base.model is None:
        return
    T_base_root = placed_root.get(base_id)
    if T_base_root is None:
        return
    inv_base = tf.inverse_matrix(T_base_root)
    r2l = _root_to_link(base.model)
    entries = {e.get("frame"): e for e in (base.sub_frames or [])}
    n = 0
    skips: list[str] = []
    insert_seams = 0
    for seam in plan.seams:
        if seam.kind != "weld" or getattr(seam, "mate_type", "") != "insert":
            continue
        if seam.parent_sub != base_id:
            continue
        insert_seams += 1
        if seam.child_sub not in placed_root:
            skips.append(f"{seam.child_sub}.{seam.child_frame}: child not in placed_root "
                         f"(have {sorted(placed_root)})")
            continue
        child = subs.get(seam.child_sub)
        if child is None or child.model is None:
            skips.append(f"{seam.child_sub}: child sub/model missing")
            continue
        try:
            _cL, T_cRoot_cf = _frame_in_root(child, seam.child_frame)   # shaft frame in child-root
        except AssemblerError:
            skips.append(f"{seam.child_sub}.{seam.child_frame}: child_frame not found in child")
            continue
        T_world_shaft = placed_root[seam.child_sub] @ T_cRoot_cf        # shaft frame in WORLD
        _sax = T_world_shaft[:3, :3] @ np.array([0, 0, 1.0])
        log(f"[mesh] DIAG bore '{seam.parent_frame}' <- '{seam.child_sub}': shaft world spin(local+Z) "
            f"{np.round(_sax,3)}  placed_root rot-diag={np.round(np.diag(placed_root[seam.child_sub])[:3],3)}")
        entry = entries.get(seam.parent_frame)
        if entry is None:
            skips.append(f"{base_id}.{seam.parent_frame}: base seat frame unrealized "
                         f"(have {sorted(entries)})")
            continue                                                   # unrealized -> gate owns it
        bore_link = entry.get("link")
        T_root_bore_link = r2l.get(bore_link)
        if T_root_bore_link is None:
            skips.append(f"{base_id}.{seam.parent_frame}: bore link '{bore_link}' not in r2l")
            continue
        # want: T_base_root @ (r2l[bore_link] @ local_new) == T_world_shaft
        local_new = tf.inverse_matrix(T_root_bore_link) @ inv_base @ T_world_shaft
        xyz, rpy = _decompose(local_new)
        entry["local_xyz_m"] = list(xyz)
        entry["local_rpy_rad"] = list(rpy)
        n += 1
        log(f"[mesh] bore '{seam.parent_frame}' on base '{base_id}' relocated onto solved shaft "
            f"'{seam.child_sub}.{seam.child_frame}' (deterministic; ignores manager bore coords)")
    if n:
        log(f"deterministic bore placement: {n} housing seat(s) relocated onto their shafts")
    else:
        log(f"[mesh] deterministic bore placement: NO seats relocated on base '{base_id}' "
            f"({insert_seams} insert seam(s) targeting it)")
        for s in skips:
            log(f"[mesh]   skip: {s}")


def assemble(plan, subs: dict, ctx, *, settings=None, log_fn=print) -> KinematicModel:
    """Stitch the built subassemblies into one final KinematicModel + model.urdf.

    `subs` is {sub_id: SubResult} from orchestrator_boss.build_all_subassemblies.
    `ctx` is the assembly RunContext (its meshes_dir receives the consolidated STLs).
    Raises AssemblerError on a structural problem (missing sub/frame, or a resulting
    non-tree, surfaced by manager._validate_model).
    """
    def log(m):
        log_fn(f"[assembler] {m}")

    for s in plan.subassemblies:
        if s.id not in subs or subs[s.id].model is None:
            raise AssemblerError(f"missing built subassembly '{s.id}'")
    if plan.root_sub not in subs:
        raise AssemblerError(f"root subassembly '{plan.root_sub}' not built")

    os.makedirs(ctx.meshes_dir, exist_ok=True)

    # A sub may declare INSTANCES (identical repeated copies — 4 rotors, 6 legs). It is
    # built ONCE; here we stamp out one namespaced copy per instance. ns_ids[sub_id] is
    # the list of namespace ids to emit for that sub ([sub_id] normally, or
    # [sub_id_0, sub_id_1, ...] for N instances). inst_root_pose[ns_id] is the GLOBAL
    # pose of that instance's ROOT link (from the spec's instances[k]); a normal sub has
    # no entry and is placed by its weld seam as before.
    ns_ids: dict = {}
    inst_root_pose: dict = {}
    for s in plan.subassemblies:
        ids = _sub_instance_ids(s, s.id)
        ns_ids[s.id] = ids
        if len(ids) > 1:
            for k, ns_id in enumerate(ids):
                inst_root_pose[ns_id] = _mat(s.instances[k]["xyz_m"],
                                             s.instances[k]["rpy_rad"])

    # 1-3. Namespace + concatenate all links/poses; consolidate meshes. An instanced
    #    sub is built ONCE but stamped out once per instance (each copy gets its own
    #    namespaced links + STL copies; _validate_model keys mesh_filename off the link
    #    name, so per-instance copies are required).
    links: list = []
    poses: list = []
    mesh_pairs: list = []
    for s in plan.subassemblies:
        sub = subs[s.id]
        for ns_id in ns_ids[s.id]:
            links.extend(_namespaced_links(sub, ctx.meshes_dir, ns_id=ns_id))
            poses.extend(_namespaced_poses(sub, ns_id=ns_id))
            # Carry each sub's internal mesh_pairs (namespaced) into the final model so
            # the transmission-fail detector sees within-sub gear meshes too.
            for (a, b) in (sub.model.mesh_pairs or []):
                mesh_pairs.append((_ns(ns_id, a), _ns(ns_id, b)))
    n_inst = sum(len(v) for v in ns_ids.values())
    log(f"merged {len(links)} links + {len(poses)} internal poses from "
        f"{len(plan.subassemblies)} subassemblies ({n_inst} instance(s) total)")

    # 4. Bridge poses from WELD seams (power/gear-mesh seams add no placement pose — the
    #    housings are welded and the gears couple by contact at sim time; a power seam's
    #    mesh_pair is recorded in mesh_pairs instead). Process welds ROOT-FIRST (BFS from
    #    root_sub) so each parent is placed before its child, giving every sub its global
    #    root pose for the next hop. A weld whose CHILD is an instanced sub expands into
    #    ONE weld per instance (each placed at its own instances[k] global root pose).
    weld_by_parent: dict = {}
    for seam in plan.seams:
        if seam.kind == "weld":
            weld_by_parent.setdefault(seam.parent_sub, []).append(seam)

    # One cross-sub placement authority. The legacy closed-form solver may seed libslvs,
    # but its poses are never accepted by the authoritative backend as fallback output.
    gear_ids, base_id = _classify_subs(plan, subs)
    seed_placed = _place_mesh_cluster(plan, subs, gear_ids, base_id, log_fn) if gear_ids else {}
    backend = getattr(settings, "cross_sub_solver", "slvs") if settings is not None else "slvs"
    if gear_ids and backend == "slvs":
        from .slvs_adapter import report_dict, solve_cross_sub_placements, SlvsSolveError
        helpers = {"frame_in_root": _frame_in_root, "link_in_root": _link_in_root,
                   "gear_link": _gear_link, "gear_radius": _gear_pitch_r_mm}
        try:
            solved, problem = solve_cross_sub_placements(
                plan, subs, seed_placed, gear_ids, base_id,
                helpers=helpers, log_fn=log_fn)
        except SlvsSolveError as e:
            try:
                os.makedirs(ctx.run_dir, exist_ok=True)
                with open(os.path.join(ctx.run_dir, "assembly_constraint_report.json"), "w",
                          encoding="utf-8") as f:
                    json.dump({"backend": "slvs", "authority": "libslvs",
                               "status": "failed", "error": str(e)}, f, indent=2)
            except Exception:
                pass
            raise AssemblerError(f"authoritative libslvs solve failed: {e}") from e
        mesh_placed = solved.placements
        try:
            os.makedirs(ctx.run_dir, exist_ok=True)
            with open(os.path.join(ctx.run_dir, "assembly_constraint_report.json"), "w",
                      encoding="utf-8") as f:
                json.dump(report_dict(solved, problem), f, indent=2)
        except Exception as e:
            raise AssemblerError(f"could not persist authoritative constraint report: {e}") from e
    elif gear_ids and backend == "closed_form":
        mesh_placed = seed_placed
        log_fn("[slvs] WARNING: using explicit legacy closed_form backend")
    elif gear_ids:
        raise AssemblerError(f"unknown cross_sub_solver '{backend}'")
    else:
        mesh_placed = {}

    placed_root: dict = {plan.root_sub: np.eye(4)}
    placed_root.update(mesh_placed)
    n_weld = 0

    # In slvs mode the simultaneous constraint solution already owns all shaft/housing
    # placement. Running a per-bore override would introduce a second source of truth.
    if backend == "closed_form" and base_id is not None and mesh_placed:
        if base_id != plan.root_sub and base_id not in placed_root:
            _derive_base_pose(plan, subs, base_id, placed_root, log_fn)
        placed_root.setdefault(base_id, np.eye(4))
        _override_base_bores(plan, subs, base_id, placed_root, log_fn)
    elif backend == "slvs" and base_id is not None:
        placed_root.setdefault(base_id, np.eye(4))
        log_fn("[slvs] housing mount constraints solved simultaneously; legacy bore override disabled")

    queue = [plan.root_sub]
    seen = {plan.root_sub} | set(mesh_placed)       # mesh-placed subs are already positioned
    queue.extend(k for k in mesh_placed if k != plan.root_sub)

    # Each mesh-placed gear stage needs a forest edge placing its root. It has an ABSOLUTE
    # world pose from the solve, so parent it to the GLOBAL root link (at world origin) with
    # that world transform. (A gear stage that IS plan.root_sub stays at identity, no edge.)
    _global_root_link = _ns(plan.root_sub, subs[plan.root_sub].model.root_link)
    for gid, T_world in mesh_placed.items():
        if gid == plan.root_sub:
            continue
        xyz, rpy = _decompose(T_world)
        poses.append(PoseSpec(
            name=f"mesh_root_{gid}",
            parent=_global_root_link,
            child=_ns(gid, subs[gid].model.root_link),
            xyz_m=xyz, rpy_rad=rpy))
        n_weld += 1
    while queue:
        cur = queue.pop(0)
        for seam in weld_by_parent.get(cur, []):
            if seam.child_sub in seen:
                continue
            child_ids = ns_ids.get(seam.child_sub, [seam.child_sub])
            try:
                for ns_id in child_ids:
                    override = inst_root_pose.get(ns_id)   # instance -> fixed global pose
                    if override is not None:
                        # An INSTANCE copy sits at its own declared per-copy global pose —
                        # the one remaining place an absolute coordinate is authored (N
                        # identical copies at N distinct poses can't be a single port mate).
                        p, child_root_global = _bridge_pose_instance(
                            seam, subs, placed_root,
                            child_ns=ns_id, child_root_global=override)
                    else:
                        # Number-free placement: weld the child's frame onto the parent's
                        # REALIZED frame (mate_type is required on every non-instance weld).
                        p, child_root_global = _bridge_pose_from_ports(
                            seam, subs, placed_root, child_ns=ns_id)
                    poses.append(p)
                    placed_root[ns_id] = child_root_global
                    n_weld += 1
            except AssemblerError:
                raise
            except Exception as e:
                raise AssemblerError(f"seam '{seam.id}' bridge failed: {e}") from e
            seen.add(seam.child_sub)
            queue.append(seam.child_sub)
    # A power/gear-mesh seam names a cross-sub meshing pair; record it (namespaced by
    # each sub's own id) so the final model's mesh_pairs covers cross-seam meshes.
    for seam in plan.seams:
        if seam.kind == "power" and getattr(seam, "mesh_pair", ()):
            mp = seam.mesh_pair
            if len(mp) == 2:
                mesh_pairs.append((_ns(seam.parent_sub, mp[0]),
                                   _ns(seam.child_sub, mp[1])))
    n_power = sum(1 for s in plan.seams if s.kind == "power")
    log(f"added {n_weld} weld bridge pose(s); {n_power} power/mesh seam(s) couple "
        f"by contact (no pose)")

    # (Base pose + deterministic bore placement already ran BEFORE the weld-BFS above, so the
    # welded seat frames are the shaft-derived ones. The old post-BFS _derive_base_pose call here
    # was too late to affect the weld and is removed.)

    # The machine's single power INPUT is the driving link of the seam marked driver.
    # Pure contact needs the driver flag on a LINK (the physics test spins that part's
    # own dof); the boss marks it on a seam, so propagate it here. The driving link is
    # the seam's mesh_pair[0] on its owner_sub (or the parent frame's realized link).
    by_name = {l.name: l for l in links}
    driver_seam = next((s for s in plan.seams if getattr(s, "driver", False)), None)
    if driver_seam is not None:
        owner = getattr(driver_seam, "owner_sub", "") or driver_seam.parent_sub
        drive_link = ""
        if getattr(driver_seam, "mesh_pair", ()) and len(driver_seam.mesh_pair) == 2:
            drive_link = _ns(owner, driver_seam.mesh_pair[0])
        if drive_link not in by_name:
            # Fall back to the parent frame's realized link on the parent sub.
            try:
                pL, _ = _frame_in_root(subs[driver_seam.parent_sub],
                                       driver_seam.parent_frame)
                drive_link = _ns(driver_seam.parent_sub, pL)
            except Exception:
                drive_link = ""
        dl = by_name.get(drive_link)
        if dl is not None:
            if dl.dof == "fixed":
                dl.dof = "spin"          # a driven part must have a dof to actuate
            dl.driver = True
            log(f"marked '{drive_link}' as the machine driver (from seam "
                f"'{driver_seam.id}')")

    # 5. Build the final model; the global root is the root sub's namespaced root.
    root_link = _ns(plan.root_sub, subs[plan.root_sub].model.root_link)
    final = KinematicModel(name=plan.name, root_link=root_link,
                           links=links, poses=poses, mesh_pairs=mesh_pairs)

    # Compiler output: the SOLVED world pose of every subassembly interface frame, from
    # the placement just computed (placed_root[sub] = that sub's world pose). Post-assemble
    # gates (gear-mesh center distance) read THIS instead of the boss's authored xyz_m —
    # the boss no longer owns coordinates, the compiler does. Non-instance subs use their
    # sub_id key; an instanced sub contributes one entry per copy (keyed by copy ns_id).
    afw: list = []
    for s in plan.subassemblies:
        sub = subs[s.id]
        for ns_id in ns_ids[s.id]:
            root_world = placed_root.get(ns_id)
            if root_world is None:
                continue
            for fr in (s.frames or []):
                try:
                    _lnk, T_root_frame = _frame_in_root(sub, fr.name)
                except AssemblerError:
                    continue                       # unrealized frame -> other gate's job
                xyz, rpy = _decompose(root_world @ T_root_frame)
                afw.append({"sub": ns_id, "frame": fr.name,
                            "xyz_m": list(xyz), "rpy_rad": list(rpy)})
    final.assembly_frames_world = afw

    # Forest guard before validation: each non-root sub-root must be placed by exactly
    # one weld pose. Catches a plan whose welds don't span the machine (a sub with no
    # inbound weld would float at the origin). Instanced subs contribute one root per copy.
    non_root_sub_roots = {_ns(ns_id, subs[s.id].model.root_link)
                          for s in plan.subassemblies if s.id != plan.root_sub
                          for ns_id in ns_ids[s.id]}
    placed_count: dict = {}
    for p in poses:
        placed_count[p.child] = placed_count.get(p.child, 0) + 1
    for sr in non_root_sub_roots:
        c = placed_count.get(sr, 0)
        if c != 1:
            raise AssemblerError(
                f"subassembly root link '{sr}' has {c} placement poses (must be 1) — "
                f"the plan's weld seams do not form a connected placement")

    try:
        _validate_model(final)          # normalizes names + weak forest validation
    except ManagerError as e:
        raise AssemblerError(f"assembled model failed validation: {e}") from e

    build_urdf(final, ctx)
    # Save the assembled model next to model.urdf so the physics evaluator can load
    # it (physics._load_model reads kinematic_model.json for pose/dof/driver info).
    # Without this the assembled run has NO model -> robot_info is empty -> the
    # strategy selector defaults to a static-stability test and never drives the
    # mechanism (the "why is the tourbillon just a still box" failure).
    try:
        save_model(final, ctx.model_json_path)
    except Exception as e:
        log(f"WARNING: could not save assembled kinematic_model.json: {e}")
    ok, err = validate_urdf(ctx.urdf_path, require_meshes=False)
    if not ok:
        raise AssemblerError(f"assembled URDF topology invalid: {err}")
    ok2, err2 = validate_urdf(ctx.urdf_path, require_meshes=True)
    log(f"wrote {ctx.urdf_path} (links={len(final.links)}, poses={len(final.poses)}, "
        f"root='{final.root_link}', meshes ok={ok2})")
    _write_assembled_mjcf(final, ctx, settings, log)
    return final


# --------------------------------------------------------------------------- #
# Silent overlap auto-nudge (Session B item 1b)
# --------------------------------------------------------------------------- #

# Two subs are "overlapping" only if their world AABBs interpenetrate by more than
# this along every axis — a small touch at a weld seam is fine.
_NUDGE_MIN_OVERLAP_M = 0.005      # 5 mm
_NUDGE_CLEAR_MARGIN_M = 0.002     # push this much past just-touching
_NUDGE_MAX_PASSES = 4


def _child_weld_pose(final, plan, subs, sub_id):
    """The single weld PoseSpec that places `sub_id`'s subtree (its inbound seam
    pose, named seam_<seamid> with child = <sub_id>_<sub_root>), or None for the root
    / an instanced sub. Nudging this pose's origin translates the whole subtree."""
    sub = subs.get(sub_id)
    if sub is None or sub.model is None:
        return None
    child_root = _ns(sub_id, sub.model.root_link)
    for p in final.poses:
        if p.child == child_root:
            return p
    return None


def _sub_world_bounds(robot, plan) -> dict:
    """World AABB (min,max) of each subassembly's geometry, or None if a sub has no
    usable mesh bounds. Resolves scene geometry robustly across yourdfpy keyings: it
    keys robot.scene.geometry by '<link>_visual' (assembled URDFs), sometimes by the
    mesh basename, or by link name — so we try each. (precheck._sub_bounds only tries
    the link name and therefore silently finds nothing on an assembled URDF.)"""
    import numpy as _np
    geom_keys = list(robot.scene.geometry.keys()) if hasattr(robot, "scene") else []
    base_by_link: dict = {}
    for l in robot.robot.links:
        fn = ""
        try:
            vis = l.visuals[0] if getattr(l, "visuals", None) else None
            if vis and vis.geometry and getattr(vis.geometry, "mesh", None):
                fn = os.path.basename(vis.geometry.mesh.filename or "")
        except Exception:
            fn = ""
        base_by_link[l.name] = fn

    def _geom_for(link_name):
        for key in (f"{link_name}_visual", base_by_link.get(link_name), link_name):
            if key and key in robot.scene.geometry:
                return robot.scene.geometry[key]
        # Last resort: a scene key that starts with the link name.
        for key in geom_keys:
            if key.startswith(link_name):
                return robot.scene.geometry[key]
        return None

    out: dict = {}
    for s in plan.subassemblies:
        lo = _np.array([_np.inf] * 3)
        hi = _np.array([-_np.inf] * 3)
        found = False
        for l in robot.robot.links:
            if not l.name.startswith(f"{s.id}_"):
                continue
            try:
                T = robot.get_transform(frame_to=l.name, frame_from=robot.base_link)
            except Exception:
                continue
            geom = _geom_for(l.name)
            if geom is None or not hasattr(geom, "bounds") or geom.bounds is None:
                continue
            corners = _corners_world(geom.bounds, T)
            if corners is None:
                continue
            lo = _np.minimum(lo, corners.min(axis=0))
            hi = _np.maximum(hi, corners.max(axis=0))
            found = True
        out[s.id] = (lo, hi) if found and _np.all(_np.isfinite(lo)) else None
    return out


def _corners_world(bounds, T):
    """World AABB corners of a local [min,max] box under transform T (mesh scale is
    already baked into the loaded geometry bounds)."""
    try:
        lo, hi = np.asarray(bounds[0]), np.asarray(bounds[1])
    except Exception:
        return None
    corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                        for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    h = np.hstack([corners, np.ones((8, 1))])
    return (h @ T.T)[:, :3]


def auto_nudge_overlaps(final, plan, subs, ctx, *, settings=None, log_fn=print) -> dict:
    """Silently separate subassemblies that interpenetrate but share NO seam, so the
    CURRENT assembled URDF always closes, and record each nudge for the manager to fix
    next iteration. Mutates `final` (weld joint origins) + rewrites model.urdf.

    Returns {sub_id: [dx, dy, dz]} (meters) of the nudges applied. For each overlapping
    NON-seamed pair, the CHILD sub (the one deeper from the root, by weld-tree BFS order)
    is translated along the shortest separation axis until the boxes just clear. The
    nudge shifts that sub's inbound weld joint origin, so its whole subtree moves with
    it. Bounded to a few passes; logs an ARTIFACT so the UI shows the move. Keeps
    precheck's within-sub interpenetration check (route-back) untouched — this only
    handles cross-sub box overlap the boss's layout produced.
    """
    from .viz import load_robot

    def log(m):
        log_fn(f"[assembler] {m}")

    def _load(urdf):
        """Load the assembled URDF resolving meshes relative to the URDF's OWN dir
        (robust to the process CWD), falling back to viz.load_robot."""
        import yourdfpy
        base = os.path.dirname(os.path.abspath(urdf))
        try:
            return yourdfpy.URDF.load(
                urdf, load_meshes=True, build_scene_graph=True, force_mesh=True,
                filename_handler=lambda fname: os.path.join(base, fname))
        except Exception:
            return load_robot(urdf)

    # Depth of each sub from the root over the weld tree -> who is the "child" in a pair.
    weld_adj: dict = {}
    for seam in plan.seams:
        if seam.kind == "weld":
            weld_adj.setdefault(seam.parent_sub, []).append(seam.child_sub)
            weld_adj.setdefault(seam.child_sub, []).append(seam.parent_sub)
    depth = {plan.root_sub: 0}
    q = [plan.root_sub]
    while q:
        cur = q.pop(0)
        for nb in weld_adj.get(cur, []):
            if nb not in depth:
                depth[nb] = depth[cur] + 1
                q.append(nb)

    seamed = set()
    for seam in plan.seams:
        seamed.add(frozenset((seam.parent_sub, seam.child_sub)))

    # Subs coupled by a gear MESH (power seam) form a rigid cluster whose relative poses are
    # solved precisely by _place_mesh_cluster to hit true center-distance. Their world AABBs
    # routinely overlap along the shaft line (large gear radii), but they must NEVER be nudged
    # apart — that breaks the mesh and flings a stage across the scene. Treat every pair of subs
    # in the same mesh cluster as seamed so the box-overlap separator leaves them alone. The
    # cluster is the transitive closure over power seams.
    mesh_adj: dict = {}
    for seam in plan.seams:
        if seam.kind == "power":
            mesh_adj.setdefault(seam.parent_sub, set()).add(seam.child_sub)
            mesh_adj.setdefault(seam.child_sub, set()).add(seam.parent_sub)
    _mesh_seen: set = set()
    for start in list(mesh_adj):
        if start in _mesh_seen:
            continue
        comp = []
        stack = [start]
        while stack:
            n = stack.pop()
            if n in _mesh_seen:
                continue
            _mesh_seen.add(n)
            comp.append(n)
            stack.extend(mesh_adj.get(n, ()))
        for i in range(len(comp)):
            for j in range(i + 1, len(comp)):
                seamed.add(frozenset((comp[i], comp[j])))

    nudges: dict = {}
    for _pass in range(_NUDGE_MAX_PASSES):
        try:
            robot = _load(ctx.urdf_path)
        except Exception as e:
            log(f"nudge: could not load URDF ({e}); skipping")
            return nudges
        bounds = _sub_world_bounds(robot, plan)
        ids = [s.id for s in plan.subassemblies]
        moved_any = False
        for i in range(len(ids)):
            for k in range(i + 1, len(ids)):
                a, b = ids[i], ids[k]
                if frozenset((a, b)) in seamed:
                    continue
                ba, bb = bounds.get(a), bounds.get(b)
                if ba is None or bb is None:
                    continue
                ov = _overlap_vec(ba, bb)                 # per-axis penetration (m), or None
                if ov is None or float(np.max(ov)) < _NUDGE_MIN_OVERLAP_M:
                    continue
                # Move the DEEPER sub (bigger depth); tie -> the later id. Never the root.
                child = a if depth.get(a, 0) >= depth.get(b, 0) else b
                if child == plan.root_sub:
                    child = b if child == a else a
                joint = _child_weld_pose(final, plan, subs, child)
                if joint is None:
                    continue
                # Shortest-axis separation: push along the axis of MIN penetration by
                # exactly that penetration + a margin, signed away from the other sub.
                axis = int(np.argmin(ov))
                ca = (ba[0] + ba[1]) / 2.0
                cb = (bb[0] + bb[1]) / 2.0
                other_center = cb if child == a else ca
                child_center = ca if child == a else cb
                sign = 1.0 if child_center[axis] >= other_center[axis] else -1.0
                delta = np.zeros(3)
                delta[axis] = sign * (float(ov[axis]) + _NUDGE_CLEAR_MARGIN_M)
                xyz = list(joint.xyz_m)
                xyz[axis] += delta[axis]
                joint.xyz_m = tuple(xyz)
                prev = np.array(nudges.get(child, [0.0, 0.0, 0.0]))
                nudges[child] = (prev + delta).tolist()
                log(f"nudge: '{child}' moved {delta[axis]*1000:+.1f} mm on axis "
                    f"{'xyz'[axis]} to clear '{a if child==b else b}'")
                moved_any = True
        if not moved_any:
            break
        # Re-emit the URDF with the shifted weld origins so the next pass re-measures.
        build_urdf(final, ctx)

    if nudges:
        try:
            save_model(final, ctx.model_json_path)
        except Exception as e:
            log(f"nudge: could not re-save kinematic_model.json: {e}")
        _write_assembled_mjcf(final, ctx, settings, log)
        import json as _json
        print("ARTIFACT_JSON:" + _json.dumps({
            "kind": "auto_nudge", "run_dir": ctx.run_dir,
            "nudges": {k: [round(x, 4) for x in v] for k, v in nudges.items()}}),
            flush=True)
        log(f"auto-nudge separated {len(nudges)} subassembly(ies): "
            f"{ {k: [round(x*1000,1) for x in v] for k, v in nudges.items()} } (mm)")
    return nudges


def _overlap_vec(ba, bb):
    """Per-axis penetration depth (meters) of two world AABBs, or None if they don't
    overlap on all three axes. penetration = min(hiA,hiB) - max(loA,loB) per axis."""
    (lo_a, hi_a), (lo_b, hi_b) = ba, bb
    lo_i = np.maximum(lo_a, lo_b)
    hi_i = np.minimum(hi_a, hi_b)
    pen = hi_i - lo_i
    if np.any(pen <= 0):
        return None
    return pen


# --------------------------------------------------------------------------- #
# CLI (Stage C verification): plan on disk + built subs -> assembled URDF.
# --------------------------------------------------------------------------- #

def main() -> int:
    import argparse
    import sys
    from .boss import load_plan
    from .orchestrator import make_run_context
    from .orchestrator_boss import _load_sub_from_disk

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except Exception:
            pass

    ap = argparse.ArgumentParser(
        description="maker2 assembler: stitch a plan's built subs into one URDF")
    ap.add_argument("--plan", required=True, help="path to subassembly_plan.json")
    a = ap.parse_args()

    session_root = os.path.dirname(os.path.abspath(a.plan))
    plan = load_plan(a.plan)
    subs = {s.id: _load_sub_from_disk(s.id, session_root, log_fn=print)
            for s in plan.subassemblies}
    missing = [sid for sid, r in subs.items() if r.model is None]
    if missing:
        print(f"[assembler] cannot assemble: missing built subs {missing}")
        return 1
    ctx = make_run_context(plan.name, session_root,
                           run_dir=os.path.join(session_root, "assembly"))
    try:
        final = assemble(plan, subs, ctx, log_fn=print)
    except AssemblerError as e:
        print(f"[assembler] FAILED: {e}")
        return 1
    print("-" * 56)
    print(f"RESULT: assembled {len(final.links)} links / {len(final.poses)} poses "
          f"-> {ctx.urdf_path}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
