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
     mesh_filename) and writes model.urdf via build_urdf.

See .claude/plans/precious-humming-wand.md.
"""

from __future__ import annotations

import os
import shutil

import numpy as np
import trimesh.transformations as tf

from .manager import ManagerError, _validate_model, save_model
from .model import KinematicModel, LinkSpec, PoseSpec
from .urdf_builder import build_urdf, validate_urdf


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

def _global_frame_pose(plan, sub_id: str, frame_name: str) -> np.ndarray:
    """The boss's GLOBAL pose (4x4) for a subassembly's interface frame."""
    sub = plan.sub_by_id(sub_id)
    for f in (sub.frames if sub else []):
        if f.name == frame_name:
            return _mat(f.xyz_m, f.rpy_rad)
    raise AssemblerError(f"plan sub '{sub_id}' has no frame '{frame_name}'")


def _bridge_pose(seam, plan, subs: dict, placed_root: dict, *,
                 child_ns: str | None = None,
                 child_root_global_override=None) -> tuple:
    """A fixed WELD pose that places the child sub at the boss's GLOBAL layout.

    The boss assigns each interface frame a GLOBAL pose; the child sub must sit so
    ITS child_frame lands at that global pose. With the parent sub already placed at
    placed_root[parent_sub] (root sub = identity), the weld hangs the child's ROOT
    under the parent's realized frame link. Returns (PoseSpec, child_root_global).

    ``child_ns`` overrides the child's namespace (an instanced sub passes its per-copy
    id). ``child_root_global_override`` (an instance's fixed global root pose from
    instances[k]) bypasses the frame-derived placement so identical copies land at
    their declared poses.
    """
    parent = subs[seam.parent_sub]
    child = subs[seam.child_sub]
    child_ns = child_ns or child.id
    pL, T_pRoot_pf = _frame_in_root(parent, seam.parent_frame)      # parent frame in parent-root

    if child_root_global_override is not None:
        child_root_global = child_root_global_override
    else:
        cL, T_cRoot_cf = _frame_in_root(child, seam.child_frame)    # child frame in child-root
        # Where the child's own frame must land, in GLOBAL coords (the boss's intent).
        G_cf = _global_frame_pose(plan, seam.child_sub, seam.child_frame)
        # Child root pose in global so its frame lands at G_cf.
        child_root_global = G_cf @ tf.inverse_matrix(T_cRoot_cf)

    # Express the weld origin relative to the PARENT's realized link (the pose parent):
    #   origin = (global->parentRoot) ∘ (parentRoot->pL)  then invert to pL frame,
    #   applied to child_root_global.
    parent_root_global = placed_root[seam.parent_sub]
    r2l_parent = _root_to_link(parent.model)
    T_global_pL = parent_root_global @ r2l_parent[pL]
    T_origin = tf.inverse_matrix(T_global_pL) @ child_root_global
    xyz, rpy = _decompose(T_origin)
    p = PoseSpec(
        name=f"seam_{seam.id}_{child_ns}" if child_root_global_override is not None
             else f"seam_{seam.id}",
        parent=_ns(parent.id, pL), child=_ns(child_ns, child.model.root_link),
        xyz_m=xyz, rpy_rad=rpy)
    return p, child_root_global


# --------------------------------------------------------------------------- #
# Assemble
# --------------------------------------------------------------------------- #

def assemble(plan, subs: dict, ctx, *, log_fn=print) -> KinematicModel:
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
    placed_root: dict = {plan.root_sub: np.eye(4)}
    n_weld = 0
    queue = [plan.root_sub]
    seen = {plan.root_sub}
    while queue:
        cur = queue.pop(0)
        for seam in weld_by_parent.get(cur, []):
            if seam.child_sub in seen:
                continue
            child_ids = ns_ids.get(seam.child_sub, [seam.child_sub])
            try:
                for ns_id in child_ids:
                    override = inst_root_pose.get(ns_id)   # instance -> fixed global pose
                    p, child_root_global = _bridge_pose(
                        seam, plan, subs, placed_root,
                        child_ns=ns_id, child_root_global_override=override)
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


def auto_nudge_overlaps(final, plan, subs, ctx, *, log_fn=print) -> dict:
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
