"""Deterministic assembler (Stage C): stitch per-subassembly KinematicModels into
ONE final KinematicModel using the boss's SubassemblyPlan.

The boss (Stage A) is the only LLM that reasons about seams; this module is PURE
PYTHON so surgical re-runs re-assemble identically, the biggest artifact never
hits the token cap, and the single-tree invariant is enforced by reusing
manager._validate_model. It:

  1. Namespaces every sub's links/joints -> "<sub_id>_<name>" (remapping endpoints,
     root_link, mesh_filename).
  2. Consolidates each sub's meshes into one meshes/ dir (copy, not symlink -> Windows).
  3. Concatenates all links + joints; the global root is the root of plan.root_sub.
  4. Adds BRIDGE joints from the plan's seams:
       weld  -> a fixed joint from the parent sub's realized frame link to the CHILD
                sub's ROOT, with an origin computed so the two frames coincide.
       power (gear MESH, the milestone) -> NO cross-seam joint; the housing weld holds
                the gear centers one mesh-distance apart and the gears couple by tooth
                contact (self_collision) at sim time.
  5. Re-validates via manager._validate_model (single tree / no cycles / mesh_filename)
     and writes model.urdf via build_urdf.

See .claude/plans/precious-humming-wand.md.
"""

from __future__ import annotations

import os
import shutil

import numpy as np
import trimesh.transformations as tf

from .manager import ManagerError, _validate_model, save_model
from .model import JointSpec, KinematicModel, LinkSpec
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
    the joint tree (root at identity). Uses each joint's own xyz_m/rpy_rad."""
    children: dict[str, list] = {}
    for j in model.joints:
        children.setdefault(j.parent, []).append(j)
    T: dict[str, np.ndarray] = {model.root_link: np.eye(4)}
    stack = [model.root_link]
    while stack:
        node = stack.pop()
        for j in children.get(node, []):
            T[j.child] = T[node] @ _mat(j.xyz_m, j.rpy_rad)
            stack.append(j.child)
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
            mesh_filename=rel))
    return out


def _namespaced_joints(sub, ns_id: str | None = None) -> list:
    ns_id = ns_id or sub.id
    out = []
    for j in sub.model.joints:
        out.append(JointSpec(
            name=_ns(ns_id, j.name), type=j.type,
            parent=_ns(ns_id, j.parent), child=_ns(ns_id, j.child),
            xyz_m=tuple(j.xyz_m), rpy_rad=tuple(j.rpy_rad), axis=tuple(j.axis),
            lower=j.lower, upper=j.upper, effort=j.effort, velocity=j.velocity,
            driver=j.driver))
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


def _bridge_joint(seam, plan, subs: dict, placed_root: dict, *,
                  child_ns: str | None = None,
                  child_root_global_override=None) -> tuple:
    """A fixed WELD joint that places the child sub at the boss's GLOBAL layout.

    The boss assigns each interface frame a GLOBAL pose; the child sub must sit so
    ITS child_frame lands at that global pose. With the parent sub already placed at
    placed_root[parent_sub] (root sub = identity), the weld hangs the child's ROOT
    under the parent's realized frame link. Returns (JointSpec, child_root_global).

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

    # Express the weld origin relative to the PARENT's realized link (the joint parent):
    #   origin = (global->parentRoot) ∘ (parentRoot->pL)  then invert to pL frame,
    #   applied to child_root_global.
    parent_root_global = placed_root[seam.parent_sub]
    r2l_parent = _root_to_link(parent.model)
    T_global_pL = parent_root_global @ r2l_parent[pL]
    T_origin = tf.inverse_matrix(T_global_pL) @ child_root_global
    xyz, rpy = _decompose(T_origin)
    j = JointSpec(
        name=f"seam_{seam.id}_{child_ns}" if child_root_global_override is not None
             else f"seam_{seam.id}",
        type="fixed",
        parent=_ns(parent.id, pL), child=_ns(child_ns, child.model.root_link),
        xyz_m=xyz, rpy_rad=rpy)
    return j, child_root_global


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

    # 1-3. Namespace + concatenate all links/joints; consolidate meshes. An instanced
    #    sub is built ONCE but stamped out once per instance (each copy gets its own
    #    namespaced links + STL copies; _validate_model keys mesh_filename off the link
    #    name, so per-instance copies are required).
    links: list = []
    joints: list = []
    for s in plan.subassemblies:
        sub = subs[s.id]
        for ns_id in ns_ids[s.id]:
            links.extend(_namespaced_links(sub, ctx.meshes_dir, ns_id=ns_id))
            joints.extend(_namespaced_joints(sub, ns_id=ns_id))
    n_inst = sum(len(v) for v in ns_ids.values())
    log(f"merged {len(links)} links + {len(joints)} internal joints from "
        f"{len(plan.subassemblies)} subassemblies ({n_inst} instance(s) total)")

    # 4. Bridge joints from WELD seams (power/gear-mesh seams add no joint — the
    #    housings are welded and the gears couple by contact at sim time). Process
    #    welds ROOT-FIRST (BFS from root_sub) so each parent is placed before its
    #    child, giving every sub its global root pose for the next hop. A weld whose
    #    CHILD is an instanced sub expands into ONE weld per instance (each placed at
    #    its own instances[k] global root pose).
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
                    j, child_root_global = _bridge_joint(
                        seam, plan, subs, placed_root,
                        child_ns=ns_id, child_root_global_override=override)
                    joints.append(j)
                    placed_root[ns_id] = child_root_global
                    n_weld += 1
            except AssemblerError:
                raise
            except Exception as e:
                raise AssemblerError(f"seam '{seam.id}' bridge failed: {e}") from e
            seen.add(seam.child_sub)
            queue.append(seam.child_sub)
    n_power = sum(1 for s in plan.seams if s.kind == "power")
    log(f"added {n_weld} weld bridge joint(s); {n_power} power/mesh seam(s) couple "
        f"by contact (no joint)")

    # 5. Build the final model; the global root is the root sub's namespaced root.
    root_link = _ns(plan.root_sub, subs[plan.root_sub].model.root_link)
    final = KinematicModel(name=plan.name, root_link=root_link,
                           links=links, joints=joints)

    # Single-tree guard before validation: each non-root sub-root must have exactly
    # one parent joint (its weld). Catches a plan whose welds don't span the machine.
    # Instanced subs contribute one root per copy.
    non_root_sub_roots = {_ns(ns_id, subs[s.id].model.root_link)
                          for s in plan.subassemblies if s.id != plan.root_sub
                          for ns_id in ns_ids[s.id]}
    child_count: dict = {}
    for j in joints:
        child_count[j.child] = child_count.get(j.child, 0) + 1
    for sr in non_root_sub_roots:
        c = child_count.get(sr, 0)
        if c != 1:
            raise AssemblerError(
                f"subassembly root link '{sr}' has {c} parent joints (must be 1) — "
                f"the plan's weld seams do not form a single tree")

    try:
        _validate_model(final)          # normalizes + enforces the single-tree invariant
    except ManagerError as e:
        raise AssemblerError(f"assembled model is not a valid single tree: {e}") from e

    build_urdf(final, ctx)
    # Save the assembled model next to model.urdf so the physics evaluator can load
    # it (physics._load_model reads kinematic_model.json for joint/driver/role info).
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
    log(f"wrote {ctx.urdf_path} (links={len(final.links)}, joints={len(final.joints)}, "
        f"root='{final.root_link}', meshes ok={ok2})")
    return final


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
    print(f"RESULT: assembled {len(final.links)} links / {len(final.joints)} joints "
          f"-> {ctx.urdf_path}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
