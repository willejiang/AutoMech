"""Authoritative libslvs cross-subassembly placement solver.

The legacy closed-form cluster is used only as an initial guess. Accepted world poses
come from a single libslvs constraint solve and backend-independent validation.
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict

import numpy as np
import trimesh.transformations as tf

from .constraint_ir import (AssemblyConstraintProblem, ConstraintKind, ConstraintSolveResult,
                            ConstraintSpec, EntityKind, EntitySpec, PlacementResult,
                            RigidStageSpec)

_GAUGE_M = 0.100
_POS_TOL_M = 1e-6
_AXIS_TOL_DEG = 0.1


class SlvsSolveError(RuntimeError):
    def __init__(self, message, *, problem=None, result=None, placements=None, failure_report=None):
        super().__init__(message)
        self.problem = problem
        self.result = result
        self.placements = placements
        self.failure_report = failure_report


def _unit(v):
    a = np.asarray(v, float)
    n = float(np.linalg.norm(a))
    if a.shape != (3,) or not np.all(np.isfinite(a)) or n < 1e-12:
        raise SlvsSolveError("non-finite or zero axis")
    return a / n


def _se3(T):
    M = np.asarray(T, float)
    if M.shape != (4, 4) or not np.all(np.isfinite(M)):
        return False
    R = M[:3, :3]
    return (np.allclose(M[3], [0, 0, 0, 1], atol=1e-8)
            and np.allclose(R.T @ R, np.eye(3), atol=1e-6)
            and abs(float(np.linalg.det(R)) - 1.0) < 1e-6)


def _rot_a_to_b(a, b):
    a, b = _unit(a), _unit(b)
    c = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if c > 1 - 1e-10:
        return np.eye(3)
    if c < -1 + 1e-10:
        p = np.cross(a, [1., 0., 0.])
        if np.linalg.norm(p) < 1e-8:
            p = np.cross(a, [0., 1., 0.])
        return tf.rotation_matrix(math.pi, _unit(p))[:3, :3]
    v = _unit(np.cross(a, b))
    return tf.rotation_matrix(math.acos(c), v)[:3, :3]


def slvs_available():
    try:
        from py_slvs import slvs
        return True, getattr(slvs, "__file__", "py_slvs.slvs")
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"


def _frame_spec(plan, sid, name):
    s = next((x for x in plan.subassemblies if x.id == sid), None)
    return next((f for f in (s.frames if s else []) if f.name == name), None)


def _insert_seam(plan, base_id, child_id):
    def typed_mount(s):
        if s.kind != "weld" or s.parent_sub != base_id or s.child_sub != child_id:
            return False
        mate = getattr(s, "mate_type", "")
        if mate == "insert":
            return True
        if mate != "seat":
            return False
        pf, cf = _frame_spec(plan, s.parent_sub, s.parent_frame), _frame_spec(plan, s.child_sub, s.child_frame)
        if pf is None or cf is None:
            return False
        try:
            pa, ca = _unit(pf.axis), _unit(cf.axis)
            return (pf.role == "mount" and cf.role in ("mount", "power_in", "power_out")
                    and float(pf.shaft_dia_mm) > 0 and float(cf.shaft_dia_mm) > 0
                    and abs(float(np.dot(pa, ca))) >= .99)
        except Exception:
            return False
    return next((s for s in plan.seams if typed_mount(s)), None)


def build_cross_sub_problem(plan, subs, seed, gear_ids, base_id, *,
                            frame_in_root, link_in_root, gear_link, gear_radius):
    """Build the authoritative reducer problem. Housing targets are hard constraints."""
    if not gear_ids or base_id is None or base_id != plan.root_sub:
        raise SlvsSolveError("slvs MVP requires one root housing and a gear cluster")
    stages = {}
    entities = []
    constraints = []
    for sid in sorted(gear_ids):
        seam = _insert_seam(plan, base_id, sid)
        if seam is None:
            raise SlvsSolveError(f"stage '{sid}' has no housing shaft-mount seam")
        try:
            _shaft_link, T_local_anchor = frame_in_root(subs[sid], seam.child_frame)
            _base_link, T_target = frame_in_root(subs[base_id], seam.parent_frame)
        except Exception as e:
            raise SlvsSolveError(f"stage '{sid}' mount frame unresolved: {e}") from e
        target_fr = _frame_spec(plan, base_id, seam.parent_frame)
        target_axis = _unit(target_fr.axis if target_fr is not None else T_target[:3, 2])
        # frames_realized.rpy is an authoring hint and is often rotated incorrectly.
        # Resolve the actual rotating shaft/carrier link (mount frames are frequently on a
        # fixed bearing whose spin_axis is zero), then transform its axis into sub-root.
        candidates=[l for l in subs[sid].model.links
                    if (getattr(l,'dof','')=='spin' or 'shaft' in l.name.lower())
                    and np.linalg.norm(np.asarray(getattr(l,'spin_axis',(0,0,0)),float))>1e-9]
        carrier=next((l for l in candidates if 'shaft' in l.name.lower()),candidates[0] if candidates else None)
        if carrier is None: raise SlvsSolveError(f"stage '{sid}' has no built shaft axis")
        local_axis, _shaft_center = link_in_root(subs[sid], carrier.name)
        local_axis = _unit(local_axis)
        stage = RigidStageSpec(sid, np.asarray(seed[sid]).tolist(),
                               tuple(T_local_anchor[:3, 3]), tuple(local_axis),
                               tuple(T_target[:3, 3]), tuple(target_axis), {},
                               seam.id, seam.parent_frame, seam.child_frame)
        stages[sid] = stage
        O, A = f"{sid}:anchor", f"{sid}:axis_end"
        seed_O = (np.asarray(seed[sid]) @ np.append(T_local_anchor[:3, 3], 1.0))[:3]
        seed_A = seed_O + _GAUGE_M * (np.asarray(seed[sid])[:3, :3] @ local_axis)
        entities.extend([EntitySpec(O, EntityKind.POINT_3D, tuple(seed_O)),
                         EntitySpec(A, EntityKind.POINT_3D, tuple(seed_A)),
                         EntitySpec(f"{sid}:axis", EntityKind.LINE_3D, refs=(O, A))])
        # Housing target is fixed and authoritative; coincidence is the hard mount constraint.
        target = f"housing:{seam.parent_frame}"
        target_end = f"housing:{seam.parent_frame}:axis_end"
        entities.extend([
            EntitySpec(target, EntityKind.POINT_3D, tuple(T_target[:3, 3]), fixed=True,
                       provenance={"seam": seam.id, "frame": seam.parent_frame}),
            EntitySpec(target_end, EntityKind.POINT_3D,
                       tuple(T_target[:3, 3] + _GAUGE_M * target_axis), fixed=True,
                       provenance={"seam": seam.id, "frame": seam.parent_frame}),
        ])
        constraints.extend([
            ConstraintSpec(f"{seam.id}:anchor", ConstraintKind.COINCIDENT,
                           (O, target), provenance={"seam": seam.id}),
            ConstraintSpec(f"{seam.id}:axis", ConstraintKind.COINCIDENT,
                           (A, target_end), provenance={"seam": seam.id}),
        ])

    gear_pairs = []
    for seam in plan.seams:
        if seam.kind != "power":
            continue
        pa, ch = subs[seam.parent_sub], subs[seam.child_sub]
        gp, gc = gear_link(pa, seam, 0), gear_link(ch, seam, 1)
        if gp is None or gc is None:
            raise SlvsSolveError(f"power seam '{seam.id}' cannot resolve built gear links")
        rp, rc = gear_radius(gp), gear_radius(gc)
        if not rp or not rc:
            raise SlvsSolveError(f"power seam '{seam.id}' missing pitch radii")
        paxis, pcenter = link_in_root(pa, gp.name)
        caxis, ccenter = link_in_root(ch, gc.name)
        for sid, stage, axis, center, gear in ((seam.parent_sub, stages[seam.parent_sub], paxis, pcenter, gp),
                                                (seam.child_sub, stages[seam.child_sub], caxis, ccenter, gc)):
            la = np.asarray(stage.local_axis)
            radial = np.asarray(center) - np.asarray(stage.local_anchor_m)
            axial = float(np.dot(radial, la))
            off = radial - axial * la
            if np.linalg.norm(off) > 1e-4:
                raise SlvsSolveError(f"gear '{sid}.{gear.name}' center is not on its stage shaft axis")
            if abs(abs(float(np.dot(_unit(axis), la))) - 1.0) > 1e-4:
                raise SlvsSolveError(f"gear '{sid}.{gear.name}' axis disagrees with its stage shaft axis")
            stage.gear_centers_local_m[gear.name] = tuple(center)
            eid = f"{sid}:gear:{gear.name}"
            seed_center = (np.asarray(seed[sid]) @ np.append(center, 1.0))[:3]
            entities.append(EntitySpec(eid, EntityKind.POINT_3D, tuple(seed_center)))
            constraints.append(ConstraintSpec(f"{eid}:on_axis", ConstraintKind.POINT_ON_LINE,
                                              (eid, f"{sid}:axis"), provenance={"gear": gear.name}))
            # py-slvs projected-distance sign is opposite its line orientation.
            constraints.append(ConstraintSpec(f"{eid}:axial", ConstraintKind.PROJECTED_DISTANCE,
                                              (f"{sid}:anchor", eid, f"{sid}:axis"), -axial,
                                              provenance={"gear": gear.name}))
        pe, ce = f"{seam.parent_sub}:gear:{gp.name}", f"{seam.child_sub}:gear:{gc.name}"
        target = (rp + rc) / 1000.0
        # The mount coincidences fully determine both stage frames; adding the implied
        # center distance to libslvs is algebraically redundant (RESULT_REDUNDANT). Keep
        # it in the authoritative IR and enforce it in post-solve validation instead.
        constraints.append(ConstraintSpec(f"{seam.id}:center_distance", ConstraintKind.DISTANCE,
                                          (pe, ce), target, enforced_by_solver=False,
                                          provenance={"seam": seam.id}))
        gear_pairs.append({"seam_id": seam.id, "parent_sub": seam.parent_sub,
                           "child_sub": seam.child_sub, "parent_entity": pe, "child_entity": ce,
                           "parent_part": gp.name, "child_part": gc.name,
                           "parent_mesh_frame": seam.parent_frame,
                           "child_mesh_frame": seam.child_frame,
                           "parent_pitch_radius_mm": float(rp),
                           "child_pitch_radius_mm": float(rc),
                           "target_m": target})
    return AssemblyConstraintProblem(stages=stages, entities=entities, constraints=constraints,
                                     gear_pairs=gear_pairs, expected_dof=0, base_id=base_id)


def solve_problem(problem):
    from py_slvs import slvs
    S = slvs.System()
    handles = {}
    constraint_handles = {}
    fixed_group, solve_group = 1, 2
    for e in problem.entities:
        if e.kind == EntityKind.POINT_3D:
            handles[e.id] = S.addPoint3dV(*[v * 1000.0 for v in e.initial_m],
                                          group=fixed_group if e.fixed else solve_group)
    for e in problem.entities:
        if e.kind == EntityKind.LINE_3D:
            handles[e.id] = S.addLineSegment(handles[e.refs[0]], handles[e.refs[1]], group=solve_group)
    for c in problem.constraints:
        if not c.enforced_by_solver:
            continue
        h = None
        if c.kind == ConstraintKind.COINCIDENT:
            h = S.addPointsCoincident(handles[c.entities[0]], handles[c.entities[1]], group=solve_group)
        elif c.kind == ConstraintKind.DISTANCE:
            h = S.addPointsDistance(c.value_m * 1000.0, handles[c.entities[0]], handles[c.entities[1]],
                                    group=solve_group)
        elif c.kind == ConstraintKind.POINT_ON_LINE:
            h = S.addPointOnLine(handles[c.entities[0]], handles[c.entities[1]], group=solve_group)
        elif c.kind == ConstraintKind.PROJECTED_DISTANCE:
            h = S.addPointsProjectDistance(c.value_m * 1000.0, handles[c.entities[0]],
                                           handles[c.entities[1]], handles[c.entities[2]],
                                           group=solve_group)
        else:
            raise SlvsSolveError(f"unsupported IR constraint {c.kind}")
        constraint_handles[int(h)] = c.id
    raw = int(S.solve(group=solve_group, reportFailed=True, findFreeParams=True))
    status = {0: "okay", 1: "inconsistent", 2: "didnt_converge", 3: "too_many_unknowns",
              4: "init_error", 5: "redundant"}.get(raw, f"unknown_{raw}")
    failed = [constraint_handles.get(int(h), f"handle:{int(h)}") for h in S.Failed]
    points = {}
    for e in problem.entities:
        if e.kind != EntityKind.POINT_3D:
            continue
        h = handles[e.id]
        points[e.id] = tuple(S.getParam(S.getEntityParam(h, i)).val / 1000.0 for i in range(3))
    return ConstraintSolveResult(status, raw, int(S.Dof), points, failed,
                                 {"entity_handles": len(handles), "constraint_handles": constraint_handles})


def reconstruct_placements(problem, result):
    if result.status != "okay" or result.dof != problem.expected_dof or result.failed_constraint_ids:
        raise SlvsSolveError(f"libslvs rejected: status={result.status} dof={result.dof} "
                             f"failed={result.failed_constraint_ids}")
    out = {}
    for sid, st in problem.stages.items():
        O = np.asarray(result.points_m[f"{sid}:anchor"])
        A = np.asarray(result.points_m[f"{sid}:axis_end"])
        solved_axis = _unit(A - O)
        seed = np.asarray(st.seed_transform, float)
        seed_axis = _unit(seed[:3, :3] @ np.asarray(st.local_axis))
        Rdelta = _rot_a_to_b(seed_axis, solved_axis)
        R = Rdelta @ seed[:3, :3]
        T = np.eye(4); T[:3, :3] = R
        T[:3, 3] = O - R @ np.asarray(st.local_anchor_m)
        out[sid] = T
    return out


def diagnose_authoritative_solution(problem, placements, result):
    pairs = []
    for gp in problem.gear_pairs:
        ps, cs = problem.stages[gp["parent_sub"]], problem.stages[gp["child_sub"]]
        pn, cn = gp["parent_part"], gp["child_part"]
        pa = (placements[gp["parent_sub"]] @ np.append(ps.gear_centers_local_m[pn], 1.0))[:3]
        pb = (placements[gp["child_sub"]] @ np.append(cs.gear_centers_local_m[cn], 1.0))[:3]
        d = pb - pa
        axis = _unit(placements[gp["parent_sub"]][:3, :3] @ np.asarray(ps.local_axis))
        axial = float(np.dot(d, axis))
        radial = float(np.linalg.norm(d - axial * axis))
        actual = float(np.linalg.norm(d))
        # Spur-gear center distance is measured perpendicular to the common shaft
        # axis. Axial offset changes face overlap, not pitch-center distance.
        pairs.append({**gp, "required_distance_mm": gp["target_m"] * 1000.0,
                      "actual_distance_mm": actual * 1000.0,
                      "signed_residual_mm": (radial - gp["target_m"]) * 1000.0,
                      "absolute_residual_mm": abs(radial - gp["target_m"]) * 1000.0,
                      "radial_distance_mm": radial * 1000.0, "axial_delta_mm": axial * 1000.0,
                      "parent_world_center_m": pa.tolist(), "child_world_center_m": pb.tolist(),
                      "parent_local_offset_m": list(ps.gear_centers_local_m[pn]),
                      "child_local_offset_m": list(cs.gear_centers_local_m[cn])})
    return pairs


def _suggested_mount_targets(problem):
    """Targets that preserve the closed-form seed's internally consistent rigid cluster,
    gauged onto the current input mount. These are candidate facts, never fallback poses."""
    if not problem.gear_pairs:
        return {}
    datum = problem.gear_pairs[0]["parent_sub"]
    ds = problem.stages[datum]
    Td = np.asarray(ds.seed_transform, float)
    seed_d = (Td @ np.append(ds.local_anchor_m, 1.0))[:3]
    seed_axis = _unit(Td[:3, :3] @ np.asarray(ds.local_axis))
    target_axis = _unit(ds.target_axis)
    R = _rot_a_to_b(seed_axis, target_axis)
    target_d = np.asarray(ds.target_anchor_m)
    out = {}
    for sid, st in problem.stages.items():
        T = np.asarray(st.seed_transform, float)
        p = (T @ np.append(st.local_anchor_m, 1.0))[:3]
        out[sid] = (target_d + R @ (p - seed_d)).tolist()
    return out


def failure_report_dict(problem, result, placements, error):
    pairs = diagnose_authoritative_solution(problem, placements, result) if placements else []
    suggested = _suggested_mount_targets(problem) if problem else {}
    stages = ({sid: {"local_anchor_m": list(st.local_anchor_m),
                     "target_anchor_m": list(st.target_anchor_m),
                     "target_axis": list(st.target_axis),
                     "suggested_target_anchor_m": suggested.get(sid),
                     "mount_seam_id": st.mount_seam_id,
                     "housing_frame": st.housing_frame,
                     "shaft_frame": st.shaft_frame,
                     "gear_centers_local_m": {k:list(v) for k,v in st.gear_centers_local_m.items()}}
               for sid,st in problem.stages.items()} if problem else {})
    core = {"backend": "slvs", "authority": "libslvs", "status": "failed",
            "error": str(error), "solver": {"status": result.status if result else "not_run",
            "raw_status": result.raw_status if result else -1, "dof": result.dof if result else -1,
            "expected_dof": problem.expected_dof if problem else -1,
            "failed_constraint_ids": result.failed_constraint_ids if result else [],
            "constraint_handles": ((result.diagnostics or {}).get("constraint_handles", {}) if result else {})},
            "base_id": problem.base_id if problem else "", "stages": stages,
            "gear_pairs": pairs}
    raw = json.dumps(core, sort_keys=True, separators=(",", ":")).encode()
    core["failure_id"] = "slvs_" + hashlib.sha256(raw).hexdigest()[:16]
    return core


def validate_authoritative_solution(problem, placements, result):
    errors = []
    if result.status != "okay": errors.append(f"status={result.status}")
    if result.dof != problem.expected_dof: errors.append(f"dof={result.dof}")
    if result.failed_constraint_ids: errors.append(f"failed={result.failed_constraint_ids}")
    for sid, T in placements.items():
        R = T[:3, :3]
        if (T.shape != (4, 4) or not np.all(np.isfinite(T)) or
                not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or
                abs(np.linalg.det(R)-1.0) > 1e-6): errors.append(f"bad_se3:{sid}")
    residuals = {}
    for gp in problem.gear_pairs:
        # Validate the RECONSTRUCTED rigid stages, not the point variables alone. This
        # catches any loss introduced when mapping solved anchor/axis back to root SE(3).
        ps = problem.stages[gp["parent_sub"]]
        cs = problem.stages[gp["child_sub"]]
        pn = gp["parent_entity"].split(":gear:", 1)[1]
        cn = gp["child_entity"].split(":gear:", 1)[1]
        pa = (placements[gp["parent_sub"]] @ np.append(ps.gear_centers_local_m[pn], 1.0))[:3]
        pb = (placements[gp["child_sub"]] @ np.append(cs.gear_centers_local_m[cn], 1.0))[:3]
        d = pb - pa
        axis = _unit(placements[gp["parent_sub"]][:3, :3] @ np.asarray(ps.local_axis))
        radial = float(np.linalg.norm(d - np.dot(d, axis) * axis))
        err = abs(radial - gp["target_m"])
        residuals[gp["seam_id"]] = err
        if err > _POS_TOL_M: errors.append(f"gear_distance:{gp['seam_id']}:{err*1000:.3f}mm")
    if errors: raise SlvsSolveError("; ".join(errors))
    return residuals


def solve_cross_sub_placements(plan, subs, seed, gear_ids, base_id, *, helpers, log_fn=print):
    ok, reason = slvs_available()
    if not ok: raise SlvsSolveError(f"py-slvs unavailable: {reason}")
    problem = result = placements = None
    try:
        problem = build_cross_sub_problem(plan, subs, seed, gear_ids, base_id, **helpers)
        result = solve_problem(problem)
        placements = reconstruct_placements(problem, result)
        residuals = validate_authoritative_solution(problem, placements, result)
    except SlvsSolveError as e:
        report = failure_report_dict(problem, result, placements, e)
        raise SlvsSolveError(str(e), problem=problem, result=result, placements=placements,
                             failure_report=report) from e
    log_fn(f"[slvs] authoritative solve OK: stages={len(placements)} constraints={len(problem.constraints)} dof={result.dof}")
    return PlacementResult(placements, result, residuals, "slvs"), problem


def report_dict(placement, problem):
    pairs = diagnose_authoritative_solution(problem, placement.placements, placement.solve)
    core = {"backend": placement.backend, "authority": "libslvs", "status": placement.solve.status,
            "raw_status": placement.solve.raw_status, "dof": placement.solve.dof,
            "failed_constraints": placement.solve.failed_constraint_ids,
            "entity_count": len(problem.entities), "constraint_count": len(problem.constraints),
            "base_id": problem.base_id, "gear_pairs": pairs,
            "gear_residuals_m": placement.residuals,
            "placements": {k: np.asarray(v).tolist() for k, v in placement.placements.items()}}
    core["failure_id"] = "slvs_" + hashlib.sha256(json.dumps(core, sort_keys=True,
        separators=(",", ":")).encode()).hexdigest()[:16]
    return core
