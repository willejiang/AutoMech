"""Authoritative libslvs cross-subassembly placement solver.

The legacy closed-form cluster is used only as an initial guess. Accepted world poses
come from a single libslvs constraint solve and backend-independent validation.
"""
from __future__ import annotations

import hashlib
import json
import math
import numpy as np
import trimesh.transformations as tf

from .constraint_ir import (AssemblyConstraintProblem, ConstraintKind, ConstraintSolveResult,
                            ConstraintSpec, EntityKind, EntitySpec, PlacementResult,
                            RigidStageSpec)
from .constraint_solver import SlvsSolveError, slvs_available, solve_problem

_GAUGE_M = 0.100
_POS_TOL_M = 1e-6
_MOUNT_LAYOUT_TOL_M = 0.002
_MIN_FACE_OVERLAP_M = 0.0001
_AXIS_TOL_DEG = 0.1


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
                    and abs(float(np.dot(pa, ca))) >= .99)
        except Exception:
            return False
    return next((s for s in plan.seams if typed_mount(s)), None)


def build_cross_sub_problem(plan, subs, seed, gear_ids, base_id, *,
                            frame_in_root, link_in_root, gear_link, gear_radius,
                            gear_face_width, gear_face_center_offset=lambda _link: 0.0):
    """Build the authoritative reducer problem. Housing targets are hard constraints."""
    if not gear_ids or base_id is None or base_id != plan.root_sub:
        raise SlvsSolveError("slvs MVP requires one root housing and a gear cluster")
    stages = {}
    entities = []
    constraints = []
    mount_records = []
    stage_carriers = {}
    def preflight_error(message, kind, pairs=None):
        diagnostics = {"phase": "preflight", "failure_kind": kind,
                       "mount_layout": {"tolerance_mm": _MOUNT_LAYOUT_TOL_M * 1000.0,
                                        "mounts": [r["diagnostic"] for r in mount_records],
                                        "pairs": pairs or []}}
        problem = AssemblyConstraintProblem(stages=stages, entities=entities,
                                            constraints=constraints, expected_dof=0,
                                            base_id=base_id, diagnostics=diagnostics)
        raise SlvsSolveError(message, problem=problem)

    seen_mount_frames = set()
    for sid in sorted(gear_ids):
        seam = _insert_seam(plan, base_id, sid)
        if seam is None:
            preflight_error(f"stage '{sid}' has no housing shaft-mount seam", "housing_mount_layout")
        if not seam.parent_frame or seam.parent_frame in seen_mount_frames:
            preflight_error(f"stage '{sid}' has missing or duplicate housing mount frame "
                            f"'{seam.parent_frame}'", "housing_mount_layout")
        seen_mount_frames.add(seam.parent_frame)
        target_fr = _frame_spec(plan, base_id, seam.parent_frame)
        shaft_fr = _frame_spec(plan, sid, seam.child_frame)
        if target_fr is None or shaft_fr is None:
            preflight_error(f"stage '{sid}' mount frame contract is incomplete", "housing_mount_layout")
        try:
            _shaft_link, T_local_anchor = frame_in_root(subs[sid], seam.child_frame)
            _base_link, T_target = frame_in_root(subs[base_id], seam.parent_frame)
        except Exception as e:
            preflight_error(f"stage '{sid}' mount frame unresolved: {e}", "housing_mount_layout")
        if not _se3(T_local_anchor) or not _se3(T_target):
            preflight_error(f"stage '{sid}' mount frame has an invalid transform", "housing_mount_layout")
        try:
            target_axis = _unit(target_fr.axis)
            contract_target = np.asarray(target_fr.xyz_m, float)
        except Exception as e:
            preflight_error(f"stage '{sid}' housing mount contract is invalid: {e}",
                            "housing_mount_layout")
        if contract_target.shape != (3,) or not np.all(np.isfinite(contract_target)):
            preflight_error(f"stage '{sid}' housing mount contract position is invalid",
                            "housing_mount_layout")
        # frames_realized.rpy is an authoring hint and is often rotated incorrectly.
        # Resolve the actual rotating shaft/carrier link (mount frames are frequently on a
        # fixed bearing whose spin_axis is zero), then transform its axis into sub-root.
        candidates=[l for l in subs[sid].model.links
                    if (getattr(l,'dof','')=='spin' or 'shaft' in l.name.lower())
                    and np.linalg.norm(np.asarray(getattr(l,'spin_axis',(0,0,0)),float))>1e-9]
        carrier=next((l for l in candidates if 'shaft' in l.name.lower()),candidates[0] if candidates else None)
        if carrier is None:
            preflight_error(f"stage '{sid}' has no built shaft axis", "housing_mount_layout")
        try:
            local_axis, _shaft_center = link_in_root(subs[sid], carrier.name)
            local_axis = _unit(local_axis)
        except Exception as e:
            preflight_error(f"stage '{sid}' built shaft axis is invalid: {e}",
                            "housing_mount_layout")
        seed_T = np.asarray(seed.get(sid), float)
        if not _se3(seed_T):
            preflight_error(f"stage '{sid}' has an invalid seed transform", "housing_mount_layout")
        stage = RigidStageSpec(sid, seed_T.tolist(),
                               tuple(T_local_anchor[:3, 3]), tuple(local_axis),
                               tuple(T_target[:3, 3]), tuple(target_axis), {},
                               seam.id, seam.parent_frame, seam.child_frame)
        rear=None
        if getattr(seam,'rear_parent_frame','') and getattr(seam,'rear_child_frame',''):
            try:
                _rl,T_local_rear=frame_in_root(subs[sid],seam.rear_child_frame)
                _hl,T_target_rear=frame_in_root(subs[base_id],seam.rear_parent_frame)
                rear_fr=_frame_spec(plan,base_id,seam.rear_parent_frame)
                rear_axis=_unit(rear_fr.axis)
            except Exception as e:
                preflight_error(f"stage '{sid}' rear mount frame unresolved: {e}","rear_mount_realization")
            local_delta=T_local_rear[:3,3]-T_local_anchor[:3,3]
            radial=local_delta-float(np.dot(local_delta,local_axis))*local_axis
            if float(np.linalg.norm(radial))>_MOUNT_LAYOUT_TOL_M:
                preflight_error(f"stage '{sid}' rear datum is off the shaft axis","rear_mount_realization")
            rear={'seam_id':seam.id,'sub_id':sid,'housing_frame':seam.rear_parent_frame,
                  'shaft_frame':seam.rear_child_frame,'local_point_m':T_local_rear[:3,3].tolist(),
                  'target_point_m':T_target_rear[:3,3].tolist(),'target_axis':rear_axis.tolist(),
                  'constraint_id':f'{seam.id}:rear_plane','tolerance_mm':_MOUNT_LAYOUT_TOL_M*1000.0}
        stages[sid] = stage
        stage_carriers[sid] = carrier.name
        mount_records.append({
            "sid": sid, "seam": seam, "stage": stage,
            "T_local_anchor": T_local_anchor, "T_target": T_target,
            "target_axis": target_axis, "local_axis": local_axis,"rear":rear,
            "diagnostic": {"stage": sid, "seam": seam.id,
                           "housing_frame": seam.parent_frame,
                           "shaft_frame": seam.child_frame,
                           "contract_position_m": contract_target.tolist(),
                           "realized_position_m": T_target[:3, 3].tolist(),
                           "contract_axis": target_axis.tolist()}})

    mount_pairs = []
    for i, a in enumerate(mount_records):
        for b in mount_records[i + 1:]:
            ca = np.asarray(a["diagnostic"]["contract_position_m"])
            cb = np.asarray(b["diagnostic"]["contract_position_m"])
            ra = np.asarray(a["diagnostic"]["realized_position_m"])
            rb = np.asarray(b["diagnostic"]["realized_position_m"])
            contract_sep = float(np.linalg.norm(cb - ca))
            realized_sep = float(np.linalg.norm(rb - ra))
            layout_error = abs(realized_sep - contract_sep)
            layout_limit = max(_MOUNT_LAYOUT_TOL_M, 0.15 * contract_sep)
            collapsed = (contract_sep > _MOUNT_LAYOUT_TOL_M and
                         realized_sep <= _MOUNT_LAYOUT_TOL_M)
            layout_mismatch = contract_sep > _MOUNT_LAYOUT_TOL_M and layout_error > layout_limit
            pair = {"stages": [a["sid"], b["sid"]],
                    "housing_frames": [a["seam"].parent_frame, b["seam"].parent_frame],
                    "contract_separation_mm": contract_sep * 1000.0,
                    "realized_separation_mm": realized_sep * 1000.0,
                    "layout_error_mm": layout_error * 1000.0,
                    "layout_limit_mm": layout_limit * 1000.0,
                    "collapsed": collapsed,
                    "layout_mismatch": layout_mismatch}
            mount_pairs.append(pair)
            if layout_mismatch:
                preflight_error("housing mount frames "
                                f"'{a['seam'].parent_frame}' and '{b['seam'].parent_frame}' "
                                f"realize {realized_sep*1000:.3f} mm apart despite a "
                                f"{contract_sep*1000:.3f} mm contract separation",
                                "housing_mount_layout", mount_pairs)

    for record in mount_records:
        sid, seam, stage = record["sid"], record["seam"], record["stage"]
        T_local_anchor, T_target = record["T_local_anchor"], record["T_target"]
        target_axis, local_axis = record["target_axis"], record["local_axis"]
        O, A = f"{sid}:anchor", f"{sid}:axis_end"
        seed_T = np.asarray(stage.seed_transform)
        seed_O = (seed_T @ np.append(T_local_anchor[:3, 3], 1.0))[:3]
        seed_A = seed_O + _GAUGE_M * (seed_T[:3, :3] @ local_axis)
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
        rear=record.get('rear')
        if rear:
            rid=f"{sid}:rear_datum";local_rear=np.asarray(rear['local_point_m'])
            axial=float(np.dot(local_rear-np.asarray(stage.local_anchor_m),local_axis))
            constraints.extend([
              ConstraintSpec(f"{seam.id}:rear_on_axis",ConstraintKind.POINT_ON_LINE,
                             (O,f"{sid}:axis"),enforced_by_solver=False,provenance={'seam':seam.id,'rear_datum':rid}),
              ConstraintSpec(rear['constraint_id'],ConstraintKind.PROJECTED_DISTANCE,
                             (O,A,f"{sid}:axis"),-axial,enforced_by_solver=False,
                             provenance={'seam':seam.id,'rear_parent_frame':rear['housing_frame'],
                                         'rear_child_frame':rear['shaft_frame']})])

    problem_diagnostics = {"phase": "ready", "mount_layout": {
        "tolerance_mm": _MOUNT_LAYOUT_TOL_M * 1000.0,
        "mounts": [r["diagnostic"] for r in mount_records], "pairs": mount_pairs},
        "rear_mounts":[r['rear'] for r in mount_records if r.get('rear')]}
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
        wp, wc = gear_face_width(gp), gear_face_width(gc)
        op, oc = gear_face_center_offset(gp), gear_face_center_offset(gc)
        if not wp or not wc or op is None or oc is None:
            problem = AssemblyConstraintProblem(
                stages=stages, entities=entities, constraints=constraints,
                gear_pairs=gear_pairs, expected_dof=0, base_id=base_id,
                diagnostics={**problem_diagnostics, "phase": "preflight",
                             "failure_kind": "gear_face_geometry",
                             "gear_face_geometry": {"seam": seam.id,
                                "parent_part": gp.name, "child_part": gc.name,
                                "parent_face_width_mm": wp,
                                "child_face_width_mm": wc,
                                "parent_face_center_offset_mm": op,
                                "child_face_center_offset_mm": oc}})
            raise SlvsSolveError(f"power seam '{seam.id}' missing unambiguous gear face geometry",
                                 problem=problem)
        paxis, pcenter = link_in_root(pa, gp.name)
        caxis, ccenter = link_in_root(ch, gc.name)
        gear_axis_signs = {}
        for sid, stage, axis, center, gear in ((seam.parent_sub, stages[seam.parent_sub], paxis, pcenter, gp),
                                                (seam.child_sub, stages[seam.child_sub], caxis, ccenter, gc)):
            la = np.asarray(stage.local_axis)
            radial = np.asarray(center) - np.asarray(stage.local_anchor_m)
            axial = float(np.dot(radial, la))
            off = radial - axial * la
            if np.linalg.norm(off) > 1e-4:
                problem = AssemblyConstraintProblem(
                    stages=stages, entities=entities, constraints=constraints,
                    gear_pairs=gear_pairs, expected_dof=0, base_id=base_id,
                    diagnostics={**problem_diagnostics, "phase": "preflight",
                      "failure_kind": "gear_off_shaft_axis",
                      "gear_off_shaft_axis": {"seam": seam.id, "sub_id": sid,
                        "part": gear.name, "carrier": stage_carriers.get(sid, ""),
                        "gear_center_local_m": np.asarray(center).tolist(),
                        "shaft_anchor_local_m": list(stage.local_anchor_m),
                        "stage_axis_local": la.tolist(),
                        "radial_offset_mm": (off * 1000.0).tolist(),
                        "radial_distance_mm": float(np.linalg.norm(off) * 1000.0)}})
                raise SlvsSolveError(
                    f"gear '{sid}.{gear.name}' center is not on its stage shaft axis",
                    problem=problem)
            axis_dot = float(np.dot(_unit(axis), la))
            if abs(abs(axis_dot) - 1.0) > 1e-4:
                raise SlvsSolveError(f"gear '{sid}.{gear.name}' axis disagrees with its stage shaft axis")
            gear_axis_signs[sid] = 1.0 if axis_dot >= 0 else -1.0
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
                           "parent_face_width_mm": float(wp),
                           "child_face_width_mm": float(wc),
                           "parent_face_center_offset_mm": float(op),
                           "child_face_center_offset_mm": float(oc),
                           "parent_gear_axis_sign": gear_axis_signs[seam.parent_sub],
                           "child_gear_axis_sign": gear_axis_signs[seam.child_sub],
                           "target_m": target})
    return AssemblyConstraintProblem(stages=stages, entities=entities, constraints=constraints,
                                     gear_pairs=gear_pairs, expected_dof=0, base_id=base_id,
                                     diagnostics=problem_diagnostics)


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


def _gear_pair_geometry(problem, placements, gp):
    ps, cs = problem.stages[gp["parent_sub"]], problem.stages[gp["child_sub"]]
    pn, cn = gp["parent_part"], gp["child_part"]
    Tp, Tc = placements[gp["parent_sub"]], placements[gp["child_sub"]]
    pa = (Tp @ np.append(ps.gear_centers_local_m[pn], 1.0))[:3]
    pb = (Tc @ np.append(cs.gear_centers_local_m[cn], 1.0))[:3]
    paxis = _unit(Tp[:3, :3] @ np.asarray(ps.local_axis))
    caxis = _unit(Tc[:3, :3] @ np.asarray(cs.local_axis))
    axis_dot = float(np.dot(paxis, caxis))
    axis_angle = math.degrees(math.acos(np.clip(abs(axis_dot), -1.0, 1.0)))
    pface = pa + paxis * gp.get("parent_gear_axis_sign", 1.0) * \
        gp.get("parent_face_center_offset_mm", 0.0) / 1000.0
    cface = pb + caxis * gp.get("child_gear_axis_sign", 1.0) * \
        gp.get("child_face_center_offset_mm", 0.0) / 1000.0
    origin_delta = pb - pa
    d = cface - pface
    axial = float(np.dot(d, paxis))
    radial = float(np.linalg.norm(origin_delta - np.dot(origin_delta, paxis) * paxis))
    wp = gp["parent_face_width_mm"] / 1000.0
    wc = gp["child_face_width_mm"] / 1000.0
    face_overlap = max(0.0, min(wp / 2.0, axial + wc / 2.0) -
                       max(-wp / 2.0, axial - wc / 2.0))
    return ps, cs, pn, cn, pa, pb, radial, axial, face_overlap, axis_angle, axis_dot


def diagnose_authoritative_solution(problem, placements, result):
    pairs = []
    for gp in problem.gear_pairs:
        ps, cs, pn, cn, pa, pb, radial, axial, overlap, axis_angle, axis_dot = \
            _gear_pair_geometry(problem, placements, gp)
        actual = float(np.linalg.norm(pb - pa))
        # Spur-gear center distance is measured perpendicular to the common shaft
        # axis. Axial offset changes face overlap, not pitch-center distance.
        pairs.append({**gp, "required_distance_mm": gp["target_m"] * 1000.0,
                      "actual_distance_mm": actual * 1000.0,
                      "signed_residual_mm": (radial - gp["target_m"]) * 1000.0,
                      "absolute_residual_mm": abs(radial - gp["target_m"]) * 1000.0,
                      "radial_distance_mm": radial * 1000.0, "axial_delta_mm": axial * 1000.0,
                      "face_overlap_mm": overlap * 1000.0,
                      "minimum_face_overlap_mm": _MIN_FACE_OVERLAP_M * 1000.0,
                      "axis_angle_error_deg": axis_angle,
                      "axis_direction_dot": axis_dot,
                      "parent_world_center_m": pa.tolist(), "child_world_center_m": pb.tolist(),
                      "parent_world_face_center_m": (pa +
                          _unit(placements[gp["parent_sub"]][:3, :3] @ np.asarray(ps.local_axis)) *
                          gp.get("parent_gear_axis_sign", 1.0) *
                          gp.get("parent_face_center_offset_mm", 0.0) / 1000.0).tolist(),
                      "child_world_face_center_m": (pb +
                          _unit(placements[gp["child_sub"]][:3, :3] @ np.asarray(cs.local_axis)) *
                          gp.get("child_gear_axis_sign", 1.0) *
                          gp.get("child_face_center_offset_mm", 0.0) / 1000.0).tolist(),
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
                     "local_axis": list(st.local_axis),
                     "target_anchor_m": list(st.target_anchor_m),
                     "target_axis": list(st.target_axis),
                     "suggested_target_anchor_m": suggested.get(sid),
                     "mount_seam_id": st.mount_seam_id,
                     "housing_frame": st.housing_frame,
                     "shaft_frame": st.shaft_frame,
                     "gear_centers_local_m": {k:list(v) for k,v in st.gear_centers_local_m.items()}}
               for sid,st in problem.stages.items()} if problem else {})
    diagnostics = dict(problem.diagnostics or {}) if problem else {}
    core = {"backend": "slvs", "authority": "libslvs", "status": "failed",
            "error": str(error), "solver": {"status": result.status if result else "not_run",
            "raw_status": result.raw_status if result else -1, "dof": result.dof if result else -1,
            "expected_dof": problem.expected_dof if problem else -1,
            "failed_constraint_ids": result.failed_constraint_ids if result else [],
            "constraint_handles": ((result.diagnostics or {}).get("constraint_handles", {}) if result else {})},
            "base_id": problem.base_id if problem else "", "stages": stages,
            "gear_pairs": pairs, "diagnostics": diagnostics}
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
    rear_failures=[]
    for rear in problem.diagnostics.get('rear_mounts',[]):
        T=placements.get(rear['sub_id'])
        if T is None:continue
        solved=(np.asarray(T)@np.append(rear['local_point_m'],1.0))[:3]
        target=np.asarray(rear['target_point_m']);axis=_unit(rear['target_axis'])
        signed=float(np.dot(solved-target,axis));absolute=abs(signed)
        rear.update({'solved_point_m':solved.tolist(),'signed_residual_mm':signed*1000.0,
                     'absolute_residual_mm':absolute*1000.0,'passed':absolute<=_MOUNT_LAYOUT_TOL_M,
                     'enforced_by_solver':False})
        residuals[rear['constraint_id']]=absolute
        if absolute>_MOUNT_LAYOUT_TOL_M:
            errors.append(f"rear_mount_plane:{rear['seam_id']}:{absolute*1000:.3f}mm")
            rear_failures.append(rear['seam_id'])
    for gp in problem.gear_pairs:
        # Validate the RECONSTRUCTED rigid stages, not the point variables alone. This
        # catches any loss introduced when mapping solved anchor/axis back to root SE(3).
        _ps, _cs, _pn, _cn, _pa, _pb, radial, _axial, overlap, axis_angle, _axis_dot = \
            _gear_pair_geometry(problem, placements, gp)
        err = abs(radial - gp["target_m"])
        residuals[gp["seam_id"]] = err
        if err > _POS_TOL_M:
            errors.append(f"gear_distance:{gp['seam_id']}:{err*1000:.3f}mm")
        if axis_angle > _AXIS_TOL_DEG:
            errors.append(f"gear_axis_alignment:{gp['seam_id']}:{axis_angle:.3f}deg")
        if overlap <= _MIN_FACE_OVERLAP_M:
            errors.append(f"gear_face_overlap:{gp['seam_id']}:{overlap*1000:.3f}mm")
    if errors:
        problem.diagnostics.update({"phase": "post_solve", "failure_kind":
            ("rear_mount_plane" if rear_failures
             else "gear_face_overlap" if any(x.startswith("gear_face_overlap:") for x in errors)
             else "gear_axis_alignment" if any(x.startswith("gear_axis_alignment:") for x in errors)
             else "gear_distance")})
        raise SlvsSolveError("; ".join(errors))
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
        problem = e.problem or problem
        result = e.result or result
        placements = e.placements or placements
        report = e.failure_report or failure_report_dict(problem, result, placements, e)
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
            "diagnostics": problem.diagnostics,
            "gear_residuals_m": placement.residuals,
            "placements": {k: np.asarray(v).tolist() for k, v in placement.placements.items()}}
    core["failure_id"] = "slvs_" + hashlib.sha256(json.dumps(core, sort_keys=True,
        separators=(",", ":")).encode()).hexdigest()[:16]
    return core
