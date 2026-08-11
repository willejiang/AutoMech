#!/usr/bin/env python3
"""Project Chrono 10 sidecar runner over maker2's backend-neutral bundle."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def _v(c, xyz):
    return c.ChVector3d(float(xyz[0]), float(xyz[1]), float(xyz[2]))


def _vec(v):
    return [float(v.x), float(v.y), float(v.z)]


def _quat(c, q):
    return c.ChQuaterniond(float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _frame(c, rec):
    return c.ChFramed(_v(c, rec["xyz_m"]), _quat(c, rec.get("quat_wxyz", [1, 0, 0, 0])))


def _axis_quat(c, axis):
    """Rotation whose local Z axis points along ``axis``."""
    z = _v(c, axis)
    n = math.sqrt(z.x*z.x + z.y*z.y + z.z*z.z)
    if n <= 1e-12:
        raise ValueError("zero joint axis")
    z = c.ChVector3d(z.x/n, z.y/n, z.z/n)
    base = c.ChVector3d(0, 0, 1)
    dot = max(-1.0, min(1.0, z.z))
    if dot > 1.0-1e-12:
        return c.QUNIT
    if dot < -1.0+1e-12:
        return c.QuatFromAngleX(math.pi)
    cross = c.ChVector3d(-z.y, z.x, 0)
    cn = math.sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z)
    cross = c.ChVector3d(cross.x/cn, cross.y/cn, cross.z/cn)
    return c.QuatFromAngleAxis(math.acos(dot), cross)


def _joint_frame(c, rec):
    jf = rec.get("world_frame") or {}
    return c.ChFramed(_v(c, jf.get("xyz_m", [0, 0, 0])),
                      _axis_quat(c, jf.get("axis_world", [0, 0, 1])))


def _density(material):
    return {"steel": 7850.0, "brass": 8500.0, "ruby": 4000.0,
            "plastic": 1200.0, "aluminum": 2700.0, "titanium": 4500.0,
            "rubber": 1100.0, "wood": 700.0, "gold": 19300.0}.get(
                (material or "steel").lower(), 7850.0)


def _material(c, name):
    friction = {"ruby": .3, "plastic": .6, "rubber": 1.5,
                "wood": .6, "aluminum": .8, "brass": .9}.get(
                    (name or "steel").lower(), 1.0)
    mat = c.ChContactMaterialNSC()
    mat.SetFriction(friction)
    mat.SetRestitution(0.0)
    return mat


def _body_mass_props(trimesh, mesh_path, density):
    mesh = trimesh.load_mesh(mesh_path, force="mesh", process=False)
    volume_m3 = abs(float(mesh.volume))*1e-9
    mass = max(volume_m3*density, 1e-6)
    center_m = [float(x)*0.001 for x in mesh.center_mass]
    inertia = mesh.moment_inertia * density * 1e-15
    diag = [max(float(inertia[i, i]), 1e-12) for i in range(3)]
    off = [float(inertia[0, 1]), float(inertia[0, 2]), float(inertia[1, 2])]
    return mass, center_m, diag, off


def _add_collision(c, system, body, rec, mesh_path):
    policy = rec.get("collision") or {}
    representation = policy.get("representation", "none")
    if representation == "none":
        return "none"
    if representation == "disabled_dynamic_triangle_mesh":
        return representation
    mat = _material(c, rec.get("material"))
    if representation == "cylinder":
        shape = c.ChCollisionShapeCylinder(mat, float(policy["radius_m"]),
                                           float(policy["length_m"]))
    else:
        mesh = c.ChTriangleMeshConnected.CreateFromSTLFile(str(mesh_path), False)
        mesh.Transform(c.ChVector3d(0, 0, 0), c.ChMatrix33d(float(policy.get("mesh_scale", .001))))
        shape = c.ChCollisionShapeTriangleMesh(
            mat, mesh, body.IsFixed(), False, float(policy.get("swept_radius_m", 0.0)))
    body.AddCollisionShape(shape)
    body.EnableCollision(True)
    return representation


class _Contacts:
    def __init__(self, c):
        class Reporter(c.ReportContactCallback):
            def __init__(self):
                super().__init__()
                self.rows = []

            def OnReportContact(self, pa, pb, plane, distance, radius, force, torque,
                                ca, cb, offset):
                ba, bb = c.CastToChBody(ca), c.CastToChBody(cb)
                fw, tw = plane*force, plane*torque
                self.rows.append({"body_a": ba.GetName(), "body_b": bb.GetName(),
                                  "point_a_m": _vec(pa), "point_b_m": _vec(pb),
                                  "distance_m": float(distance),
                                  "force_world_n": _vec(fw),
                                  "torque_world_nm": _vec(tw)})
                return True
        self.reporter = Reporter()

    def collect(self, system):
        self.reporter.rows = []
        system.GetContactContainer().ReportAllContacts(self.reporter)
        return list(self.reporter.rows)


def _constraint(c, rec, bodies, ground, driver_name, torque):
    parent = bodies.get(rec.get("parent")) or ground
    child = bodies.get(rec.get("child"))
    if child is None:
        raise ValueError(f"missing motion child {rec.get('child')}")
    frame = _joint_frame(c, rec)
    kind = rec.get("type")
    if kind == "fixed":
        link = c.ChLinkMateFix()
        link.Initialize(parent, child)
        return link
    elif kind == "hinge" and rec.get("child") == driver_name:
        link = c.ChLinkMotorRotationTorque()
    elif kind == "hinge":
        link = c.ChLinkLockRevolute()
    elif kind == "slide":
        link = c.ChLinkLockPrismatic()
    else:
        raise ValueError(f"unsupported motion joint {kind}")
    link.Initialize(parent, child, frame)
    return link


def _relation(c, rec, bodies):
    a, b = bodies.get(rec.get("body_a")), bodies.get(rec.get("body_b"))
    if a is None or b is None:
        raise ValueError("missing relation body")
    fa, fb = rec.get("frame_a"), rec.get("frame_b")
    if not fa or not fb:
        raise ValueError("missing relation frame")
    frame_a, frame_b = _frame(c, fa), _frame(c, fb)
    kind = rec.get("type")
    if kind in {"pin", "revolute", "journal_bearing"}:
        link = c.ChLinkLockRevolute()
    elif kind in {"cylindrical", "coaxial"}:
        link = c.ChLinkMateGeneric()
        link.SetConstrainedCoords(True, True, False, True, True, False)
    elif kind in {"press_fit", "fixed", "weld", "welded", "bolted"}:
        link = c.ChLinkMateFix()
    else:
        raise ValueError(f"unsupported relation {kind}")
    link.Initialize(a, b, False, frame_a, frame_b)
    return link


def _gear(c, rec, bodies):
    a, b = bodies.get(rec.get("driving_link")), bodies.get(rec.get("driven_link"))
    if a is None or b is None:
        raise ValueError("missing transmission body")
    ratio = float(rec.get("ratio") or 1.0)
    if rec.get("type") == "compound_1to1":
        link = c.ChLinkMateFix()
        link.Initialize(a, b, c.ChFramed())
        return link
    link = c.ChLinkLockGear()
    link.Initialize(a, b, c.ChFramed())
    link.SetFrameShaft1(c.ChFramed(c.VNULL, c.QUNIT))
    link.SetFrameShaft2(c.ChFramed(c.VNULL, c.QUNIT))
    link.SetTransmissionRatio(abs(ratio))
    link.SetEpicyclic(rec.get("type") == "gear_internal")
    link.SetEnforcePhase(True)
    return link


def _rotation_delta(q0, q1):
    dot = abs(q0[0]*q1.e0 + q0[1]*q1.e1 + q0[2]*q1.e2 + q0[3]*q1.e3)
    return 2.0*math.acos(max(-1.0, min(1.0, dot)))


def _constraint_violation(link):
    try:
        values = link.GetConstraintViolation()
        return [float(values[i]) for i in range(values.Size())]
    except Exception:
        return []


def _relative_speed(body, parent, axis_world):
    child_w = body.GetAngVelParent()
    parent_w = parent.GetAngVelParent() if parent is not None else None
    wx = child_w.x - (parent_w.x if parent_w is not None else 0.0)
    wy = child_w.y - (parent_w.y if parent_w is not None else 0.0)
    wz = child_w.z - (parent_w.z if parent_w is not None else 0.0)
    return wx*axis_world[0] + wy*axis_world[1] + wz*axis_world[2]


def _render_trajectory_frames(bundle, trajectory, out):
    """Render a dependency-light kinematic preview from actual Chrono body states.

    This is deliberately a trajectory/orientation view, not a claim of photorealistic mesh
    rendering. It gives the frontend a recording while preserving the sidecar's headless use.
    """
    from PIL import Image, ImageDraw

    times = trajectory.get("t") or []
    states = trajectory.get("bodies") or {}
    if not times or not states:
        return None
    points = [sample["xyz_m"] for samples in states.values() for sample in samples]
    if not points:
        return None
    spans = [max(p[i] for p in points)-min(p[i] for p in points) for i in range(3)]
    axes = sorted(range(3), key=lambda i: spans[i], reverse=True)[:2]
    mins = [min(p[i] for p in points) for i in axes]
    maxs = [max(p[i] for p in points) for i in axes]
    for i in range(2):
        if maxs[i]-mins[i] < 1e-4:
            mins[i] -= .05
            maxs[i] += .05
    width, height, margin = 720, 540, 40
    scale = min((width-2*margin)/(maxs[0]-mins[0]),
                (height-2*margin)/(maxs[1]-mins[1]))
    body_info = {x["source_name"]: x for x in bundle.get("bodies") or []}
    driver = bundle.get("driver")
    output = bundle.get("output_link")
    frames = out/"frames"
    frames.mkdir(parents=True, exist_ok=True)
    for stale in frames.glob("rgb_*.png"):
        stale.unlink()

    def screen(pos):
        return (margin+(pos[axes[0]]-mins[0])*scale,
                height-margin-(pos[axes[1]]-mins[1])*scale)

    count = min(len(times), min((len(x) for x in states.values() if x), default=0))
    for index in range(count):
        image = Image.new("RGB", (width, height), (17, 20, 26))
        draw = ImageDraw.Draw(image)
        draw.text((16, 12), f"Project Chrono trajectory  t={times[index]:.3f}s",
                  fill=(225, 230, 238))
        draw.text((16, height-24),
                  f"projection axes: {'xyz'[axes[0]]}/{'xyz'[axes[1]]}  (schematic)",
                  fill=(130, 140, 155))
        for name, samples in states.items():
            if index >= len(samples):
                continue
            sample = samples[index]
            x, y = screen(sample["xyz_m"])
            info = body_info.get(name) or {}
            color = ((255, 183, 77) if name == driver else
                     (239, 83, 80) if name == output else
                     (68, 181, 214) if info.get("dof") != "fixed" else
                     (105, 115, 130))
            radius = 5 if info.get("dof") != "fixed" else 3
            draw.ellipse((x-radius, y-radius, x+radius, y+radius), fill=color)
            q = sample.get("quat_wxyz") or [1, 0, 0, 0]
            w, qx, qy, qz = q
            direction = [1-2*(qy*qy+qz*qz), 2*(qx*qy+w*qz), 2*(qx*qz-w*qy)]
            dx = direction[axes[0]]*14
            dy = -direction[axes[1]]*14
            draw.line((x, y, x+dx, y+dy), fill=color, width=2)
            if name in {driver, output}:
                draw.text((x+7, y-9), name, fill=color)
        image.save(frames/f"rgb_{index:04d}.png")
    return str(frames)


def run(bundle_path: str, spec_path: str, out_dir: str) -> dict:
    import numpy as np
    import pychrono as c
    import trimesh

    bundle = json.loads(Path(bundle_path).read_text(encoding="utf-8"))
    spec = json.loads(Path(spec_path).read_text(encoding="utf-8"))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    system = c.ChSystemNSC()
    system.SetCollisionSystemType(c.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(c.ChVector3d(0, 0, -9.81))
    ground = c.ChBody()
    ground.SetName("__world__")
    ground.SetFixed(True)
    system.AddBody(ground)

    bodies, collision_repr, inertia_max = {}, {}, {}
    for rec in bundle["bodies"]:
        if not rec.get("compiled"):
            continue
        body = c.ChBodyAuxRef()
        body.SetName(rec["source_name"])
        body.SetFrameRefToAbs(_frame(c, rec["world_pose"]))
        mesh_path = Path(bundle["run_dir"])/(rec.get("mesh") or f"meshes/{rec['source_name']}.stl")
        mass, center, diag, off = .05, [0, 0, 0], [1e-4]*3, [0, 0, 0]
        if mesh_path.exists():
            try:
                mass, center, diag, off = _body_mass_props(trimesh, mesh_path, _density(rec.get("material")))
            except Exception:
                pass
        body.SetMass(mass)
        body.SetFrameCOMToRef(c.ChFramed(_v(c, center), c.QUNIT))
        body.SetInertiaXX(_v(c, diag))
        body.SetInertiaXY(_v(c, off))
        inertia_max[rec["source_name"]] = max(diag)
        # A fixed DOF means no authored coordinate of its own, not necessarily fixed to
        # world: press-fit accessories and carrier pins must inherit through constraints.
        has_constraint = any(r.get("compiled") and
            (r.get("child") == rec["source_name"] or r.get("body_a") == rec["source_name"]
             or r.get("body_b") == rec["source_name"])
            for r in bundle["constraints"])
        if rec.get("dof") == "fixed" and not has_constraint:
            body.SetFixed(True)
        system.AddBody(body)
        bodies[rec["source_name"]] = body
        if mesh_path.exists():
            collision_repr[rec["source_name"]] = _add_collision(c, system, body, rec, mesh_path)

    # Chrono exclusions are family-mask based. Give each involved body a family and
    # disable only the declared rigid pair; unrelated shaft/web contacts remain active.
    excluded_pairs = [(x.get("body_a"), x.get("body_b"))
                      for x in bundle.get("contact_excludes") or []]
    family_by_body = {}
    collision_models = {name: body.GetCollisionModel() for name, body in bodies.items()
                        if body.GetCollisionModel() is not None}
    next_family = 1
    for a, b in excluded_pairs:
        for name in (a, b):
            if name in collision_models and name not in family_by_body and next_family < 16:
                family_by_body[name] = next_family
                collision_models[name].SetFamily(next_family)
                next_family += 1
    for a, b in excluded_pairs:
        if a in family_by_body and b in family_by_body:
            collision_models[a].DisallowCollisionsWith(family_by_body[b])
            collision_models[b].DisallowCollisionsWith(family_by_body[a])

    drive = spec.get("drive") or {}
    if drive.get("torque") is not None:
        torque = float(drive["torque"])
    else:
        moving_inertia = sum(inertia_max.get(rec["source_name"], 0.0)
                             for rec in bundle["bodies"] if rec.get("dof") != "fixed")
        torque = max(1e-8, moving_inertia*float(drive.get("target_accel_rad_s2") or 2.0))
    links, link_records, compile_errors = {}, {}, []
    torque_pending = []
    for rec in bundle["constraints"]:
        if not rec.get("compiled"):
            continue
        try:
            link = (_constraint(c, rec, bodies, ground,
                                bundle.get("driver_coordinate") or bundle.get("driver"), torque)
                    if rec["source_kind"] == "motion_joint" else
                    _relation(c, rec, bodies))
            link.SetName(rec["source_name"])
            system.AddLink(link)
            links[rec["source_name"]] = link
            link_records[rec["source_name"]] = rec
            if isinstance(link, c.ChLinkMotorRotationTorque):
                torque_pending.append(link)
        except Exception as exc:
            compile_errors.append({"name": rec.get("source_name"), "error": str(exc)})
    motor_links = [name for name, link in links.items()
                   if isinstance(link, c.ChLinkMotorRotationTorque)]
    fixed_pairs = {frozenset((r.get("body_a"), r.get("body_b")))
                   for r in bundle["constraints"]
                   if r.get("compiled") and r.get("source_kind") == "relation"
                   and r.get("type") in {"press_fit", "fixed", "weld", "welded", "bolted"}}
    for rec in bundle["transmissions"]:
        if not rec.get("compiled"):
            continue
        if rec.get("type") == "compound_1to1" and frozenset(
                (rec.get("driving_link"), rec.get("driven_link"))) in fixed_pairs:
            continue
        try:
            link = _gear(c, rec, bodies)
            link.SetName(rec["source_name"])
            system.AddLink(link)
            links[rec["source_name"]] = link
            link_records[rec["source_name"]] = rec
        except Exception as exc:
            compile_errors.append({"name": rec.get("source_name"), "error": str(exc)})

    # Complete epicyclic constraint graph. Chrono's body gear link accounts for the moving
    # planet centers, so external sun/planet plus internal ring/planet meshes naturally
    # produce the Willis carrier relation and each planet's carrier-relative self-spin.
    planetary_links = []
    for rec in bundle["planetary_stages"]:
        if not rec.get("compiled") or bundle.get("mode") != "ideal_dynamic":
            continue
        try:
            fixed_name = rec.get({"sun": "sun", "ring": "ring", "carrier": "carrier"}.get(
                rec.get("fixed_member"), rec.get("fixed_member")))
            if fixed_name in bodies:
                bodies[fixed_name].SetFixed(True)
            sun, ring = bodies[rec["sun"]], bodies[rec["ring"]]
            for index, planet in enumerate(rec.get("planets") or []):
                gear = bodies[planet["gear"]]
                outer = c.ChLinkLockGear()
                outer.Initialize(sun, gear, c.ChFramed())
                outer.SetFrameShaft1(c.ChFramed(c.VNULL, c.QUNIT))
                outer.SetFrameShaft2(c.ChFramed(c.VNULL, c.QUNIT))
                outer.SetTransmissionRatio(float(rec["sun_teeth"])/float(rec["planet_teeth"]))
                outer.SetEnforcePhase(False)
                outer.SetName(f"{rec['source_name']}_sun_planet_{index+1}")
                system.AddLink(outer); planetary_links.append(outer)
                inner = c.ChLinkLockGear()
                inner.Initialize(gear, ring, c.ChFramed())
                inner.SetFrameShaft1(c.ChFramed(c.VNULL, c.QUNIT))
                inner.SetFrameShaft2(c.ChFramed(c.VNULL, c.QUNIT))
                inner.SetTransmissionRatio(float(rec["planet_teeth"])/float(rec["ring_teeth"]))
                inner.SetEpicyclic(True); inner.SetEnforcePhase(False)
                inner.SetName(f"{rec['source_name']}_ring_planet_{index+1}")
                system.AddLink(inner); planetary_links.append(inner)
        except Exception as exc:
            compile_errors.append({"name": rec.get("source_name"), "error": str(exc)})

    dt = float(spec.get("timestep_s") or .001)
    settle = float(spec.get("settle_s") or .2)
    for _ in range(max(0, int(settle/dt))):
        system.DoStepDynamics(dt)
    initial = {name: (_vec(body.GetPos()),
                      (float(body.GetRot().e0), float(body.GetRot().e1),
                       float(body.GetRot().e2), float(body.GetRot().e3)))
               for name, body in bodies.items()}
    for motor in torque_pending:
        motor.SetTorqueFunction(c.ChFunctionConst(torque))
    duration = float(spec.get("duration_s") or 1.0)
    contact_pairs_expected = set()
    for stage in bundle.get("planetary_stages") or []:
        if not stage.get("compiled") or bundle.get("mode") != "contact_dynamic":
            continue
        for planet in stage.get("planets") or []:
            contact_pairs_expected.add(frozenset((stage["sun"], planet["gear"])))
            contact_pairs_expected.add(frozenset((stage["ring"], planet["gear"])))
    trajectory = {"t": [], "bodies": {name: [] for name in bodies}}
    max_contacts, sampled_contacts, observed_contact_pairs = 0, [], set()
    reporter = _Contacts(c)
    for i in range(max(1, int(duration/dt))):
        system.DoStepDynamics(dt)
        if i % max(1, int(.01/dt)) == 0:
            trajectory["t"].append(float(system.GetChTime()))
            for name, body in bodies.items():
                p, q = body.GetPos(), body.GetRot()
                trajectory["bodies"][name].append({"xyz_m": _vec(p),
                    "quat_wxyz": [float(q.e0), float(q.e1), float(q.e2), float(q.e3)]})
            contacts = reporter.collect(system)
            max_contacts = max(max_contacts, len(contacts))
            for contact in contacts:
                observed_contact_pairs.add(frozenset((contact["body_a"], contact["body_b"])))
            if contacts:
                sampled_contacts = contacts[:200]

    translations, rotations = {}, {}
    finite = True
    for name, body in bodies.items():
        p0, q0 = initial[name]
        p, q = body.GetPos(), body.GetRot()
        translations[name] = math.dist(p0, _vec(p))
        rotations[name] = _rotation_delta(q0, q)
        finite = finite and all(math.isfinite(x) for x in (*_vec(p), q.e0, q.e1, q.e2, q.e3))
    moved = [name for name in bodies if translations[name] > 1e-4 or rotations[name] > 1e-3]
    watched = bundle.get("watch_links") or ([bundle.get("output_link")] if bundle.get("output_link") else [])
    watched = [name for name in watched if name in bodies]
    output_moved = any(name in moved for name in watched) if watched else bool(moved)

    motor_state = {name: {"angle_rad": float(link.GetMotorAngle()),
                          "speed_rad_s": float(link.GetMotorAngleDt()),
                          "torque_nm": float(link.GetMotorTorque())}
                   for name, link in links.items()
                   if isinstance(link, c.ChLinkMotorRotationTorque)}
    reactions, residuals, joint_coordinates = {}, {}, {}
    for name, link in links.items():
        values = _constraint_violation(link)
        if values:
            residuals[name] = {"values": values, "max_abs": max(abs(x) for x in values)}
        try:
            wrench = link.GetReaction1()
            reactions[name] = {"force_local_n": _vec(wrench.force),
                               "torque_local_nm": _vec(wrench.torque)}
        except Exception:
            pass
        rec = link_records.get(name) or {}
        if rec.get("source_kind") != "motion_joint":
            continue
        axis = (rec.get("world_frame") or {}).get("axis_world") or [0.0, 0.0, 1.0]
        child = bodies.get(rec.get("child"))
        parent = bodies.get(rec.get("parent"))
        if child is None:
            continue
        coordinate = {"type": rec.get("type")}
        if rec.get("type") == "hinge":
            try:
                coordinate["position"] = float(link.GetMotorAngle() if isinstance(
                    link, c.ChLinkMotorRotationTorque) else link.GetRelAngle())
            except Exception:
                pass
            coordinate["velocity"] = float(_relative_speed(child, parent, axis))
        elif rec.get("type") == "slide":
            try:
                coordinate["position"] = float(link.GetRelCoordsys().pos.z)
            except Exception:
                pass
            velocity = child.GetPosDt()
            parent_velocity = parent.GetPosDt() if parent is not None else None
            coordinate["velocity"] = float(
                (velocity.x-(parent_velocity.x if parent_velocity else 0.0))*axis[0]
                +(velocity.y-(parent_velocity.y if parent_velocity else 0.0))*axis[1]
                +(velocity.z-(parent_velocity.z if parent_velocity else 0.0))*axis[2])
        joint_coordinates[name] = coordinate
    Path(out/"trajectory.json").write_text(json.dumps(trajectory, indent=2), encoding="utf-8")
    Path(out/"contacts.json").write_text(json.dumps({"max_count": max_contacts,
                                                       "sample": sampled_contacts}, indent=2), encoding="utf-8")
    Path(out/"reactions.json").write_text(json.dumps(reactions, indent=2), encoding="utf-8")
    Path(out/"constraint_residuals.json").write_text(json.dumps(
        residuals, indent=2), encoding="utf-8")

    transmission_checks = []
    for stage in bundle.get("planetary_stages") or []:
        if not stage.get("compiled") or bundle.get("mode") != "ideal_dynamic":
            continue
        by_child = {rec.get("child"): joint_coordinates.get(rec.get("source_name"), {})
                    for rec in bundle["constraints"]
                    if rec.get("source_kind") == "motion_joint"}
        sun = by_child.get(stage["sun"], {})
        carrier = by_child.get(stage["carrier"], {})
        expected = stage["sun_teeth"]/float(stage["sun_teeth"]+stage["ring_teeth"])
        # Legacy Chrono hinges expose GetRelAngle() wrapped to [-pi, pi], whereas the
        # torque motor is continuous. Ratios taken from those mixed positions fail after
        # one revolution. Angular velocities are continuous and authoritative here.
        sun_value, carrier_value, basis = sun.get("velocity"), carrier.get("velocity"), "velocity"
        if sun_value is None or abs(sun_value) <= 1e-6 or carrier_value is None:
            sun_value, carrier_value, basis = sun.get("position"), carrier.get("position"), "position"
        if sun_value is not None and abs(sun_value) > 1e-6 and carrier_value is not None:
            actual = carrier_value/sun_value
            transmission_checks.append({"stage": stage["source_name"],
                "kind": "fixed_ring_carrier_ratio", "basis": basis,
                "expected": expected, "actual": actual, "error": abs(actual-expected),
                "passed": abs(actual-expected) <= 0.01})
        planet_expected = -(stage["ring_teeth"]/float(stage["planet_teeth"]))
        for planet in stage.get("planets") or []:
            planet_coord = by_child.get(planet["gear"], {})
            planet_value = planet_coord.get(basis)
            if carrier_value is None or abs(carrier_value) <= 1e-6 or planet_value is None:
                continue
            actual = planet_value/carrier_value
            transmission_checks.append({"stage": stage["source_name"],
                "kind": "planet_local_spin", "gear": planet["gear"], "basis": basis,
                "expected": planet_expected, "actual": actual,
                "error": abs(actual-planet_expected),
                "passed": abs(actual-planet_expected) <= 0.02})
    unsafe_collision_bodies = sorted(name for name, representation in collision_repr.items()
                                     if representation == "disabled_dynamic_triangle_mesh")
    contact_pair_checks = [{"pair": sorted(pair), "observed": pair in observed_contact_pairs}
                           for pair in sorted(contact_pairs_expected, key=lambda p: sorted(p))]
    contact_mesh_healthy = (not unsafe_collision_bodies and
                            (all(check["observed"] for check in contact_pair_checks)
                             if contact_pair_checks else True))
    transmission_healthy = bool(transmission_checks) and all(
        check["passed"] for check in transmission_checks) if bundle.get(
            "planetary_stages") and bundle.get("mode") == "ideal_dynamic" else True

    max_residual = max((x["max_abs"] for x in residuals.values()), default=0.0)
    # Exact constraint components mix metres and radians. This catches centimeter-scale
    # separation/explosion without pretending to be an engineering tolerance certificate.
    constraints_healthy = max_residual <= 1e-3
    exploded = max(translations.values(), default=0.0) > .5
    contact_output_healthy = output_moved
    passed = (not compile_errors and finite and not exploded and constraints_healthy
              and transmission_healthy and contact_mesh_healthy and contact_output_healthy)
    metrics = {"verdict": "PASS" if passed else "FAIL", "test_kind": "driven_mechanism",
               "moved_bodies": moved, "translation_m": translations,
               "rotation_rad": rotations, "watched_links": watched,
               "output_moved": output_moved, "max_contact_count": max_contacts,
               "contact_pair_checks": contact_pair_checks,
               "unsafe_collision_bodies": unsafe_collision_bodies,
               "contact_mesh_healthy": contact_mesh_healthy,
               "collision_representation": collision_repr,
               "motor_links": motor_links, "motor_state": motor_state,
               "joint_coordinates": joint_coordinates,
               "constraint_residuals": residuals,
               "max_constraint_residual": max_residual,
               "constraints_healthy": constraints_healthy,
               "transmission_checks": transmission_checks,
               "transmission_healthy": transmission_healthy,
               "compile_errors": compile_errors,
               "numerical_health": {"finite": finite, "exploded": exploded}}
    frames_dir = _render_trajectory_frames(bundle, trajectory, out)
    result = {"passed": passed, "verdict": metrics["verdict"],
              "summary": f"Chrono moved {len(moved)}/{len(bodies)} bodies; output_moved={output_moved}",
              "metrics": metrics, "tests": [], "frames_dir": frames_dir,
              "cause": "none" if passed else "backend" if compile_errors else
                       "numerics" if not constraints_healthy or exploded else "structure",
              "reason": (json.dumps(compile_errors) if compile_errors else
                         f"constraint residual {max_residual:.6g}" if not constraints_healthy else
                         "body displacement exceeded 0.5m" if exploded else
                         "declared transmission ratio failed" if not transmission_healthy else
                         "unsafe dynamic triangle meshes disabled" if unsafe_collision_bodies else
                         "expected gear contact pairs were not observed" if not contact_mesh_healthy else
                         "output did not move" if not contact_output_healthy else "")}
    Path(out/"sim_result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle", required=True)
    parser.add_argument("--spec", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    run(args.bundle, args.spec, args.out)
