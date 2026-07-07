"""Golden 2-gear transmission test (maker2-mujoco-contact Phase 3, the KEY de-risk).

Two meshing spur gears, built as real toothed cylinders (concave -> convex-decomposed
so the teeth collide as pieces). Gear A is driven by an applied torque on its own hinge;
gear B is free to spin on its own hinge. If pure tooth CONTACT transmits, gear B spins
up (opposite sign to A). If it cannot, the whole pure-contact premise fails and we must
escalate (per the plan: MuJoCo <equality> gear-ratio constraint behind a flag).

Run:  python -m maker2.tests.golden_two_gears
Exit 0 = transmission confirmed; exit 1 = FAILED to transmit (escalate).
"""
from __future__ import annotations

import math
import os
import sys
import tempfile

import numpy as np
import trimesh

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _ROOT)


def make_gear(pitch_r_mm: float, n_teeth: int, thick_mm: float,
              tooth_h_mm: float = 3.0) -> trimesh.Trimesh:
    """A crude spur gear: a hub cylinder + radial box teeth around the rim. Concave,
    so convex decomposition yields per-tooth pieces that can interlock."""
    hub = trimesh.creation.cylinder(radius=pitch_r_mm - tooth_h_mm * 0.5,
                                    height=thick_mm, sections=max(24, n_teeth * 2))
    parts = [hub]
    tooth_w = (2 * math.pi * pitch_r_mm / n_teeth) * 0.55   # ~half the pitch as tooth
    for k in range(n_teeth):
        ang = 2 * math.pi * k / n_teeth
        tooth = trimesh.creation.box((tooth_h_mm * 2.0, tooth_w, thick_mm))
        # place the tooth centered on the pitch circle, long axis radial
        T = trimesh.transformations.rotation_matrix(ang, (0, 0, 1))
        T[:3, 3] = [pitch_r_mm * math.cos(ang), pitch_r_mm * math.sin(ang), 0.0]
        tooth.apply_transform(T)
        parts.append(tooth)
    gear = trimesh.util.concatenate(parts)
    return gear


def build_model(td: str):
    from maker2.model import KinematicModel, LinkSpec, PoseSpec, RunContext
    from maker2.manager import _validate_model
    from maker2 import urdf_builder as ub

    md = os.path.join(td, "meshes")
    os.makedirs(md, exist_ok=True)

    pitch_r = 20.0   # mm
    n = 12
    thick = 8.0
    # Two identical gears; centers exactly 2*pitch_r apart so pitch circles are tangent.
    ga = make_gear(pitch_r, n, thick)
    gb = make_gear(pitch_r, n, thick)
    # Rotate gear B by half a tooth so teeth interleave rather than tooth-on-tooth.
    gb.apply_transform(trimesh.transformations.rotation_matrix(math.pi / n, (0, 0, 1)))
    ga.export(os.path.join(md, "gear_a.stl"))
    gb.export(os.path.join(md, "gear_b.stl"))
    # A base plate the gears sit above (fixed, rests on the plane).
    base = trimesh.creation.box((120, 60, 6))
    base.export(os.path.join(md, "base.stl"))

    center_dist_m = 2 * pitch_r / 1000.0
    model = KinematicModel(
        name="two_gears", root_link="base",
        links=[
            LinkSpec(name="base", description="plate", dof="fixed",
                     color=(0.3, 0.3, 0.32, 1.0)),
            LinkSpec(name="gear_a", description="drive gear", dof="spin",
                     spin_axis=(0, 0, 1), driver=True, color=(0.80, 0.62, 0.20, 1.0)),
            LinkSpec(name="gear_b", description="driven gear", dof="spin",
                     spin_axis=(0, 0, 1), color=(0.55, 0.7, 0.85, 1.0)),
        ],
        poses=[
            PoseSpec(name="p_base", parent="", child="base", xyz_m=(0, 0, 0)),
            # gears sit above the base top; on the same Z, centers center_dist apart in X
            PoseSpec(name="p_ga", parent="base", child="gear_a",
                     xyz_m=(-center_dist_m / 2, 0, 0.02)),
            PoseSpec(name="p_gb", parent="base", child="gear_b",
                     xyz_m=(center_dist_m / 2, 0, 0.02)),
        ],
        mesh_pairs=[("gear_a", "gear_b")],
    )
    _validate_model(model)
    ctx = RunContext(project_slug="two_gears", run_dir=td,
                     urdf_path=os.path.join(td, "model.urdf"), meshes_dir=md,
                     logs_dir=td, model_json_path=os.path.join(td, "kinematic_model.json"))
    ub.scaffold_meshes(model, ctx)
    return model, ctx


def _drive_and_measure(mjcf: str, torque: float, settle: int, steps: int):
    import mujoco
    m = mujoco.MjModel.from_xml_path(mjcf)
    d = mujoco.MjData(m)
    ja = m.joint("gear_a_spin").id
    jb = m.joint("gear_b_spin").id
    dofa = m.jnt_dofadr[ja]
    qa, qb = m.jnt_qposadr[ja], m.jnt_qposadr[jb]
    for _ in range(settle):
        mujoco.mj_step(m, d)
    a0, b0 = float(d.qpos[qa]), float(d.qpos[qb])
    for _ in range(steps):
        d.qfrc_applied[dofa] = torque
        mujoco.mj_step(m, d)
    a, b = float(d.qpos[qa]) - a0, float(d.qpos[qb]) - b0
    return a, b, float(d.qvel[dofa])


def main() -> int:
    from maker2 import mjcf_builder
    from maker2.config import Settings

    # 1) PURE CONTACT — proves tooth contact couples the gears in the correct
    #    direction on engagement (the core premise of the plan).
    td = tempfile.mkdtemp(prefix="golden_contact_")
    model, ctx = build_model(td)
    metrics = {}
    mjcf = mjcf_builder.build_mjcf(model, ctx,
                                   settings=Settings(engine="mujoco",
                                                     base_rests_on_plane=False),
                                   metrics=metrics, base_height=0.0, log_fn=print)
    print(f"[contact] contact_degraded={metrics.get('contact_degraded')}")
    a, b, _ = _drive_and_measure(mjcf, torque=0.3, settle=200, steps=4000)
    ratio = (b / a) if a else 0.0
    contact_couples = (abs(b) > 0) and (b * a < 0)
    print(f"[contact] gearA {a:+.3f} gearB {b:+.3f} ratio {ratio:+.3f} "
          f"-> {'couples (correct direction)' if contact_couples else 'NO coupling'}")

    # 2) GEAR-RATIO CONSTRAINT — the escape hatch; must give sustained transmission at
    #    the correct ratio (~-1.0 for equal gears).
    td2 = tempfile.mkdtemp(prefix="golden_constraint_")
    model2, ctx2 = build_model(td2)
    m2 = {}
    mjcf2 = mjcf_builder.build_mjcf(model2, ctx2,
                                    settings=Settings(engine="mujoco",
                                                      base_rests_on_plane=False,
                                                      allow_gear_constraint=True),
                                    metrics=m2, base_height=0.0, log_fn=print)
    a2, b2, avel2 = _drive_and_measure(mjcf2, torque=0.2, settle=200, steps=8000)
    ratio2 = (b2 / a2) if a2 else 0.0
    sustained = abs(a2) > 1.0 and b2 * a2 < 0 and abs(ratio2 + 1.0) < 0.2
    print(f"[constraint] constrained_meshes={m2.get('constrained_meshes')} "
          f"gearA {a2:+.2f} gearB {b2:+.2f} ratio {ratio2:+.3f} Avel {avel2:+.2f} "
          f"-> {'sustained @ correct ratio' if sustained else 'FAILED'}")

    ok = contact_couples and sustained
    print("RESULT: PASS — contact couples + constraint sustains transmission."
          if ok else
          "RESULT: FAIL — see per-path lines above.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
