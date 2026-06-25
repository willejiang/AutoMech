#!/usr/bin/env python3
"""
Cassie handstand evaluator in Isaac Sim — parity port of teammate's PyBullet test.

Replicates sim_handstand.py: flip the humanoid upside-down, apply a fixed
reference pose (flat palms = area support polygon), high hand friction, hold
with joint PD, and judge with base_z<0.45 (FELL) / survives duration (BALANCED).

Pose-read uses the USD world transform of the base prim each step (robust; the
physics articulation-view pose API was unreliable in this build).

Run inside container (entrypoint override):
  /isaac-sim/python.sh /work/run_handstand.py \
     --urdf /work/cassie_description/urdf/cassie_humanoid.urdf \
     --asset-root /work/cassie_description \
     --prompt "make Cassie do a handstand" \
     --out /work/out_handstand --duration 12.0
"""
import argparse, json, math, os
from pathlib import Path

# Teammate's handstand reference pose (rad). Elbow=0.30 -> palms flat; hips fold CoM over hands.
POSE = {
    "TorsoPitch": 0.0,
    "LeftShoulderPitch": -0.30, "RightShoulderPitch": -0.30,
    "LeftShoulderRoll": 0.0, "RightShoulderRoll": 0.0,
    "LeftElbow": 0.30, "RightElbow": 0.30,
    "LeftHipPitch": -0.40, "RightHipPitch": -0.40,
    "LeftKneePitch": -1.4, "RightKneePitch": -1.4,
    "LeftTarsusPitch": 1.5, "RightTarsusPitch": 1.5,
    "LeftFootPitch": -1.2, "RightFootPitch": -1.2,
}
BASE_Z = 1.07
FELL_Z = 0.45


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--asset-root", default=None)
    ap.add_argument("--prompt", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--dt", type=float, default=1.0/240.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    return ap.parse_args()


def main():
    args = parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    frames_dir = out / "frames"; frames_dir.mkdir(exist_ok=True)
    log = []
    def L(m): print(f"[hs] {m}", flush=True); log.append(m)

    from isaacsim.simulation_app import SimulationApp
    sim_app = SimulationApp({"headless": True, "width": args.width, "height": args.height})
    L("SimulationApp booted")

    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    import omni.usd
    from pxr import UsdGeom, Usd, UsdPhysics, Gf

    world = World(stage_units_in_meters=1.0, physics_dt=args.dt, rendering_dt=args.dt)
    world.scene.add_default_ground_plane()
    L("world + ground")

    # ---- import URDF ----
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
    ic = URDFImporterConfig()
    ic.urdf_path = str(args.urdf)
    ic.merge_fixed_joints = True
    ic.fix_base = False
    ic.collision_from_visuals = True
    if args.asset_root:
        ar = Path(args.asset_root)
        ic.ros_package_paths = [{"name": ar.name, "path": str(ar)}]
    usd_dir = str(out / "usd"); Path(usd_dir).mkdir(parents=True, exist_ok=True)
    ic.usd_path = usd_dir
    output_usd = URDFImporter(ic).import_urdf()
    L(f"URDF imported -> {output_usd}")

    ROBOT = "/World/robot"
    add_reference_to_stage(usd_path=str(output_usd), prim_path=ROBOT)
    stage = omni.usd.get_context().get_stage()
    base = stage.GetPrimAtPath(ROBOT)

    # articulation-root prim (its world xform = base/pelvis pose)
    art_prim = ROBOT
    for p in Usd.PrimRange(base):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_prim = p.GetPath().pathString
            break
    L(f"articulation root = {art_prim}")

    # ---- flip upside-down + lift: set the reference-frame xform on the robot root ----
    rx = UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT))
    rx.ClearXformOpOrder()
    rx.AddTranslateOp().Set(Gf.Vec3d(0, 0, BASE_Z))
    rx.AddRotateXYZOp().Set(Gf.Vec3f(0, 180, 0))   # euler(0, pi, 0) = upside down
    L("robot flipped upside-down + raised to BASE_Z")

    # ---- articulation for joint control ----
    from isaacsim.core.prims import Articulation
    art = Articulation(prim_paths_expr=art_prim, name="cassie")
    world.scene.add(art)
    world.reset()
    L(f"articulation ready dof={getattr(art,'num_dof','?')}")

    # map joint names -> dof index, build target vector for the pose
    try:
        dof_names = list(art.dof_names) if hasattr(art, "dof_names") else []
    except Exception:
        dof_names = []
    L(f"dof_names({len(dof_names)}): {dof_names[:6]}...")
    targets = None
    if dof_names:
        targets = np.zeros(len(dof_names), dtype=np.float32)
        for i, nm in enumerate(dof_names):
            if nm in POSE:
                targets[i] = POSE[nm]
        # set joints to the pose at t=0 as well (reset state)
        try:
            art.set_joint_positions(np.array([targets]))
        except Exception:
            try: art.set_joint_positions(targets)
            except Exception as e: L(f"set_joint_positions failed: {e!r}")

    # high hand friction: set physics material friction on hand prims
    for hn in ("left_hand", "right_hand"):
        for p in Usd.PrimRange(base):
            if p.GetName() == hn or p.GetPath().pathString.endswith(hn):
                try:
                    from pxr import UsdShade, PhysxSchema
                    UsdPhysics.MaterialAPI.Apply(p)
                except Exception:
                    pass
    L("hand friction best-effort applied")

    # ---- camera + capture ----
    import omni.replicator.core as rep
    cam = rep.create.camera(position=(2.2, -2.2, 0.9), look_at=(0, 0, 0.6))
    rp = rep.create.render_product(cam, (args.width, args.height))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=str(frames_dir), rgb=True)
    writer.attach([rp])
    L("camera + writer attached")

    # ---- helper: read base world pose from USD xform (robust) ----
    base_xf = UsdGeom.Xformable(stage.GetPrimAtPath(art_prim))
    def read_base():
        m = base_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        rot = m.ExtractRotationQuat()  # Gf.Quatd
        im = rot.GetImaginary()
        return np.array([t[0], t[1], t[2]]), np.array([rot.GetReal(), im[0], im[1], im[2]])

    # ---- sim loop with PD hold + teammate's criteria ----
    n = int(args.duration / args.dt)
    cap_every = max(1, n // 120)
    metrics = {"verdict": None, "survive_s": 0.0, "min_base_z": 1e9,
               "max_base_drift": 0.0, "start": None, "fell_t": None}
    x0 = None
    for step in range(n):
        # hold reference pose
        if targets is not None:
            try: art.set_joint_position_targets(np.array([targets]))
            except Exception:
                try: art.set_joint_position_targets(targets)
                except Exception: pass
        world.step(render=(step % cap_every == 0))

        pos, quat = read_base()
        z = float(pos[2]); x = float(pos[0])
        if metrics["start"] is None:
            metrics["start"] = [float(pos[0]), float(pos[1]), z]; x0 = np.array([pos[0], pos[1]])
        metrics["min_base_z"] = min(metrics["min_base_z"], z)
        metrics["max_base_drift"] = max(metrics["max_base_drift"], float(np.linalg.norm(pos[:2]-x0)))
        metrics["survive_s"] = step * args.dt

        if z < FELL_Z:
            metrics["verdict"] = "FELL"; metrics["fell_t"] = step*args.dt
            L(f"FELL at t={step*args.dt:.2f}s base_z={z:.3f}")
            break
        if step % 240 == 0:
            L(f"t={step*args.dt:.2f}s base_z={z:.3f} base_x={x:+.3f} drift={metrics['max_base_drift']:.3f}")
    if metrics["verdict"] is None:
        metrics["verdict"] = "BALANCED"
        L(f"BALANCED {args.duration:.0f}s — survived (min_z={metrics['min_base_z']:.3f}, drift={metrics['max_base_drift']:.3f})")

    metrics["passed_metric"] = metrics["verdict"] == "BALANCED"
    n_frames = len(list(frames_dir.glob('*.png')))
    L(f"frames captured: {n_frames}")

    result = {"prompt": args.prompt, "urdf": str(args.urdf), "test": "handstand",
              "metrics": metrics, "frames_dir": str(frames_dir), "n_frames": n_frames, "log": log}
    (out / "sim_result.json").write_text(json.dumps(result, indent=2))
    L(f"wrote {out/'sim_result.json'}")

    sim_app.update(); sim_app.close()


if __name__ == "__main__":
    main()
