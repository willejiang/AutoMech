#!/usr/bin/env python3
"""
URDF-direct evaluator runner — runs INSIDE isaac-sim:6.0.1.

Takes a URDF + a task prompt, imports into Isaac, runs the physics, captures
frames, measures stability metrics, writes sim_result.json. The VLM pass/fail
(analyze.py) runs on the host afterwards.

Usage (inside container):
  ./python.sh /work/run_eval_urdf.py \
     --urdf /work/anymal_c_simple_description/urdf/anymal.urdf \
     --asset-root /work/anymal_c_simple_description \
     --prompt "make sure it could stand still" \
     --out /work/out_anymal \
     --duration 4.0

This is intentionally minimal + heavily logged so we can debug the Isaac API
surface by running it and reading the failures.
"""
import argparse, json, math, os, sys
from pathlib import Path


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--asset-root", default=None, help="dir that package:// resolves to (parent of meshes/)")
    ap.add_argument("--prompt", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=4.0)
    ap.add_argument("--dt", type=float, default=1.0/120.0)
    ap.add_argument("--spawn-height", type=float, default=0.65, help="z to drop the base from (m)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    return ap.parse_args()


def main():
    args = parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    frames_dir = out / "frames"; frames_dir.mkdir(exist_ok=True)
    log = []
    def L(m):
        print(f"[eval] {m}", flush=True); log.append(m)

    # ---- boot Isaac headless ----
    from isaacsim.simulation_app import SimulationApp
    sim_app = SimulationApp({"headless": True, "width": args.width, "height": args.height})
    L("SimulationApp booted")

    import numpy as np
    from isaacsim.core.api import World
    import omni.usd

    world = World(stage_units_in_meters=1.0, physics_dt=args.dt, rendering_dt=args.dt)
    world.scene.add_default_ground_plane()
    L("world + ground created")

    # ---- import URDF using the real 6.0.1 API (URDFImporter/URDFImporterConfig) ----
    try:
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
        ic = URDFImporterConfig()
        ic.urdf_path = str(args.urdf)
        ic.merge_fixed_joints = True
        ic.fix_base = False
        ic.collision_from_visuals = True
        # resolve package:// : list of {name, path} dicts. package name = first path
        # segment after package://, path = the dir that name maps to.
        if args.asset_root:
            ar = Path(args.asset_root)
            ic.ros_package_paths = [{"name": ar.name, "path": str(ar)}]
        # usd_path is a DIRECTORY where USD output is written
        usd_dir = str(out / "usd"); Path(usd_dir).mkdir(parents=True, exist_ok=True)
        ic.usd_path = usd_dir
        importer = URDFImporter(ic)
        output_usd = importer.import_urdf()
        L(f"URDF imported -> usd={output_usd}")
    except Exception as e:
        import traceback
        L(f"URDF IMPORT FAILED: {e!r}")
        L("TRACE: " + traceback.format_exc().replace(chr(10), " | "))
        sim_app.close(); _dump(out, log, None); raise

    # ---- add the imported USD into our live stage as a reference ----
    from isaacsim.core.utils.stage import add_reference_to_stage
    from pxr import UsdGeom, Usd
    ROBOT_PRIM = "/World/robot"
    add_reference_to_stage(usd_path=str(output_usd), prim_path=ROBOT_PRIM)
    stage = omni.usd.get_context().get_stage()
    base = stage.GetPrimAtPath(ROBOT_PRIM)
    L(f"robot referenced at {ROBOT_PRIM}; valid={base.IsValid()}")

    # find the articulation root prim under the referenced robot (the importer marks it)
    prim_path = ROBOT_PRIM
    for p in Usd.PrimRange(base):
        if p.HasAPI(__import__('pxr').UsdPhysics.ArticulationRootAPI):
            prim_path = p.GetPath().pathString
            break
    L(f"articulation root prim = {prim_path}")

    # ---- wrap as an Articulation so we can read joints/pose ----
    world.reset()
    art = None
    try:
        from isaacsim.core.prims import Articulation
        art = Articulation(prim_paths_expr=prim_path, name="robot")
        world.scene.add(art)
        world.reset()
        ndof = art.num_dof if hasattr(art, "num_dof") else "?"
        L(f"Articulation wrapped: dof={ndof}")
    except Exception as e:
        L(f"Articulation wrap failed (will fall back to prim pose): {e!r}")

    # ---- bind a RigidPrim to the actual BASE LINK body for trustworthy pose ----
    # (articulation-root pose can be a fixed reference frame that does NOT track the
    #  body as it topples -> the original metric bug that reported a fallen robot as stable)
    base_body = None
    try:
        from isaacsim.core.prims import RigidPrim
        # the articulation root prim path IS the base link; read its rigid-body pose
        base_body = RigidPrim(prim_paths_expr=prim_path, name="base_body")
        world.reset()
        bp, bq = base_body.get_world_poses()
        L(f"base RigidPrim bound at {prim_path}; init z={float(np.asarray(bp)[0][2]):.3f}")
    except Exception as e:
        L(f"base RigidPrim bind failed (will fall back to articulation pose): {e!r}")

    # ---- camera + frame capture via replicator ----
    import omni.replicator.core as rep
    cam = rep.create.camera(position=(3.0, -3.0, 1.5), look_at=(0, 0, 0.4))
    rp = rep.create.render_product(cam, (args.width, args.height))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=str(frames_dir), rgb=True)
    writer.attach([rp])
    L("replicator camera + writer attached")

    # ---- sim loop ----
    n = int(args.duration / args.dt)
    cap_every = max(1, n // 120)
    metrics = {"start_z": None, "min_z": 1e9, "end_z": None,
               "max_tilt_deg": 0.0, "max_dof_drift": 0.0, "exploded": False,
               "fell_over": False, "moved_dist": 0.0,
               "pose_source": None, "metric_read_errors": 0}
    base_xy0 = None
    dof0 = None
    for step in range(n):
        world.step(render=(step % cap_every == 0))
        try:
            # prefer the base rigid body; fall back to articulation root
            if base_body is not None:
                pos, quat = base_body.get_world_poses()
                pos = np.asarray(pos)[0]; quat = np.asarray(quat)[0]
                metrics["pose_source"] = "base_rigid_body"
            elif art is not None:
                pos, quat = art.get_world_poses()
                pos = np.asarray(pos)[0]; quat = np.asarray(quat)[0]
                metrics["pose_source"] = "articulation_root"
            else:
                xf = UsdGeom.Xformable(base)
                m = xf.ComputeLocalToWorldTransform(0)
                t = m.ExtractTranslation(); pos = np.array([t[0], t[1], t[2]]); quat = None
                metrics["pose_source"] = "usd_xform"
            z = float(pos[2])
            if metrics["start_z"] is None:
                metrics["start_z"] = z; base_xy0 = pos[:2].copy()
            metrics["min_z"] = min(metrics["min_z"], z)
            metrics["end_z"] = z
            if z > 30: metrics["exploded"] = True
            metrics["moved_dist"] = float(np.linalg.norm(pos[:2] - base_xy0))
            if quat is not None:
                tilt = _tilt_deg(quat)
                metrics["max_tilt_deg"] = max(metrics["max_tilt_deg"], tilt)
                if tilt > 50: metrics["fell_over"] = True
            if art is not None:
                dof = np.asarray(art.get_joint_positions())[0]
                if dof0 is None: dof0 = dof.copy()
                metrics["max_dof_drift"] = max(metrics["max_dof_drift"], float(np.max(np.abs(dof - dof0))))
        except Exception as e:
            metrics["metric_read_errors"] += 1
            if metrics["metric_read_errors"] <= 3:
                L(f"metric read error (step {step}): {e!r}")
    L(f"sim loop done ({n} steps); pose_source={metrics['pose_source']} read_errors={metrics['metric_read_errors']}")

    # ---- verdict from metrics ----
    failures = []
    if metrics["exploded"]:
        failures.append("articulation unstable (exploded)")
    if metrics["fell_over"]:
        failures.append(f"fell over (tilt {metrics['max_tilt_deg']:.0f} deg)")
    if metrics["start_z"] and metrics["end_z"] and (metrics["start_z"] - metrics["end_z"]) > 0.25:
        failures.append(f"sank/collapsed (z {metrics['start_z']:.2f}->{metrics['end_z']:.2f})")
    if metrics["moved_dist"] > 0.3:
        failures.append(f"drifted {metrics['moved_dist']:.2f}m (not still)")
    metrics["failures_metric"] = failures
    metrics["passed_metric"] = len(failures) == 0
    L(f"metric verdict: passed={metrics['passed_metric']} failures={failures}")

    n_frames = len(list(frames_dir.glob('*.png')))
    L(f"frames captured: {n_frames}")

    # write result BEFORE closing — sim_app.close() hard-exits the process
    result = {"prompt": args.prompt, "urdf": str(args.urdf),
              "metrics": metrics, "frames_dir": str(frames_dir),
              "n_frames": n_frames, "log": log}
    (out / "sim_result.json").write_text(json.dumps(result, indent=2))
    L(f"wrote {out/'sim_result.json'}")

    sim_app.update()
    sim_app.close()


def _tilt_deg(quat):
    # angle between body-up and world-up; quat = [w,x,y,z]
    w, x, y, z = quat
    up_z = 1 - 2*(x*x + y*y)
    up_z = max(-1.0, min(1.0, up_z))
    return math.degrees(math.acos(up_z))


def _dump(out, log, extra):
    (Path(out) / "sim_result.json").write_text(json.dumps({"log": log, "error": extra}, indent=2))


if __name__ == "__main__":
    main()
