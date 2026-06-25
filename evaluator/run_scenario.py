#!/usr/bin/env python3
"""
Spec-driven scenario runner — runs INSIDE isaac-sim:6.0.1.

Consumes a SCENARIO SPEC (produced by scenario_designer.py / the LLM) and a URDF,
executes the physics test, captures frames, evaluates the spec's pass_criteria,
writes sim_result.json. Nothing task-specific is hardcoded — the spec drives it.

  /isaac-sim/python.sh /work/run_scenario.py \
     --urdf .../robot.urdf --asset-root ... \
     --spec /work/spec.json --task "..." --out /work/out_iterN
"""
import argparse, json, math
from pathlib import Path


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--asset-root", default=None)
    ap.add_argument("--spec", required=True)
    ap.add_argument("--task", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--dt", type=float, default=1.0/240.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    return ap.parse_args()


def main():
    a = parse_args()
    spec = json.loads(Path(a.spec).read_text())
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    frames_dir = out / "frames"; frames_dir.mkdir(exist_ok=True)
    log = []
    def L(m): print(f"[scn] {m}", flush=True); log.append(m)
    L(f"spec reasoning: {spec.get('reasoning','')[:200]}")

    from isaacsim.simulation_app import SimulationApp
    sim_app = SimulationApp({"headless": True, "width": a.width, "height": a.height})

    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    import omni.usd
    from pxr import UsdGeom, Usd, UsdPhysics, Gf

    dt = a.dt
    world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)
    world.scene.add_default_ground_plane()

    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
    ic = URDFImporterConfig()
    ic.urdf_path = str(a.urdf); ic.merge_fixed_joints = True
    ic.fix_base = False; ic.collision_from_visuals = True
    if a.asset_root:
        ar = Path(a.asset_root); ic.ros_package_paths = [{"name": ar.name, "path": str(ar)}]
    usd_dir = str(out / "usd"); Path(usd_dir).mkdir(parents=True, exist_ok=True)
    ic.usd_path = usd_dir
    output_usd = URDFImporter(ic).import_urdf()
    L(f"URDF imported -> {output_usd}")

    ROBOT = "/World/robot"
    add_reference_to_stage(usd_path=str(output_usd), prim_path=ROBOT)
    stage = omni.usd.get_context().get_stage()
    base = stage.GetPrimAtPath(ROBOT)
    art_prim = ROBOT
    for p in Usd.PrimRange(base):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_prim = p.GetPath().pathString; break

    # ---- apply spec: orientation + height ----
    eul = spec.get("base_orientation_euler", [0, 0, 0])
    h = float(spec.get("base_height", 0.8))
    rx = UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT))
    rx.ClearXformOpOrder()
    rx.AddTranslateOp().Set(Gf.Vec3d(0, 0, h))
    rx.AddRotateXYZOp().Set(Gf.Vec3f(math.degrees(eul[0]), math.degrees(eul[1]), math.degrees(eul[2])))
    L(f"oriented euler(deg)={[round(math.degrees(e),1) for e in eul]} height={h}")

    from isaacsim.core.prims import Articulation
    art = Articulation(prim_paths_expr=art_prim, name="robot")
    world.scene.add(art); world.reset()
    try: dof_names = list(art.dof_names)
    except Exception: dof_names = []
    L(f"dof={len(dof_names)}")

    # ---- apply spec: joint pose (list of {joint, angle}) ----
    pose = {d["joint"]: float(d["angle"]) for d in spec.get("joint_pose", [])}
    targets = np.zeros(len(dof_names), dtype=np.float32)
    matched = 0
    for i, nm in enumerate(dof_names):
        if nm in pose:
            targets[i] = pose[nm]; matched += 1
    L(f"joint_pose matched {matched}/{len(pose)} named joints")
    try: art.set_joint_positions(np.array([targets]))
    except Exception:
        try: art.set_joint_positions(targets)
        except Exception as e: L(f"set_joint_positions: {e!r}")

    # ---- apply spec: high friction on contact links (best-effort physics material) ----
    hf = set(spec.get("high_friction_links", []))
    if hf:
        try:
            from pxr import UsdShade, PhysxSchema
            mat_path = "/World/highfric_mat"
            mat = UsdShade.Material.Define(stage, mat_path)
            pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            pm.CreateStaticFrictionAttr().Set(2.0)
            pm.CreateDynamicFrictionAttr().Set(1.8)
            for p in Usd.PrimRange(base):
                if any(p.GetName() == n or p.GetPath().pathString.endswith(n) for n in hf):
                    UsdShade.MaterialBindingAPI.Apply(p).Bind(
                        mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                        materialPurpose="physics")
            L(f"high friction applied to {len(hf)} link(s)")
        except Exception as e:
            L(f"friction apply failed: {e!r}")

    # ---- camera + capture ----
    import omni.replicator.core as rep
    cam = rep.create.camera(position=(2.4, -2.4, 1.0), look_at=(0, 0, 0.6))
    rp = rep.create.render_product(cam, (a.width, a.height))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=str(frames_dir), rgb=True); writer.attach([rp])

    base_xf = UsdGeom.Xformable(stage.GetPrimAtPath(art_prim))
    def read_base():
        m = base_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        return np.array([t[0], t[1], t[2]])

    ctrl = spec.get("control", {})
    pc = spec.get("pass_criteria", {})
    duration = float(spec.get("duration_s", 8.0))
    floor = float(pc.get("min_base_height", 0.3))
    max_drift = float(pc.get("max_drift", 0.5))
    need_s = float(pc.get("survive_s", duration))
    hold = ctrl.get("mode", "position_hold") == "position_hold"

    # capture at a fixed 30 fps in SIM TIME so even fast-fail runs get dense,
    # smooth frames (true recording). VLM samples ~12 from these later.
    rec_fps = 30
    cap_every = max(1, int(round((1.0 / rec_fps) / dt)))
    n = int(duration / dt)
    metrics = {"verdict": None, "survive_s": 0.0, "min_base_z": 1e9,
               "max_drift": 0.0, "start_z": None}
    xy0 = None
    for step in range(n):
        if hold and len(dof_names):
            try: art.set_joint_position_targets(np.array([targets]))
            except Exception:
                try: art.set_joint_position_targets(targets)
                except Exception: pass
        world.step(render=(step % cap_every == 0))
        pos = read_base(); z = float(pos[2])
        if metrics["start_z"] is None:
            metrics["start_z"] = z; xy0 = pos[:2].copy()
        metrics["min_base_z"] = min(metrics["min_base_z"], z)
        metrics["max_drift"] = max(metrics["max_drift"], float(np.linalg.norm(pos[:2]-xy0)))
        metrics["survive_s"] = step*dt
        if z < floor:
            metrics["verdict"] = "FAIL"; L(f"FAIL: base_z {z:.3f} < floor {floor} at t={step*dt:.2f}s"); break
        if metrics["max_drift"] > max_drift:
            metrics["verdict"] = "FAIL"; L(f"FAIL: drift {metrics['max_drift']:.2f} > {max_drift} at t={step*dt:.2f}s"); break
        if step % 240 == 0:
            L(f"t={step*dt:.2f}s base_z={z:.3f} drift={metrics['max_drift']:.3f}")
    if metrics["verdict"] is None:
        metrics["verdict"] = "PASS" if metrics["survive_s"] >= need_s - dt else "FAIL"
        L(f"end: verdict={metrics['verdict']} survived={metrics['survive_s']:.2f}s/{need_s}s")
    metrics["passed_metric"] = metrics["verdict"] == "PASS"

    n_frames = len(list(frames_dir.glob("*.png")))
    L(f"frames={n_frames}")
    (out / "sim_result.json").write_text(json.dumps(
        {"task": a.task, "spec": spec, "metrics": metrics,
         "frames_dir": str(frames_dir), "n_frames": n_frames, "log": log}, indent=2))
    L(f"wrote sim_result.json")
    sim_app.update(); sim_app.close()


if __name__ == "__main__":
    main()
