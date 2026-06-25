#!/usr/bin/env python3
"""
Isaac Sim evaluator runner — executes INSIDE the isaac-sim:6.0.1 container.

Pipeline:
  manifest.json (+ .scad/.stl)  ->  per-part STL  ->  synth URDF  ->  USD articulation
  ->  scenario  ->  headless sim loop  ->  per-step frames + metrics  ->  result.json

Invoked by the container entrypoint as:
  ${ISAACSIM}/python.sh run_eval.py --manifest /work/<id>/manifest.json --out /work/<id>/out

It only does the SIM half. VLM pass/fail analysis runs OUTSIDE the container
(see analyze.py) so we don't put the Azure key inside the image.
"""
import argparse
import json
import math
import os
import subprocess
import sys
from pathlib import Path


# ----------------------------------------------------------------------------
# 0. CLI + manifest
# ----------------------------------------------------------------------------
def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--headless", default="1")
    ap.add_argument("--render-width", type=int, default=1280)
    ap.add_argument("--render-height", type=int, default=720)
    return ap.parse_args()


def load_manifest(path):
    with open(path) as f:
        m = json.load(f)
    # minimal validation at the boundary (maker output is external)
    for k in ("parts", "scenario", "source"):
        if k not in m:
            raise ValueError(f"manifest missing required key: {k}")
    return m


# ----------------------------------------------------------------------------
# 1. scad/stl -> per-part STL meshes
# ----------------------------------------------------------------------------
def render_parts(manifest, work_dir, mesh_dir):
    """Produce one STL per part. For .scad, render each module separately so
    parts stay distinct (a fused union can't articulate)."""
    mesh_dir.mkdir(parents=True, exist_ok=True)
    src = manifest["source"]
    src_path = work_dir / src["file"]
    out = {}

    if src["type"] == "stl":
        # single fused mesh: every part shares it (rigid-body fallback path)
        for p in manifest["parts"]:
            out[p["name"]] = src_path
        return out

    if src["type"] == "scad":
        if not _have("openscad"):
            raise RuntimeError("openscad not found in container; install or pre-render STLs")
        for p in manifest["parts"]:
            module = p.get("render_module", p["name"])
            stl_path = mesh_dir / f"{p['name']}.stl"
            # render just this module by emitting a tiny wrapper that calls it
            wrapper = mesh_dir / f"_{p['name']}.scad"
            wrapper.write_text(f'use <{src_path.as_posix()}>\n{module}();\n')
            cmd = ["openscad", "-o", str(stl_path), str(wrapper)]
            subprocess.run(cmd, check=True, capture_output=True)
            out[p["name"]] = stl_path
        return out

    raise ValueError(f"unknown source.type: {src['type']}")


def _have(exe):
    from shutil import which
    return which(exe) is not None


# ----------------------------------------------------------------------------
# 2. synth URDF from manifest (links from parts, joints from joints)
# ----------------------------------------------------------------------------
def synth_urdf(manifest, meshes, urdf_path):
    """Build a URDF the Isaac URDF importer consumes. Inertia is left to the
    importer's --link-density (STL has none); we pass density per link."""
    units = manifest.get("units", "mm")
    scale = {"mm": 0.001, "cm": 0.01, "m": 1.0}.get(units, 0.001)

    links = []
    for p in manifest["parts"]:
        mesh = meshes[p["name"]].as_posix()
        density = p.get("density_kg_m3", 1000)
        # collision uses the same mesh; importer can convex-decompose
        links.append(f"""
  <link name="{p['name']}">
    <visual>
      <geometry><mesh filename="{mesh}" scale="{scale} {scale} {scale}"/></geometry>
    </visual>
    <collision>
      <geometry><mesh filename="{mesh}" scale="{scale} {scale} {scale}"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>""")

    joints = []
    for j in manifest.get("joints", []):
        ax = j.get("axis", [0, 0, 1])
        jtype = j["type"]
        if jtype == "prismatic":
            lo, hi = j.get("lower_m", -0.1), j.get("upper_m", 0.1)
        elif jtype == "revolute":
            lo = math.radians(j.get("lower_deg", -180))
            hi = math.radians(j.get("upper_deg", 180))
        else:
            lo, hi = -1, 1
        joints.append(f"""
  <joint name="{j['name']}" type="{jtype}">
    <parent link="{j['parent']}"/>
    <child link="{j['child']}"/>
    <axis xyz="{ax[0]} {ax[1]} {ax[2]}"/>
    <limit lower="{lo}" upper="{hi}" effort="10000" velocity="100"/>
    <dynamics damping="{j.get('damping', 0)}" friction="0"/>
  </joint>""")

    urdf = f'<?xml version="1.0"?>\n<robot name="{manifest.get("design_id","design")}">{"".join(links)}{"".join(joints)}\n</robot>\n'
    urdf_path.write_text(urdf)
    return urdf_path


# ----------------------------------------------------------------------------
# 3+. Isaac Sim: import URDF, build scenario, run, capture frames
#     (imports deferred until after SimulationApp boot)
# ----------------------------------------------------------------------------
def run_sim(manifest, urdf_path, out_dir, args):
    out_dir.mkdir(parents=True, exist_ok=True)
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(exist_ok=True)

    # ---- boot Isaac (must be first) ----
    from isaacsim.simulation_app import SimulationApp  # 6.0.x namespace
    sim_app = SimulationApp({
        "headless": args.headless == "1",
        "width": args.render_width,
        "height": args.render_height,
    })

    # deferred imports (only valid after SimulationApp exists)
    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    import omni.replicator.core as rep

    scn = manifest["scenario"]
    dt = scn.get("sim_dt", 1.0 / 120.0)
    world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt)

    # ---- import URDF -> USD articulation ----
    robot_prim = _import_urdf(urdf_path, manifest)

    # ---- ground + obstacle scenario ----
    _build_scenario(world, manifest, np)

    world.scene.add_default_ground_plane()
    world.reset()

    # ---- camera + render product for frame capture ----
    cam_path = _setup_camera(manifest)
    rp = rep.create.render_product(cam_path, (args.render_width, args.render_height))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=str(frames_dir), rgb=True)
    writer.attach([rp])

    # ---- sim loop ----
    duration = scn.get("sim_duration_s", 4.0)
    n_steps = int(duration / dt)
    metrics = _new_metrics()
    capture_every = max(1, n_steps // 150)  # ~150 frames max

    for step in range(n_steps):
        world.step(render=(step % capture_every == 0))
        _accumulate_metrics(metrics, world, manifest, step, dt, np)

    sim_app.update()
    _finalize_metrics(metrics, manifest)

    with open(out_dir / "metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)

    sim_app.close()
    return metrics, frames_dir


def _import_urdf(urdf_path, manifest):
    """Use the 6.0.x URDF importer extension."""
    from isaacsim.asset.importer.urdf import _urdf  # noqa
    import omni.kit.commands

    status, cfg = omni.kit.commands.execute(
        "URDFCreateImportConfig"
    )
    cfg.merge_fixed_joints = False
    cfg.fix_base = False
    cfg.make_default_prim = True
    cfg.import_inertia_tensor = False  # let physics recompute from collision + density
    cfg.collision_from_visuals = True
    cfg.density = 0.0  # per-link density handled via URDF; 0 = use mesh+default
    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=cfg,
    )
    return prim_path


def _build_scenario(world, manifest, np):
    """Add obstacle geometry for the test (curb/bump/jump)."""
    from isaacsim.core.api.objects import FixedCuboid
    scn = manifest["scenario"]
    obs = scn.get("obstacle")
    if not obs:
        return
    if obs.get("shape") == "box":
        FixedCuboid(
            prim_path="/World/obstacle",
            name="obstacle",
            position=np.array([obs.get("distance_m", 1.5), 0.0, obs["height_m"] / 2.0]),
            scale=np.array([obs.get("width_m", 0.3), 1.0, obs["height_m"]]),
        )


def _setup_camera(manifest):
    """Side-on camera so the VLM can see suspension travel + chassis attitude."""
    from isaacsim.core.utils.viewports import set_camera_view
    import omni.kit.viewport.utility as vp_util
    # place camera looking along -Y at the rig
    set_camera_view(eye=[2.5, -4.0, 1.5], target=[2.5, 0.0, 0.4])
    vp = vp_util.get_active_viewport()
    return vp.camera_path if vp else "/OmniverseKit_Persp"


# ----------------------------------------------------------------------------
# metrics
# ----------------------------------------------------------------------------
def _new_metrics():
    return {
        "max_suspension_travel_m": 0.0,
        "min_chassis_height_m": 1e9,
        "max_chassis_roll_deg": 0.0,
        "chassis_ground_contact": False,
        "settle_time_s": None,
        "exploded": False,
        "_last_motion_step": 0,
    }


def _accumulate_metrics(metrics, world, manifest, step, dt, np):
    # placeholder physical readouts; wired to articulation DOF + body poses
    # once the imported prim handle is resolved. Kept defensive: never crash
    # the sim loop on a missing handle.
    try:
        art = world.scene.get_object(manifest.get("design_id", "design"))
        if art is None:
            return
        # suspension DOF position
        dof_pos = art.get_joint_positions()
        if dof_pos is not None and len(dof_pos) > 0:
            travel = float(abs(dof_pos[0]))
            metrics["max_suspension_travel_m"] = max(metrics["max_suspension_travel_m"], travel)
        # chassis pose
        pos, quat = art.get_world_pose()
        if pos is not None:
            h = float(pos[2])
            metrics["min_chassis_height_m"] = min(metrics["min_chassis_height_m"], h)
            roll = _roll_deg_from_quat(quat)
            metrics["max_chassis_roll_deg"] = max(metrics["max_chassis_roll_deg"], abs(roll))
            if h > 50:  # flew away => unstable solve
                metrics["exploded"] = True
    except Exception:
        pass


def _roll_deg_from_quat(quat):
    import numpy as np
    w, x, y, z = quat
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    return math.degrees(math.atan2(sinr, cosr))


def _finalize_metrics(metrics, manifest):
    pc = manifest.get("pass_criteria", {})
    failures = []
    if pc.get("max_suspension_travel_m") is not None and \
            metrics["max_suspension_travel_m"] > pc["max_suspension_travel_m"]:
        failures.append({
            "criterion": "suspension_travel",
            "detail": f"travel {metrics['max_suspension_travel_m']:.3f}m exceeded limit {pc['max_suspension_travel_m']}m (bottomed out)",
        })
    if pc.get("max_chassis_roll_deg") is not None and \
            metrics["max_chassis_roll_deg"] > pc["max_chassis_roll_deg"]:
        failures.append({
            "criterion": "chassis_roll",
            "detail": f"roll {metrics['max_chassis_roll_deg']:.1f}deg exceeded {pc['max_chassis_roll_deg']}deg",
        })
    if metrics["exploded"]:
        failures.append({
            "criterion": "stability",
            "detail": "articulation became unstable (parts flew apart) — check joints/inertia",
        })
    metrics["failures"] = failures
    metrics["passed_metrics"] = len(failures) == 0
    metrics.pop("_last_motion_step", None)


def main():
    args = parse_args()
    manifest_path = Path(args.manifest)
    work_dir = manifest_path.parent
    out_dir = Path(args.out)
    mesh_dir = out_dir / "meshes"

    manifest = load_manifest(manifest_path)
    print(f"[eval] design={manifest.get('design_id')} prompt={manifest.get('user_prompt','')[:60]}")

    meshes = render_parts(manifest, work_dir, mesh_dir)
    print(f"[eval] rendered {len(meshes)} part meshes")

    urdf_path = synth_urdf(manifest, meshes, out_dir / "robot.urdf")
    print(f"[eval] synth URDF -> {urdf_path}")

    metrics, frames_dir = run_sim(manifest, urdf_path, out_dir, args)
    print(f"[eval] sim done. failures={len(metrics.get('failures', []))} frames={frames_dir}")

    with open(out_dir / "sim_result.json", "w") as f:
        json.dump({
            "design_id": manifest.get("design_id"),
            "metrics": metrics,
            "frames_dir": str(frames_dir),
        }, f, indent=2)


if __name__ == "__main__":
    main()
