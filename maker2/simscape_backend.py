#!/usr/bin/env python3
"""Simscape bridge backend for maker2.

This backend does three concrete jobs:

- export a rich `simscape_bundle.json` with deterministic mechanism semantics,
  world/local port frames, geometry paths, and estimated body properties
- generate `build_simscape_model.m`, a MATLAB entrypoint that reads the bundle and
  optionally hands off to a user-supplied `maker2_simscape_run(...)`
- run a configurable external backend (MATLAB by default when available), then
  ingest `sim_result.json` / frames / videos back into the existing maker2 result
  contract

So the bridge is no longer only a one-way scaffold export: if a real runner writes
`sim_result.json`, maker2 now consumes it exactly like the other physics backends.
"""
from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
from pathlib import Path


def _simscape_dir(run_dir: str, iteration: int | None = None) -> Path:
    tag = "simscape" if iteration is None else f"simscape_{iteration}"
    out = Path(run_dir) / "physics" / tag
    out.mkdir(parents=True, exist_ok=True)
    return out


def _ident_pose() -> dict:
    return {"xyz_m": [0.0, 0.0, 0.0],
            "rpy_rad": [0.0, 0.0, 0.0],
            "R": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]}


def _v3(x) -> list[float]:
    if not isinstance(x, (list, tuple)) or len(x) != 3:
        return [0.0, 0.0, 0.0]
    return [float(x[0]), float(x[1]), float(x[2])]


def _round_vec(x, ndigits: int = 6) -> list[float]:
    return [round(float(v), ndigits) for v in x]


def _round_mat(R, ndigits: int = 6) -> list[list[float]]:
    return [[round(float(v), ndigits) for v in row] for row in R]


def _matmul3(A, B) -> list[list[float]]:
    out = []
    for r in range(3):
        out.append([
            A[r][0] * B[0][c] + A[r][1] * B[1][c] + A[r][2] * B[2][c]
            for c in range(3)
        ])
    return out


def _matvec3(A, v) -> list[float]:
    v = _v3(v)
    return [
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    ]


def _normalize(v) -> list[float]:
    v = _v3(v)
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n <= 1e-12:
        return [0.0, 0.0, 1.0]
    return [v[0] / n, v[1] / n, v[2] / n]


def _add3(a, b) -> list[float]:
    a = _v3(a); b = _v3(b)
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _rpy_matrix(rpy) -> list[list[float]]:
    roll, pitch, yaw = _v3(rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]
    Ry = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
    Rz = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
    return _matmul3(_matmul3(Rz, Ry), Rx)


def _world_poses(model) -> dict[str, dict]:
    by_child = {p.child: p for p in model.poses}
    cache: dict[str, dict] = {}

    def solve(link_name: str) -> dict:
        if link_name in cache:
            return cache[link_name]
        pose = by_child.get(link_name)
        if pose is None or not pose.parent:
            cache[link_name] = _ident_pose()
            return cache[link_name]
        parent = solve(pose.parent)
        R_rel = _rpy_matrix(pose.rpy_rad)
        xyz_rel = _v3(pose.xyz_m)
        world = {
            "xyz_m": _add3(parent["xyz_m"], _matvec3(parent["R"], xyz_rel)),
            "rpy_rad": _v3(pose.rpy_rad),
            "R": _matmul3(parent["R"], R_rel),
        }
        cache[link_name] = world
        return world

    for link in model.links:
        solve(link.name)
    return cache


def _mesh_metrics(mesh_abs: str, material: str) -> dict:
    info = {
        "exists": bool(mesh_abs and Path(mesh_abs).exists()),
        "bbox_local_mm": None,
        "volume_mm3": None,
        "mass_kg": None,
        "inertia_diag_kg_m2": None,
    }
    if not info["exists"]:
        return info
    try:
        import trimesh
        from .materials import density_of

        mesh = trimesh.load_mesh(mesh_abs, force="mesh")
        if getattr(mesh, "is_empty", False):
            return info
        bounds = getattr(mesh, "bounds", None)
        if bounds is not None and len(bounds) == 2:
            lo = [float(v) for v in bounds[0]]
            hi = [float(v) for v in bounds[1]]
            info["bbox_local_mm"] = [round(v, 6) for v in (lo + hi)]
            ext_m = [(hi[i] - lo[i]) * 1e-3 for i in range(3)]
        else:
            ext_m = [0.0, 0.0, 0.0]
        vol_mm3 = float(getattr(mesh, "volume", 0.0) or 0.0)
        info["volume_mm3"] = round(vol_mm3, 6) if vol_mm3 > 0 else None
        if vol_mm3 > 0:
            mass = max(density_of(material) * vol_mm3 * 1e-9, 1e-9)
            info["mass_kg"] = round(mass, 9)
            x, y, z = ext_m
            info["inertia_diag_kg_m2"] = [
                round(mass * (y * y + z * z) / 12.0, 12),
                round(mass * (x * x + z * z) / 12.0, 12),
                round(mass * (x * x + y * y) / 12.0, 12),
            ]
    except Exception:
        return info
    return info


def _body_records(model, run_dir: str, world_poses: dict[str, dict]) -> list[dict]:
    out = []
    for link in model.links:
        local = model.pose_to_child(link.name)
        world = world_poses.get(link.name, _ident_pose())
        mesh_abs = ""
        if link.mesh_filename:
            mesh_abs = str((Path(run_dir) / link.mesh_filename).resolve())
        carriers = [c for c in ([getattr(link, "mount", "")] +
                                list(getattr(link, "extra_mounts", []) or [])) if c]
        geom = {"mesh_stl": mesh_abs}
        geom.update(_mesh_metrics(mesh_abs, getattr(link, "material", "steel")))
        out.append({
            "name": link.name,
            "description": link.description,
            "shape_hint": link.shape_hint,
            "size_mm": dict(link.size_mm),
            "origin_note": link.origin_note,
            "dof": link.dof,
            "spin_axis": _round_vec(getattr(link, "spin_axis", (0.0, 0.0, 1.0))),
            "slide_axis": _round_vec(getattr(link, "slide_axis", (1.0, 0.0, 0.0))),
            "driver": bool(getattr(link, "driver", False)),
            "material": getattr(link, "material", "steel"),
            "mount": getattr(link, "mount", ""),
            "extra_mounts": list(getattr(link, "extra_mounts", []) or []),
            "carriers": carriers,
            "pose": {
                "parent": local.parent if local is not None else "",
                "xyz_m": _round_vec(local.xyz_m if local is not None else (0.0, 0.0, 0.0)),
                "rpy_rad": _round_vec(local.rpy_rad if local is not None else (0.0, 0.0, 0.0)),
            },
            "world_pose": {
                "xyz_m": _round_vec(world["xyz_m"]),
                "R": _round_mat(world["R"]),
            },
            "geometry": geom,
        })
    return out


def _joint_records(model, world_poses: dict[str, dict]) -> list[dict]:
    out = []
    for j in model.joints:
        parent_world = world_poses.get(j.parent, _ident_pose())
        joint_world_xyz = _add3(parent_world["xyz_m"], _matvec3(parent_world["R"], j.xyz_m))
        joint_world_axis = _normalize(_matvec3(parent_world["R"], j.axis))
        unit = "m" if j.type == "prismatic" else ("rad" if j.type != "fixed" else None)
        out.append({
            "name": j.name,
            "type": j.type,
            "parent": j.parent,
            "child": j.child,
            "xyz_m": _round_vec(j.xyz_m),
            "rpy_rad": _round_vec(j.rpy_rad),
            "axis": _round_vec(j.axis),
            "world_origin_m": _round_vec(joint_world_xyz),
            "world_axis": _round_vec(joint_world_axis),
            "driver": bool(getattr(j, "driver", False)),
            "unit": unit,
        })
    return out


def _port_records(model, world_poses: dict[str, dict]) -> tuple[dict, dict]:
    out: dict[str, list[dict]] = {}
    index: dict[tuple[str, str], dict] = {}
    for link, ports in (getattr(model, "ports_by_link", None) or {}).items():
        pose = world_poses.get(link, _ident_pose())
        rows = []
        for p in ports:
            local_xyz_m = [float(v) * 1e-3 for v in p.xyz_mm]
            local_axis = _normalize(p.axis)
            world_xyz_m = _add3(pose["xyz_m"], _matvec3(pose["R"], local_xyz_m))
            world_axis = _normalize(_matvec3(pose["R"], local_axis))
            rec = {
                "name": p.name,
                "type": p.type,
                "xyz_mm": _round_vec(p.xyz_mm),
                "xyz_m": _round_vec(local_xyz_m),
                "axis": _round_vec(local_axis),
                "world_xyz_m": _round_vec(world_xyz_m),
                "world_axis": _round_vec(world_axis),
                "diameter_mm": float(p.diameter_mm),
                "depth_mm": float(p.depth_mm),
                "pitch_radius_mm": float(p.pitch_radius_mm),
                "normal_sign": float(p.normal_sign),
            }
            rows.append(rec)
            index[(link, p.name)] = rec
        out[link] = rows
    return out, index


def _relation_class(mate_type: str) -> str:
    mt = (mate_type or "").strip().lower()
    if mt in {"revolute", "pin", "journal_bearing", "bearing"}:
        return "revolute"
    if mt in {"cylindrical"}:
        return "cylindrical"
    if mt in {"coaxial_face", "face_to_face", "welded", "bolted", "press_fit"}:
        return "fixed"
    if mt.startswith("gear") or mt == "worm":
        return "gear"
    if mt in {"coaxial"}:
        return "coaxial"
    return mt or "unknown"


def _relation_records(model, port_index: dict) -> list[dict]:
    rels = []
    for r in getattr(model, "relations", []) or []:
        base_frame = port_index.get((r.base_part, r.base_port))
        incoming_frame = port_index.get((r.incoming_part, r.incoming_port))
        rels.append({
            "name": r.name,
            "mate_type": r.mate_type,
            "constraint_class": _relation_class(r.mate_type),
            "base_part": r.base_part,
            "base_port": r.base_port,
            "incoming_part": r.incoming_part,
            "incoming_port": r.incoming_port,
            "offset_mm": float(r.offset_mm),
            "angle_rad": float(r.angle_rad),
            "flip": bool(r.flip),
            "axis_angle_deg": float(r.axis_angle_deg),
            "separation_axis": _round_vec(r.separation_axis) if r.separation_axis else [],
            "offset_e_mm": float(r.offset_e_mm),
            "base_frame": base_frame,
            "incoming_frame": incoming_frame,
        })
    return rels


def _mesh_pair_records(model, port_index: dict) -> list[dict]:
    out = []
    for a, b in (getattr(model, "mesh_pairs", None) or []):
        a_ports = [p for (ln, _), p in port_index.items()
                   if ln == a and p.get("type") == "gear_mesh"]
        b_ports = [p for (ln, _), p in port_index.items()
                   if ln == b and p.get("type") == "gear_mesh"]
        out.append({
            "driver_link": a,
            "driven_link": b,
            "driver_pitch_radius_mm": a_ports[0].get("pitch_radius_mm") if a_ports else None,
            "driven_pitch_radius_mm": b_ports[0].get("pitch_radius_mm") if b_ports else None,
        })
    return out


def _expected_outputs(out_dir: Path) -> dict:
    return {
        "sim_result_json": str((out_dir / "sim_result.json").resolve()),
        "trajectory_json": str((out_dir / "trajectory.json").resolve()),
        "frames_dir": str((out_dir / "frames").resolve()),
        "video_mp4": str((out_dir / "model.mp4").resolve()),
    }


def _write_readme(out_dir: Path, bundle_path: str, matlab_script: str) -> str:
    readme = out_dir / "README.txt"
    readme.write_text(
        "maker2 Simscape bridge\n"
        "======================\n\n"
        "Inputs:\n"
        f"- bundle: {bundle_path}\n"
        f"- matlab entrypoint: {matlab_script}\n\n"
        "Runtime contract:\n"
        "- A real runner should write sim_result.json in this directory.\n"
        "- Optional: trajectory.json and frames/rgb_*.png.\n"
        "- If frames exist, maker2 will encode model.mp4 automatically.\n\n"
        "Execution options:\n"
        "- Set SIMSCAPE_RUNNER_JSON to a JSON argv array for an external runner.\n"
        "- Else, if MATLAB is installed, maker2 runs build_simscape_model.m.\n"
        "- The generated MATLAB entrypoint calls maker2_simscape_run(bundlePath,outDir)\n"
        "  automatically when that function exists on the MATLAB path.\n",
        encoding="utf-8",
    )
    return str(readme)


def _generate_matlab_script(bundle_path: Path, out_dir: Path) -> str:
    return f"""function build_simscape_model(bundlePath, outDir)
% Auto-generated by maker2.simscape_backend.
% Reads the exported Simscape bridge bundle, writes a bundle report, and when a
% project-specific runner exists on the MATLAB path hands off to it.

if nargin < 1 || strlength(string(bundlePath)) == 0
    bundlePath = '{bundle_path.as_posix()}';
end
if nargin < 2 || strlength(string(outDir)) == 0
    outDir = '{out_dir.as_posix()}';
end
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

bundle = jsondecode(fileread(bundlePath));
report = make_bundle_report(bundle);
write_json(fullfile(outDir, 'simscape_report.json'), report);

if exist('maker2_simscape_run', 'file') == 2
    disp('[simscape] calling maker2_simscape_run(bundlePath, outDir)');
    maker2_simscape_run(bundlePath, outDir);
    return;
end

write_export_only_result(bundle, outDir, report);
end

function report = make_bundle_report(bundle)
report = struct();
report.backend = 'simscape';
report.task = bundle.task;
report.name = bundle.name;
report.root_link = bundle.root_link;
report.body_count = numel(bundle.mechanism.bodies);
report.joint_count = numel(bundle.mechanism.joints);
report.relation_count = numel(bundle.mechanism.relations);
report.mesh_pair_count = numel(bundle.mechanism.mesh_pairs);
report.watch_links = bundle.mechanism.watch_links;
report.output_link = bundle.mechanism.output_link;
report.driver_joint = '';
report.driver_link = '';
for i = 1:numel(bundle.mechanism.joints)
    if isfield(bundle.mechanism.joints(i), 'driver') && bundle.mechanism.joints(i).driver
        report.driver_joint = bundle.mechanism.joints(i).name;
        break;
    end
end
for i = 1:numel(bundle.mechanism.bodies)
    if isfield(bundle.mechanism.bodies(i), 'driver') && bundle.mechanism.bodies(i).driver
        report.driver_link = bundle.mechanism.bodies(i).name;
        break;
    end
end
end

function write_export_only_result(bundle, outDir, report)
metrics = struct();
metrics.verdict = 'FAIL';
metrics.test_kind = 'simscape_bridge';
metrics.input_joint = report.driver_joint;
metrics.input_part = report.driver_link;
metrics.input_travel = 0.0;
metrics.input_unit = '';
metrics.moved_count = 0;
metrics.watched_count = numel(report.watch_links);
metrics.output_reached = false;
metrics.output_travel = [];
metrics.output_unit = '';
metrics.ratio_in_out = [];
metrics.exploded = false;
metrics.end_z = 0.0;
metrics.max_tilt_deg = 0.0;
metrics.max_drift = 0.0;
metrics.body_count = report.body_count;
metrics.joint_count = report.joint_count;
metrics.relation_count = report.relation_count;

res = struct();
res.task = bundle.task;
res.spec = struct('backend', 'simscape', ...
                  'bundle', bundle.geometry, ...
                  'bridge_report', fullfile(outDir, 'simscape_report.json'));
res.metrics = metrics;
res.frames_dir = '';
res.n_frames = 0;
res.log = {{'Simscape bridge exported.', ...
            'No maker2_simscape_run(bundlePath, outDir) function was found on the MATLAB path.'}};
write_json(fullfile(outDir, 'sim_result.json'), res);
end

function write_json(path, payload)
fid = fopen(path, 'w');
fwrite(fid, jsonencode(payload), 'char');
fclose(fid);
end
"""


def write_bundle(model, run_dir: str, out_dir: str, task: str) -> tuple[str, str, str]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    world_poses = _world_poses(model)
    ports_by_link, port_index = _port_records(model, world_poses)
    bodies = _body_records(model, run_dir, world_poses)
    joints = _joint_records(model, world_poses)
    relations = _relation_records(model, port_index)
    mesh_pairs = _mesh_pair_records(model, port_index)
    step_path = Path(run_dir) / "machine.step"
    bundle = {
        "version": 2,
        "backend": "simscape",
        "task": task,
        "name": model.name,
        "root_link": model.root_link,
        "geometry": {
            "step": str(step_path.resolve()) if step_path.exists() else "",
            "mesh_dir": str((Path(run_dir) / "meshes").resolve()),
        },
        "expected_outputs": _expected_outputs(out),
        "mechanism": {
            "output_link": getattr(model, "output_link", "") or "",
            "watch_links": list(getattr(model, "watch_links", []) or []),
            "mesh_pairs": mesh_pairs,
            "bodies": bodies,
            "joints": joints,
            "relations": relations,
            "ports_by_link": ports_by_link,
            "summary": {
                "body_count": len(bodies),
                "joint_count": len(joints),
                "relation_count": len(relations),
                "mesh_pair_count": len(mesh_pairs),
            },
        },
    }
    bundle_path = out / "simscape_bundle.json"
    bundle_path.write_text(json.dumps(bundle, indent=2), encoding="utf-8")

    matlab_script = out / "build_simscape_model.m"
    matlab_script.write_text(_generate_matlab_script(bundle_path, out), encoding="utf-8")
    readme = _write_readme(out, str(bundle_path), str(matlab_script))
    return str(bundle_path), str(matlab_script), readme


def _expand_runner_value(value: str, subs: dict[str, str]) -> str:
    text = str(value)
    for k, v in subs.items():
        text = text.replace("{" + k + "}", v)
    return text


def _runner_cmd(bundle_path: str, out_dir: Path, matlab_script: str, task: str):
    subs = {
        "bundle": bundle_path,
        "out_dir": str(out_dir),
        "script": matlab_script,
        "task": task,
        "result": str((out_dir / "sim_result.json").resolve()),
    }
    raw_json = os.environ.get("SIMSCAPE_RUNNER_JSON", "").strip()
    if raw_json:
        try:
            argv = json.loads(raw_json)
            if isinstance(argv, list) and argv:
                return [_expand_runner_value(str(x), subs) for x in argv]
        except Exception:
            pass
    raw_cmd = os.environ.get("SIMSCAPE_RUNNER_CMD", "").strip()
    if raw_cmd:
        return _expand_runner_value(raw_cmd, subs)
    matlab = (os.environ.get("SIMSCAPE_MATLAB_BIN") or
              os.environ.get("MATLAB_BIN") or
              shutil.which("matlab"))
    if matlab:
        batch = (
            f"addpath('{out_dir.as_posix()}'); "
            f"build_simscape_model('{Path(bundle_path).as_posix()}', '{out_dir.as_posix()}');"
        )
        return [matlab, "-batch", batch]
    return None


def _run_external(cmd, out_dir: Path, log_fn=print) -> tuple[bool, str, str]:
    if not cmd:
        return False, "", ""
    shell = isinstance(cmd, str)
    log_fn(f"[simscape] running backend: {cmd if shell else ' '.join(cmd)}")
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=1800,
                              cwd=str(out_dir), shell=shell)
        stdout = proc.stdout or ""
        stderr = proc.stderr or ""
        (out_dir / "runner_stdout.txt").write_text(stdout, encoding="utf-8")
        (out_dir / "runner_stderr.txt").write_text(stderr, encoding="utf-8")
        log_fn(f"[simscape] backend rc={proc.returncode}")
        return proc.returncode == 0, stdout, stderr
    except Exception as e:
        msg = f"{type(e).__name__}: {e}"
        (out_dir / "runner_stderr.txt").write_text(msg, encoding="utf-8")
        log_fn(f"[simscape] backend invocation failed: {msg}")
        return False, "", msg


def _fallback_result(model, task: str, run_dir: str, out_dir: Path, *, summary: str,
                     reason: str, bundle_path: str, matlab_script: str,
                     readme: str) -> dict:
    metrics = {
        "verdict": "FAIL",
        "test_kind": "simscape_bridge",
        "input_joint": next((j.name for j in model.joints if getattr(j, "driver", False)), None),
        "input_part": next((l.name for l in model.links if getattr(l, "driver", False)), None),
        "input_travel": 0.0,
        "input_unit": None,
        "moved_count": 0,
        "watched_count": len(getattr(model, "watch_links", []) or []),
        "output_reached": False,
        "output_travel": None,
        "output_unit": None,
        "ratio_in_out": None,
        "exploded": False,
        "end_z": 0.0,
        "max_tilt_deg": 0.0,
        "max_drift": 0.0,
        "body_count": len(getattr(model, "links", []) or []),
        "joint_count": len(getattr(model, "joints", []) or []),
        "relation_count": len(getattr(model, "relations", []) or []),
    }
    sim_result = {
        "task": task,
        "spec": {
            "run_dir": run_dir,
            "backend": "simscape",
            "bundle": bundle_path,
            "matlab_script": matlab_script,
            "readme": readme,
        },
        "metrics": metrics,
        "frames_dir": None,
        "n_frames": 0,
        "log": [summary, reason],
    }
    (out_dir / "sim_result.json").write_text(json.dumps(sim_result, indent=2),
                                               encoding="utf-8")
    return {
        "passed": False,
        "verdict": "FAIL",
        "summary": summary,
        "metrics": metrics,
        "frames_dir": None,
        "video": None,
        "tests": [{
            "name": "simscape_bridge",
            "verdict": "FAIL",
            "summary": summary,
            "metrics": metrics,
            "frames_dir": None,
            "video": None,
            "design": {
                "bundle": bundle_path,
                "matlab_script": matlab_script,
                "readme": readme,
            },
            "cause": "backend",
            "reason": reason,
        }],
        "cause": "backend",
        "reason": reason,
    }


def _resolve_path(value, out_dir: Path):
    if not value:
        return None
    p = Path(str(value))
    if p.is_absolute():
        return str(p)
    return str((out_dir / p).resolve())


def _normalize_result(raw: dict, out_dir: Path, bundle_path: str, matlab_script: str,
                      readme: str) -> dict:
    from diagnose import encode_mp4

    raw = dict(raw or {})
    raw.setdefault("spec", {})
    raw["spec"].setdefault("backend", "simscape")
    raw["spec"].setdefault("bundle", bundle_path)
    raw["spec"].setdefault("matlab_script", matlab_script)
    raw["spec"].setdefault("readme", readme)

    metrics = dict(raw.get("metrics") or {})
    verdict = str(metrics.get("verdict") or raw.get("verdict") or "FAIL").upper()
    passed = raw.get("passed")
    if passed is None:
        passed = verdict == "PASS"
    summary = (raw.get("summary") or metrics.get("summary") or
               f"Simscape backend returned {verdict}.")
    frames_dir = _resolve_path(raw.get("frames_dir"), out_dir)
    video = raw.get("video")
    if not video and frames_dir and Path(frames_dir).exists():
        mp4 = encode_mp4(frames_dir, os.path.join(out_dir, "model.mp4"))
        if mp4:
            video = str(Path(mp4))
    tests = raw.get("tests") or [{
        "name": metrics.get("test_kind", "simscape"),
        "verdict": verdict,
        "summary": summary,
        "metrics": metrics,
        "frames_dir": frames_dir,
        "video": video,
    }]
    norm_tests = []
    for t in tests:
        row = dict(t or {})
        row.setdefault("name", metrics.get("test_kind", "simscape"))
        row.setdefault("verdict", verdict)
        row.setdefault("summary", summary)
        row.setdefault("metrics", metrics)
        row["frames_dir"] = _resolve_path(row.get("frames_dir"), out_dir)
        if not row.get("video") and row["frames_dir"] and Path(row["frames_dir"]).exists():
            mp4 = encode_mp4(row["frames_dir"], os.path.join(out_dir, f"{row['name']}.mp4"))
            if mp4:
                row["video"] = str(Path(mp4))
        row.setdefault("design", {})
        row["design"].setdefault("bundle", bundle_path)
        row["design"].setdefault("matlab_script", matlab_script)
        row["design"].setdefault("readme", readme)
        row.setdefault("cause", raw.get("cause", "none" if passed else "scenario"))
        row.setdefault("reason", raw.get("reason", summary))
        norm_tests.append(row)

    sim_result = dict(raw)
    sim_result["passed"] = bool(passed)
    sim_result["verdict"] = verdict
    sim_result["summary"] = summary
    sim_result["metrics"] = metrics
    sim_result["frames_dir"] = frames_dir
    sim_result["video"] = video
    sim_result["tests"] = norm_tests
    sim_result["cause"] = raw.get("cause", "none" if passed else "scenario")
    sim_result["reason"] = raw.get("reason", summary)
    (out_dir / "sim_result.json").write_text(json.dumps(sim_result, indent=2),
                                               encoding="utf-8")

    return {
        "passed": bool(passed),
        "verdict": verdict,
        "summary": summary,
        "metrics": metrics,
        "frames_dir": frames_dir,
        "video": video,
        "tests": norm_tests,
        "cause": sim_result["cause"],
        "reason": sim_result["reason"],
    }


def run_simscape_backend(model, urdf_path: str, task: str, run_dir: str,
                         settings=None, iteration: int | None = None,
                         log_fn=print) -> dict:
    out = _simscape_dir(run_dir, iteration)
    bundle_path, matlab_script, readme = write_bundle(model, run_dir, out, task)
    result_path = out / "sim_result.json"

    cmd = _runner_cmd(bundle_path, out, matlab_script, task)
    if cmd is None:
        return _fallback_result(
            model, task, run_dir, out,
            summary=("Simscape bridge exported, but no external runner is available in "
                     "this environment."),
            reason=("Configure SIMSCAPE_RUNNER_JSON / SIMSCAPE_RUNNER_CMD, or install "
                    "MATLAB so maker2 can execute build_simscape_model.m."),
            bundle_path=bundle_path,
            matlab_script=matlab_script,
            readme=readme,
        )

    ok, stdout, stderr = _run_external(cmd, out, log_fn=log_fn)
    if result_path.exists():
        try:
            raw = json.loads(result_path.read_text(encoding="utf-8"))
            return _normalize_result(raw, out, bundle_path, matlab_script, readme)
        except Exception as e:
            return _fallback_result(
                model, task, run_dir, out,
                summary="Simscape runner wrote an unreadable sim_result.json.",
                reason=f"{type(e).__name__}: {e}",
                bundle_path=bundle_path,
                matlab_script=matlab_script,
                readme=readme,
            )

    tail = ((stderr or stdout or "").strip()[-1200:] or "(no backend output)")
    why = "Simscape runner failed before producing sim_result.json."
    if ok:
        why = "Simscape runner returned success but did not produce sim_result.json."
    return _fallback_result(
        model, task, run_dir, out,
        summary=why,
        reason=tail,
        bundle_path=bundle_path,
        matlab_script=matlab_script,
        readme=readme,
    )
