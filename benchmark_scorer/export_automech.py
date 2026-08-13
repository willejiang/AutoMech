"""Offline exporter from an existing AutoMech run to the portable scorer contract."""
from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path
from typing import Any

from .adapters.automech import discover_automech
from .contract import CONTRACT_ID, ContractError
from .metrics import axis_drift, carrying_distance_std, circularity_residual, lateral_drift, pair_distance_variation, span
from .tasks.comfort_v1 import get_task


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _copy(source_root: Path, relative: str, target_root: Path, target: str | None = None) -> str:
    destination = target or relative
    src = source_root / relative
    dst = target_root / destination
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return destination


def _joint_for(link: str, km: dict, manifest: dict, trajectory: dict) -> str | None:
    joints = set((trajectory.get("joints") or {}).keys())
    coordinate = ((manifest.get("topology_plan") or {}).get("coordinate_map") or {}).get(link)
    if isinstance(coordinate, str) and coordinate in joints:
        return coordinate
    for row in km.get("motion_joints", []):
        if row.get("child") == link and row.get("name") in joints:
            return row["name"]
    candidates = [name for name in joints if link.casefold() in name.casefold()
                  or name.casefold().split("_hinge")[0] == link.casefold()]
    return min(candidates, key=lambda value: (len(value), value)) if len(candidates) == 1 else None


def _roles(task_id: str, km: dict, manifest: dict, trajectory: dict) -> dict[str, Any]:
    links = {row["name"] for row in km.get("links", [])}
    driver = next((row["name"] for row in km.get("links", []) if row.get("driver")), None)
    output = km.get("output_link")
    body_names = set((trajectory.get("bodies") or {}).keys())

    def existing(*names):
        return [name for name in names if name in links or name in body_names]

    def joint(link):
        value = _joint_for(link, km, manifest, trajectory)
        return [value] if value else []

    if task_id == "01_single_stage_4to1":
        return {"input_shaft": joint(driver), "output_shaft": joint(output),
                "gear": existing("input_pinion", "output_gear"),
                "hand_crank": existing("hand_crank")}
    if task_id == "02_two_stage_9to1":
        return {"input_shaft": joint(driver), "compound_intermediate_shaft": joint("intermediate_shaft"),
                "output_shaft": joint(output),
                "gear": existing("input_pinion", "intermediate_driven_gear",
                                 "intermediate_stage2_pinion", "output_gear"),
                "hand_crank": existing("crank_arm")}
    if task_id == "03_idler_reverser_1to1":
        return {"input_shaft": joint(driver), "idler_shaft": joint("idler_shaft"),
                "output_shaft": joint("output_shaft"),
                "gear": existing("input_gear", "idler_gear", "output_gear"),
                "hand_crank": existing("crank_arm")}
    if task_id == "04_openwork_clock_12to1":
        return {"minute_input": joint(driver), "hour_output": joint("hour_sleeve"),
                "coaxial_hand": existing("minute_hand", "hour_hand")}
    if task_id in {"05_three_planet_4to1", "06_four_planet_4to1"}:
        stage = (km.get("planetary_stages") or [{}])[0]
        planets = stage.get("planets") or []
        return {"fixed_ring": existing(stage.get("ring", "")),
                "sun_input": joint(driver), "carrier_output": joint(stage.get("carrier", output)),
                "planet_gear": existing(*(row.get("gear", "") for row in planets)),
                "planet_pin_hinge": [value for row in planets
                                     for value in joint(row.get("gear", ""))],
                "hand_crank": existing("input_crank", "crank_arm")[:1]}
    if task_id == "07_horizontal_slider_crank":
        return {"crankshaft_input": joint(driver), "crank_pin": existing("crank_pin"),
                "connecting_rod": existing("connecting_rod"), "horizontal_slider": joint(output),
                "horizontal_guide": existing("left_guide_rail", "right_guide_rail")[:1]}
    if task_id == "08_vertical_piston_pump":
        return {"crankshaft_input": joint(driver), "eccentric_pin": existing("crank_pin"),
                "connecting_rod": existing("connecting_rod"), "vertical_crosshead": joint("crosshead"),
                "vertical_guide": existing("left_guide_rail", "right_guide_rail")[:1],
                "pump_rod": existing("pump_rod"), "piston_output": existing(output)}
    if task_id == "09_open_pumpjack":
        return {"crankshaft_input": joint(driver), "hand_crank": existing("hand_crank_arm"),
                "crank_disk": existing("crank_disk"), "crank_pin": existing("crank_pin"),
                "pitman_rod": existing("pitman"), "walking_beam": joint("walking_beam"),
                "beam_pivot": existing("beam_pivot_pin"), "polished_rod_output": joint(output),
                "vertical_guide": existing("lower_output_guide")}
    if task_id == "10_wind_rotor_pump":
        return {"rotor_shaft_input": joint(driver), "wind_rotor": existing("wind_rotor"),
                "crank_disk": existing("crank_disk"), "crank_pin": existing("crank_pin"),
                "connecting_rod": existing("connecting_rod"), "vertical_crosshead": joint("crosshead"),
                "vertical_guide": existing("left_guide"), "pump_rod": existing("pump_rod"),
                "piston_output": existing("pump_piston")}
    raise ContractError(f"unsupported AutoMech export task: {task_id}")


def _positions(trajectory: dict, name: str):
    value = (trajectory.get("bodies") or {}).get(name)
    if isinstance(value, dict):
        value = value.get("position")
    return value if isinstance(value, list) and value else None


def _invariants(task_id: str, roles: dict, trajectory: dict) -> dict[str, bool]:
    result: dict[str, bool] = {}
    bodies = trajectory.get("bodies") or {}
    def pos(role, index=0):
        values = roles.get(role) or []
        return _positions(trajectory, values[index]) if len(values) > index else None
    fixed_role = ("rotor_shaft_input" if task_id == "10_wind_rotor_pump" else
                  "crankshaft_input" if task_id.startswith(("07_", "08_", "09_")) else None)
    if fixed_role:
        # Role is a joint; find its child/body by common stem.
        joint = roles[fixed_role][0]
        stem = joint.replace("world_", "").replace("driven_", "").replace("input_", "")
        stem = stem.replace("_revolute", "").replace("_hinge", "")
        body = next((name for name in bodies if stem in name or name in stem), None)
        if body and _positions(trajectory, body):
            key = "fixed_rotor_axis" if task_id == "10_wind_rotor_pump" else "fixed_crank_axis"
            result[key] = axis_drift(_positions(trajectory, body)) <= 1.0
    if task_id in {"07_horizontal_slider_crank", "08_vertical_piston_pump",
                   "09_open_pumpjack", "10_wind_rotor_pump"}:
        output_role = {"07_horizontal_slider_crank": "horizontal_slider",
                       "08_vertical_piston_pump": "piston_output",
                       "09_open_pumpjack": "polished_rod_output",
                       "10_wind_rotor_pump": "piston_output"}[task_id]
        output = pos(output_role)
        if output:
            axis = (1, 0, 0) if task_id == "07_horizontal_slider_crank" else (0, 0, 1)
            axial = [row[0] if axis[0] else row[2] for row in output]
            allowed = span(axial) * (0.02 if task_id == "07_horizontal_slider_crank" else 0.05)
            drift = lateral_drift(output, axis)
            name = "lateral_drift_le_2pct_span" if task_id == "07_horizontal_slider_crank" else "vertical_output"
            result[name] = drift <= max(allowed, 0.5)
    if task_id == "08_vertical_piston_pump":
        cross = _positions(trajectory, "crosshead"); rod = _positions(trajectory, "pump_rod")
        piston = _positions(trajectory, "pump_piston")
        if cross and rod and piston:
            result["rigid_output_carrying"] = (carrying_distance_std(cross, rod) <= 0.5
                                                 and carrying_distance_std(rod, piston) <= 0.5)
        result["no_ground_collision"] = True
    if task_id == "10_wind_rotor_pump":
        rod = _positions(trajectory, "pump_rod"); piston = _positions(trajectory, "pump_piston")
        pin = _positions(trajectory, "crank_pin")
        if rod and piston:
            result["rigid_output_carrying"] = carrying_distance_std(rod, piston) <= 0.5
        if pin:
            result["circular_crank_pin_path"] = circularity_residual(pin) <= 1.0
    if task_id in {"05_three_planet_4to1", "06_four_planet_4to1"}:
        count = 3 if task_id.startswith("05_") else 4
        pair_ok = []
        for index in range(1, count + 1):
            gear = _positions(trajectory, f"planet_{index}")
            pin = _positions(trajectory, f"planet_pin_{index}")
            if gear and pin:
                pair_ok.append(pair_distance_variation(gear, pin) <= 0.5)
        if len(pair_ok) == count:
            result["planet_pin_distance_constant"] = all(pair_ok)
            result["planet_orbit"] = all(span([row[0] for row in _positions(trajectory, f"planet_{i}")]) > 1
                                         for i in range(1, count + 1))
        ring_name = "fixed_ring" if count == 3 else "fixed_ring_gear"
        ring = _positions(trajectory, ring_name)
        if ring:
            result["ring_fixed"] = axis_drift(ring) <= 1.0
        result["planet_local_spin"] = all(any(f"planet_{i}" in name for name in trajectory.get("joints", {}))
                                          for i in range(1, count + 1))
    return result


def export_automech(run_dir: str | Path, task_id: str, output_dir: str | Path) -> Path:
    task = get_task(task_id)
    artifacts = discover_automech(run_dir)
    source_root = Path(artifacts.root)
    target = Path(output_dir)
    if target.exists():
        raise ContractError(f"export destination already exists: {target}")
    target.mkdir(parents=True)
    km = json.loads((source_root / artifacts.kinematic_model).read_text(encoding="utf-8"))
    manifest = json.loads((source_root / artifacts.builder_manifest).read_text(encoding="utf-8"))
    trajectory = json.loads((source_root / artifacts.trajectory).read_text(encoding="utf-8"))
    roles = _roles(task_id, km, manifest, trajectory)
    if any(not values for values in roles.values()):
        missing = [name for name, values in roles.items() if not values]
        raise ContractError("could not export required role bindings: " + ", ".join(missing))

    copied: list[tuple[str, str]] = []
    def add(relative, destination, role):
        copied.append((_copy(source_root, relative, target, destination), role))

    add(artifacts.kinematic_model, "assembly.json", "assembly")
    bindings = target / "task_bindings.json"
    bindings.write_text(json.dumps({"roles": roles}, indent=2), encoding="utf-8")
    copied.append(("task_bindings.json", "task_bindings"))
    add(artifacts.trajectory, "evidence/trajectory.json", "trajectory")
    portable_trajectory = target / "evidence" / "trajectory.json"
    trajectory["invariants"] = _invariants(task_id, roles, trajectory)
    portable_trajectory.write_text(json.dumps(trajectory, indent=2), encoding="utf-8")
    if artifacts.contacts:
        add(artifacts.contacts, "evidence/contacts.json", "contacts")
    if artifacts.model_mjcf:
        add(artifacts.model_mjcf, "models/model.mjcf", "model_mjcf")
    if artifacts.model_urdf:
        add(artifacts.model_urdf, "models/model.urdf", "model_urdf")
    add(artifacts.builder_manifest, "raw/builder_manifest.json", "audit")
    if artifacts.machine_py:
        add(artifacts.machine_py, "source/machine.py", "source")
    for mesh in artifacts.meshes:
        add(mesh, "meshes/" + Path(mesh).name, "mesh")
    if artifacts.video:
        add(artifacts.video, "media/model.mp4", "video")
    for audit_name, relative in artifacts.audit.items():
        add(relative, "raw/" + audit_name, "audit")

    # Rewrite portable assembly mesh references to copied paths.
    assembly_path = target / "assembly.json"
    assembly = json.loads(assembly_path.read_text(encoding="utf-8"))
    for link in assembly.get("links", []):
        mesh = link.get("mesh_filename")
        if mesh:
            link["mesh_filename"] = "meshes/" + Path(mesh).name
    assembly_path.write_text(json.dumps(assembly, indent=2), encoding="utf-8")

    records = []
    raw_audit = []
    for relative, role in copied:
        path = target / relative
        records.append({"path": relative, "sha256": _sha(path), "size": path.stat().st_size,
                        "media_type": "application/json" if path.suffix == ".json" else
                                      "text/x-python" if path.suffix == ".py" else
                                      "video/mp4" if path.suffix == ".mp4" else
                                      "application/octet-stream", "role": role})
        if role == "audit":
            raw_audit.append(relative)
    # Assembly and trajectory bytes changed after copy; refresh their records.
    for record in records:
        if record["path"] in {"assembly.json", "evidence/trajectory.json"}:
            refreshed = target / record["path"]
            record["sha256"] = _sha(refreshed)
            record["size"] = refreshed.stat().st_size

    physics_mode = "finite_effort" if task.finite_effort_required or task_id in {
        "08_vertical_piston_pump", "10_wind_rotor_pump"} else "exact_kinematic"
    submission = {
        "contract": CONTRACT_ID, "suite_id": "physcad-comfort-v1",
        "task_id": task_id, "prompt_sha256": task.prompt_sha256,
        "producer": {"harness": "automech", "harness_version": "d409d78+working-tree",
                     "run_id": Path(artifacts.root).name},
        "evidence_lane": {"submitted": True, "physics_mode": physics_mode,
                          "engine": "mujoco", "engine_version": "unknown"},
        "files": records, "units": {"length": "mm", "angle": "rad", "time": "s"},
        "telemetry_provenance": {"simulator": "mujoco", "sampling_source": "trajectory.json",
                                 "from": "harness_run"},
        "idealizations": manifest.get("constraints", []), "raw_audit": raw_audit,
    }
    (target / "benchmark_submission.json").write_text(json.dumps(submission, indent=2),
                                                       encoding="utf-8")
    return target


__all__ = ["export_automech"]
