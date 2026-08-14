"""Build scorer-owned evidence from an archived portable submission without generation."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

from .contract import ContractError, ResourceLimits
from .geometry import analyze_geometry
from .replay import replay_model
from .tasks.comfort_v1 import axis_drift_limit_mm, get_task

ARCHIVE_EVIDENCE_VERSION = "comfort-archive-evidence/1.0"


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ContractError(f"expected JSON object: {path}")
    return value


def _joint_series(trajectory: Mapping[str, Any], joint: str) -> list[float] | None:
    raw = (trajectory.get("joints") or {}).get(joint)
    if isinstance(raw, Mapping):
        raw = raw.get("qpos", raw.get("position"))
    if not isinstance(raw, list) or not raw:
        return None
    try:
        return [float(row[0] if isinstance(row, list) else row) for row in raw]
    except (TypeError, ValueError, IndexError):
        return None


def _positions(trajectory: Mapping[str, Any], body: str) -> list[list[float]] | None:
    raw = (trajectory.get("bodies") or {}).get(body)
    if isinstance(raw, Mapping):
        raw = raw.get("position")
    if not isinstance(raw, list) or not raw:
        return None
    try:
        return [[float(value) for value in row[:3]] for row in raw]
    except (TypeError, ValueError, IndexError):
        return None


def _extent(positions: list[list[float]]) -> float:
    return max(max(row[axis] for row in positions) - min(row[axis] for row in positions)
               for axis in range(3))


def _distance_std(first: list[list[float]], second: list[list[float]]) -> float:
    values = [math.dist(a, b) for a, b in zip(first, second)]
    mean = sum(values) / len(values)
    return math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))


def _manifest_sources(root: Path) -> dict[str, Any]:
    path = root / "raw" / "builder_manifest.json"
    return _read(path) if path.is_file() else {}


def _derived_invariants(task_id: str, assembly: Mapping[str, Any], manifest: Mapping[str, Any],
                        trajectory: Mapping[str, Any], bindings: Mapping[str, Any],
                        replay_trajectory: Mapping[str, Any] | None = None) -> dict[str, bool]:
    task = get_task(task_id)
    result = dict(trajectory.get("invariants") or {})
    links = {str(row.get("name")): row for row in assembly.get("links", ()) if isinstance(row, Mapping)}
    topology = manifest.get("topology_plan") if isinstance(manifest.get("topology_plan"), Mapping) else {}
    coordinate = topology.get("coordinate_map") if isinstance(topology.get("coordinate_map"), Mapping) else {}
    rigid = topology.get("rigid_carried") or ()
    constraints = set(manifest.get("constraints") or ())
    residuals = ((replay_trajectory or {}).get("equality_residuals") or {}
                 if isinstance(replay_trajectory, Mapping) else {})

    def closure_ok(name: str) -> bool:
        values = residuals.get(name)
        return (isinstance(values, list) and bool(values)
                and max(abs(float(value)) for value in values) <= 0.002)

    def role(name: str) -> list[str]:
        raw = bindings.get(name, ())
        return list(raw) if isinstance(raw, list) else ([raw] if isinstance(raw, str) else [])

    def body_for_joint(joint: str) -> str | None:
        direct = next((str(row.get("child")) for row in assembly.get("motion_joints", ())
                       if isinstance(row, Mapping) and row.get("name") == joint), None)
        if direct and _positions(trajectory, direct):
            return direct
        folded = joint.casefold()
        for token in ("driven_", "world_", "_hinge", "_revolute", "_spin", "_joint"):
            folded = folded.replace(token, "")
        candidates = [str(name) for name in (trajectory.get("bodies") or {})
                      if folded in str(name).casefold() or str(name).casefold() in folded]
        return min(candidates, key=lambda value: (len(value), value)) if candidates else None

    all_positions = [position for name in (trajectory.get("bodies") or {})
                     for position in (_positions(trajectory, str(name)) or ())]
    machine_diagonal = math.sqrt(sum(
        (max(row[axis] for row in all_positions) - min(row[axis] for row in all_positions)) ** 2
        for axis in range(3))) if all_positions else 100.0
    fixed_axis_limit = axis_drift_limit_mm(machine_diagonal)

    def fixed_axes(*names: str) -> bool:
        for name in names:
            link = links.get(name)
            joint = coordinate.get(name)
            if (not link or link.get("dof") != "spin" or not link.get("mount")
                    or not isinstance(joint, str) or not joint):
                return False
            body = body_for_joint(joint)
            positions = _positions(trajectory, body) if body else None
            if not positions or max(math.dist(positions[0], row) for row in positions) > fixed_axis_limit:
                return False
        return True

    if task_id == "01_single_stage_4to1":
        result["fixed_shaft_axes"] = fixed_axes("input_shaft", "output_shaft")
        gear_names = role("gear")
        result["rigid_gear_carrying"] = (len(gear_names) == 2 and
            all(links.get(name, {}).get("mount") for name in gear_names))
        mesh_relations = [row for row in assembly.get("relations", ()) if isinstance(row, Mapping)
                          and "gear" in str(row.get("mate_type", ""))]
        result["single_mesh"] = len(mesh_relations) == 1
    elif task_id == "02_two_stage_9to1":
        result["fixed_shaft_axes"] = fixed_axes("input_shaft", "intermediate_shaft", "output_shaft")
        pair = ("intermediate_driven_gear", "intermediate_stage2_pinion")
        result["rigid_compound_pair"] = all(links.get(name, {}).get("mount") == "intermediate_shaft" for name in pair)
        mesh_relations = [row for row in assembly.get("relations", ()) if isinstance(row, Mapping)
                          and "gear" in str(row.get("mate_type", ""))]
        result["two_live_meshes"] = len(mesh_relations) == 2
    elif task_id == "03_idler_reverser_1to1":
        result["fixed_shaft_axes"] = fixed_axes("input_shaft", "idler_shaft", "output_shaft")
        joints = role("input_shaft") + role("idler_shaft") + role("output_shaft")
        result["independent_idler_hinge"] = len(joints) == 3 and len(set(joints)) == 3
        mesh_relations = [row for row in assembly.get("relations", ()) if isinstance(row, Mapping)
                          and "gear" in str(row.get("mate_type", ""))]
        result["two_live_meshes"] = len(mesh_relations) == 2
    elif task_id == "04_openwork_clock_12to1":
        minute, hour = role("minute_input"), role("hour_output")
        result["coaxial_independent_hands"] = bool(minute and hour and minute[0] != hour[0])
        result["hands_remain_carried"] = (links.get("minute_hand", {}).get("mount") is not None
                                           and links.get("hour_hand", {}).get("mount") is not None)
    elif task_id in {"05_three_planet_4to1", "06_four_planet_4to1"}:
        count = 3 if task_id.startswith("05_") else 4
        poses = {str(row.get("child")): row for row in assembly.get("poses", ()) if isinstance(row, Mapping)}
        angles = []
        for index in range(1, count + 1):
            name = f"planet_pin_{index}"
            xyz = poses.get(name, {}).get("xyz_m")
            if isinstance(xyz, list) and len(xyz) >= 2:
                angles.append(math.atan2(float(xyz[1]), float(xyz[0])))
        spacing_ok = False
        if len(angles) == count:
            values = sorted((angle % (2 * math.pi)) for angle in angles)
            gaps = [((values[(index + 1) % count] - values[index]) % (2 * math.pi))
                    for index in range(count)]
            target = 2 * math.pi / count
            spacing_ok = max(abs(value - target) for value in gaps) <= math.radians(2)
        result["equally_spaced" if count == 3 else "spacing_90_deg"] = spacing_ok
    elif task_id == "07_horizontal_slider_crank":
        result["fixed_crank_axis"] = fixed_axes("crankshaft")
        slider = _positions(trajectory, "slider")
        if slider:
            axial_span = max(row[0] for row in slider) - min(row[0] for row in slider)
            lateral = max(max(row[1] for row in slider) - min(row[1] for row in slider),
                          max(row[2] for row in slider) - min(row[2] for row in slider))
            result["lateral_drift_le_2pct_span"] = lateral <= max(0.5, axial_span * 0.02)
        result["closures_below_2pct_scale"] = closure_ok("slider_end_pin_connect")
    elif task_id == "08_vertical_piston_pump":
        result["fixed_crank_axis"] = fixed_axes("crankshaft")
        result["rod_crosshead_closure"] = closure_ok("wrist_pin_small_end_connect")
        contacts = trajectory.get("contacts")
        if isinstance(contacts, Mapping):
            samples = contacts.get("samples")
            if isinstance(samples, list):
                result["no_ground_collision"] = not any(
                    any(str(row.get("body1", "")).casefold() == "world" or
                        str(row.get("body2", "")).casefold() == "world" or
                        "ground" in str(row.get("geom1", "")).casefold() or
                        "ground" in str(row.get("geom2", "")).casefold()
                        for row in sample if isinstance(row, Mapping))
                    for sample in samples if isinstance(sample, list))
    elif task_id == "09_open_pumpjack":
        result["fixed_crank_axis"] = fixed_axes("crankshaft")
    elif task_id == "10_wind_rotor_pump":
        result["fixed_rotor_axis"] = fixed_axes("rotor_shaft")
        closure_names = [name for name in constraints if "connect" in name or "closure" in name]
        result["closures_below_2pct_scale"] = bool(closure_names) and all(
            closure_ok(name) for name in closure_names)

    return {name: bool(result[name]) for name in task.invariants if name in result}


def build_archive_evidence(portable_dir: str | Path, original_run: str | Path,
                           work_dir: str | Path, *, limits: ResourceLimits | None = None) -> dict[str, Path]:
    portable = Path(portable_dir)
    original = Path(original_run)
    work = Path(work_dir)
    work.mkdir(parents=True, exist_ok=True)
    task_id = portable.name
    assembly = _read(portable / "assembly.json")
    manifest = _manifest_sources(portable)
    bindings = _read(portable / "task_bindings.json").get("roles", {})
    selected_trajectory = _read(portable / "evidence" / "trajectory.json")

    geometry = analyze_geometry(original, assembly, manifest)
    geometry_path = work / "geometry.json"
    geometry_path.write_text(json.dumps(geometry, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")

    input_role = next((item.role for item in get_task(task_id).roles if "input" in item.role), None)
    if task_id == "10_wind_rotor_pump":
        input_role = "rotor_shaft_input"
    if not input_role or not bindings.get(input_role):
        raise ContractError(f"missing replay input binding for {task_id}")
    replay = replay_model(portable / "models" / "model.mjcf", work / "replay", task_id,
                          model_root=portable, input_joint=bindings[input_role][0], limits=limits)
    execution = {
        "schema": "physcad-scorer-execution/1.0",
        "analyzer_version": ARCHIVE_EVIDENCE_VERSION,
        "model_compiled": True, "initialized": True,
        "all_finite": replay.metadata["finite_health"]["all_finite"],
        "input_hashes": {
            "models/model.mjcf": _sha(portable / "models" / "model.mjcf"),
            **{path: digest for path, digest in geometry["input_hashes"].items()},
        },
        "replay_metadata": replay.metadata,
    }
    execution_path = work / "execution.json"
    execution_path.write_text(json.dumps(execution, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")

    replay_trajectory = _read(Path(replay.trajectory_path))
    selected_trajectory["invariants"] = _derived_invariants(
        task_id, assembly, manifest, selected_trajectory, bindings, replay_trajectory)
    trajectory_path = work / "trajectory.json"
    trajectory_path.write_text(json.dumps(selected_trajectory, indent=2, sort_keys=True,
                                          allow_nan=False), encoding="utf-8")
    return {"execution": execution_path, "geometry": geometry_path,
            "trajectory": trajectory_path}


__all__ = ["ARCHIVE_EVIDENCE_VERSION", "build_archive_evidence"]
