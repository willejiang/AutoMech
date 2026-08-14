"""Generate scorer-owned evidence for the task-05 independent-values pilot."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from benchmark_scorer.geometry import analyze_geometry

EXPERIMENT = Path(__file__).resolve().parent
RUN = EXPERIMENT / "runs" / "design_this_as_an_open_demonstra_20260813_153115"
BASELINE_RUN = (REPO / "output" / "comfort_benchmark_v1_20260811" /
                "05_three_planet_4to1" /
                "design_this_as_an_open_demonstra_20260811_170424")
BASELINE_SCORE = (REPO / "output" / "comfort_benchmark_v1_rescored_scores" /
                  "05_three_planet_4to1" / "score.json")
BASELINE_GEOMETRY = (REPO / "output" / "comfort_benchmark_v1_rescored_portable" /
                     "05_three_planet_4to1" / "evidence" / "geometry.json")


def read_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False),
                    encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def world_positions(model: dict) -> dict[str, tuple[float, float, float]]:
    poses = {row["child"]: row for row in model.get("poses", [])}
    cache: dict[str, tuple[float, float, float]] = {"": (0.0, 0.0, 0.0)}

    def position(name: str) -> tuple[float, float, float]:
        if name in cache:
            return cache[name]
        row = poses[name]
        parent = position(str(row.get("parent", "")))
        xyz = row.get("xyz_m", [0.0, 0.0, 0.0])
        result = tuple(parent[i] + 1000.0 * float(xyz[i]) for i in range(3))
        cache[name] = result
        return result

    return {row["name"]: position(row["name"]) for row in model.get("links", [])}


def metric(name: str, expected, observed, residual: float, tolerance: float,
           units: str, evidence: str, note: str = "") -> dict:
    return {
        "name": name,
        "expected": expected,
        "observed": observed,
        "normalized_residual_rho": residual,
        "tolerance_epsilon": tolerance,
        "units": units,
        "status": "PASS" if residual <= tolerance else "FAIL",
        "evidence": evidence,
        "note": note,
    }


def unavailable(name: str, reason: str, evidence: str) -> dict:
    return {"name": name, "status": "UNAVAILABLE", "reason": reason,
            "evidence": evidence}


def main() -> None:
    model_path = RUN / "kinematic_model.json"
    gate_path = RUN / "mjcf_gate_report.json"
    source_path = RUN / "machine.py"
    eval_path = RUN / "machine_eval.json"
    model = read_json(model_path)
    gate = read_json(gate_path)
    machine_eval = read_json(eval_path)

    geometry = analyze_geometry(RUN, model, {})
    write_json(EXPERIMENT / "geometry.json", geometry)

    stage = model["planetary_stages"][0]
    sun_teeth = int(stage["sun_teeth"])
    planet_teeth = int(stage["planet_teeth"])
    ring_teeth = int(stage["ring_teeth"])
    positions = world_positions(model)
    ports = model["ports_by_link"]

    def port(link: str, name: str) -> dict:
        return next(row for row in ports[link] if row["name"] == name)

    metrics: list[dict] = []
    metrics.append(metric(
        "planetary_tooth_arithmetic", sun_teeth + 2 * planet_teeth, ring_teeth,
        abs(ring_teeth - (sun_teeth + 2 * planet_teeth)) / max(ring_teeth, 1),
        0.0, "relative", "kinematic_model.json#/planetary_stages/0"))
    expected_gain = 1.0 / (1.0 + ring_teeth / sun_teeth)
    metrics.append(metric(
        "theoretical_carrier_gain", 0.25, expected_gain,
        abs(expected_gain - 0.25) / 0.25, 0.025, "relative",
        "kinematic_model.json#/planetary_stages/0"))
    gear_relations = [row for row in model.get("relations", [])
                      if str(row.get("mate_type", "")).startswith("gear_")]
    sun_relations = [row for row in gear_relations
                     if {row.get("base_part"), row.get("incoming_part")} & {"sun_gear"}]
    ring_relations = [row for row in gear_relations
                      if {row.get("base_part"), row.get("incoming_part")} & {"fixed_ring_gear"}]
    metrics.append(metric(
        "authored_sun_planet_mesh_count", 3, len(sun_relations),
        abs(len(sun_relations) - 3) / 3.0, 0.0, "relative",
        "kinematic_model.json#/relations"))
    metrics.append(metric(
        "authored_ring_planet_mesh_count", 3, len(ring_relations),
        abs(len(ring_relations) - 3) / 3.0, 0.0, "relative",
        "kinematic_model.json#/relations",
        "Planetary-stage membership does not replace the required explicit physical mesh relations."))

    sun_xy = positions["sun_gear"][:2]
    ring_xy = positions["fixed_ring_gear"][:2]
    sun_r = float(port("sun_gear", "planet_mesh_1")["pitch_radius_mm"])
    ring_r = float(port("fixed_ring_gear", "internal_mesh_1")["pitch_radius_mm"])
    angles = []
    for index in range(1, 4):
        gear = f"planet_gear_{index}"
        pin = f"planet_pin_{index}"
        gear_xy = positions[gear][:2]
        pin_xy = positions[pin][:2]
        planet_r = float(port(gear, "sun_mesh")["pitch_radius_mm"])
        sun_distance = math.dist(sun_xy, gear_xy)
        expected_sun_distance = sun_r + planet_r
        metrics.append(metric(
            f"sun_planet_{index}_center_distance", expected_sun_distance,
            sun_distance, abs(sun_distance - expected_sun_distance) /
            expected_sun_distance, 0.01, "relative",
            f"kinematic_model.json#/poses and /ports_by_link/{gear}"))
        ring_distance = math.dist(ring_xy, gear_xy)
        expected_ring_distance = ring_r - planet_r
        metrics.append(metric(
            f"ring_planet_{index}_center_distance", expected_ring_distance,
            ring_distance, abs(ring_distance - expected_ring_distance) /
            expected_ring_distance, 0.01, "relative",
            f"kinematic_model.json#/poses and /ports_by_link/{gear}"))
        coaxial = math.dist(gear_xy, pin_xy)
        metrics.append(metric(
            f"planet_{index}_gear_pin_coaxiality", 0.0, coaxial,
            coaxial, 0.5, "mm", "kinematic_model.json#/poses"))
        angles.append(math.atan2(gear_xy[1] - sun_xy[1], gear_xy[0] - sun_xy[0]) %
                      (2.0 * math.pi))

    angles.sort()
    target_gap = 2.0 * math.pi / 3.0
    gaps = [((angles[(i + 1) % 3] - angles[i]) % (2.0 * math.pi)) for i in range(3)]
    for index, gap in enumerate(gaps, 1):
        error_deg = abs(math.degrees(gap - target_gap))
        metrics.append(metric(
            f"planet_spacing_gap_{index}", 120.0, math.degrees(gap),
            error_deg, 2.0, "degrees", "kinematic_model.json#/poses"))

    fit_specs = [
        ("input_bearing_press_fit", "press", 12.2, 12.0,
         "relation/input_bearing_in_pedestal"),
        ("input_shaft_running_fit", "running", 8.2, 8.0,
         "relation/input_shaft_lower_journal"),
        ("sun_shaft_press_fit", "press", 7.9, 8.0,
         "relation/sun_to_input_shaft_press_fit"),
        ("carrier_bearing_pedestal_running_fit", "running", 16.2, 16.0,
         "relation/carrier_bearing_on_pedestal"),
        ("carrier_central_running_fit", "running", 18.3, 18.0,
         "relation/carrier_on_central_bearing"),
        ("planet_1_bushing_running_fit", "running", 8.2, 8.0,
         "relation/planet_1_bushing_journal"),
        ("planet_2_bushing_running_fit", "running", 8.2, 8.0,
         "relation/planet_2_bushing_journal"),
        ("planet_3_bushing_running_fit", "running", 8.2, 8.0,
         "relation/planet_3_bushing_journal"),
    ]
    for name, kind, bore, shaft, evidence in fit_specs:
        signed_clearance = bore - shaft
        passed = signed_clearance < 0.0 if kind == "press" else signed_clearance > 0.0
        metrics.append(metric(
            name, "negative" if kind == "press" else "positive", signed_clearance,
            0.0 if passed else 1.0, 0.0, "diametral_clearance_mm", evidence,
            "Sign-only fit criterion; zero clearance does not pass."))

    dynamic_probe = gate["dynamic_transmission_probe"]
    contact_on = float(dynamic_probe["contact_on"]["driver_travel"])
    contact_off = float(dynamic_probe["contact_off"]["driver_travel"])
    metrics.append(metric(
        "contact_preserved_input_mobility", 1.0, contact_on / contact_off,
        1.0 - contact_on / contact_off, 0.10, "fraction_of_no_contact_travel",
        "mjcf_gate_report.json#/dynamic_transmission_probe",
        "Fails when retained CAD collision materially stalls the mechanism."))

    metrics.extend([
        unavailable("trajectory_carrier_sun_ratio",
                    "No MJCF compiler was accepted; no selected final trajectory exists.",
                    "result.json#/physics"),
        unavailable("planet_orbit",
                    "No accepted physics trajectory exists.", "result.json#/physics"),
        unavailable("planet_local_spin",
                    "No accepted physics trajectory exists.", "result.json#/physics"),
        unavailable("ring_fixed_trajectory",
                    "No accepted physics trajectory exists.", "result.json#/physics"),
        unavailable("planet_pin_carrying_constancy",
                    "No accepted physics trajectory exists.", "result.json#/physics"),
    ])
    evaluated = [row for row in metrics if row["status"] in {"PASS", "FAIL"}]
    failed = [row for row in evaluated if row["status"] == "FAIL"]
    dependency = {
        "schema": "physcad-representation-dependency-metrics/1.0",
        "task_id": "05_three_planet_4to1",
        "arm": "independent_values",
        "definition": "Q_dep = 1 - failed_evaluated_dependencies / evaluated_dependencies",
        "unavailable_excluded_from_denominator": True,
        "evaluated_count": len(evaluated),
        "failed_count": len(failed),
        "unavailable_count": sum(row["status"] == "UNAVAILABLE" for row in metrics),
        "q_dep": 1.0 - len(failed) / len(evaluated),
        "metrics": metrics,
        "input_hashes": {
            "kinematic_model.json": sha256(model_path),
            "machine.py": sha256(source_path),
            "machine_eval.json": sha256(eval_path),
            "mjcf_gate_report.json": sha256(gate_path),
            "scorer_reexecution/machine_eval.json": sha256(
                EXPERIMENT / "scorer_reexecution" / "machine_eval.json"),
        },
    }
    write_json(EXPERIMENT / "dependency_metrics.json", dependency)

    baseline_score = read_json(BASELINE_SCORE)
    baseline_geometry = read_json(BASELINE_GEOMETRY)
    baseline_metrics = read_json(BASELINE_RUN / "benchmark_metrics.json")
    pilot_metrics = read_json(RUN / "benchmark_metrics.json")
    score = {
        "schema": "physcad-representation-ablation-score/1.0",
        "task_id": "05_three_planet_4to1",
        "arm": "independent_values",
        "raw_harness_ok": False,
        "layers": [
            {"index": 1, "name": "execution", "weight": 10, "status": "PASS",
             "points_awarded": 10,
             "reason": "Scorer-owned source reexecution produced 23 nonempty solids; rejected candidate MJCF compiled with finite nq=nv=5."},
            {"index": 2, "name": "assembly", "weight": 15, "status": "PASS",
             "points_awarded": 15,
             "reason": "Final iteration passed the connectivity gate and contains all required task-05 roles, three planet hinges, distinct input/output, and a fixed ring."},
            {"index": 3, "name": "geometry", "weight": 15,
             "status": "PASS" if geometry["non_exempt_conflict_count"] == 0 else "FAIL",
             "points_awarded": 15 if geometry["non_exempt_conflict_count"] == 0 else 0,
             "reason": f"Scorer-owned exact-solid non-exempt conflict count={geometry['non_exempt_conflict_count']}."},
            {"index": 4, "name": "physics_ready", "weight": 20, "status": "FAIL",
             "points_awarded": 0,
             "reason": "No MJCF compiler was accepted: contact materially stalled the closed mechanism."},
            {"index": 5, "name": "functional", "weight": 40, "status": "FAIL",
             "points_awarded": 0,
             "reason": "Prerequisite physics-ready layer failed; no accepted trajectory can establish input, output, ratio, orbit, spin, or fixed-ring invariants."},
        ],
        "total_points": 25,
        "overall_verdict": "FAIL",
        "observed_mechanical_verdict": "FAIL",
        "evidence": {
            "geometry": "geometry.json",
            "dependencies": "dependency_metrics.json",
            "gate": str(gate_path.relative_to(REPO)).replace("\\", "/"),
            "machine_eval": str(eval_path.relative_to(REPO)).replace("\\", "/"),
        },
    }
    # Geometry is a cumulative prerequisite: a failure leaves the 25 points from layers 1-2.
    if geometry["non_exempt_conflict_count"] == 0:
        score["total_points"] = 40
    write_json(EXPERIMENT / "score.json", score)

    comparison = {
        "schema": "physcad-representation-ablation-comparison/1.0",
        "task_id": "05_three_planet_4to1",
        "study_design": "single archived executable-dependency baseline versus one fresh independent-values pilot",
        "causal_scope": "case-study evidence only; not a paired statistical estimate",
        "baseline": {
            "arm": "executable_dependencies",
            "score": baseline_score["total_points"],
            "verdict": baseline_score["overall_verdict"],
            "iterations": baseline_metrics["outcome"]["iterations"],
            "duration_s": baseline_metrics["duration_s"],
            "non_exempt_conflicts": baseline_geometry["non_exempt_conflict_count"],
            "requests": baseline_metrics["usage"]["requests"],
            "total_tokens": baseline_metrics["usage"]["tokens"]["total_tokens"],
            "tool_calls": baseline_metrics["tools"]["calls"],
            "compiler_candidates": baseline_metrics["mjcf_compiler"]["candidates"],
            "compiler_submissions": baseline_metrics["mjcf_compiler"]["submissions"],
        },
        "independent_values": {
            "score": score["total_points"],
            "verdict": score["overall_verdict"],
            "iterations": pilot_metrics["outcome"]["iterations"],
            "duration_s": pilot_metrics["duration_s"],
            "non_exempt_conflicts": geometry["non_exempt_conflict_count"],
            "q_dep_evaluated_only": dependency["q_dep"],
            "q_dep_evaluated_count": dependency["evaluated_count"],
            "q_dep_unavailable_count": dependency["unavailable_count"],
            "requests": pilot_metrics["usage"]["requests"],
            "total_tokens": pilot_metrics["usage"]["tokens"]["total_tokens"],
            "tool_calls": pilot_metrics["tools"]["calls"],
            "compiler_candidates": pilot_metrics["mjcf_compiler"]["candidates"],
            "compiler_submissions": pilot_metrics["mjcf_compiler"]["submissions"],
            "failure_domain": "builder_compiler/mjcf_agent_compile_failed",
            "contact_on_to_off_driver_travel": contact_on / contact_off,
        },
        "deltas_independent_minus_baseline": {
            "score": score["total_points"] - baseline_score["total_points"],
            "iterations": pilot_metrics["outcome"]["iterations"] - baseline_metrics["outcome"]["iterations"],
            "duration_s": pilot_metrics["duration_s"] - baseline_metrics["duration_s"],
            "non_exempt_conflicts": geometry["non_exempt_conflict_count"] - baseline_geometry["non_exempt_conflict_count"],
            "requests": pilot_metrics["usage"]["requests"] - baseline_metrics["usage"]["requests"],
            "total_tokens": pilot_metrics["usage"]["tokens"]["total_tokens"] - baseline_metrics["usage"]["tokens"]["total_tokens"],
            "tool_calls": pilot_metrics["tools"]["calls"] - baseline_metrics["tools"]["calls"],
        },
    }
    write_json(EXPERIMENT / "comparison.json", comparison)

    manifest = read_json(EXPERIMENT / "experiment_manifest.after.json")
    report = f"""# Task 05 representation ablation pilot\n\n## Result\n\n| Arm | Score | Verdict | Iterations | Runtime | Exact non-exempt conflicts |\n|---|---:|---|---:|---:|---:|\n| Executable dependencies (archived baseline) | {baseline_score['total_points']}/100 | {baseline_score['overall_verdict']} | {baseline_metrics['outcome']['iterations']} | {baseline_metrics['duration_s']:.3f} s | {baseline_geometry['non_exempt_conflict_count']} |\n| Independent values (fresh pilot) | {score['total_points']}/100 | FAIL | {pilot_metrics['outcome']['iterations']} | {pilot_metrics['duration_s']:.3f} s | {geometry['non_exempt_conflict_count']} |\n\nThe independent-values candidate passed source execution and final structural role/cardinality checks, but failed the cumulative Geometry gate and therefore received no Physics-ready or Functional points. Its MJCF candidate compiled and remained finite, but retained contacts reduced driver travel from `{contact_off:.6f}` rad to `{contact_on:.6f}` rad ({100.0 * contact_on / contact_off:.3f}% of the no-contact control), so policy v5 correctly rejected it.\n\n## Dependency consistency\n\nEvaluated-only $Q_{{dep}}={dependency['q_dep']:.4f}$ ({dependency['evaluated_count'] - dependency['failed_count']}/{dependency['evaluated_count']} checks pass); {dependency['unavailable_count']} dynamic checks are explicitly unavailable and excluded from the denominator. This high static score does **not** imply a functional mechanism: repeated literals happened to preserve tooth arithmetic, centers, and spacing, while the independent axial/radial choices produced invalid fit/contact realization.\n\nFailed evaluated dependencies:\n"""
    for row in failed:
        report += f"- `{row['name']}`: expected {row['expected']}, observed {row['observed']}\n"
    report += f"""\nUnavailable dynamic dependencies:\n"""
    for row in metrics:
        if row["status"] == "UNAVAILABLE":
            report += f"- `{row['name']}`: {row['reason']}\n"
    report += f"""\n## Cost and refinement\n\n- Runtime: `{baseline_metrics['duration_s']:.3f} s -> {pilot_metrics['duration_s']:.3f} s`\n- Requests: `{baseline_metrics['usage']['requests']} -> {pilot_metrics['usage']['requests']}`\n- Total tokens: `{baseline_metrics['usage']['tokens']['total_tokens']} -> {pilot_metrics['usage']['tokens']['total_tokens']}`\n- Tool calls: `{baseline_metrics['tools']['calls']} -> {pilot_metrics['tools']['calls']}`\n- MJCF candidates/submissions: `{baseline_metrics['mjcf_compiler']['candidates']}/{baseline_metrics['mjcf_compiler']['submissions']} -> {pilot_metrics['mjcf_compiler']['candidates']}/{pilot_metrics['mjcf_compiler']['submissions']}`\n- Iterations 0 and 1 both failed declared crank/shaft proximity; iteration 2 repaired that gate but still failed retained-contact dynamics.\n- Both runs observed provider/project cache activity, so neither qualifies as a strict cold run.\n\n## Isolation\n\nProduction source hashes unchanged: `{manifest['production_hashes_unchanged']}`. Production prompt hash unchanged: `{manifest['production_prompt_unchanged']}`. The overlay existed only in the experiment process and all added files are under ignored `output/representation_ablation_05_three_planet_4to1/`.\n\n## Interpretation\n\nThis single pilot is direct evidence of degradation on the selected task (`100/PASS -> {score['total_points']}/FAIL`), but it is not sufficient for a population estimate or statistical claim. No additional stochastic rerun was performed.\n"""
    (EXPERIMENT / "pilot_report.md").write_text(report, encoding="utf-8")

    print(json.dumps({
        "geometry_conflicts": geometry["non_exempt_conflict_count"],
        "q_dep": dependency["q_dep"],
        "score": score["total_points"],
        "production_hashes_unchanged": manifest["production_hashes_unchanged"],
        "production_prompt_unchanged": manifest["production_prompt_unchanged"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
