from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil

REPO = Path(__file__).resolve().parents[1]
DOWNLOADS = REPO.parent
ROOT = REPO / "benchmark_results" / "comfort_v1_20260811"


def copy_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    if ROOT.exists():
        raise RuntimeError(f"refusing to overwrite existing snapshot: {ROOT}")
    (ROOT / "automech_scores").mkdir(parents=True)
    (ROOT / "external_strict").mkdir()
    (ROOT / "representation_ablation_task05").mkdir()
    (ROOT / "protocols").mkdir()

    report = (DOWNLOADS / "benchmark_results.md").read_text(encoding="utf-8")
    replacements = (
        ("C:/Users/t-zhijjiang/Downloads/PhysCADResearcher/", ""),
        ("C:/Users/t-zhijjiang/Downloads/benchmark.md",
         "benchmark_results/comfort_v1_20260811/protocols/benchmark.md"),
        ("C:/Users/t-zhijjiang/Downloads/external_harness_benchmark.md",
         "benchmark_results/comfort_v1_20260811/protocols/external_harness_benchmark.md"),
        ("C:/Users/t-zhijjiang/Downloads/all_benchmark_orbit_videos/",
         "GitHub release assets (see release notes)"),
        ("C:/benchmark-runs/codex-20260812-172221",
         "submitted Codex run archived in the external strict release asset"),
        ("C:/benchmark-runs/claude-code-gpt-5.6-sol-1m-20260812",
         "submitted Claude Code run archived in the external strict release asset"),
    )
    for old, new in replacements:
        report = report.replace(old, new)
    (REPO / "benchmark_results.md").write_text(report, encoding="utf-8")

    for name in ("benchmark.md", "external_harness_benchmark.md"):
        source = DOWNLOADS / name
        if source.is_file():
            copy_file(source, ROOT / "protocols" / name)

    source_scores = REPO / "output" / "comfort_benchmark_v1_rescored_scores"
    for source in source_scores.rglob("*"):
        if not source.is_file():
            continue
        relative = source.relative_to(source_scores)
        destination = ROOT / "automech_scores" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if source.suffix == ".json":
            data = json.loads(source.read_text(encoding="utf-8"))
            if source.name == "suite_score.json":
                for task in data.get("tasks", []):
                    task["path"] = f"automech_scores/{task['task_id']}/score.json"
            destination.write_text(
                json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n",
                encoding="utf-8")
        else:
            copy_file(source, destination)

    strict_source = REPO / "output" / "external_benchmark_strict"
    for method in ("codex", "claude-code"):
        for task_directory in sorted((strict_source / method).iterdir()):
            strict = task_directory / "strict"
            if not strict.is_dir():
                continue
            for source in sorted(strict.glob("*.json")):
                copy_file(
                    source,
                    ROOT / "external_strict" / method / task_directory.name / source.name)

    declared_source = REPO / "output" / "external_declared_realization_audit"
    for source in declared_source.rglob("*.json"):
        copy_file(
            source,
            ROOT / "external_strict" / "declared_realization" /
            source.relative_to(declared_source))

    reasons = {
        "codex": (
            ("01_single_stage_4to1", "PASS", "Source, geometry, one input actuator, ratio binding, collision proxies, and replay pass."),
            ("02_two_stage_9to1", "FAIL", "Incomplete collision coverage and geometrically detached declared assembly."),
            ("03_idler_reverser_1to1", "FAIL", "Incomplete gear/crank collision coverage and detached physical islands."),
            ("04_openwork_clock_12to1", "FAIL", "Invalid bearing solids and incomplete realization/collision semantics."),
            ("05_three_planet_4to1", "FAIL", "Non-exempt conflict, detached structure, and unverified integrated ring teeth."),
            ("06_four_planet_4to1", "FAIL", "Non-exempt conflict, detached structure, and unverified integrated ring teeth."),
            ("07_horizontal_slider_crank", "FAIL", "Collision conflicts and a persistent geometrically detached link."),
            ("08_vertical_piston_pump", "FAIL", "Seven non-exempt conflicts and eight detached physical links."),
            ("09_open_pumpjack", "FAIL", "Eleven non-exempt conflicts and eleven detached physical links."),
            ("10_wind_rotor_pump", "FAIL", "Eight conflicts, incomplete closure/transmission semantics, and eleven detached links."),
        ),
        "claude-code": (
            ("01_single_stage_4to1", "FAIL", "Five non-exempt conflicts and most links are not connected to the base."),
            ("02_two_stage_9to1", "FAIL", "Moving collisions disabled, fragmented gears, and eight detached links."),
            ("03_idler_reverser_1to1", "FAIL", "Moving collisions disabled and three fragmented gear compounds."),
            ("04_openwork_clock_12to1", "FAIL", "Two undeclared material clashes and all moving collision disabled."),
            ("05_three_planet_4to1", "FAIL", "Six conflicts and unverified ring-tooth realization."),
            ("06_four_planet_4to1", "FAIL", "Seventeen geometry/unavailable failures and invalid transmission lowering."),
            ("07_horizontal_slider_crank", "FAIL", "Three conflicts and five detached physical links."),
            ("08_vertical_piston_pump", "FAIL", "Two conflicts, invalid closure semantics, and fourteen detached links."),
            ("09_open_pumpjack", "FAIL", "Two conflicts and ten detached physical links."),
            ("10_wind_rotor_pump", "FAIL", "No authored transmission, inactive closure, stationary replay output, and nine detached links."),
        ),
    }
    summary = {
        "schema": "physcad-external-strict-summary/1.0",
        "suite_id": "physcad-comfort-v1",
        "evidence_policy": "Strict mechanical realization requires CAD geometry, collision semantics, active constraints, and scorer-owned replay.",
        "methods": {},
    }
    for method, rows in reasons.items():
        summary["methods"][method] = {
            "strict_passes": sum(verdict == "PASS" for _, verdict, _ in rows),
            "tasks_total": len(rows),
            "aggregate_score": None,
            "tasks": [
                {"task_id": task, "verdict": verdict, "reason": reason}
                for task, verdict, reason in rows
            ],
        }
    (ROOT / "external_strict" / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8")

    ablation_source = REPO / "output" / "representation_ablation_05_three_planet_4to1"
    for name in (
            "pilot_report.md", "score.json", "dependency_metrics.json", "geometry.json",
            "comparison.json", "experiment_manifest.after.json"):
        copy_file(ablation_source / name, ROOT / "representation_ablation_task05" / name)

    files = []
    for path in sorted(ROOT.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            files.append({
                "path": path.relative_to(ROOT).as_posix(),
                "sha256": sha256(path),
                "bytes": path.stat().st_size,
            })
    manifest = {
        "schema": "physcad-benchmark-release-snapshot/1.0",
        "suite_id": "physcad-comfort-v1",
        "release_tag": "v0.5.0-mjcf-agent-benchmark",
        "source_baseline_commit": "d409d782129de4ab842675f59246d49db034b69f",
        "authoritative_results": {
            "automech": {"score": 810, "points_possible": 1000, "strict_passes": 6, "tasks_total": 10},
            "codex": {"strict_passes": 1, "tasks_total": 10, "aggregate_score": None},
            "claude-code": {"strict_passes": 0, "tasks_total": 10, "aggregate_score": None},
            "representation_ablation_task05": {"score": 25, "points_possible": 100, "verdict": "FAIL", "pilot_only": True},
        },
        "raw_vs_derived": "AutoMech scores and external strict audits are scorer-owned derived evidence; complete submitted/raw portable evidence is distributed as release assets.",
        "withdrawn_external_aggregates_are_not_included": True,
        "files": files,
    }
    (ROOT / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8")
    total = sum(row["bytes"] for row in files)
    print(f"created {ROOT} with {len(files) + 1} files, {total / 1024 / 1024:.2f} MiB")


if __name__ == "__main__":
    main()
