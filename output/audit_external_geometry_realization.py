from __future__ import annotations

import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import numpy as np
import trimesh
from scipy.spatial import cKDTree
from scipy.signal import find_peaks

from benchmark_scorer.geometry import _world_transforms

ROOT = REPO / "output" / "external_benchmark_strict"
OUT = REPO / "output" / "external_geometry_realization_audit"


def load_solids(task: Path, assembly: dict):
    transforms = _world_transforms(assembly)
    meshes = {}
    for row in assembly.get("links", []):
        relative = row.get("mesh_filename")
        if not relative:
            continue
        mesh = trimesh.load_mesh(task / relative, force="mesh")
        mesh.apply_transform(transforms[row["name"]])
        meshes[row["name"]] = mesh
    return meshes


def bbox_gap(first, second):
    delta = np.maximum(np.maximum(first.bounds[0] - second.bounds[1],
                                  second.bounds[0] - first.bounds[1]), 0.0)
    return float(np.linalg.norm(delta))


def sampled_distance(first, second, cap=2500):
    a = np.asarray(first.vertices)
    b = np.asarray(second.vertices)
    if len(a) > cap:
        a = a[np.linspace(0, len(a) - 1, cap).astype(int)]
    if len(b) > cap:
        b = b[np.linspace(0, len(b) - 1, cap).astype(int)]
    da = cKDTree(b).query(a, k=1)[0]
    db = cKDTree(a).query(b, k=1)[0]
    return float(min(da.min(), db.min()))


def components(nodes, edges):
    adjacency = {name: set() for name in nodes}
    for first, second in edges:
        adjacency[first].add(second)
        adjacency[second].add(first)
    result = []
    seen = set()
    for start in nodes:
        if start in seen:
            continue
        comp = set()
        todo = [start]
        while todo:
            name = todo.pop()
            if name in comp:
                continue
            comp.add(name)
            todo.extend(adjacency[name] - comp)
        seen |= comp
        result.append(sorted(comp))
    return sorted(result, key=lambda value: (-len(value), value))


def contour_metrics(mesh, axis):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    center = mesh.bounds.mean(axis=0)
    section = mesh.section(plane_origin=center, plane_normal=axis)
    metrics = []
    if section is None:
        return metrics
    helper = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.8 else np.array([0.0, 1.0, 0.0])
    first = np.cross(axis, helper)
    first /= np.linalg.norm(first)
    second = np.cross(axis, first)
    for points in section.discrete:
        points = np.asarray(points)
        delta = points - center
        angle = np.arctan2(delta @ second, delta @ first) % (2 * np.pi)
        radius = np.sqrt((delta @ first) ** 2 + (delta @ second) ** 2)
        order = np.argsort(angle)
        angle = angle[order]
        radius = radius[order]
        unique_angle = []
        unique_radius = []
        for value, radial in zip(angle, radius):
            if unique_angle and abs(value - unique_angle[-1]) < 1e-8:
                unique_radius[-1] = max(unique_radius[-1], radial)
            else:
                unique_angle.append(value)
                unique_radius.append(radial)
        angle = np.asarray(unique_angle)
        radius = np.asarray(unique_radius)
        if len(angle) < 3:
            continue
        grid = np.linspace(0, 2 * np.pi, 4096, endpoint=False)
        extended_angle = np.r_[angle[-1] - 2 * np.pi, angle, angle[0] + 2 * np.pi]
        extended_radius = np.r_[radius[-1], radius, radius[0]]
        signal = np.interp(grid, extended_angle, extended_radius)
        amplitude = float(np.percentile(signal, 95) - np.percentile(signal, 5))
        prominence = max(0.05, amplitude * 0.20)
        peaks, _ = find_peaks(np.r_[signal, signal, signal], prominence=prominence, distance=4)
        peaks = peaks[(peaks >= len(signal)) & (peaks < 2 * len(signal))] - len(signal)
        metrics.append({
            "radius_median_mm": float(np.median(signal)),
            "radial_amplitude_mm": amplitude,
            "periodic_peak_count": int(len(peaks)),
            "loop_points": int(len(points)),
        })
    return sorted(metrics, key=lambda row: row["radius_median_mm"], reverse=True)


def gear_names(assembly):
    names = set()
    for pair in assembly.get("mesh_pairs", []):
        if isinstance(pair, list):
            names.update(str(value) for value in pair[:2])
    return names


def audit_task(task: Path):
    assembly = json.loads((task / "assembly.json").read_text(encoding="utf-8"))
    links = {row["name"]: row for row in assembly.get("links", [])}
    meshes = load_solids(task, assembly)
    names = sorted(meshes)
    pair_rows = []
    edges_1_5 = []
    edges_10 = []
    for index, first_name in enumerate(names):
        for second_name in names[index + 1:]:
            first = meshes[first_name]
            second = meshes[second_name]
            gap = bbox_gap(first, second)
            if gap > 10.0:
                continue
            distance = sampled_distance(first, second)
            pair_rows.append({"pair": [first_name, second_name],
                              "aabb_gap_mm": gap,
                              "sampled_surface_distance_mm": distance})
            if distance <= 10.0:
                edges_10.append((first_name, second_name))
            if distance <= 1.5:
                edges_1_5.append((first_name, second_name))
    root = str(assembly.get("root_link") or "")
    strict_components = components(names, edges_1_5)
    conservative_components = components(names, edges_10)
    root_strict = next((set(comp) for comp in strict_components if root in comp), set())
    root_conservative = next((set(comp) for comp in conservative_components if root in comp), set())
    gears = []
    for name in sorted(gear_names(assembly)):
        mesh = meshes.get(name)
        row = links.get(name, {})
        if mesh is None:
            gears.append({"name": name, "status": "missing_mesh"})
            continue
        split = mesh.split(only_watertight=False)
        contours = contour_metrics(mesh, row.get("spin_axis") or [0, 0, 1])
        maximum_amplitude = max((item["radial_amplitude_mm"] for item in contours), default=0.0)
        maximum_peaks = max((item["periodic_peak_count"] for item in contours), default=0)
        if maximum_amplitude >= 0.25 and maximum_peaks >= 4:
            status = "integrated_periodic_tooth_contour"
        elif len(split) > 1:
            status = "disconnected_components_possible_teeth"
        else:
            status = "smooth_or_unverified_gear"
        gears.append({"name": name, "status": status,
                       "connected_component_count": len(split),
                       "contours": contours})
    return {
        "task_id": task.name,
        "root_link": root,
        "physical_link_count": len(names),
        "strict_1_5mm": {
            "component_count": len(strict_components),
            "floating_links": sorted(set(names) - root_strict),
            "components": strict_components,
        },
        "conservative_10mm": {
            "component_count": len(conservative_components),
            "floating_links": sorted(set(names) - root_conservative),
            "components": conservative_components,
        },
        "near_pair_measurements": pair_rows,
        "gears": gears,
    }


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    summary = []
    for method in ("claude-code", "codex"):
        for task in sorted(path for path in (ROOT / method).iterdir() if path.is_dir()):
            result = audit_task(task)
            target = OUT / method
            target.mkdir(parents=True, exist_ok=True)
            (target / f"{task.name}.json").write_text(
                json.dumps(result, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
            tooth_failures = [row["name"] for row in result["gears"]
                              if row["status"] != "integrated_periodic_tooth_contour"]
            row = {"method": method, "task_id": task.name,
                   "floating_1_5mm": len(result["strict_1_5mm"]["floating_links"]),
                   "floating_10mm": len(result["conservative_10mm"]["floating_links"]),
                   "gear_failures": tooth_failures}
            summary.append(row)
            print(method, task.name,
                  "floating1.5", row["floating_1_5mm"],
                  "floating10", row["floating_10mm"],
                  "gear_failures", tooth_failures, flush=True)
    (OUT / "suite_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")


if __name__ == "__main__":
    main()
