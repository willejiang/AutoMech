from __future__ import annotations

import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import numpy as np
import trimesh

from benchmark_scorer.geometry import _world_transforms

ROOT = REPO / "output" / "external_benchmark_strict"
OUT = REPO / "output" / "external_declared_realization_audit"
STRICT_MM = 1.5
CONSERVATIVE_MM = 10.0


def load_meshes(task: Path, assembly: dict):
    transforms = _world_transforms(assembly)
    result = {}
    for row in assembly.get("links", []):
        relative = row.get("mesh_filename")
        if not relative:
            continue
        mesh = trimesh.load_mesh(task / relative, force="mesh")
        mesh.apply_transform(transforms[row["name"]])
        result[row["name"]] = mesh
    return result


def surface_distance(first, second, samples=1200):
    points_first, _ = trimesh.sample.sample_surface(first, samples)
    points_second, _ = trimesh.sample.sample_surface(second, samples)
    try:
        _, distance_first, _ = trimesh.proximity.closest_point(second, points_first)
        _, distance_second, _ = trimesh.proximity.closest_point(first, points_second)
    except Exception:
        _, distance_first, _ = trimesh.proximity.closest_point_naive(second, points_first)
        _, distance_second, _ = trimesh.proximity.closest_point_naive(first, points_second)
    return float(min(distance_first.min(), distance_second.min()))


def relation_edges(assembly):
    edges = []
    links = {row["name"] for row in assembly.get("links", [])}
    def add(first, second, kind, source):
        first = str(first or "").split(".", 1)[0]
        second = str(second or "").split(".", 1)[0]
        if first in links and second in links and first != second:
            key = (tuple(sorted((first, second))), kind)
            if not any((tuple(sorted((row["a"], row["b"]))), row["kind"]) == key for row in edges):
                edges.append({"a": first, "b": second, "kind": kind, "source": source})
    for row in assembly.get("links", []):
        if row.get("mount"):
            add(row["mount"], row["name"], "mount", f"link/{row['name']}")
        for parent in row.get("extra_mounts") or []:
            add(parent, row["name"], "extra_mount", f"link/{row['name']}")
    for index, row in enumerate(assembly.get("relations", [])):
        if not isinstance(row, dict):
            continue
        kind = str(row.get("mate_type") or "")
        if kind in {"press_fit", "journal_bearing", "ball_bearing", "pin", "revolute",
                    "rigid_mount", "rigid_carry", "fixed", "weld", "point_closure",
                    "pin_hinge", "running_guide", "rigid_carrying",
                    "gear_spur_external", "gear_spur_internal"}:
            add(row.get("base_part"), row.get("incoming_part"), kind,
                f"relation/{row.get('name', index)}")
    return edges


def components(nodes, edges):
    adjacency = {name: set() for name in nodes}
    for row in edges:
        adjacency[row["a"]].add(row["b"])
        adjacency[row["b"]].add(row["a"])
    seen = set()
    result = []
    for start in nodes:
        if start in seen:
            continue
        component = set()
        todo = [start]
        while todo:
            name = todo.pop()
            if name in component:
                continue
            component.add(name)
            todo.extend(adjacency[name] - component)
        seen |= component
        result.append(sorted(component))
    return sorted(result, key=lambda row: (-len(row), row))


def gear_classification(mesh):
    parts = mesh.split(only_watertight=False)
    count = len(parts)
    # A valid manufactured gear should be one connected solid. Multiple components are
    # reported as fragmented teeth/parts regardless of visible tooth-like silhouettes.
    if count > 1:
        sizes = sorted((len(part.faces) for part in parts), reverse=True)
        return {"status": "fragmented_multi_solid_gear", "component_count": count,
                "component_face_counts": sizes[:12]}
    # One component still needs radial complexity beyond a plain faceted cylinder.
    vertices = np.asarray(mesh.vertices)
    center = mesh.bounds.mean(axis=0)
    extents = mesh.extents
    axial_axis = int(np.argmin(extents))
    radial_axes = [axis for axis in range(3) if axis != axial_axis]
    delta = vertices - center
    radius = np.sqrt(delta[:, radial_axes[0]] ** 2 + delta[:, radial_axes[1]] ** 2)
    upper = radius[radius >= np.quantile(radius, 0.60)]
    levels = len(np.unique(np.round(upper, 2)))
    status = "integrated_tooth_geometry" if levels >= 5 else "smooth_or_unverified_gear"
    return {"status": status, "component_count": count,
            "upper_radial_levels_0_01mm": levels,
            "radial_range_mm": [float(radius.min()), float(radius.max())]}


def audit(task: Path):
    assembly = json.loads((task / "assembly.json").read_text(encoding="utf-8"))
    meshes = load_meshes(task, assembly)
    declared = relation_edges(assembly)
    measured = []
    for row in declared:
        first = meshes.get(row["a"])
        second = meshes.get(row["b"])
        if first is None or second is None:
            measured.append({**row, "status": "unmeasurable_semantic_group",
                             "surface_distance_mm": None})
            continue
        distance = surface_distance(first, second)
        measured.append({**row, "surface_distance_mm": distance,
                         "status": "strict_realized" if distance <= STRICT_MM else
                                   "conservative_realized" if distance <= CONSERVATIVE_MM else
                                   "geometrically_unrealized"})
    physical = sorted(meshes)
    strict_edges = [row for row in measured if row["status"] == "strict_realized"]
    conservative_edges = [row for row in measured
                          if row["status"] in {"strict_realized", "conservative_realized"}]
    strict_components = components(physical, strict_edges)
    conservative_components = components(physical, conservative_edges)
    root = str(assembly.get("root_link") or "")
    strict_root = next((set(row) for row in strict_components if root in row), set())
    conservative_root = next((set(row) for row in conservative_components if root in row), set())
    gear_names = set()
    for pair in assembly.get("mesh_pairs", []):
        if isinstance(pair, list):
            gear_names.update(str(value) for value in pair[:2])
    gears = []
    for name in sorted(gear_names):
        mesh = meshes.get(name)
        gears.append({"name": name, **(gear_classification(mesh) if mesh is not None
                                       else {"status": "missing_mesh"})})
    return {"task_id": task.name, "root_link": root,
            "physical_links": physical, "declared_edges": measured,
            "strict_1_5mm": {"components": strict_components,
                             "floating_links": sorted(set(physical) - strict_root)},
            "conservative_10mm": {"components": conservative_components,
                                  "floating_links": sorted(set(physical) - conservative_root)},
            "gears": gears}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    summary = []
    for method in ("claude-code", "codex"):
        method_out = OUT / method
        method_out.mkdir(parents=True, exist_ok=True)
        for task in sorted(path for path in (ROOT / method).iterdir() if path.is_dir()):
            result = audit(task)
            (method_out / f"{task.name}.json").write_text(
                json.dumps(result, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
            gear_issues = [row["name"] + ":" + row["status"] for row in result["gears"]
                           if row["status"] != "integrated_tooth_geometry"]
            row = {"method": method, "task_id": task.name,
                   "floating_strict": len(result["strict_1_5mm"]["floating_links"]),
                   "floating_conservative": len(result["conservative_10mm"]["floating_links"]),
                   "unrealized_declared_edges": sum(edge["status"] == "geometrically_unrealized"
                                                    for edge in result["declared_edges"]),
                   "gear_issues": gear_issues}
            summary.append(row)
            print(method, task.name, "floating1.5", row["floating_strict"],
                  "floating10", row["floating_conservative"],
                  "unrealized", row["unrealized_declared_edges"],
                  "gears", gear_issues, flush=True)
    (OUT / "suite_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True),
                                             encoding="utf-8")


if __name__ == "__main__":
    main()
