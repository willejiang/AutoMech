"""Focused task #115 validation goldens.

Run: python -m maker2.tests.golden_task115_validation
"""
from __future__ import annotations

import tempfile
from pathlib import Path

import trimesh

from maker2.manager import ManagerError, _validate_model, parse_model
from maker2.mjcf_facts import query_pair_geometry
from maker2.mjcf_validation import execute_compiler, validate_candidate
from maker2.model import KinematicModel, LinkSpec, TransmissionSpec
from maker2.precheck import _solid_intersection_frac


def _frame(x_mm: float = 0.0) -> dict:
    x_m = x_mm / 1000.0
    return {"xyz_m": [x_m, 0.0, 0.0], "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
            "matrix": [[1.0, 0.0, 0.0, x_m], [0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]}


def _write_box(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    trimesh.creation.box(extents=(10.0, 10.0, 10.0)).export(path)


def _facts(root: Path, *, relation: dict | None = None,
           mesh_pair: bool = False) -> dict:
    for name in ("a", "b"):
        _write_box(root / "meshes" / f"{name}.stl")
    relations = [relation] if relation else []
    entity_ids = ["link/a", "link/b"]
    ports = {}
    if relation:
        entity_ids.append(f"relation/{relation['name']}")
        if relation.get("mate_type") == "press_fit":
            ports = {
                "a": {"shaft": {"type": "shaft", "diameter_mm": 6.0,
                                  "world_frame": _frame()}},
                "b": {"bore": {"type": "bore", "diameter_mm": 5.9,
                                  "world_frame": _frame()}},
            }
            entity_ids += ["port/a/shaft", "port/b/bore"]
    return {
        "run_dir": str(root), "entity_ids": entity_ids,
        "model": {"name": "task115", "root_link": "a",
                  "links": [{"name": "a"}, {"name": "b"}],
                  "mesh_pairs": [["a", "b"]] if mesh_pair else [],
                  "relations": relations},
        "ports": ports,
        "links": {
            name: {"world_frame": _frame(), "mesh_path": f"meshes/{name}.stl",
                   "mass_kg": 1.0, "com_m": [0.0, 0.0, 0.0],
                   "inertia_kg_m2": [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0],
                                     [0.0, 0.0, 0.1]],
                   "friction": [1.0, 0.05, 0.005], "dof": "fixed",
                   "bounds_mm": [[-5.0, -5.0, -5.0], [5.0, 5.0, 5.0]],
                   "extents_mm": [10.0, 10.0, 10.0]}
            for name in ("a", "b")},
        "simulation": {"gravity": [0.0, 0.0, -9.81], "timestep": 0.001,
                       "solver": "Newton", "iterations": 10}}


def _source(source_ids: list[str]) -> str:
    return f'''
def compile_mjcf(facts,out):
    out.topology_plan({{"support_ground":"a","coordinate_map":{{}},"tree_edges":[],"closure_edges":[],"rigid_carried":[],"independent_coaxial":[],"transmissions":[],"contact_decisions":[{{"pair":["a","b"],"action":"exclude","reason":"fixture","source_entity_ids":{source_ids!r},"fact_ids":["pair/a/b"]}}],"support_strategy":[]}})
    out.body("a")
    out.body("b")
    out.exclude("a","b","fixture",{source_ids!r},["pair/a/b"])
    out.decision("link/a","emitted",["a"],"fixture",["link/a"])
    out.decision("link/b","emitted",["b"],"fixture",["link/b"])
'''


def transmission_ratio_gate() -> None:
    prefix = ('{"name":"x","root_link":"a","links":['
              '{"name":"a","description":"a"},{"name":"b","description":"b"}],'
              '"poses":[],"transmissions":[')
    suffix = ']}'
    base = ('{"name":"g","type":"gear_external","driving_link":"a",'
            '"driven_link":"b"')
    for tail in ("}", ',"ratio":0}', ',"ratio":1e999}'):
        try:
            model = parse_model(prefix + base + tail + suffix)
            _validate_model(model)
        except (ValueError, ManagerError):
            pass
        else:
            raise AssertionError(f"invalid ratio accepted: {tail}")
    valid = parse_model(prefix + base + ',"ratio":-0.25}' + suffix)
    _validate_model(valid)


def cad_gate_fails_closed() -> None:
    a = trimesh.creation.box(extents=(10.0, 10.0, 10.0))
    b = trimesh.creation.box(extents=(10.0, 10.0, 10.0))
    b.apply_translation((2.0, 0.0, 0.0))
    original = trimesh.boolean.intersection
    trimesh.boolean.intersection = lambda *args, **kwargs: (_ for _ in ()).throw(
        RuntimeError("boolean unavailable"))
    try:
        assert _solid_intersection_frac(a, b, fail_closed=True) == 1.0
    finally:
        trimesh.boolean.intersection = original


def excludes_require_geometry_proof() -> None:
    root = Path(tempfile.mkdtemp(prefix="golden_task115_"))
    facts = _facts(root)
    xml, manifest = execute_compiler(_source(["link/a", "link/b"]), facts)
    rejected = validate_candidate(xml, manifest, facts, root / "bad.mjcf", run_smoke=False)
    assert not rejected["ok"] and any("unsupported by geometry facts" in error
                                      for error in rejected["errors"]), rejected

    # Even a sampled positive surface distance cannot rescue an overlapping-AABB pair
    # whose exact solid boolean is unavailable.
    original = trimesh.boolean.intersection
    trimesh.boolean.intersection = lambda *args, **kwargs: (_ for _ in ()).throw(
        RuntimeError("boolean unavailable"))
    try:
        geometry = query_pair_geometry(facts, "a", "b")
        assert all(x > 0.0 for x in geometry["aabb_overlap_extents_mm"])
        assert geometry["solid_overlap_mm3"] is None
        unavailable = validate_candidate(xml, manifest, facts, root / "null.mjcf",
                                         run_smoke=False)
        assert not unavailable["ok"] and any("exact solid overlap is unavailable" in error
                                              for error in unavailable["errors"]), unavailable
    finally:
        trimesh.boolean.intersection = original

    relation = {"name": "fit", "mate_type": "press_fit", "base_part": "a",
                "base_port": "shaft", "incoming_part": "b", "incoming_port": "bore"}
    exempt = _facts(root / "press", relation=relation)
    exempt_source = _source(["link/a", "link/b", "relation/fit"])
    exempt_source += ('    out.decision("relation/fit","represented_by",["a"],'
                      '"fixture",["relation/fit"])\n'
                      '    out.decision("port/a/shaft","represented_by",["a"],'
                      '"fixture",["port/a/shaft"])\n'
                      '    out.decision("port/b/bore","represented_by",["b"],'
                      '"fixture",["port/b/bore"])\n')
    xml2, manifest2 = execute_compiler(exempt_source, exempt)
    accepted = validate_candidate(xml2, manifest2, exempt, root / "press.mjcf",
                                  run_smoke=False)
    assert accepted["ok"], accepted

    # Gear exemption is exact: both a declared gear relation and mesh_pairs membership.
    gear_relation = {"name": "mesh", "mate_type": "gear_spur_external",
                     "base_part": "a", "incoming_part": "b"}
    gear = _facts(root / "gear", relation=gear_relation, mesh_pair=True)
    gear_source = _source(["link/a", "link/b", "relation/mesh"])
    gear_source += ('    out.decision("relation/mesh","represented_by",["a"],'
                    '"fixture",["relation/mesh"])\n')
    xml3, manifest3 = execute_compiler(gear_source, gear)
    accepted_gear = validate_candidate(xml3, manifest3, gear, root / "gear.mjcf",
                                       run_smoke=False)
    assert accepted_gear["ok"], accepted_gear


def main() -> None:
    transmission_ratio_gate()
    cad_gate_fails_closed()
    excludes_require_geometry_proof()
    print("golden task 115 validation: PASS")


if __name__ == "__main__":
    main()
