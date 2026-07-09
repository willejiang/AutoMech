"""Golden solve test for the manager's CONNECTION GRAPH (Part A, mate_solver).

Gates the WHOLE of Part A: a hand-authored connection-graph IR (parts + mates), fed through
`mate_solver.solve_connection_graph` + `manager._validate_model`, must SOLVE to a
KinematicModel whose part WORLD positions (reconstructed via `assembler._root_to_link`) match
a known-good layout. Covers the cases raised in plan review:

  * two meshing gears (parallel spur): centers exactly r_a + r_b apart, axes parallel.
  * a gear TRAIN (3 gears): each mesh uses its own separation_axis (no default-direction guess).
  * perpendicular gears (axis_angle_deg=90): the two axes are 90 deg apart.
  * shaft through a gear's hole: coaxial(gear.bore, shaft.outer) -> colinear axes.
  * negative paths: a part in NO mate; a >1-peer gear missing separation_axis; an unknown
    port; a gear missing pitch radius -> all raise MateSolveError (caught by the retry loop).

Run:  python -m maker2.tests.golden_mate_solver_roundtrip
Exit 0 = all layouts + error paths correct; exit 1 = a mismatch (block the prompt flip).
"""
from __future__ import annotations

import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _ROOT)

from maker2.assembler import _root_to_link                       # noqa: E402
from maker2.manager import _validate_model                        # noqa: E402
from maker2.mate_solver import MateSolveError, solve_connection_graph  # noqa: E402

_TOL_MM = 1.0  # world-position tolerance for the layout assertions


def _worlds(model) -> dict:
    """{link_name: world xyz in mm} from the solved model's pose forest."""
    T = _root_to_link(model)
    return {n: np.asarray(m)[:3, 3] * 1000.0 for n, m in T.items()}


def _fail(msg: str):
    print(f"  FAIL: {msg}")
    sys.exit(1)


def _close(a, b, tol=_TOL_MM) -> bool:
    return float(np.linalg.norm(np.asarray(a) - np.asarray(b))) <= tol


# --------------------------------------------------------------------------- #
# Case 1: a single-stage gear reducer (two meshing gears + shafts through bores + a cap)
# --------------------------------------------------------------------------- #
_REDUCER = {
    "name": "single_stage_reducer",
    "root_part": "base",
    "parts": [
        {"name": "base", "shape_hint": "box", "size_mm": {"x": 120, "y": 80, "z": 10},
         "dof": "fixed"},
        {"name": "bearing_a", "shape_hint": "cylinder",
         "size_mm": {"radius": 12, "height": 8, "bore_dia": 8}, "dof": "fixed"},
        {"name": "input_shaft", "shape_hint": "cylinder", "size_mm": {"radius": 4, "height": 60},
         "dof": "spin", "spin_axis": [0, 0, 1], "driver": True},
        {"name": "pinion", "shape_hint": "gear",
         "size_mm": {"module": 2, "teeth": 20, "thickness": 10, "bore_dia": 8}, "dof": "spin",
         "spin_axis": [0, 0, 1]},
        {"name": "bearing_b", "shape_hint": "cylinder",
         "size_mm": {"radius": 12, "height": 8, "bore_dia": 10}, "dof": "fixed"},
        {"name": "output_shaft", "shape_hint": "cylinder", "size_mm": {"radius": 5, "height": 60},
         "dof": "spin", "spin_axis": [0, 0, 1]},
        {"name": "big_gear", "shape_hint": "gear",
         "size_mm": {"module": 2, "teeth": 60, "thickness": 10, "bore_dia": 10}, "dof": "spin",
         "spin_axis": [0, 0, 1]},
        {"name": "cap", "shape_hint": "cylinder", "size_mm": {"radius": 14, "height": 4},
         "dof": "fixed"},
    ],
    "mates": [
        {"name": "brgA_on_base", "mate_type": "face_to_face",
         "base_part": "base", "base_port": "face_pz",
         "incoming_part": "bearing_a", "incoming_port": "end_a"},
        {"name": "shaft_in_brgA", "mate_type": "coaxial",
         "base_part": "bearing_a", "base_port": "bore",
         "incoming_part": "input_shaft", "incoming_port": "outer"},
        {"name": "pinion_on_shaft", "mate_type": "coaxial",
         "base_part": "input_shaft", "base_port": "outer",
         "incoming_part": "pinion", "incoming_port": "bore", "offset_mm": 30},
        {"name": "gears_mesh", "mate_type": "gear_spur_external",
         "base_part": "pinion", "base_port": "teeth",
         "incoming_part": "big_gear", "incoming_port": "teeth", "separation_axis": "+x"},
        {"name": "biggear_on_outshaft", "mate_type": "coaxial",
         "base_part": "big_gear", "base_port": "bore",
         "incoming_part": "output_shaft", "incoming_port": "outer", "offset_mm": -30},
        {"name": "outshaft_in_brgB", "mate_type": "coaxial",
         "base_part": "output_shaft", "base_port": "outer",
         "incoming_part": "bearing_b", "incoming_port": "bore", "offset_mm": 30},
        {"name": "cap_on_brgA", "mate_type": "face_to_face",
         "base_part": "bearing_a", "base_port": "end_b",
         "incoming_part": "cap", "incoming_port": "end_a"},
    ],
}


def test_reducer():
    print("[1] single-stage reducer …")
    model = solve_connection_graph(_REDUCER)
    _validate_model(model)
    w = _worlds(model)
    for p in ("base", "bearing_a", "input_shaft", "pinion", "big_gear", "cap"):
        if p not in w:
            _fail(f"part '{p}' was not placed")
    # pinion & big_gear meshing: centers r_pinion(20) + r_big(60) = 80 mm apart.
    d = float(np.linalg.norm(w["pinion"] - w["big_gear"]))
    if abs(d - 80.0) > _TOL_MM:
        _fail(f"pinion~big_gear center distance {d:.1f} mm, expected 80 mm (r_a+r_b)")
    # meshing along +x: the two gear centers differ (mostly) in X.
    if abs((w["big_gear"] - w["pinion"])[0] - 80.0) > _TOL_MM:
        _fail(f"gears not separated along +x: delta={np.round(w['big_gear']-w['pinion'],1)}")
    # both shafts spin about +Z (parallel axes) — check via the solved pose rotation.
    T = _root_to_link(model)
    za = np.asarray(T["input_shaft"])[:3, 2]
    zb = np.asarray(T["output_shaft"])[:3, 2]
    if abs(abs(float(np.dot(za, zb))) - 1.0) > 1e-3:
        _fail(f"input/output shaft axes not parallel: z_a={np.round(za,3)} z_b={np.round(zb,3)}")
    print(f"    OK — gears 80mm apart on +x, shafts parallel, {len(w)} parts placed")


# --------------------------------------------------------------------------- #
# Case 2: a 3-gear TRAIN — the middle gear meshes TWO peers, each needs its own sep axis
# --------------------------------------------------------------------------- #
_TRAIN = {
    "name": "gear_train",
    "root_part": "g_mid",
    "parts": [
        {"name": "g_mid", "shape_hint": "gear",
         "size_mm": {"module": 2, "teeth": 30, "thickness": 8}, "dof": "spin"},
        {"name": "g_left", "shape_hint": "gear",
         "size_mm": {"module": 2, "teeth": 20, "thickness": 8}, "dof": "spin"},
        {"name": "g_right", "shape_hint": "gear",
         "size_mm": {"module": 2, "teeth": 20, "thickness": 8}, "dof": "spin"},
    ],
    "mates": [
        {"name": "mid_left", "mate_type": "gear_spur_external",
         "base_part": "g_mid", "base_port": "teeth",
         "incoming_part": "g_left", "incoming_port": "teeth", "separation_axis": "-x"},
        {"name": "mid_right", "mate_type": "gear_spur_external",
         "base_part": "g_mid", "base_port": "teeth",
         "incoming_part": "g_right", "incoming_port": "teeth", "separation_axis": "+x"},
    ],
}


def test_train():
    print("[2] 3-gear train (separation direction disambiguates) …")
    model = solve_connection_graph(_TRAIN)
    _validate_model(model)
    w = _worlds(model)
    # g_mid r=30, g_left r=20 -> 50mm on -x; g_right r=20 -> 50mm on +x.
    if not _close(w["g_left"] - w["g_mid"], [-50, 0, 0]):
        _fail(f"g_left not 50mm on -x: {np.round(w['g_left']-w['g_mid'],1)}")
    if not _close(w["g_right"] - w["g_mid"], [50, 0, 0]):
        _fail(f"g_right not 50mm on +x: {np.round(w['g_right']-w['g_mid'],1)}")
    print("    OK — left at -50x, right at +50x (no default-direction ambiguity)")


# --------------------------------------------------------------------------- #
# Case 3: perpendicular (bevel) gears — one horizontal + one vertical
# --------------------------------------------------------------------------- #
_BEVEL = {
    "name": "bevel_pair",
    "root_part": "g_horiz",
    "parts": [
        {"name": "g_horiz", "shape_hint": "gear",
         "size_mm": {"module": 2, "teeth": 20, "thickness": 8}, "dof": "spin",
         "spin_axis": [0, 0, 1]},
        {"name": "g_vert", "shape_hint": "gear",
         "size_mm": {"module": 2, "teeth": 20, "thickness": 8}, "dof": "spin"},
    ],
    "mates": [
        {"name": "bevel", "mate_type": "gear_bevel", "axis_angle_deg": 90,
         "base_part": "g_horiz", "base_port": "teeth",
         "incoming_part": "g_vert", "incoming_port": "teeth", "separation_axis": "+x"},
    ],
}


def test_bevel():
    print("[3] perpendicular bevel gears (axis_angle_deg=90) …")
    model = solve_connection_graph(_BEVEL)
    _validate_model(model)
    T = _root_to_link(model)
    za = np.asarray(T["g_horiz"])[:3, 2]
    zb = np.asarray(T["g_vert"])[:3, 2]
    dot = abs(float(np.dot(za, zb)))
    if dot > 1e-2:
        _fail(f"bevel axes not perpendicular: dot(z_a,z_b)={dot:.3f} (want ~0)")
    print(f"    OK — axes perpendicular (dot={dot:.3f})")


# --------------------------------------------------------------------------- #
# Case 4: shaft through a gear's hole
# --------------------------------------------------------------------------- #
_SHAFT_GEAR = {
    "name": "shaft_through_gear",
    "root_part": "gear",
    "parts": [
        {"name": "gear", "shape_hint": "gear",
         "size_mm": {"module": 2, "teeth": 24, "thickness": 10, "bore_dia": 8}, "dof": "fixed"},
        {"name": "shaft", "shape_hint": "cylinder", "size_mm": {"radius": 4, "height": 80},
         "dof": "spin", "spin_axis": [0, 0, 1]},
    ],
    "mates": [
        {"name": "shaft_in_gear", "mate_type": "coaxial",
         "base_part": "gear", "base_port": "bore",
         "incoming_part": "shaft", "incoming_port": "outer"},
    ],
}


def test_shaft_through_gear():
    print("[4] shaft through a gear's hole …")
    model = solve_connection_graph(_SHAFT_GEAR)
    _validate_model(model)
    T = _root_to_link(model)
    za = np.asarray(T["gear"])[:3, 2]
    zb = np.asarray(T["shaft"])[:3, 2]
    if abs(abs(float(np.dot(za, zb))) - 1.0) > 1e-3:
        _fail(f"shaft axis not colinear with gear bore: dot={float(np.dot(za,zb)):.3f}")
    # centers coincide (both on the bore axis line at the same origin, before offset).
    w = _worlds(model)
    radial = float(np.linalg.norm((w["shaft"] - w["gear"])[:2]))
    if radial > _TOL_MM:
        _fail(f"shaft not concentric with gear: radial offset {radial:.1f} mm")
    print("    OK — shaft colinear + concentric with the gear bore")


# --------------------------------------------------------------------------- #
# Negative paths — every mis-authored graph must raise MateSolveError
# --------------------------------------------------------------------------- #

def _expect_error(ir, substr, label):
    try:
        solve_connection_graph(ir)
    except MateSolveError as e:
        if substr.lower() not in str(e).lower():
            _fail(f"{label}: error did not mention '{substr}': {e}")
        return
    _fail(f"{label}: expected MateSolveError, got none")


def test_negatives():
    print("[5] negative paths …")
    # (a) a part in NO mate.
    orphan = {"name": "x", "root_part": "a",
              "parts": [{"name": "a", "shape_hint": "box", "size_mm": {"x": 10, "y": 10, "z": 10}},
                        {"name": "b", "shape_hint": "box", "size_mm": {"x": 10, "y": 10, "z": 10}}],
              "mates": []}
    _expect_error(orphan, "no mate", "orphan part")
    # (b) a gear meshing >1 peer with a mate missing separation_axis.
    train_bad = {k: v for k, v in _TRAIN.items()}
    train_bad["mates"] = [dict(m) for m in _TRAIN["mates"]]
    del train_bad["mates"][0]["separation_axis"]
    _expect_error(train_bad, "separation_axis", "missing separation axis")
    # (c) a mate naming an unknown port.
    bad_port = {"name": "x", "root_part": "a",
                "parts": [{"name": "a", "shape_hint": "cylinder", "size_mm": {"radius": 5, "height": 10}},
                          {"name": "b", "shape_hint": "cylinder", "size_mm": {"radius": 5, "height": 10}}],
                "mates": [{"name": "m", "mate_type": "coaxial", "base_part": "a",
                           "base_port": "nonesuch", "incoming_part": "b", "incoming_port": "outer"}]}
    _expect_error(bad_port, "nonesuch", "unknown port")
    # (d) a gear with no pitch radius.
    no_pitch = {"name": "x", "root_part": "a",
                "parts": [{"name": "a", "shape_hint": "gear", "size_mm": {"thickness": 8}},
                          {"name": "b", "shape_hint": "gear", "size_mm": {"thickness": 8}}],
                "mates": [{"name": "m", "mate_type": "gear_spur_external", "base_part": "a",
                           "base_port": "teeth", "incoming_part": "b", "incoming_port": "teeth"}]}
    _expect_error(no_pitch, "pitch radius", "missing pitch radius")
    print("    OK — orphan / missing-sep-axis / unknown-port / no-pitch all raise")


def test_shipped_fewshot():
    print("[6] shipped IR_FEWSHOT_JSON solves …")
    import json as _json

    from maker2.prompts.schema import IR_FEWSHOT_JSON
    model = solve_connection_graph(_json.loads(IR_FEWSHOT_JSON))
    _validate_model(model)
    names = {l.name for l in model.links}
    if names != {"base", "bearing_block", "shaft", "platter"}:
        _fail(f"shipped IR few-shot parts changed: {sorted(names)}")
    print(f"    OK — shipped few-shot solves to {len(model.links)} parts, root='{model.root_link}'")


def main() -> int:
    print("golden mate-solver round-trip\n" + "=" * 40)
    test_reducer()
    test_train()
    test_bevel()
    test_shaft_through_gear()
    test_negatives()
    test_shipped_fewshot()
    print("=" * 40 + "\nALL GOLDEN CASES PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
