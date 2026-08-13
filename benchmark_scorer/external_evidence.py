"""Scorer-owned invariant evidence for normalized external-harness submissions."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping

from .geometry import _world_transforms
from .metrics import (axis_drift, carrying_distance_std, circularity_residual,
                      lateral_drift, net_angular_travel, pair_distance_variation, span)
from .tasks.comfort_v1 import get_task


def _positions(trajectory: Mapping[str, Any], name: str) -> list[list[float]] | None:
    raw = (trajectory.get("bodies") or {}).get(name)
    if isinstance(raw, Mapping): raw = raw.get("position", raw.get("positions"))
    if not isinstance(raw, list) or not raw: return None
    try: return [[float(value) for value in row[:3]] for row in raw]
    except (TypeError, ValueError, IndexError): return None


def _joint(trajectory: Mapping[str, Any], name: str) -> list[float] | None:
    raw = (trajectory.get("joints") or {}).get(name)
    if isinstance(raw, Mapping): raw = raw.get("qpos", raw.get("position"))
    if not isinstance(raw, list) or not raw: return None
    try: return [float(row[0] if isinstance(row, list) else row) for row in raw]
    except (TypeError, ValueError, IndexError): return None


def _role(bindings: Mapping[str, Any], name: str) -> list[str]:
    raw = bindings.get(name, ())
    if isinstance(raw, str): return [raw]
    return [str(value) for value in raw] if isinstance(raw, list) else []


def _relations(assembly: Mapping[str, Any]) -> dict[frozenset[str], set[str]]:
    out: dict[frozenset[str], set[str]] = {}
    for row in assembly.get("relations", ()):
        if not isinstance(row, Mapping): continue
        a, b = row.get("base_part"), row.get("incoming_part")
        if a and b: out.setdefault(frozenset((str(a), str(b))), set()).add(str(row.get("mate_type", "")))
    for row in assembly.get("links", ()):
        if isinstance(row, Mapping) and row.get("name") and row.get("mount"):
            out.setdefault(frozenset((str(row["name"]), str(row["mount"]))), set()).add("rigid_mount")
    return out


def _machine_diagonal_mm(root: Path, assembly: Mapping[str, Any]) -> float:
    import numpy as np
    import trimesh
    transforms = _world_transforms(assembly)
    lower = np.array([np.inf, np.inf, np.inf]); upper = -lower
    for row in assembly.get("links", ()):
        if not isinstance(row, Mapping) or not row.get("mesh_filename"): continue
        mesh = trimesh.load_mesh(root / str(row["mesh_filename"]), force="mesh")
        mesh.apply_transform(transforms[str(row["name"])])
        lower = np.minimum(lower, mesh.bounds[0]); upper = np.maximum(upper, mesh.bounds[1])
    value = float(np.linalg.norm(upper - lower))
    return value if math.isfinite(value) and value > 0 else 100.0


def _fixed_joint(assembly: Mapping[str, Any], joint_name: str) -> bool:
    links = {str(row.get("name")): row for row in assembly.get("links", ()) if isinstance(row, Mapping)}
    for row in assembly.get("motion_joints", ()):
        if not isinstance(row, Mapping) or row.get("name") != joint_name: continue
        parent = links.get(str(row.get("parent", "")))
        kind = str(row.get("type", row.get("kind", "")))
        return kind in {"spin", "revolute", "hinge"} and bool(parent) and parent.get("dof") == "fixed"
    return False


def _closure_ok(replay: Mapping[str, Any], names: list[str], limit_mm: float) -> bool:
    residuals = replay.get("equality_residuals") or {}
    if not names or not isinstance(residuals, Mapping): return False
    for name in names:
        values = residuals.get(name)
        if not isinstance(values, list) or not values: return False
        # MuJoCo constraint positions are metres for connect/weld residuals.
        if max(abs(float(value)) for value in values) * 1000.0 > limit_mm: return False
    return True


def _contact_ground_clear(contacts: Mapping[str, Any], moving: set[str]) -> bool:
    def rows(value):
        if isinstance(value, Mapping):
            if any(key in value for key in ("body1", "body2", "geom1", "geom2")): yield value
            for child in value.values(): yield from rows(child)
        elif isinstance(value, list):
            for child in value: yield from rows(child)
    for row in rows(contacts):
        text = " ".join(str(row.get(key, "")) for key in ("body1", "body2", "geom1", "geom2")).casefold()
        if ("ground" in text or "world" in text) and any(name.casefold() in text for name in moving):
            return False
    return True


def derive_external_invariants(root: str | Path, assembly: Mapping[str, Any],
                               bindings: Mapping[str, Any], trajectory: Mapping[str, Any],
                               replay: Mapping[str, Any], contacts: Mapping[str, Any]) -> dict[str, bool]:
    root = Path(root); task_id = root.name; task = get_task(task_id)
    links = {str(row.get("name")): row for row in assembly.get("links", ()) if isinstance(row, Mapping)}
    relations = _relations(assembly); diagonal = _machine_diagonal_mm(root, assembly)
    closure_limit = 0.02 * diagonal
    result: dict[str, bool] = {}

    def rigid_pair(a: str, b: str) -> bool:
        return bool(relations.get(frozenset((a, b)), set()) & {"press_fit", "rigid_carry", "rigid_mount", "fixed", "weld"})
    def fixed_role(role: str) -> bool:
        names = _role(bindings, role); return len(names) == 1 and _fixed_joint(assembly, names[0])
    def body_for(role: str) -> str | None:
        names = _role(bindings, role)
        if not names: return None
        if _positions(trajectory, names[0]): return names[0]
        folded = names[0].casefold().replace("_hinge", "").replace("_slide", "")
        candidates = [name for name in (trajectory.get("bodies") or {})
                      if folded in str(name).casefold() or str(name).casefold() in folded]
        return min(candidates, key=lambda value: (len(str(value)), str(value))) if candidates else None

    if task_id == "01_single_stage_4to1":
        result["fixed_shaft_axes"] = fixed_role("input_shaft") and fixed_role("output_shaft")
        gears = _role(bindings, "gear")
        result["rigid_gear_carrying"] = (len(gears) == 2 and
            any(rigid_pair(gears[0], name) for name in links) and any(rigid_pair(gears[1], name) for name in links))
        result["single_mesh"] = sum("gear" in kind for kinds in relations.values() for kind in kinds) == 1
    elif task_id == "02_two_stage_9to1":
        result["fixed_shaft_axes"] = all(fixed_role(role) for role in
                                           ("input_shaft", "compound_intermediate_shaft", "output_shaft"))
        gears = _role(bindings, "gear"); intermediate = body_for("compound_intermediate_shaft") or ""
        result["rigid_compound_pair"] = len(gears) == 4 and rigid_pair(gears[1], intermediate) and rigid_pair(gears[2], intermediate)
        result["two_live_meshes"] = sum("gear" in kind for kinds in relations.values() for kind in kinds) == 2
    elif task_id == "03_idler_reverser_1to1":
        result["fixed_shaft_axes"] = all(fixed_role(role) for role in ("input_shaft", "idler_shaft", "output_shaft"))
        joints = _role(bindings, "input_shaft") + _role(bindings, "idler_shaft") + _role(bindings, "output_shaft")
        result["independent_idler_hinge"] = len(joints) == 3 and len(set(joints)) == 3
        result["two_live_meshes"] = sum("gear" in kind for kinds in relations.values() for kind in kinds) == 2
    elif task_id == "04_openwork_clock_12to1":
        minute, hour = _role(bindings, "minute_input"), _role(bindings, "hour_output")
        result["coaxial_independent_hands"] = bool(minute and hour and minute[0] != hour[0])
        hands = _role(bindings, "coaxial_hand")
        result["hands_remain_carried"] = len(hands) == 2 and all(
            links.get(name, {}).get("mount") or any(rigid_pair(name, other) for other in links)
            for name in hands)
    elif task_id in {"05_three_planet_4to1", "06_four_planet_4to1"}:
        count = 3 if task_id.startswith("05_") else 4
        ring = body_for("fixed_ring") or (_role(bindings, "fixed_ring") or [None])[0]
        ring_pos = _positions(trajectory, ring) if ring else None
        result["ring_fixed"] = bool(
            ring and links.get(ring, {}).get("dof") == "fixed"
            and (not ring_pos or axis_drift(ring_pos) <= max(1.0, 0.01 * diagonal)))
        planets = _role(bindings, "planet_gear"); pins = _role(bindings, "planet_pin_hinge")
        planet_bodies = [name for name in planets if _positions(trajectory, name)]
        pin_bodies = []
        for joint in pins:
            stem = joint.replace("_hinge", "")
            candidates = [name for name in (trajectory.get("bodies") or {}) if stem in str(name) or str(name) in stem]
            pin_bodies.append(min(candidates, key=lambda value: len(str(value))) if candidates else "")
        result["planet_orbit"] = len(planet_bodies) == count and all(
            span([row[0] for row in _positions(trajectory, name)]) > 1.0 or
            span([row[2] for row in _positions(trajectory, name)]) > 1.0 for name in planet_bodies)
        result["planet_local_spin"] = len(pins) == count and all(
            (_joint(trajectory, name) is not None and abs(net_angular_travel(_joint(trajectory, name))) > 0.1) for name in pins)
        result["planet_pin_distance_constant"] = (len(planet_bodies) == count and len(pin_bodies) == count
            and all(pin and pair_distance_variation(_positions(trajectory, planet), _positions(trajectory, pin)) <= max(0.5, 0.005 * diagonal)
                    for planet, pin in zip(planet_bodies, pin_bodies)))
        transforms = _world_transforms(assembly)
        centers = [transforms[name][:3, 3] * 1000.0 for name in planets
                   if name in transforms]
        carrier_bound = (_role(bindings, "carrier_output") or [None])[0]
        carrier_link = next((str(row.get("child")) for row in assembly.get("motion_joints", ())
                             if isinstance(row, Mapping) and row.get("name") == carrier_bound), None)
        carrier_center = (transforms.get(carrier_link)[:3, 3] * 1000.0
                          if carrier_link in transforms else None)
        spacing = False
        if len(centers) == count and carrier_center is not None:
            deltas = [center - carrier_center for center in centers]
            # Pick the two coordinate axes with the largest aggregate radial spread;
            # planetary planes may be XY, XZ, or YZ.
            spreads = [sum(float(delta[axis]) ** 2 for delta in deltas) for axis in range(3)]
            axes = sorted(range(3), key=lambda axis: spreads[axis], reverse=True)[:2]
            angles = sorted(math.atan2(float(delta[axes[1]]), float(delta[axes[0]])) % (2*math.pi)
                            for delta in deltas)
            gaps = [((angles[(i+1)%count]-angles[i]) % (2*math.pi)) for i in range(count)]
            spacing = max(abs(gap - 2*math.pi/count) for gap in gaps) <= math.radians(2)
        result["equally_spaced" if count == 3 else "spacing_90_deg"] = spacing
    elif task_id == "07_horizontal_slider_crank":
        result["fixed_crank_axis"] = fixed_role("crankshaft_input")
        slider_name = body_for("horizontal_slider"); slider = _positions(trajectory, slider_name) if slider_name else None
        result["lateral_drift_le_2pct_span"] = bool(slider) and lateral_drift(slider, (1,0,0)) <= max(0.5, span([row[0] for row in slider]) * .02)
        names = [name for name in (replay.get("equality_residuals") or {}) if "closure" in name or "connect" in name]
        result["closures_below_2pct_scale"] = _closure_ok(replay, names, closure_limit)
    elif task_id == "08_vertical_piston_pump":
        result["fixed_crank_axis"] = fixed_role("crankshaft_input")
        names = [name for name in (replay.get("equality_residuals") or {}) if "crosshead" in name or "closure" in name]
        result["rod_crosshead_closure"] = _closure_ok(replay, names, closure_limit)
        bodies = [body_for(role) for role in ("vertical_crosshead", "pump_rod", "piston_output")]
        result["rigid_output_carrying"] = all(bodies) and all(
            carrying_distance_std(_positions(trajectory, a), _positions(trajectory, b)) <= max(.5,.005*diagonal)
            for a,b in zip(bodies,bodies[1:]))
        result["no_ground_collision"] = _contact_ground_clear(contacts, set(name for name in bodies if name))
    elif task_id == "09_open_pumpjack":
        result["fixed_crank_axis"] = fixed_role("crankshaft_input")
        result["fixed_beam_pivot"] = fixed_role("beam_pivot")
        beam_joint = _role(bindings,"walking_beam"); result["beam_pivots"] = bool(beam_joint and _joint(trajectory,beam_joint[0]) and span(_joint(trajectory,beam_joint[0])) > .01)
        names = [name for name in (replay.get("equality_residuals") or {}) if "closure" in name or "connect" in name]
        result["closures_below_2pct_scale"] = _closure_ok(replay,names,closure_limit)
        output_name=body_for("polished_rod_output"); output=_positions(trajectory,output_name) if output_name else None
        result["lateral_drift_le_5pct_span"] = bool(output) and lateral_drift(output,(0,0,1)) <= max(.5,span([row[2] for row in output])*.05)
    elif task_id == "10_wind_rotor_pump":
        result["fixed_rotor_axis"] = fixed_role("rotor_shaft_input")
        pin_name=body_for("crank_pin"); pin=_positions(trajectory,pin_name) if pin_name else None
        result["circular_crank_pin_path"] = bool(pin) and circularity_residual(pin) <= max(1.0,.01*diagonal)
        output_name=body_for("piston_output"); output=_positions(trajectory,output_name) if output_name else None
        result["vertical_output"] = bool(output) and lateral_drift(output,(0,0,1)) <= max(.5,span([row[2] for row in output])*.05)
        rod=body_for("pump_rod"); result["rigid_output_carrying"] = bool(rod and output_name) and carrying_distance_std(_positions(trajectory,rod),output) <= max(.5,.005*diagonal)
        names=[name for name in (replay.get("equality_residuals") or {}) if "closure" in name or "connect" in name]
        result["closures_below_2pct_scale"] = _closure_ok(replay,names,closure_limit)
    return {name: bool(result.get(name, False)) for name in task.invariants}


__all__ = ["derive_external_invariants"]
