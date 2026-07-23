"""Deterministic parallel-shaft two-stage reducer template."""
from __future__ import annotations

import math

from ..catalog import Catalog, CatalogError
from ..contracts import FunctionalEnvelope, Hardpoint, HardpointContract
from ..ir import DesignIntentIR, RequirementFact, SelectedComponent, fingerprint
from ..parameter_dag import ParameterDAG, ParameterValue
from ..problem_builder import build_parallel_shaft_problem
from .base import DesignTemplate, TemplateCandidate

TEMPLATE_ID = "parallel_shaft_two_stage_reducer_v1"
_ROLES = ("housing", "input_stage", "intermediate_stage", "output_stage")
_I4 = ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
       (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def _transform(x: float, y: float, z: float):
    return ((1.0, 0.0, 0.0, x), (0.0, 1.0, 0.0, y),
            (0.0, 0.0, 1.0, z), (0.0, 0.0, 0.0, 1.0))


class ParallelShaftTwoStageReducerTemplate(DesignTemplate):
    id = TEMPLATE_ID
    layouts = ("linear", "zig_zag")
    required_roles = _ROLES

    def validate_intent(self, intent, facts, catalog):
        errors = []
        if intent.template_id != self.id:
            errors.append(f"unsupported template '{intent.template_id}'")
        roles = intent.role_map()
        missing = sorted(set(self.required_roles) - set(roles))
        if missing:
            errors.append(f"missing topology roles: {missing}")
        if len(set(roles.values())) != len(roles):
            errors.append("topology roles must bind distinct subassemblies")
        if intent.layout not in self.layouts:
            errors.append(f"unsupported reducer layout '{intent.layout}'")
        fact_ids = {fact.id for fact in facts}
        unknown = sorted(set(intent.requirement_fact_ids) - fact_ids)
        if unknown:
            errors.append(f"unknown requirement facts: {unknown}")
        if any(f.kind == "ratio" and f.value <= 1 for f in facts if f.id in intent.requirement_fact_ids):
            errors.append("reducer ratio must be greater than one")
        for profile_id in intent.standards_profile_ids:
            if not any(entry.get("id") == profile_id for section in
                       ("gear_profiles", "gear_face_width_rules", "fit_profiles",
                        "clearance_profiles", "housing_wall_profiles") for entry in catalog.entries(section)):
                errors.append(f"unknown standards profile '{profile_id}'")
        return tuple(errors)

    def enumerate_candidates(self, intent, facts, catalog):
        ratio = next((f.value for f in facts if f.id in intent.requirement_fact_ids and f.kind == "ratio"), 9.0)
        modules = catalog.entries("gear_modules_mm")
        tooth_options = tuple(range(12, 61))
        stage_target = math.sqrt(ratio)
        stage_pairs = sorted(((abs(g / p - stage_target), p + g, p, g)
                              for p in tooth_options for g in tooth_options if g > p))[:32]
        raw = []
        for module in modules:
            for _e1, _s1, p1, g1 in stage_pairs:
                for _e2, _s2, p2, g2 in stage_pairs:
                    raw.append({"module_mm": module, "pinion_1": p1, "gear_1": g1,
                                "pinion_2": p2, "gear_2": g2})
        raw.sort(key=lambda c: (abs((c["gear_1"] / c["pinion_1"]) *
                                     (c["gear_2"] / c["pinion_2"]) - ratio),
                                c["module_mm"] * (c["pinion_1"] + c["gear_1"] +
                                                  c["pinion_2"] + c["gear_2"]),
                                c["module_mm"], c["pinion_1"], c["gear_1"],
                                c["pinion_2"], c["gear_2"]))
        profile = catalog.by_id("gear_profiles", "spur_20deg_full_depth")
        family_ids = set(intent.allowed_component_family_ids)
        shaft_family = "shaft_metric_light" if not family_ids else next(
            (family for family in sorted(family_ids) if family == "shaft_metric_light"), "")
        bearing_family = "bearing_6000" if not family_ids else next(
            (family for family in sorted(family_ids) if family == "bearing_6000"), "")
        if not shaft_family or not bearing_family:
            raise CatalogError("reducer requires shaft_metric_light and bearing_6000 families")
        candidates = []
        for index, choice in enumerate(raw):
            actual = (choice["gear_1"] / choice["pinion_1"]) * (choice["gear_2"] / choice["pinion_2"])
            stage_max = max(choice["gear_1"] / choice["pinion_1"],
                            choice["gear_2"] / choice["pinion_2"])
            shaft_diameter = next((d for d in catalog.by_id("shaft_families", shaft_family)["diameters_mm"]
                                   if d >= 8 + 2 * max(0, math.ceil(stage_max - 3))), None)
            if shaft_diameter is None:
                continue
            bearing = catalog.bearing_for_shaft(bearing_family, shaft_diameter)
            selected = (
                SelectedComponent("gear_profile", profile["id"], profile["kind"]),
                SelectedComponent("shaft_family", shaft_family, shaft_family,
                                  (("diameter_mm", shaft_diameter),)),
                SelectedComponent("bearing", bearing["id"], bearing_family,
                                  tuple(sorted((k, v) for k, v in bearing.items() if k != "id"))),
            )
            inputs = tuple(sorted({**choice, "target_ratio": ratio, "actual_ratio": actual,
                                   "shaft_diameter_mm": shaft_diameter,
                                   "bearing_outer_mm": bearing["outer_mm"],
                                   "bearing_width_mm": bearing["width_mm"]}.items()))
            candidates.append(TemplateCandidate(selected, inputs,
                              (abs(actual - ratio), choice["module_mm"] *
                               (choice["pinion_1"] + choice["gear_1"] +
                                choice["pinion_2"] + choice["gear_2"]), index),
                              fingerprint(inputs, "reducer_candidate_v1")))
            if len(candidates) >= 64:
                break
        return tuple(sorted(candidates, key=lambda candidate: candidate.score))

    def build_parameter_graph(self, intent, candidate, facts, catalog):
        graph = ParameterDAG()
        fact_ids = {fact.id for fact in facts}
        for name, value in candidate.inputs:
            graph.add_input(name, value, "1" if "teeth" in name or "ratio" in name else "mm",
                            tuple(sorted(set(intent.requirement_fact_ids) & fact_ids)) or (candidate.candidate_id,))
        wall = catalog.by_id("housing_wall_profiles", "housing_printed_light")
        face = catalog.by_id("gear_face_width_rules", "spur_face_10m")
        graph.add_input("housing_wall_mm", wall["minimum_wall_mm"], "mm", (wall["id"],))
        graph.add_input("housing_end_clearance_mm", wall["end_clearance_mm"], "mm", (wall["id"],))
        graph.add_input("face_multiplier", face["module_multiplier"], "1", (face["id"],))
        graph.add_input("face_minimum_mm", face["minimum_mm"], "mm", (face["id"],))
        graph.add_input("layout", intent.layout, "id", (f"choice:layout:{intent.layout}",))
        for gear in ("pinion_1", "gear_1", "pinion_2", "gear_2"):
            graph.add_formula(f"{gear}_pitch_diameter_mm", "mm", ("module_mm", gear), lambda m, z: m * z)
        graph.add_formula("stage_1_ratio", "1", ("gear_1", "pinion_1"), lambda g, p: g / p)
        graph.add_formula("stage_2_ratio", "1", ("gear_2", "pinion_2"), lambda g, p: g / p)
        graph.add_formula("compiled_ratio", "1", ("stage_1_ratio", "stage_2_ratio"), lambda a, b: a * b)
        graph.add_formula("stage_1_center_distance_mm", "mm", ("module_mm", "pinion_1", "gear_1"),
                          lambda m, a, b: m * (a + b) / 2.0)
        graph.add_formula("stage_2_center_distance_mm", "mm", ("module_mm", "pinion_2", "gear_2"),
                          lambda m, a, b: m * (a + b) / 2.0)
        graph.add_formula("face_width_mm", "mm", ("module_mm", "face_multiplier", "face_minimum_mm"),
                          lambda m, factor, minimum: max(minimum, m * factor))
        graph.add_formula("bearing_plane_front_mm", "mm", ("housing_end_clearance_mm", "bearing_width_mm"),
                          lambda clearance, width: clearance + width / 2.0)
        graph.add_formula("stage_1_gear_plane_mm", "mm", ("bearing_plane_front_mm", "bearing_width_mm", "face_width_mm"),
                          lambda front, bearing_width, face_width: front + bearing_width / 2.0 + face_width / 2.0)
        graph.add_formula("stage_2_gear_plane_mm", "mm", ("stage_1_gear_plane_mm", "face_width_mm", "housing_end_clearance_mm"),
                          lambda first, face_width, clearance: first + face_width + clearance)
        graph.add_formula("bearing_plane_rear_mm", "mm", ("stage_2_gear_plane_mm", "face_width_mm", "bearing_width_mm"),
                          lambda second, face_width, bearing_width: second + face_width / 2.0 + bearing_width / 2.0)
        graph.add_formula("shaft_length_mm", "mm", ("bearing_plane_rear_mm", "housing_end_clearance_mm", "bearing_width_mm"),
                          lambda rear, clearance, width: rear + clearance + width / 2.0)
        graph.add_formula("housing_x_mm", "mm", ("shaft_length_mm", "housing_wall_mm"), lambda length, wall: length + 2 * wall)
        graph.add_formula("housing_y_mm", "mm", ("layout", "stage_1_center_distance_mm",
                                                   "stage_2_center_distance_mm", "bearing_outer_mm",
                                                   "housing_wall_mm"),
                          lambda layout, c1, c2, outer, wall:
                              c1 + (c2 if layout == "linear" else 0.0) + outer + 2 * wall)
        graph.add_formula("housing_z_mm", "mm", ("layout", "stage_2_center_distance_mm",
                                                   "bearing_outer_mm", "housing_wall_mm"),
                          lambda layout, c2, outer, wall:
                              (c2 if layout == "zig_zag" else 0.0) + outer + 2 * wall)
        return graph

    def _stage_points(self, intent, values):
        c1 = values["stage_1_center_distance_mm"].value / 1000.0
        c2 = values["stage_2_center_distance_mm"].value / 1000.0
        if intent.layout == "linear":
            return {"input_stage": (0.0, 0.0, 0.0), "intermediate_stage": (0.0, c1, 0.0),
                    "output_stage": (0.0, c1 + c2, 0.0)}
        return {"input_stage": (0.0, 0.0, 0.0), "intermediate_stage": (0.0, c1, 0.0),
                "output_stage": (0.0, c1, c2)}

    def build_problem(self, intent, values):
        points = self._stage_points(intent, values)
        return build_parallel_shaft_problem(points, (
            ("input_stage", "intermediate_stage", values["stage_1_center_distance_mm"].value / 1000.0),
            ("intermediate_stage", "output_stage", values["stage_2_center_distance_mm"].value / 1000.0),
        ), layout=intent.layout)

    def project_contract(self, intent, values, solve_result, *, compiler_version, catalog_version, design_hash):
        roles = intent.role_map()
        points = {role: tuple(solve_result.points_m[f"shaft:{role}"]) for role in
                  ("input_stage", "intermediate_stage", "output_stage")}
        front = values["bearing_plane_front_mm"].value / 1000.0
        rear = values["bearing_plane_rear_mm"].value / 1000.0
        shaft_diameter = values["shaft_diameter_mm"].value
        hardpoints = []
        roots = [(roles["housing"], _I4)]
        for role in ("input_stage", "intermediate_stage", "output_stage"):
            sub_id = roles[role]
            x, y, z = points[role]
            root = _transform(0.0, y, z)
            roots.append((sub_id, root))
            for side, axial in (("front", front), ("rear", rear)):
                local = _transform(axial, 0.0, 0.0)
                world = _transform(axial, y, z)
                params = (("diameter_mm", shaft_diameter),)
                hardpoints.append(Hardpoint(f"{role}_{side}_bearing", sub_id, "mount", world,
                                            local, (1.0, 0.0, 0.0), "YZ", params,
                                            (f"parameter:bearing_plane_{side}_mm",)))
                hardpoints.append(Hardpoint(f"housing_{role}_{side}_bore", roles["housing"], "mount",
                                            world, world, (1.0, 0.0, 0.0), "YZ", params,
                                            (f"solved:shaft:{role}",)))
            mesh_planes = (("stage_1", values["stage_1_gear_plane_mm"].value / 1000.0),
                           ("stage_2", values["stage_2_gear_plane_mm"].value / 1000.0))
            # Which gear body sits on THIS shaft at THIS mesh stage, so the manager can
            # build the gear to its exact pitch diameter (input drives with pinion_1,
            # intermediate carries gear_1 + pinion_2, output carries gear_2).
            _pitch_key = {
                ("input_stage", "stage_1"): "pinion_1_pitch_diameter_mm",
                ("intermediate_stage", "stage_1"): "gear_1_pitch_diameter_mm",
                ("intermediate_stage", "stage_2"): "pinion_2_pitch_diameter_mm",
                ("output_stage", "stage_2"): "gear_2_pitch_diameter_mm",
            }
            for mesh_name, axial in mesh_planes:
                if ((role == "input_stage" and mesh_name != "stage_1") or
                    (role == "output_stage" and mesh_name != "stage_2")):
                    continue
                mesh_world = _transform(axial, y, z)
                mesh_local = _transform(axial, 0.0, 0.0)
                pitch_dia = values[_pitch_key[(role, mesh_name)]].value
                hardpoints.append(Hardpoint(f"{role}_{mesh_name}_mesh", sub_id, "mesh",
                                            mesh_world, mesh_local, (1.0, 0.0, 0.0), "YZ",
                                            (("face_width_mm", values["face_width_mm"].value),
                                             ("diameter_mm", pitch_dia)),
                                            (f"solved:shaft:{role}", f"parameter:{mesh_name}_gear_plane_mm")))
        hx, hy, hz = (values[key].value / 1000.0 for key in ("housing_x_mm", "housing_y_mm", "housing_z_mm"))
        envelopes = (FunctionalEnvelope(roles["housing"], (-hx / 2, -hz / 2, -hz / 2),
                                        (hx / 2, hy, hz / 2)),)
        for role in ("input_stage", "intermediate_stage", "output_stage"):
            envelopes += (FunctionalEnvelope(roles[role], (0.0, -hz / 2, -hz / 2),
                                             (values["shaft_length_mm"].value / 1000.0, hz / 2, hz / 2)),)
        return HardpointContract(tuple(sorted(roots)), tuple(sorted(hardpoints, key=lambda h: h.id)),
                                 envelopes, compiler_version, catalog_version, design_hash)
