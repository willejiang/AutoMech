#!/usr/bin/env python3
"""
urdf_author.py — the "VLM authors the URDF" step (NO FreeCAD / CAD kernel).

Given the task, the generated .scad, its top-level module names, and the 6
renders, a VLM authors a manifest.json describing the articulated assembly:
which parts (each -> a render_module in the .scad), the joints between them
(prismatic/revolute with axis/limits/stiffness/damping), the test scenario, and
pass criteria.

Authoring the manifest IS authoring the URDF: the evaluator's
evaluator/run_eval.py turns this manifest into a URDF (render_parts ->
synth_urdf) by rendering each render_module separately and wiring joints. The
schema below mirrors evaluator/examples/quarter_car_manifest.json exactly so
evaluate.sh consumes the design dir unchanged.

Env: AZURE_OPENAI_ENDPOINT + AZURE_OPENAI_API_KEY + AZURE_VLM_DEPLOYMENT.
"""
import base64
import json
import os
from pathlib import Path

AUTHOR_SYSTEM = """You are a robotics assembly engineer. You are given a natural-language task, the OpenSCAD source of a CAD model, the list of its top-level module names (each is an independently renderable PART), and orthographic renders.

Author a MANIFEST describing how to turn this geometry into an ARTICULATED physics model for an Isaac Sim evaluator. Reason from rigid-body mechanics:

- PARTS: map each mechanically-distinct module to a part. Set render_module to the EXACT module name from the provided list (the evaluator renders each by calling that module). Pick a role (sprung_mass / unsprung_mass / wheel / chassis / link / base), a plausible material + density_kg_m3 (steel 7850, aluminum 2700, rubber 1100, plastic 1200). The ground/base part that should not fall sets fixed_base true ONLY if it is meant to be anchored; for a free vehicle leave all fixed_base false.
- JOINTS: connect parts with prismatic (linear, e.g. a suspension travel along an axis, lower_m/upper_m in meters) or revolute (rotational, e.g. a wheel hub, lower_deg/upper_deg) joints. Give each a parent and child part name, an axis [x,y,z], and stiffness/damping that suit the task (a suspension spring might be stiffness 35000 damping 1500; a free-spinning hub stiffness 0 damping 5). Only connect parts that physically attach. A model with one part has no joints.
- SCENARIO: design the physical test of the task. Choose a type, ground_friction, an optional obstacle (box with height_m/width_m/distance_m), an initial_velocity_mps [x,y,z], sim_duration_s, sim_dt (~0.008), gravity_mps2 -9.81.
- PASS_CRITERIA: measurable success for THIS task.

Set units to "mm" (OpenSCAD default) unless the model is clearly in meters, and up_axis "Z". Output ONLY the JSON manifest."""

# Mirrors evaluator/examples/quarter_car_manifest.json. Kept permissive
# (additionalProperties false but optional fields allowed) so the author can
# describe 1-part rigid drops or multi-part assemblies.
MANIFEST_SCHEMA = {
    "type": "json_schema",
    "json_schema": {
        "name": "design_manifest",
        "strict": True,
        "schema": {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "design_id": {"type": "string"},
                "user_prompt": {"type": "string"},
                "units": {"type": "string", "enum": ["mm", "cm", "m"]},
                "up_axis": {"type": "string", "enum": ["Z", "Y"]},
                "parts": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "additionalProperties": False,
                        "properties": {
                            "name": {"type": "string"},
                            "render_module": {"type": "string"},
                            "role": {"type": "string"},
                            "material": {"type": "string"},
                            "density_kg_m3": {"type": "number"},
                            "fixed_base": {"type": "boolean"},
                            "static_friction": {"type": "number"},
                            "dynamic_friction": {"type": "number"},
                        },
                        "required": ["name", "render_module", "role", "material",
                                     "density_kg_m3", "fixed_base",
                                     "static_friction", "dynamic_friction"],
                    },
                },
                "joints": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "additionalProperties": False,
                        "properties": {
                            "name": {"type": "string"},
                            "type": {"type": "string", "enum": ["prismatic", "revolute", "fixed"]},
                            "parent": {"type": "string"},
                            "child": {"type": "string"},
                            "axis": {"type": "array", "items": {"type": "number"}},
                            "lower": {"type": "number",
                                      "description": "meters for prismatic, degrees for revolute"},
                            "upper": {"type": "number"},
                            "stiffness": {"type": "number"},
                            "damping": {"type": "number"},
                        },
                        "required": ["name", "type", "parent", "child", "axis",
                                     "lower", "upper", "stiffness", "damping"],
                    },
                },
                "scenario": {
                    "type": "object",
                    "additionalProperties": False,
                    "properties": {
                        "type": {"type": "string"},
                        "ground_friction": {"type": "number"},
                        "obstacle_height_m": {"type": "number"},
                        "obstacle_width_m": {"type": "number"},
                        "obstacle_distance_m": {"type": "number"},
                        "initial_velocity_mps": {"type": "array", "items": {"type": "number"}},
                        "sim_duration_s": {"type": "number"},
                        "sim_dt": {"type": "number"},
                        "gravity_mps2": {"type": "number"},
                    },
                    "required": ["type", "ground_friction", "obstacle_height_m",
                                 "obstacle_width_m", "obstacle_distance_m",
                                 "initial_velocity_mps", "sim_duration_s",
                                 "sim_dt", "gravity_mps2"],
                },
                "pass_criteria": {"type": "string",
                                  "description": "measurable success condition in plain language"},
            },
            "required": ["design_id", "user_prompt", "units", "up_axis", "parts",
                         "joints", "scenario", "pass_criteria"],
        },
    },
}


def _b64(path):
    return base64.b64encode(Path(path).read_bytes()).decode()


def _normalize(manifest, scad_filename, source=None):
    """Reshape the flat LLM schema into the nested form run_eval.py expects
    (source block, joint lower_m/upper_m vs lower_deg/upper_deg, nested
    obstacle/contact). Keeps the evaluator contract authoritative.

    `source` overrides the default scad source — pass {"type":"stl","file":...}
    for the no-top-level-module fallback (the evaluator renders per-part by
    calling `use <src>; module();`, which has nothing to call when the .scad has
    no part modules; a fused STL is the rigid-body drop-test fallback)."""
    out = dict(manifest)
    out["source"] = source or {"type": "scad", "file": scad_filename}
    # joints: split lower/upper by joint type into the evaluator's field names
    for j in out.get("joints", []):
        if j["type"] == "prismatic":
            j["lower_m"], j["upper_m"] = j.pop("lower"), j.pop("upper")
        elif j["type"] == "revolute":
            j["lower_deg"], j["upper_deg"] = j.pop("lower"), j.pop("upper")
        else:
            j.pop("lower", None); j.pop("upper", None)
    # parts: fold friction into a contact block (evaluator reads part["contact"])
    for p in out.get("parts", []):
        sf, df = p.pop("static_friction", None), p.pop("dynamic_friction", None)
        if sf is not None or df is not None:
            p["contact"] = {"static_friction": sf, "dynamic_friction": df,
                            "restitution": 0.1}
    # scenario: nest the obstacle the way the example does
    sc = out.get("scenario", {})
    sc["obstacle"] = {"shape": "box",
                      "height_m": sc.pop("obstacle_height_m", 0.0),
                      "width_m": sc.pop("obstacle_width_m", 0.0),
                      "distance_m": sc.pop("obstacle_distance_m", 0.0)}
    return out


def author(task, scad_path, module_names, view_pngs, feedback=None, stl_path=None):
    """-> normalized manifest dict ready to write next to the .scad.

    If the .scad has NO top-level part modules, the evaluator can't render parts
    separately (it calls `use <src>; module();`), so we fall back to a single
    fused rigid part backed by the already-rendered STL (the evaluator's
    source.type=="stl" drop-test path)."""
    scad_path = Path(scad_path)
    scad_text = scad_path.read_text(encoding="utf-8", errors="replace")
    no_modules = not module_names
    fb = f"\n\nPREVIOUS SIM FEEDBACK to address:\n{feedback}" if feedback else ""
    mod_hint = (
        "This model has NO separable top-level modules — treat it as a SINGLE "
        "rigid part (one part, no joints) and design a rigid-body drop/impact test. "
        f"Use render_module \"{scad_path.stem}\" for that one part."
        if no_modules else
        f"TOP-LEVEL MODULE NAMES (use these EXACT names as render_module): {module_names}"
    )
    content = [{
        "type": "text",
        "text": (f"TASK: {task}\n\n{mod_hint}\n\n"
                 f"OPENSCAD SOURCE:\n{scad_text[:6000]}\n{fb}\n\n"
                 f"Author the manifest."),
    }]
    for fp in view_pngs:
        content.append({"type": "image_url",
                        "image_url": {"url": f"data:image/png;base64,{_b64(fp)}"}})

    from openai import OpenAI
    client = OpenAI(base_url=os.environ["AZURE_OPENAI_ENDPOINT"].rstrip("/"),
                    api_key=os.environ["AZURE_OPENAI_API_KEY"])
    resp = client.chat.completions.create(
        model=os.environ.get("AZURE_VLM_DEPLOYMENT", "gpt-5.4"),
        messages=[{"role": "system", "content": AUTHOR_SYSTEM},
                  {"role": "user", "content": content}],
        response_format=MANIFEST_SCHEMA,
        max_completion_tokens=3000,
    )
    raw = json.loads(resp.choices[0].message.content)
    # No modules -> point the evaluator at the fused STL instead of the .scad.
    source = None
    if no_modules and stl_path:
        source = {"type": "stl", "file": Path(stl_path).name}
    return _normalize(raw, scad_path.name, source)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--scad", required=True)
    ap.add_argument("--views", nargs="*", default=[])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    import render_views
    mods = render_views.parse_modules(Path(a.scad).read_text(encoding="utf-8", errors="replace"))
    m = author(a.task, a.scad, mods, a.views)
    s = json.dumps(m, indent=2)
    if a.out:
        Path(a.out).write_text(s, encoding="utf-8"); print(f"[author] -> {a.out}")
    else:
        print(s)
