"""One-shot task-05 independent-values representation ablation.

This script intentionally lives under ignored output/. It monkeypatches prompt objects only in
this Python process, runs one normal single-agent job, restores the objects, and verifies that
no production file changed.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import sys
import traceback

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
EXPERIMENT = Path(__file__).resolve().parent
RUNS = EXPERIMENT / "runs"

from benchmark_scorer.tasks.comfort_v1 import get_task
from maker2.benchmarks.telemetry import start_recorder, stop_recorder
from maker2.config import Settings
from maker2.jsonutil import strict_json_dumps
from maker2.single_agent import persist_single_agent_run, run_single_agent
from maker2.prompts import single_agent_prompt as prompt_module

TASK_ID = "05_three_planet_4to1"
TASK = get_task(TASK_ID)
PRODUCTION_FILES = (
    REPO / "maker2" / "prompts" / "single_agent_prompt.py",
    REPO / "maker2" / "prompts" / "manager_prompt.py",
    REPO / "maker2" / "single_agent.py",
    REPO / "maker2" / "config.py",
    REPO / "maker2" / "run.py",
)


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def file_hashes() -> dict[str, str]:
    return {path.relative_to(REPO).as_posix(): sha256_bytes(path.read_bytes())
            for path in PRODUCTION_FILES}


def replace_between(text: str, start: str, end: str, replacement: str) -> str:
    first = text.index(start)
    last = text.index(end, first)
    return text[:first] + replacement.rstrip() + "\n\n" + text[last:]


def build_overlay(system: str) -> tuple[str, list[str]]:
    changes: list[str] = []
    overlay = system.replace(
        "as ONE parametric build123d Python script",
        "as ONE build123d Python script")
    changes.append("removed parametric-script label")

    independent_block = """INDEPENDENT-VALUES REPRESENTATION ABLATION — THIS RUN ONLY.
Author every mechanically dependent dimension and placement as an independently selected local
numeric value. Do NOT create shared root parameters, dependency helper functions, or formulas that
propagate one dimension into another: no shared module variable, pitch_r/center_dist helpers,
formula-derived gear centers, formula-derived axial stations, hardpoint-derived local ports, or
shaft-derived bore variables. Repeating a value as separate literals is allowed and desired for
this experiment. You must still satisfy the SAME mechanism semantics, named relations, tooth-count
ratios, fits, collision rules, and evaluator measurements; the ablation changes only how numbers
are represented, not what the machine is required to do."""
    overlay = replace_between(overlay, "COMPUTE THE NUMBERS FIRST", "OUTPUT:",
                              independent_block)
    changes.append("replaced compute-first/shared-root drivetrain block")

    overlay = replace_between(
        overlay,
        "- For a crank-slider:",
        "- `watch_links` should list",
        """- For a crank-slider, preserve the same explicit crank, rod, pin, slide, and closure
  semantics, but choose every hardpoint, port coordinate, pocket size, clearance, and placement as
  an independent numeric literal. Do not derive one from another or convert a shared world
  hardpoint into local coordinates by formula. The resulting geometry must nevertheless leave
  real rod/pin/guide clearance and pass the same measured closure and motion checks.""")
    changes.append("removed hardpoint-first and formula-derived linkage geometry instructions")

    overlay = replace_between(
        overlay,
        "HOW TO CONSTRUCT A PART (build123d modeling method",
        "- STABLE SELECTORS:",
        """HOW TO CONSTRUCT A PART (independent-values experiment):
- Use robust build123d primitives, profiles, booleans, and named intermediate shapes.
- You may use Align.MIN or centered primitives, but independently type each final size and
  placement as a literal rather than deriving it from another part's face or dimension.
- Build bores, pockets, supports, ring gears, and hole patterns as real geometry. Choose each bore,
  radius, wall, tooth, hole, and station independently; do not create shared controlling variables
  or coordinate lists derived from other dimensions.
- Keep boolean tools oversized enough to cut reliably and apply fillets/chamfers last.""")
    changes.append("replaced embedded dependency-oriented modeling method")

    overlay = replace_between(
        overlay,
        "- Space coaxial parts",
        "- OMIT torque-lock hardware",
        """- Place coaxial parts at independently chosen literal axial stations. They must be
  distinct, physically supported, and free of unintended overlap, but no station may be computed
  from another part's face or height.
- A passive accessory must use its own independent literal bore and placement. It must assemble
  physically and have the declared press/running fit, but do not express the bore in terms of the
  shaft radius and do not reuse a shared fit parameter.
- For every bore, independently choose a literal value that produces the intended measured sign:
  slightly smaller for a press fit, slightly larger for a running fit. Do not define or reuse
  clearance variables and do not calculate the bore from the shaft.""")
    changes.append("removed face-stacking and shaft-derived bore formulas")

    overlay = replace_between(
        overlay,
        "- WHEN A SHELL/BODY COLLIDES",
        "Respond with a short NOTES plan",
        """- When a shell or body collides with a mechanism, change only independent literal
  dimensions or placements of the named parts until measured geometry passes. Do not introduce a
  shared envelope, wheel, shaft, or housing parameter and do not propagate one edit through a
  formula.""")
    changes.append("removed shell/envelope parameter reuse requirement")

    overlay += """

FINAL ABLATION ENFORCEMENT: in the emitted Python code, do not define reusable dimensional root
constants or arithmetic helpers for dependency propagation. Mechanically related dimensions and
placements must appear as independently authored literals, even if this duplicates values. The
MECHANISM semantic dict remains mandatory and truthful."""
    return overlay, changes


def scrub_physics_feedback(text: str) -> str:
    replacements = (
        ("This is a DESIGN-ARITHMETIC fault", "This is a local independent-value fault"),
        ("recheck the numbers the function depends on (tooth counts and the ratio they produce, lever lengths, travel distances)",
         "independently edit the local literal numbers associated with the failed measurement"),
        ("move ONE of the two", "independently edit ONE of the two"),
        ("past that gear's tip radius (module*(teeth+2)/2)", "to a literal position beyond the gear"),
        ("compute each one's z from the real top face of the part below it", "independently choose each axial z literal"),
        ("Re-solve the hardpoints first", "Independently edit the crank-pin, wrist-pin, rod, pocket, and guide literals"),
        ("re-derive the rod pocket and guide passage from that geometry", "independently edit the rod pocket and guide passage"),
        ("solve z from the seat center: flywheel_z = seat_center_z - flywheel_h/2", "choose an independent flywheel z literal"),
        ("set its bore to `shaft_outer_r - 0.005`", "independently choose a slightly smaller literal bore"),
        ("takes `shaft_outer_r + 0.05`", "takes an independently chosen slightly larger literal bore"),
        ("Take the bore radius FROM the shaft it rides", "Independently edit the bore radius"),
        ("`bore_r = <shaft>_outer_r - 0.005`", "a slightly smaller literal bore"),
        ("`<shaft>_outer_r + 0.05`", "a slightly larger literal bore"),
        ("Never write a bore as a bare number, and never size it before the shaft exists.",
         "Use a bare local bore number; do not derive it from the shaft."),
        ("Recompute each meshing pair's center distance = module*(z1+z2)/2 and place the gear centers EXACTLY that far apart, using the SAME module for both.",
         "Independently edit each gear size and center-position literal until the measured mesh and transmission checks pass."),
    )
    for old, new in replacements:
        text = text.replace(old, new)
    return text + """

INDEPENDENT-VALUES ABLATION REMINDER: make only independent local literal edits. Do not add shared
root parameters or derive one dimension/placement from another. Preserve the same mechanical
semantics and pass the measured diagnosis."""


def main() -> int:
    EXPERIMENT.mkdir(parents=True, exist_ok=True)
    RUNS.mkdir(parents=True, exist_ok=True)
    before_files = file_hashes()
    before_prompt = prompt_module.SINGLE_AGENT_SYSTEM
    before_prompt_hash = sha256_bytes(before_prompt.encode("utf-8"))
    overlay, changes = build_overlay(before_prompt)
    overlay_hash = sha256_bytes(overlay.encode("utf-8"))
    original_feedback = prompt_module.build_single_agent_physics_feedback

    def independent_feedback(*args, **kwargs):
        return scrub_physics_feedback(original_feedback(*args, **kwargs))

    manifest = {
        "schema": "physcad-representation-ablation/1.0",
        "task_id": TASK_ID,
        "task_prompt": TASK.prompt,
        "task_prompt_sha256": TASK.prompt_sha256,
        "arm": "independent_values",
        "model": "gpt-5.6-sol",
        "engine": "mujoco",
        "mjcf_compiler_mode": "agent",
        "max_iters": 3,
        "reference_tools": False,
        "kb": False,
        "production_prompt_sha256_before": before_prompt_hash,
        "overlay_prompt_sha256": overlay_hash,
        "overlay_changes": changes,
        "production_file_hashes_before": before_files,
        "normal_production_files_modified": False,
        "pilot_only": True,
    }
    (EXPERIMENT / "independent_values_system_prompt.txt").write_text(overlay,
                                                                       encoding="utf-8")
    (EXPERIMENT / "experiment_manifest.before.json").write_text(
        strict_json_dumps(manifest, indent=2), encoding="utf-8")

    settings = Settings.load()
    settings.model = "gpt-5.6-sol"
    settings.engine = "mujoco"
    settings.mjcf_compiler_mode = "agent"
    settings.enable_reference_tools = False
    settings.enable_kb = False
    recorder = start_recorder(task=TASK.prompt, out_dir=str(RUNS),
                              pipeline="single_agent_independent_values",
                              cold_requested=True)
    recorder.configure(provider=settings.provider_name, model=settings.model,
                       engine=settings.engine, representation="independent_values")
    settings.mjcf_compiler_cache_dir = recorder.cold_cache_dir
    result = {"ok": False, "run_dir": None, "error": "experiment did not start"}
    caught = None
    try:
        prompt_module.SINGLE_AGENT_SYSTEM = overlay
        prompt_module.build_single_agent_physics_feedback = independent_feedback
        result = run_single_agent(TASK.prompt, str(RUNS), settings,
                                  do_physics=True, max_iters=3, log_fn=print)
        result = persist_single_agent_run(
            result, prompt=TASK.prompt, model=settings.model, max_iters=3,
            refine_message=None, thread=None)
        recorder.finalize(result)
    except Exception as exc:
        caught = f"{type(exc).__name__}: {exc}"
        result = {"ok": False, "run_dir": result.get("run_dir"),
                  "error": caught, "trace": traceback.format_exc()[-4000:]}
        try:
            recorder.finalize(result)
        except Exception:
            pass
    finally:
        prompt_module.SINGLE_AGENT_SYSTEM = before_prompt
        prompt_module.build_single_agent_physics_feedback = original_feedback
        stop_recorder(recorder)

    after_files = file_hashes()
    after_prompt_hash = sha256_bytes(prompt_module.SINGLE_AGENT_SYSTEM.encode("utf-8"))
    manifest.update({
        "production_prompt_sha256_after": after_prompt_hash,
        "production_file_hashes_after": after_files,
        "production_hashes_unchanged": before_files == after_files,
        "production_prompt_unchanged": before_prompt_hash == after_prompt_hash,
        "result_ok": bool(result.get("ok")),
        "result_run_dir": result.get("run_dir"),
        "exception": caught,
    })
    (EXPERIMENT / "experiment_manifest.after.json").write_text(
        strict_json_dumps(manifest, indent=2), encoding="utf-8")
    (EXPERIMENT / "result.json").write_text(strict_json_dumps(result, indent=2),
                                              encoding="utf-8")
    print("EXPERIMENT_RESULT_JSON:" + strict_json_dumps(result, separators=(",", ":")))
    print("PRODUCTION_HASHES_UNCHANGED:", manifest["production_hashes_unchanged"])
    print("PRODUCTION_PROMPT_UNCHANGED:", manifest["production_prompt_unchanged"])
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
