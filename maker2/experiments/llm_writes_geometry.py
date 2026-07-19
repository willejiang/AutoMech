"""Experiment 丙: can an LLM (gpt-5.6-sol) WRITE the geometry-computing code itself?

Not part of the pipeline. A standalone probe to answer one question with DATA:
when we ask the LLM to author Python that COMPUTES a mechanism's key geometry
(instead of guessing numbers in JSON), is the result (a) runnable, (b) internally
self-consistent, and (c) — for the reducer, where we have an authoritative
deterministic compiler — numerically correct?

Three tasks of rising difficulty:
  1. two-stage spur gear reducer  -> has an authoritative answer (maker2 gear_reducer.py)
  2. tourbillon gear train        -> no template; test self-consistency only
  3. Torsen-style differential    -> hardest; test runnable + self-consistency

For each: prompt the model to emit ONE python code block that defines
`compute() -> dict` returning named geometric quantities in mm, run it in a
sandbox, then run deterministic checks (gear mesh center distance == sum of pitch
radii, axial planes consistent, no NaN/negative radii, etc.).

Run:  python -m maker2.experiments.llm_writes_geometry
"""
from __future__ import annotations

import io
import json
import re
import textwrap
import traceback
import urllib.request

GATEWAY = "http://127.0.0.1:8313/v1/chat/completions"
MODEL = "gpt-5.6-sol"

_SYS = (
    "You are a mechanical design engineer who writes PRECISE, deterministic Python. "
    "When asked for a mechanism's geometry you DERIVE every number from first principles "
    "(gear module math, center distances, pitch radii, axial stack-up) inside code — you "
    "never hard-code a coordinate you could compute. Standard library + math only; no CAD "
    "libs, no external packages. Output EXACTLY ONE ```python code block and nothing else."
)

_TASKS = {
    "reducer": textwrap.dedent("""\
        Write Python defining `compute() -> dict` for a TWO-STAGE parallel-shaft SPUR GEAR
        REDUCER with overall ratio 9:1. Use module m=1.0 mm for every gear. Pick integer
        tooth counts so stage-1 and stage-2 ratios multiply to 9 (e.g. 3:1 then 3:1).
        Three parallel shafts (input, intermediate, output) on the X axis; gears mesh in
        the XY-plane offset (Y). Derive and return, all in mm:
          - per gear: teeth, pitch_radius_mm
          - stage_1_center_distance_mm, stage_2_center_distance_mm
          - the axial (along-shaft, call it z) plane of each gear: input_gear_z,
            inter_gear1_z, inter_pinion2_z, output_gear_z  (two gears share the
            intermediate shaft at DIFFERENT z; a meshing pair must share the SAME z)
          - front/rear bearing plane z for each shaft: *_front_z, *_rear_z
        Return a flat dict of floats/ints. compute() must be pure and deterministic.
        """),
    "tourbillon": textwrap.dedent("""\
        Write Python defining `compute() -> dict` for a TOURBILLON gear train: a fixed
        fourth-wheel pinion, a rotating carriage (cage) carrying the escape wheel and
        lever, the cage rotating once per 60 s. Model the going train as meshing gear
        pairs with module m=0.20 mm. Choose sensible tooth counts for: fourth wheel,
        third wheel, centre wheel, escape wheel, and the cage pinion rolling on the fixed
        fourth-wheel pinion. Derive and return, all in mm where geometric:
          - each wheel/pinion: teeth, pitch_radius_mm
          - each meshing pair's center_distance_mm and the two members it couples
          - cage_rotation_period_s (must be 60)
          - the effective train ratio from fourth wheel to escape wheel
        Return a flat dict. Pure, deterministic.
        """),
    "torsen": textwrap.dedent("""\
        Write Python defining `compute() -> dict` for a TORSEN-style limited-slip
        DIFFERENTIAL using crossed-axis helical (worm-like) gearing: two side worm gears
        (one per output shaft) each driven by a pair of element/worm wheels, all inside a
        rotating housing. Use module m=1.5 mm. Choose tooth counts giving a 1:1 average
        split between the two output shafts. Derive and return, all in mm where geometric:
          - side_gear teeth + pitch_radius_mm (both sides equal)
          - element_worm teeth + pitch_radius_mm
          - the center_distance_mm between a side gear and its meshing element
          - the crossed-axis angle_deg between side-gear axis and element axis (90 for
            classic Torsen)
          - torque_bias_ratio (a plausible value, and the code should COMPUTE it from the
            worm lead angle / friction assumption you state, not hard-code it)
        Return a flat dict. Pure, deterministic.
        """),
}


def _ask(task_prompt: str) -> str:
    body = json.dumps({
        "model": MODEL,
        "messages": [{"role": "system", "content": _SYS},
                     {"role": "user", "content": task_prompt}],
    }).encode()
    req = urllib.request.Request(GATEWAY, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=180) as r:
        data = json.load(r)
    return data["choices"][0]["message"]["content"]


def _extract_code(text: str) -> str:
    m = re.search(r"```(?:python)?\s*(.*?)```", text, re.S)
    return (m.group(1) if m else text).strip()


def _run(code: str) -> tuple[dict | None, str]:
    ns: dict = {}
    try:
        exec(compile(code, "<llm_geo>", "exec"), ns)
        if "compute" not in ns:
            return None, "no compute() defined"
        out = ns["compute"]()
        if not isinstance(out, dict):
            return None, f"compute() returned {type(out).__name__}, not dict"
        return out, ""
    except Exception:
        return None, traceback.format_exc(limit=3)


def _num(d: dict, *keys):
    for k in keys:
        if k in d:
            try:
                return float(d[k])
            except (TypeError, ValueError):
                return None
    return None


def _check_common(out: dict) -> list[str]:
    """Deterministic sanity checks that apply to any geometry dict."""
    issues = []
    for k, v in out.items():
        if isinstance(v, (int, float)):
            if v != v:  # NaN
                issues.append(f"{k} is NaN")
            if ("radius" in k or "center_distance" in k) and v <= 0:
                issues.append(f"{k}={v} should be positive")
    return issues


def _check_reducer(out: dict) -> list[str]:
    issues = _check_common(out)
    # mesh center distance == sum of pitch radii, for each stage we can identify
    for stage, (a, b) in {
        "stage_1": (("input_gear_pitch_radius_mm", "input_pitch_radius_mm", "input_gear_r_mm"),
                    ("inter_gear1_pitch_radius_mm", "inter_gear_pitch_radius_mm")),
        "stage_2": (("inter_pinion2_pitch_radius_mm", "inter_pinion_pitch_radius_mm"),
                    ("output_gear_pitch_radius_mm", "output_pitch_radius_mm")),
    }.items():
        ra, rb = _num(out, *a), _num(out, *b)
        cd = _num(out, f"{stage}_center_distance_mm")
        if ra and rb and cd:
            want = ra + rb
            if abs(cd - want) > 0.05:
                issues.append(f"{stage}: center_distance {cd:.3f} != r_a+r_b {want:.3f}")
    # axial coplanarity: a meshing pair must share z
    z_in, z_g1 = _num(out, "input_gear_z"), _num(out, "inter_gear1_z")
    if z_in is not None and z_g1 is not None and abs(z_in - z_g1) > 0.05:
        issues.append(f"stage-1 gears not coplanar: input_z {z_in} vs inter_gear1_z {z_g1}")
    z_p2, z_out = _num(out, "inter_pinion2_z"), _num(out, "output_gear_z")
    if z_p2 is not None and z_out is not None and abs(z_p2 - z_out) > 0.05:
        issues.append(f"stage-2 gears not coplanar: inter_pinion2_z {z_p2} vs output_z {z_out}")
    return issues


def _check_meshpairs(out: dict) -> list[str]:
    """Generic: any key holding a center_distance we can cross-check against radii is nice,
    but train dicts vary; we at least verify no degenerate geometry + ratio present."""
    return _check_common(out)


_CHECKERS = {"reducer": _check_reducer, "tourbillon": _check_meshpairs,
             "torsen": _check_meshpairs}


def _authoritative_reducer() -> dict | None:
    """The deterministic compiler's numbers for a comparable reducer, if available."""
    try:
        from maker2.design.templates.gear_reducer import ParallelShaftTwoStageReducerTemplate  # noqa
        # The compiler needs a full intent/catalog; too heavy to stand up here. We instead
        # report the closed-form truth for m=1, 3:1 x 3:1 so the reader can eyeball.
    except Exception:
        pass
    m = 1.0
    # canonical: pinion 12T, gear 36T each stage (3:1). center distance = m*(zp+zg)/2
    zp, zg = 12, 36
    return {
        "note": "closed-form truth, m=1, 12T->36T per stage (3:1 x 3:1 = 9:1)",
        "pinion_pitch_radius_mm": m * zp / 2,
        "gear_pitch_radius_mm": m * zg / 2,
        "stage_center_distance_mm": m * (zp + zg) / 2,
    }


def main() -> int:
    print(f"== Experiment 丙: {MODEL} writes geometry-computing Python ==\n")
    summary = []
    for name, prompt in _TASKS.items():
        print(f"### TASK: {name}")
        try:
            reply = _ask(prompt)
        except Exception as e:
            print(f"  LLM call failed: {e}\n")
            summary.append((name, "LLM_ERROR", []))
            continue
        code = _extract_code(reply)
        out, err = _run(code)
        if out is None:
            print(f"  CODE DID NOT RUN:\n{textwrap.indent(err, '    ')}\n")
            summary.append((name, "RUN_FAIL", []))
            # show first 30 lines of the offending code for the reader
            print("  --- code (first 30 lines) ---")
            for ln in code.splitlines()[:30]:
                print("   ", ln)
            print()
            continue
        issues = _CHECKERS[name](out)
        verdict = "SELF-CONSISTENT" if not issues else f"{len(issues)} ISSUE(S)"
        print(f"  ran OK, returned {len(out)} fields -> {verdict}")
        for i in issues:
            print(f"    - {i}")
        # print the returned numbers compactly
        nums = {k: v for k, v in out.items() if isinstance(v, (int, float))}
        print("  numbers:", json.dumps(nums, indent=None)[:600])
        if name == "reducer":
            print("  authoritative (closed-form):",
                  json.dumps(_authoritative_reducer(), indent=None))
        print()
        summary.append((name, verdict, issues))

    print("== SUMMARY ==")
    for name, verdict, issues in summary:
        print(f"  {name:12s} {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
