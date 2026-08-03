#!/usr/bin/env python3
"""
Environment designer — the evaluator's SINGLE test-planning call.

Replaces the old two-call split (strategy_selector's task-TYPE enum + scenario_designer).
One LLM call looks at the MODEL (joints/links/roles/subsystems) + the user's prompt and
decides, in one shot:
  1. WHAT SIMULATION ENVIRONMENT makes this object's PURPOSE observable
     (bench-mounted + driven for a mechanism; a ground plane for a stand/locomote task;
      a fluid domain for airflow), and which sim backend runs it, and
  2. the SCENARIO SETUP for that environment (spawn pose, drive, watch, pass criteria).

There is NO task-TYPE enum: the brittle "static_stability | driven_mechanism | ..." label
was a single point of failure (one wrong word made a tourbillon run a stand-still topple
test). Instead the model tells us what's drivable and the LLM frames the environment around
that. The DETERMINISTIC driver enforcement (drive the true driver_input, watch the
transmission, declare the output + propagation path) is applied by physics._design_spec
AFTER this call, so correctness never depends on the LLM getting joint names right.

Reuses scenario_designer.SPEC_SCHEMA (the scenario fields) and re-exports revise().
Runs against the configured gateway (base_url/api_key/model injected by physics).
"""
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline_context import PIPELINE
from sim_backends import model_id
# Reuse the proven scenario schema + robot block + revise (don't duplicate them).
from scenario_designer import SPEC_SCHEMA, _robot_block, revise  # noqa: F401


SYSTEM = PIPELINE + """You are a robotics simulation ENVIRONMENT DESIGNER. Given a natural-language
task and a robot's ACTUAL joints/links (from its URDF), you decide — in ONE step — the
simulation ENVIRONMENT that makes the object's PURPOSE observable, and you output the
full SCENARIO SPEC that sets it up. There is NO "task type" to pick; reason directly about
what the object IS and what would prove it works.

DECIDE THE ENVIRONMENT FROM THE OBJECT (put this in "environment", one or two sentences):
- If the object is a MECHANISM whose purpose is transmitting motion (a clock/watch
  movement, tourbillon, gearbox, gear train, cryptex, winch, linkage) and it has a
  drivable input joint: the environment is BENCH-MOUNTED and DRIVEN. Set "fixed_base": true,
  fill "drive" (drive the ONE input; you will be given a ROLE MAP naming it), and success =
  motion reaching the output. Do NOT test such a thing by letting it sit still — that proves
  nothing.
- If the object's purpose is to HOLD ITSELF UP or not topple (a stool, bracket, statue, a
  standing robot pose): the environment is a GROUND PLANE, "drive": null, success = it stays
  up within tolerances.
- If the object's purpose is LOCOMOTION or a scripted maneuver: ground plane, actuate the
  relevant joints over time.
- If the task is about AIRFLOW / drag / lift / cooling / fluid: choose the fluid backend.

PICK THE SIM BACKEND ("sim_backend") from the physics:
- "pybullet": rigid-body dynamics + contact (mechanisms, locomotion, stability). CPU, no
  GPU needed. DEFAULT.
- "isaac_sim": GPU rigid/robotics + photoreal — only when RL-at-scale/high-fidelity is
  needed AND a GPU box is available.
- "openfoam": fluid / aerodynamics / CFD — airflow, drag, lift, pressure, cooling.

Then reason from first principles of rigid-body mechanics for the SCENARIO itself:
- SUPPORT & STABILITY: a body is stable only if its CoM projects inside its support polygon.
- INITIAL CONTACT: spawn so intended contact links just touch (not floating, not
  interpenetrating).
- ORIENTATION: euler XYZ radians composing with the rest frame.
- JOINT LIMITS & POSE: only command reachable angles.
- FRICTION: load-bearing contact links that must not slip need high lateral friction.
- CONTROL: hold a pose (position control) or drive joints; give stiffness/damping.

MACHINERY DRIVE (when the environment is bench-mounted + driven): you are given a ROLE MAP
(deterministic, from the URDF). Drive EXACTLY the ONE "driver_input" as "input_joint"
(velocity mode, ~3-6 rad/s for a rotating input, or position_sweep for a lever). Set
"self_collision": true so gears mesh by tooth contact. Set "watch_joints" = the role map's
"transmission" joints (observed, never driven). Declare "propagation_path" (input->...->
output) and "output_joint"; the test PASSES only when driving the input makes motion REACH
the output. Never drive a transmission or unrelated joint, never drive more than one input.

You may be given a SUBSYSTEM (one independent power unit of a larger machine) — design the
environment/spec for THAT subsystem's input only.

FIRST, in "success_definition", state what success physically means here (STAY STILL/HOLD,
MOVE/ACTUATE, or TRANSMIT MOTION) and the ONE observable that decides pass/fail. The rest of
the spec must serve it.

FRAME THE CAMERA ("camera"): the recording is judged by a vision model, so it must SEE the
moving parts. The camera is LOCKED for the whole run and does NOT follow a part that flies
off — so aim it at the mechanism's WORKING region and choose an angle where the gear train /
moving joints are unobstructed. For a flat bench-mounted movement, a 3/4 top-down view
(elevation ~ -20 to -35) reads best; for a tall mechanism, a lower side view. Set
distance_scale ~2-3 to frame the whole machine with margin. Leave a value 0 to auto-fit it.

WRITE THE DRIVE ("control_code"), when the two `drive` modes cannot express this machine.
`drive` offers only "spin the input at a constant rate" and "sweep it across a range". A
mechanism that STORES energy and RELEASES it fits neither: a trebuchet, a catapult, a
mousetrap, a spring-driven escapement is set to a wound pose and then LET GO — nothing
drives it, gravity or the spring does the work. Driving such a thing at a constant rate
does not test it; it turns the throwing arm in circles and measures nothing.

So you may instead write the drive as code, defining either or both of:

    def setup(m, d):
        # runs ONCE after the assembly settles, before any measurement baseline is taken.
        # Set the opening pose: d.qpos[m.joint("arm_spin").qposadr[0]] = -1.0 to wind it.

    def control(m, d, t):
        # runs EVERY step; t is elapsed driven seconds. Drive (d.qfrc_applied[...] = ...,
        # or command d.qpos[...]), release at a moment (if t < 1.0: drive, else: nothing),
        # phase several joints, or do nothing at all for a pure-gravity release.

Reach a joint by NAME, never by a guessed index: `j = m.joint("<part>_spin")` then
`d.qpos[j.qposadr[0]]` / `d.qfrc_applied[j.dofadr[0]]`. The joint names are the TRAJECTORY
KEYS listed below. stdlib + math + numpy as np only; no file or network access. Keep it
short and defensive — a joint you name may be missing, so guard the lookup rather than
raise. Return "" for control_code when the ordinary `drive` block already says it (a
hand-cranked gearbox, a clock wound by its input arbor): the built-in drive is the
comparable, well-tested path and code is only worth it when the form cannot say the thing.

WRITE THE FUNCTIONAL CHECK ("metrics_code"): say IN CODE what this machine has to achieve.
Everything else in this spec only establishes that the thing stayed put and something moved
— which is equally true of a gear train whose tooth counts are wrong, a gripper that never
closes, a ratchet that slips backwards. Those are the failures that matter, they differ for
every mechanism, and no fixed list of numeric fields can express them. A short program can.

Define exactly one function:

    def check(traj, result):
        # traj["t"]       -> [seconds]
        # traj["joints"]  -> {joint_name: [angle_rad, ...]}   same length as t
        # traj["bodies"]  -> {body_name: [[x, y, z] mm, ...]} same length as t
        # result          -> the measured metrics dict (input_travel, moved_count, ...)
        return [ {"name": ..., "value": ..., "expected": ...,
                  "passed": True/False, "detail": "..."} ]

Derive the criterion from the USER'S OWN DESCRIPTION, not from what the model happens to
contain. Read the domain word for the number hiding in it: a device showing hours and
minutes on one axis means those two outputs are geared 12:1; "a 20:1 reducer" states the
number outright; "a hand-cranked fan" pins down nothing but that the output must turn.
Then express it over the trajectory — a ratio of total travels, a displacement that must
reach some value, a sign that must never reverse, a gap that must close.

Rules: stdlib + math only, no imports of anything else, no file or network access. Keep it
short and defensive (a joint you name may be missing — return a failed check saying so
rather than raising). If the description pins down nothing checkable, return [] — an
invented criterion fails a machine that was built exactly as asked, and the next iteration
then "fixes" something that was never broken.

Output ONLY the JSON spec matching the provided schema."""


def _env_schema():
    """SPEC_SCHEMA extended with the two environment fields (sim_backend, environment).

    Built from a deep-ish copy of scenario_designer.SPEC_SCHEMA so we don't mutate the
    shared schema; strict mode requires every property to appear in `required`.
    """
    import copy
    schema = copy.deepcopy(SPEC_SCHEMA)
    node = schema["json_schema"]["schema"]
    node["properties"]["sim_backend"] = {
        "type": "string", "enum": ["pybullet", "isaac_sim", "openfoam"],
        "description": "the sim backend the task's physics needs"}
    node["properties"]["environment"] = {
        "type": "string",
        "description": "one or two sentences: the sim environment you chose and WHY "
                       "(bench-mounted+driven / ground plane / fluid domain)"}
    node["properties"]["camera"] = {
        "type": "object", "additionalProperties": False,
        "description": "how to FRAME the recording so the mechanism's motion is clearly "
                       "visible. The camera is LOCKED for the whole run (it does NOT "
                       "auto-track), so aim it at the mechanism's working center. Fill "
                       "azimuth/elevation/distance_scale; leave any value 0 to let the "
                       "runner auto-fit that aspect from the assembly's bounding box.",
        "properties": {
            "azimuth": {"type": "number",
                "description": "horizontal camera angle in degrees (0-360); 90 looks "
                               "along +Y. Pick the side that best shows the gear train."},
            "elevation": {"type": "number",
                "description": "vertical angle in degrees; negative looks DOWN at the "
                               "bench (e.g. -25 for a top-ish 3/4 view of a flat movement)."},
            "distance_scale": {"type": "number",
                "description": "multiplier on the assembly's bounding-box diagonal for "
                               "camera distance; ~2-3 frames the whole machine. 0 = auto."},
        },
        "required": ["azimuth", "elevation", "distance_scale"]}
    node["required"] = ["sim_backend", "environment", "camera"] + list(node["required"])
    schema["json_schema"]["name"] = "environment_spec"
    return schema


ENV_SCHEMA = _env_schema()


def _client(base_url=None, api_key=None):
    from openai import OpenAI
    return OpenAI(base_url=(base_url or os.environ["AZURE_OPENAI_ENDPOINT"]).rstrip("/"),
                  api_key=api_key or os.environ["AZURE_OPENAI_API_KEY"])


def _call(messages, base_url=None, api_key=None, model=None):
    c = _client(base_url, api_key)
    r = c.chat.completions.create(
        model=model or model_id(),
        messages=messages, response_format=ENV_SCHEMA,
        max_completion_tokens=4000)
    txt = (r.choices[0].message.content or "").strip()
    if not txt.lstrip().startswith("{"):        # gateway ignored the schema -> strip prose
        import re
        m = re.search(r"\{.*\}", txt, re.DOTALL)
        if m:
            txt = m.group(0)
    return json.loads(txt)


def _web_research(task, robot_info, base_url=None, api_key=None, model=None):
    """Web-search research turn returning collected result text (to inject as context
    into the schema-locked design call). Best-effort; returns "" on any failure.

    Uses maker2's LLMClient + run_tool_loop rather than the raw OpenAI SDK: the gateway
    signals a tool call via finish_reason but does NOT populate the SDK's typed
    .tool_calls, so the raw SDK never sees the call. LLMClient has a custom tool-call
    parser that reads the raw JSON, so tool-calling actually works there (verified)."""
    try:
        from maker2.config import Settings
        from maker2.llm.conversation import Conversation
        from maker2.tools import (WEB_SEARCH_TOOL, EXECUTORS, run_tool_loop,
                                  _RESEARCH_SYSTEM)
    except Exception:
        return ""
    try:
        s = Settings.load()
        client = s.make_client(2000)          # small budget; research is text lookups
        conv = Conversation()
        conv.add_user_message(
            f"Research realistic physics-test parameters (typical drive speeds/RPM, "
            f"friction, gear ratios, expected torque, how it is bench-tested) for: "
            f"{task} (robot: {robot_info.get('name', 'robot')}). A few focused searches, "
            f"then stop.")
        run_tool_loop(client, conv, _RESEARCH_SYSTEM, [WEB_SEARCH_TOOL], EXECUTORS,
                      max_rounds=3)
        notes = [Conversation.extract_text(m.get("content", ""))
                 for m in conv.messages if m.get("role") == "tool_result"]
        return "\n".join(n for n in notes if n).strip()
    except Exception:
        return ""



def _kb_notes(task: str, k: int = 3) -> str:
    """Top matches from the evaluator corpus for this task. Best-effort: the KB is a local
    index that may be absent or unbuilt, and a designer with no notes is still a designer."""
    try:
        from maker2 import kb
        hits = kb.search(f"functional criterion for: {task}", collection="evaluator", k=k)
    except Exception:
        return ""
    out = []
    for h in hits or []:
        text = (h.get("text") if isinstance(h, dict) else None) or ""
        if text.strip():
            out.append(text.strip()[:1200])
    return "\n\n".join(out)


def design_environment(task, robot_info, subsystem=None, *,
                       base_url=None, api_key=None, model=None, web=False):
    """One call: choose the sim environment + emit the scenario spec for it.

    ``subsystem`` (optional) is one independent power unit of a multi-subsystem machine;
    when given, the spec targets that subsystem's input. ``web`` (from
    enable_reference_tools) runs a web-search research turn first to look up realistic
    test parameters, injected as context. Returns the spec dict (same shape
    scenario_designer.design returned, plus sim_backend + environment). The caller
    (physics._design_spec) still applies the deterministic driver enforcement on top.
    """
    sub = ""
    if subsystem:
        sid = subsystem.get("id") if isinstance(subsystem, dict) else str(subsystem)
        drv = subsystem.get("driver") if isinstance(subsystem, dict) else None
        sub = (f"\n\nTHIS SUBSYSTEM: {sid}"
               + (f" (its power input is joint '{drv}')" if drv else "")
               + " — design the environment/spec for THIS subsystem's input only.")
    research = ""
    if web:
        notes = _web_research(task, robot_info, base_url, api_key, model)
        if notes:
            research = ("\n\nWEB RESEARCH (realistic test parameters — use where "
                        f"relevant):\n{notes[:3000]}")
    # Local KB: how to turn a product description into a criterion that can be checked.
    # Without this the designer had no guidance on `metrics_code` beyond the schema text,
    # and the corpus written for exactly this question sat unread.
    kb_notes = _kb_notes(task)
    if kb_notes:
        research += f"\n\nMETHOD NOTES (from the local knowledge base):\n{kb_notes}"
    msgs = [{"role": "system", "content": SYSTEM},
            {"role": "user", "content":
                f"TASK: {task}{sub}\n\n{_robot_block(robot_info)}{research}\n"
                f"Decide the simulation environment and output the scenario spec. Use ONLY "
                f"the real joint/link names above."}]
    return _call(msgs, base_url, api_key, model)


def revise_environment(task, robot_info, prev_spec, feedback, *,
                       base_url=None, api_key=None, model=None):
    """Re-design the environment/scenario spec (INCLUDING the camera) after a physics run
    whose diagnosis was a TEST-side fault (the camera could not see the mechanism, or the
    scenario drove/watched the wrong thing). Uses this module's SYSTEM + ENV_SCHEMA so the
    revised spec keeps the camera field. Returns the revised spec dict."""
    msgs = [{"role": "system", "content": SYSTEM},
            {"role": "user", "content":
                f"TASK: {task}\n\n{_robot_block(robot_info)}\n"
                f"PREVIOUS SPEC:\n{json.dumps(prev_spec, indent=2)}\n\n"
                f"THE TEST DID NOT PRODUCE A JUDGEABLE RESULT. DIAGNOSIS:\n{feedback}\n\n"
                f"If the camera could not see the mechanism, RE-FRAME it (fix camera "
                f"azimuth/elevation/distance_scale so the moving parts fill the frame). "
                f"If the wrong joint was driven or watched, fix the drive/watch_joints. "
                f"KEEP the previous spec's \"control_code\" and \"metrics_code\" verbatim "
                f"unless the diagnosis says THEY are what was wrong. A retry is meant to "
                f"fix how the machine is OBSERVED, and dropping control_code silently "
                f"reverts the drive to a constant-velocity motor — which for a mechanism "
                f"that is wound and released tests nothing. "
                f"Output a REVISED spec using ONLY the real joint/link names above."}]
    spec = _call(msgs, base_url, api_key, model)
    # Carry the previous drive/criterion across if the model dropped it anyway. Losing
    # control_code does not read as an error downstream — the runner just falls back to
    # `drive`, so the gate that was supposed to be swung open once and released came back
    # as a motor turning at 5 rad/s, and the retry silently tested a different machine.
    for _k in ("control_code", "metrics_code"):
        if not (spec.get(_k) or "").strip() and (prev_spec.get(_k) or "").strip():
            spec[_k] = prev_spec[_k]
    return spec


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--robot-info", required=True, help="JSON file with name/joints/links")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    ri = json.loads(Path(a.robot_info).read_text())
    spec = design_environment(a.task, ri)
    Path(a.out).write_text(json.dumps(spec, indent=2))
    print(f"[env-designer] wrote spec -> {a.out}")
    print("environment:", spec.get("environment", "")[:200])
    print("backend:", spec.get("sim_backend"))
    print("success:", spec.get("success_definition", "")[:200])
