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
    node["required"] = ["sim_backend", "environment"] + list(node["required"])
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


def design_environment(task, robot_info, subsystem=None, *,
                       base_url=None, api_key=None, model=None):
    """One call: choose the sim environment + emit the scenario spec for it.

    ``subsystem`` (optional) is one independent power unit of a multi-subsystem machine;
    when given, the spec targets that subsystem's input. Returns the spec dict (same shape
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
    msgs = [{"role": "system", "content": SYSTEM},
            {"role": "user", "content":
                f"TASK: {task}{sub}\n\n{_robot_block(robot_info)}\n"
                f"Decide the simulation environment and output the scenario spec. Use ONLY "
                f"the real joint/link names above."}]
    return _call(msgs, base_url, api_key, model)


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
