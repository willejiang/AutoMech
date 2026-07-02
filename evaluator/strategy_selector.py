#!/usr/bin/env python3
"""
Strategy selector — the evaluator's FIRST decision: given a task + a robot's
URDF info, decide HOW to evaluate it.

This is the "does the evaluator know to use RL" test. The LLM picks among
evaluation strategies based on whether the task needs a static-stability check,
a scripted motion, or a LEARNED controller (RL training).

Runs against the configured VLM gateway (AZURE_VLM_DEPLOYMENT). Outputs a structured
decision the orchestrator acts on.
"""
import json, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_backends import model_id
from pipeline_context import PIPELINE

SYSTEM = PIPELINE + """You are the planning stage of a robotics design evaluator. Given a TASK and a
robot's URDF structure (joints, links, actuated DOF), you decide the best way to
TEST whether the robot can accomplish the task in simulation.

Choose ONE evaluation strategy:

- "static_stability": the task is about holding a configuration / not falling
  (e.g. "stand upright", "is this pose stable"). Spawn the robot in a pose and
  check it holds. No motion, no learning. 

- "scripted_motion": the task is a short, prescribable maneuver achievable by a
  fixed timed joint trajectory (e.g. "squat and stand", "lift a leg"). No
  learning needed; a keyframe sequence suffices.

- "driven_mechanism": the object is a MACHINE whose point is INTERNAL MOTION
  TRANSMISSION — a gearbox, worm drive, gear train, crank-slider, clock movement,
  cryptex, winch. Success is NOT standing still; it is that turning/driving the
  INPUT joint makes the downstream parts move as the mechanism intends. Test it by
  mounting the mechanism fixed, driving its input joint, and watching whether the
  connected joints transmit the motion. Prefer this over static_stability whenever
  the task describes gears/shafts/linkages meant to move.

- "rl_training": the task requires a LEARNED control policy because the motion
  is dynamic, continuous, and not hand-scriptable (e.g. "walk", "run",
  "locomote", "move forward", "trot"). Locomotion of a legged robot is the
  canonical case — balance + gait cannot be reliably hand-coded, so you train a
  policy with reinforcement learning and judge the learned behavior.

Reason from the physics of the task. Key signal for rl_training: does success
require COORDINATED, CONTINUOUS control of multiple actuated joints against
ground contact over time (locomotion/manipulation) rather than a single static
pose or a short open-loop motion?

Also assess feasibility from the URDF: how many ACTUATED (non-fixed) joints does
the robot have? A robot with zero actuated joints CANNOT be controlled at all —
flag this as a structural blocker. If the TASK needs motion but the robot has no
joints, set action="return_to_worker" with a worker_message saying exactly what to
add (e.g. "task wants walking but URDF has 0 joints — add leg joints").

Define a TEST SET: list the distinct things to verify for the task, not just one.
A walkable quadruped: stand (hold pose), walk (forward locomotion), get_up (rise
after a fall). A bridge: load-bearing, deflection. Each test = {name, goal,
strategy}. The designer builds + runs each; the judge scores all.

MULTIPLE SUBSYSTEMS: you may be given a SUBSYSTEMS list — independent functional
units of the machine, each with its OWN power input (e.g. a car: engine, steering,
drivetrain). When present with >1 entry, return ONE driven_mechanism test PER
subsystem, named after that subsystem, so each independent input is exercised
separately. A single-mechanism object (one subsystem) returns ONE test.

Also pick the SIMULATOR BACKEND best suited to the task's physics:
- "pybullet": rigid-body dynamics, contact, locomotion/manipulation — runs on CPU
  with no server/GPU. Default for most robotics tasks and when no GPU is available.
- "isaac_sim": GPU-accelerated rigid/robotics with photoreal frames — pick when the
  task needs RL at scale or high-fidelity rendering AND a GPU box is available.
- "openfoam": fluid / aerodynamics / CFD — anything about airflow, drag, lift,
  flow, pressure, cooling, hydro. Rigid sims cannot do this; choose openfoam.

Output ONLY the JSON schema."""

SCHEMA = {
    "type": "json_schema",
    "json_schema": {
        "name": "strategy_decision",
        "strict": True,
        "schema": {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "strategy": {"type": "string",
                             "enum": ["static_stability", "scripted_motion",
                                      "driven_mechanism", "rl_training"]},
                "sim_backend": {"type": "string",
                                "enum": ["isaac_sim", "pybullet", "openfoam"]},
                "backend_reason": {"type": "string"},
                "reasoning": {"type": "string"},
                "actuated_dof_count": {"type": "integer"},
                "structurally_feasible": {"type": "boolean"},
                "structural_concern": {"type": "string"},
                "action": {"type": "string", "enum": ["proceed", "return_to_worker"]},
                "worker_message": {"type": "string"},
                "tests": {
                    "type": "array",
                    "items": {
                        "type": "object", "additionalProperties": False,
                        "properties": {
                            "name": {"type": "string"},
                            "goal": {"type": "string"},
                            "strategy": {"type": "string",
                                "enum": ["static_stability", "scripted_motion",
                                         "driven_mechanism", "rl_training"]}},
                        "required": ["name", "goal", "strategy"]}},
            },
            "required": ["strategy", "sim_backend", "backend_reason", "reasoning",
                         "actuated_dof_count", "structurally_feasible", "structural_concern",
                         "action", "worker_message", "tests"],
        },
    },
}


def _coerce_decision(raw: str, subsystems=None) -> dict:
    """Parse the model's reply into the decision shape. Some gateways (the 8313
    Copilot proxy) ignore strict response_format and return their own key names, so
    normalize common aliases and backfill a tests list rather than hard-failing.
    When `subsystems` (>1) is given, ensure one driven test per uncovered subsystem."""
    import re as _re
    txt = (raw or "").strip()
    if not txt:
        raise ValueError("empty strategy response")
    if not txt.lstrip().startswith("{"):                # strip prose/fences
        m = _re.search(r"\{.*\}", txt, _re.DOTALL)
        if m:
            txt = m.group(0)
    d = json.loads(txt)
    # key aliases seen from non-conforming gateways
    if "strategy" not in d and "primary_strategy" in d:
        d["strategy"] = d["primary_strategy"]
    if "sim_backend" not in d and "backend" in d:
        d["sim_backend"] = d["backend"]
    d.setdefault("strategy", "static_stability")
    d.setdefault("sim_backend", "pybullet")
    d.setdefault("actuated_dof_count", 0)
    d.setdefault("action", "proceed")
    # backfill a test set if the model didn't emit one
    tests = d.get("tests")
    if not isinstance(tests, list) or not tests:
        d["tests"] = [{"name": d["strategy"], "goal": d.get("reasoning", "")[:120],
                       "strategy": d["strategy"]}]
    else:
        for t in tests:
            t.setdefault("strategy", d["strategy"])
            t.setdefault("name", t.get("strategy", "test"))
            t.setdefault("goal", "")
    # Multi-subsystem: one driven test per subsystem the model left uncovered.
    subs = subsystems or []
    if len(subs) > 1:
        named = " ".join((t.get("name") or "").lower() for t in d["tests"])
        for s in subs:
            sid = (s.get("id") or "").lower()
            if sid and sid not in named:
                d["tests"].append({"name": s.get("id"),
                                   "goal": f"drive the {s.get('id')} subsystem",
                                   "strategy": "driven_mechanism"})
    return d


def decide(task, robot_info, *, base_url=None, api_key=None, model=None):
    """Pick an evaluation strategy for `task` on `robot_info`.

    By default calls the Azure gateway (standalone CLI use). maker2 passes
    base_url/api_key/model to route through its localhost:8313 gateway instead."""
    from openai import OpenAI
    c = OpenAI(base_url=(base_url or os.environ["AZURE_OPENAI_ENDPOINT"]).rstrip("/"),
               api_key=api_key or os.environ["AZURE_OPENAI_API_KEY"])
    subs = robot_info.get("subsystems") or []
    subs_txt = ""
    if len(subs) > 1:
        subs_txt = ("\nSUBSYSTEMS (independent inputs — return one driven test each):\n"
                    + "\n".join(f"  - {s.get('id')}: input={s.get('driver')}, "
                                f"output={s.get('output_joint')}" for s in subs) + "\n")
    msg = (f"TASK: {task}\n\n"
           f"ROBOT: {robot_info.get('name','robot')}\n"
           f"ACTUATED JOINTS ({len(robot_info.get('joints',[]))}): {robot_info.get('joints',[])}\n"
           f"LINKS ({len(robot_info.get('links',[]))}): {robot_info.get('links',[])}\n"
           f"{subs_txt}\n"
           f"Decide how to evaluate this task on this robot.")
    r = c.chat.completions.create(
        model=model or model_id(),
        messages=[{"role": "system", "content": SYSTEM},
                  {"role": "user", "content": msg}],
        response_format=SCHEMA, max_completion_tokens=1200)
    return _coerce_decision(r.choices[0].message.content, subsystems=subs)


if __name__ == "__main__":
    import argparse, xml.etree.ElementTree as ET
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--name", default="robot")
    a = ap.parse_args()
    root = ET.parse(a.urdf).getroot()
    joints = [j.get("name") for j in root.findall("joint")
              if j.get("type") in ("revolute", "prismatic", "continuous")]
    links = [l.get("name") for l in root.findall("link")]
    info = {"name": a.name, "joints": joints, "links": links}
    d = decide(a.task, info)
    print(json.dumps(d, indent=2))
