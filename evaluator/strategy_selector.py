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
                             "enum": ["static_stability", "scripted_motion", "rl_training"]},
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
                                "enum": ["static_stability", "scripted_motion", "rl_training"]}},
                        "required": ["name", "goal", "strategy"]}},
            },
            "required": ["strategy", "sim_backend", "backend_reason", "reasoning",
                         "actuated_dof_count", "structurally_feasible", "structural_concern",
                         "action", "worker_message", "tests"],
        },
    },
}


def decide(task, robot_info):
    from openai import OpenAI
    c = OpenAI(base_url=os.environ["AZURE_OPENAI_ENDPOINT"].rstrip("/"),
               api_key=os.environ["AZURE_OPENAI_API_KEY"])
    msg = (f"TASK: {task}\n\n"
           f"ROBOT: {robot_info.get('name','robot')}\n"
           f"ACTUATED JOINTS ({len(robot_info.get('joints',[]))}): {robot_info.get('joints',[])}\n"
           f"LINKS ({len(robot_info.get('links',[]))}): {robot_info.get('links',[])}\n\n"
           f"Decide how to evaluate this task on this robot.")
    r = c.chat.completions.create(
        model=model_id(),
        messages=[{"role": "system", "content": SYSTEM},
                  {"role": "user", "content": msg}],
        response_format=SCHEMA, max_completion_tokens=1200)
    return json.loads(r.choices[0].message.content)


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
