#!/usr/bin/env python3
"""
Strategy selector — the evaluator's FIRST decision: given a task + a robot's
URDF info, decide HOW to evaluate it.

This is the "does the evaluator know to use RL" test. The LLM picks among
evaluation strategies based on whether the task needs a static-stability check,
a scripted motion, or a LEARNED controller (RL training).

Runs locally against the Copilot proxy (claude-opus-4.8). Outputs a structured
decision the orchestrator acts on.
"""
import json, os
from pathlib import Path

SYSTEM = """You are the planning stage of a robotics design evaluator. Given a TASK and a
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
flag this as a structural blocker regardless of task.

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
                "reasoning": {"type": "string"},
                "actuated_dof_count": {"type": "integer"},
                "structurally_feasible": {"type": "boolean"},
                "structural_concern": {"type": "string"},
            },
            "required": ["strategy", "reasoning", "actuated_dof_count",
                         "structurally_feasible", "structural_concern"],
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
        model=os.environ.get("AZURE_VLM_DEPLOYMENT", "claude-opus-4.8"),
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
