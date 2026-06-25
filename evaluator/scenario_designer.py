#!/usr/bin/env python3
"""
Scenario designer — the LLM that turns a natural-language task + a robot's
joint/link names into a STRUCTURED SCENARIO SPEC the Isaac runner executes,
and revises that spec given simulation feedback.

Hybrid system prompt: it coaches GENERAL physical reasoning (support polygons,
center of mass, contact, joint limits, control strategy) WITHOUT hardcoding any
task-specific answer. Task specifics (this is a handstand, use elbow=0.30, etc.)
are left to the model + the iteration loop.

Runs on the HOST (uses the Azure key from .env). Two entry points:
  design(task, robot_info)                    -> first spec
  revise(task, robot_info, prev_spec, feedback)-> improved spec
"""
import json, os
from pathlib import Path


SYSTEM = """You are a robotics simulation SCENARIO DESIGNER. Given a natural-language
task and a robot's actual joint and link names (from its URDF), you output a
STRUCTURED SCENARIO SPEC (JSON) that sets up a physics test of whether the robot
can accomplish the task in a simulator.

You reason from first principles of rigid-body mechanics. Apply these GENERAL
principles to whatever task you are given — do not rely on memorized task recipes:

1. SUPPORT & STABILITY. A body is statically stable only if its center of mass
   projects (along gravity) INSIDE its support polygon — the convex hull of the
   ground contact points. A support polygon with AREA (e.g. flat feet/palms,
   wide stance) is far more stable than points or a line. If a task needs the
   robot to hold a pose, first ensure the contacting links form an area support
   and the CoM sits over it.

2. INITIAL CONTACT. The robot must START in or near its intended contact state.
   If it should rest on certain links, spawn it at a height where those links
   just touch the ground — not floating above (it will fall before control acts)
   and not interpenetrating (it will explode).

3. ORIENTATION. Think carefully about base orientation. Euler angles compose with
   the robot's rest frame; a 180-deg flip about one axis inverts the robot only
   if that axis matches its long/up axis. State orientation as euler XYZ radians.

4. JOINT LIMITS & POSE. Only command joint angles that are physically reachable.
   Choose a pose that achieves the task geometry (e.g. folds limbs so contact
   links reach the ground and the CoM lands over the support).

5. FRICTION. Contact links that must not slip (hands/feet bearing load) need high
   lateral friction. List those links.

6. CONTROL. Decide whether to HOLD a fixed pose with position control (good for
   static balance — note: aggressive feedback can DESTABILIZE delicate contact)
   or to drive joints. Give stiffness/damping.

7. SUCCESS CRITERIA. Define measurable pass/fail: e.g. base height stays above a
   floor, horizontal drift stays small, the pose is held for a duration.

You will sometimes be given FEEDBACK from a prior simulation attempt (what the
camera saw + measured metrics). Use it to diagnose the physical failure and
REVISE the spec — change orientation, height, pose, friction, or control to fix
the specific failure observed. Explain your reasoning briefly in "reasoning".

Output ONLY the JSON spec matching the provided schema."""


SPEC_SCHEMA = {
    "type": "json_schema",
    "json_schema": {
        "name": "scenario_spec",
        "strict": True,
        "schema": {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "reasoning": {"type": "string"},
                "base_orientation_euler": {
                    "type": "array", "items": {"type": "number"},
                    "description": "XYZ euler radians for the base"},
                "base_height": {"type": "number", "description": "spawn z in meters"},
                "joint_pose": {
                    "type": "array",
                    "items": {
                        "type": "object", "additionalProperties": False,
                        "properties": {
                            "joint": {"type": "string"},
                            "angle": {"type": "number"}},
                        "required": ["joint", "angle"]},
                    "description": "list of {joint, angle(rad)}. Only real joint names."},
                "high_friction_links": {
                    "type": "array", "items": {"type": "string"}},
                "control": {
                    "type": "object", "additionalProperties": False,
                    "properties": {
                        "mode": {"type": "string", "enum": ["position_hold", "free"]},
                        "stiffness": {"type": "number"},
                        "damping": {"type": "number"}},
                    "required": ["mode", "stiffness", "damping"]},
                "duration_s": {"type": "number"},
                "pass_criteria": {
                    "type": "object", "additionalProperties": False,
                    "properties": {
                        "min_base_height": {"type": "number"},
                        "max_drift": {"type": "number"},
                        "survive_s": {"type": "number"}},
                    "required": ["min_base_height", "max_drift", "survive_s"]},
            },
            "required": ["reasoning", "base_orientation_euler", "base_height",
                         "joint_pose", "high_friction_links", "control",
                         "duration_s", "pass_criteria"],
        },
    },
}


def _client():
    from openai import OpenAI
    return OpenAI(base_url=os.environ["AZURE_OPENAI_ENDPOINT"].rstrip("/"),
                  api_key=os.environ["AZURE_OPENAI_API_KEY"])


def _call(messages):
    c = _client()
    r = c.chat.completions.create(
        model=os.environ.get("AZURE_VLM_DEPLOYMENT", "gpt-5.4"),
        messages=messages, response_format=SPEC_SCHEMA,
        max_completion_tokens=2000)
    return json.loads(r.choices[0].message.content)


def _robot_block(robot_info):
    return (f"ROBOT: {robot_info.get('name','robot')}\n"
            f"JOINT NAMES ({len(robot_info.get('joints',[]))}): {robot_info.get('joints',[])}\n"
            f"LINK NAMES ({len(robot_info.get('links',[]))}): {robot_info.get('links',[])}\n")


def design(task, robot_info):
    msgs = [{"role": "system", "content": SYSTEM},
            {"role": "user", "content":
                f"TASK: {task}\n\n{_robot_block(robot_info)}\n"
                f"Design the scenario spec to test this task. Use ONLY the real joint/link "
                f"names above."}]
    return _call(msgs)


def revise(task, robot_info, prev_spec, feedback):
    msgs = [{"role": "system", "content": SYSTEM},
            {"role": "user", "content":
                f"TASK: {task}\n\n{_robot_block(robot_info)}\n"
                f"PREVIOUS SPEC:\n{json.dumps(prev_spec, indent=2)}\n\n"
                f"SIMULATION FEEDBACK (what happened):\n{feedback}\n\n"
                f"Diagnose the physical failure and output a REVISED spec that fixes it."}]
    return _call(msgs)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True)
    ap.add_argument("--robot-info", required=True, help="JSON file with name/joints/links")
    ap.add_argument("--prev-spec", default=None)
    ap.add_argument("--feedback", default=None)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    ri = json.loads(Path(a.robot_info).read_text())
    if a.prev_spec and a.feedback:
        spec = revise(a.task, ri, json.loads(Path(a.prev_spec).read_text()),
                      Path(a.feedback).read_text())
    else:
        spec = design(a.task, ri)
    Path(a.out).write_text(json.dumps(spec, indent=2))
    print(f"[designer] wrote spec -> {a.out}")
    print("reasoning:", spec.get("reasoning", "")[:300])
