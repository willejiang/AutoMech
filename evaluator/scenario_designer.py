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
import json, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline_context import PIPELINE
from sim_backends import model_id


SYSTEM = PIPELINE + """You are a robotics simulation SCENARIO DESIGNER. Given a natural-language
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

POLICY MATCH (RL tasks): existing trained policies assume a standard morphology —
quadruped (Cassie/ANYmal/dog12: 4 legs, hips+knees) or biped. If the robot needs a
learned gait but its structure matches NO known policy template, that is a
worker-level problem: the structure should be redesigned to match a policy-ready
morphology, OR kept as-is and trained from scratch (slow). Note this in reasoning so
the loop can ask the user redesign-vs-keep.

MACHINERY (gearbox / worm drive / gear train / clock / tourbillon / cryptex):
When the object is a MECHANISM whose point is transmitting motion, do NOT test it by
standing still — that proves nothing. You are given a ROLE MAP (deterministic, from the
URDF topology) naming every joint's role. OBEY IT:
- Set "fixed_base": true (the mechanism is bench-mounted; it must not just topple).
- Drive EXACTLY the ONE "driver_input" from the role map as "input_joint". This is the
  single real power source (the crank/handle/winder/mainspring). A real mechanism has ONE
  input that PROPAGATES through the train — NEVER drive multiple joints at once, and NEVER
  drive a "transmission" or "free_unrelated" joint.
- Use "mode":"velocity" with a modest "target_velocity" (~3-6 rad/s) for a rotating input,
  or "position_sweep" over the joint's range for a lever. Set "self_collision": true so
  gears/teeth actually contact and MESH (gears couple by tooth contact, not by a joint).
- Set "watch_joints" = the role map's "transmission" joints — the downstream gears the mesh
  should drive. These are OBSERVED ONLY, never actuated. Set "min_watched_travel" to a small
  angle (~0.05-0.2 rad) that counts as "it moved".
- Declare "propagation_path" (the role map's ordered input->...->output chain) and
  "output_joint" (the role map's output — the far end of the train). The test PASSES only
  when driving the input makes motion REACH "output_joint" — NOT merely that "some watched
  joint moved". A train that turns near the input but is dead at the output is a FAILURE.
For a NON-machine (a bracket, a stool, a statue) leave "drive": null and use the
static support/CoM stability reasoning above.

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
                "fixed_base": {"type": "boolean",
                    "description": "bench-mount the object (true for a machine/mechanism)"},
                "drive": {
                    "type": ["object", "null"],
                    "additionalProperties": False,
                    "description": "for a MACHINE: actuate an input joint and watch "
                                   "transmission. null for a static-stability test.",
                    "properties": {
                        "input_joint": {"type": "string",
                            "description": "the joint to drive (crank/input). Real name."},
                        "mode": {"type": "string", "enum": ["velocity", "position_sweep"]},
                        "target_velocity": {"type": "number", "description": "rad/s for velocity mode"},
                        "sweep": {"type": "array", "items": {"type": "number"},
                            "description": "[lo, hi] rad for position_sweep mode"},
                        "duration_s": {"type": "number"},
                        "self_collision": {"type": "boolean",
                            "description": "true so gears/teeth actually contact & mesh"},
                        "watch_joints": {"type": "array", "items": {"type": "string"},
                            "description": "downstream joints that should move if it transmits"},
                        "min_watched_travel": {"type": "number",
                            "description": "rad a watched joint must move to count as driven"},
                        "output_joint": {"type": "string",
                            "description": "the far end of the train; motion must REACH this to pass"},
                        "propagation_path": {"type": "array", "items": {"type": "string"},
                            "description": "ordered input->...->output joint chain"}},
                    "required": ["input_joint", "mode", "target_velocity", "sweep",
                                 "duration_s", "self_collision", "watch_joints",
                                 "min_watched_travel", "output_joint", "propagation_path"]},
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
                         "duration_s", "fixed_base", "drive", "pass_criteria"],
        },
    },
}


def _client(base_url=None, api_key=None):
    from openai import OpenAI
    return OpenAI(base_url=(base_url or os.environ["AZURE_OPENAI_ENDPOINT"]).rstrip("/"),
                  api_key=api_key or os.environ["AZURE_OPENAI_API_KEY"])


def _call(messages, base_url=None, api_key=None, model=None):
    c = _client(base_url, api_key)
    r = c.chat.completions.create(
        model=model or model_id(),
        messages=messages, response_format=SPEC_SCHEMA,
        max_completion_tokens=2000)
    txt = (r.choices[0].message.content or "").strip()
    if not txt.lstrip().startswith("{"):        # gateway ignored the schema -> strip prose
        import re
        m = re.search(r"\{.*\}", txt, re.DOTALL)
        if m:
            txt = m.group(0)
    return json.loads(txt)


def _robot_block(robot_info):
    roles = robot_info.get("roles") or {}
    role_txt = ""
    if roles.get("driver_input"):
        role_txt = (
            f"\nROLE MAP (deterministic — obey it for a driven test):\n"
            f"  driver_input (drive ONLY this): {roles.get('driver_input')}\n"
            f"  output_joint (motion must reach): {roles.get('output_joint')}\n"
            f"  transmission (watch, never drive): {roles.get('transmission')}\n"
            f"  free_unrelated (ignore): {roles.get('free_unrelated')}\n"
            f"  propagation_path: {roles.get('propagation_path')}\n")
    return (f"ROBOT: {robot_info.get('name','robot')}\n"
            f"JOINT NAMES ({len(robot_info.get('joints',[]))}): {robot_info.get('joints',[])}\n"
            f"LINK NAMES ({len(robot_info.get('links',[]))}): {robot_info.get('links',[])}\n"
            f"{role_txt}")


def design(task, robot_info, test=None, *, base_url=None, api_key=None, model=None):
    sub = f"\n\nTHIS TEST: {test.get('name')} — {test.get('goal')}" if test else ""
    msgs = [{"role": "system", "content": SYSTEM},
            {"role": "user", "content":
                f"TASK: {task}{sub}\n\n{_robot_block(robot_info)}\n"
                f"Design the scenario spec to test this. Use ONLY the real joint/link "
                f"names above."}]
    return _call(msgs, base_url, api_key, model)


def revise(task, robot_info, prev_spec, feedback, *, base_url=None, api_key=None, model=None):
    msgs = [{"role": "system", "content": SYSTEM},
            {"role": "user", "content":
                f"TASK: {task}\n\n{_robot_block(robot_info)}\n"
                f"PREVIOUS SPEC:\n{json.dumps(prev_spec, indent=2)}\n\n"
                f"SIMULATION FEEDBACK (what happened):\n{feedback}\n\n"
                f"Diagnose the physical failure and output a REVISED spec that fixes it."}]
    return _call(msgs, base_url, api_key, model)


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
