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

0. DEFINE SUCCESS FIRST (do this before anything else, in "success_definition").
   Before you design any spec, decide what SUCCESS physically means for THIS task
   and THIS object. Pick the mode and say why:
     - STAY STILL / HOLD: the object should support itself, hold a pose, or not
       topple (a stool, bracket, statue, a mounted structure). Success = it stays
       up / stays put within tolerances. Usually "drive": null, static support/CoM.
     - MOVE / ACTUATE: the object should physically move or change pose under its
       own actuation (a limb reaching, a lever swinging, a slider extending, a gait).
     - TRANSMIT MOTION: the object is a MECHANISM whose whole point is passing motion
       from an input to an output (gearbox, clock, tourbillon, cryptex, worm drive).
       Success = driving the ONE input makes motion REACH the output — NOT standing
       still, and NOT merely "some part jiggles".
   State which mode applies and the ONE observable that decides pass/fail (e.g.
   "base height stays > 0.1 m", "output_joint rotates > 0.2 rad when input is
   driven"). The rest of the spec must SERVE this definition — do not, for example,
   set up a stand-still stability test for a gearbox whose success is transmission.

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
- Use "mode":"velocity" with a modest "target_velocity" (~3-6 rad/s for a rotating input,
  or a modest linear speed for a sliding one), or "position_sweep" over the joint's range
  for a lever. Choose drive method by WHAT THE TEST MUST PROVE, not by weight alone:
  * "servo" when friction, contact force, collision/jamming, load capacity, slip, or soft
    pin/closure tracking affects the verdict. The input must be allowed to slow or stall.
  * "direct_qpos" for a kinematic transmission whose motion is already expressed by exact
    equality/ratio constraints and whose test only asks whether coordinates propagate with
    the right path/ratio. This includes ultra-light watches and a planetary stage compiled
    to Willis constraints; no physical tooth-friction claim may be made from that run.
  Never choose direct_qpos merely because a mechanism is heavy: it can force geometry through
  collisions. Never choose servo merely because it is medium-weight: if contact/friction is
  intentionally replaced by exact constraints, servo adds numerical sensitivity without
  testing extra physics. Set
  "self_collision": true so gears/teeth actually contact and MESH (gears
  couple by tooth contact, not by a joint).
- Set "watch_joints" = the role map's "transmission" joints — the downstream joints the
  mechanism should drive. These are OBSERVED ONLY, never actuated. Set
  "min_watched_travel" to a small travel that counts as "it moved" (radians for rotary
  joints, meters for sliders).
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
                "success_definition": {"type": "string",
                    "description": "FIRST decide what success physically means for "
                                   "this task/object: STAY STILL/HOLD, MOVE/ACTUATE, "
                                   "or TRANSMIT MOTION — and the ONE observable that "
                                   "decides pass/fail. The rest of the spec must serve "
                                   "this."},
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
                    "description": "list of {joint, qpos}. Rotary joints use radians; slide joints use meters. Only real joint names."},
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
                        "drive_method": {"type": "string", "enum": ["servo", "direct_qpos"],
                            "description": "servo when friction/contact/collision/load/stall is part of the claim; direct_qpos when exact kinematic equality/ratio constraints replace contact and only propagation/ratio is being tested. Weight alone is not the criterion"},
                        "target_velocity": {"type": "number", "description": "velocity-mode rate (rad/s for spin joints, m/s for slide joints)"},
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
                    "required": ["input_joint", "mode", "drive_method", "target_velocity", "sweep",
                                 "duration_s", "self_collision", "watch_joints",
                                 "min_watched_travel", "output_joint", "propagation_path"]},
                "pass_criteria": {
                    "type": "object", "additionalProperties": False,
                    "properties": {
                        "min_base_height": {"type": "number"},
                        "max_drift": {"type": "number"},
                        "survive_s": {"type": "number"}},
                    "required": ["min_base_height", "max_drift", "survive_s"]},
                # WHAT THIS MACHINE HAS TO ACHIEVE, as code. The fields above only say
                # "it stayed put and something moved" — true of a machine whose tooth
                # counts are wrong, whose gripper never closes, whose ratchet slips
                # backwards. No fixed set of numeric fields can cover every mechanism, so
                # the criterion is a small Python program instead: it reads the recorded
                # trajectory and returns the checks it cares about.
                # HOW THIS MACHINE IS DRIVEN, as code. `drive` above is a form with two
                # boxes (velocity / position_sweep) and a whole class of mechanism fits in
                # neither: anything that stores energy and releases it is set to a pose and
                # then let go. Asked to express a trebuchet through `drive`, the designer
                # could only say "spin the throwing arm at 5 rad/s" — which turned the arm
                # two full turns and tested nothing about throwing. Same move as
                # `metrics_code`: where a fixed schema cannot span the space, use code.
                "control_code": {
                    "type": "string",
                    "description": (
                        "Optional Python defining setup(m, d) and/or control(m, d, t) "
                        "against the MuJoCo model/data. setup() runs once after settle "
                        "(set d.qpos[...] to wind/preload a pose); control() runs EVERY "
                        "step with elapsed seconds t and may drive, release at a moment, "
                        "or phase several joints. Overrides `drive` when present. stdlib "
                        "+ math + numpy as np only. For a machine NOTHING should drive "
                        "(a gravity release), still define control() with a bare `pass` "
                        "— an empty string means 'use the built-in `drive`', which spins "
                        "the input at a constant rate, and that is the opposite of "
                        "letting it go. Leave it \"\" only when a steady driven input is "
                        "genuinely right (a hand-cranked gearbox).")},
                "metrics_code": {
                    "type": "string",
                    "description": (
                        "Python defining check(traj, result) -> list of "
                        "{name, value, expected, passed, detail}. `traj` has t[], "
                        "joints{name:[rad]}, bodies{name:[[x,y,z] mm]}; `result` is the "
                        "sim metrics dict. Use only the stdlib and math. Return [] when "
                        "the description pins down nothing checkable.")},
            },
            "required": ["success_definition", "reasoning",
                         "base_orientation_euler", "base_height",
                         "joint_pose", "high_friction_links", "control",
                         "duration_s", "fixed_base", "drive", "pass_criteria",
                         "control_code", "metrics_code"],
        },
    },
}


def _client(base_url=None, api_key=None):
    from openai import OpenAI
    return OpenAI(base_url=(base_url or os.environ["AZURE_OPENAI_ENDPOINT"]).rstrip("/"),
                  api_key=api_key or os.environ["AZURE_OPENAI_API_KEY"])


def _call(messages, base_url=None, api_key=None, model=None):
    c = _client(base_url, api_key)
    selected_model = model or model_id()
    r = c.chat.completions.create(
        model=selected_model,
        messages=messages, response_format=SPEC_SCHEMA,
        max_completion_tokens=16000)
    try:
        from maker2.benchmarks.telemetry import record_openai_response
        record_openai_response(r, model=selected_model)
    except Exception:
        pass
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
    # The KEYS the recorded trajectory will actually carry. These are NOT necessarily
    # "<part>_spin" any more: sliders emit "<part>_slide" and free bodies "<part>_free".
    motion_key_by_part = robot_info.get("motion_key_by_part") or {}
    traj_txt = ""
    if motion_key_by_part:
        traj_txt = (
            "\nTRAJECTORY KEYS (metrics_code MUST use exactly these — traj[\"joints\"] is "
            "keyed by them, NOT by the URDF joint names above):\n"
            f"  joints_by_part: {motion_key_by_part}\n"
            f"  allowed_joint_keys: {robot_info.get('trajectory_joint_names', [])}\n"
            f"  bodies: {robot_info.get('links', [])}\n")
    # The user-facing output parts and the joint each one's motion is actually readable
    # through. These are dof=fixed — they have no joint of their own — so measuring one
    # means measuring its carrier. Resolved here rather than left to the designer: asked
    # to work it out itself it reached for a nearby gear, and a gear one press fit from
    # the output turns at nearly the right rate, so the ratio looked plausible while the
    # two parts the user actually watches were locked together turning as one.
    carried = robot_info.get("carried_parts") or {}
    carry_txt = ""
    if carried:
        lines = []
        for part, mount in sorted(carried.items()):
            # A mount may name another fixed part; walk to the first carrier with a real
            # trajectory key (spin/slide/free).
            seen, cur = set(), mount
            while cur and cur not in motion_key_by_part and cur not in seen:
                seen.add(cur)
                cur = carried.get(cur, "")
            lines.append(
                f"  {part} rides {mount}"
                + (f" -> measure traj[\"joints\"][\"{motion_key_by_part[cur]}\"]" if cur in motion_key_by_part
                   else "  (no moving carrier: this part cannot move)"))
        carry_txt = (
            "\nOUTPUT PARTS AND THE JOINT THAT CARRIES EACH (these are the parts the USER "
            "watches — hands, jaws, an output shaft. Measure the joints named here and no "
            "others. An intermediate gear/slider/arbor is NOT an output just because its "
            "reading looks convenient):\n" + "\n".join(lines) + "\n")
    return (f"ROBOT: {robot_info.get('name','robot')}\n"
            f"JOINT NAMES ({len(robot_info.get('joints',[]))}): {robot_info.get('joints',[])}\n"
            f"LINK NAMES ({len(robot_info.get('links',[]))}): {robot_info.get('links',[])}\n"
            f"{role_txt}{traj_txt}{carry_txt}")


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
    print("success:", spec.get("success_definition", "")[:200])
    print("reasoning:", spec.get("reasoning", "")[:300])
