#!/usr/bin/env python3
"""Evaluator diagnosis + video: the provider-agnostic engine pieces the maker loop
calls after a physics run.

- diagnose_physics(): a VLM WATCHES the recording (keyframes) + the metrics and
  decides pass/fail — overriding the raw moved_count/exploded numbers — and on FAIL
  classifies the CAUSE so the caller's loop knows what to fix:
    structure  -> the MODEL is wrong (gears don't mesh, a part missing/mis-sized)
    scenario   -> the TEST is wrong (wrong input joint, absurd velocity/params)
    framing    -> the CAMERA can't SEE the mechanism (can't judge -> reframe)
- encode_mp4(): stitch rgb_*.png frames into a browser-playable MP4 (+faststart) via
  imageio-ffmpeg's bundled ffmpeg (no system ffmpeg needed).

Like strategy_selector/scenario_designer, the LLM call is gateway-injectable
(base_url/api_key/model) so a maker can route it through its own gateway; it defaults
to the Azure env vars for standalone use. Tolerates a gateway that ignores strict
response_format (parses the JSON out of prose + accepts renamed keys)."""
from __future__ import annotations

import base64
import glob
import json
import os
import re
from pathlib import Path


def _client(base_url=None, api_key=None):
    from openai import OpenAI
    return OpenAI(base_url=(base_url or os.environ["AZURE_OPENAI_ENDPOINT"]).rstrip("/"),
                  api_key=api_key or os.environ["AZURE_OPENAI_API_KEY"])


def _parse_json(txt: str) -> dict:
    t = (txt or "").strip()
    if not t:
        return {}
    if not t.lstrip().startswith("{"):
        m = re.search(r"\{.*\}", t, re.DOTALL)
        if m:
            t = m.group(0)
    try:
        return json.loads(t)
    except Exception:
        return {}


def _pick(d: dict, keys, allowed, default):
    """First value under any of `keys` (gateways rename schema keys) that is in
    `allowed`; else scan ALL values for a valid enum member; else default."""
    for k in keys:
        v = d.get(k)
        if isinstance(v, str) and v.strip().lower() in allowed:
            return v.strip().lower()
    for v in d.values():
        if isinstance(v, str) and v.strip().lower() in allowed:
            return v.strip().lower()
    return default


def _sample_frames(frames_dir: str, k: int = 10):
    frames = sorted(glob.glob(os.path.join(frames_dir, "rgb_*.png")))
    if not frames:
        return []
    if len(frames) <= k:
        return frames
    step = len(frames) / k
    return [frames[int(i * step)] for i in range(k)]


def _b64(path: str) -> str:
    return base64.b64encode(Path(path).read_bytes()).decode()


# --------------------------------------------------------------------------- #
# Video: frames -> MP4
# --------------------------------------------------------------------------- #

def encode_mp4(frames_dir: str, out_path: str, fps: int = 12) -> str | None:
    """Stitch rgb_*.png frames into an MP4 via imageio-ffmpeg's bundled ffmpeg.
    Returns out_path on success, else None (missing binary / zero frames -> no
    video). +faststart puts the moov atom first so a browser <video> can play it."""
    import subprocess
    frames = sorted(glob.glob(os.path.join(frames_dir, "rgb_*.png")))
    if not frames:
        return None
    try:
        import imageio_ffmpeg
        ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception as e:
        print(f"[video] no ffmpeg ({e}); skipping video")
        return None
    pattern = os.path.join(frames_dir, "rgb_*.png").replace("\\", "/")
    cmd = [ffmpeg, "-y", "-framerate", str(fps), "-pattern_type", "glob",
           "-i", pattern, "-c:v", "libx264", "-pix_fmt", "yuv420p",
           "-movflags", "+faststart",
           "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", out_path]
    try:
        subprocess.run(cmd, check=True, capture_output=True, timeout=120)
    except Exception:
        # Windows ffmpeg glob support varies; fall back to a frame-list concat.
        try:
            lst = os.path.join(frames_dir, "_frames.txt")
            with open(lst, "w") as f:
                for fr in frames:
                    f.write(f"file '{os.path.abspath(fr)}'\n")
            cmd2 = [ffmpeg, "-y", "-r", str(fps), "-f", "concat", "-safe", "0",
                    "-i", lst, "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart",
                    "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", out_path]
            subprocess.run(cmd2, check=True, capture_output=True, timeout=120)
        except Exception as e2:
            print(f"[video] mp4 encode failed ({e2}); skipping video")
            return None
    return out_path if os.path.exists(out_path) and os.path.getsize(out_path) > 0 else None


# --------------------------------------------------------------------------- #
# Diagnosis: VLM verdict over the recording + cause classification
# --------------------------------------------------------------------------- #

_DIAG_SYSTEM = """You are the physics EVALUATOR for an automated CAD pipeline. You are shown
keyframes (time order, 0 = start) from a simulation that DROVE a mechanism's input joint,
plus measured metrics AND a set of pre-computed METRIC SIGNALS. Decide, from what the PARTS
ACTUALLY DO on screen AND the signals, whether the mechanism works for the task. Your visual
verdict OVERRIDES the raw pass/fail, but you MUST respect the hard metric signals below.

If it FAILS, classify the CAUSE precisely AND name the exact culprit:
- "structure": the MODEL is at fault — gears/teeth don't mesh, parts interpenetrate or
  fly apart, a needed part is missing or the wrong size, the drivetrain is not actually
  connected, OR the driven input joint is JAMMED and cannot rotate. Fixing this needs the
  CAD to be redesigned. Set "culprit_part" to the EXACT namespaced link/joint at fault
  (link/joint names look like "<sub_id>_<name>", e.g. "sub_going_train_escape_wheel").
  For a dead drivetrain, that is the FIRST joint in the propagation path whose downstream
  travel is ~0 — the break point. Set "culprit_sub" to that part's "<sub_id>" prefix.
- "interface": two subassemblies don't couple where they meet — a gear on one sub doesn't
  reach the gear on another, a seam is misaligned. The individual subs may be fine; their
  JUNCTION is wrong. Set culprit_sub to one side if identifiable, else "".
- "scenario": the TEST is at fault — the wrong joint was driven, the velocity/params are
  absurd, the watched joints are wrong, so the test doesn't fairly exercise the machine.
  The model may be fine; the TEST should be redesigned. culprit_part/sub = "".
- "camera": you CANNOT SEE the mechanism clearly enough to judge — it's too small/far,
  off-frame, or occluded. The model and test may be fine; the CAMERA must be fixed.
  culprit_part/sub = "".

When the cause is not a single part (a whole-sub or interface fault), leave "culprit_part"
empty but still set "culprit_sub" when you can identify the responsible subassembly.

HARD RULES (these override your visual impression):
1. If the signal INPUT_STALLED is true, the driven joint did NOT turn even though it was
   commanded — the drivetrain is physically jammed. This is NEVER "camera". Return
   verdict=fail with cause "structure" (the input is blocked) unless the driven joint is
   clearly the WRONG joint to drive, in which case "scenario".
2. If the signal FREE_SPIN is true, a downstream joint spun FAR faster than the input
   command could ever produce — it is flying loose / not actually meshed, not
   transmitting. This is NEVER "camera"; it is "structure" (parts not connected).
3. Only choose "camera" when the input DID move a reasonable amount (INPUT_MOVED true)
   yet you still cannot see the resulting downstream motion. If the input didn't move,
   the problem is the mechanism or the test, not the camera.
4. If the signal OUTPUT_DEAD is true, the input turned and some gears moved but motion
   NEVER REACHED the declared output joint — the drivetrain is broken MIDWAY (a missing
   or non-meshing gear between the moving part and the output). This is NEVER "camera";
   return verdict=fail with cause "structure", and set culprit_part to the BREAK JOINT
   named in the signals (the first path joint with ~0 travel).
5. If the signal BLEW_APART is true, one or more parts were EJECTED far off their settle
   position (see EXPLODED_PARTS) — the assembly is unconstrained and physically flew
   apart. This is NEVER "camera" even if the frames look empty (the parts left the
   frame). Return verdict=fail with cause "structure"; set culprit_part to the
   worst-ejected part named in EXPLODED_PARTS.
6. TWO-STAGE JUDGMENT. STAGE 1 is STABILITY: the machine is dropped under gravity with NO
   drive and must hold together. If STABILITY_VERDICT is FAIL or STABILITY_EXPLODED is
   true, the machine falls apart just sitting on the bench — return verdict=fail, cause
   "structure", and do NOT pass it on any function grounds. A thing that cannot even exist
   stably transmits nothing. Only if STAGE 1 held do you evaluate STAGE 2 (function).
7. If BARELY_TURNED is true, the driven input swept almost no angle (< ~0.3 rad), so the
   downstream parts that "moved" only jittered during settle — this does NOT prove the
   mechanism transmits. Do NOT pass on 5/5-moved when the input barely turned; return
   verdict=fail, cause "structure" (the input is effectively jammed / the train does not
   turn), unless the input clearly SHOULD not turn for this task.

Always fill "reason" with a concrete one-sentence explanation. Respond ONLY with the JSON
schema.

REFERENCE CASES — how a failure's MOTION reveals its ROOT CAUSE (use these to write a
precise, actionable reason, and to set culprit_part):
- COLLAPSE / SINKING (parts move DOWNWARD, end_z drops below its settle value, the stack
  tilts or falls over): something is NOT FIXED to the world. A plate/bridge/frame that
  should be the static base was left free (dof != fixed) or has no ground support, so
  gravity pulls the assembly down. Fix = anchor the base structure (make the lower
  plate/frame fixed and grounded). Reason should say "X collapsed downward -> the base is
  unanchored".
- LATERAL EJECTION / FLING (parts shoot SIDEWAYS or explode to huge displacement while
  end_z stays ~0 and tilt ~0): this is NOT gravity — it is CONTACT FORCE from parts that
  START OVERLAPPING. Two solids initialized interpenetrating generate an enormous spring
  push-apart impulse the instant the sim steps, flinging them apart (often hundreds of
  metres). But FIRST decide WHICH overlap it is — the fix differs:
    (a) Parts that should be SEPARATE overlap (two non-coaxial parts placed on top of each
        other, meshing gears set closer than one center-distance): the coordinates are
        WRONG. Fix = move them to correct positions (meshing gears exactly one
        center-distance apart; distinct parts to distinct locations).
    (b) Parts that are SUPPOSED to be nested/coaxial overlap (a gear sitting on ITS OWN
        arbor/shaft, a wheel on its staff, a sleeve on a post): they are correctly placed —
        the fault is that their solids intersect (a bore smaller than the shaft, or no
        collision filtering between a part and its own shaft). Fix = give the gear a bore
        that clears the shaft, OR exclude that coaxial pair from self-collision — do NOT
        move them apart, they belong on the same axis.
  A tell for (b): the two ejected parts share the SAME x,y and differ only slightly in z,
  and one is a shaft/arbor/staff while the other is a gear/wheel/sleeve. Do NOT recommend
  separating a gear from its own arbor. Name the pair and say which case it is.
- AXIAL SLIP-OFF (a gear/wheel flies along its own shaft axis and off the end): the shaft
  is too SHORT or the gear sits at the shaft tip with no axial constraint, so meshing
  contact's axial component pushes it off. Fix = lengthen the shaft and seat the gear on a
  mid-shaft station with a shoulder/bearing on each side. This is still a structure fault.
- DEAD / JAMMED (input barely turns, ~0 downstream motion, nothing ejects): meshing teeth
  are too far apart to engage OR two parts are wedged. Not a fixturing problem — a spacing
  one. Fix = set the meshing pair exactly one center-distance apart.

READING CONTACTS (call read_contacts for any explosion/blew-apart) — the depth and the
radial distance r of each contact point decode the ROOT CAUSE without guessing:
- FIRST check is_mesh_pair on each contact. If a contact is between a DECLARED MESH PAIR
  (is_mesh_pair=true), an overlap there means the two meshing gears' CENTER DISTANCE IS TOO
  SMALL — they are jammed into each other (a big gear can swallow a small one, so its
  contact point sits deep inside and r LOOKS small, but they are NOT coaxial: check
  center_dist_xy_mm > 0). The fix is ALWAYS to SPREAD the two gear centers apart to
  center distance = module*(z1+z2)/2. NEVER report "open a bore" or "coaxial poke" for a
  mesh pair — that is the wrong fix and sends the agent in circles. Name the two gears and
  say "meshing gears overlap -> center distance too small -> spread them apart".
- A pair MuJoCo reports as overlapping that precheck (real geometry) does NOT report is a
  COLLISION-HULL ARTIFACT, not a design fault. The commonest is an AXIAL POKE: a shaft's
  flat end-face pushes into a coaxial part it passes through (bridge/bearing/plate). Tell:
  the two bodies are coaxial (coaxial=true / center_dist_xy_mm ~ 0) and the contact r is
  SMALL (~0-1.5 mm, inside the bore) yet depth is large. The bore is big enough (real
  geometry clears) — the shaft is just too LONG / seated at the wrong z. Fix = shorten the
  shaft or raise the coaxial part's z; do NOT widen the bore, do NOT separate them.
- A pair BOTH MuJoCo and precheck report is a REAL overlap. If r ~ the bore-wall radius,
  the bore is too small for the shaft (radial) -> widen the bore. If the two are unrelated
  parts placed on top of each other, the coordinates are wrong -> move one.
- A contact out at a gear's TOOTH radius (r ~ pitch radius, large) with SHALLOW depth is
  NORMAL meshing between a declared mesh pair — do NOT flag it as a fault."""

_DIAG_TOOL_HINT = """

You may FIRST call read-only tools to investigate before deciding:
- read_contacts() — THE decisive tool for any explosion / blew-apart / structure fault:
  the simulator's own overlap verdict at the design pose, per body pair, with depth and
  the contact point's radial distance to each body's axis, PLUS the real-geometry overlap
  check to compare against. Call this BEFORE blaming a part — it tells you exactly which
  pairs overlap and whether it is a real fault or a hull artifact. Do not guess "part A is
  too wide" from keyframes when this gives you the measured truth.
- read_sim_result(pointer?) — the FULL metrics incl. per-part displaced_parts (how far
  each body flew, mm). ALWAYS call this when the keyframes look empty/ambiguous: parts
  that left the frame are still recorded here, so you can judge structure from the numbers.
- read_run_log(regex) — search this run's build/precheck/physics narration.
- view_frame(index) — re-render one keyframe to look closer.
When you have enough evidence, STOP calling tools and reply with ONLY the JSON verdict."""

_DIAG_SCHEMA = {
    "type": "json_schema",
    "json_schema": {
        "name": "physics_diagnosis", "strict": True,
        "schema": {
            "type": "object", "additionalProperties": False,
            "properties": {
                "verdict": {"type": "string", "enum": ["pass", "fail"]},
                "cause": {"type": "string",
                          "enum": ["none", "structure", "interface",
                                   "scenario", "camera"]},
                "culprit_part": {"type": "string",
                    "description": "the EXACT namespaced link/joint at fault (e.g. "
                                   "'sub_going_train_escape_wheel'), or '' if not a "
                                   "single-part fault. For a dead drivetrain, name the "
                                   "FIRST joint in the path whose downstream motion is 0."},
                "culprit_sub": {"type": "string",
                    "description": "the subassembly id the culprit part belongs to "
                                   "(the '<sub_id>' namespace prefix), or '' if unknown."},
                "reason": {"type": "string"},
            },
            "required": ["verdict", "cause", "culprit_part", "culprit_sub", "reason"],
        },
    },
}


def _kb_notes(task: str, k: int = 2) -> str:
    """Diagnosis method notes from the evaluator corpus. Best-effort: an unbuilt or
    missing index simply means no notes, never a failed diagnosis."""
    try:
        from maker2 import kb
        hits = kb.search(f"locate the fault when a mechanism partly works: {task}",
                         collection="evaluator", k=k)
    except Exception:
        return ""
    out = []
    for h in hits or []:
        text = (h.get("text") if isinstance(h, dict) else None) or ""
        if text.strip():
            out.append(text.strip()[:1500])
    return "\n\n".join(out)


def _per_joint_travel_txt(frames_dir: str, drive: dict) -> str:
    """How far each joint ACTUALLY turned, from the recorded trajectory.

    This is the measurement that turns "5/6 joints moved" into "hour_wheel_spin is the
    one that did not" — and, read along the propagation path, into "the chain is alive up
    to intermediate_pinion and dead after it", which names the culprit. Returns "" when
    no trajectory was recorded, so an older run degrades to the previous behaviour."""
    try:
        p = Path(frames_dir).parent / "trajectory.json"
        doc = json.loads(p.read_text(encoding="utf-8"))
        joints = doc.get("joints") or {}
        if not joints:
            return ""
    except Exception:
        return ""
    driver = str(drive.get("input_joint") or "")
    rows = []
    for name, seq in joints.items():
        if not seq:
            continue
        net = abs(float(seq[-1]) - float(seq[0]))
        total = sum(abs(float(b) - float(a)) for a, b in zip(seq, seq[1:]))
        rows.append((total, net, name))
    if not rows:
        return ""
    rows.sort(reverse=True)
    lines = ["PER-JOINT TRAVEL (measured, radians — this is who moved and who did not):"]
    for total, net, name in rows:
        tag = ""
        if driver and (name == driver or name.startswith(driver)
                       or driver.endswith(name.replace("_spin", ""))):
            tag = "  <- DRIVEN INPUT"
        elif total < 0.05:
            tag = "  <- DID NOT MOVE"
        lines.append(f"  {name:32s} total={total:9.4f}  net={net:9.4f}{tag}")
    return "\n".join(lines) + "\n\n"


def _metric_signals(spec: dict, metrics: dict) -> dict:
    """Turn raw metrics into hard, unambiguous physics signals that gate the verdict.

    These catch the failure modes a VLM misreads from keyframes alone:
    - input_stalled: the driven joint barely moved though it was velocity-commanded ->
      the drivetrain is JAMMED (input blocked). NOT a camera problem.
    - free_spin: a watched joint moved FAR more than the input command could produce ->
      it's flying loose / not meshed, not transmitting torque. NOT a camera problem.
    Returns {input_moved, input_stalled, free_spin, expected_input_travel, notes[]}."""
    drive = (spec or {}).get("drive") or {}
    m = metrics or {}
    it = float(m.get("input_travel") or 0.0)

    # What the input SHOULD have swept if the motor turned freely: |vel| * duration
    # (velocity mode) or the sweep span (position mode). A conservative floor.
    dur = float(drive.get("duration_s") or (spec or {}).get("duration_s") or 3.0)
    if drive.get("mode") == "position_sweep":
        sw = drive.get("sweep") or [0.0, 6.283]
        expected = abs(float(sw[1]) - float(sw[0]))
    else:
        expected = abs(float(drive.get("target_velocity") or 3.0)) * dur
    expected = max(expected, 0.1)

    # Stalled: commanded to sweep a meaningful angle but moved <5% of it (and <0.1 rad).
    input_stalled = bool(it < 0.1 and it < 0.05 * expected)
    input_moved = not input_stalled

    # Free-spin: any watched joint swept WAY past the input command (2x the expected
    # input sweep) while the input itself barely moved -> not a real reduction, it's
    # an unconstrained joint kicked loose. (A real reducer never outruns its input.)
    watched = m.get("watched") or {}
    max_watched = max((abs(float(v)) for v in watched.values()), default=0.0)
    free_spin = bool(max_watched > 2.0 * expected and it < 0.5 * max_watched)

    # Output-not-reached: the input turned and SOME transmission moved, but the
    # declared OUTPUT joint (the far end of the train) did NOT -> the drivetrain is
    # broken MIDWAY. This is a structure fault, never a framing one. Only meaningful
    # when the runner reported output_reached (E-BENCH declared an output_joint).
    moved_count = int(m.get("moved_count") or 0)
    output_reached = m.get("output_reached")
    output_dead = bool(input_moved and moved_count >= 1
                       and output_reached is False)

    # Break joint: the FIRST joint in the propagation path (input -> ... -> output) whose
    # travel is ~0 while an upstream joint moved. That is the exact point the drivetrain
    # dies -> the culprit part. Uses the ordered path + per-joint `watched` travel.
    break_joint = _break_joint(drive, m, input_moved)

    # Exploded parts: bodies that flew far from their settle position (the runner's
    # displaced_parts, worst-first, disp in mm). A part >0.5 m out is physically ejected
    # -> the assembly is unconstrained / blew apart. This is a hard structure fault the
    # VLM cannot see once the part has left the frame.
    displaced = m.get("displaced_parts") or []
    exploded_parts = [p for p in displaced
                      if isinstance(p, dict) and float(p.get("disp_mm") or 0.0) > 500.0]
    blew_apart = bool(m.get("exploded")) or bool(exploded_parts)

    # Distinguish the TWO explosion modes from the motion (deterministic, so the reason
    # is grounded in physics, not a guess): a COLLAPSE sinks (end_z drops / it tilts),
    # a FLING shoots sideways with little vertical change -> initial rigid overlap.
    fling_mode = ""
    if blew_apart:
        end_z = float(m.get("end_z") or 0.0)
        tilt = float(m.get("max_tilt_deg") or 0.0)
        if end_z < -0.02 or tilt > 15.0:
            fling_mode = "collapse"       # went down / toppled -> base not anchored
        else:
            fling_mode = "lateral"        # shot sideways, level -> parts started overlapping

    notes = []
    if input_stalled:
        notes.append(f"INPUT_STALLED: input joint '{m.get('input_joint')}' moved only "
                     f"{it:.3f} rad but was commanded to sweep ~{expected:.2f} rad "
                     f"-> the drivetrain is jammed (structure). The usual cause is that a "
                     f"part is WEDGED into the rotating input: either (a) a passive part "
                     f"(bearing/washer/spacer/plate/hand) interpenetrates the shaft/gear it "
                     f"surrounds and the contact friction brakes it -> give that part a "
                     f"clearing bore or exclude the coaxial contact, do NOT move the whole "
                     f"train; or (b) the driven gear's teeth overlap its meshing partner "
                     f"deeper than a clearance fit (center distance too small) -> set the "
                     f"pair exactly module*(z1+z2)/2 apart. Look for the deepest contact on "
                     f"the input joint to tell which.")
    if free_spin:
        notes.append(f"FREE_SPIN: a watched joint moved {max_watched:.1f} rad, far "
                     f"beyond the ~{expected:.2f} rad input command, while the input "
                     f"moved {it:.3f} rad -> parts flying loose / not meshed (structure).")
    if output_dead:
        notes.append(f"OUTPUT_NOT_REACHED: the input turned and some gears moved, but "
                     f"the output joint '{m.get('output_joint')}' did NOT move "
                     f"(travel {m.get('output_travel')}) -> the train is broken midway "
                     f"(structure).")
    if break_joint:
        notes.append(f"BREAK_JOINT: '{break_joint}' is the first joint in the path whose "
                     f"motion is ~0 while an upstream joint moved -> it is the exact "
                     f"culprit part.")
    if exploded_parts:
        worst = ", ".join(f"{p.get('part')} ({float(p.get('disp_mm')) / 1000.0:.1f} m)"
                          for p in exploded_parts[:4])
        if fling_mode == "collapse":
            diag = ("-> the assembly COLLAPSED downward: its base structure is not "
                    "anchored/fixed to the world.")
        elif fling_mode == "lateral":
            diag = ("-> parts shot SIDEWAYS while staying level: they STARTED OVERLAPPING "
                    "(rigid interpenetration), so the contact solver flung them apart. If "
                    "the pair is a gear on its OWN arbor/shaft (same x,y, small z gap), the "
                    "fix is a clearing bore or self-collision exclusion — NOT moving them "
                    "apart; otherwise their coordinates are wrong.")
        else:
            diag = "-> the assembly is unconstrained and blew apart (structure)."
        notes.append(f"EXPLODED_PARTS: parts flew off their settle position: {worst} "
                     f"{diag} (the video may look empty because the parts left the frame.)")
    return {"input_moved": input_moved, "input_stalled": input_stalled,
            "free_spin": free_spin, "output_dead": output_dead,
            "break_joint": break_joint, "exploded_parts": exploded_parts,
            "blew_apart": blew_apart, "fling_mode": fling_mode,
            "output_reached": bool(output_reached) if output_reached is not None else None,
            "expected_input_travel": round(expected, 3), "notes": notes}


_MOVE_EPS = 0.05          # rad; below this a joint is considered "did not move"


def _break_joint(drive: dict, metrics: dict, input_moved: bool) -> str:
    """The first joint in the propagation path whose travel is ~0 while an upstream
    joint moved — the exact point the drivetrain dies. "" if none (or no path)."""
    if not input_moved:
        return ""
    path = (drive or {}).get("propagation_path") or []
    if not path:
        return ""
    watched = metrics.get("watched") or {}
    input_joint = metrics.get("input_joint") or (drive or {}).get("input_joint")
    it = abs(float(metrics.get("input_travel") or 0.0))

    def travel(jn: str) -> float:
        if jn == input_joint:
            return it
        if jn == metrics.get("output_joint"):
            return abs(float(metrics.get("output_travel") or 0.0))
        return abs(float(watched.get(jn, 0.0)))

    moved_upstream = False
    for jn in path:
        t = travel(jn)
        if t >= _MOVE_EPS:
            moved_upstream = True
        elif moved_upstream:
            return jn          # first dead joint after something moved = the break
    return ""


def _culprit_sub_of(part: str, robot_info: dict) -> str:
    """Infer the subassembly id from a namespaced link/joint name '<sub_id>_<name>'.
    Matches the longest known sub id that prefixes `part`; else the first two tokens."""
    if not part:
        return ""
    subs = [s.get("id", "") for s in (robot_info or {}).get("subsystems", []) if s.get("id")]
    # subsystems ids are stems (e.g. 'going_train'); the namespaced link is
    # 'sub_going_train_<link>'. Match the longest sub id that appears as a segment.
    best = ""
    for sid in subs:
        if sid and (f"_{sid}_" in f"_{part}_" or part.startswith(f"sub_{sid}_")
                    or part.startswith(f"{sid}_")):
            if len(sid) > len(best):
                best = sid
    if best:
        return best
    # Fallback: strip a leading 'sub_' and take the id up to the last 2 underscores.
    toks = part.split("_")
    return "_".join(toks[:2]) if len(toks) > 2 else part


def _measured_reason(metrics) -> str:
    """A reason built from DETERMINISTIC geometry already measured while the MJCF was
    built, for when the VLM returns a verdict with no explanation. These faults are exact
    and are the usual cause of a partial transmission, so they beat an empty string and
    they beat a generic note."""
    m = metrics or {}
    parts = []
    inter = m.get("interferences") or []
    if inter:
        worst = ", ".join(f"{i['a']} and {i['b']} ({i['overlap_mm3']}mm3)"
                          for i in inter[:4])
        parts.append(f"{len(inter)} pair(s) of parts occupy the same space: {worst}. "
                     f"A solid driven through a wheel stops it turning.")
    loose = m.get("loose_gears") or []
    if loose:
        worst = ", ".join(f"{g['gear']} on {g['shaft']} ({g['clearance_mm']}mm)"
                          for g in loose[:4])
        parts.append(f"{len(loose)} gear(s) sit too loosely on their shafts to be driven: "
                     f"{worst}.")
    fits = [f for f in (m.get("bore_fit_faults") or []) if f.get("impossible")]
    if fits:
        worst = ", ".join(f"{f['part']} on {f['shaft']}" for f in fits[:4])
        parts.append(f"{len(fits)} part(s) have a bore smaller than the shaft they mount "
                     f"on and cannot be assembled: {worst}.")
    return " ".join(parts)


def _diag(verdict, cause, reason, culprit_part="", culprit_sub=""):
    # `reason` is the ACTIONABLE part of the diagnosis — for a jammed train it carries the
    # whole "here is what to look at and what to change" instruction. A 400-char cap cut
    # that mid-word ("...exclude the coaxial con"), so the agent was handed half a sentence
    # and acted on it. These notes are machine-generated and bounded (a few hundred chars
    # per signal, a handful of signals), so the cap only ever truncated real content.
    return {"verdict": verdict, "cause": cause, "reason": str(reason)[:2000],
            "culprit_part": culprit_part or "", "culprit_sub": culprit_sub or ""}


# --------------------------------------------------------------------------- #
# Read-only investigation tools: the VLM can pull the FULL sim data (not just the
# summarized metrics it is prompted with) and re-inspect any frame, so it can still
# reason about an "empty" video — the parts flew off, but the numbers are all here.
# --------------------------------------------------------------------------- #

_DIAG_TOOLS = [
    {"type": "function", "function": {
        "name": "read_sim_result",
        "description": "Read the FULL sim_result.json for this run: every metric, the "
                       "per-body displaced_parts (name + how far each flew, mm), input "
                       "travel, exploded flag, n_frames. Use this when the video looks "
                       "empty or ambiguous — the numbers say what physically happened.",
        "parameters": {"type": "object", "properties": {
            "pointer": {"type": "string", "description": "optional dotted sub-path, e.g. "
                        "'metrics.displaced_parts'"}}, "additionalProperties": False}}},
    {"type": "function", "function": {
        "name": "read_run_log",
        "description": "Regex-search this run's run.log with a few lines of context "
                       "(build/precheck/physics narration). Use to find WHY a part is "
                       "missing or where the drivetrain broke.",
        "parameters": {"type": "object", "properties": {
            "regex": {"type": "string"},
            "max_matches": {"type": "integer"}},
            "required": ["regex"], "additionalProperties": False}}},
    {"type": "function", "function": {
        "name": "view_frame",
        "description": "Re-render a specific keyframe index (0 = first) back to you as an "
                       "image, to look closer at one moment in the recording.",
        "parameters": {"type": "object", "properties": {
            "index": {"type": "integer"}},
            "required": ["index"], "additionalProperties": False}}},
    {"type": "function", "function": {
        "name": "read_joint_travel",
        "description": "Recorded rotation of the sim's joints. With no argument: net and "
                       "total travel for every joint. With `name`: that joint's angle "
                       "sampled over time, to see WHEN it stopped or whether it tracks "
                       "another joint exactly.",
        "parameters": {"type": "object", "additionalProperties": False,
                       "properties": {"name": {"type": "string",
                                               "description": "joint name, or empty for all"}},
                       "required": ["name"]}}},
    {"type": "function", "function": {
        "name": "read_part_mounts",
        "description": "Each part's dof and the part it is mounted on, from the authored "
                       "script. Use it to find which TURNING part an output part (a hand, "
                       "a jaw, an output flange) actually rides — that is the joint whose "
                       "motion the user sees.",
        "parameters": {"type": "object", "additionalProperties": False,
                       "properties": {"pattern": {"type": "string",
                                                  "description": "substring filter on the "
                                                                 "part name, or empty for all"}},
                       "required": ["pattern"]}}},
    {"type": "function", "function": {
        "name": "read_contacts",
        "description": "The MOST RELIABLE evidence for a structure fault: the simulator's "
                       "OWN collision verdict at the design pose. Returns each "
                       "interpenetrating body pair with depth_mm (how deep they overlap), "
                       "and r1_mm/r2_mm (radial distance of the contact point to each "
                       "body's axis). ALSO returns precheck_overlaps = the same run's "
                       "REAL-geometry overlap check. COMPARE them: a pair MuJoCo reports "
                       "but precheck does NOT is a convex-hull artifact (a decomposed "
                       "piece filled a bore, or an axial end-face pokes a coaxial part) — "
                       "not a real design fault; a pair BOTH report is a real overlap "
                       "(bore too small, wrong coordinates). Interpreting r: a deep "
                       "contact at small r (~0-1mm, bodies coaxial) = an AXIAL end-face "
                       "poking through (part too long / wrong z), NOT radial; a contact "
                       "near a bore-wall radius = radial (bore too small to fit the "
                       "shaft); a contact out at a gear's tooth radius with shallow depth "
                       "= NORMAL meshing, do not flag it.",
        "parameters": {"type": "object", "properties": {},
                       "additionalProperties": False}}},
]


def _diag_executors(run_dir: str, frames_dir: str, metrics: dict):
    """Sandboxed read-only executors for the diagnosis tool loop, bound to this run."""
    def _dig(obj, pointer):
        for key in (pointer or "").split("."):
            if not key:
                continue
            if isinstance(obj, dict):
                obj = obj.get(key)
            elif isinstance(obj, list):
                try:
                    obj = obj[int(key)]
                except (ValueError, IndexError):
                    return None
            else:
                return None
        return obj

    def read_sim_result(pointer=""):
        p = os.path.join(run_dir, "sim_result.json") if run_dir else ""
        data = None
        if p and os.path.exists(p):
            try:
                data = json.loads(Path(p).read_text(encoding="utf-8"))
            except Exception:
                data = None
        if data is None:
            data = {"metrics": metrics or {}}       # fall back to what we were handed
        sub = _dig(data, pointer) if pointer else data
        return json.dumps(sub, ensure_ascii=False)[:6000]

    def read_run_log(regex, max_matches=20):
        p = os.path.join(run_dir, "run.log") if run_dir else ""
        if not p or not os.path.exists(p):
            return "(no run.log for this run)"
        try:
            rx = re.compile(regex, re.I)
        except re.error as e:
            return f"(bad regex: {e})"
        hits = []
        for ln in Path(p).read_text(encoding="utf-8", errors="replace").splitlines():
            if rx.search(ln):
                hits.append(ln[:300])
                if len(hits) >= int(max_matches or 20):
                    break
        return "\n".join(hits) if hits else "(no matches)"

    def view_frame(index):
        frames = sorted(glob.glob(os.path.join(frames_dir, "rgb_*.png"))) if frames_dir else []
        if not frames:
            return {"error": "no frames"}
        i = max(0, min(int(index), len(frames) - 1))
        return {"__image__": frames[i], "index": i, "total": len(frames)}

    def read_joint_travel(name=""):
        """Per-sample angles from the recorded trajectory. The prompt carries only totals,
        which cannot answer WHEN a joint stopped or whether two joints move in lockstep —
        both of which name the fault."""
        p = os.path.join(run_dir, "trajectory.json") if run_dir else ""
        if not p or not os.path.exists(p):
            return "(no trajectory recorded for this run)"
        try:
            doc = json.loads(Path(p).read_text(encoding="utf-8"))
        except Exception as e:
            return f"(trajectory unreadable: {e})"
        joints = doc.get("joints") or {}
        if not joints:
            return "(trajectory has no joints)"
        if name and name in joints:
            seq = joints[name]
            step = max(1, len(seq) // 24)
            return json.dumps({"joint": name, "t": (doc.get("t") or [])[::step],
                               "angle_rad": seq[::step]})[:4000]
        rows = {}
        for n, seq in joints.items():
            if not seq:
                continue
            rows[n] = {
                "net": round(abs(float(seq[-1]) - float(seq[0])), 5),
                "total": round(sum(abs(float(b) - float(a))
                                   for a, b in zip(seq, seq[1:])), 5)}
        return json.dumps(rows)[:4000]

    def read_part_mounts(pattern=""):
        """Which part each part is MOUNTED on, from the authored machine script. A hand,
        a jaw, an output flange is usually dof=fixed and carries no joint of its own, so
        the only way to know which turning part it rides — and therefore which joint's
        motion the user actually sees — is this label."""
        # run_dir here is the PHYSICS output dir (…/physics/mujoco_N); the authored
        # script lives at the run root, two levels up. Look in both.
        p = ""
        roots = []
        if run_dir:
            roots = [run_dir, os.path.dirname(os.path.dirname(run_dir)),
                     os.path.dirname(run_dir)]
        for root in roots:
            for cand in ("machine.py", "manager_sub.py"):
                q = os.path.join(root, cand)
                if os.path.exists(q):
                    p = q
                    break
            if p:
                break
        if not p or not os.path.exists(p):
            return "(no authored script for this run)"
        try:
            txt = Path(p).read_text(encoding="utf-8", errors="replace")
        except Exception as e:
            return f"(script unreadable: {e})"
        rows = []
        for m2 in re.finditer(r'"([A-Za-z0-9_]+)\|([^"]*)"', txt):
            label, meta = m2.group(1), m2.group(2)
            if pattern and pattern.lower() not in label.lower():
                continue
            mount = ""
            dof = ""
            for field in meta.split("|"):
                if field.startswith("mount="):
                    mount = field[6:]
                elif field.startswith("dof="):
                    dof = field[4:]
            rows.append(f"{label}: dof={dof or '?'} mount={mount or '(none)'}")
        return "\n".join(rows[:60]) if rows else "(no labelled parts found)"

    def read_contacts():
        # MuJoCo's own collision verdict, dumped by the runner at the design pose.
        mj_pairs = []
        cp = os.path.join(run_dir, "contacts.json") if run_dir else ""
        if cp and os.path.exists(cp):
            try:
                mj_pairs = (json.loads(Path(cp).read_text(encoding="utf-8"))
                            or {}).get("contacts", [])
            except Exception:
                mj_pairs = []
        # Load the model so we KNOW which contact pairs are MESHING GEARS (mesh_pairs) vs
        # coaxial stacks — this is what tells a "gears too close, open the center distance"
        # fault apart from an "axial poke, coaxial" one. Without it a big gear eating a
        # small one (contact point deep inside the big gear, so r looks small) gets
        # mis-read as a coaxial poke.
        mesh_pairs = set()
        poses = {}
        precheck = []
        try:
            import sys
            root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            if root not in sys.path:
                sys.path.insert(0, root)
            from maker2.manager import load_model
            from maker2.subcheck import sub_conflicts
            mp = os.path.join(run_dir, "kinematic_model.json")
            urdf = os.path.join(run_dir, "model.urdf")
            if os.path.exists(mp):
                model = load_model(mp)
                for pr in (getattr(model, "mesh_pairs", None) or []):
                    if len(pr) >= 2:
                        mesh_pairs.add(frozenset((pr[0], pr[1])))
                for ps in (getattr(model, "poses", None) or []):
                    xyz = getattr(ps, "xyz_m", None)
                    child = getattr(ps, "child", None)
                    if child and xyz is not None:
                        poses[child] = [float(v) * 1000.0 for v in xyz]  # mm
                if os.path.exists(urdf):
                    for c in sub_conflicts(model, urdf, log_fn=lambda *_: None):
                        precheck.append(c.describe())
        except Exception as e:
            precheck = [f"(precheck/model unavailable: {type(e).__name__})"]

        # Annotate each contact: is it a declared MESH pair? If so, meshing overlap ==
        # center distance too small -> the fix is to SPREAD the gear centers apart, NEVER
        # to open a bore. Also give the coaxial vs non-coaxial hint from the poses.
        import math
        for c in mj_pairs:
            b1, b2 = c.get("body1"), c.get("body2")
            is_mesh = frozenset((b1, b2)) in mesh_pairs
            c["is_mesh_pair"] = is_mesh
            p1, p2 = poses.get(b1), poses.get(b2)
            if p1 and p2:
                dxy = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                c["center_dist_xy_mm"] = round(dxy, 2)
                c["coaxial"] = dxy < 0.5
            if is_mesh:
                c["fix"] = ("MESHING GEARS overlapping -> their center distance is TOO "
                            "SMALL; move the two gear centers APART (center distance = "
                            "module*(z1+z2)/2). Do NOT open a bore, they are NOT coaxial.")

        return json.dumps({"mujoco_contacts": mj_pairs,
                           "precheck_real_overlaps": precheck,
                           "note": "is_mesh_pair=true -> the two are DECLARED meshing "
                                   "gears; any overlap means center distance too small, "
                                   "fix = SPREAD them apart, never a bore. is_mesh_pair="
                                   "false + coaxial=true + small r -> axial poke (shorten "
                                   "shaft/clear bore). A pair in mujoco_contacts but not in "
                                   "precheck_real_overlaps is a hull artifact."},
                          ensure_ascii=False)[:6000]

    return {"read_sim_result": read_sim_result,
            "read_joint_travel": read_joint_travel,
            "read_part_mounts": read_part_mounts, "read_run_log": read_run_log,
            "view_frame": view_frame, "read_contacts": read_contacts}


def diagnose_physics(task, robot_info, spec, metrics, frames_dir, *,
                     stability=None, frames_dirs=None, base_url=None, api_key=None,
                     model=None) -> dict:
    """VLM verdict over the sim recording. Returns {verdict, cause, reason}.

    `frames_dirs` (optional) maps camera_name -> frames dir; when given, a few
    keyframes from EACH camera are sent in ONE message (labeled per camera) so the VLM
    judges from multiple angles. Falls back to the single `frames_dir` otherwise.

    Degrades safely: if there are no frames or the call fails, fall back to the raw
    metric verdict with cause based on the numbers, so the loop still progresses."""
    # Build the (camera_label, frame_path) list: multi-view if frames_dirs given.
    view_frames: list[tuple[str, str]] = []
    if frames_dirs:
        # Cap total images so a 3-cam run stays within a sane payload (~5 per cam).
        per = max(3, 12 // max(1, len(frames_dirs)))
        for cam, fd in frames_dirs.items():
            for fp in _sample_frames(fd, per):
                view_frames.append((cam, fp))
    else:
        for fp in _sample_frames(frames_dir, 10):
            view_frames.append(("cam", fp))

    raw_pass = (metrics or {}).get("verdict") == "PASS"
    sig = _metric_signals(spec, metrics)
    # Stage-1 stability + barely-turned are hard, deterministic FAILs the VLM cannot
    # override: a machine that explodes just settling, or an input that never really
    # turned, cannot be a PASS regardless of what the keyframes look like.
    _st = stability or {}
    stability_failed = (str(_st.get("verdict", "PASS")).upper() == "FAIL"
                        or bool(_st.get("exploded")))
    _it = float((metrics or {}).get("input_travel") or 0.0)
    _exp_in = float(sig.get("expected_input_travel") or 0.0)
    barely_turned = bool(_it < max(0.3, 0.25 * _exp_in))
    hard_fail = (sig["input_stalled"] or sig["free_spin"] or sig["output_dead"]
                 or sig["blew_apart"] or stability_failed or barely_turned)
    sig_note = " ".join(sig["notes"])
    # Deterministic culprit from the metrics (independent of the VLM): the break joint.
    det_part = sig.get("break_joint") or ""
    if not det_part and sig["input_stalled"]:
        det_part = (metrics or {}).get("input_joint") or (spec.get("drive") or {}).get("input_joint") or ""
    if not det_part and sig.get("exploded_parts"):
        det_part = str(sig["exploded_parts"][0].get("part") or "")
    det_sub = _culprit_sub_of(det_part, robot_info)

    if not view_frames:
        # No video to look at: trust the hard signals, else the metric verdict.
        if hard_fail:
            return _diag("fail", "structure",
                         sig_note or "input jammed / parts loose (no frames)",
                         det_part, det_sub)
        return _diag("pass" if raw_pass else "fail",
                     "none" if raw_pass else "structure",
                     "no frames to inspect; used metric verdict",
                     "" if raw_pass else det_part, "" if raw_pass else det_sub)

    drive = (spec or {}).get("drive") or {}
    st = stability or {}
    it_val = _it
    stability_txt = (
        f"STAGE 1 — STABILITY (settle under gravity, NO drive; a machine must first hold "
        f"together on the bench before any function test is meaningful):\n"
        f"  STABILITY_VERDICT: {st.get('verdict', '(none)')}\n"
        f"  STABILITY_EXPLODED (flew apart just settling): {st.get('exploded')}\n"
        f"  settle_max_disp_m: {st.get('max_disp_m')}\n"
        f"  settle_displaced_parts: {st.get('displaced_parts')}\n")
    signals_txt = (
        stability_txt +
        f"\nSTAGE 2 — FUNCTION (drive the input, only meaningful if STAGE 1 passed):\n"
        f"METRIC SIGNALS (authoritative):\n"
        f"  INPUT_MOVED: {sig['input_moved']}\n"
        f"  INPUT_STALLED: {sig['input_stalled']}\n"
        f"  BARELY_TURNED (input swept < ~0.3 rad -> downstream 'motion' is settle jitter, "
        f"NOT proven transmission): {barely_turned}\n"
        f"  FREE_SPIN: {sig['free_spin']}\n"
        f"  OUTPUT_REACHED: {sig['output_reached']}\n"
        f"  OUTPUT_DEAD (moved but output not reached): {sig['output_dead']}\n"
        f"  BLEW_APART (parts ejected off their settle position): {sig['blew_apart']}\n"
        f"  BREAK_JOINT (first dead joint in the path): {sig.get('break_joint') or '(none)'}\n"
        f"  expected_input_travel_rad: {sig['expected_input_travel']}\n"
        f"  actual_input_travel_rad: {round(it_val, 4)}\n"
        + ("  " + sig_note + "\n" if sig_note else ""))
    # PER-JOINT TRAVEL. "5 of 6 joints moved" names no part, so a diagnosis built on it
    # can only say something moved and something did not — which is exactly the useless
    # verdict this produced ("no specific fault was isolated") while the trajectory on
    # disk knew precisely which joint was dead. Rank every joint by how far it actually
    # turned, mark the driver, and hand that over with the frames.
    joints_txt = _per_joint_travel_txt(frames_dir, drive)
    ncam = len({c for c, _ in view_frames})
    content = [{"type": "text", "text": (
        f"TASK: {task}\n"
        f"DRIVEN INPUT JOINT: {drive.get('input_joint')}\n"
        f"OUTPUT JOINT (motion must reach): {drive.get('output_joint')}\n"
        f"WATCHED JOINTS: {drive.get('watch_joints')}\n"
        f"METRICS: {json.dumps(metrics)}\n\n"
        f"{signals_txt}\n"
        f"You are shown keyframes from {ncam} camera view(s), each labeled. Judge whether "
        f"driving the input made the mechanism work AND motion reached the output, and "
        f"classify the cause if it failed. Obey the HARD RULES about the signals above.")}]
    last_cam = None
    idx = 0
    for cam, fp in view_frames:
        if cam != last_cam:
            content.append({"type": "text", "text": f"--- camera {cam} ---"})
            last_cam = cam
            idx = 0
        idx += 1
        content.append({"type": "text", "text": f"{cam} frame {idx}:"})
        content.append({"type": "image_url",
                        "image_url": {"url": f"data:image/png;base64,{_b64(fp)}"}})

    # Investigation loop: the VLM sees the keyframes + signals, then may call the
    # read-only tools (full sim_result, run.log, re-view a frame) before it commits to a
    # verdict. This lets it reason about an empty-looking video from the hard numbers.
    run_dir = str(Path(frames_dir).parent) if frames_dir else ""
    execs = _diag_executors(run_dir, frames_dir, metrics)
    # Method notes: how to turn per-joint numbers into a named culprit. The diagnoser had
    # no KB at all, so its guidance was whatever fitted in the system prompt — and it kept
    # returning "no specific fault was isolated" while the measurements on disk named the
    # dead joint exactly.
    system = _DIAG_SYSTEM + _DIAG_TOOL_HINT
    notes = _kb_notes(task)
    if notes:
        system += ("\n\nMETHOD NOTES (from the local knowledge base — how to localise a "
                   f"partial failure):\n{notes}")
    messages = [{"role": "system", "content": system},
                {"role": "user", "content": content}]
    try:
        c = _client(base_url, api_key)
        mdl = model or os.environ.get("AZURE_VLM_DEPLOYMENT", "claude-opus-4.8")
        d = {}
        for _round in range(6):
            r = c.chat.completions.create(
                model=mdl, messages=messages, tools=_DIAG_TOOLS,
                max_completion_tokens=16000)
            msg = r.choices[0].message
            calls = getattr(msg, "tool_calls", None) or []
            if not calls:
                d = _parse_json(msg.content)
                break
            messages.append({"role": "assistant", "content": msg.content or "",
                             "tool_calls": [{"id": tc.id, "type": "function",
                                 "function": {"name": tc.function.name,
                                              "arguments": tc.function.arguments}}
                                            for tc in calls]})
            for tc in calls:
                fn = execs.get(tc.function.name)
                try:
                    a = json.loads(tc.function.arguments or "{}")
                except Exception:
                    a = {}
                out = fn(**a) if fn else f"(no such tool: {tc.function.name})"
                # A tool returning an image is fed back as an image content block so the
                # model can actually look at the requested frame.
                if isinstance(out, dict) and out.get("__image__"):
                    messages.append({"role": "tool", "tool_call_id": tc.id,
                                     "content": f"frame {out.get('index')}/"
                                                f"{out.get('total')} attached below"})
                    messages.append({"role": "user", "content": [
                        {"type": "text", "text": f"requested frame {out.get('index')}:"},
                        {"type": "image_url", "image_url": {
                            "url": f"data:image/png;base64,{_b64(out['__image__'])}"}}]})
                else:
                    messages.append({"role": "tool", "tool_call_id": tc.id,
                                     "content": out if isinstance(out, str) else json.dumps(out)})
        else:
            # Ran out of rounds without a final answer — force one plain JSON reply.
            messages.append({"role": "user", "content":
                "Stop investigating and answer NOW with ONLY the JSON verdict."})
            r = c.chat.completions.create(model=mdl, messages=messages,
                                          response_format=_DIAG_SCHEMA,
                                          max_completion_tokens=16000)
            d = _parse_json(r.choices[0].message.content)
    except Exception as e:
        if hard_fail:
            return _diag("fail", "structure",
                         sig_note or f"diagnosis call failed ({e})", det_part, det_sub)
        return _diag("pass" if raw_pass else "fail",
                     "none" if raw_pass else "structure",
                     f"diagnosis call failed ({e}); used metric verdict",
                     "" if raw_pass else det_part, "" if raw_pass else det_sub)

    verdict = _pick(d, ("verdict", "result", "pass_fail", "status"),
                    {"pass", "fail"}, "pass" if raw_pass else "fail")
    cause = _pick(d, ("cause", "category", "fault", "reason_category"),
                  {"none", "structure", "interface", "scenario", "camera"},
                  "none" if verdict == "pass" else "structure")
    reason = (d.get("reason") or d.get("explanation") or d.get("detail") or "")
    reason = str(reason).strip()
    culprit_part = str(d.get("culprit_part") or "").strip()
    culprit_sub = str(d.get("culprit_sub") or "").strip()

    # Enforce the hard metric rules AFTER the model — a stalled input, a free-spinning
    # output, or motion that never reaches the declared output joint are physical facts
    # the VLM cannot override, and none of them is a camera issue.
    if hard_fail:
        verdict = "fail"
        if cause not in ("structure", "scenario", "interface"):
            cause = "structure"
        if not reason:
            reason = sig_note
        # The metrics KNOW the exact break point — trust it over the VLM's guess.
        if det_part:
            culprit_part = det_part
    elif cause == "camera" and not sig["input_moved"]:
        # It asked to reframe, but the input never moved -> reframing won't help; the
        # mechanism or the test is at fault, not the camera.
        cause = "structure"
        reason = (reason + " (input did not move; not a camera issue)").strip()
        if det_part:
            culprit_part = det_part

    # A FAIL MUST SAY WHY. The backfill above only fires on `hard_fail`, so a run where
    # every hard signal passed (the input turned, motion reached the output) but the VLM
    # still called it a fail came back with an EMPTY reason — the agent was told "FAIL,
    # cause=structure" and nothing else, while the measured cause was sitting in the
    # metrics all along (four posts driven through two wheels, 11.4mm^3 each).
    if verdict == "fail" and not reason:
        reason = _measured_reason(metrics) or sig_note or (
            "the mechanism did not work, but no specific fault was isolated")

    # Backfill culprit_sub from the part namespace if the model left it blank.
    if culprit_part and not culprit_sub:
        culprit_sub = _culprit_sub_of(culprit_part, robot_info)
    return _diag(verdict, cause, reason, culprit_part, culprit_sub)
