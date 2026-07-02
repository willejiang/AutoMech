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

If it FAILS, classify the CAUSE precisely:
- "structure": the MODEL is at fault — gears/teeth don't mesh, parts interpenetrate or
  fly apart, a needed part is missing or the wrong size, the drivetrain is not actually
  connected, OR the driven input joint is JAMMED and cannot rotate. Fixing this needs the
  CAD to be redesigned.
- "scenario": the TEST is at fault — the wrong joint was driven, the velocity/params are
  absurd, the watched joints are wrong, so the test doesn't fairly exercise the machine.
  The model may be fine; the TEST should be redesigned.
- "framing": you CANNOT SEE the mechanism clearly enough to judge — it's too small/far,
  off-frame, or occluded. The model and test may be fine; the CAMERA must be fixed.

HARD RULES (these override your visual impression):
1. If the signal INPUT_STALLED is true, the driven joint did NOT turn even though it was
   commanded — the drivetrain is physically jammed. This is NEVER "framing". Return
   verdict=fail with cause "structure" (the input is blocked) unless the driven joint is
   clearly the WRONG joint to drive, in which case "scenario".
2. If the signal FREE_SPIN is true, a downstream joint spun FAR faster than the input
   command could ever produce — it is flying loose / not actually meshed, not
   transmitting. This is NEVER "framing"; it is "structure" (parts not connected).
3. Only choose "framing" when the input DID move a reasonable amount (INPUT_MOVED true)
   yet you still cannot see the resulting downstream motion. If the input didn't move,
   the problem is the mechanism or the test, not the camera.
4. If the signal OUTPUT_DEAD is true, the input turned and some gears moved but motion
   NEVER REACHED the declared output joint — the drivetrain is broken MIDWAY (a missing
   or non-meshing gear between the moving part and the output). This is NEVER "framing";
   return verdict=fail with cause "structure".

Always fill "reason" with a concrete one-sentence explanation. Respond ONLY with the JSON
schema."""

_DIAG_SCHEMA = {
    "type": "json_schema",
    "json_schema": {
        "name": "physics_diagnosis", "strict": True,
        "schema": {
            "type": "object", "additionalProperties": False,
            "properties": {
                "verdict": {"type": "string", "enum": ["pass", "fail"]},
                "cause": {"type": "string",
                          "enum": ["none", "structure", "scenario", "framing"]},
                "reason": {"type": "string"},
            },
            "required": ["verdict", "cause", "reason"],
        },
    },
}


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

    notes = []
    if input_stalled:
        notes.append(f"INPUT_STALLED: input joint '{m.get('input_joint')}' moved only "
                     f"{it:.3f} rad but was commanded to sweep ~{expected:.2f} rad "
                     f"-> the drivetrain is jammed (structure).")
    if free_spin:
        notes.append(f"FREE_SPIN: a watched joint moved {max_watched:.1f} rad, far "
                     f"beyond the ~{expected:.2f} rad input command, while the input "
                     f"moved {it:.3f} rad -> parts flying loose / not meshed (structure).")
    if output_dead:
        notes.append(f"OUTPUT_NOT_REACHED: the input turned and some gears moved, but "
                     f"the output joint '{m.get('output_joint')}' did NOT move "
                     f"(travel {m.get('output_travel')}) -> the train is broken midway "
                     f"(structure).")
    return {"input_moved": input_moved, "input_stalled": input_stalled,
            "free_spin": free_spin, "output_dead": output_dead,
            "output_reached": bool(output_reached) if output_reached is not None else None,
            "expected_input_travel": round(expected, 3), "notes": notes}


def diagnose_physics(task, robot_info, spec, metrics, frames_dir, *,
                     frames_dirs=None, base_url=None, api_key=None, model=None) -> dict:
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
    hard_fail = sig["input_stalled"] or sig["free_spin"] or sig["output_dead"]
    sig_note = " ".join(sig["notes"])

    if not view_frames:
        # No video to look at: trust the hard signals, else the metric verdict.
        if hard_fail:
            return {"verdict": "fail", "cause": "structure",
                    "reason": sig_note or "input jammed / parts loose (no frames)"}
        return {"verdict": "pass" if raw_pass else "fail",
                "cause": "none" if raw_pass else "structure",
                "reason": "no frames to inspect; used metric verdict"}

    drive = (spec or {}).get("drive") or {}
    signals_txt = (
        f"METRIC SIGNALS (authoritative):\n"
        f"  INPUT_MOVED: {sig['input_moved']}\n"
        f"  INPUT_STALLED: {sig['input_stalled']}\n"
        f"  FREE_SPIN: {sig['free_spin']}\n"
        f"  OUTPUT_REACHED: {sig['output_reached']}\n"
        f"  OUTPUT_DEAD (moved but output not reached): {sig['output_dead']}\n"
        f"  expected_input_travel_rad: {sig['expected_input_travel']}\n"
        + ("  " + sig_note + "\n" if sig_note else ""))
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

    try:
        c = _client(base_url, api_key)
        r = c.chat.completions.create(
            model=model or os.environ.get("AZURE_VLM_DEPLOYMENT", "claude-opus-4.8"),
            messages=[{"role": "system", "content": _DIAG_SYSTEM},
                      {"role": "user", "content": content}],
            response_format=_DIAG_SCHEMA, max_completion_tokens=600)
        d = _parse_json(r.choices[0].message.content)
    except Exception as e:
        if hard_fail:
            return {"verdict": "fail", "cause": "structure",
                    "reason": sig_note or f"diagnosis call failed ({e})"}
        return {"verdict": "pass" if raw_pass else "fail",
                "cause": "none" if raw_pass else "structure",
                "reason": f"diagnosis call failed ({e}); used metric verdict"}

    verdict = _pick(d, ("verdict", "result", "pass_fail", "status"),
                    {"pass", "fail"}, "pass" if raw_pass else "fail")
    cause = _pick(d, ("cause", "category", "fault", "reason_category"),
                  {"none", "structure", "scenario", "framing"},
                  "none" if verdict == "pass" else "structure")
    reason = (d.get("reason") or d.get("explanation") or d.get("detail") or "")
    reason = str(reason).strip()

    # Enforce the hard metric rules AFTER the model — a stalled input, a free-spinning
    # output, or motion that never reaches the declared output joint are physical facts
    # the VLM cannot override, and none of them is a camera issue.
    if hard_fail:
        verdict = "fail"
        if cause not in ("structure", "scenario"):
            cause = "structure"
        if not reason:
            reason = sig_note
    elif cause == "framing" and not sig["input_moved"]:
        # It asked to reframe, but the input never moved -> reframing won't help; the
        # mechanism or the test is at fault, not the camera.
        cause = "structure"
        reason = (reason + " (input did not move; not a framing issue)").strip()

    return {"verdict": verdict, "cause": cause, "reason": reason[:400]}
