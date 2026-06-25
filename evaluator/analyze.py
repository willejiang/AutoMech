#!/usr/bin/env python3
"""
VLM pass/fail analysis — runs OUTSIDE the Isaac container (host venv).

Takes the sim output (frames + metrics) and asks the Azure VLM to judge whether
the design passed the user's task. Azure has no native-video model, so we sample
keyframes and send them as a multi-image message (the standard pattern).

Output: result.json with per-failure {related frames + feedback} for the maker loop.

Reads endpoint + key from environment (loaded from /data/physcad/.env, chmod 600):
  AZURE_OPENAI_ENDPOINT   e.g. https://hackathondefault.openai.azure.com/openai/v1
  AZURE_OPENAI_API_KEY
  AZURE_VLM_DEPLOYMENT    e.g. gpt-4.1  (deployment name)
"""
import argparse
import base64
import json
import os
import sys
from pathlib import Path


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim-result", required=True, help="sim_result.json from run_eval.py")
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--out", required=True, help="result.json for the maker handoff")
    ap.add_argument("--max-frames", type=int, default=12)
    return ap.parse_args()


def sample_frames(frames_dir, k):
    fd = Path(frames_dir)
    # the sim writes a CONTAINER path (/work/...) into sim_result.json, but analyze.py
    # runs on the HOST where that mount is /data/physcad/... — remap if needed.
    if not fd.exists() and str(fd).startswith("/work/"):
        fd = Path(str(fd).replace("/work/", "/data/physcad/", 1))
    # last resort: if sim_result.json sits next to a frames/ dir, use that
    if not fd.exists():
        alt = Path(frames_dir)
        for cand in (alt, alt.parent / "frames"):
            if cand.exists():
                fd = cand; break
    frames = sorted(fd.glob("rgb_*.png")) or sorted(fd.glob("*.png"))
    if not frames:
        return []
    if len(frames) <= k:
        return frames
    step = len(frames) / k
    return [frames[int(i * step)] for i in range(k)]


def b64_image(path):
    return base64.b64encode(Path(path).read_bytes()).decode()


SYSTEM = """You are a mechanical-engineering simulation judge. You are given keyframes
from a physics simulation of a CAD design under a specific task, plus measured metrics.
Decide if the design PASSES the user's task. Be strict and concrete: name the physical
failure mode (bottoming out, rollover, parts detaching, insufficient travel, instability)
and which frames show it. Respond ONLY with the JSON schema requested."""


def build_messages(manifest, metrics, frames):
    content = [{
        "type": "text",
        "text": (
            f"USER TASK: {manifest.get('user_prompt','')}\n\n"
            f"PASS CRITERIA: {json.dumps(manifest.get('pass_criteria', {}))}\n\n"
            f"MEASURED METRICS: {json.dumps(metrics)}\n\n"
            f"Frames are in time order (0 = start). Judge pass/fail and, for each failure, "
            f"list the 1-based frame indices that show it and a fix hint for the design generator."
        ),
    }]
    for idx, fp in enumerate(frames, 1):
        content.append({"type": "text", "text": f"frame {idx}:"})
        content.append({
            "type": "image_url",
            "image_url": {"url": f"data:image/png;base64,{b64_image(fp)}"},
        })
    return [
        {"role": "system", "content": SYSTEM},
        {"role": "user", "content": content},
    ]


RESULT_SCHEMA = {
    "type": "json_schema",
    "json_schema": {
        "name": "evaluation",
        "strict": True,
        "schema": {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "passed": {"type": "boolean"},
                "summary": {"type": "string"},
                "failures": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "additionalProperties": False,
                        "properties": {
                            "failure_mode": {"type": "string"},
                            "explanation": {"type": "string"},
                            "frame_indices": {"type": "array", "items": {"type": "integer"}},
                            "fix_hint": {"type": "string"},
                        },
                        "required": ["failure_mode", "explanation", "frame_indices", "fix_hint"],
                    },
                },
            },
            "required": ["passed", "summary", "failures"],
        },
    },
}


def call_vlm(messages):
    from openai import OpenAI
    endpoint = os.environ["AZURE_OPENAI_ENDPOINT"].rstrip("/")
    client = OpenAI(
        base_url=endpoint,
        api_key=os.environ["AZURE_OPENAI_API_KEY"],
    )
    resp = client.chat.completions.create(
        model=os.environ.get("AZURE_VLM_DEPLOYMENT", "gpt-5.4"),
        messages=messages,
        response_format=RESULT_SCHEMA,
        max_completion_tokens=2000,
    )
    return json.loads(resp.choices[0].message.content)


def main():
    args = parse_args()
    sim = json.loads(Path(args.sim_result).read_text())
    manifest = json.loads(Path(args.manifest).read_text())
    metrics = sim["metrics"]
    frames = sample_frames(sim["frames_dir"], args.max_frames)

    if not frames:
        print("[analyze] WARNING: no frames captured; falling back to metrics-only verdict")
        verdict = {
            "passed": metrics.get("passed_metrics", False),
            "summary": "metrics-only (no frames)",
            "failures": [
                {"failure_mode": f["criterion"], "explanation": f["detail"],
                 "frame_indices": [], "fix_hint": ""}
                for f in metrics.get("failures", [])
            ],
        }
    else:
        verdict = call_vlm(build_messages(manifest, metrics, frames))

    # attach the actual frame file paths for each failure (the maker handoff)
    frame_paths = [str(f) for f in frames]
    for fail in verdict.get("failures", []):
        fail["frame_files"] = [
            frame_paths[i - 1] for i in fail.get("frame_indices", [])
            if 1 <= i <= len(frame_paths)
        ]

    result = {
        "design_id": manifest.get("design_id"),
        "user_prompt": manifest.get("user_prompt"),
        "passed": verdict["passed"],
        "summary": verdict["summary"],
        "failures": verdict["failures"],
        "metrics": metrics,
        "all_frames": frame_paths,
    }
    Path(args.out).write_text(json.dumps(result, indent=2))
    print(f"[analyze] passed={result['passed']} failures={len(result['failures'])} -> {args.out}")


if __name__ == "__main__":
    main()
