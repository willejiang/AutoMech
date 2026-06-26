#!/usr/bin/env python3
"""Before/after RL judge — compares untrained (model_0) vs trained (model_500)
keyframes of the maker-generated dog and asks the local opus-4.8 proxy whether
RL produced a real behavioral improvement. Runs on the laptop (proxy is local)."""
import base64, json, os, sys
from pathlib import Path
from openai import OpenAI

BASE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "physcad_videos" / "cmp_frames"
ENDPOINT = os.environ.get("AZURE_OPENAI_ENDPOINT", "http://localhost:8313/v1").rstrip("/")
MODEL = os.environ.get("AZURE_VLM_DEPLOYMENT", "claude-opus-4.8")
KEY = os.environ.get("AZURE_OPENAI_API_KEY", "not-needed-for-local-proxy")

def imgs(d):
    fs = sorted(Path(d).glob("*.png"))
    return [f for f in fs if f.stat().st_size > 5000]  # drop blank intro frames

def b64(p): return base64.b64encode(Path(p).read_bytes()).decode()

SYSTEM =("You judge RL locomotion progress for a 12-DOF quadruped robot. You see BEFORE "
          "frames (policy at iteration 0, untrained) then AFTER frames (iteration 500). "
          "Judge whether training produced a real behavioral improvement: more upright posture, "
          "controlled/coordinated leg motion, staying stable vs collapsing or flailing. Be honest "
          "and concrete — if it is only partial progress (stabilized but not yet a clean walking "
          "gait) say so. Respond ONLY with the requested JSON.")

content = [{"type": "text", "text": "BEFORE (untrained, iteration 0):"}]
for p in imgs(BASE / "before"):
    content.append({"type": "image_url", "image_url": {"url": f"data:image/png;base64,{b64(p)}"}})
content.append({"type": "text", "text": "AFTER (trained, iteration 500):"})
for p in imgs(BASE / "after"):
    content.append({"type": "image_url", "image_url": {"url": f"data:image/png;base64,{b64(p)}"}})
content.append({"type": "text", "text": "Did RL improve the dog? Compare before vs after. "
                "Reply ONLY with JSON keys: improved(bool), before_behavior(str), "
                "after_behavior(str), verdict(str), still_missing(str)."})

def parse(text):
    s = text.strip()
    if s.startswith("```"):
        s = s.split("\n", 1)[1].rsplit("```", 1)[0]  # strip ```json fence
    return json.loads(s)

client = OpenAI(base_url=ENDPOINT, api_key=KEY)
# the local Copilot proxy ignores response_format json_schema and fences the JSON,
# so we ask for JSON in-prompt and strip the fence instead. Keep the JSON instruction
# in the SAME user turn as the images — a trailing text-only turn makes the model
# lose the image context and claim no frames were sent.
resp = client.chat.completions.create(
    model=MODEL,
    messages=[{"role": "system", "content": SYSTEM},
              {"role": "user", "content": content}],
    max_completion_tokens=1200)
v = parse(resp.choices[0].message.content)
print(json.dumps(v, indent=2, ensure_ascii=False))
