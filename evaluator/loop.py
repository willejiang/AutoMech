#!/usr/bin/env python3
"""
Iteration loop orchestrator (runs on HOST). Ties the task-oriented evaluator together:

  1. extract robot joint/link names from the URDF
  2. scenario_designer.design(task, robot)         -> spec_0
  3. run_scenario.py in container with spec_N       -> frames + metrics
  4. analyze.py (VLM) critiques frames + metrics    -> verdict + feedback
  5. if PASS -> done; else scenario_designer.revise(..., feedback) -> spec_{N+1}; back to 3
  6. stop at PASS or max iterations

Usage (host, .env sourced):
  python3 loop.py --urdf .../robot.urdf --asset-root ... \
     --task "make Cassie do a handstand" --workdir /data/physcad/loop_cassie --max-iters 4
"""
import argparse, json, os, re, subprocess, sys
from pathlib import Path
import xml.etree.ElementTree as ET

import scenario_designer
import analyze


def robot_info_from_urdf(urdf_path, name="robot"):
    root = ET.parse(urdf_path).getroot()
    joints = [j.get("name") for j in root.findall("joint")
              if j.get("type") in ("revolute", "prismatic", "continuous")]
    links = [l.get("name") for l in root.findall("link")]
    return {"name": name, "joints": joints, "links": links}


def run_in_container(urdf, asset_root, spec_path, task, out_dir):
    """Run run_scenario.py inside the isaac-sim container (detached not needed; we block)."""
    img = "nvcr.io/nvidia/isaac-sim:6.0.1"
    cname = "scn-" + Path(out_dir).name
    subprocess.run(["bash", "-c", f"echo zjjiang | sudo -S -p '' docker rm -f {cname} 2>/dev/null"],
                   capture_output=True)
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    os.system(f"chmod -R 777 {out_dir} 2>/dev/null")
    # map host /data/physcad -> container /work
    def c(p): return str(p).replace("/data/physcad", "/work", 1)
    cmd = (f"echo zjjiang | sudo -S -p '' docker run --rm --name {cname} "
           f"--runtime=nvidia --gpus all --entrypoint /isaac-sim/python.sh "
           f"-e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y "
           f"-v /data/physcad:/work "
           f"-v /data/isaac-cache/kit:/root/.local/share/ov/data/Kit "
           f"-v /data/isaac-cache/cache:/root/.cache "
           f"{img} /work/run_scenario.py "
           f"--urdf {c(urdf)} --asset-root {c(asset_root)} "
           f"--spec {c(spec_path)} --task {json.dumps(task)} --out {c(out_dir)}")
    print(f"[loop] running sim in container ({cname})...", flush=True)
    r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)
    # surface [scn] lines
    for line in (r.stdout + r.stderr).splitlines():
        if "[scn]" in line or "FAIL" in line or "PASS" in line:
            print("   " + line, flush=True)
    return Path(out_dir) / "sim_result.json"


def feedback_text(sim_result, vlm_result):
    m = sim_result["metrics"]
    parts = [f"MEASURED: verdict={m['verdict']} survived={m.get('survive_s')}s "
             f"min_base_z={m.get('min_base_z'):.3f} max_drift={m.get('max_drift'):.3f}"]
    if vlm_result:
        parts.append(f"CAMERA (VLM): {vlm_result.get('summary','')}")
        for f in vlm_result.get("failures", []):
            parts.append(f" - {f['failure_mode']}: {f['explanation']}")
    return "\n".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--asset-root", required=True)
    ap.add_argument("--task", required=True)
    ap.add_argument("--workdir", required=True)
    ap.add_argument("--max-iters", type=int, default=4)
    ap.add_argument("--robot-name", default="robot")
    a = ap.parse_args()

    wd = Path(a.workdir); wd.mkdir(parents=True, exist_ok=True)
    robot = robot_info_from_urdf(a.urdf, a.robot_name)
    print(f"[loop] robot: {len(robot['joints'])} joints, {len(robot['links'])} links")

    spec = scenario_designer.design(a.task, robot)
    history = []
    for it in range(a.max_iters):
        print(f"\n===== ITERATION {it} =====", flush=True)
        print(f"[loop] spec reasoning: {spec.get('reasoning','')[:240]}", flush=True)
        spec_path = wd / f"spec_{it}.json"; spec_path.write_text(json.dumps(spec, indent=2))
        out_dir = wd / f"iter_{it}"

        sim_result_path = run_in_container(a.urdf, a.asset_root, spec_path, a.task, out_dir)
        if not sim_result_path.exists():
            print("[loop] sim produced no result; stopping.", flush=True); break
        sim_result = json.loads(sim_result_path.read_text())
        m = sim_result["metrics"]
        print(f"[loop] sim verdict={m['verdict']} survived={m.get('survive_s')}s frames={sim_result.get('n_frames')}", flush=True)

        # VLM critique (only if frames exist)
        vlm = None
        if sim_result.get("n_frames", 0) > 0:
            vlm_out = out_dir / "vlm.json"
            try:
                frames = analyze.sample_frames(sim_result["frames_dir"], 12)
                if frames:
                    verdict = analyze.call_vlm(analyze.build_messages(
                        {"user_prompt": a.task, "pass_criteria": spec.get("pass_criteria", {})},
                        m, frames))
                    vlm = verdict
                    vlm_out.write_text(json.dumps(verdict, indent=2))
                    print(f"[loop] VLM passed={verdict['passed']} :: {verdict['summary'][:200]}", flush=True)
            except Exception as e:
                print(f"[loop] VLM step error: {e!r}", flush=True)

        passed = m["verdict"] == "PASS" and (vlm is None or vlm.get("passed", True))
        history.append({"iter": it, "sim_verdict": m["verdict"],
                        "vlm_passed": (vlm or {}).get("passed"),
                        "survive_s": m.get("survive_s")})
        if passed:
            print(f"\n[loop] *** SUCCESS at iteration {it} ***", flush=True); break
        if it == a.max_iters - 1:
            print(f"\n[loop] reached max iters ({a.max_iters}) without pass.", flush=True); break

        fb = feedback_text(sim_result, vlm)
        print(f"[loop] feeding back:\n{fb}", flush=True)
        spec = scenario_designer.revise(a.task, robot, spec, fb)

    (wd / "history.json").write_text(json.dumps(history, indent=2))
    print(f"\n[loop] done. history: {json.dumps(history)}", flush=True)


if __name__ == "__main__":
    main()
