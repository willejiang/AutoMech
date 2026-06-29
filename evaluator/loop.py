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

# auto-load evaluator/.env so a normal user just needs the proxy running
_env = Path(__file__).resolve().parent / ".env"
if _env.exists():
    for line in _env.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1); os.environ.setdefault(k, v)

import scenario_designer
import analyze
import strategy_selector
from sim_backends import select_backend


def run_pybullet(urdf, spec_path, task, out_dir):
    """Local CPU run — no docker, no GPU, no sudo."""
    here = Path(__file__).resolve().parent
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    r = subprocess.run([sys.executable, str(here / "run_scenario_pybullet.py"),
                        "--urdf", str(urdf), "--spec", str(spec_path),
                        "--out", str(out_dir), "--task", task],
                       capture_output=True, text=True)
    print((r.stdout + r.stderr)[-600:], flush=True)
    return Path(out_dir) / "sim_result.json"


def run_openfoam(stl, spec_path, task, out_dir):
    """CFD run — uses simpleFoam if present, else writes a stub (box offline)."""
    here = Path(__file__).resolve().parent
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    r = subprocess.run([sys.executable, str(here / "run_scenario_openfoam.py"),
                        "--stl", str(stl), "--spec", str(spec_path),
                        "--out", str(out_dir), "--task", task],
                       capture_output=True, text=True)
    print((r.stdout + r.stderr)[-600:], flush=True)
    return Path(out_dir) / "sim_result.json"


def run_backend(backend, urdf, asset_root, spec_path, task, out_dir):
    """Dispatch to the simulator the evaluator chose."""
    if backend == "pybullet":
        return run_pybullet(urdf, spec_path, task, out_dir)
    if backend == "openfoam":
        return run_openfoam(urdf, spec_path, task, out_dir)
    return run_in_container(urdf, asset_root, spec_path, task, out_dir)


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
    ap.add_argument("--asset-root", default="", help="only needed for isaac_sim backend")
    ap.add_argument("--task", required=True)
    ap.add_argument("--workdir", required=True)
    ap.add_argument("--max-iters", type=int, default=4)
    ap.add_argument("--robot-name", default="robot")
    ap.add_argument("--backend", default="pybullet",
                    help="pybullet|isaac_sim|openfoam|auto (auto = strategy_selector picks)")
    a = ap.parse_args()

    wd = Path(a.workdir); wd.mkdir(parents=True, exist_ok=True)
    robot = robot_info_from_urdf(a.urdf, a.robot_name)
    print(f"[loop] robot: {len(robot['joints'])} joints, {len(robot['links'])} links")

    if a.backend == "auto":
        decision = strategy_selector.decide(a.task, robot)
        backend = select_backend(decision)
        print(f"[loop] backend={backend} :: {decision.get('backend_reason','')[:120]}")
    else:
        backend, decision = a.backend, {}

    # early return to worker: jointless/unfixable structure
    if decision.get("action") == "return_to_worker" or decision.get("structurally_feasible") is False:
        msg = decision.get("worker_message") or decision.get("structural_concern") or "design cannot be evaluated"
        result = {"passed": False, "returned_to_worker": True, "summary": msg, "failures": []}
        out0 = wd / "iter_0"; out0.mkdir(parents=True, exist_ok=True)
        (out0 / "result.json").write_text(json.dumps(result, indent=2))
        print(f"[loop] RETURN TO WORKER: {msg}"); return [result]

    tests = decision.get("tests") or [{"name": "task", "goal": a.task, "strategy": decision.get("strategy", "static_stability")}]
    print(f"[loop] test set: {[t['name'] for t in tests]}")
    failures, history = [], []
    for ti, test in enumerate(tests):
        print(f"\n===== TEST {ti}: {test['name']} — {test['goal']} =====", flush=True)
        spec = scenario_designer.design(a.task, robot, test)
        spec_path = wd / f"spec_{test['name']}.json"; spec_path.write_text(json.dumps(spec, indent=2))
        out_dir = wd / f"test_{test['name']}"
        sim_result_path = run_backend(backend, a.urdf, a.asset_root, spec_path, a.task, out_dir)
        if not sim_result_path.exists():
            failures.append({"failure_mode": test["name"], "explanation": "sim produced no result", "fix_hint": ""}); continue
        sim_result = json.loads(sim_result_path.read_text()); m = sim_result["metrics"]
        vlm = None
        if sim_result.get("n_frames", 0) > 0:
            frames = analyze.sample_frames(sim_result["frames_dir"], 12)
            if frames:
                vlm = analyze.call_vlm(analyze.build_messages(
                    {"user_prompt": f"{a.task} :: {test['goal']}", "pass_criteria": spec.get("pass_criteria", {})}, m, frames))
        passed = m["verdict"] == "PASS" and (vlm is None or vlm.get("passed", True))
        history.append({"test": test["name"], "passed": passed, "verdict": m["verdict"]})
        if not passed:
            for f in (vlm or {}).get("failures", []) or [{"failure_mode": test["name"], "explanation": m["verdict"], "fix_hint": ""}]:
                failures.append({**f, "test": test["name"]})
        print(f"[loop] {test['name']}: passed={passed}", flush=True)

    result = {"passed": not failures, "returned_to_worker": False,
              "summary": "all tests passed" if not failures else f"{len(failures)} failure(s) across {len(tests)} tests",
              "failures": failures}
    (wd / "result.json").write_text(json.dumps(result, indent=2))
    (wd / "history.json").write_text(json.dumps(history, indent=2))
    print(f"\n[loop] done. {result['summary']}", flush=True)
    return [result]


if __name__ == "__main__":
    main()
