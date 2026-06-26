#!/usr/bin/env python3
"""
evaluator_bridge.py — hand a design dir to the Isaac Sim evaluator and read back
its verdict. This is the SLOW, physical half of the loop.

The design dir (manifest.json + model.scad) is exactly what evaluator/evaluate.sh
consumes; it runs run_eval.py in the isaac-sim container (scad -> per-part STL ->
synth URDF -> sim -> frames + metrics) then analyze.py (VLM pass/fail), writing
out/result.json. We just shell it and parse that file.

OFFLINE NOTE: the Aliyun A10 box is down, so the real sim can't run. dry_run=True
(or Docker/evaluate.sh absent) writes a STUB result.json — passed:false with a
synthetic fix hint — and logs loudly that the sim was SKIPPED. The wiring is then
exercisable end-to-end without a GPU; flip dry_run off once the box is back.
"""
import json
import subprocess
from pathlib import Path
from shutil import which

EVAL_SH = Path(__file__).resolve().parents[1] / "evaluator" / "evaluate.sh"


def _stub_result(design_dir, reason):
    """Synthetic verdict so the loop keeps flowing when the sim can't run."""
    manifest = {}
    mpath = Path(design_dir) / "manifest.json"
    if mpath.exists():
        manifest = json.loads(mpath.read_text())
    return {
        "design_id": manifest.get("design_id"),
        "user_prompt": manifest.get("user_prompt"),
        "passed": False,
        "stubbed": True,
        "summary": f"[SIM SKIPPED — {reason}] No physics verdict available. "
                   f"Wiring exercised; real Isaac Sim run required for a true verdict.",
        "failures": [{
            "failure_mode": "not_simulated",
            "explanation": reason,
            "frame_indices": [],
            "frame_files": [],
            "fix_hint": "Bring the Isaac Sim box online and re-run without --dry-run.",
        }],
        "metrics": {},
        "all_frames": [],
    }


def evaluate(design_dir, dry_run=False):
    """-> result dict (the maker handoff). Reads evaluator out/result.json, or a
    stub when the sim is skipped."""
    design_dir = Path(design_dir).resolve()
    out_result = design_dir / "out" / "result.json"

    if dry_run:
        res = _stub_result(design_dir, "dry_run requested")
        _write_stub(out_result, res)
        print(f"[eval] DRY RUN — wrote stub verdict to {out_result}")
        return res

    if not EVAL_SH.exists():
        res = _stub_result(design_dir, f"evaluate.sh not found at {EVAL_SH}")
        _write_stub(out_result, res); return res
    if not which("docker"):
        res = _stub_result(design_dir, "docker not available on this host (box offline?)")
        _write_stub(out_result, res)
        print("[eval] docker missing — stubbed verdict (sim skipped).")
        return res

    print(f"[eval] running evaluator: {EVAL_SH} {design_dir}")
    r = subprocess.run(["bash", str(EVAL_SH), str(design_dir)],
                       capture_output=True, text=True)
    print(r.stdout[-2000:] if r.stdout else "")
    if r.returncode != 0:
        print(f"[eval] evaluate.sh failed (rc={r.returncode}):\n{r.stderr[-2000:]}")
        if not out_result.exists():
            res = _stub_result(design_dir, f"evaluate.sh exited {r.returncode}")
            _write_stub(out_result, res); return res
    if not out_result.exists():
        res = _stub_result(design_dir, "evaluator produced no result.json")
        _write_stub(out_result, res); return res
    return json.loads(out_result.read_text())


def _write_stub(out_result, res):
    out_result.parent.mkdir(parents=True, exist_ok=True)
    out_result.write_text(json.dumps(res, indent=2))


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("design_dir")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    print(json.dumps(evaluate(a.design_dir, a.dry_run), indent=2))
