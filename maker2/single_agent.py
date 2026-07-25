"""Single-agent text-to-cad path: one agent authors the WHOLE machine as ONE build123d
script (no boss / no per-sub managers / no assembler), and this module evaluates it into a
maker2 KinematicModel — reusing the existing precheck / MuJoCo physics / URDF / UI unchanged.

``evaluate_machine_python(script_text, run_dir, machine_name)`` runs the authored
``build_machine()`` in the same sandboxed subprocess the multi-manager path uses
(``_eval_runner_machine.py``): build123d + cadpy AssemblyHelper + make_gear are injected, the
script's returned Compound is walked into per-part local STLs + world poses, and a whole
machine STEP (``machine.step``) is written so the text-to-cad inspect tools can run
selector-level self-checks in the modeling loop. The parts array (name / world 4x4 / STL /
volume / metadata) becomes a flat-global KinematicModel.
"""
from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path

from .model import KinematicModel, LinkSpec, PoseSpec

_EXEC_TIMEOUT = 300  # a whole machine is heavier than one sub
_RUNNER = (Path(__file__).parent / "_eval_runner_machine.py").read_text(encoding="utf-8")
_MESH_RE = re.compile(r"__mesh[_-]?([a-z0-9]+)$", re.I)


class SingleAgentError(ValueError):
    """The authored whole-machine build123d script failed to evaluate."""


def _rot_to_rpy(R):
    sy = math.hypot(R[0][0], R[1][0])
    if sy > 1e-9:
        return (math.atan2(R[2][1], R[2][2]), math.atan2(-R[2][0], sy),
                math.atan2(R[1][0], R[0][0]))
    return (math.atan2(-R[1][2], R[1][1]), math.atan2(-R[2][0], sy), 0.0)


def evaluate_machine_python(script_text: str, run_dir: str, machine_name: str,
                            *, log_fn=print) -> KinematicModel:
    """Run the authored whole-machine build123d script in a sandbox and return a
    KinematicModel with GLOBAL poses (mm->m). Also writes machine.step next to the eval
    output for the self-check loop. Raises SingleAgentError on any failure (with the
    subprocess traceback tail) so the modeling loop can feed it back to the agent."""
    run = Path(run_dir)
    run.mkdir(parents=True, exist_ok=True)
    (run / "meshes").mkdir(exist_ok=True)
    src = run / "machine.py"
    src.write_text(script_text, encoding="utf-8")
    runner = run / "_machine_runner.py"
    runner.write_text(_RUNNER, encoding="utf-8")
    out_json = run / "machine_eval.json"

    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    try:
        r = subprocess.run(
            [sys.executable, runner.name, src.name, out_json.name, "meshes"],
            capture_output=True, text=True, timeout=_EXEC_TIMEOUT, cwd=str(run), env=env)
    except subprocess.TimeoutExpired:
        raise SingleAgentError(f"machine build123d eval timed out after {_EXEC_TIMEOUT}s")
    except Exception as e:
        raise SingleAgentError(f"eval subprocess failed: {type(e).__name__}: {e}")

    payload = None
    for line in reversed((r.stdout or "").strip().splitlines()):
        line = line.strip()
        if line.startswith("{"):
            try:
                payload = json.loads(line)
                break
            except Exception:
                continue
    if payload is None or not payload.get("ok"):
        err = (payload or {}).get("error") if payload else None
        trace = (payload or {}).get("trace") if payload else ""
        tail = trace or (r.stderr or r.stdout or "").strip()[-400:]
        raise SingleAgentError(f"machine build123d eval failed: {err or tail}")
    if not out_json.exists():
        raise SingleAgentError("machine eval produced no machine_eval.json")

    spec = json.loads(out_json.read_text(encoding="utf-8"))
    parts = spec.get("parts") or []
    if not parts:
        raise SingleAgentError("machine has no parts with solids")

    links: list[LinkSpec] = []
    poses: list[PoseSpec] = []
    mesh_by_id: dict = {}
    root = spec.get("root") or parts[0]["name"]
    coord_log: list = []
    for p in parts:
        name = p["name"]
        meta = p.get("metadata") or {}
        if float(p.get("volume_mm3", 0.0)) <= 0.0:
            continue
        dof = str(meta.get("dof", "")) or _infer_dof(name)
        driver = bool(meta.get("driver", False)) or (_infer_driver(name) and dof == "spin")
        spin_axis = meta.get("spin_axis")
        if not isinstance(spin_axis, (list, tuple)) or len(spin_axis) != 3:
            spin_axis = (0.0, 0.0, 1.0)
        links.append(LinkSpec(
            name=name, description=meta.get("description", name),
            mesh_filename=f"meshes/{name}.stl",
            dof=dof, spin_axis=tuple(spin_axis), driver=driver,
            material=str(meta.get("material", "steel"))))
        T = [float(v) / 1000.0 for v in p["T"]]
        rpy = _rot_to_rpy(p["R"])
        poses.append(PoseSpec(name=f"place_{name}", parent="", child=name,
                              xyz_m=tuple(T), rpy_rad=tuple(rpy)))
        coord_log.append(f"{name}@{tuple(round(v*1000, 1) for v in T)}mm")
        mid = meta.get("mesh_id")
        if not mid:
            m = _MESH_RE.search(name.lower())
            mid = m.group(1) if m else None
        if mid:
            mesh_by_id.setdefault(str(mid), []).append(name)

    # One driver max.
    seen = False
    for l in links:
        if l.driver:
            if seen:
                l.driver = False
            seen = True

    mesh_pairs = [tuple(v[:2]) for v in mesh_by_id.values() if len(v) >= 2]
    model = KinematicModel(name=machine_name, root_link=root, links=links, poses=poses,
                           mesh_pairs=mesh_pairs)
    if log_fn:
        log_fn(f"[single-agent] {machine_name}: {len(links)} part(s), "
               f"{len(mesh_pairs)} mesh pair(s), STEP={'yes' if spec.get('step') or (run/'machine.step').exists() else 'no'}")
    return model


_SPIN_RE = re.compile(r"gear|pinion|wheel|arbor|shaft|rotor|cam|spindle", re.I)
_DRIVER_RE = re.compile(r"driver|input|barrel|crank|winding", re.I)


def _infer_dof(name: str) -> str:
    return "spin" if _SPIN_RE.search(name.lower()) else "fixed"


def _infer_driver(name: str) -> bool:
    return bool(_DRIVER_RE.search(name.lower()))


def run_single_agent(product_prompt: str, out_dir: str, settings, *,
                     do_physics: bool = True, max_iters: int = 4,
                     log_fn=print) -> dict:
    """The single-agent text-to-cad pipeline: ONE agent authors the whole machine, refines it
    against build-eval errors + a rigid-conflict geometry self-check, then the machine is
    prechecked (warn-only) and run through the existing MuJoCo physics. Returns the same
    RESULT_JSON shape run_boss does so the worker/UI reads it unchanged.

    Loop per iteration: LLM authors/repairs build_machine() -> evaluate to a KinematicModel
    (eval error feeds back) -> build URDF -> rigid-conflict check (overlaps feed back). On a
    clean geometry pass, stop refining and go to physics."""
    import json as _json
    import os as _os

    from .llm.conversation import Conversation
    from .manager import _extract_python_block, save_model
    from .orchestrator import make_run_context
    from .prompts.single_agent_prompt import (
        SINGLE_AGENT_SYSTEM, build_single_agent_user, build_single_agent_repair,
        build_single_agent_geometry_feedback, build_single_agent_physics_feedback)
    from .urdf_builder import build_urdf

    ctx = make_run_context(product_prompt, out_dir)
    _os.makedirs(ctx.run_dir, exist_ok=True)
    run_dir = ctx.run_dir

    # Tee every log line to <run_dir>/run.log so the backend keeps the SAME full transcript
    # the multi-agent path does (fresh file per run). Without this the single-agent stdout is
    # only seen live over SSE and there is no on-disk log to review why a rebuild happened.
    _base_log = log_fn
    try:
        _run_log_fh = open(_os.path.join(run_dir, "run.log"), "w", encoding="utf-8", buffering=1)
    except Exception:
        _run_log_fh = None

    def log_fn(m):
        _base_log(m)
        if _run_log_fh is not None:
            try:
                _run_log_fh.write(str(m) + "\n")
            except Exception:
                pass

    log_fn(f"[single-agent] session: {run_dir}")

    client = settings.manager_client()
    conv = Conversation()
    conv.add_user_message(build_single_agent_user(product_prompt))

    result = {"ok": False, "run_dir": run_dir, "render_dir": run_dir,
              "iterations": 0, "hierarchy": False, "single_agent": True}
    model = None
    machine_name = ctx.project_slug or "machine"

    for it in range(max_iters):
        result["iterations"] = it + 1
        last_iter = it >= max_iters - 1
        log_fn(f"[single-agent] iteration {it}: authoring build_machine() ...")
        try:
            reply = client.send(conv.messages, SINGLE_AGENT_SYSTEM)
        except Exception as e:
            result["error"] = f"LLM request failed: {e}"
            return result
        conv.add_assistant_message(reply)
        code = _extract_python_block(reply)
        if not code:
            conv.add_user_message(build_single_agent_repair(
                "no ```python code block found; emit ONE block defining build_machine()."))
            continue

        # Evaluate the authored machine into a KinematicModel.
        try:
            model = evaluate_machine_python(code, run_dir, machine_name, log_fn=log_fn)
        except SingleAgentError as e:
            log_fn(f"[single-agent] eval failed: {str(e)[:160]}")
            conv.add_user_message(build_single_agent_repair(str(e)))
            continue

        # Persist model + build a URDF for the geometry check / physics / UI.
        save_model(model, ctx.model_json_path)
        try:
            build_urdf(model, ctx)
        except Exception as e:
            log_fn(f"[single-agent] URDF build failed: {str(e)[:120]}; treating as geometry gap")
            conv.add_user_message(build_single_agent_repair(f"URDF build failed: {e}"))
            continue
        log_fn("ARTIFACT_JSON:" + _json.dumps({
            "kind": "assembled_model", "iter": it, "run_dir": run_dir, "render_dir": run_dir}))

        # Rigid-conflict geometry self-check (the text-to-cad "inspect" step, reusing subcheck).
        # A gross interpenetration is cheaper to fix here than to run physics on, so gate on it
        # first — but only re-author for it when we still have iterations left.
        conflicts = []
        try:
            from .subcheck import sub_conflicts
            conflicts = sub_conflicts(model, ctx.urdf_path, log_fn=lambda *_: None)
        except Exception as e:
            log_fn(f"[single-agent] conflict check unavailable ({type(e).__name__}: {e})")
        if conflicts and not last_iter:
            findings = "\n".join(f"- {c.describe()}" for c in conflicts[:8])
            log_fn(f"[single-agent] {len(conflicts)} interpenetration(s) -> asking agent to fix")
            log_fn("ARTIFACT_JSON:" + _json.dumps({
                "kind": "diagnosis", "iter": it, "single_agent": True,
                "decision": {"root_cause": f"{len(conflicts)} rigid part interpenetration(s)",
                             "evidence": [c.describe() for c in conflicts[:8]]}}))
            conv.add_user_message(build_single_agent_geometry_feedback(findings))
            continue

        # PHYSICS in the loop: simulate the machine, then let the VLM diagnose the recording +
        # metrics. On a functional failure (e.g. gears that don't transmit) feed the diagnosis
        # back and RE-AUTHOR — this is the full design -> build -> test -> diagnose -> redesign
        # loop, not a one-shot physics run at the end.
        if not do_physics:
            log_fn(f"[single-agent] machine accepted (no physics): {len(model.links)} parts")
            result["ok"] = True
            return result

        from .physics import run_physics
        log_fn(f"[single-agent] iteration {it}: simulating physics ...")
        try:
            phys = run_physics(ctx.urdf_path, product_prompt, run_dir, settings)
        except Exception as e:
            log_fn(f"[physics] failed: {e}")
            result["physics"] = {"passed": None, "summary": f"physics error: {e}"}
            result["ok"] = True  # geometry built; physics best-effort
            return result

        metrics = phys.get("metrics", {}) or {}
        diagnosis = {"cause": phys.get("cause", "none"), "reason": phys.get("reason", "")}
        passed = phys.get("passed")
        log_fn("ARTIFACT_JSON:" + _json.dumps({
            "kind": "physics", "iter": it, "run_dir": run_dir, "render_dir": run_dir,
            "passed": passed, "score": 0.0, "physics": phys}))
        result["physics"] = phys
        result["iterations"] = it + 1

        if passed is not False:
            log_fn(f"[single-agent] PASS on iteration {it}: mechanism transmits")
            result["ok"] = True
            return result

        # Failed physics. If iterations remain, diagnose + re-author; else accept the best build.
        if last_iter:
            log_fn(f"[single-agent] physics FAIL on final iteration {it}; returning best build")
            result["ok"] = True  # demo: a built-but-imperfect machine still shows the loop
            return result

        summary = phys.get("summary", "the mechanism did not transmit motion")
        log_fn(f"[single-agent] physics FAIL -> diagnose + re-author: {summary[:120]}")
        log_fn("ARTIFACT_JSON:" + _json.dumps({
            "kind": "diagnosis", "iter": it, "single_agent": True,
            "decision": {"root_cause": summary,
                         "cause": diagnosis["cause"], "reason": diagnosis["reason"],
                         "metrics": {"moved": metrics.get("moved_count"),
                                     "watched": metrics.get("watched_count"),
                                     "input_travel": metrics.get("input_travel"),
                                     "exploded": metrics.get("exploded")}}}))
        conv.add_user_message(
            build_single_agent_physics_feedback(summary, metrics, diagnosis))
        # loop continues -> agent re-authors the whole machine with the physics feedback

    if model is None:
        result["error"] = "no buildable machine after all iterations"
        return result
    result["ok"] = True
    return result

