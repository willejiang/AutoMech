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


def _homogeneous(R, T):
    return [[R[0][0], R[0][1], R[0][2], float(T[0])],
            [R[1][0], R[1][1], R[1][2], float(T[1])],
            [R[2][0], R[2][1], R[2][2], float(T[2])],
            [0.0, 0.0, 0.0, 1.0]]


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _mat_inv(m):
    # Inverse of a rigid transform: R^-1 = R^T, t^-1 = -R^T @ t.
    rt = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [m[0][3], m[1][3], m[2][3]]
    nt = [-sum(rt[i][k] * t[k] for k in range(3)) for i in range(3)]
    return [[rt[0][0], rt[0][1], rt[0][2], nt[0]],
            [rt[1][0], rt[1][1], rt[1][2], nt[1]],
            [rt[2][0], rt[2][1], rt[2][2], nt[2]],
            [0.0, 0.0, 0.0, 1.0]]


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

    # World transforms keyed by part name — authoritative (the agent's .moved() coords, which
    # the move-vs-connect experiment showed give 0 positioning error). A mounted part's pose is
    # made RELATIVE to its parent by parent_world^-1 @ child_world, so the parent/child body tree
    # anchors the train to the world instead of every part being a free root.
    names = {p["name"] for p in parts if float(p.get("volume_mm3", 0.0)) > 0.0}
    world_by_name: dict = {}
    for p in parts:
        if float(p.get("volume_mm3", 0.0)) <= 0.0:
            continue
        world_by_name[p["name"]] = _homogeneous(p["R"], p["T"])

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
        # A valid mount names another part; otherwise treat as a world root (parent="").
        mount = str(meta.get("mount", "") or "").strip()
        parent = mount if (mount and mount != name and mount in names) else ""
        if parent:
            rel = _mat_mul(_mat_inv(world_by_name[parent]), world_by_name[name])
            T = [rel[0][3] / 1000.0, rel[1][3] / 1000.0, rel[2][3] / 1000.0]
            rpy = _rot_to_rpy([[rel[r][c] for c in range(3)] for r in range(3)])
        else:
            T = [float(v) / 1000.0 for v in p["T"]]
            rpy = _rot_to_rpy(p["R"])
        poses.append(PoseSpec(name=f"place_{name}", parent=parent, child=name,
                              xyz_m=tuple(T), rpy_rad=tuple(rpy)))
        coord_log.append(f"{name}@{tuple(round(v*1000, 1) for v in T)}mm"
                         + (f"<-{parent}" if parent else ""))
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


def _iter_score(phys: dict) -> float:
    """Score one physics result so iterations are comparable and we can keep the BEST
    version (and roll back to it when a later edit makes things worse).

    The machine is a TRANSMISSION mechanism, so the ranking is dominated by FUNCTION —
    how far the drive actually propagates — NOT by how sturdily a dead machine sits. An
    earlier version gave +1000 for merely settling and +300 for not exploding, so every
    jammed-but-stable machine tied at ~1300 and `best` was decided by input-travel noise;
    a genuinely-turning machine that settled slightly imperfectly could score LOWER than a
    welded brick and get rolled back. Now stability is a THRESHOLD (a hard penalty when it
    fails, a small base when it holds), and the big gradient is functional:

      passed (diagnoser verdict)      -> +10000   (a working mechanism, uncatchable by any FAIL)
      exploded / blew apart           -> hard floor near -100 (worst; never 'best')
      stability FAIL (but no explode) -> -500 base (a machine that can't even sit is bad)
      stability PASS                  -> +200 base (the precondition, not a jackpot)
      output_reached                  -> +4000   (drive crossed the whole train — the point)
      fraction of downstream moved    -> +0..3000 (how much of the train transmits)
      input actually turned           -> +0..800  (at least the driver broke free of a jam)
    So a jammed machine (input ~0, nothing moved) lands near its stability base (~200) while
    ANY real transmission outranks it, and `best`/rollback follow the diagnoser, not noise.
    """
    if not phys:
        return -1.0
    m = phys.get("metrics") or {}
    st = phys.get("stability") or {}

    # A working mechanism (the diagnoser passed it) is in a class of its own.
    if phys.get("passed") is True:
        return 10000.0

    # Exploded/blew apart under drive is the worst outcome — hard floor, never near 'best'.
    if m.get("exploded"):
        return -100.0 + 100.0 * min(1.0, (m.get("moved_count") or 0) / max(1, m.get("watched_count") or 1))

    # Stability is a THRESHOLD, not a jackpot: a small base when it holds, a penalty when
    # it fails (a machine that can't sit on the bench is worse than one that can).
    stable = str(st.get("verdict", "")).upper() == "PASS" and not st.get("exploded")
    score = 200.0 if stable else -500.0

    # FUNCTION is the dominant gradient. Drive crossing the whole train is the goal.
    if m.get("output_reached") is True:
        score += 4000.0
    watched = m.get("watched_count") or 0
    moved = m.get("moved_count") or 0
    if watched:
        score += 3000.0 * min(1.0, moved / watched)

    # The driver at least breaking free of a jam is a weak-but-real signal, scaled to how
    # much of the commanded sweep it achieved (fall back to a nominal 10 rad target).
    it = float(m.get("input_travel") or 0.0)
    if 0.0 < it < 1000.0:
        score += 800.0 * min(1.0, it / 10.0)
    return score


def _restore_best(best: dict, best_dir: str, run_dir: str, ctx, machine_name, log_fn):
    """Make the main run_dir hold the BEST version's artifacts so the UI/return shows the
    best machine, not the last (often divergent) iteration. Prefers copying the snapshot
    files back; falls back to re-evaluating best['code'] if the snapshot is missing."""
    import os
    import shutil
    copied = False
    try:
        if os.path.isdir(best_dir):
            for fn in ("machine.py", "kinematic_model.json", "model.urdf"):
                src = os.path.join(best_dir, fn)
                if os.path.exists(src):
                    shutil.copy2(src, os.path.join(run_dir, fn))
                    copied = True
    except Exception as e:
        log_fn(f"[single-agent] restore-best copy failed: {e}")
    if copied:
        return
    # Fallback: rebuild from the best code string.
    code = best.get("code")
    if not code:
        return
    try:
        from .urdf_builder import build_urdf
        model = evaluate_machine_python(code, run_dir, machine_name, log_fn=log_fn)
        save_model_ref = _lazy_save_model()
        save_model_ref(model, ctx.model_json_path)
        build_urdf(model, ctx)
    except Exception as e:
        log_fn(f"[single-agent] restore-best rebuild failed: {e}")


def _lazy_save_model():
    from .manager import save_model
    return save_model


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

    # RESEARCH PRE-STEP (web + local KB), same as the multi-agent manager gets. The single
    # agent authors the WHOLE drivetrain from memory otherwise — it guesses gear modules,
    # tooth counts and center distances, which is exactly what keeps going wrong. Look up
    # gear math / standard sizes / worked examples FIRST so the numbers are grounded. Gated
    # by settings.enable_reference_tools (web) / enable_kb (local); a no-op if both are off.
    try:
        from .tools import maybe_research
        maybe_research(client, conv, settings,
                       f"authoring the complete mechanism for: {product_prompt} — gear "
                       f"module/tooth-count/center-distance math, standard sizes, and any "
                       f"worked gear-train / watch-movement examples",
                       collection="manager", log_fn=log_fn)
    except Exception as e:
        log_fn(f"[single-agent] research pre-step skipped: {e}")

    result = {"ok": False, "run_dir": run_dir, "render_dir": run_dir,
              "iterations": 0, "hierarchy": False, "single_agent": True}
    model = None
    machine_name = ctx.project_slug or "machine"

    # max_iters <= 0 means "iterate until physics PASSes" (the design->test->fix loop is
    # not artificially capped). A hard ceiling still bounds a pathological run that can
    # never converge, so it can't spin forever burning tokens.
    _HARD_CEILING = 50
    unlimited = max_iters <= 0
    iter_cap = _HARD_CEILING if unlimited else max_iters

    # Keep the BEST version seen so far (highest _iter_score). When a later iteration
    # regresses (a new edit made things worse — exploded / fell apart), we feed this best
    # code back as the starting point so the agent refines it instead of rewriting the whole
    # machine and losing hard-won stability. On finish we return the BEST, not the last
    # (often-divergent) iteration.
    best = {"score": float("-inf"), "code": None, "phys": None}
    best_dir = _os.path.join(run_dir, "best")

    # GEOMETRY rollback: the geometry gate (interpenetration + floating) has no physics
    # score, so a run that kept re-authoring for geometry could DIVERGE — the agent adds
    # filler parts / restructures and the fault count grows instead of shrinking. Track the
    # fewest geometry faults seen and the code that achieved it; when a later attempt is
    # WORSE, hand that best-geometry code back so it refines the closest version, not its
    # own worse one. (Lower badness = better; badness = interpenetrations + floaters + gap.)
    geo_best = {"badness": float("inf"), "code": None}

    for it in range(iter_cap):
        result["iterations"] = it + 1
        last_iter = (not unlimited) and it >= max_iters - 1
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
        floaters = []
        try:
            from .subcheck import floating_parts, sub_conflicts
            conflicts = sub_conflicts(model, ctx.urdf_path, log_fn=lambda *_: None)
            floaters = floating_parts(model, ctx.meshes_dir, log_fn=lambda *_: None)
        except Exception as e:
            log_fn(f"[single-agent] geometry check unavailable ({type(e).__name__}: {e})")
        if (conflicts or floaters) and not last_iter:
            findings = "\n".join([f"- {c.describe()}" for c in conflicts[:8]]
                                 + [f"- {f.describe()}" for f in floaters[:8]])
            # Geometry badness: count of faults + total floating gap (mm/100 as a tiebreak).
            badness = (len(conflicts) + len(floaters)
                       + sum(getattr(f, "parent_gap_mm", 0.0) or f.gap_mm
                             for f in floaters) / 100.0)
            geo_regressed = badness > geo_best["badness"]
            if badness < geo_best["badness"]:
                geo_best = {"badness": badness, "code": code}
            log_fn(f"[single-agent] {len(conflicts)} interpenetration(s) + "
                   f"{len(floaters)} floating part(s) -> asking agent to fix "
                   f"(badness={badness:.2f}, best={geo_best['badness']:.2f}"
                   f"{', REGRESSED' if geo_regressed else ''})")
            log_fn("ARTIFACT_JSON:" + _json.dumps({
                "kind": "diagnosis", "iter": it, "single_agent": True,
                "decision": {"root_cause": f"{len(conflicts)} interpenetration(s), "
                                           f"{len(floaters)} unsupported part(s)",
                             "evidence": [c.describe() for c in conflicts[:8]]
                                         + [f.describe() for f in floaters[:8]]}}))
            # On regression, refine the BEST-geometry code instead of the worse latest one.
            rollback_code = geo_best["code"] if (geo_regressed and geo_best["code"]) else None
            conv.add_user_message(build_single_agent_geometry_feedback(
                findings, best_code=rollback_code))
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

        # Score this iteration and update BEST. A higher score = closer to a working,
        # stable machine. Snapshot the best version's code + built model/urdf so we can
        # return it (and roll back to it) instead of the last, possibly-divergent build.
        score = _iter_score(phys)
        regressed = score < best["score"]
        if score > best["score"]:
            best = {"score": score, "code": code, "phys": phys}
            try:
                _os.makedirs(best_dir, exist_ok=True)
                import shutil as _shutil
                for fn in ("machine.py", "kinematic_model.json", "model.urdf"):
                    src = _os.path.join(run_dir, fn)
                    if _os.path.exists(src):
                        _shutil.copy2(src, _os.path.join(best_dir, fn))
            except Exception as e:
                log_fn(f"[single-agent] best snapshot failed: {e}")
        log_fn(f"[single-agent] iter {it} score={score:.0f} (best={best['score']:.0f}"
               f"{', REGRESSED' if regressed else ''})")

        log_fn("ARTIFACT_JSON:" + _json.dumps({
            "kind": "physics", "iter": it, "run_dir": run_dir, "render_dir": run_dir,
            "passed": passed, "score": score, "physics": phys}))
        result["physics"] = phys
        result["iterations"] = it + 1

        if passed is not False:
            log_fn(f"[single-agent] PASS on iteration {it}: mechanism transmits")
            result["ok"] = True
            return result

        # Failed physics. If iterations remain, diagnose + re-author; else return BEST.
        if last_iter:
            log_fn(f"[single-agent] physics FAIL on final iteration {it}; returning BEST "
                   f"(score={best['score']:.0f})")
            _restore_best(best, best_dir, run_dir, ctx, machine_name, log_fn)
            result["physics"] = best["phys"] or result.get("physics")
            result["ok"] = True
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
        # When this iteration REGRESSED below the best, feed the best code back so the agent
        # refines the known-good version instead of rewriting from its own worse attempt.
        conv.add_user_message(
            build_single_agent_physics_feedback(
                summary, metrics, diagnosis, stability=phys.get("stability"),
                best_code=(best["code"] if regressed else None), regressed=regressed))
        # loop continues -> agent refines with the physics feedback (+ rollback if regressed)

    if model is None:
        result["error"] = "no buildable machine after all iterations"
        return result
    # Ran the full cap without a PASS: return the BEST version, not the last (divergent) one.
    log_fn(f"[single-agent] cap reached; returning BEST (score={best['score']:.0f})")
    _restore_best(best, best_dir, run_dir, ctx, machine_name, log_fn)
    if best.get("phys"):
        result["physics"] = best["phys"]
    result["ok"] = True
    return result

