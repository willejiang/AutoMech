#!/usr/bin/env python3
"""Physics evaluation of a maker2 URDF — category-aware, not a fool stand-still.

The old version dropped the model from 0.5 m and PASSed if it didn't topple — a
solid brick passed, and a gearbox was never actually driven. This version routes
through the evaluator's planner:

  strategy_selector.decide  -> pick static_stability / driven_mechanism / ... + a
                               test set (via maker2's 8313 gateway)
  scenario_designer.design  -> a scenario spec per test (a `drive` block for a
                               machine: which input joint to turn, what downstream
                               joints should move)
  run_scenario_pybullet.run -> actually actuate + measure transmission (or hold +
                               measure stability), capture frames

For a MACHINE this drives the input joint and checks the mechanism transmits; for a
structure/toy it keeps the stability check. Falls back gracefully (a static test)
if the planner is unavailable, so --physics never hard-crashes the run.

This module is the maker2-SIDE BRIDGE: it loads maker2's KinematicModel, builds the
selector/designer input, and wires maker2's gateway creds into the provider-agnostic
evaluator engine (strategy_selector, scenario_designer, run_scenario_pybullet, and
evaluator.diagnose's `diagnose_physics` verdict + `encode_mp4` video). The VLM verdict
+ fault classification and the frames->MP4 encode live in evaluator/diagnose.py.
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import threading
from pathlib import Path

# evaluator/ holds the planner + PyBullet runner; add repo root for the import.
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "evaluator"))
sys.path.insert(0, str(_ROOT))

_MOVABLE = {"revolute", "prismatic", "continuous"}
_DRIVER_HINT = re.compile(r"crank|handle|input|winder|drive|knob", re.I)
_OUTPUT_HINT = re.compile(r"output|escape|second|minute|hour|hand|tip|end|final", re.I)


def _static_spec() -> dict:
    """The legacy stability spec: hold, settle, check it doesn't sink/topple/drift."""
    return {
        "base_height": 0.5, "base_orientation_euler": [0, 0, 0],
        "self_collision": False, "control": {}, "joint_pose": [], "drive": None,
        "fixed_base": False, "duration_s": 4.0,
        "pass_criteria": {"min_base_height": 0.05, "max_drift": 0.5, "survive_s": 4.0},
    }


def _load_model(run_dir: str):
    """Load the KinematicModel saved next to this run, if present (for joint info)."""
    try:
        from maker2.manager import load_model
        p = os.path.join(run_dir, "kinematic_model.json")
        return load_model(p) if os.path.exists(p) else None
    except Exception:
        return None


def _robot_info(model) -> dict:
    """Selector/designer input: movable joints (with chain) + links + driver tag +
    a deterministic ROLE MAP (what can move and what can't) derived from topology."""
    joints = []
    for j in model.joints:
        if j.type in _MOVABLE:
            joints.append({"name": j.name, "type": j.type,
                           "parent": j.parent, "child": j.child,
                           "driver": bool(getattr(j, "driver", False))})
    return {"name": model.name,
            "joints": joints,
            "links": [l.name for l in model.links],
            "roles": _roles(model),
            "subsystems": _subsystems(model)}


def _infer_driver(model) -> str | None:
    """Pick the input joint when the manager didn't tag one: (a) driver flag, else
    (b) a movable joint on a crank/handle/input-named link, else (c) the movable
    joint nearest the root, else (d) the first movable joint. Returns joint name."""
    movable = [j for j in model.joints if j.type in _MOVABLE]
    if not movable:
        return None
    for j in movable:                                   # (a) explicit tag
        if getattr(j, "driver", False):
            return j.name
    for j in movable:                                   # (b) name heuristic
        if _DRIVER_HINT.search(j.child) or _DRIVER_HINT.search(j.parent):
            return j.name
    root = model.root_link                              # (c) nearest the root
    for j in movable:
        if j.parent == root:
            return j.name
    return movable[0].name                              # (d) first movable


def _adjacency(model):
    """Undirected link graph over ALL joints (fixed included) -> {link: [links]}.
    Fixed joints are rigid welds but still CONNECT links into one body, so they
    belong in the connectivity graph used to group parts into subsystems."""
    adj: dict[str, set] = {}
    for j in model.joints:
        adj.setdefault(j.parent, set()).add(j.child)
        adj.setdefault(j.child, set()).add(j.parent)
    return adj


def _movable_link_component(model, seed_joint_name, adj):
    """Links rigidly/kinematically connected to `seed_joint_name`'s child, walking
    through joints but STOPPING at the root hub (base/case) — so two gear trains that
    only share the common base plate are still seen as distinct components. Returns a
    set of link names."""
    seed = next((j for j in model.joints if j.name == seed_joint_name), None)
    if seed is None:
        return set()
    root = model.root_link
    seen = {seed.child}
    stack = [seed.child]
    while stack:
        cur = stack.pop()
        for nb in adj.get(cur, ()):
            if nb in seen or nb == root:
                continue
            seen.add(nb)
            stack.append(nb)
    return seen


def _roles(model) -> dict:
    """Deterministic "what can move and what can't" contract, derived from the URDF
    tree — NOT guessed by the LLM. The models this pipeline builds are hub-and-spoke
    (every gear hangs off the base plate; gears couple by TOOTH MESH, i.e. geometry,
    not by a kinematic joint), so a pure joint-tree walk finds no transmission. We
    therefore treat every non-driver movable joint as transmission (observed, never
    actuated) unless it belongs to a SEPARATE component with its OWN driver.

    Returns {driver_input, transmission[], fixed_structural[], free_unrelated[],
    output_joint, propagation_path[]}."""
    movable = [j for j in model.joints if j.type in _MOVABLE]
    fixed_structural = [j.name for j in model.joints if j.type == "fixed"]
    driver = _infer_driver(model)
    if not movable:
        return {"driver_input": None, "transmission": [], "output_joint": None,
                "fixed_structural": fixed_structural, "free_unrelated": [],
                "propagation_path": []}

    adj = _adjacency(model)
    driver_comp = _movable_link_component(model, driver, adj) if driver else set()

    transmission, free_unrelated = [], []
    for j in movable:
        if j.name == driver:
            continue
        # A joint is "free_unrelated" only if it is in a DIFFERENT connected
        # component AND that component has its own driver-like joint (a genuine
        # second subsystem). Otherwise it is downstream of the mesh -> transmission.
        if j.child in driver_comp or j.parent in driver_comp:
            transmission.append(j.name)
        elif _has_own_driver(model, j, adj):
            free_unrelated.append(j.name)
        else:
            transmission.append(j.name)

    output_joint = _output_joint(model, driver, transmission, adj)
    path = [driver] + [t for t in transmission if t != output_joint]
    if output_joint and output_joint != driver:
        path.append(output_joint)
    return {"driver_input": driver, "transmission": transmission,
            "output_joint": output_joint, "fixed_structural": fixed_structural,
            "free_unrelated": free_unrelated, "propagation_path": path}


def _has_own_driver(model, joint, adj) -> bool:
    """True if `joint` sits in a movable-link component (excluding the root hub) that
    contains a driver-named link -> it is its own subsystem, not downstream noise."""
    comp = _movable_link_component(model, joint.name, adj)
    for ln in comp:
        if _DRIVER_HINT.search(ln):
            return True
    return False


def _output_joint(model, driver, transmission, adj) -> str | None:
    """The mechanism's OUTPUT: the movable joint motion should ultimately reach.
    Prefer an output/hand/escape-named joint; else the transmission joint whose link
    is topologically farthest from the driver; else the last transmission joint."""
    if not transmission:
        return driver
    named = [t for t in transmission
             if _OUTPUT_HINT.search(_joint_child(model, t) or "")
             or _OUTPUT_HINT.search(t)]
    if named:
        return _farthest(model, driver, named, adj) or named[-1]
    return _farthest(model, driver, transmission, adj) or transmission[-1]


def _joint_child(model, joint_name) -> str | None:
    j = next((jj for jj in model.joints if jj.name == joint_name), None)
    return j.child if j else None


def _farthest(model, driver, candidates, adj) -> str | None:
    """Of `candidates` (joint names), the one whose child link is graph-farthest from
    the driver's child link (BFS hop count over the link graph). Ties -> first."""
    seed = _joint_child(model, driver)
    if not seed:
        return None
    dist = {seed: 0}
    q = [seed]
    while q:
        cur = q.pop(0)
        for nb in adj.get(cur, ()):
            if nb not in dist:
                dist[nb] = dist[cur] + 1
                q.append(nb)
    best, best_d = None, -1
    for t in candidates:
        child = _joint_child(model, t)
        d = dist.get(child, -1)
        if d > best_d:
            best, best_d = t, d
    return best


def _subsystems(model) -> list[dict]:
    """Group movable joints into INDEPENDENT functional subsystems for multi-test.

    CAUTION: in the models this pipeline builds, gears couple by TOOTH MESH
    (geometry), which is NOT a URDF joint — so topological connectivity CANNOT tell a
    single gear train apart from two independent ones (every gear hangs off the base
    plate). Splitting by topology therefore over-fragments one power train into a
    "subsystem" per gear, which is wrong. The only reliable signal for a genuinely
    separate subsystem is a SECOND explicit power input: a distinct joint the user
    would drive (driver flag, or a crank/handle/winder/input-named link). So:

    - Find the driver-like joints (explicit `driver=true`, else name heuristic).
    - If there are >=2 distinct ones, each anchors one subsystem; every other movable
      joint is assigned to the NEAREST driver over the link graph (its transmission).
    - If there is 0 or 1, the whole thing is ONE subsystem (today's behavior) — the
      conservative default that keeps a monolith as a single driven test.

    Returns [{id, driver, joints[], transmission[], output_joint}]."""
    movable = [j for j in model.joints if j.type in _MOVABLE]
    if not movable:
        return []
    adj = _adjacency(model)

    drivers = [j for j in movable if getattr(j, "driver", False)]
    if not drivers:
        drivers = [j for j in movable
                   if _DRIVER_HINT.search(j.child) or _DRIVER_HINT.search(j.parent)]
    # Dedup driver joints that sit on the SAME shaft/component (one real input, not N).
    drivers = _distinct_drivers(model, drivers, adj)

    if len(drivers) < 2:
        # One subsystem: the whole mechanism, one inferred driver.
        driver = _infer_driver(model)
        names = [j.name for j in movable]
        transmission = [n for n in names if n != driver]
        out = _output_joint(model, driver, transmission, adj)
        return [{"id": _sub_id(model, movable, 0), "driver": driver,
                 "joints": names, "transmission": transmission, "output_joint": out}]

    # >=2 real inputs: assign every movable joint to its nearest driver.
    buckets = {d.name: [d.name] for d in drivers}
    for j in movable:
        if j.name in buckets:
            continue
        nearest = _nearest_driver(model, j, drivers, adj)
        buckets[nearest].append(j.name)

    subs = []
    for idx, d in enumerate(drivers):
        names = buckets[d.name]
        transmission = [n for n in names if n != d.name]
        out = _output_joint(model, d.name, transmission, adj)
        grp = [j for j in movable if j.name in names]
        subs.append({"id": _sub_id(model, grp, idx), "driver": d.name,
                     "joints": names, "transmission": transmission,
                     "output_joint": out})
    return subs


def _distinct_drivers(model, drivers, adj):
    """Collapse driver joints whose child links are in the same movable component
    (they drive one shaft) down to a single representative, so a crank+its shaft
    don't count as two inputs."""
    out, seen = [], []
    for d in drivers:
        comp = _movable_link_component(model, d.name, adj) | {d.child}
        if any(comp & s for s in seen):
            continue
        seen.append(comp)
        out.append(d)
    return out


def _nearest_driver(model, joint, drivers, adj):
    """Name of the driver joint graph-closest to `joint` (BFS hops over links)."""
    seed = joint.child
    dist = {seed: 0}
    q = [seed]
    while q:
        cur = q.pop(0)
        for nb in adj.get(cur, ()):
            if nb not in dist:
                dist[nb] = dist[cur] + 1
                q.append(nb)
    best, best_d = drivers[0].name, 10**9
    for d in drivers:
        dd = dist.get(d.child, 10**9)
        if dd < best_d:
            best, best_d = d.name, dd
    return best


def _infer_driver_in(model, joints) -> str | None:
    """Pick the driver of a joint subset: explicit driver flag, else name heuristic,
    else the joint whose parent link is nearest the root, else the first."""
    if not joints:
        return None
    for j in joints:
        if getattr(j, "driver", False):
            return j.name
    for j in joints:
        if _DRIVER_HINT.search(j.child) or _DRIVER_HINT.search(j.parent):
            return j.name
    root = model.root_link
    for j in joints:
        if j.parent == root:
            return j.name
    return joints[0].name


def _sub_id(model, group, idx) -> str:
    """A stable, human id for a subsystem: its driver-ish link stem, else index."""
    drv = _infer_driver_in(model, group)
    child = _joint_child(model, drv) if drv else None
    if child:
        stem = re.sub(r"(_shaft|_pin|_bearing|_block|_gear|_wheel).*$", "", child)
        stem = re.sub(r"[^a-z0-9]+", "_", stem.lower()).strip("_")
        if stem:
            return stem
    return f"sub{idx}"


def _gateway():
    """maker2's gateway creds, for the selector/designer LLM calls. Reads the same
    env-configurable Settings the rest of maker2 uses (FREECAD_AI_BASE_URL etc.),
    defaulting to the local 8313 proxy."""
    try:
        from maker2.config import Settings
        s = Settings.load()
        return {"base_url": s.base_url, "api_key": s.api_key, "model": s.model}
    except Exception:
        return {"base_url": None, "api_key": None, "model": None}


def _plan(task: str, model) -> list[dict] | None:
    """Ask the strategy selector how to test this. Returns its `tests` list (each
    driven test annotated with its subsystem's driver/transmission/output so the
    designer targets the right input), or None if the planner/gateway is unavailable
    (caller then does a static test)."""
    try:
        import strategy_selector
        gw = _gateway()
        robot_info = _robot_info(model)
        d = strategy_selector.decide(task, robot_info,
                                     base_url=gw["base_url"], api_key=gw["api_key"],
                                     model=gw["model"])
        print(f"[physics] strategy: {d.get('strategy')} | "
              f"backend={d.get('sim_backend')} | dof={d.get('actuated_dof_count')} "
              f"| tests={[t.get('name') for t in d.get('tests', [])]}")
        tests = d.get("tests") or [{"name": d.get("strategy", "test"),
                                    "goal": task, "strategy": d.get("strategy")}]
        return _attach_subsystems(tests, robot_info.get("subsystems") or [])
    except Exception as e:
        print(f"[physics] planner unavailable ({e}); static stability test only")
        return None


def _attach_subsystems(tests: list[dict], subs: list[dict]) -> list[dict]:
    """Give each DRIVEN test the subsystem it exercises (so _design_spec drives that
    subsystem's input), and ensure EVERY subsystem is covered — backfill a driven test
    for any subsystem the planner didn't name. A single-subsystem model is unchanged
    except its one driven test gets the subsystem's driver/output hints."""
    if not subs:
        return tests
    driven = [t for t in tests if t.get("strategy") == "driven_mechanism"]
    # Map each driven test to a subsystem by name match, else round-robin by order.
    covered = set()
    for i, t in enumerate(driven):
        sub = _match_sub(t, subs) or subs[min(i, len(subs) - 1)]
        _tag_test(t, sub)
        covered.add(sub["id"])
    # Backfill uncovered subsystems (only meaningful when there are >=2 subsystems).
    for sub in subs:
        if sub["id"] in covered:
            continue
        if len(subs) == 1 and driven:
            continue  # single subsystem already has a driven test
        t = {"name": sub["id"], "goal": f"drive the {sub['id']} subsystem",
             "strategy": "driven_mechanism"}
        _tag_test(t, sub)
        tests.append(t)
        covered.add(sub["id"])
    return tests


def _match_sub(test: dict, subs: list[dict]) -> dict | None:
    name = (test.get("name") or "").lower()
    for s in subs:
        if s["id"] and (s["id"] in name or name in s["id"]):
            return s
        if s.get("driver") and s["driver"].lower() in name:
            return s
    return None


def _tag_test(test: dict, sub: dict) -> None:
    test["subsystem"] = sub["id"]
    test.setdefault("driver", sub.get("driver"))
    test.setdefault("transmission", sub.get("transmission"))
    test.setdefault("output_joint", sub.get("output_joint"))


def _design_spec(task: str, model, test: dict) -> dict:
    """scenario_designer -> a spec for this test. For a driven test, ENFORCE the role
    map deterministically (the gateway often ignores schema keys / the role contract):
    drive ONLY the true driver_input, watch the transmission joints, and declare the
    propagation path to the output."""
    from scenario_designer import design
    gw = _gateway()
    spec = design(task, _robot_info(model), test,
                  base_url=gw["base_url"], api_key=gw["api_key"], model=gw["model"])
    if test.get("strategy") == "driven_mechanism":
        roles = _roles(model)
        # A test may target a specific subsystem's driver (E-MULTI); else the model's.
        driver = test.get("driver") or roles.get("driver_input") or _infer_driver(model)
        drive = spec.get("drive") or {}
        # Override the input joint: the LLM must NOT drive a transmission/unrelated joint.
        if drive.get("input_joint") != driver:
            if drive.get("input_joint"):
                print(f"[physics] corrected input joint "
                      f"'{drive.get('input_joint')}' -> '{driver}' (role: driver_input)")
            drive["input_joint"] = driver
        drive.setdefault("mode", "velocity")
        if not drive.get("target_velocity"):
            drive["target_velocity"] = 5.0
        if not drive.get("duration_s"):
            drive["duration_s"] = 3.0
        if drive.get("self_collision") is None:
            drive["self_collision"] = True
        if not drive.get("min_watched_travel"):
            drive["min_watched_travel"] = 0.05
        # Watch the transmission joints (NOT all movable, NOT the input, NOT unrelated).
        transmission = test.get("transmission") or roles.get("transmission") or []
        if not transmission:
            transmission = [j.name for j in model.joints
                            if j.type in _MOVABLE and j.name != driver]
        drive["watch_joints"] = [t for t in transmission if t != driver]
        # Declare the propagation path + output so the diagnoser can check REACH.
        if not drive.get("output_joint"):
            drive["output_joint"] = (test.get("output_joint")
                                     or roles.get("output_joint"))
        if not drive.get("propagation_path"):
            drive["propagation_path"] = roles.get("propagation_path") or []
        print(f"[physics] driving '{driver}' -> output '{drive.get('output_joint')}', "
              f"watching {len(drive['watch_joints'])} transmission joints")
        spec["drive"] = drive
        spec["fixed_base"] = True
    return spec


def _summarize(test: dict, m: dict) -> str:
    if m.get("test_kind") == "driven_mechanism":
        reach = ("" if m.get("output_reached") is None
                 else " reached output" if m.get("output_reached")
                 else " but output DEAD")
        return (f"{test.get('name','drive')}: {m.get('verdict')} — input turned "
                f"{m.get('input_travel')} rad, {m.get('moved_count')}/"
                f"{m.get('watched_count')} downstream joints moved" + reach
                + (" (JAMMED/EXPLODED)" if m.get('exploded') else ""))
    return (f"{test.get('name','stability')}: {m.get('verdict')} — settled z="
            f"{m.get('end_z')} tilt {m.get('max_tilt_deg')}deg "
            f"drift {m.get('max_drift')}m")


def run_physics(urdf_path: str, task: str, run_dir: str) -> dict:
    """Category-aware physics on maker2's URDF. Returns the same shape the UI reads:
    {passed, verdict, summary, metrics, frames_dir}. `metrics` is the FINAL/primary
    test; `summary` spans all tests run."""
    import run_scenario_pybullet as pyb
    from diagnose import diagnose_physics, encode_mp4

    model = _load_model(run_dir)
    tests = _plan(task, model) if model is not None else None

    # No model or no plan -> the legacy single static stability test.
    if not tests:
        out = str(Path(run_dir) / "physics" / "test_0")
        res = pyb.run(urdf_path, _static_spec(), out, task or "settle stably")
        m = res.get("metrics", {})
        video = None
        if res.get("frames_dir"):
            mp4 = encode_mp4(res["frames_dir"], os.path.join(out, "model.mp4"))
            if mp4:
                video = "physics/test_0/model.mp4"
        entry = {"name": "stability", "strategy": "static_stability",
                 "verdict": m.get("verdict"), "metrics": m,
                 "summary": _summarize({"name": "stability"}, m),
                 "frames_dir": res.get("frames_dir"), "video": video}
        return {"passed": m.get("verdict") == "PASS", "verdict": m.get("verdict", "FAIL"),
                "summary": entry["summary"], "metrics": m,
                "frames_dir": res.get("frames_dir"), "video": video,
                "tests": [entry]}

    # Fan out design->sim->diagnose per test. Design (LLM) + diagnose (VLM) are
    # HTTP-bound; the sim runs in a SUBPROCESS (PyBullet's DIRECT client is process-
    # global, so two in one process would collide). Results are placed back by index
    # so physics/test_i and the UI's ?test=i stay stable regardless of finish order.
    gw = _gateway()
    from concurrent.futures import ThreadPoolExecutor, as_completed
    try:
        from maker2.config import Settings
        max_workers = min(len(tests), max(1, Settings.load().max_workers))
    except Exception:
        max_workers = min(len(tests), 4)
    log_lock = threading.Lock()
    results: list = [None] * len(tests)

    def _work(i, test):
        return i, _run_one_test(i, test, urdf_path, task, model, run_dir, gw, log_lock)

    if max_workers <= 1 or len(tests) == 1:
        for i, test in enumerate(tests):
            results[i] = _run_one_test(i, test, urdf_path, task, model, run_dir, gw, log_lock)
    else:
        with ThreadPoolExecutor(max_workers=max_workers) as ex:
            futs = [ex.submit(_work, i, test) for i, test in enumerate(tests)]
            for f in as_completed(futs):
                i, entry = f.result()
                results[i] = entry

    per_test = [r for r in results if r is not None]
    primary = next((t for t in per_test
                    if (t.get("metrics") or {}).get("test_kind") == "driven_mechanism"),
                   per_test[-1] if per_test else None)

    # Aggregate the per-test verdicts into one overall verdict + a per-subsystem cause
    # map + fault localization (what a hierarchy boss loop consumes to route a rebuild).
    agg = _aggregate(task, per_test, gw)
    return {"passed": agg["passed"], "verdict": "PASS" if agg["passed"] else "FAIL",
            "summary": agg["summary"],
            "metrics": (primary or {}).get("metrics", {}),
            "cause": (primary or {}).get("cause", "none"),
            "reason": (primary or {}).get("reason", ""),
            "frames_dir": (primary or {}).get("frames_dir"),
            "video": (primary or {}).get("video"),
            "cause_map": agg["cause_map"],
            "blamed_subs": agg["blamed_subs"],
            "blamed_kind": agg["blamed_kind"],
            "tests": per_test}


def _run_sim(urdf_path, spec, out_base, task, log):
    """Run ONE pybullet sim in a SUBPROCESS and read back its sim_result.json.

    PyBullet's DIRECT client is process-global, so concurrent in-process sims collide;
    a subprocess gives each sim its own client + true parallelism. Only the sim is
    subprocessed — design/diagnose stay in-thread. Falls back to an in-process run if
    the subprocess fails (so a broken subprocess never dead-ends a test)."""
    import run_scenario_pybullet as pyb
    out = Path(out_base)
    out.mkdir(parents=True, exist_ok=True)
    spec_json = out / "spec.json"
    spec_json.write_text(json.dumps(spec))
    try:
        r = subprocess.run(
            [sys.executable, pyb.__file__, "--urdf", urdf_path,
             "--spec", str(spec_json), "--out", out_base, "--task", task],
            capture_output=True, text=True, timeout=300,
            cwd=str(_ROOT))
        result_path = out / "sim_result.json"
        if r.returncode == 0 and result_path.exists():
            return json.loads(result_path.read_text())
        log(f"[physics] sim subprocess rc={r.returncode}; "
            f"stderr tail: {(r.stderr or '')[-200:]}")
    except Exception as e:
        log(f"[physics] sim subprocess failed ({e}); running in-process")
    # Fallback: run in this process (serializes with any other in-process sim).
    return pyb.run(urdf_path, spec, out_base, task)


def _run_one_test(i, test, urdf_path, task, model, run_dir, gw, log_lock):
    """Design -> sim (subprocess) -> diagnose for ONE test, with the in-place
    framing/scenario retry loop. Returns the per-test entry. Thread-safe: it only
    touches its own physics/test_i dir and prints under `log_lock`."""
    from diagnose import diagnose_physics, encode_mp4

    def log(msg):
        with log_lock:
            print(msg)

    try:
        if test.get("strategy") == "driven_mechanism":
            spec = _design_spec(task, model, test)
        elif test.get("strategy") in ("static_stability", None):
            spec = _design_spec(task, model, test) if model is not None else _static_spec()
            spec.setdefault("drive", None)
        else:
            log(f"[physics] test '{test.get('name')}' strategy "
                f"'{test.get('strategy')}' not runnable here; stability proxy")
            spec = _static_spec()
    except Exception as e:
        log(f"[physics] designer failed for '{test.get('name')}' ({e}); static")
        spec = _static_spec()

    out_base = str(Path(run_dir) / "physics" / f"test_{i}")
    driven = bool(spec.get("drive"))

    diagnosis = {"verdict": None, "cause": "none", "reason": ""}
    res = None
    m = {}
    for attempt in range(3 if driven else 1):
        res = _run_sim(urdf_path, spec, out_base,
                       f"{task} :: {test.get('name','')}", log)
        m = res.get("metrics", {})
        if not driven:
            break
        robot_info = _robot_info(model) if model is not None else {"name": "robot"}
        diagnosis = diagnose_physics(task, robot_info, spec, m,
                                     res.get("frames_dir", ""),
                                     frames_dirs=res.get("frames_dirs"),
                                     base_url=gw["base_url"], api_key=gw["api_key"],
                                     model=gw["model"])
        log(f"[physics] test {i} attempt {attempt}: VLM verdict={diagnosis['verdict']} "
            f"cause={diagnosis['cause']} :: {diagnosis['reason'][:100]}")
        if diagnosis["verdict"] == "pass":
            break
        if diagnosis["cause"] == "framing" and attempt < 2:
            d = spec.setdefault("drive", {}) or {}
            d["cam_dist_scale"] = d.get("cam_dist_scale", 1.0) * 0.6
            d["cam_pitch"] = -60
            log("[physics] framing fault -> reframing camera, re-recording")
            continue
        if diagnosis["cause"] == "scenario" and attempt < 2 and model is not None:
            try:
                from scenario_designer import revise
                spec = revise(task, _robot_info(model), spec, diagnosis["reason"],
                              base_url=gw["base_url"], api_key=gw["api_key"],
                              model=gw["model"])
                spec["fixed_base"] = True
                log("[physics] scenario fault -> revised test spec, re-simulating")
                continue
            except Exception as e:
                log(f"[physics] scenario revise failed ({e})")
                break
        break   # structure fault, or retries exhausted -> escalate

    # Encode a video PER camera; model.mp4 stays the primary-camera alias the UI
    # already fetches, and videos{} exposes every angle for the multi-view UI.
    video = None
    videos = {}
    if res:
        fdirs = res.get("frames_dirs") or (
            {"iso": res["frames_dir"]} if res.get("frames_dir") else {})
        primary_cam = next(iter(fdirs), None)
        for cam, fdir in fdirs.items():
            mp4 = encode_mp4(fdir, os.path.join(out_base, f"{cam}.mp4"))
            if mp4:
                videos[cam] = f"physics/test_{i}/{cam}.mp4"
        if primary_cam and primary_cam in videos:
            alias = encode_mp4(fdirs[primary_cam], os.path.join(out_base, "model.mp4"))
            if alias:
                video = f"physics/test_{i}/model.mp4"
                log(f"[physics] test {i} videos {list(videos)} -> {video}")

    final_verdict = ("PASS" if diagnosis["verdict"] == "pass"
                     else "FAIL" if diagnosis["verdict"] == "fail"
                     else m.get("verdict", "FAIL"))
    return {"name": test.get("name"), "strategy": test.get("strategy"),
            "subsystem": test.get("subsystem"),
            "verdict": final_verdict, "metrics": m,
            "cause": diagnosis["cause"], "reason": diagnosis["reason"],
            "frames_dir": res.get("frames_dir") if res else None,
            "output_reached": m.get("output_reached"),
            "propagation": (spec.get("drive") or {}).get("propagation_path"),
            "summary": _summarize(test, m), "video": video, "videos": videos}


def _aggregate(task, per_test, gw):
    """Deterministic aggregation of per-test verdicts (LLM optional, only for prose)."""
    try:
        from aggregate import aggregate_verdicts
        return aggregate_verdicts(task, per_test, gw=gw)
    except Exception as e:
        # Fallback: never let aggregation break the run.
        passed = all(t.get("verdict") == "PASS" for t in per_test)
        return {"passed": passed,
                "summary": " | ".join(t.get("summary", "") for t in per_test),
                "cause_map": {}, "blamed_subs": [], "blamed_kind": None}


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--task", default="")
    ap.add_argument("--out", default=".", help="run_dir (holds kinematic_model.json)")
    a = ap.parse_args()
    print(json.dumps(run_physics(a.urdf, a.task, a.out), indent=2))
