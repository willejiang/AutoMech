#!/usr/bin/env python3
"""Physics evaluation of a maker2 URDF — category-aware, not a fool stand-still.

The old version dropped the model from 0.5 m and PASSed if it didn't topple — a
solid brick passed, and a gearbox was never actually driven. This version routes
through the evaluator's planner:

  _environments(task, model) -> decide WHICH tests to run DETERMINISTICALLY from the
                               model (one driven test per drivable subsystem, or a
                               stand-still test) — no task-TYPE enum / no selector LLM
  environment_designer.design_environment -> ONE call: pick the sim environment +
                               emit the scenario spec for it (a `drive` block for a
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
            # Parts that get a hinge in the sim; the trajectory keys are "<part>_spin".
            "spin_links": [l.name for l in model.links
                           if getattr(l, "dof", "fixed") in ("spin", "free")],
            # What each dof=fixed part RIDES ON. Such a part has no joint of its own, so
            # its motion is only readable through its carrier. Without this the designer
            # cannot tell which joint a clock hand corresponds to, picks a plausible gear
            # nearby, and reports a ratio between two parts the user never sees.
            "carried_parts": {l.name: getattr(l, "mount", "")
                              for l in model.links
                              if getattr(l, "dof", "fixed") == "fixed"
                              and getattr(l, "mount", "")},
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
        return {"base_url": s.base_url, "api_key": s.api_key, "model": s.model,
                "web": bool(getattr(s, "enable_reference_tools", False))}
    except Exception:
        return {"base_url": None, "api_key": None, "model": None, "web": False}


def _environments(task: str, model) -> list[dict] | None:
    """Decide WHICH tests to run — deterministically, from the model (no task-TYPE
    enum / no selector LLM). Each independent subsystem with a drivable input gets its
    own driven test (tagged with that subsystem's driver/transmission/output so
    _design_spec drives the right input); a model with no drivable input gets one
    stand-still test. The per-test SCENARIO (including the sim environment) is authored
    later by environment_designer inside _design_spec. Returns a list of test
    descriptors, or None if we can't introspect the model (caller does a static test)."""
    try:
        subs = _subsystems(model) or []
        tests: list[dict] = []
        for sub in subs:
            if not sub.get("driver"):
                continue
            t = {"name": sub["id"], "goal": f"drive the {sub['id']} subsystem"}
            _tag_test(t, sub)
            tests.append(t)
        if tests:
            print(f"[physics] {len(tests)} driven subsystem test(s): "
                  f"{[t['name'] for t in tests]}")
            return tests
        # No drivable subsystem — fall back to a single stand-still test. (A genuine
        # stool/statue; the environment_designer will keep drive=null.)
        driver = _infer_driver(model)
        if driver:
            t = {"name": "drive", "goal": task, "driver": driver}
            print(f"[physics] 1 driven test on inferred input '{driver}'")
            return [t]
        print("[physics] no drivable joint -> single stand-still test")
        return [{"name": "stability", "goal": task}]
    except Exception as e:
        print(f"[physics] could not introspect model ({e}); static stability test only")
        return None


def _tag_test(test: dict, sub: dict) -> None:
    test["subsystem"] = sub["id"]
    test.setdefault("driver", sub.get("driver"))
    test.setdefault("transmission", sub.get("transmission"))
    test.setdefault("output_joint", sub.get("output_joint"))


_METRICS_TIMEOUT = 30
# The support test runs in parallel with the designer call and the simulation, so this is
# only a backstop against a wedged convex decomposition, not a normal wait: by the time it
# is joined it has already had those seconds to work in.
_SUPPORT_TIMEOUT = 900

# Names a metrics check uses when it is reporting on ITSELF rather than on the machine.
_SELF_REPORT_CHECKS = ("joints_present", "joint_missing", "missing_joints")


def _check_is_broken(metrics: dict) -> bool:
    """True if the functional check failed because it could not RUN, not because the
    machine failed it. Such a result must never count against the CAD: the measurement
    never happened."""
    checks = (metrics or {}).get("functional_checks") or []
    if not checks:
        return False
    for c in checks:
        if c.get("passed"):
            continue
        name = str(c.get("name") or "").lower()
        detail = str(c.get("detail") or "").lower()
        if name in _SELF_REPORT_CHECKS or "missing" in detail or "not found" in detail:
            return True
    return False


def _traj_keys(out_base: str) -> list:
    """Joint keys actually present in this run's trajectory, to hand back to the designer."""
    try:
        p = Path(out_base) / "trajectory.json"
        return list((json.loads(p.read_text(encoding="utf-8")).get("joints") or {}).keys())
    except Exception:
        return []


def _run_metrics_code(designed, out_base: str, *, log_fn=print) -> list:
    """Run the designer's `metrics_code` over this run's trajectory, in a subprocess.

    Returns the list of checks, or [] when there is nothing to run (no code, no
    trajectory) or the code failed — a broken check is a broken CHECK, never a verdict
    about the machine, so it must not turn into a functional failure."""
    code = ((designed or {}).get("metrics_code") or "").strip() if isinstance(designed, dict) else ""
    if not code:
        return []
    base = Path(out_base)
    traj = base / "trajectory.json"
    result = base / "sim_result.json"
    if not traj.exists():
        log_fn("[physics] metrics_code skipped: this run recorded no trajectory")
        return []
    code_path = base / "metrics.py"
    out_path = base / "metrics_result.json"
    try:
        code_path.write_text(code, encoding="utf-8")
        r = subprocess.run(
            [sys.executable, str(Path(_ROOT) / "evaluator" / "_metrics_runner.py"),
             str(code_path), str(traj), str(result), str(out_path)],
            capture_output=True, text=True, timeout=_METRICS_TIMEOUT)
        if not out_path.exists():
            log_fn(f"[physics] metrics_code produced no result "
                   f"({(r.stderr or r.stdout or '').strip()[-200:]})")
            return []
        payload = json.loads(out_path.read_text(encoding="utf-8"))
    except subprocess.TimeoutExpired:
        log_fn(f"[physics] metrics_code timed out after {_METRICS_TIMEOUT}s; ignored")
        return []
    except Exception as e:
        log_fn(f"[physics] metrics_code could not run ({type(e).__name__}: {e}); ignored")
        return []
    if not payload.get("ok"):
        log_fn(f"[physics] metrics_code error: {payload.get('error')}")
        return []
    return payload.get("checks") or []


def _design_spec(task: str, model, test: dict) -> dict:
    """environment_designer -> a spec for this test (ONE call that picks the sim
    environment + emits the scenario). For a DRIVEN test we still ENFORCE the role map
    deterministically on top (the gateway often ignores schema keys / the role
    contract): drive ONLY the true driver_input, watch the transmission joints, and
    declare the propagation path to the output. `test` may carry a `subsystem` tag."""
    from environment_designer import design_environment, revise_environment
    gw = _gateway()
    subsystem = None
    if test.get("subsystem"):
        subsystem = {"id": test.get("subsystem"), "driver": test.get("driver"),
                     "transmission": test.get("transmission"),
                     "output_joint": test.get("output_joint")}
    revise = test.get("revise")
    if revise and revise.get("prev"):
        # Re-design after a TEST-side diagnosis (camera/scenario): feed the previous spec
        # + the failure back so the designer fixes the observation/drive, camera included.
        spec = revise_environment(task, _robot_info(model), revise["prev"],
                                  revise.get("feedback", ""),
                                  base_url=gw["base_url"], api_key=gw["api_key"],
                                  model=gw["model"])
    else:
        spec = design_environment(task, _robot_info(model), subsystem=subsystem,
                                  base_url=gw["base_url"], api_key=gw["api_key"],
                                  model=gw["model"], web=gw.get("web", False))
    # A driven test is now signalled by the model having a drivable input (the caller
    # decides), not by a strategy enum — enforce the driver iff this test targets one.
    driver_present = bool(test.get("driver") or _roles(model).get("driver_input"))
    if driver_present and (test.get("strategy") == "driven_mechanism"
                           or test.get("subsystem") or spec.get("drive")):
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


def blame_faults(phys: dict, precheck_report=None, plan=None) -> dict:
    """Attribute physics/geometry faults to specific subassemblies for per-manager
    routing (answers "which manager is to blame"). Returns {sub_id: [reason, ...]}.

    Sources:
      - FLOATING/EXPLODING: metrics["displaced_parts"] (a part that drifted far from its
        settle pose — unsupported, or a structure part wrongly marked spin so it flew
        apart). Each entry's body name is the namespaced link `<sub_id>_<part>`.
      - OVERLAP: precheck_report part_overlap/aabb_overlap violations (already carry
        sub_id + the two parts).
    Sub ids come from the plan; a part name is mapped to a sub by prefix (the assembler
    namespaces links `<sub_id>_<name>`), longest-prefix-wins so nested ids don't collide."""
    out: dict = {}
    sub_ids = sorted((s.id for s in plan.subassemblies), key=len, reverse=True) \
        if plan is not None else []

    def _sub_of(part: str) -> str:
        for sid in sub_ids:                       # longest id first
            if part == sid or part.startswith(f"{sid}_"):
                return sid
        return ""

    m = (phys or {}).get("metrics", {}) if isinstance(phys, dict) else {}
    for e in (m.get("displaced_parts") or []):
        part = e.get("part", "")
        sid = _sub_of(part)
        if not sid:
            continue
        bare = part[len(sid) + 1:] if part.startswith(f"{sid}_") else part
        out.setdefault(sid, []).append(
            f"part '{bare}' drifted {e.get('disp_mm')} mm from its settle pose — it is "
            f"FLOATING/unsupported or was marked spin when it should be fixed (a "
            f"structure part that flew apart). Make it 'fixed' and/or place it on real "
            f"support.")

    for v in getattr(precheck_report, "violations", []) or []:
        if getattr(v, "kind", "") in ("part_overlap", "aabb_overlap"):
            sid = getattr(v, "sub_id", "") or ""
            if sid:
                out.setdefault(sid, []).append(
                    f"rigid OVERLAP ({getattr(v, 'value', 0):.0%}): {getattr(v, 'detail', '')}")
    return out


def _run_physics_mujoco(urdf_path: str, task: str, run_dir: str, settings,
                        iteration: int | None = None, log_fn=print) -> dict:
    """Pure-contact MuJoCo physics. Builds the MJCF from the run's model, runs the
    MuJoCo scenario runner (subprocess, with in-process fallback), and returns the
    same result shape run_physics does.

    Scenario design + diagnosis: even though pure contact needs no motor, the LLM
    environment_designer is still used to DECIDE WHAT THE TEST CHECKS — which input the
    driver PART represents, which downstream joints/parts should move, and the pass
    criteria — and the VLM diagnose_physics is run afterwards to CLASSIFY the failure
    (structure / interface / scenario / camera). This is what surfaces the "designed a
    test, ran it, analyzed the result" value in the UI. Falls back gracefully to the
    hardcoded contact spec if the designer/gateway is unavailable."""
    from diagnose import encode_mp4
    from maker2 import mjcf_builder
    from maker2.model import RunContext

    model = _load_model(run_dir)
    if model is None:
        return {"passed": False, "verdict": "FAIL",
                "summary": "no model to simulate", "metrics": {}, "tests": []}

    # Build the MJCF next to the URDF. meshes live in run_dir/meshes.
    ctx = RunContext(project_slug=model.name or "assembly", run_dir=run_dir,
                     urdf_path=urdf_path, meshes_dir=os.path.join(run_dir, "meshes"),
                     logs_dir=run_dir,
                     model_json_path=os.path.join(run_dir, "kinematic_model.json"))
    metrics_side: dict = {}
    try:
        mjcf = mjcf_builder.build_mjcf(model, ctx, settings=settings,
                                       metrics=metrics_side, log_fn=log_fn)
    except Exception as e:
        return {"passed": False, "verdict": "FAIL",
                "summary": f"MJCF build failed: {e}", "metrics": {}, "tests": []}

    # One directory PER ITERATION: a fixed "mujoco" dir made every iteration overwrite
    # the previous model.mp4, so only the last run had a video and there was no way to
    # watch the machine get better (or worse) across the loop.
    sub = "mujoco" if iteration is None else f"mujoco_{iteration}"
    out_base = str(Path(run_dir) / "physics" / sub)

    # STAGE 1 (SUPPORT), merged into the stability verdict below. The settle test measures
    # whether the machine holds together as ASSEMBLED — but a part hung on a `mount=` label
    # it never touches is welded to that mount in the real MJCF, so it sits there happily
    # and the settle sees nothing wrong. The support test dissolves every weld and lets
    # gravity answer the question the welds hide: what is actually held by geometry?
    #
    # RUN IT IN THE BACKGROUND. It shares no data with the scenario design or the sim —
    # its verdict is only merged into `stability` once both are done — and it is the
    # slowest thing in the iteration: it builds its own MJCF from the CONVEX
    # decomposition (SDF drops contacts in a full assembly; see build_support_mjcf), and
    # decomposing a 14-part movement takes minutes while the designer call and the sim
    # take seconds. Overlapping them hides nearly all of that cost.
    support_fell: list = []
    _support_err: list = []

    def _run_support():
        try:
            from .support_test import support_faults
            support_fell.extend(support_faults(model, ctx, settings=settings,
                                               log_fn=log_fn))
        except Exception as e:
            _support_err.append(f"{type(e).__name__}: {e}")

    _support_thread = threading.Thread(target=_run_support, daemon=True)
    _support_thread.start()

    # SCENARIO DESIGN: ask environment_designer what this test should check (best-effort;
    # the MuJoCo runner is contact-driven, so we only borrow the drive INTENT + criteria).
    designed = None
    driver_part = next((l.name for l in model.links
                        if getattr(l, "driver", False)), None)
    spec = {"duration_s": 4.0, "run_dir": run_dir, "drive": {"torque": 0.5}}
    try:
        designed = _design_spec(task, model, {"name": "drive", "driver": driver_part,
                                              "strategy": "driven_mechanism"})
        d = designed.get("drive") or {}
        # HONOUR THE DESIGNED DRIVE. _design_spec already settles `mode` and
        # `target_velocity` (defaulting to velocity @ 5 rad/s), but those were dropped
        # here and the runner fell back to a fixed 0.5 N.m for every machine. A watch
        # movement weighs milligrams, so that torque spun its input 3131 rad against a
        # 12 rad command — and because the pass test only asks "did it move", the runaway
        # scored as a PASS. Sweeping at the designed rate makes input travel a property
        # of the TEST rather than of how heavy the parts happen to be, which is also what
        # makes an output/input ratio meaningful.
        spec["drive"] = dict(spec.get("drive") or {})
        for _k in ("mode", "target_velocity", "torque"):
            if d.get(_k) is not None:
                spec["drive"][_k] = d[_k]
        spec["design"] = {
            "input_joint": d.get("input_joint"),
            "output_joint": d.get("output_joint"),
            "watch_joints": d.get("watch_joints", []),
            "propagation_path": d.get("propagation_path", []),
            "pass_criteria": designed.get("pass_criteria"),
            "environment": designed.get("environment") or designed.get("scenario"),
            "camera": designed.get("camera"),
        }
        if d.get("duration_s"):
            spec["duration_s"] = d["duration_s"]
        # The designer's own drive script, when the `drive` form could not express this
        # machine (anything wound and released). The runner falls back to `drive` if this
        # is empty or fails to compile, so passing it through is always safe.
        _cc = (designed.get("control_code") or "").strip()
        if _cc:
            spec["control_code"] = _cc
            log_fn(f"[physics] designer authored control_code ({len(_cc)} chars); "
                   f"it drives this run instead of drive.mode="
                   f"{spec['drive'].get('mode')}")
        log_fn(f"[physics] mujoco scenario designed: watch "
              f"{len(spec['design']['watch_joints'])} joints, output "
              f"'{spec['design'].get('output_joint')}'")
    except Exception as e:
        log_fn(f"[physics] mujoco scenario designer unavailable ({e}); "
              f"using default contact spec")

    # An output part = a spin/free link name-hinted as output, if any.
    out_link = next((l.name for l in model.links
                     if getattr(l, "dof", "fixed") in ("spin", "free")
                     and _OUTPUT_HINT.search(l.name)), None)
    if out_link:
        spec["drive"]["output_link"] = out_link

    # RUN -> DIAGNOSE, with a TEST-side retry loop. If the diagnosis blames the TEST
    # (camera can't see it / wrong joint driven) rather than the MODEL, the scenario
    # designer RE-DESIGNS the spec (including the camera) and we re-run — the machine is
    # unchanged, we just fix how we observe/drive it. A MODEL fault (structure/interface)
    # is NOT retried here: that needs the CAD redesigned, which the outer loop owns.
    _MAX_TEST_RETRIES = 2
    from diagnose import diagnose_physics
    res = m = video = None
    diagnosis = {"verdict": None, "cause": "none", "reason": ""}
    stability = {}
    for attempt in range(_MAX_TEST_RETRIES + 1):
        res = _run_sim_mujoco(mjcf, spec, out_base, task, log_fn=log_fn)
        m = dict(res.get("metrics", {}))
        stability = res.get("stability") or {}
        # The background support test must be in before its verdict is merged. By now it
        # has had the designer call and a full simulation to run in, so this usually
        # returns at once; the timeout only stops a wedged decomposition from hanging the
        # whole run (an absent support check is a missing check, never a machine verdict).
        if _support_thread is not None:
            _support_thread.join(timeout=_SUPPORT_TIMEOUT)
            if _support_thread.is_alive():
                log_fn(f"[support] still running after {_SUPPORT_TIMEOUT}s; "
                       f"continuing without its verdict")
            elif _support_err:
                log_fn(f"[support] support test unavailable ({_support_err[0]})")
            _support_thread = None
        # MERGE the support verdict into stage 1. The settle can only see parts that move;
        # an unsupported part is welded to a mount it never touches, so it never moves and
        # the settle passes it. A part nothing holds up is a stability failure regardless.
        if support_fell:
            stability = dict(stability)
            stability["unsupported_parts"] = [f.part for f in support_fell]
            stability["support_faults"] = [f.describe() for f in support_fell]
            stability["verdict"] = "FAIL"
        if metrics_side.get("contact_degraded"):
            m["contact_degraded"] = True
        if metrics_side.get("constrained_meshes"):
            m["constrained_meshes"] = metrics_side["constrained_meshes"]
        # Bore-fit faults are measured off the STLs while the MJCF is built — they exist
        # BEFORE a single sim step, and they are the defect that actually jams the train.
        # Carry them into the metrics the outer loop feeds back, so the agent is told
        # "this bore is smaller than the shaft" on iteration 0 instead of after several
        # rounds of chasing whatever the VLM happened to notice in the recording.
        if metrics_side.get("bore_fit_faults"):
            m["bore_fit_faults"] = metrics_side["bore_fit_faults"]
        # Solids that genuinely occupy the same space. Measured by boolean intersection
        # while the MJCF is built, so it needs no simulation and no VLM to notice it.
        if metrics_side.get("interferences"):
            m["interferences"] = metrics_side["interferences"]
        # FUNCTIONAL CHECK: did the machine do its JOB? Everything above establishes that
        # it held together and that parts moved — equally true of a train whose tooth
        # counts are wrong, a gripper that never closes, a ratchet that slips back. What
        # counts as success differs per mechanism, so the designer wrote it as code and it
        # runs here against the recorded trajectory.
        checks = _run_metrics_code(designed, out_base, log_fn=log_fn)
        if checks:
            m["functional_checks"] = checks
            failed = [c for c in checks if not c.get("passed")]
            m["functional_ok"] = not failed
            for c in checks:
                log_fn(f"[physics] check '{c['name']}': value={c['value']} "
                      f"expected={c['expected']} -> {'OK' if c['passed'] else 'FAILED'}"
                      + (f" ({c['detail']})" if c.get("detail") else ""))

        video = None
        if res.get("frames_dir"):
            mp4 = encode_mp4(res["frames_dir"], os.path.join(out_base, "model.mp4"))
            if mp4:
                video = f"physics/{sub}/model.mp4"

        diagnosis = {"verdict": None, "cause": "none", "reason": ""}
        if res.get("frames_dir"):
            try:
                gw = _gateway()
                diagnosis = diagnose_physics(task, _robot_info(model),
                                             designed or spec, m, res.get("frames_dir", ""),
                                             stability=stability,
                                             base_url=gw["base_url"], api_key=gw["api_key"],
                                             model=gw["model"])
                log_fn(f"[physics] mujoco VLM verdict={diagnosis.get('verdict')} "
                      f"cause={diagnosis.get('cause')} :: {diagnosis.get('reason','')[:100]}")
            except Exception as e:
                # Say WHY, with the traceback. This branch silently swallowed the only
                # record of a failed diagnosis: the message went to stdout (which run.log
                # never captured) and the caller was handed cause="none"/reason="" with no
                # way to tell a broken call from a healthy machine.
                import traceback as _tb
                log_fn(f"[physics] mujoco diagnose FAILED ({type(e).__name__}: {e})")
                log_fn("[physics] " + _tb.format_exc()[-1200:].replace(chr(10), " | "))
                diagnosis = {"verdict": None, "cause": "none", "reason": "",
                             "diagnose_error": f"{type(e).__name__}: {e}"}

        cause = diagnosis.get("cause", "none")
        # A BROKEN CHECK IS A TEST FAULT, not a machine fault. When the designer's
        # metrics_code cannot even find the joints it wants to measure, it has told us
        # nothing about the machine — and letting that stand as a functional failure would
        # blame the CAD for the test's mistake. (Measured: the designer reached for the
        # URDF names `place_minute_arbor`/`place_hour_arbor` while the trajectory is keyed
        # `minute_arbor_spin`/`hour_arbor_spin`, so its 12:1 check never ran.) Re-design
        # the same way a camera or scenario fault is re-designed, feeding back the exact
        # keys it should have used.
        if _check_is_broken(m) and attempt < _MAX_TEST_RETRIES:
            cause = "scenario"
            diagnosis = dict(diagnosis)
            diagnosis["reason"] = (
                "metrics_code could not evaluate: "
                + "; ".join(c.get("detail") or c.get("name")
                            for c in (m.get("functional_checks") or [])[:3])
                + ". The trajectory is keyed by SIM joint names "
                + f"{list((_traj_keys(out_base) or [])[:12])} — use those exactly.")
        # Only a TEST-side fault (camera/scenario) is worth re-designing + re-running here.
        if cause not in ("camera", "scenario") or attempt >= _MAX_TEST_RETRIES:
            break
        try:
            from environment_designer import revise_environment
            gw = _gateway()
            fb = (f"cause={cause}. {diagnosis.get('reason', '')} "
                  f"(input_travel={m.get('input_travel')}, "
                  f"exploded={m.get('exploded')}, "
                  f"moved={m.get('moved_count')}/{m.get('watched_count')})")
            log_fn(f"[physics] test-side fault '{cause}' (attempt {attempt + 1}) -> "
                  f"re-designing scenario + camera and re-running")
            designed = _design_spec(task, model, {"name": "drive", "driver": driver_part,
                                                  "strategy": "driven_mechanism",
                                                  "revise": {"prev": designed, "feedback": fb}})
            d = designed.get("drive") or {}
            spec["design"] = {
                "input_joint": d.get("input_joint"),
                "output_joint": d.get("output_joint"),
                "watch_joints": d.get("watch_joints", []),
                "propagation_path": d.get("propagation_path", []),
                "pass_criteria": designed.get("pass_criteria"),
                "environment": designed.get("environment") or designed.get("scenario"),
                "camera": designed.get("camera"),
            }
            if d.get("duration_s"):
                spec["duration_s"] = d["duration_s"]
        except Exception as e:
            log_fn(f"[physics] scenario re-design unavailable ({e}); keeping result")
            break

    # FINAL verdict is the DIAGNOSER's (VLM + hard signals + two-stage stability), not the
    # runner's mechanical threshold — the runner's `input_travel>0.05 and moved>=1` passes a
    # machine that barely twitched. Fall back to the mechanical verdict only if the
    # diagnoser was unavailable (verdict is None).
    dv = diagnosis.get("verdict")
    if dv in ("pass", "fail"):
        final_pass = (dv == "pass")
        final_verdict = "PASS" if final_pass else "FAIL"
    else:
        final_pass = (m.get("verdict") == "PASS")
        final_verdict = m.get("verdict", "FAIL")
    # A measured support fault OVERRIDES a pass. The VLM watches the recording, where a part
    # welded to a mount it never touches looks perfectly fine — it cannot see that nothing
    # holds it. Gravity already answered; that measurement wins.
    # A FAILED FUNCTIONAL CHECK FAILS THE RUN. Reporting it and then passing anyway made
    # the whole mechanism decorative: the watch below scored 10000 and PASSED with
    # functional_ok=False sitting in its metrics. A check that could not RUN is excluded —
    # that is a broken test, handled above by re-designing it.
    functional_failed = bool(m.get("functional_ok") is False and not _check_is_broken(m))
    if functional_failed and final_pass:
        names = ", ".join(c.get("name", "?") for c in (m.get("functional_checks") or [])
                          if not c.get("passed"))
        log_fn(f"[physics] overriding PASS: functional check failed ({names})")
        final_pass, final_verdict = False, "FAIL"

    overridden_by_support = bool(support_fell and final_pass)
    if overridden_by_support:
        log_fn(f"[support] overriding PASS: {len(support_fell)} unsupported part(s)")
        final_pass, final_verdict = False, "FAIL"

    # Say WHY in one voice. _summarize only knows the drive metrics, so on a support
    # override it reported "drive: PASS - 7/7 downstream joints moved" next to an overall
    # FAIL, which reads as a contradiction and tells the agent nothing about what to fix.
    summary_text = _summarize({"name": "drive"}, m)
    if functional_failed:
        bad = [c for c in (m.get("functional_checks") or []) if not c.get("passed")]
        detail = "; ".join(f"{c.get('name')}: {c.get('value')} vs expected "
                           f"{c.get('expected')}" for c in bad[:3])
        summary_text = (f"the machine runs ({summary_text}) but does NOT do its job: "
                        f"{detail}")
    if overridden_by_support:
        fell = ", ".join(f.part for f in support_fell[:4])
        summary_text = (f"transmission WORKS ({summary_text}) but the machine does not hold "
                        f"together: {len(support_fell)} part(s) have nothing supporting them "
                        f"({fell}) — they fall when the mount welds are released")

    entry = {"name": "drive", "strategy": "driven_mechanism",
             "verdict": final_verdict, "metrics": m, "stability": stability,
             "summary": summary_text,
             "cause": diagnosis.get("cause", "none"),
             "evidence": diagnosis.get("evidence") or [],
             "reason": diagnosis.get("reason", ""),
             "design": spec.get("design"),
             "frames_dir": res.get("frames_dir"), "video": video}
    return {"passed": final_pass, "verdict": final_verdict,
            "summary": entry["summary"], "metrics": m, "stability": stability,
            "cause": diagnosis.get("cause", "none"),
            "reason": diagnosis.get("reason", ""),
            "design": spec.get("design"),
            "frames_dir": res.get("frames_dir"), "video": video,
            "tests": [entry]}


def _run_sim_mujoco(mjcf, spec, out_base, task, *, log_fn=print):
    """Run the MuJoCo runner in a SUBPROCESS (isolates any GL/renderer state), reading
    back sim_result.json. Falls back to in-process on subprocess failure."""
    import run_scenario_mujoco as mjr
    out = Path(out_base)
    out.mkdir(parents=True, exist_ok=True)
    spec_json = out / "spec.json"
    spec_json.write_text(json.dumps(spec))
    try:
        r = subprocess.run(
            [sys.executable, mjr.__file__, "--mjcf", mjcf,
             "--spec", str(spec_json), "--out", out_base, "--task", task],
            capture_output=True, text=True, timeout=600, cwd=str(_ROOT))
        result_path = out / "sim_result.json"
        if r.returncode == 0 and result_path.exists():
            return json.loads(result_path.read_text())
        log_fn(f"[physics] mujoco subprocess rc={r.returncode}; "
              f"stderr tail: {(r.stderr or '')[-1500:]}")
    except Exception as e:
        log_fn(f"[physics] mujoco subprocess failed ({type(e).__name__}: {e}); "
              f"running in-process")
    return mjr.run(mjcf, spec, out_base, task)


def run_physics(urdf_path: str, task: str, run_dir: str, settings=None,
                iteration: int | None = None, log_fn=print) -> dict:
    """Category-aware physics on maker2's model. Returns the same shape the UI reads:
    {passed, verdict, summary, metrics, frames_dir}. `metrics` is the FINAL/primary
    test; `summary` spans all tests run.

    Engine dispatch (maker2-mujoco-contact): when settings.engine == "mujoco", run the
    pure-contact MuJoCo path (build the MJCF from the model, drive the driver PART's
    own dof, transmission by tooth contact). Otherwise the legacy PyBullet path."""
    engine = getattr(settings, "engine", "pybullet") if settings is not None else "pybullet"
    if engine == "mujoco":
        return _run_physics_mujoco(urdf_path, task, run_dir, settings,
                                   iteration=iteration, log_fn=log_fn)

    import run_scenario_pybullet as pyb
    from diagnose import diagnose_physics, encode_mp4

    model = _load_model(run_dir)
    tests = _environments(task, model) if model is not None else None

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
            "culprit_parts": agg.get("culprit_parts", []),
            "culprit_subs": agg.get("culprit_subs", []),
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
        # A test targets a driver when it (or the model) has a drivable input; then
        # _design_spec authors the environment AND enforces the drive. Otherwise it's a
        # stand-still test (environment_designer keeps drive=null).
        wants_drive = bool(test.get("driver") or test.get("subsystem"))
        if wants_drive:
            spec = _design_spec(task, model, test)
        elif model is not None:
            spec = _design_spec(task, model, test)
            spec.setdefault("drive", None)
        else:
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
        if diagnosis["cause"] == "camera" and attempt < 2:
            d = spec.setdefault("drive", {}) or {}
            d["cam_dist_scale"] = d.get("cam_dist_scale", 1.0) * 0.6
            d["cam_pitch"] = -60
            log("[physics] camera fault -> reframing camera, re-recording")
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
            "culprit_part": diagnosis.get("culprit_part", ""),
            "culprit_sub": diagnosis.get("culprit_sub", ""),
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
                "cause_map": {}, "blamed_subs": [], "blamed_kind": None,
                "culprit_parts": [], "culprit_subs": []}


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--task", default="")
    ap.add_argument("--out", default=".", help="run_dir (holds kinematic_model.json)")
    a = ap.parse_args()
    print(json.dumps(run_physics(a.urdf, a.task, a.out), indent=2))
