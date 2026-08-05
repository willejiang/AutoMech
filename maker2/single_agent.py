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
from itertools import combinations
from pathlib import Path

from .model import KinematicModel, LinkSpec, PoseSpec

_EXEC_TIMEOUT = 300  # a whole machine is heavier than one sub
_RUNNER = (Path(__file__).parent / "_eval_runner_machine.py").read_text(encoding="utf-8")
_MESH_RE = re.compile(r"__mesh[_-]?([a-z0-9]+)$", re.I)
# Frames in the AGENT'S file, which is the only place it can fix anything. build123d and
# OCCT frames tell it nothing it can act on.
_MACHINE_FRAME_RE = re.compile(r'File "[^"]*machine\.py", line (\d+), in (\w+)\n\s*(.+)')


def _fault_site(trace: str) -> str:
    """' at machine.py:455 in build_machine() -> `loft()`', or '' if not locatable.

    OCCT raises with an empty message (`Standard_DomainError: `), so without this the
    agent is told only which C++ exception class fired. The last frame inside machine.py
    is the line it actually wrote, which is what it needs to change."""
    frames = _MACHINE_FRAME_RE.findall(trace or "")
    if not frames:
        return ""
    line, func, src = frames[-1]
    return f" at machine.py:{line} in {func}() -> `{src.strip()[:120]}`"


# Conservative connectivity gate. The graph is DELIBERATELY over-connected:
# two parts get an edge whenever their REAL mesh distance is < 10mm. That admits many
# false edges (a shaft near a shell, two clearanced fits), but that is the point — we want
# a one-sided test that only fails on clear disconnection. If even this supergraph is not
# connected, the assembly is obviously in pieces. Likewise, a declared connection whose two
# parts are not even within 10mm is almost certainly fiction.
_CONNECT_TOL_MM = 10.0


def _declared_edges(model) -> set[frozenset]:
    """Edges the agent has already DECLARED in today's language: mounts and mesh pairs.

    This is gate 2 before a richer JSON schema exists. When a support/mesh relation has
    been declared but does not even appear in the conservative proximity graph, we know the
    declaration and the geometry disagree badly enough to stop here rather than score the
    shape."""
    edges: set[frozenset] = set()
    for l in model.links:
        for m in [getattr(l, "mount", "")] + list(getattr(l, "extra_mounts", []) or []):
            m = (m or "").strip()
            if m:
                edges.add(frozenset((l.name, m)))
    groups: dict[str, list[str]] = {}
    for l in model.links:
        mid = str(getattr(l, "mesh_id", "") or "").strip()
        if mid:
            groups.setdefault(mid, []).append(l.name)
    for names in groups.values():
        for a, b in combinations(sorted(set(names)), 2):
            edges.add(frozenset((a, b)))
    return edges


def _bbox_gap_mm(a, b) -> float:
    """Minimum gap between two axis-aligned boxes in mm (0 if they overlap on all axes)."""
    dx = max(0.0, a[0] - b[3], b[0] - a[3])
    dy = max(0.0, a[1] - b[4], b[1] - a[4])
    dz = max(0.0, a[2] - b[5], b[2] - a[5])
    return float((dx * dx + dy * dy + dz * dz) ** 0.5)


def _proximity_graph(machine_eval_json: str, *, tol_mm: float = _CONNECT_TOL_MM):
    """Undirected supergraph over parts whose REAL mesh distance is < tol_mm.

    Uses the build123d-evaluated world transforms from machine_eval.json, not the derived
    KinematicModel poses, so rotated parts are tested in the coordinates the authoring
    script actually produced."""
    import trimesh

    ev = json.loads(Path(machine_eval_json).read_text(encoding="utf-8"))
    root = Path(machine_eval_json).parent
    nodes, meshes, bbs = [], {}, {}
    for p in ev.get("parts") or []:
        stl = root / "meshes" / f"{p['name']}.stl"
        if not stl.exists():
            continue
        try:
            m = trimesh.load(stl, force="mesh")
        except Exception:
            continue
        if not isinstance(m, trimesh.Trimesh) or len(m.faces) == 0:
            continue
        import numpy as _np
        R = _np.array(p["R"], dtype=float)
        T = _np.array(p["T"], dtype=float)
        v = m.vertices @ R.T + T
        wm = trimesh.Trimesh(vertices=v, faces=m.faces, process=False)
        nodes.append(p["name"])
        meshes[p["name"]] = wm
        lo, hi = wm.bounds
        bbs[p["name"]] = (float(lo[0]), float(lo[1]), float(lo[2]),
                           float(hi[0]), float(hi[1]), float(hi[2]))
    E: set[frozenset] = set()
    for a, b in combinations(nodes, 2):
        if _bbox_gap_mm(bbs[a], bbs[b]) > tol_mm:
            continue
        ma, mb = meshes[a], meshes[b]
        # If the AABBs overlap, these parts are certainly in the conservative graph.
        if _bbox_gap_mm(bbs[a], bbs[b]) <= 0.0:
            E.add(frozenset((a, b)))
            continue
        # Real mesh distance (sampled both ways to avoid one sparse mesh hiding a gap).
        try:
            va, vb = ma.vertices, mb.vertices
            if len(va) > 400:
                va = va[::max(1, len(va) // 400)]
            if len(vb) > 400:
                vb = vb[::max(1, len(vb) // 400)]
            da = float(trimesh.proximity.closest_point(mb, va)[1].min())
            db = float(trimesh.proximity.closest_point(ma, vb)[1].min())
            if min(da, db) < tol_mm:
                E.add(frozenset((a, b)))
        except Exception:
            pass
    return nodes, E


def _connected_components(nodes: list[str], edges: set[frozenset]) -> list[list[str]]:
    """Connected components of the conservative proximity graph, largest first."""
    adj = {n: set() for n in nodes}
    for e in edges:
        if len(e) != 2:
            continue
        a, b = tuple(e)
        adj[a].add(b); adj[b].add(a)
    seen, comps = set(), []
    for n in nodes:
        if n in seen:
            continue
        q, comp = [n], []
        seen.add(n)
        while q:
            x = q.pop()
            comp.append(x)
            for y in adj[x]:
                if y not in seen:
                    seen.add(y); q.append(y)
        comps.append(sorted(comp))
    comps.sort(key=lambda c: (-len(c), c[0]))
    return comps


def _summarize_components(comps: list[list[str]], max_parts: int = 10) -> list[str]:
    """`subgraph1{a,b,c,...}` lines for the agent/log, largest-first."""
    out = []
    for i, comp in enumerate(comps, start=1):
        head = ", ".join(comp[:max_parts])
        more = f", ... (+{len(comp) - max_parts} more)" if len(comp) > max_parts else ""
        out.append(f"subgraph{i}{{{head}{more}}}")
    return out


def _connectivity_gate(model, machine_eval_json: str):
    """Return None if the conservative proximity graph is connected and every declared edge
    appears in it; else a dict describing the failure for the agent.

    Gate 1: BFS connectivity on the over-connected graph.
    Gate 2: every declared edge must at least appear in that graph (e ∈ E).
    """
    nodes, edges = _proximity_graph(machine_eval_json)
    comps = _connected_components(nodes, edges)
    declared = _declared_edges(model)
    missing = sorted(declared - edges, key=lambda e: tuple(sorted(e)))
    if len(comps) == 1 and not missing:
        return None
    evidence = []
    if len(comps) > 1:
        evidence.append(
            f"conservative proximity graph is DISCONNECTED at {_CONNECT_TOL_MM:.0f}mm: "
            + "; ".join(_summarize_components(comps)))
    if missing:
        show = []
        for e in missing[:12]:
            a, b = tuple(sorted(e))
            show.append(f"{a}<->{b}")
        tail = f", ... (+{len(missing)-12} more)" if len(missing) > 12 else ""
        evidence.append("declared edges missing from the proximity graph: "
                        + ", ".join(show) + tail)
    return {"root_cause": "connectivity gate failed", "evidence": evidence}


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
        # `err or tail` used to DROP the traceback whenever there was an error string, and
        # OCCT's exceptions carry no message: the agent was handed exactly
        # "Standard_DomainError: " — no line, no operation, nothing to act on — and could
        # only rewrite the whole script and hope. Keep both, and pull out the frames in
        # the agent's OWN file so the useful line is first.
        where = _fault_site(trace)
        detail = f"{err}{where}" if err else tail
        # Only append the raw traceback when it ADDS something. A SyntaxError already
        # carries its own file and line, and its traceback is all runner frames — pure
        # noise for the agent, which has to be told what to change, not how Python got
        # there. `[-600:]` also used to slice mid-header, printing
        # "Traceback (last frames):\nTraceback (most recent " and nothing else.
        if err and trace and not where and "SyntaxError" not in str(err):
            frames = [ln for ln in trace.splitlines()
                      if ln.strip() and not ln.startswith("Traceback (")]
            if frames:
                detail = f"{detail}\n\nLast frames:\n" + "\n".join(frames[-8:])
        raise SingleAgentError(f"machine build123d eval failed: {detail}")
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
        driver = bool(meta.get("driver", False)) or (_infer_driver(name) and dof in ("spin", "slide"))
        spin_axis = meta.get("spin_axis")
        if not isinstance(spin_axis, (list, tuple)) or len(spin_axis) != 3:
            spin_axis = (0.0, 0.0, 1.0)
        slide_axis = meta.get("slide_axis")
        if not isinstance(slide_axis, (list, tuple)) or len(slide_axis) != 3:
            slide_axis = (1.0, 0.0, 0.0)
        # A valid mount names another part; otherwise treat as a world root (parent="").
        # Resolved BEFORE the link is built, because it belongs on the LINK as well as on
        # the pose. A dof=fixed part has no joint of its own, so the link record is the
        # only place the test designer can learn what carries it; when this lived only on
        # the pose, every link serialized with no mount and the designer fell back to
        # guessing a nearby gear -- reporting a ratio between two parts nobody can see.
        # SUPPORT IS NOT A TREE. `mount=a,b` lists every part that carries this one — a
        # shaft running through two bearings is held by both. Only the FIRST is used as
        # the pose parent (poses stay a forest; MJCF bodies are flat anyway, so the
        # parent is now just the frame the relative pose is expressed in and moves
        # nothing). The rest ride along on the link as extra_mounts, where the support
        # and fit checks read them.
        raw = str(meta.get("mount", "") or "")
        wanted = [s.strip() for s in raw.split(",") if s.strip()]
        valid = [s for s in wanted if s != name and s in names]
        mount = valid[0] if valid else ""
        parent = mount
        links.append(LinkSpec(
            name=name, description=meta.get("description", name),
            mesh_filename=f"meshes/{name}.stl",
            dof=dof, spin_axis=tuple(spin_axis), slide_axis=tuple(slide_axis), driver=driver,
            material=str(meta.get("material", "steel")), mount=parent,
            extra_mounts=valid[1:]))
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

    # A MESHING GEAR CANNOT BE `fixed`. In this model `dof` is relative to the WORLD:
    # `fixed` means welded to the world and never turning. Agents keep writing
    # `dof=fixed` on a wheel to mean "it is pressed onto its arbor and turns with it" —
    # which is the right MECHANICS but the wrong word, and the consequences are silent
    # and total: _add_transmission_constraints only emits a ratio joint when BOTH sides
    # of a mesh pair spin, so a fixed pair gets no constraint AND no contact exclusion,
    # and the two gears then grind tooth-on-tooth. (Measured on 1_12_20260728_130546:
    # zero <equality> constraints, 1.7e17 N between hour_wheel_45t and minute_pinion_15t,
    # and the driver managed 0.036 of 12 commanded rad.)
    #
    # Being a mesh_pair member is unambiguous evidence the part rotates, so promote it and
    # take the spin axis from the spinning part it is mounted on (the arbor defines the
    # axis; the wheel just rides it). The 1:1 press-fit lock is then re-derived downstream
    # from bore-vs-shaft geometry, exactly as for any other part on that arbor.
    by_name = {l.name: l for l in links}
    parent_of = {p.child: p.parent for p in poses if p.parent}
    for gear in {n for pair in mesh_pairs for n in pair}:
        link = by_name.get(gear)
        if link is None or link.dof != "fixed":
            continue
        host = by_name.get(parent_of.get(gear, ""))
        link.dof = "spin"
        if host is not None and host.dof == "spin":
            link.spin_axis = tuple(host.spin_axis)
        if log_fn:
            log_fn(f"[single-agent] '{gear}' meshes with another gear but was declared "
                   f"dof=fixed (welded to the world) — promoted to dof=spin"
                   + (f", axis from '{host.name}'" if host is not None
                      and host.dof == "spin" else ""))

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


def _snapshot_iteration(run_dir: str, it: int, log_fn) -> str:
    """Freeze this iteration's geometry into ``iter_<n>/`` and return that directory.

    machine.py, model.urdf and meshes/ all live at FIXED names in run_dir, so every
    iteration overwrote the last one. Only the videos survived, because they alone were
    written per-iteration. The canvas looked like it kept each version, but it only held
    blobs fetched live in that browser session -- reopening a finished run re-fetched the
    one directory three times and showed the final model under every version tab, and the
    earlier geometry was gone from disk entirely.

    The URDF references meshes by the relative path "meshes/<part>.stl", so a directory
    holding both is self-contained and the existing GLB route renders it as-is.
    Best-effort: a snapshot failure must never take the iteration down with it.
    """
    import os
    import shutil
    dst = os.path.join(run_dir, f"iter_{it}")
    try:
        os.makedirs(dst, exist_ok=True)
        for fn in ("machine.py", "kinematic_model.json", "model.urdf", "machine.step"):
            src = os.path.join(run_dir, fn)
            if os.path.exists(src):
                shutil.copy2(src, os.path.join(dst, fn))
        meshes = os.path.join(run_dir, "meshes")
        if os.path.isdir(meshes):
            # Part meshes only: the *_cvx_*.stl convex pieces are physics scratch, are
            # regenerated per run, and would multiply the snapshot size for nothing.
            mdst = os.path.join(dst, "meshes")
            os.makedirs(mdst, exist_ok=True)
            for fn in os.listdir(meshes):
                if fn.endswith(".stl") and "_cvx" not in fn:
                    shutil.copy2(os.path.join(meshes, fn), os.path.join(mdst, fn))
        return dst
    except Exception as e:
        log_fn(f"[single-agent] iteration snapshot failed: {e}")
        return run_dir


def _summarize_conflicts(conflicts: list, max_lines: int = 40) -> str:
    """Group interpenetrations by the part they centre on, worst offender first.

    A big machine produces far more conflicts than fit in a feedback message, and the
    truncated list this replaces (`conflicts[:8]`) was actively harmful: on the P-51 run
    the agent saw 8 of 63, and the NEXT round showed a different 8 (zero overlap with the
    previous set). It never saw the whole picture, so each round it fixed what it could
    see and broke something it could not — 63 -> 39 over six rounds, with a regression
    (47 -> 51) in the middle, while rewriting ~1000 lines each time.

    Grouping is what makes the list short enough to send whole. Measured on that run's
    iteration 5: 39 conflicts across 41 parts, but 19 of them involve `main_fuselage`
    alone. Reported as one line — "main_fuselage overlaps 19 parts" — the agent gets one
    fixable cause instead of 19 symptoms, and the remaining tail is small enough to list
    in full."""
    if not conflicts:
        return ""
    from collections import defaultdict
    partners: dict = defaultdict(list)
    for c in conflicts:
        partners[c.part_a].append((c.part_b, c.frac))
        partners[c.part_b].append((c.part_a, c.frac))

    # Cover the conflict set greedily: repeatedly take the part in most remaining pairs,
    # so a hub that overlaps 19 things is reported once, not once per victim.
    remaining = {frozenset((c.part_a, c.part_b)): c for c in conflicts}
    lines: list[str] = []
    while remaining and len(lines) < max_lines:
        counts: dict = defaultdict(int)
        for pair in remaining:
            for p in pair:
                counts[p] += 1
        hub = max(counts, key=lambda p: (counts[p], p))
        hits = [(pair, c) for pair, c in remaining.items() if hub in pair]
        if len(hits) == 1:
            lines.append(f"- {hits[0][1].describe()}")
        else:
            others = sorted(
                ((next(iter(pair - {hub})), c.frac) for pair, c in hits),
                key=lambda t: -t[1])
            worst = ", ".join(f"{n} ({f:.0%})" for n, f in others[:6])
            more = f", and {len(others) - 6} more" if len(others) > 6 else ""
            lines.append(
                f"- '{hub}' interpenetrates {len(others)} parts — one cause, not "
                f"{len(others)} separate faults. Overlaps: {worst}{more}. Check "
                f"'{hub}' itself: its size, its origin, or where it is placed.")
        for pair, _ in hits:
            remaining.pop(pair, None)

    if remaining:
        lines.append(f"- ...and {len(remaining)} further overlapping pair(s) not listed.")
    return "\n".join(lines)


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
                     image_path: str | None = None,
                     log_fn=print) -> dict:
    """The single-agent text-to-cad pipeline: ONE agent authors the whole machine, refines it
    against build-eval errors + a rigid-conflict geometry self-check, then the machine is
    prechecked (warn-only) and run through the existing MuJoCo physics. Returns the same
    RESULT_JSON shape run_boss does so the worker/UI reads it unchanged.

    Loop per iteration: LLM authors/repairs build_machine() -> evaluate to a KinematicModel
    (eval error feeds back) -> build URDF -> rigid-conflict check (overlaps feed back). On a
    clean geometry pass, stop refining and go to physics.

    ``image_path`` attaches a reference photo/drawing to the FIRST message, so the agent
    builds what it sees instead of what the prompt says. Only the first: every later message
    is feedback about the agent's own geometry, and re-sending the picture each round costs
    tokens without adding information the conversation does not already carry."""
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
    # An image that cannot be loaded must STOP the run. Degrading to text-only would build
    # a different machine than the one asked for and report success, and the divergence
    # would only surface at the judge — as "wrong shape", with no hint that the picture was
    # never seen.
    images = None
    if image_path:
        from .imageutil import load_image_block
        images = [load_image_block(image_path)]
        log_fn(f"[single-agent] using input image: {image_path}")
    conv.add_user_message(build_single_agent_user(product_prompt,
                                                 has_image=bool(image_path)),
                          images=images)

    # RESEARCH PRE-STEP (web + local KB), same as the multi-agent manager gets. The single
    # agent authors the WHOLE machine from memory otherwise — it guesses dimensions and
    # standard sizes, which is exactly what keeps going wrong. Look them up FIRST so the
    # numbers are grounded. Gated by settings.enable_reference_tools (web) /
    # enable_kb (local); a no-op if both are off.
    #
    # THE BRIEF MUST NOT NAME A MACHINE CLASS. This used to ask for "gear
    # module/tooth-count/center-distance math ... worked gear-train / watch-movement
    # examples", which was written when the pipeline only built gearboxes. It then steered
    # EVERY run at gears: asked for a P-51, the agent dutifully searched for gear modules
    # and never once searched for how to model a wing — so the curved-geometry doc sitting
    # in the KB was never retrieved and the aircraft came out of boxes and cylinders.
    # Name the CATEGORIES of thing to look up and let the prompt decide which apply.
    try:
        from .tools import maybe_research
        maybe_research(client, conv, settings,
                       f"authoring the complete machine for: {product_prompt} — look up "
                       f"whatever THIS machine actually needs: the build123d/CadQuery "
                       f"operations its shapes require (curved or organic bodies need "
                       f"loft/revolve/sweep/fillet, not just boxes and cylinders), how to "
                       f"place and orient parts correctly, the mechanisms that produce the "
                       f"motion it must produce, the arithmetic that governs them (gear "
                       f"module/tooth-count/centre-distance if it has gears), standard "
                       f"sizes and fits, and any worked example of a similar machine",
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
    # Impossible bore fits from the PREVIOUS iteration, so each round can tell the agent
    # which of its edits landed and which regressed.
    prev_fits: set = set()

    for it in range(iter_cap):
        result["iterations"] = it + 1
        last_iter = (not unlimited) and it >= max_iters - 1
        log_fn(f"[single-agent] iteration {it}: authoring build_machine() ...")
        try:
            # get_messages_for_api, NOT conv.messages: the raw list is the INTERNAL format,
            # whose image blocks are {"type":"image","media_type",...}. Only this call
            # converts them to the provider's shape (OpenAI's image_url data-URI), and a
            # gateway handed the internal block rejects the whole request with
            # HTTP 400 "type has to be either 'image_url' or 'text'". Text-only messages
            # pass through either way, which is why this went unnoticed until an image was
            # attached. manager.py already sends through this path.
            reply = client.send(
                conv.get_messages_for_api(api_style=client.api_style),
                SINGLE_AGENT_SYSTEM)
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
            # First line only: it now carries the exception, the file, the line and the
            # failing call, and that is the whole diagnosis. Slicing at 160 characters cut
            # through the traceback that follows and printed a dangling "Traceback (most
            # recent " to the console. The AGENT still gets the full text below.
            log_fn(f"[single-agent] eval failed: {str(e).splitlines()[0][:300]}")
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
        # Freeze this version before the next iteration overwrites the fixed filenames,
        # and point the UI at the snapshot so reopening the run shows each version's own
        # geometry instead of the final one under every tab.
        snap = _snapshot_iteration(run_dir, it, log_fn)
        log_fn("ARTIFACT_JSON:" + _json.dumps({
            "kind": "assembled_model", "iter": it, "run_dir": run_dir,
            "render_dir": snap}))

        # Connectivity gate before any overlap counting. The graph is deliberately
        # over-connected: any two STLs within 10mm get an edge. If even THAT graph is
        # disconnected, the machine is obviously in pieces; and if a declared mount/mesh
        # edge is not even present in that graph, the declaration and the geometry disagree
        # so strongly there is no point counting interpenetrations yet. This is the cheap,
        # one-sided gate the overlap score was missing: it fails only on clear disconnection,
        # never on a correct 0.05mm fit.
        gate = None
        try:
            gate = _connectivity_gate(model, str(Path(run_dir) / "machine_eval.json"))
        except Exception as e:
            log_fn(f"[single-agent] connectivity gate unavailable ({type(e).__name__}: {e})")
        if gate and not last_iter:
            findings = "\n".join(f"- {x}" for x in gate.get("evidence") or [])
            log_fn(f"[single-agent] connectivity gate failed -> asking agent to fix")
            log_fn("ARTIFACT_JSON:" + _json.dumps({
                "kind": "diagnosis", "iter": it, "single_agent": True,
                "decision": {"root_cause": gate.get("root_cause", "connectivity gate failed"),
                             "evidence": gate.get("evidence") or []}}))
            conv.add_user_message(build_single_agent_geometry_feedback(findings))
            continue

        # Rigid-conflict geometry self-check (the text-to-cad "inspect" step, reusing subcheck).
        # A gross interpenetration is cheaper to fix here than to run physics on, so gate on it
        # next — but only re-author for it when we still have iterations left.
        #
        # Support is NOT checked here any more. The ray-cast floating detector it used to run
        # passed a stack of parts propping each other up and read a 1.75mm air gap as contact;
        # support is now measured under gravity as part of stage-1 stability (support_test).
        conflicts = []
        try:
            from .subcheck import sub_conflicts
            conflicts = sub_conflicts(model, ctx.urdf_path, log_fn=lambda *_: None)
        except Exception as e:
            log_fn(f"[single-agent] geometry check unavailable ({type(e).__name__}: {e})")
        if conflicts and not last_iter:
            findings = _summarize_conflicts(conflicts)
            badness = float(len(conflicts))
            geo_regressed = badness > geo_best["badness"]
            if badness < geo_best["badness"]:
                geo_best = {"badness": badness, "code": code}
            log_fn(f"[single-agent] {len(conflicts)} interpenetration(s) -> asking agent to fix "
                   f"(badness={badness:.2f}, best={geo_best['badness']:.2f}"
                   f"{', REGRESSED' if geo_regressed else ''})")
            # The artifact carries the GROUPED view, the same thing the agent was shown.
            # It used to log conflicts[:8] — so a run's record said "63 interpenetrations"
            # and then listed 8 of them, and which 8 changed every round, which is what
            # made the loop's behaviour so hard to read from the log afterwards.
            log_fn("ARTIFACT_JSON:" + _json.dumps({
                "kind": "diagnosis", "iter": it, "single_agent": True,
                "decision": {"root_cause": f"{len(conflicts)} interpenetration(s)",
                             "evidence": findings.split("\n")}}))
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
            # Pass the run's logger in. physics used bare print() throughout, which the
            # SSE stream showed but run.log never captured — so when a diagnosis failed,
            # the ONLY record of why went to stdout and died with the process, leaving
            # cause="none"/reason="" on disk and no way to tell a broken call from a
            # healthy machine.
            phys = run_physics(ctx.urdf_path, product_prompt, run_dir, settings,
                               iteration=it, log_fn=log_fn)
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

        # What did THIS edit actually change? A single score tells the agent it got worse
        # but not WHICH edit did it, so it "fixes" the wrong thing and undoes the parts it
        # had already got right (measured: iteration 2 correctly opened the centre bores
        # from 1.1 to 4.3mm, clearing every minute_pinion fault; iteration 3 put them back
        # at 1.1 and the input travel fell from 0.016 to 0.003 rad). Diff the impossible
        # fits against the previous iteration so the feedback can name the ones it FIXED
        # (keep these) separately from the ones it broke or never touched.
        fits_now = {(f["part"], f["shaft"]) for f in (metrics.get("bore_fit_faults") or [])
                    if f.get("impossible")}
        fixed_fits = sorted(prev_fits - fits_now)
        broke_fits = sorted(fits_now - prev_fits)
        prev_fits = fits_now

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
                best_code=(best["code"] if regressed else None), regressed=regressed,
                fixed_fits=fixed_fits, broke_fits=broke_fits))
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

