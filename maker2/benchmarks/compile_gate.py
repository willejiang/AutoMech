"""Phase 5 — compile_gate: the ultimate zero-false-positive sub gate (C5).

Every other gate reasons ABOUT the model; this one hands the model to the real simulator and
asks "does it load?". `mjcf_builder.build_mjcf` is the sole simulation compiler (CoACD convex
decomposition, mm->m scaling, arena sizing, solver/gravity tuning, per-part mass) — an LLM
cannot reproduce it — so a sub that survives every declarative gate can still fail to compile
(a degenerate solid, a mesh MuJoCo rejects, a body tree it won't parse). Running build_mjcf +
mujoco.MjModel.from_xml_path in-loop makes "this sub passed" start to mean "this sub loads in
the simulator that will score it". There are NO false positives: it IS the sim.

Runs AFTER the worker renders (needs real STLs on disk) and AFTER the real-mesh conflict gate,
as the final sub gate in orchestrator_boss._finish_subassembly. Guarded by settings.engine ==
"mujoco"; DEGRADES to a pass (empty list) if mujoco or build_mjcf are unavailable or error in a
way that isn't a model fault — it must never crash the pipeline, only catch genuine
won't-load models. See .claude/plans/precious-humming-wand.md C5.
"""

from __future__ import annotations

from . import GateError


def compile_gate(model, ctx, settings, *, log_fn=print) -> list[GateError]:
    """Build the sub's MJCF and try to load it in MuJoCo. Returns [] if it loads (or if the
    check can't run), or a single ERR_COMPILE naming the MuJoCo error if it won't.

    ``ctx`` is the sub's RunContext (build_mjcf writes model.mjcf next to ctx.urdf_path and
    reads the rendered meshes under ctx.meshes_dir). Only meaningful for the MuJoCo engine."""
    if getattr(settings, "engine", "mujoco") != "mujoco":
        return []
    if getattr(settings, "mjcf_compiler_mode", "agent") == "agent":
        # Topology requires the assembled machine: a sub alone cannot decide whether an
        # interface edge becomes tree inheritance, closure or an independent coordinate.
        # The final physics compile is the single agent-owned acceptance point.
        log_fn("[compile] agent MJCF compiler deferred until final assembled KinematicModel")
        return []
    try:
        import mujoco                                    # noqa: F401
        from ..mjcf_builder import build_mjcf
    except Exception as e:
        # No simulator / builder available in this environment -> can't dry-run; do not
        # block (the physics stage, if it runs, is the backstop).
        log_fn(f"[compile] dry-run skipped (mujoco/build_mjcf unavailable: {e})")
        return []

    # 1. Compile the MJCF. A failure HERE is a build_mjcf bug or a truly malformed model;
    #    either way the sub can't be simulated, so surface it as ERR_COMPILE.
    try:
        mjcf_path = build_mjcf(model, ctx, settings=settings, log_fn=log_fn)
    except Exception as e:
        return [GateError(
            "manager", "ERR_COMPILE",
            f"this subassembly could not be compiled to MJCF ({type(e).__name__}: "
            f"{str(e)[:200]}) — a part is degenerate or unbuildable; fix its size/geometry",
            "")]

    # 2. Load the compiled MJCF in MuJoCo — the exact parse the physics stage will do.
    try:
        import mujoco
        mujoco.MjModel.from_xml_path(mjcf_path)
    except Exception as e:
        return [GateError(
            "manager", "ERR_COMPILE",
            f"this subassembly's MJCF will not load in MuJoCo ({type(e).__name__}: "
            f"{str(e)[:200]}) — the simulator rejects it; the most common cause is a "
            "degenerate/zero-size solid or a mesh with no volume. Fix the offending part.",
            "")]
    return []
