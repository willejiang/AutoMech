#!/usr/bin/env python3
"""SCAD worker prompt: build the WHOLE model as one .scad, one module per link.

Unlike makerv2's FreeCAD worker (one part per call), the cadam SCAD worker gets
the ENTIRE kinematic model in a single call and emits ONE OpenSCAD file with a
top-level `module <link.name>()` per link. cadam generates whole assemblies well;
this plays to that strength. The per-link UNITS/ORIGIN contract is preserved from
makerv2 (model.UNITS_CONVENTION) so each module renders in isolation at the right
local frame, and the manager's joint origins line everything up.
"""
from __future__ import annotations

from ..model import KinematicModel, LinkSpec


SCAD_WORKER_SYSTEM = """You are an expert OpenSCAD CAD engineer building parts for an articulated robot.

You are given a kinematic plan: a list of LINKS (rigid parts) and the JOINTS that
connect them. Your job is to write ONE complete OpenSCAD file that defines the
geometry of every link.

HARD REQUIREMENTS (a downstream tool renders each module separately and assembles
them via the joints — break these and assembly fails):

1. Define exactly ONE top-level `module <name>() { ... }` per link, where <name>
   is EXACTLY the link name given (already snake_case). No more, no fewer.
2. Build each part in MILLIMETERS, in its OWN local frame, with its
   joint-attachment point at the ORIGIN (0,0,0). Orient the primary axis along +Z
   unless the link's origin_note says otherwise. Follow each link's size_mm and
   shape_hint for proportions.
3. Do NOT position parts relative to each other. Do NOT fuse parts together.
   Assembly is handled by the joints — each module must render ALONE and correctly.
4. Do NOT call the modules at the top level (no top-level `name();` statements),
   and do NOT wrap them in translate()/union() at file scope. Just DEFINE the
   modules. The renderer invokes each one in isolation.
5. Each module must produce a single, manifold, 3D-printable solid. Use $fn=48 or
   higher for curved surfaces. You may use BOSL2 (include <BOSL2/std.scad>) for
   advanced primitives if helpful.

Output ONLY the raw OpenSCAD code — no markdown, no code fences, no prose."""


def build_scad_worker_user(model: KinematicModel) -> str:
    lines = [f"PRODUCT: {model.name}", "", "LINKS (one module per link):"]
    for l in model.links:
        sz = ", ".join(f"{k}={v}" for k, v in (l.size_mm or {}).items())
        lines.append(
            f"- module {l.name}() : {l.description}"
            + (f" | shape: {l.shape_hint}" if l.shape_hint else "")
            + (f" | size_mm: {sz}" if sz else "")
            + (f" | origin: {l.origin_note}" if l.origin_note else "")
        )
    lines += ["", "JOINTS (context only — do NOT encode these in geometry):"]
    for j in model.joints:
        lines.append(f"- {j.name}: {j.type} {j.parent} -> {j.child}")
    lines += [
        "",
        "Write the OpenSCAD file: one top-level module per link above, each built "
        "at its own local origin per the rules. Define modules only; do not call them.",
    ]
    return "\n".join(lines)


def build_scad_worker_retry(failed: list[tuple[str, str]]) -> str:
    """failed = [(link_name, error), ...] -> a focused fix request."""
    lines = ["Some modules did not render to a valid STL. Fix ONLY these modules "
             "(keep the others working) and return the COMPLETE corrected file:"]
    for name, err in failed:
        lines.append(f"- module {name}(): {err[:300]}")
    lines.append("Remember: define each module at its local origin, one solid, "
                 "no top-level calls, no fusing.")
    return "\n".join(lines)


def _link_line(l: "LinkSpec") -> str:
    sz = ", ".join(f"{k}={v}" for k, v in (l.size_mm or {}).items())
    return (f"- module {l.name}() : {l.description}"
            + (f" | shape: {l.shape_hint}" if l.shape_hint else "")
            + (f" | size_mm: {sz}" if sz else "")
            + (f" | origin: {l.origin_note}" if l.origin_note else ""))


def build_scad_worker_batch(model: KinematicModel, batch: list["LinkSpec"],
                            done: list[str], peers: list[str]) -> str:
    """Prompt for ONE batch of links, built in parallel with peer batches.

    Gives the worker three context sections so parts stay coherent across the
    parallel/waved build without waiting on anyone:
      - already built (by PRIOR waves): names it should match in style/scale/fit.
      - being built RIGHT NOW (peer batches this wave): names, so it doesn't
        duplicate or collide conceptually.
      - YOUR batch: the ONLY modules this call must define.
    """
    names = [l.name for l in batch]
    lines = [f"PRODUCT: {model.name}", ""]
    if done:
        lines += ["ALREADY BUILT (earlier — match their style/scale so parts fit "
                  "together): " + ", ".join(done), ""]
    if peers:
        lines += ["BEING BUILT RIGHT NOW by parallel colleagues (do NOT define these "
                  "— just be consistent with them): " + ", ".join(peers), ""]
    lines += [f"YOUR BATCH — define EXACTLY these {len(names)} module(s) and NOTHING "
              f"else:"]
    for l in batch:
        lines.append(_link_line(l))
    # Joints touching this batch, for spatial context only.
    touch = [j for j in model.joints if j.parent in names or j.child in names]
    if touch:
        lines += ["", "JOINTS touching your parts (context only — do NOT encode in "
                  "geometry):"]
        for j in touch:
            lines.append(f"- {j.name}: {j.type} {j.parent} -> {j.child}")
    lines += [
        "",
        f"Write an OpenSCAD file defining ONLY the {len(names)} module(s) in YOUR "
        "BATCH ({}), each at its own local origin per the rules. Do NOT define the "
        "already-built or colleague modules. Define modules only; do not call them."
        .format(", ".join(names)),
    ]
    return "\n".join(lines)


def build_scad_worker_batch_retry(failed: list[tuple[str, str]]) -> str:
    """Batch-scoped retry: fix only this batch's failed modules, return the whole
    (batch) file with all its modules."""
    lines = ["Some of YOUR batch's modules did not render to a valid STL. Fix these "
             "and return the COMPLETE file with ALL your batch's modules:"]
    for name, err in failed:
        lines.append(f"- module {name}(): {err[:300]}")
    lines.append("Remember: define each module at its local origin, one solid, "
                 "no top-level calls, no fusing.")
    return "\n".join(lines)


def build_scad_worker_continue(remaining: list[str]) -> str:
    """Continuation prompt: the previous reply was cut off at the output cap. The
    already-COMPLETE modules were kept; ask for the rest, starting fresh (the caller
    concatenates). `remaining` = module names not yet fully defined."""
    return (
        "Your previous reply was cut off at the output limit before you finished. "
        "The complete modules you already wrote have been kept. CONTINUE from where "
        "you stopped: output ONLY the remaining module(s) not yet fully defined "
        f"({', '.join(remaining)}), each complete and at its own local origin. Do "
        "NOT repeat the modules you already finished. Output raw OpenSCAD only — no "
        "fences, no prose.")
