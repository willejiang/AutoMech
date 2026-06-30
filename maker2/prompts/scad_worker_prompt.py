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
