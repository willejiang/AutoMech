#!/usr/bin/env python3
"""CadQuery worker prompt: build the WHOLE model as one Python script, one
`build_<link>()` function per link, each returning a `cq.Workplane` solid.

This replaces the OpenSCAD worker. CadQuery is a parametric Python CAD kernel
(OpenCASCADE) whose whole reason for being here is CURVED geometry — fillets,
lofts, sweeps, revolves, splines — that OpenSCAD approximated poorly. The per-link
UNITS/ORIGIN contract is identical to the SCAD worker (model.UNITS_CONVENTION): each
function builds in MILLIMETERS, in its own local frame, with the joint-attachment
point at the origin, so a downstream tool exports each solid in isolation and the
manager's joint origins line everything up.

The runner executes the returned script in a sandboxed subprocess and calls
`build_<link>().val().exportStl("meshes/<link>.stl")` per link, then validates with
the same validation.check_stl gate the SCAD path used (mm STL, loadable,
non-degenerate).
"""
from __future__ import annotations

from ..model import KinematicModel, LinkSpec


CQ_WORKER_SYSTEM = """You are an expert CadQuery (Python) CAD engineer building parts for an articulated robot.

You are given a kinematic plan: a list of LINKS (rigid parts) and the JOINTS that
connect them. Write ONE complete Python script that defines the geometry of every link
using CadQuery.

HARD REQUIREMENTS (a downstream tool executes each function separately, exports its
solid to an STL, and assembles them via the joints — break these and assembly fails):

1. Define exactly ONE top-level function `build_<name>()` per link, where <name> is
   EXACTLY the link name given (already snake_case). Each returns a single CadQuery
   solid (a `cq.Workplane` whose `.val()` is one Solid). No more, no fewer functions.
2. Build each part in MILLIMETERS, in its OWN local frame, with its joint-attachment
   point at the ORIGIN (0,0,0). Orient the primary axis along +Z unless the link's
   origin_note says otherwise. Follow each link's size_mm and shape_hint for proportions.
3. Do NOT position parts relative to each other. Do NOT fuse parts together. Do NOT
   call the functions at module scope. Assembly is handled by the joints — each
   function must build ALONE and correctly.
4. Each function must return ONE manifold, 3D-printable solid. USE CadQuery's curved
   features where the part is curved — `.fillet()`, `.chamfer()`, `.loft()`, `.sweep()`,
   `.revolve()`, spline profiles — this is the whole point of CadQuery over OpenSCAD.
5. `import cadquery as cq` at the top. Use ONLY cadquery + the Python stdlib (math).
   No file I/O, no network, no other third-party imports. The runner supplies the
   export; your script must NOT read or write files itself.

Output ONLY the raw Python code — no markdown, no code fences, no prose."""


def _link_line(l: "LinkSpec") -> str:
    sz = ", ".join(f"{k}={v}" for k, v in (l.size_mm or {}).items())
    return (f"- build_{l.name}() : {l.description}"
            + (f" | shape: {l.shape_hint}" if l.shape_hint else "")
            + (f" | size_mm: {sz}" if sz else "")
            + (f" | origin: {l.origin_note}" if l.origin_note else ""))


def build_cq_worker_batch(model: KinematicModel, batch: list["LinkSpec"],
                          done: list[str], peers: list[str]) -> str:
    """Prompt for ONE batch of links, built in parallel with peer batches.

    Mirrors build_scad_worker_batch: three context sections (already built by prior
    waves, being built now by peers, YOUR batch) keep parts coherent across the
    parallel/waved build. In the common single-batch case (whole model in one call),
    the interpenetration guard fires.
    """
    names = [l.name for l in batch]
    lines = [f"PRODUCT: {model.name}", ""]
    if done:
        lines += ["ALREADY BUILT (earlier — match their style/scale so parts fit "
                  "together): " + ", ".join(done), ""]
    if peers:
        lines += ["BEING BUILT RIGHT NOW by parallel colleagues (do NOT define these "
                  "— just be consistent with them): " + ", ".join(peers), ""]
    lines += [f"YOUR BATCH — define EXACTLY these {len(names)} function(s) and NOTHING "
              f"else:"]
    for l in batch:
        lines.append(_link_line(l))
    touch = [j for j in model.joints if j.parent in names or j.child in names]
    if touch:
        lines += ["", "JOINTS touching your parts (context only — do NOT encode in "
                  "geometry):"]
        for j in touch:
            lines.append(f"- {j.name}: {j.type} {j.parent} -> {j.child}")
    lines += [
        "",
        f"Write a CadQuery Python script defining ONLY the {len(names)} function(s) in "
        "YOUR BATCH ({}), each `build_<name>()` returning one solid at its own local "
        "origin per the rules. Do NOT define the already-built or colleague functions. "
        "Define functions only; do not call them.".format(", ".join(names)),
    ]
    if not peers and not done:
        lines += [
            "",
            "You are building the ENTIRE assembly in one pass — hold all the parts in "
            "mind together. Size each part from its size_mm and keep parts that share a "
            "joint compatible (a shaft's diameter fits the bore it turns in; mating "
            "faces meet, they do not overlap) so that when the joints place them, no "
            "two solids interpenetrate.",
        ]
    return "\n".join(lines)


def build_cq_worker_batch_retry(failed: list[tuple[str, str]]) -> str:
    """Batch-scoped retry: fix only this batch's failed functions, return the whole
    (batch) file with all its functions."""
    lines = ["Some of YOUR batch's functions did not execute or export a valid STL. "
             "Fix these and return the COMPLETE script with ALL your batch's functions:"]
    for name, err in failed:
        lines.append(f"- build_{name}(): {err[:300]}")
    lines.append("Remember: `import cadquery as cq`, each build_<name>() returns one "
                 "solid at its local origin, no module-scope calls, no fusing, no file I/O.")
    return "\n".join(lines)


def build_cq_worker_continue(remaining: list[str]) -> str:
    """Continuation prompt: the previous reply was cut off at the output cap. The
    already-COMPLETE functions were kept; ask for the rest (the caller concatenates).
    `remaining` = function names not yet fully defined."""
    return (
        "Your previous reply was cut off at the output limit before you finished. "
        "The complete functions you already wrote have been kept. CONTINUE from where "
        "you stopped: output ONLY the remaining function(s) not yet fully defined "
        f"({', '.join(remaining)}), each complete `build_<name>()` returning one solid "
        "at its own local origin. Do NOT repeat the functions you already finished, and "
        "do NOT repeat the import. Output raw Python only — no fences, no prose.")


def build_cq_worker_edit(prior_script: str, part: str, fault: str) -> str:
    """2b: minimal-edit prompt (Claude-Code-style). Given ONE part's current CadQuery
    function and the exact fault, change as FEW lines as possible to fix it and return
    the smallest complete edited script for that part. Consumes Session A's culprit_part
    + the manager model-patch `modify_links` signal."""
    return (
        "Here is the current CadQuery script for ONE part and the exact problem with it. "
        "Change ONLY the lines needed to fix the problem — e.g. one dimension, add one "
        "fillet, fix one face — like a surgical code edit. Keep everything else IDENTICAL.\n\n"
        f"PART: build_{part}()\n"
        f"PROBLEM: {fault[:500]}\n\n"
        "CURRENT SCRIPT:\n"
        f"{prior_script}\n\n"
        "Return the COMPLETE corrected script for this part (the single `build_"
        f"{part}()` function), minimally changed. `import cadquery as cq` at the top, "
        "one solid returned at its local origin, no module-scope calls, no file I/O. "
        "Output raw Python only — no fences, no prose.")
