"""Prompt for the per-subassembly rigid-conflict DEBUGGER (maker2/subdebugger.py).

The debugger is invoked when subcheck.sub_conflicts finds two rigid parts of ONE
subassembly interpenetrating. Unlike the single-part edit prompt (build_cq_worker_edit),
this agent sees the WHOLE subassembly — the user's request, the boss's brief + immovable
interface frames, the URDF, and EVERY part's CadQuery script — and returns a minimal
patch that may MOVE a part (edit its joint origin) or RESHAPE it (edit its CadQuery
script), or both.

It respects the same coordinate convention as the worker: each part is built in its OWN
local frame with its joint-attachment point at the ORIGIN, primary axis +Z unless the
link's origin_note says otherwise; the manager's joint xyz_m/rpy_rad places a child's
origin relative to its parent. So an overlap is either a bad placement (fix the joint) or
geometry that reaches further from its attach point than the placement assumed (fix the
script).
"""

from __future__ import annotations

from ..twophase import JSON_SENTINEL


SUBDEBUGGER_SYSTEM = f"""You are a CAD DEBUGGER for an articulated-robot pipeline. Two or
more rigid parts of ONE subassembly INTERPENETRATE (occupy the same space) — a physical
impossibility. Your job is to make the parts CLEAR each other while keeping the machine
faithful to what was asked, changing as LITTLE as possible.

HOW THE PARTS ARE PLACED (the convention every part obeys):
- Each part's `build_<name>()` builds it in MILLIMETERS in its OWN local frame, with its
  natural attach/rotation point at the ORIGIN (0,0,0), primary axis along +Z unless the
  part's origin_note says otherwise.
- A POSE places a CHILD part relative to its PARENT: `xyz_m` (METERS) is the translation
  of the child's origin, `rpy_rad` (radians) its rotation. This is the ONLY thing that
  positions one part relative to another. Parts MOVE by their own dof (a "spin" part
  rotates on an implied axle); motion transmits by CONTACT, not by joints or motors.

Root causes you fix (pick the right one per fault):
1. WRONG PLACEMENT — a pose puts a part too close/inside another, OR two gears that must
   mesh are too far apart / not touching. FIX: change that pose's `xyz_m`/`rpy_rad` to
   separate overlapping parts, or to bring meshing gears to exactly one pitch-center
   distance so their teeth engage.
2. OVERSIZED / MISORIENTED GEOMETRY — the part's solid reaches further from its attach
   point than the placement assumed. FIX: edit that ONE part's script minimally (a
   dimension, a translate to re-seat it at its origin, an extrude direction).
3. UNSUPPORTED / FLOATING — a part rests on nothing and would fall under gravity. FIX:
   move it (pose) down onto real support, or add/extend the supporting part.
4. JAM — meshing parts lock instead of moving. FIX: relieve the interference (resize a
   tooth, widen a clearance) or re-space the pose so they roll rather than wedge.

HARD RULES:
- The INTERFACE FRAME parts listed below are FROZEN: never change their placement pose
  AND never edit their CadQuery script (do not move, shrink, or reshape them). Their pose
  and geometry are the contract the rest of the machine welds to — an edit to them is
  rejected. If a frozen part overlaps a neighbor, fix the OTHER (non-frozen) part instead.
- Prefer the SMALLEST change. Usually ONE pose edit OR ONE script edit clears a pair.
- Do not rename parts, add parts, or restructure the placement. Do not fuse parts.
- Edited scripts keep the convention: `import cadquery as cq`, exactly one
  `build_<name>()` returning one solid at its local origin, no module-scope calls, no
  file I/O.

Respond in TWO parts, in this exact order:
1. NOTES — briefly: for each conflicting pair, which root cause (placement vs geometry)
   and the specific change you will make. This is your scratchpad.
2. A line containing exactly:  {JSON_SENTINEL}
3. ONE JSON object with this shape (omit an array if empty; do NOT wrap in fences):
{{
  "pose_edits":   [{{"pose": "<pose name>", "xyz_m": [x, y, z], "rpy_rad": [r, p, y]}}],
  "script_edits": [{{"link": "<link name>", "script": "import cadquery as cq\\n\\ndef build_<link>():\\n    ..."}}],
  "reason": "<one line: what you changed and why the parts now clear>"
}}"""


def _fmt_mount_frames(frames: list) -> str:
    if not frames:
        return "  (none — nothing in this subassembly is pinned to an interface frame)"
    out = []
    for fr in frames:
        xyz = getattr(fr, "xyz_m", (0, 0, 0))
        axis = getattr(fr, "axis", (0, 0, 1))
        out.append(f"  - {fr.name}: global xyz_m={list(xyz)}, axis={list(axis)} "
                   f"(IMMOVABLE — the part realizing this frame must stay here)")
    return "\n".join(out)


def build_subdebugger_user(user_prompt: str, brief: str, frames: list,
                           model_json: str, urdf_text: str,
                           part_scripts: dict[str, str],
                           conflicts_desc: list[str],
                           frozen_links=None, physics_metrics=None) -> str:
    """Assemble the full debugging context (the user's "let the debugger know
    everything"): the request, the boss brief + immovable frames, the model JSON, the
    URDF, every part's script, the worst-first conflict list, and (when a physics test
    ran) the MuJoCo metrics so the debugger can name a floating/transmission/jam fault,
    not only geometric overlap."""
    scripts_block = "\n\n".join(
        f"# ---- {name} ----\n{src}" for name, src in part_scripts.items()) or "(none)"
    conflicts_block = "\n".join(f"  {i+1}. {d}" for i, d in enumerate(conflicts_desc)) \
        or "  (no rigid overlap — the fault is dynamic; see the physics metrics)"
    frozen = sorted(frozen_links or [])
    frozen_block = (", ".join(f"'{n}'" for n in frozen)
                    if frozen else "(none)")
    phys_block = ""
    if physics_metrics:
        m = physics_metrics
        hints = []
        if m.get("exploded"):
            hints.append("the assembly EXPLODED/JAMMED (a part flew off or locked)")
        if m.get("moved_count") == 0 and (m.get("input_travel") or 0) > 0.05:
            hints.append("the input turned but NOTHING downstream moved — a "
                         "TRANSMISSION failure (gears not touching / too far apart)")
        if m.get("output_reached") is False:
            hints.append("motion did not REACH the output part")
        if (m.get("input_travel") or 0) <= 0.05:
            hints.append("the input barely turned under torque — a JAM at the input")
        if float(m.get("max_tilt_deg") or 0) > 20:
            hints.append(f"the assembly TILTED {m.get('max_tilt_deg')}deg — unstable/toppling")
        phys_block = (
            "\n\nPHYSICS TEST RESULT (MuJoCo, pure contact under gravity):\n"
            f"  {m}\n"
            + ("  Likely fault(s): " + "; ".join(hints) + "\n" if hints else ""))
    return f"""\
The user asked for this machine:
"{user_prompt}"

FROZEN interface-frame parts (do NOT move or edit these links — fix the OTHER part in a
conflicting pair): {frozen_block}

This subassembly's brief (from the boss):
{brief or "(none)"}

IMMOVABLE INTERFACE FRAMES for this subassembly:
{_fmt_mount_frames(frames)}

RIGID CONFLICTS detected in the assembled subassembly (worst first):
{conflicts_block}
{phys_block}
CURRENT KINEMATIC MODEL (links + poses + per-part dof) as JSON:
{model_json}

CURRENT URDF:
{urdf_text}

CURRENT PART SCRIPTS (one build_<name>() per part):
{scripts_block}

Fix the fault(s) with the SMALLEST change — move a part via its pose, or reshape a
part's script, per the rules. Return your NOTES, then the sentinel line, then the single
JSON patch object."""


def build_subdebugger_json_from_notes(notes: str) -> str:
    """Regeneration message when the JSON overran the output cap: hand the notes back and
    ask for ONLY the JSON patch now."""
    return f"""\
Here is the plan you already worked out (your NOTES):

{notes}

Now output ONLY the single JSON patch object that implements this plan, following the
shape exactly. Do NOT repeat the notes, do NOT include the `{JSON_SENTINEL}` line, and do
NOT use markdown fences — output only the JSON object."""
