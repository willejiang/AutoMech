"""Prompt for the SINGLE-AGENT text-to-cad path: ONE agent authors the WHOLE machine as one
build123d script (no boss, no per-sub managers, no assembler), refines it against the
text-to-cad inspect self-check, and hands off a build_machine() the executor turns into a
KinematicModel. Reuses the distilled build123d modeling method from the manager prompt.
"""
from __future__ import annotations

from .manager_prompt import _MODELING_METHOD

# The whole-machine authoring contract. The agent writes ONE `build_machine()` that returns a
# cadpy AssemblyHelper; make_gear + build123d names + AssemblyHelper are already injected.
SINGLE_AGENT_SYSTEM = f"""\
You are a mechanical CAD engineer. From a one-line product description you author the COMPLETE
machine as ONE parametric build123d Python script — every real part (gears, pinions, shafts,
arbors, bearings, plates, housings, hands) in ONE assembly. There is no boss, no subassembly
split, no separate assembler: YOU own the whole geometry and the whole spatial layout.

PURE CONTACT — NO JOINTS, NO MOTORS. The finished machine is simulated in MuJoCo as rigid
bodies under GRAVITY; motion transmits ONLY by physical contact (meshing gear teeth push one
another). So parts that must interact MUST actually touch: meshing gears sit exactly one
pitch-center-distance apart (center distance = module*(z1+z2)/2) with teeth engaged, and every
part rests on real support.

COMPUTE THE NUMBERS FIRST — DO NOT GUESS COORDINATES. Before you place anything, DERIVE the
drivetrain arithmetic at the TOP of your script as plain Python variables, then compute EVERY
gear position FROM those variables. This is the single most important step: a gear placed at a
guessed coordinate will NOT be one center-distance from its partner, so the teeth never engage
and the mechanism transmits nothing (the physics test then reports 0 downstream motion). Author
it like an engineer:
  M = 0.8                      # ONE module shared by all meshing gears (so any pair can mesh)
  Z_INPUT, Z_OUTPUT = 12, 36   # tooth counts set the reduction ratio (=Z_OUTPUT/Z_INPUT)
  def pitch_r(z):     return M * z / 2.0
  def center_dist(za, zb): return M * (za + zb) / 2.0
  cd1 = center_dist(Z_INPUT, Z_OUTPUT)        # the EXACT distance between the two gear centers
Then place the two meshing gears their centers EXACTLY `cd1` apart (e.g. one at x=0, its partner
at x=cd1, or split along a chosen axis), and build each gear with `make_gear(M, z, face_width,
bore)` using the SAME M. A gear train chains this: each stage's center distance comes from its
own tooth counts, and a shaft carrying two gears puts them at DISTINCT axial stations. NEVER
write a gear-center coordinate as a bare number you eyeballed — it must be an expression over M
and the tooth counts (pitch_r / center_dist). Bearings, plates and housings can use round
sizes, but every GEAR-to-GEAR spacing is computed.

OUTPUT: exactly ONE ```python code block defining `build_machine()` that returns a cadpy
`AssemblyHelper`. These names are ALREADY INJECTED — do NOT import them:
  AssemblyHelper, make_gear, Box, Cylinder, BuildPart, BuildSketch, Circle, Polygon, extrude,
  Location, Plane, Align, Mode, and build123d as b3d.

HOW TO ASSEMBLE (cadpy AssemblyHelper):
    a = AssemblyHelper("machine")
    plate = Box(60, 40, 4)
    a.add(plate, "baseplate|dof=fixed")
    g1 = make_gear(2.0, 20, 6, 8).moved(Location((-20, 0, 6)))   # module, teeth, face_width, bore
    a.add(g1, "input_gear|dof=spin|driver=True|mesh_id=stage1")
    g2 = make_gear(2.0, 40, 6, 8).moved(Location((20, 0, 6)))
    a.add(g2, "output_gear|dof=spin|mesh_id=stage1")
    return a.build()

PART LABEL CONVENTION (critical — this is how downstream physics reads your intent). The FIRST
field of each `a.add(part, "<label>")` label is the URDF-safe part NAME; the rest are `key=value`
metadata separated by `|`:
  - `dof=fixed|spin|free`  — fixed for plates/housings/bearings; spin for gears/wheels/shafts.
  - `driver=True`          — on the ONE input part the physics test turns.
  - `mesh_id=<id>`         — put the SAME id on the TWO gears meant to mesh, so the transmission
                             check pairs them. A gear with no partner needs no mesh_id.
  - `spin_axis=z`          — for a spin part; round parts are built along local +Z, so this is z.
Give every part a UNIQUE name. Build each part at the ORIGIN along local +Z, then `.moved(
Location((x, y, z)))` to its GLOBAL millimetre position — YOU choose the layout so parts touch
where they must and clear where they must not.

{_MODELING_METHOD}

DISCIPLINE:
- Include EVERY real part; do not merge a shaft into a gear or drop a bearing.
- Space coaxial parts (front bearing | gear | pinion | rear bearing) at DISTINCT axial stations
  so their solids never interpenetrate.
- OMIT torque-lock hardware (keys, keyways, setscrews, retaining collars) — pure contact does
  not use them and they always clash the gear hub.
- Use capitalized True/False/None (this is Python, not JSON).

Respond with a short NOTES plan, then the ONE ```python block."""


def build_single_agent_user(product_prompt: str) -> str:
    return f"""Author the complete machine for this product as one build_machine() script.

Product: "{product_prompt}"

Write NOTES (the parts you will build, their rough sizes, dof, and how they mesh/touch), then
the single ```python block."""


def build_single_agent_repair(error: str) -> str:
    """Feedback after the script failed to evaluate (a build123d/cadpy error)."""
    return f"""Your build_machine() script could not be evaluated:

{error}

Fix ONLY what this error points to and return the COMPLETE corrected script — the single
```python block defining build_machine(). Keep every working part; do not restate the notes."""


def build_single_agent_geometry_feedback(findings: str) -> str:
    """Feedback from the text-to-cad inspect self-check / precheck: concrete geometry faults
    (interpenetration, gears not one center-distance apart, a part floating) the agent must fix
    by editing the script."""
    return f"""The machine built, but a geometry check found problems:

{findings}

Edit the smallest part of your build_machine() to fix these — move a part to a clear axial
station, set two meshing gears exactly one center-distance apart, or reposition a floating part
onto real support. Return the COMPLETE corrected ```python block for build_machine()."""


def build_single_agent_physics_feedback(summary: str, metrics: dict, diagnosis: dict,
                                        stability: dict | None = None) -> str:
    """Feedback after a PHYSICS run: the machine was simulated under gravity + drive, and it
    did not work as intended. Hand the agent the measured transmission metrics + the VLM's
    read of the recording so it can fix the mechanism (usually a mesh that does not engage)."""
    m = metrics or {}
    st = stability or {}
    moved = m.get("moved_count")
    watched = m.get("watched_count")
    it = m.get("input_travel")
    exploded = m.get("exploded")
    stability_failed = (str(st.get("verdict", "PASS")).upper() == "FAIL"
                        or bool(st.get("exploded")))
    cause = (diagnosis or {}).get("cause")
    reason = (diagnosis or {}).get("reason")
    lines = [f"PHYSICS RESULT: {summary}"]
    if stability_failed:
        lines.append(f"- STAGE 1 (STABILITY) FAILED: dropped on the bench under gravity with "
                     f"NO drive, the machine did not hold together "
                     f"(max_disp={st.get('max_disp_m')}m, displaced={st.get('displaced_parts')}).")
    if it is not None:
        lines.append(f"- the driver turned {it} rad but {moved or 0}/{watched or 0} downstream "
                     f"parts moved.")
    if exploded:
        lines.append("- the assembly flew apart (parts ejected from their start pose).")
    if cause and cause != "none":
        lines.append(f"- diagnosis cause: {cause}.")
    if reason:
        lines.append(f"- DIAGNOSIS (this is the measured root cause — fix THIS): {reason}")
    body = "\n".join(lines)

    # The fix guidance must follow the DIAGNOSIS, not a fixed guess. STABILITY comes FIRST:
    # a machine that falls apart just settling can't be judged on function at all. Then an
    # explosion is a start-pose overlap (not a center-distance problem); only a DEAD/jammed
    # train (turned but nothing moved, no explosion) points at gear spacing.
    if stability_failed:
        guidance = (
            "FIX STABILITY FIRST — nothing else matters until the machine can EXIST on the "
            "bench. Dropped under gravity with no drive, it fell apart. It must sit as a "
            "coherent, self-supporting assembly:\n"
            "- Give it a solid BASE (a plate/mainplate) that is dof=fixed and rests on the "
            "ground, and mount everything off that base.\n"
            "- Remove any START-POSE OVERLAPS (two solids intersecting fling apart the "
            "instant contact is computed): shafts must not poke through the plates they "
            "pass — shorten them or fix their z; separate parts must not intersect.\n"
            "- Only once it settles intact does the gear-train function get tested.")
    elif exploded:
        guidance = (
            "The assembly EXPLODED — parts overlapped at the START pose and the contact "
            "solver flung them apart. This is NOT a gear center-distance problem. Follow "
            "the DIAGNOSIS above literally:\n"
            "- If a SHAFT/ARBOR pokes axially through a plate/baseplate/bearing (a coaxial "
            "contact deep inside the bore), the shaft is too LONG or its z is wrong — "
            "SHORTEN that shaft or move its z so its end-face clears the part it passes "
            "through. Do NOT widen bores, do NOT move it off-axis.\n"
            "- If two SEPARATE parts were placed intersecting, move one so their solids no "
            "longer overlap.\n"
            "- Meshing gears should touch only at their teeth (one center-distance apart), "
            "NOT be buried in each other.")
    else:
        guidance = (
            "The train did not transmit motion though nothing exploded — the usual cause is "
            "meshing gears NOT exactly one center-distance apart, so their teeth never "
            "engage. Recompute each meshing pair's center distance = module*(z1+z2)/2 and "
            "place the gear centers EXACTLY that far apart, using the SAME module for both. "
            "If a gear is the wrong size to reach its partner, fix its tooth count or position.")

    return f"""The machine was simulated in physics and it did NOT work correctly:

{body}

{guidance}
Return the COMPLETE corrected ```python block for build_machine()."""
