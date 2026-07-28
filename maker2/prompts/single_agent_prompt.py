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
    a.add(g1, "input_gear|dof=spin|driver=True|mesh_id=stage1|mount=baseplate")
    g2 = make_gear(2.0, 40, 6, 8).moved(Location((20, 0, 6)))
    a.add(g2, "output_gear|dof=spin|mesh_id=stage1|mount=baseplate")
    return a.build()

PART LABEL CONVENTION (critical — this is how downstream physics reads your intent). The FIRST
field of each `a.add(part, "<label>")` label is the URDF-safe part NAME; the rest are `key=value`
metadata separated by `|`:
  - `dof=fixed|spin|free`  — `spin` ONLY for a part that is MEANT to rotate to transmit motion:
                             a gear, a pinion, a wheel, or the shaft/arbor that carries them.
                             EVERYTHING ELSE is `fixed`, INCLUDING every accessory that merely
                             sits on a rotating shaft — thrust washers, spacers, pedestals,
                             collars, retainers, caps, hands. A `fixed` accessory is welded to its
                             mount and does not move on its own. Never mark a washer/spacer/
                             pedestal/retainer/cap/hand `spin`: making a passive collar spinnable
                             lets it grind against the shaft it hugs and the contact solver turns
                             that grind into friction that STALLS the driver — the whole train
                             then reads as "does not turn". If in doubt, it is `fixed`.
  - `driver=True`          — on the ONE input part the physics test turns.
  - `mesh_id=<id>`         — put the SAME id on the TWO gears meant to mesh, so the transmission
                             check pairs them. A gear with no partner needs no mesh_id.
  - `spin_axis=z`          — for a spin part; round parts are built along local +Z, so this is z.
  - `mount=<part_name>`    — the NAME of the part this one is physically carried by / mounted on
                             (a gear mounts on its arbor or the plate; an arbor on the base; a
                             bearing on the plate). This declares the load path — it does NOT move
                             anything (you already placed the part with `.moved()`), it just tells
                             the physics which part holds this one so it is anchored, not floating.
                             The base/mainplate itself has NO mount (it rests on the ground). Every
                             OTHER part MUST name a mount, and that mount must be an existing part.
Give every part a UNIQUE name. Build each part at the ORIGIN along local +Z, then `.moved(
Location((x, y, z)))` to its GLOBAL millimetre position — YOU choose the layout so parts touch
where they must and clear where they must not.

{_MODELING_METHOD}

DISCIPLINE:
- Include EVERY real part; do not merge a shaft into a gear or drop a bearing.
- Space coaxial parts (front bearing | gear | pinion | rear bearing) at DISTINCT axial stations
  so their solids never interpenetrate. Compute each station from the PREVIOUS part's real top
  face — and remember `Cylinder`/`Box` are CENTERED unless you pass
  `align=(Align.CENTER, Align.CENTER, Align.MIN)` (see MODELING METHOD above). Getting this wrong
  leaves every stacked part floating half its height above what it is supposed to rest on, which
  the gravity support test WILL catch and send back to you.
- A passive accessory that hugs a shaft (thrust washer, spacer, pedestal, collar, cap) must be
  `dof=fixed` AND sit at its OWN axial station flush against the face it backs — it must NOT
  overlap the shaft cross-section it rings. A washer buried a few mm into the shaft it surrounds
  becomes a deep coaxial contact that brakes the driver; give it a bore clearing the shaft and a
  z that places it beside, not inside, the part it serves.
- OMIT torque-lock hardware (keys, keyways, setscrews, retaining collars) — pure contact does
  not use them and they always clash the gear hub.
- Use capitalized True/False/None (this is Python, not JSON).
- Give EVERY part except the base a `mount=<part_name>`: the base holds the plates/arbors, the
  arbors hold the gears. This is what anchors the whole train to the world so it cannot drift.

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


def build_single_agent_geometry_feedback(findings: str, best_code: str | None = None) -> str:
    """Feedback from the text-to-cad inspect self-check / precheck: concrete geometry faults
    (interpenetration, gears not one center-distance apart, a part floating) the agent must fix
    by editing the script. On a regression, best_code is the fewest-fault version so far —
    hand it back so the agent refines THAT instead of its own worse attempt."""
    rollback = ""
    if best_code:
        rollback = (
            "\n\nIMPORTANT — YOUR LAST EDIT MADE THE GEOMETRY WORSE (more faults than before). "
            "Do NOT continue from it. Below is the version with the FEWEST faults so far. START "
            "FROM THIS EXACT CODE and make only the smallest change that fixes the faults listed "
            "above — do NOT add parts, do NOT restructure:\n\n"
            f"```python\n{best_code}\n```\n")
    return f"""The machine built, but a geometry check found problems:

{findings}

Fix EXACTLY these parts and NOTHING else. Make the SMALLEST possible edit — change only the
position or length of the parts named above. Do NOT add new parts, do NOT rename or remove
parts, do NOT restructure the machine, do NOT touch any part not named above. A floating part
is fixed by moving it until it TOUCHES the part it mounts on, or by lengthening the shaft/
arbor/tube so it reaches — NOT by adding a support under it. Keep everything that already
works.{rollback}
Return the COMPLETE corrected ```python block for build_machine()."""


def _drift_symptom(reason: str | None) -> bool:
    """The diagnosis describes the WHOLE assembly sliding/drifting when driven (i.e. it is
    not anchored to the world), as opposed to a local overlap or gear-spacing fault."""
    r = (reason or "").lower()
    return any(k in r for k in ("drift", "slid", "sliding", "unanchored", "off the base",
                                "off its base", "across the bench", "not fixed"))


def build_single_agent_physics_feedback(summary: str, metrics: dict, diagnosis: dict,
                                        stability: dict | None = None,
                                        best_code: str | None = None,
                                        regressed: bool = False,
                                        fixed_fits: list | None = None,
                                        broke_fits: list | None = None) -> str:
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

    # BORE FITS FIRST. These are MEASURED off the STLs while the MJCF is built, before a
    # single sim step, so unlike the VLM's read of the recording they are available on
    # iteration 0 and they are exact. They also outrank everything below: a part whose
    # bore is smaller than the shaft it is supposed to sit on cannot be assembled at all,
    # so no amount of repositioning fixes it, and the train stays jammed while the agent
    # chases whatever the camera happened to show (usually "the arbor pokes through the
    # plate into the ground" — true, but not what is stopping the machine).
    # CREDIT THE EDITS THAT WORKED. The agent only ever saw one number go down and had no
    # way to tell WHICH of its changes did it, so it reverted good work along with bad
    # (measured: it opened the centre bores 1.1 -> 4.3mm and cleared every minute_pinion
    # fault, then put them back at 1.1 the next round and the input travel fell from 0.016
    # to 0.003 rad). Naming the fixed ones makes "keep this" actionable instead of a plea.
    if fixed_fits:
        lines.append("- YOUR LAST EDIT FIXED THESE — KEEP THEM EXACTLY AS THEY ARE, do not "
                     "revert these bores or move these parts:")
        for part, shaft in fixed_fits[:10]:
            lines.append(f"    * {part} now fits on {shaft}")
    if broke_fits:
        lines.append("- YOUR LAST EDIT BROKE THESE (they fitted before):")
        for part, shaft in broke_fits[:10]:
            lines.append(f"    * {part} no longer fits on {shaft}")

    # A GEAR TOO LOOSE TO BE DRIVEN. Distinct from a jam and from an impossible fit: the
    # geometry assembles, the mesh is correct, every constraint is healthy — the shaft
    # simply does not grip the gear, so the input turns against nothing. Deterministic and
    # measured while the MJCF is built, so it is available on iteration 0.
    loose = (m.get("loose_gears") or [])
    if loose:
        lines.append("- NOT DRIVEN — these gears sit too loosely on their shafts to be "
                     "turned by them (measured from your geometry):")
        for g in loose[:8]:
            tail = ("  <- this is the INPUT shaft, so NOTHING downstream can turn"
                    if g.get("driver_shaft") else "")
            lines.append(f"    * {g['gear']} rides {g['shaft']} with "
                         f"{g['clearance_mm']}mm clearance; a shaft only drives a gear at "
                         f"<= {g['press_fit_max_mm']}mm{tail}")

    fits = (m.get("bore_fit_faults") or [])
    if fits:
        lines.append("- IMPOSSIBLE FITS (measured from your geometry, not from the video). "
                     "Each of these parts is declared ON a shaft it cannot physically go "
                     "onto, because its hole is smaller than that shaft:")
        for f in fits[:10]:
            note = ("  <- the WHOLE PART is thinner than the shaft: no bore can fit, the "
                    "part itself must get bigger (or it does not belong on this shaft)"
                    if f.get("impossible") else "")
            lines.append(f"    * {f['part']}: bore r={f['bore_mm']}mm, but it sits on "
                         f"{f['shaft']} whose outer r={f['shaft_r_mm']}mm "
                         f"(part outer r={f['part_outer_mm']}mm){note}")
    unsupported = st.get("support_faults") or []
    if unsupported:
        lines.append("- STAGE 1 (SUPPORT) FAILED. Every mount weld was dissolved and the "
                     "assembly settled under gravity; these parts fell, so nothing real "
                     "holds them up:")
        lines += [f"    * {f}" for f in unsupported[:8]]
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
    if loose:
        guidance = (
            "MAKE THE DRIVEN GEARS GRIP THEIR SHAFTS — this is why the input turns and "
            "nothing follows. The fit is decided by ONE measured number:\n"
            "    clearance = bore_radius - shaft_outer_radius\n"
            "    0 < clearance <= 0.10mm  -> PRESS FIT: the gear turns WITH the shaft\n"
            "        clearance >  0.10mm  -> RUNNING FIT: it spins freely, undriven\n"
            "For EACH gear listed above, shrink its bore to `shaft_outer_r + 0.05` so it is "
            "a press fit. Do NOT move anything, do NOT change tooth counts or centre "
            "distances — the mesh geometry already checks out. Only the bores are wrong.\n"
            "Leave a part that is SUPPOSED to rotate freely (an hour wheel riding the "
            "centre arbor) well above 0.10mm — that free rotation is what makes the 12:1.")
    elif fits:
        guidance = (
            "FIX THE IMPOSSIBLE FITS FIRST — nothing downstream can work while a part is "
            "declared on a shaft it cannot go onto. This is measured geometry, not a guess, "
            "and it is why the train is jammed. For EACH part listed above:\n"
            "- Take the bore radius FROM the shaft it rides: `bore_r = <shaft>_outer_r + 0.05`. "
            "Never write a bore as a bare number, and never size it before the shaft exists.\n"
            "- If the note says the WHOLE PART is thinner than the shaft, the bore cannot be "
            "cut at all: either make that part large enough to ring the shaft (outer radius "
            "clearly bigger than the shaft's), or take it off that shaft — a 3mm jewel cannot "
            "sit on a 7mm arbor.\n"
            "- If the 'shaft' is a WHEEL or a GEAR with a big radius (say over 8mm), the "
            "mount itself is wrong, not the bore: bearings, washers, spacers and hands ride "
            "on an ARBOR or a PIPE, never on the rim of a wheel. Re-`mount=` that part onto "
            "the thin arbor/pipe that runs through the wheel, and size its bore from THAT.\n"
            "- Do NOT move parts around to work past this, and do NOT touch anything not "
            "listed above. Fix the radii, then the rest of the diagnosis is worth reading.\n"
            "- KEEP every fit that already works. Changing a bore that was not listed as a "
            "fault can only break it.")
    elif unsupported:
        guidance = (
            "FIX SUPPORT FIRST — the parts listed above are floating. Each is declared on a "
            "mount it does not physically touch, so the label is the only thing holding it. "
            "For EACH one: move it down/along until its solid actually MEETS the part named as "
            "its mount, or lengthen that shaft/arbor/tube until it reaches the part. Do NOT add "
            "a new support part, do NOT move anything not listed above.")
    elif stability_failed:
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
    elif _drift_symptom(reason):
        # Settled fine but the WHOLE assembly slides/drifts when driven — it is not anchored
        # to the world. This is the failure that kept recurring; nail it every iteration.
        guidance = (
            "ANCHOR THE WHOLE MOVEMENT — the gear train is NOT fixed to the world, so driving "
            "the input makes the entire tower slide/drift across the bench instead of the "
            "gears turning in place. This is the ONE thing to fix now:\n"
            "- There MUST be a base/mainplate labelled dof=fixed that rests on the ground; it "
            "is the anchor everything else hangs off.\n"
            "- Every arbor/shaft/bridge must be positioned relative to that fixed base so the "
            "train is held in place — nothing should be floating free.\n"
            "- Only the gears/arbors that are meant to rotate are dof=spin; plates, bridges, "
            "bearings, the base are dof=fixed.\n"
            "Do not touch gear spacing until the assembly stays put under drive.")
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

    # ROLLBACK: this iteration scored WORSE than the best version so far. Rather than let
    # the agent keep drifting from its own worse attempt, hand back the known-best code and
    # tell it to refine THAT — this is what stops the loop from diverging over many rounds.
    rollback = ""
    if regressed and best_code:
        rollback = (
            "\n\nIMPORTANT — YOUR LAST EDIT MADE THINGS WORSE. Do NOT continue from it. Below "
            "is the BEST version so far (the closest to a stable, working machine). START FROM "
            "THIS EXACT CODE and make only the SMALLEST change that fixes the problem above. "
            "Do NOT rewrite the machine, do NOT renumber or drop parts — keep everything that "
            "already works and touch only what the diagnosis points at:\n\n"
            f"```python\n{best_code}\n```\n")

    return f"""The machine was simulated in physics and it did NOT work correctly:

{body}

{guidance}{rollback}
Return the COMPLETE corrected ```python block for build_machine()."""
