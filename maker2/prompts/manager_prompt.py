"""Manager agent prompt: decompose a product prompt into a KinematicModel.

Two authoring formats, selected by ``settings.manager_ir`` (default ON):
  * CONNECTION GRAPH (on): parts + MATES, solved by mate_solver. IR_SCHEMA_TEXT + IR_FEWSHOT_JSON.
  * MJCF SKELETON (off, the fallback): parts + an MJCF XML skeleton. SCHEMA_TEXT + FEWSHOT_JSON.
Callers pass ``manager_ir`` (from settings) to the builder functions; the module-level
MANAGER_SYSTEM* constants are prebuilt for both and selected by ``manager_system(manager_ir)``.
"""

from __future__ import annotations

from ..twophase import JSON_SENTINEL
from .schema import (SCHEMA_TEXT, FEWSHOT_PRODUCT, FEWSHOT_JSON, MJCF_SENTINEL,
                     IR_SCHEMA_TEXT, IR_FEWSHOT_JSON)


# The shared preamble (identity + pure-contact rules), format-agnostic. The output-contract
# TAIL differs by format and is appended per variant below.
_MANAGER_PREAMBLE = """\
You are the MANAGER of an automated CAD pipeline. You turn a one-line product
description into a complete mechanical decomposition: every part the product is
made of, where each part sits, and how each part is allowed to move.

Your decomposition is a CONTRACT. Downstream, one CAD worker builds each part in
isolation (it sees only that part's description and origin_note, never the other
parts), and an assembler positions the parts. So you alone own all geometry
relationships and the spatial layout.

PURE CONTACT — THERE ARE NO JOINTS AND NO MOTORS. This is the most important thing
to understand. The finished machine is simulated in MuJoCo as rigid bodies resting
under GRAVITY. Motion is transmitted ONLY by physical contact: meshing gear teeth
push one another, a cam lifts a follower, a falling weight drives a train. Nothing
is moved by a motor and nothing is held in place by an invisible joint.
- Instead of joints, each PART declares how it moves via `dof`:
    "fixed" = welded in place (frames, housings, brackets, the base).
    "spin"  = rotates on an implied axle along `spin_axis` (gears, wheels, rotors,
              shafts). The axle is implicit — you do NOT model a joint for it.
    "free"  = a fully free 6-DOF body (rare; a loose part).
- The physics test spins the ONE part you mark `driver": true` and checks that
  motion propagates by contact. So parts that must interact MUST actually touch:
  meshing gears exactly one pitch-center-distance apart with teeth engaged, and
  every part on real support (under gravity, anything unsupported falls).
- When several parts sit on ONE base (a plate with multiple bosses / bearing seats /
  pillars), mate EACH to the base independently — the base is the single anchor and the
  parts fan off it in parallel. Do NOT chain them to each other (boss → next boss); that
  fixes a part by two paths and over-constrains it. Their spacing comes from where each
  mates on the base, not from mating one to the next.

Decompose thoroughly but sensibly: split the product into all the distinct rigid
parts INCLUDING ALL INTERNAL HARDWARE (every gear, shaft, arbor, bearing/jewel,
pin, spring), give each a clear build brief and a sensible local origin, set each
part's `dof`. DO NOT replace a real shaft or bearing with anything virtual — a
spinning part turns on a real shaft in a real bearing, and BOTH are their own parts
(the shaft is dof "spin", the bearing is dof "fixed").

OMIT TORQUE-LOCK HARDWARE on a gear/wheel-on-shaft mount: do NOT add a KEY, keyway,
setscrew, spline, or retaining collar to lock a gear/pinion/wheel to its shaft. This is
a pure-CONTACT sim — torque transfers gear-to-gear by tooth contact, not through a key,
so a key does nothing the physics uses. Worse, a key must sit at the shaft surface, which
is INSIDE the solid gear hub, and the gear is a frozen interface part with no keyway slot
— so the key ALWAYS interpenetrates the gear ~80% and no debugger can clear it (the gear
can't be re-cut). Mount a gear on its shaft by placing it COAXIALLY at its axial position;
the gear + shaft simply share the axle. Skip keys, keyways, and retaining collars entirely.

HIGHLIGHT:
1. This CAD is used for physics simulation AND actual production, so care about the
   SMALLEST piece — each gear, drive shaft, hinge, bearing.
2. For anything that must move, model it as a real part with dof "spin"/"free" and
   place it in actual contact with the parts it drives — motion comes from contact,
   not from a joint or a motor.
3. Always give every part a real-world material color.
4. If the EVALUATOR returns FALSE with feedback, forget every other guideline and
   strictly follow the evaluator.

AVOID: cute/decorative structure that isn't a real functional part.

CASE EXAMPLE:
    USER ASK: "I want you to build a clock"
    GOOD DESIGN: not only the APPEARANCE — each gear, shaft, and bearing is a single
        physical part; the gears mesh one pitch-center-distance apart so torque
        transmits by contact; the going train rests in its frame under gravity.
        Marking the input arbor `driver": true` spins the train.
    BAD CASE: only the clock's outer appearance; internal gears/shafts/bearings are
        missing or merged, or gears placed not touching so nothing transmits."""


# Connection-graph output contract (DEFAULT).
_IR_OUTPUT_TAIL = f"""\
Respond in TWO parts, in this exact order:
1. NOTES — a concise plaintext plan: the parts you will emit (every gear, shaft,
   bearing, pin, etc.), their rough sizes, each part's dof, and how they CONNECT to
   one another (which part mates to which, and how). This is your scratchpad and is
   SAVED AS MEMORY; if you run out of room you will be asked to CONTINUE these notes,
   so list the load-bearing parts and connections FIRST and keep every real part.
2. A line containing exactly:  {JSON_SENTINEL}
   then the single connection-graph JSON object (parts + mates). No markdown fences
   after {JSON_SENTINEL} — only the one JSON object."""


# MJCF-skeleton output contract (the --no-manager-ir fallback).
_MJCF_OUTPUT_TAIL = f"""\
Respond in THREE parts, in this exact order:
1. NOTES — a concise plaintext plan: the parts you will emit and how they are placed
   relative to one another. SAVED AS MEMORY; list load-bearing parts FIRST.
2. A line containing exactly:  {JSON_SENTINEL}
   then BLOCK 1 (the PARTS JSON object), then a line containing exactly:  {MJCF_SENTINEL}
   then BLOCK 2 (the MJCF XML skeleton). Every <body> carries an XML comment on its
   role/frame/meshing, and every "spin" part has a <joint type="hinge"> matching its
   spin_axis. No markdown fences after {JSON_SENTINEL}."""


MANAGER_SYSTEM = f"{_MANAGER_PREAMBLE}\n{IR_SCHEMA_TEXT}\n\n{_IR_OUTPUT_TAIL}"
MANAGER_SYSTEM_MJCF = f"{_MANAGER_PREAMBLE}\n{SCHEMA_TEXT}\n\n{_MJCF_OUTPUT_TAIL}"


def manager_system(manager_ir: bool = True) -> str:
    """The manager system prompt for the chosen authoring format."""
    return MANAGER_SYSTEM if manager_ir else MANAGER_SYSTEM_MJCF


def build_manager_json_from_notes(notes: str, manager_ir: bool = True) -> str:
    """Regeneration message: the manager already wrote its decomposition as NOTES
    (saved when its payload overran the output cap); hand the notes back and ask for
    ONLY the payload now, so the whole output budget goes to it — no dropped parts."""
    if manager_ir:
        return f"""\
Here is the decomposition you already worked out (your NOTES):

{notes}

Now output ONLY the connection-graph JSON object for this decomposition, in full, following
the schema exactly: EVERY part listed in the notes (all gears, shafts, bearings, pins) with
its `dof`/`spin_axis`/`size_mm`/`color`/`material`, and the `mates` that connect them.

Do NOT repeat the notes, do NOT include the `{JSON_SENTINEL}` line, and do NOT use markdown
fences — output only the single JSON object."""
    return f"""\
Here is the decomposition you already worked out (your NOTES):

{notes}

Now output ONLY the two payload blocks for this decomposition, in full, following the
schema exactly:
1. BLOCK 1 — the PARTS JSON object: include EVERY part listed in the notes (all gears,
   shafts, bearings, pins) with its `dof`, `spin_axis`, `size_mm`, `color`, and `material`.
2. A line containing exactly:  {MJCF_SENTINEL}
3. BLOCK 2 — the MJCF XML skeleton: one nested <body> per part (matching the PARTS names),
   each with an XML comment on its role/frame/meshing, a <joint type="hinge"> for every
   "spin" part, and a <freejoint> for every "free" part.

Do NOT repeat the notes, do NOT include the `{JSON_SENTINEL}` line, and do NOT use markdown
fences — output only the PARTS object, the `{MJCF_SENTINEL}` line, and the XML skeleton."""


def build_manager_user(product_prompt: str, has_image: bool = False,
                       manager_ir: bool = True) -> str:
    """The manager's user message: the product + a worked example (in the active format).

    When ``has_image`` is set, an image is attached to this message by the caller
    (Conversation.add_user_message(images=...)); the wording then makes the IMAGE
    the authoritative source and treats the text as a hint.
    """
    example = IR_FEWSHOT_JSON if manager_ir else FEWSHOT_JSON
    if has_image:
        task = (
            "NOW DO THIS ONE\n"
            "The product is shown in the ATTACHED IMAGE. Decompose the product you\n"
            "SEE in the image. Use this text only as a hint about what it is:\n"
            f'"{product_prompt}". Reproduce the parts, proportions, and\n'
            "articulation visible in the image."
        )
    else:
        task = f'NOW DO THIS ONE\nProduct: "{product_prompt}"'
    return f"""\
Decompose this product into a kinematic model following the schema exactly.

EXAMPLE
Product: "{FEWSHOT_PRODUCT}"
Output:
{example}

{task}
Output:"""


def build_manager_repair(error: str, manager_ir: bool = True) -> str:
    """Feedback message appended after a parse/validation failure."""
    if manager_ir:
        return f"""\
Your previous response could not be used:

{error}

Return a corrected decomposition that fixes this: the single connection-graph JSON object
(parts + mates). Output ONLY that one JSON object, no prose, no markdown fences."""
    return f"""\
Your previous response could not be used:

{error}

Return a corrected decomposition that fixes this: the PARTS JSON object, then a line with
exactly `{MJCF_SENTINEL}`, then the MJCF XML skeleton. Output ONLY those, no prose, no
markdown fences."""


def build_manager_repair_diff(error: str, delta_note: str, manager_ir: bool = True) -> str:
    """Diff-carrying repair feedback (C13): the same repair request, PLUS a one-line
    'gradient' telling the manager whether its last change got CLOSER to buildable and
    which specific checks moved. This turns a blind retry into a guided one — the manager
    sees the direction, not just another error string. ``delta_note`` comes from
    badness.format_delta over consecutive attempts. Wraps build_manager_repair so a later
    refactor of the base message still composes."""
    base = build_manager_repair(error, manager_ir=manager_ir)
    return (base + "\n\n"
            "PROGRESS SIGNAL (lower is closer to a buildable model):\n"
            f"  {delta_note}\n"
            "If your last change made things WORSE, do NOT repeat it — revert that idea and "
            "try a different fix. If it helped, keep going in that direction. Reduce the "
            "specific checks named above.")


def build_manager_coarser(error: str) -> str:
    """Feedback message appended after the response overran the output cap.

    The full-detail decomposition was too large to fit in one response, so this
    OVERRIDES the fine-grained HIGHLIGHT guidance for the retry: ask for a
    smaller tree of major parts that will fit under the cap.
    """
    return f"""\
Your previous response was TOO LONG and was cut off before it finished, so none
of it could be used:

{error}

There is a hard limit on how much you can output in one response. You MUST make
this decomposition SMALLER so it fits. For THIS response only, override the
"smallest piece / each gear" guidance and decompose COARSELY:

- Emit AT MOST 40 parts total.
- Do NOT break parts down to individual gears, fasteners, or tiny internals.
  Represent each sub-mechanism as ONE part (e.g. a single "landing_gear" part,
  not separate struts + axles + wheels + bolts; a single "engine" part, not its
  internal parts).
- Keep every description and origin_note to ONE short sentence.
- Still cover all the MAJOR structural and moving parts, set each part's `dof`,
  and place them as nested bodies in the skeleton.

Output ONLY the PARTS JSON object, a line with exactly `{MJCF_SENTINEL}`, then the MJCF
XML skeleton — no prose, no markdown fences."""


def build_manager_evaluator_feedback(feedback: str) -> str:
    """Follow-up message delivering the evaluator's FALSE verdict to the manager.

    Used on loop iterations after the first: the evaluator reviewed the rendered
    CAD against the user's request, judged it NOT good enough, and returned
    concrete suggestions. The MANAGER_SYSTEM prompt already says to obey the
    evaluator over every other guideline, so this just hands over the verdict and
    asks for a fresh full decomposition that addresses it.
    """
    return f"""\
An EVALUATOR reviewed the CAD built from your previous decomposition (rendered
from six viewpoints) against the user's request and judged that it DID NOT PASS.

The evaluator's required changes:

{feedback}

Regenerate the COMPLETE decomposition for the SAME product, strictly applying
every change above. This OVERRIDES any conflicting earlier guidance. Keep the
same schema, units, and origin contract (PARTS object + MJCF skeleton + per-part dof;
motion is by contact under gravity, no joints or motors).

Output ONLY the PARTS JSON object, a line with exactly `{MJCF_SENTINEL}`, then the MJCF
XML skeleton — no prose, no markdown fences."""


def build_manager_prior_model(prior_model_json: str) -> str:
    """Hand the manager the PREVIOUS turn's model as the starting point.

    Used for multi-turn refine: the user is iterating on an existing model, so we
    show it the exact model it produced last time and then (via
    build_manager_refine) ask for a specific change on top of it.
    """
    return f"""\
Here is the CURRENT kinematic model (the one you produced on the previous turn).
It is the starting point for the user's next request:

{prior_model_json}"""


def build_manager_refine(refine_message: str) -> str:
    """Follow-up delivering the USER's refinement request (not the evaluator's).

    Distinct from build_manager_evaluator_feedback: this is the human asking for a
    change to the model they already have (e.g. "make the gears bigger", "add a
    handle"). Modify the current model to satisfy it and return the whole thing.
    """
    return f"""\
The user wants this CHANGE to the current model:

"{refine_message}"

Apply ONLY what this asks for and keep everything else the same wherever possible
(same parts, names, origins, placements, and dof that the change does not touch). You may
add, remove, resize, recolor, or re-place parts as needed to satisfy it. Keep the
same schema, units, and origin contract (PARTS object + MJCF skeleton + per-part dof;
motion is by contact under gravity, no joints or motors).

Output ONLY the COMPLETE updated PARTS JSON object, a line with exactly `{MJCF_SENTINEL}`,
then the updated MJCF XML skeleton — no prose, no markdown fences."""


def build_manager_subassembly(frame_contract, manager_ir: bool = True) -> str:
    """Constrain this manager to build ONE SUBASSEMBLY under the boss's interface/
    frame contract (Stage B of the hierarchy).

    The boss has split a big machine into subassemblies and assigned this one a set
    of INTERFACE FRAMES in GLOBAL coordinates (where this sub sits in the finished
    machine). The manager must (1) build ONLY this subassembly's parts, in its own
    local frame, (2) place a real part at each interface frame, and (3) report which
    part realizes each frame — so the assembler can weld this sub to its neighbors.
    """
    fc = frame_contract
    lines = []
    for fr in getattr(fc, "frames", []):
        x, y, z = fr.xyz_m
        ax, ay, az = fr.axis
        dia = getattr(fr, "shaft_dia_mm", 0.0) or 0.0
        dia_txt = (f", shaft/gear dia {dia:.2f} mm (build the mating shaft/bore/gear "
                   f"to EXACTLY this diameter)") if dia > 0 else ""
        mp_txt = (' — realize this frame ON a bore port you cut in your body at this position '
                  '(name the port yourself); it must sit here, not at the body origin') \
                 if (getattr(fr, "role", "mount") == "mount") else ""
        lines.append(
            f'  - "{fr.name}" (role: {fr.role}): GLOBAL position '
            f'[{x:.4f}, {y:.4f}, {z:.4f}] m, axis [{ax:.3f}, {ay:.3f}, {az:.3f}]'
            f'{dia_txt}{mp_txt}')
    frames_txt = "\n".join(lines) if lines else "  (none)"
    origin = getattr(fc, "global_origin_note", "") or "(the machine's shared origin)"
    nbrs = getattr(fc, "neighbors", []) or []
    if nbrs:
        nb_lines = "\n".join(
            f'  - {n.get("id","?")}: {n.get("function","") or n.get("brief","")[:100]}'
            for n in nbrs)
        neighbors_txt = (
            "\nTHE REST OF THE MACHINE (built by OTHER managers — do NOT build these "
            "parts yourself; this is context so your subassembly mates with them):\n"
            f"{nb_lines}\n")
    else:
        neighbors_txt = ""
    appearance = getattr(fc, "appearance_summary", "") or ""
    appearance_txt = ("\n" + appearance + "\n") if appearance else ""
    move_rule = (
        "- Set each part's `dof` (fixed/spin/free) and connect the parts with MATES so they\n"
        "  mate and transmit motion by CONTACT — there are no joints or motors."
        if manager_ir else
        "- Set each part's `dof` (fixed/spin/free) and nest bodies in the skeleton so they mate\n"
        "  and transmit motion by CONTACT — there are no joints or motors.")
    if manager_ir:
        frames_decl = f"""\
DECLARE EACH INTERFACE FRAME in the top-level `frames` array, pointing at the PART (and one
of its ports) that realizes it:

  "frames": [
    {{"frame": "<interface frame name from the list above>",
      "part": "<the part positioned at this frame>",
      "port": "<a port on that part where the frame sits, e.g. end_b / face_pz / center / bore>"}}
  ]

Emit exactly ONE entry per interface frame listed above, using the EXACT frame name. The
assembler reads these to weld this subassembly to its neighbors, so every listed frame MUST
appear. Place the realizing part (via your mates) so the frame lands at its GLOBAL location.

Output ONLY the single connection-graph JSON object (parts + mates + frames). No prose, no
markdown fences."""
    else:
        frames_decl = f"""\
DECLARE EACH INTERFACE FRAME AS A <site> in the MJCF skeleton, INSIDE the body of the
part that realizes it:

  <body name="<the part at this frame>" pos="..." quat="...">
    ...
    <site name="frame_<interface frame name from the list above>"
          pos="<x y z>"        (the frame point in THIS part's LOCAL frame, METERS)
          euler="<r p y>"/>    (the frame orientation in that part's LOCAL frame, RAD; omit if axis-aligned)
  </body>

Emit exactly ONE <site> per interface frame listed above, and name it
`frame_<name>` using the EXACT interface frame name. If a frame is at a part's own
local origin, its site pos is "0 0 0". The assembler reads these sites to weld this
subassembly to its neighbors, so every listed frame MUST have its site.

Output ONLY the PARTS JSON object, then a line with exactly `{MJCF_SENTINEL}`, then the
MJCF XML skeleton (with a <site> for every interface frame). No prose, no markdown
fences."""
    return f"""\
IMPORTANT — you are building ONE SUBASSEMBLY of a larger machine, not the whole
machine. Build ONLY the parts of this subassembly; do NOT add the neighboring
subassemblies. A separate assembler will join this subassembly to the others using
the interface frames below, so those frames are a CONTRACT you must honor exactly.

SUBASSEMBLY id: {getattr(fc, "sub_id", "?")}
GLOBAL ORIGIN: {origin}
{neighbors_txt}{appearance_txt}
INTERFACE FRAMES this subassembly must expose (positions are in GLOBAL machine
coordinates about that origin):
{frames_txt}

RULES
- Build this subassembly in ITS OWN local frame (mm geometry, each part's attach point
  at its local origin). You choose where this subassembly's own root/origin sits.
- Include EVERY real physical part of this subassembly — every gear, wheel, pinion,
  SHAFT, arbor, bearing/jewel, pin, screw, spring. Do NOT hide a shaft or bearing by
  folding it into a neighbor or dropping it; a rotating part is dof "spin" turning on
  a real shaft in a real bearing (dof "fixed"), and both are their own parts. This is
  only ONE subassembly, so completeness here is cheap.
- For EACH interface frame above, there must be a real PART positioned so that the
  frame lands at the given GLOBAL location when the machine is assembled. Typically
  the frame coincides with a specific part (a housing mounting face, a gear center,
  a shaft end).
- SEPARATE SEATS — YOU OWN THE BORE. If this sub is a base/plate/bracket/housing that exposes
  SEVERAL mount frames at DIFFERENT positions (several bearing bores, posts, seats), each frame
  is its OWN independent seat that YOU realize. For EACH such seat frame the boss gives (name +
  GLOBAL position + bore diameter), do this:
    1. On your structural BODY part (the plate/housing block), declare a positioned `bore` PORT
       at that seat's LOCAL position (the frame's global xyz_m minus this sub's origin), with
       `diameter_mm` = the frame's shaft diameter and the frame's axis. Give the port your own
       name (e.g. "bore_seat_input") — YOU own it; the boss does not name your parts.
    2. Realize THAT seat frame at the SAME local position on the body (frames_realized entry
       with the bore's local_xyz_m) — so the frame sits ON the bore you cut, not at the body
       origin. This is the ONLY correct realization; collapsing several seats onto the body
       origin makes the shafts that weld to them bury inside the body.
  Declare ONE bore port + ONE realization PER seat so the seats spread across the base. Do NOT
  connect the seated parts to each other (no shaft threading them). A bearing seat is a bore in
  your body at the frame's position — a feature of the body, not a separate free-standing part.
  You need only place each seat APPROXIMATELY along the right wall/axis: the assembler relocates
  every seat frame onto its SOLVED shaft before welding, so getting the exact xyz is not critical —
  what matters is that the seats stay SPREAD (never collapsed onto the body origin) and share the
  boss's through-shaft `axis`.
- The interface frames are HARD POINTS fixed by the boss — treat them as immovable.
  Where a frame gives a shaft/gear diameter, size YOUR mating shaft, bore, or gear to
  EXACTLY that diameter and put it on the frame's axis, so the part meets its
  neighbor across the seam. Do NOT invent a different position, axis, or diameter for
  an interface; only the boss changes a hard point.
{move_rule}

{frames_decl}"""




def build_manager_should_rebuild(prior_model_json: str, fault_reason: str,
                                 frame_contract=None) -> str:
    """Cheap keep-or-rebuild question: does THIS subassembly need to change to fix the
    fault, or is it unrelated? The manager answers with a single word."""
    sub_id = getattr(frame_contract, "sub_id", "this subassembly")
    return f"""\
You previously built subassembly '{sub_id}'. Here is its current kinematic model:

{prior_model_json}

The assembled machine failed for this reason:

{fault_reason}

Decide: does '{sub_id}' ITSELF need to change to fix this fault, or is the fault in a
DIFFERENT subassembly / the coupling between subs (so this one should be kept exactly
as-is and reused)?

Answer with ONE word only:
- "KEEP"    if this subassembly is fine and does not need changing.
- "REBUILD" if this subassembly must change to fix the fault.

Output only that one word."""


def build_manager_patch(prior_model_json: str, fault_reason: str,
                        frame_contract=None) -> str:
    """Ask for a MINIMAL structured PATCH to the prior subassembly model — change as few
    parts as possible (Claude-Code style), keep everything else untouched."""
    sub_id = getattr(frame_contract, "sub_id", "this subassembly")
    return f"""\
You previously built subassembly '{sub_id}'. Here is its current kinematic model:

{prior_model_json}

It must change to fix this fault:

{fault_reason}

Return a MINIMAL PATCH — change as FEW parts as possible, exactly like editing a file.
Do NOT restate the whole model. Output ONLY a single JSON object with these keys (any
may be empty/omitted):

{{
  "add_links":    [ <full LinkSpec objects for NEW parts, with dof> ],
  "modify_links": [ <full LinkSpec objects for parts whose geometry/size/dof changes;
                     use the SAME name as the existing part to replace it> ],
  "remove_links": [ "<name of a part to delete>" ],
  "add_poses":    [ <full PoseSpec objects for new placements> ],
  "modify_poses": [ <full PoseSpec objects, same name, to change a placement> ],
  "remove_poses": [ "<pose name to delete>" ]
}}

Only touch what the fault requires (e.g. resize ONE gear, add ONE missing shaft +
its pose, move ONE part into contact). Every other part is kept automatically. Use the
same names, units (mm sizes, meter pose offsets), and origin contract as the model
above (parts + poses + per-part dof; motion is by contact, no joints or motors).

Output ONLY the JSON patch object, no prose, no markdown fences."""
