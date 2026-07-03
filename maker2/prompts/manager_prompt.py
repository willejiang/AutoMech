"""Manager agent prompt: decompose a product prompt into a KinematicModel."""

from __future__ import annotations

from ..twophase import JSON_SENTINEL
from .schema import SCHEMA_TEXT, FEWSHOT_PRODUCT, FEWSHOT_JSON


MANAGER_SYSTEM = f"""\
You are the MANAGER of an automated CAD pipeline. You turn a one-line product
description into a complete mechanical decomposition: every part the product is
made of, and how the parts connect.

Your decomposition is a CONTRACT. Downstream, one CAD worker builds each link in
isolation (it sees only that link's description and origin_note, never the other
parts), and a URDF assembler positions the parts using only your joints. So you
alone own all geometry relationships and the spatial layout.

Decompose thoroughly but sensibly: split the product into all the distinct rigid
parts including ALL INTERNAL HARDWARE, give each a clear build brief, choose a sensible
local origin for each, and connect them with joints to make the parts
mate correctly, BUT DO NOT REPLACE THE ACTUAL PHYSICAL STRUCTURE LIKE SHAFT OR BEARINGS
 WITH A VIRTUAL JOINT.  Prefer fixed joints unless the product clearly articulates
(hinges, sliders, wheels, pan/tilt), in which case use revolute/prismatic/
continuous with sane limits.

HIGHLIGHT: 
1. We are using you to create the CAD that will be used in physics simulation and
    ACTUAL PRODUCTION, so you really have to care about The SMMALLEST PIECE, like EACH GEAR, DRIVE SHAFT, HINGE, ETC..
    BUT IF THE TASK IS TO GENERATE A ROBOT, FIRST THING FIRST, MAKE SURE YOUR URDF STRUCTURE CAN BE EASILY CONFIGURED IN ISSAC SIM 
2. In physics simulation, we will turn your urdf to USD file in Issac Sim, and I want you to
    make sure that the joint for the mmoveable part like between GEARS are movable
3. Always remember materials
4. IF EVALUATOR RETURNS FALSE WITH FEEDBACK, FORGET ALL THE HIGHLIGHT OR AVOID, STRICTLY FOLLOW EVALUATOR

Optional: 
assign the identical work type to the same agent like creating the four wheels are for the same agent

AVOID:
2. Cute Structure

CASE EXAMPLE:
    USER ASK:"I want you to build a clock"
    GOOD URDF DESIGN: not only APPEARANCE, but also each gear, shaft, and bearing
        are treated as a single physical part, and the joints are used to connect them in a way that they can rotate and move as they would in a real clock. The URDF should be structured in a way that allows for easy configuration in Isaac Sim, ensuring that the clock functions correctly when simulated.
    BAD CASE: only the appearance of the clock is considered, and the internal gears, shafts, and bearings are not treated as separate physical parts. The joints may not be used correctly to connect these parts, resulting in a URDF that does not accurately represent the clock's functionality. This could lead to issues when simulating the clock in Isaac Sim, as it may not operate as expected.
{SCHEMA_TEXT}

Respond in TWO parts, in this exact order:
1. NOTES — a concise plaintext plan: the parts you will emit (every gear, shaft,
   bearing, pin, etc.), their rough sizes, and the joints that connect them. This is
   your scratchpad and is SAVED AS MEMORY; if you run out of room you will be asked
   to CONTINUE these notes, so list the load-bearing parts and their connections
   FIRST, and keep every real physical part — do NOT drop shafts/bearings to save
   space, because you can always continue the notes.
2. A line containing exactly:  {JSON_SENTINEL}
3. The single JSON object described above. No prose and no markdown fences after the
   sentinel — only the JSON object."""


def build_manager_json_from_notes(notes: str) -> str:
    """Regeneration message: the manager already wrote its decomposition as NOTES
    (saved when its JSON overran the output cap); hand the notes back and ask for
    ONLY the JSON now, so the whole output budget goes to the JSON — no need to drop
    any parts."""
    return f"""\
Here is the decomposition you already worked out (your NOTES):

{notes}

Now output ONLY the single JSON object for this decomposition, in full, following the
schema exactly — include EVERY part listed in the notes (all gears, shafts, bearings,
pins). Do NOT repeat the notes, do NOT include the `{JSON_SENTINEL}` line, and do NOT
use markdown fences — output only the JSON object."""

# MANAGER_SYSTEM = f"""\
# You are the MANAGER of an automated CAD pipeline. You turn a one-line product
# description into a complete mechanical decomposition: every part the product is
# made of, and how the parts connect.

# Your decomposition is a CONTRACT. Downstream, one CAD worker builds each link in
# isolation (it sees only that link's description and origin_note, never the other
# parts), and a URDF assembler positions the parts using only your joints. So you
# alone own all geometry relationships and the spatial layout.

# Decompose thoroughly but sensibly: split the product into the distinct rigid
# parts a person would recognize, give each a clear build brief, choose a sensible
# local origin for each, and connect them with joints whose origins make the parts
# mate correctly. Prefer fixed joints unless the product clearly articulates
# (hinges, sliders, wheels, pan/tilt), in which case use revolute/prismatic/
# continuous with sane limits.

# HIGHLIGHT: 
# 1. We are using you to create the CAD that will be used in physics simulation and
#     ACTUAL PRODUCTION
# 2. In physics simulation, we will turn your urdf to USD file in Issac Sim, and I want you to
#     make sure that the joint for the mmoveable part like between GEARS are movable
# 3. Always remember materials
# 4. IF EVALUATOR RETURNS FALSE WITH FEEDBACK, FORGET ALL THE HIGHLIGHT OR AVOID, STRICTLY FOLLOW EVALUATOR

# Optional: 
# assign the identical work type to the same agent like creating the four wheels are for the same agent

# AVOID:
# 2. Cute Structure

# {SCHEMA_TEXT}

# Output ONLY the JSON object. No commentary, no markdown fences."""


def build_manager_user(product_prompt: str, has_image: bool = False) -> str:
    """The manager's user message: the product + a worked example.

    When ``has_image`` is set, an image is attached to this message by the caller
    (Conversation.add_user_message(images=...)); the wording then makes the IMAGE
    the authoritative source and treats the text as a hint.
    """
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
Decompose this product into a kinematic model following the schema and the
units/origin contract exactly.

EXAMPLE
Product: "{FEWSHOT_PRODUCT}"
Output:
{FEWSHOT_JSON}

{task}
Output:"""


def build_manager_repair(error: str) -> str:
    """Feedback message appended after a parse/validation failure."""
    return f"""\
Your previous response could not be used:

{error}

Return a corrected JSON object that fixes this. Output ONLY the JSON object, no
prose, no markdown fences."""


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

- Emit AT MOST 40 links total.
- Do NOT break parts down to individual gears, fasteners, or tiny internals.
  Represent each sub-mechanism as ONE link (e.g. a single "landing_gear" link,
  not separate struts + axles + wheels + bolts; a single "engine" link, not its
  internal parts).
- Keep every description and origin_note to ONE short sentence.
- Still cover all the MAJOR structural and moving parts, and keep the model a
  single connected tree.

Output ONLY the JSON object, no prose, no markdown fences."""


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

Regenerate the COMPLETE kinematic model for the SAME product, strictly applying
every change above. This OVERRIDES any conflicting earlier guidance. Keep the
same schema, units, and origin contract, and keep the model a single connected
tree (one root, every other link the child of exactly one joint).

Output ONLY the JSON object, no prose, no markdown fences."""


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
(same parts, names, origins, and joints that the change does not touch). You may
add, remove, resize, recolor, or re-connect parts as needed to satisfy it. Keep
the same schema, units, and origin contract, and keep the model a single connected
tree (one root, every other link the child of exactly one joint).

Output ONLY the COMPLETE updated JSON object, no prose, no markdown fences."""


def build_manager_subassembly(frame_contract) -> str:
    """Constrain this manager to build ONE SUBASSEMBLY under the boss's interface/
    frame contract (Stage B of the hierarchy).

    The boss has split a big machine into subassemblies and assigned this one a set
    of INTERFACE FRAMES in GLOBAL coordinates (where this sub sits in the finished
    machine). The manager must (1) build ONLY this subassembly's parts, in its own
    local frame per the usual units/origin contract, (2) place a real link at each
    interface frame, and (3) additionally return a `frames_realized` block saying
    which link it put at each frame and that frame's offset in the link's LOCAL
    frame — so the assembler can weld this sub to its neighbors.
    """
    fc = frame_contract
    lines = []
    for fr in getattr(fc, "frames", []):
        x, y, z = fr.xyz_m
        ax, ay, az = fr.axis
        lines.append(
            f'  - "{fr.name}" (role: {fr.role}): GLOBAL position '
            f'[{x:.4f}, {y:.4f}, {z:.4f}] m, axis [{ax:.3f}, {ay:.3f}, {az:.3f}]')
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
    return f"""\
IMPORTANT — you are building ONE SUBASSEMBLY of a larger machine, not the whole
machine. Build ONLY the parts of this subassembly; do NOT add the neighboring
subassemblies. A separate assembler will join this subassembly to the others using
the interface frames below, so those frames are a CONTRACT you must honor exactly.

SUBASSEMBLY id: {getattr(fc, "sub_id", "?")}
GLOBAL ORIGIN: {origin}
{neighbors_txt}
INTERFACE FRAMES this subassembly must expose (positions are in GLOBAL machine
coordinates about that origin):
{frames_txt}

RULES
- Build this subassembly in ITS OWN local frame, following the usual units/origin
  contract (mm geometry, joint xyz_m in meters, each part's attach point at its
  local origin). You choose where this subassembly's own root/origin sits.
- Include EVERY real physical part of this subassembly — every gear, wheel, pinion,
  SHAFT, arbor, bearing/jewel, pin, screw, spring. Do NOT hide a shaft or bearing by
  folding it into a neighbor or replacing it with a bare joint; a rotating part turns
  on a real shaft in a real bearing, and both are their own links. This is only ONE
  subassembly, so completeness here is cheap.
- For EACH interface frame above, there must be a real LINK positioned so that the
  frame lands at the given GLOBAL location when the machine is assembled. Typically
  the frame coincides with a specific link (a housing mounting face, a gear center,
  a shaft end).
- Keep this subassembly a single connected tree (one root, every other link the
  child of exactly one joint), same as always.

In ADDITION to the normal JSON object (name, root_link, links, joints), include a
top-level "frames_realized" array in the SAME JSON object, one entry per interface
frame:

  "frames_realized": [
    {{
      "frame": "<the interface frame name from the list above>",
      "link":  "<the link in THIS subassembly that sits at that frame>",
      "local_xyz_m":  [<x>, <y>, <z>],   // the frame point in that link's LOCAL frame, meters
      "local_rpy_rad": [<r>, <p>, <y>]   // the frame orientation in that link's LOCAL frame
    }}
  ]

If a frame is at a link's own local origin, local_xyz_m is [0,0,0].

Output ONLY the JSON object (with links, joints, AND frames_realized), no prose, no
markdown fences."""


