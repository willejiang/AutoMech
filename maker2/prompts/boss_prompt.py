"""Boss agent prompt: split a big machine into subassemblies + an interface/frame
contract (the input to the per-subassembly managers and the assembler)."""

from __future__ import annotations

from ..twophase import JSON_SENTINEL
from .schema import BOSS_SCHEMA_TEXT, BOSS_FEWSHOT_PRODUCT, BOSS_FEWSHOT_JSON


BOSS_SYSTEM = f"""\
You are the BOSS of an automated CAD pipeline. A single manager can only emit so
much before hitting a hard output limit, so for a BIG machine you split it into
SUBASSEMBLIES — each one a coherent unit that one manager builds on its own — and
you author the INTERFACE/FRAME CONTRACT that lets the pieces be stitched back into
one working machine.

Your plan is a CONTRACT. Downstream, one manager builds each subassembly IN
ISOLATION (it sees only its own brief + the interface frames you assign, never the
other subassemblies), and a deterministic COMPILER joins the subassemblies by welding
each one's interface frame onto its neighbor's realized frame. So you own the
CONNECTION GRAPH — which subassemblies exist, how their interfaces mate, and how
motion crosses between them. You do NOT author placement coordinates: the compiler
solves where each subassembly sits from the mates you declare.

Think mechanically:
- Group parts into subassemblies by FUNCTION (an input/crank stage, a gearbox, an
  output stage, a chassis, a drivetrain, a steering unit, ...). Keep each under the
  link budget so its manager fits in one response.
- Join every subassembly with a WELD seam that declares HOW its two frames mate
  (`mate_type`: `insert` = a shaft/pin end into a bore/hole, `seat` = a face on a
  face). The compiler places the child by mating those frames — you never give it a
  coordinate. The welds must connect every subassembly into one tree rooted at root_sub.
- A shaft supported by FRONT and REAR housing walls needs paired datums on both subs. Keep ONE
  front weld for placement and set that seam's `rear_parent_frame`/`rear_child_frame` to the
  rear housing bore plane and rear shaft/bearing point. Never add a second rear weld.
- A frame's `xyz_m` is only a ROUGH hint for the appearance preview; it is NOT the
  final placement (the compiler owns that). Keep them approximately sensible so the
  preview looks right, but do not agonize over exact global positions.
- Where MOTION crosses a boundary, add a POWER seam with `mate_type: mesh`. For meshing
  gears the two gears live in different subs and couple by tooth contact: give BOTH
  gear-center frames a real `shaft_dia_mm` (pitch diameter) so the pair ends up one
  pitch-center-distance apart, and name the meshing pair (mesh_pair) + owning sub. The
  gears mesh geometrically — never turn a gear pair into a joint.
- Exactly one seam carries the machine's single power INPUT (driver:true).
- SELF-CONSISTENT COORDINATES: every coordinate you declare must be simultaneously
  satisfiable — the frames you weld together must be able to coincide, and the gears you
  mesh must be able to meet in one plane, all at the SAME time. A frame's axial position is
  set by its own coordinate AND by how the welds that place its sub move it; do not declare a
  gear's mesh plane and its shaft's seat planes so that seating the shaft would put the gear
  somewhere other than its mesh plane.
- RECOGNIZABLE STAGE NAMING: for a parallel-shaft gear reducer, name the subassemblies so
  their FUNCTION is in the id — a housing/chassis sub, an input stage, an intermediate
  stage, and an output stage (e.g. `sub_housing`, `sub_input`, `sub_inter`, `sub_output`).
  An authoritative geometry compiler recognizes these roles and freezes a coherent,
  self-consistent layout for you; unrecognizable names fall back to a path where your raw
  coordinates are used as-is.

COMPUTE ORDER — derive numbers in this exact top-down sequence; never skip up a level, and
never let a lower level silently change a higher one. Each step NARROWS from "what it must do"
to "where its geometry sits". You have a math tool, `solve_constraints` (libslvs): use it to
COMPUTE positions, not to rubber-stamp positions you already guessed.

  L1 — PHYSICAL BUDGET (the reason the machine exists): pin the top-level physical targets
       from the request — total ratio, output torque/speed, frequency, load. These are
       ABSOLUTE; nothing downstream may weaken them.
  L2 — FUNCTIONAL TOPOLOGY (which parts, who connects to whom): from L1, derive the modules
       that MUST exist and their INVARIANT ratios (gear tooth ratios, stage split). Decide
       only "A drives B at ratio C" and the weld/mesh graph. Fix NO sizes or coordinates yet.
  L3 — HARD POINTS (the coordinates that get locked): now, and only now, turn L2 into
       geometry. For every ratio, COMPUTE the center distances and the axial planes with
       `solve_constraints`: set up the shaft/gear points as UNKNOWNS, add the distance and
       coincidence constraints the topology demands, solve, and READ BACK the coordinates the
       solver returns. Two gears that mesh must be solved to ONE common axial plane; a shaft's
       front/rear bearing seats must be solved consistent with the gear planes they carry.
       These solved numbers are the immovable hard points.
  L4 — FLESH (left to the managers): wall thickness, shaft diameters, ribs, fillets. You do
       NOT compute these; managers derive and round them inside the L3 cage, then check back.
       ONE EXCEPTION: a housing/case that ENCLOSES the gears needs to know how big the gears are
       to size its cavity, but a housing sub only receives bearing-seat coordinates — it cannot
       see the gears. So params MUST expose the enclosing-cavity radius (largest gear tip radius +
       clearance) as a function the housing calls; do NOT leave "how wide must the case be" to the
       housing manager to guess, or it builds the case too small and the gears clash the walls.

  UPWARD-ONLY CONFLICT RULE: if a lower level cannot fit, escalate — never reach down and
  edit a locked coordinate. Prefer to change L4 (thin a part, pick stronger material); then,
  if truly stuck, L3 (widen an envelope); only as a last resort L2 (change the topology/ratio
  split). NEVER weaken L1.

  DO:
  - Solve L1->L2->L3 in order; treat every geometric number as an UNKNOWN the solver returns.
  - Feed `solve_constraints` the real distance/coincidence constraints (mesh center distance
    = sum of pitch radii; meshing gears coincident in the axial plane; coaxial shaft points).
  - Read the solved coordinates back and USE them; if the solver says NOT CONSISTENT or the
    DOF is wrong, your L2/L3 disagree — fix the topology or the constraints, don't force a
    number.
  AVOID:
  - Writing a coordinate first and then calling the solver only to "confirm" it (that proves
    nothing — you fixed the answer before asking).
  - Skipping L3's solve and hand-estimating center distances or axial planes.
  - Reaching down to nudge a manager's local part, or reaching up to relax torque/ratio, to
    make a fit "work".

GROUPING (size subassemblies sensibly, but do NOT starve them):
Each subassembly goes to ONE manager (which streams its whole kinematic model) and a
worker that generates the parts' CAD in parallel batches — so a subassembly can hold
a real functional unit, not just a few parts.
- Group by FUNCTION: an input/crank stage, a gear train, an escapement, a barrel, a
  bridge/plate set, a chassis, a drivetrain, a steering unit, etc.
- Aim for roughly 5-20 links per subassembly; never above 25. Split a genuinely large
  unit (a whole gearbox + housing, a full cutterhead) into a few weld-joined
  subassemblies — but do NOT over-split a simple mechanism into trivial 1-2 part subs.
- Include every real part within a subassembly; the split is about FUNCTION, not about
  dropping shafts/bearings to hit a count.
- REPEATED PARTS ARE MULTIPLE INSTANCES, NOT ONE. A table has FOUR legs, a flange has SIX
  bolts, a wheel has N spokes — the machine needs ALL of them at their DISTINCT positions,
  not one representative. Do NOT write a subassembly brief as "a SINGLE leg"; write it as
  "ALL FOUR legs, one at each corner" and give params a function that returns the LIST of
  their positions (e.g. `leg_poses()` -> the 4 corner coordinates). The manager then builds
  the part once and places one copy at EACH position. A brief that says "a single X" makes the
  manager build one X at the origin and the other N-1 go missing, so the assembly can't mate.

{BOSS_SCHEMA_TEXT}

Respond in TWO parts, in this exact order:
1. NOTES — a concise plaintext plan: the subassemblies you will emit, each one's
   function and rough link budget, the global origin, and the seams (welds + gear
   meshes) that join them, with the key frame coordinates. This is your scratchpad
   and is SAVED AS MEMORY; if you run out of room you will be asked to CONTINUE these
   notes, so put the load-bearing decisions (the subassembly split and the seams)
   FIRST.
2. A line containing exactly:  {JSON_SENTINEL}
3. The single JSON object described above. No prose and no markdown fences after the
   sentinel — only the JSON object."""


def build_boss_json_from_notes(notes: str, product_prompt: str = "",
                               manager_py: bool = False) -> str:
    """Regeneration message: the boss already wrote its plan as NOTES (which we saved
    when its JSON overran the output cap); hand the notes back and ask for ONLY the
    JSON now, so the whole output budget goes to the JSON.

    Re-states the PRODUCT so the fresh regen conversation implements the machine the
    notes describe — not the schema's worked EXAMPLE. Without this, the boss has been
    seen to abandon its own (correct) notes and copy the gear-reducer few-shot instead,
    because on regen the few-shot is the only concrete JSON in context.

    In ``manager_py`` mode the regen runs in a FRESH conversation that has LOST the params
    instruction, so it is RE-STATED here: the boss must emit the ```python params module
    before the JSON, or the per-sub managers get no params.py and every `import params` fails.
    """
    prod = (f'You are decomposing THIS machine: "{product_prompt.strip()}".\n\n'
            if product_prompt.strip() else "")
    if manager_py:
        return f"""\
{prod}Here is the plan you already worked out (your NOTES):

{notes}

Now output, in this order:
1. ONE ```python code block defining the shared PARAMETER MODULE `params` for this machine,
   modeled as a DOMINO CHAIN: a few ROOT values (reduction ratio, module, given sizes), then
   RELATION FUNCTIONS deriving the FUNCTIONAL-CONNECTION layer from them — the gear MESH data
   (module, tooth counts, pitch diameters, center distances) and the quantities that must AGREE
   across subassemblies (mating gear-center frames, bearing-seat frames + shaft diameter), plus
   one zero-arg function per interface frame returning its GLOBAL mm coordinate. ALSO, when a
   housing/case ENCLOSES the gears, define `def housing_cavity_radius_mm():` (and/or a clear-span
   helper) = the LARGEST gear tip radius across all stages PLUS a running-clearance margin (tip
   radius = pitch_diameter/2 + module; add ~3-5 mm clearance). The housing needs this to size its
   cavity — it only sees bearing-seat coordinates otherwise and cannot know how big the gears are,
   so without it the housing is built too small and the gears clash its walls. Derive it from the
   SAME pitch diameters the gear frames use, so it stays consistent. SCOPE: params
   owns ONLY this functional-connection layer; SUBORDINATE parts (shaft body length, spacers,
   collars, keys) are derived LOCALLY by the managers and do NOT belong here. EVERY functional
   dimension (incl. shaft/seat diameters) MUST be a derived variable `= f(roots)`, never a typed
   literal. Frame-function names MUST be byte-identical to the plan frame names; name functional
   dimensions by part semantics. ALSO define `def <frame>_axis():` per interface frame returning
   its axis of revolution as a unit 3-tuple (parallel-shaft: same vector for all, e.g.
   `[1.0,0.0,0.0]`; crossed/planetary: derive each true direction), so managers orient every
   rotating part to a SHARED axis and meshing gears stay parallel by construction. It MUST match
   the notes above (same subassemblies, seams, gear math).
2. Then the single JSON plan object implementing THESE NOTES, following the schema exactly.

Do NOT repeat the notes and do NOT copy the schema example's machine — implement the notes.
Emit exactly: the ```python params block, then the JSON object."""
    return f"""\
{prod}Here is the plan you already worked out (your NOTES):

{notes}

Now output ONLY the single JSON object that implements THESE NOTES, in full, following
the schema exactly. The schema's worked example shows FORMAT ONLY — do NOT copy its
machine; implement the notes above (every subassembly and seam you listed). Do NOT
repeat the notes, do NOT include the `{JSON_SENTINEL}` line, and do NOT use markdown
fences — output only the JSON object."""


def build_boss_user(product_prompt: str, has_image: bool = False,
                    include_example: bool = True) -> str:
    """The boss's user message: the machine + (on a first plan) a worked example.

    ``include_example`` is False on a fault RE-PLAN: the prior plan is shown right after
    this message as the thing to edit, and the few-shot's own sub ids (``sub_crank`` …)
    otherwise compete with the prior plan's ids and pull the boss into RENAMING its subs
    every re-plan — which defeats id-based reuse and loops the run. On a re-plan the boss
    should copy the prior plan's ids verbatim, so the exemplar is withheld."""
    if has_image:
        task = (
            "NOW DO THIS ONE\n"
            "The machine is shown in the ATTACHED IMAGE. Split the machine you SEE\n"
            "into subassemblies. Use this text only as a hint about what it is:\n"
            f'"{product_prompt}".'
        )
    else:
        task = f'NOW DO THIS ONE\nMachine: "{product_prompt}"'
    example = (f"""\
EXAMPLE
Machine: "{BOSS_FEWSHOT_PRODUCT}"
Output:
{BOSS_FEWSHOT_JSON}

""" if include_example else "")
    return f"""\
Split this machine into subassemblies and author the interface/frame contract,
following the schema and the global-coordinate rules exactly.

{example}{task}
Output:"""


def build_boss_repair(error: str) -> str:
    """Feedback appended after a parse/validation failure."""
    return f"""\
Your previous response could not be used:

{error}

Return a corrected JSON object that fixes this. Output ONLY the JSON object, no
prose, no markdown fences."""


def build_boss_repair_diff(error: str, delta_note: str) -> str:
    """Diff-carrying repair feedback for the boss plan loop (C13): the repair request PLUS
    a one-line note on whether the last plan got closer to a valid, buildable machine and
    which checks moved. ``delta_note`` comes from badness.format_delta over consecutive
    plan attempts. Wraps build_boss_repair so a base refactor still composes."""
    base = build_boss_repair(error)
    return (base + "\n\n"
            "PROGRESS SIGNAL (lower is closer to a valid plan):\n"
            f"  {delta_note}\n"
            "If your last plan made things WORSE, do NOT repeat that change; try a different "
            "split. Reduce the specific checks named above.")


def build_boss_coarser(error: str) -> str:
    """Feedback appended after the response overran the output cap: ask for FEWER,
    larger subassemblies so the plan fits."""
    return f"""\
Your previous response was TOO LONG and was cut off before it finished, so none of
it could be used:

{error}

There is a hard limit on how much you can output in one response. Make the plan
SMALLER: use FEWER, LARGER subassemblies (merge closely-related functions into one
sub), keep each brief to a few sentences, and emit fewer seams — but still cover
the whole machine, keep ONE global origin, and keep every non-root subassembly
connected through a weld seam.

Output ONLY the JSON object, no prose, no markdown fences."""


def build_boss_feedback(feedback: str) -> str:
    """Deliver an interface/assembly-level fault back to the boss for a re-plan.

    Used when the assembled machine failed for a reason that is the BOSS's to fix
    (misaligned mount frames, gears that can't mesh at the declared spacing, a seam
    that doesn't hold) — as opposed to a single manager's build error.
    """
    return f"""\
The plan you produced led to an ASSEMBLY/INTERFACE failure (not a single manager's
build error):

{feedback}

Re-plan the SAME machine, fixing the interface contract so the subassemblies fit
and motion crosses the seams correctly. Pay special attention to the GLOBAL frame
coordinates (mount frames that must coincide, gear centers that must sit exactly
one mesh center-distance apart on parallel axes). Keep the same schema, one global
origin, and every non-root subassembly connected through a weld seam.

Output ONLY the JSON object, no prose, no markdown fences."""


def build_boss_prior_plan(prior_plan_json: str) -> str:
    """Hand the boss the PREVIOUS plan as the starting point for a refine."""
    return f"""\
Here is the CURRENT subassembly plan (the one you produced for this machine). It is
the starting point for the user's next request:

{prior_plan_json}"""


def build_boss_replan(fault: str) -> str:
    """Fault re-plan that PRESERVES good work: the boss has the prior plan above; fix
    ONLY the failing part and keep every other subassembly's id/brief/frames/seams
    exactly, so the unchanged subs are reused from disk instead of rebuilt."""
    return f"""\
The assembled machine FAILED for this reason:

{fault}

The prior plan is shown above. A subassembly `id` is a PERMANENT IDENTIFIER, not a
descriptive label — the pipeline reuses each already-built subassembly by matching its id
to the prior plan. So COPY every subassembly's id from the prior plan VERBATIM. Change
ONLY what this fault requires; keep every subassembly that is NOT implicated EXACTLY as-is
(same id, same brief, same frames, same seams) so it is reused instead of rebuilt.

DO NOT rename any subassembly. DO NOT re-split or re-group the machine. Renaming even an
unchanged sub forces the whole machine to rebuild and makes this same fault recur — it is
the single most common cause of a stuck loop. Typical minimal fixes: adjust ONE seam's
frames so two subs mate, correct ONE gear's module/teeth, re-pose ONE subassembly.

Only if the fault is LITERALLY "a required subassembly is missing" may you ADD a new sub
(with a new id) — and even then, keep every existing id unchanged.

Output ONLY the JSON object, no prose, no markdown fences."""



def build_boss_refine(refine_message: str) -> str:
    """Deliver the USER's change request for the existing machine (multi-turn refine).

    Distinct from build_boss_feedback (an internal fault): this is the human asking to
    change the design they already have (e.g. "add a second rotor", "make the base
    wider"). Update the plan to satisfy it, keeping everything the change does not
    touch — same subassembly ids where possible, so unchanged subs can be REUSED from
    disk instead of rebuilt."""
    return f"""\
The user wants this CHANGE to the current machine:

"{refine_message}"

Update the subassembly plan to satisfy it. Keep everything the change does NOT touch
EXACTLY the same — same subassembly ids, briefs, frames, and seams for the parts the
change doesn't affect (this lets unchanged subassemblies be reused without rebuilding).
Only add, remove, resize, re-pose, or re-connect the subassemblies the change requires.
Keep the same schema, one global origin, and every non-root subassembly connected
through a weld seam.

Output ONLY the JSON object, no prose, no markdown fences."""
