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
              shafts). The axle is implicit — you do NOT model a joint for it. Every round/
              axial part is built around LOCAL +Z, so its local `spin_axis` is always [0,0,1].
              Boss interface-frame axes are GLOBAL placement targets; never copy a global +X/+Y
              frame axis into a round part's local spin_axis.
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


# Distilled build123d MODELING METHOD (方案B py mode). This is not API syntax — it is the
# METHOD for constructing a non-trivial part correctly, distilled from the text-to-cad CAD skill's
# build123d-modeling + positioning references. It exists because a manager that only knows
# `make_gear` cannot express the shapes make_gear does not cover (an internal ring gear, a cam, a
# non-circular profile) — and those are exactly the parts that make a machine impressive. Teach the
# method, not one more helper.
_MODELING_METHOD = """\
HOW TO CONSTRUCT A PART (build123d modeling method — read before writing geometry):
- CHOOSE THE CONSTRUCTION FIRST so the spec's controlling dimensions become DIRECT named
  parameters, not derived magic numbers. Profile-driven shapes (gears, rings, plates, brackets):
  ONE closed BuildSketch profile + `extrude`/`revolve`. Block-and-feature parts (housings, bases):
  a base solid then SUBTRACT features. Decide which makes the key dimension a parameter.
- ORDER OPERATIONS so fragile steps come last and a failure localizes to ONE feature:
  base solid -> major additions -> subtractive features (bores, pockets) -> through-holes ->
  fillets/chamfers LAST. Make each feature a named step (own variable/helper) so one bad op points
  at one line. Every boolean invalidates prior face selectors, so postpone fillets.
- OVERSHOOT BOOLEAN TOOLS: extend a cutting tool ~1 mm past the faces it enters and exits;
  coincident/coplanar tool-and-target faces are a classic kernel failure. Cut a patterned set of
  features (a bolt circle, a row of pockets) in ONE combined operation.
- NON-TRIVIAL SHAPES YOU BUILD YOURSELF (make_gear only does EXTERNAL spur gears):
  * INTERNAL RING GEAR (annular internal gear — teeth point INWARD, the hard part of a planetary
    set): pitch_r = module*teeth/2; internal TIP radius is SMALLER than pitch, ROOT is LARGER:
    `tip_r = pitch_r - module`, `root_r = pitch_r + 1.25*module`, outer wall beyond root. Build:
    a ring disk (`Circle(outer_r)` SUBTRACT `Circle(root_r)` -> the smooth cavity is at root_r),
    then ADD N trapezoidal teeth pointing inward from root_r to tip_r (loop i in range(teeth),
    angle a=2*pi*i/teeth, a 6-point Polygon from (root_r, +half) through (tip_r, +/-tip_half) to
    (root_r, -half)). This is ~15 lines of the same primitives make_gear uses — write it, do not
    ask for a helper.
  * BORE / ANNULUS (any part a shaft passes through): SUBTRACT a `Circle(bore/2)` extrude so the
    shaft has real clearance through the solid — never leave a gear/spacer/collar bore-less.
  * HOLE PATTERN: `with Locations(*positions): Hole(radius=...)` cuts them all at once; put the
    positions in a named list, not inline constants.
- STABLE SELECTORS: pick faces/edges by axis, normal, or bounding position (top/bottom by Z),
  never by a fragile topology-order index — a selector index shifts after any boolean.
- SANITY-CHECK PROPORTIONS before returning: compare the part's expected bounding box to the real
  object and its wall thickness to overall size. Order-of-magnitude and collision errors pass a
  build but fail the assembly."""


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


def manager_system(manager_ir: bool = True, manager_py: bool = False) -> str:
    """The manager system prompt for the chosen authoring format."""
    if manager_py:
        return MANAGER_SYSTEM_PY
    return MANAGER_SYSTEM if manager_ir else MANAGER_SYSTEM_MJCF


MANAGER_SYSTEM_PY = """\
You are the MANAGER of ONE subassembly in an automated CAD pipeline, authoring it as
PARAMETRIC build123d PYTHON with a cadpy AssemblyHelper. Instead of describing parts in JSON,
you WRITE the code that BUILDS and MATES every part.

Emit EXACTLY ONE ```python code block defining a function `build_subassembly()` that returns a
`cadpy.assembly.AssemblyHelper` (or its built Compound). Rules:

- `import params` and `import math`. `AssemblyHelper`, `make_gear`, and the build123d names
  (`BuildPart`, `BuildSketch`, `Cylinder`, `Box`, `Circle`, `Polygon`, `extrude`, `Location`,
  `Plane`, `Align`, `Mode`) are ALREADY INJECTED — do NOT import build123d or cadquery. The
  `params` module is the machine's SINGLE SOURCE OF TRUTH (constants + relation functions + one
  zero-arg function per interface frame returning its GLOBAL mm coordinate, and `<frame>_axis()`).
  DERIVE every dimension AND every coordinate from it — never hard-code a number you could compute.
- BUILD each part as a build123d solid in millimeters, along LOCAL +Z with its ORIGIN AT THE -Z
  END FACE. Use `align=(Align.CENTER, Align.CENTER, Align.MIN)` on a Cylinder so it spans local
  z in [0, height]. Do NOT center it on the origin. Build at the origin — mating (below) places it.
- GEARS AND PINIONS MUST HAVE REAL TEETH — an EXTERNAL spur gear/pinion is built with the injected
  `make_gear(module, teeth, face_width, bore)`, NEVER a plain cylinder (a smooth disk cannot mesh).
  `module`/`teeth` come from params (same values that set pitch diameter and center distance);
  `face_width`/`bore` are your local choices (bore = the shaft diameter it rides on). make_gear
  returns a toothed spur gear along +Z, origin at the -Z face. make_gear ONLY does external spur
  gears — for an INTERNAL ring gear, a cam, or any non-circular profile, WRITE the build123d
  yourself following the MODELING METHOD below (it is the same primitives, ~15 lines). Do not
  substitute a smooth cylinder for a toothed part because a helper does not exist.
- ASSEMBLE with the cadpy AssemblyHelper — this is how you MATE parts WITHOUT hand-computing any
  coordinate. Build every part in a LOCAL frame (each shaft along +Z from z=0), connect them
  relative to each other, then ANCHOR THE WHOLE SUB to its global params location. Pattern:
    from build123d import Location
    a = AssemblyHelper("<sub_id>")
    shaft = a.add(<built part>, "input_shaft")      # add each part with a NAME (its link name)
    pinion = a.add(make_gear(params.<module_const>, params.<pinion_teeth>, fw, bore), "pinion")
    # ^ params.<module_const>/<pinion_teeth> are PLACEHOLDERS — the boss names them for THIS
    #   machine (it may call the module `M` or `MODULE_MM` or `m`; teeth `Z_PINION` or `z_pinion()`).
    #   NEVER copy a literal name from this example; READ the params.py shown to you and use the
    #   names it ACTUALLY defines. A guessed name (e.g. `params.MODULE_MM` when params defines `M`)
    #   raises AttributeError and wastes an attempt.
    # declare NAMED mating frames (part-local), then CONNECT them (all in the LOCAL frame):
    a.rigid_frame(shaft, "pinion_seat", Location((0, 0, <axial station on the shaft>)))
    a.rigid_frame(pinion, "base", Location((0, 0, 0)))           # gear -Z face
    a.connect((shaft, "pinion_seat"), (pinion, "base"))         # pinion base lands on the seat
    # ...add/connect the rest of the sub's parts locally...
    # FINALLY anchor the entire built sub to its GLOBAL params interface frame — BOTH its
    # position AND its axis direction. This is the single most important step; without it every
    # sub sits at the local origin pointing +Z, so the shafts collapse onto one line and point the
    # wrong way. Build a Plane at the params coordinate whose normal is the params axis, and move
    # the whole compound by that Plane's location (this rotates local +Z onto the params axis AND
    # translates — the exact analogue of the old place_axial(axis=, frame_xyz=)):
    from build123d import Plane
    anchor = Plane(origin=params.<this_sub's_frame>(), z_dir=params.<this_sub's_frame>_axis())
    return a.build().moved(anchor.location)     # orient + place the WHOLE sub in world coords
  Frame helpers: `a.rigid_frame(part, name, Location((x,y,z)))` declares a fixed part-local datum
  (USE THIS for all mating — position is enough; the shaft/gear spin is recorded separately as
  metadata, you do NOT need a revolute joint here).
  Relations: `a.connect(fixed, moving)` welds moving's frame onto fixed's; `a.coaxial(...)` /
  `a.face_to_face(..., offset=...)` for those semantics. You MATE parts by named frames (no basis
  math), and you PLACE + ORIENT the whole sub by moving the built compound to the params Plane.
- GLOBAL ANCHORING IS MANDATORY. The assembler concatenates every sub's parts VERBATIM in world
  coordinates with NO cross-sub solve — it trusts that you already oriented+moved this sub to its
  global params frame. If you skip the final `.moved(Plane(origin=params.<frame>(), z_dir=params.
  <frame>_axis()).location)`, your sub stays at the origin pointing +Z, overlaps the other subs,
  and the assembly is rejected. Build local along +Z, anchor global with the params Plane.
- REPEATED PARTS: build ALL instances, not one. If your brief says N identical parts (4 legs,
  6 bolts, N spokes) and params gives a list of positions (e.g. `params.leg_poses()` -> N coords),
  build the part ONCE then `a.add` a `.moved(Location(pos))` copy at EACH position with a UNIQUE
  name (`leg_0`, `leg_1`, ...). Building only one and leaving it at the origin means the other N-1
  are missing and their mates (tenon/mortise, bolt/hole) can't meet — the assembly is rejected.
- CROSS-SUBASSEMBLY interface frames: the boss hands you SEVERAL interface frames (e.g. a front
  bearing seat AND a rear bearing seat on the same shaft). You anchor the sub with ONE `.moved(
  Location(params.<front_frame>()))`, so the OTHER frames land correctly ONLY IF your LOCAL axial
  layout already matches the params spacing. Concretely: build the shaft with its FRONT feature at
  local z=0 and its REAR feature at local z = |params.<rear_frame>() - params.<front_frame>()| (the
  true front-to-rear distance from params). Then moving the sub to the front frame places the rear
  feature on the rear frame automatically. Meshing gears across subs are one center-distance apart
  because both managers read the SAME params center-distance and both anchor to their own frames.
- `dof`/`spin_axis`/`driver`/`mesh_id` — MOTION METADATA via the part's LABEL, in this EXACT
  pipe format: the FIRST segment is the CLEAN part name, then `|key=value` pairs. Example:
    pn = a.add(pinion, "pinion")
    pn.label = "pinion|dof=spin|spin_axis=z|mesh_id=stage1"      # driver adds |driver=True
  CRITICAL: the first `|`-segment MUST be the same clean name you used in `a.add(part, name)` —
  it becomes the part's link name AND its STL filename. Do NOT put the whole metadata string as the
  name and do NOT bake `dof=`/`mesh_id=` into `a.add`'s name argument: if the name is not clean, the
  link name and the exported STL name diverge and the part SILENTLY VANISHES from the assembly
  (this is exactly how gears disappeared — a gear labelled "pinion|dof=spin|..." got a long link
  name that no longer matched its STL). Keep `a.add(part, "pinion")` clean; put ALL motion metadata
  AFTER the first `|` in the label. Recognized keys: `dof` (spin|fixed|free), `spin_axis` (x|y|z),
  `driver` (True on the ONE input part the physics drives), `mesh_id` (SAME id on the two meshing
  gears so the pair is recovered). You do NOT need a revolute joint — this metadata is enough.
- DIVISION OF LABOR (this is the core of the method — get it right):
  * FUNCTIONAL-CONNECTION parts come FROM params: any part that realizes the machine's function
    (meshing gears) or must line up with a neighbor subassembly (the gear it meshes with, the
    bearing seat it mounts into). Take its `axis` from `params.<frame>_axis()`, its `xyz` from
    `params.<frame>()`, and its functional dimensions (gear pitch diameter, module, the shaft
    diameter that seats in a bearing) by CALLING the matching `params.<name>()` / `params.<CONST>`
    handed to you. Because you and every sibling manager read the SAME params, meshing gears land
    one center-distance apart, point the same way, and weld points coincide BY CONSTRUCTION —
    there is no separate solving step.
  * A FUNCTIONAL GEAR THAT RIDES ON A CARRIER STILL ANCHORS TO ITS OWN params MESH FRAME — this is
    the #1 planetary-gear mistake. A planet gear sits on a carrier pin, so it is TEMPTING to just
    stack it on the pin (pin base -> pin seat -> gear.base) and let the mate chain decide where it
    lands. THAT IS WRONG: a planet MESHES with the sun and the ring, so it is a FUNCTIONAL part and
    its center MUST sit at `params.planet_mesh_<i>()` (or the single `params.planet_mesh()` the boss
    gave), the SAME gear plane the sun and ring anchor to. The sun/ring anchor there; if the planet
    only stacks on the pin it lands at some pin-derived axial height (e.g. plate_thickness + fw/2)
    and misses the gear plane by that much — the three gears no longer share one mesh plane and do
    not mesh. RULE: for every part that carries a `mesh_id` OR has a matching `params.<name>()`
    frame, place its center at that params coordinate — even if it also rides on a pin/shaft/carrier.
    Use the pin only for the RADIAL position (which planet hole) and for spin; take the AXIAL station
    (the gear-plane coordinate) from params, not from the mate stack. Concretely: connect the
    planet's `base`/`center` frame so its center reaches `params.planet_mesh_<i>()`, or build the
    pin/gear so the gear's mid-plane coincides with the params gear plane — do not let plate
    thickness silently set the gear's axial height.
  * SUBORDINATE dimensions you DERIVE LOCALLY: the shaft BODY length and diameter, fillets, the
    axial station of each part. params does NOT define these and is not supposed to. Compute them
    yourself in THIS module from the functional anchors params gave you — e.g. `shaft_len =
    (rear_bearing_station - front_bearing_station) + margin`. (Do NOT derive a spacer/collar/key
    width — you are not building those parts; see KEEP IT SIMPLE. Space the real parts with EMPTY
    gaps instead.) Mate the parts with the AssemblyHelper — `a.add(part, name)` then a `rigid_frame`
    + `connect` onto a nearby part's frame — never by typing a world coordinate.
  * INSERT-FIT DIAMETERS ARE NOT LOCAL — take them from params. A bearing OD, its seat bore, and
    its inner bore are a mating triplet that MUST agree across subassemblies, so params owns them.
    Size a bearing's OUTER diameter to `params.<role>_bearing_od_mm()`, its bore to
    `params.<role>_bearing_bore_mm()`; bore a housing seat to `params.<role>_seat_bore_mm()` (which
    equals the bearing OD). NEVER write `bearing_od = shaft_dia + 12` locally — if the housing and
    the shaft each invent their own bearing/seat diameter they interpenetrate and the assembly is
    rejected. If params is missing a `<role>_bearing_od_mm` / `<role>_seat_bore_mm` you need, that
    is a params gap to surface (call the params name and let the AttributeError name it), NOT a
    number to invent locally.
- Every FRAME location you pass to `rigid_frame`/`connect` must be a part-LOCAL point (e.g.
  `Location((0,0,axial_station))`), and every DIMENSION a `params.<name>()` call or an expression
  over params names you can SEE — never a bare functional number you typed. Calling a params name
  the module does not define raises AttributeError listing the names params DOES define.
- Do NOT write a defensive wrapper like
  `def _frame(name, default): return getattr(params, name)() if hasattr(params, name) else default`
  or `_p(name, default)`. Such a wrapper HIDES a wrong functional-connection name behind a
  hard-coded default — it was the #1 cause of a broken subassembly. Reference each params function
  by its literal name. No `hasattr`/`getattr`/try-except around params.
- The connection/topology is expressed BY the cadpy frames + connect relations + mesh_id tags you
  write — there is no separate mate list.
- No file I/O, no network; build123d + cadpy + math + the params module only (all injected). Every
  part must be a real non-empty solid.
- PYTHON, not JSON: use `True`/`False`/`None` (capitalized) — NOT `true`/`false`/`null`.
- NO INTERPENETRATION: parts must not overlap in solid. Give each part its own volume:
  * A gear/collar/spacer on a shaft is an ANNULUS — make_gear already cuts the bore; for a plain
    ring cut a bore in build123d (a SUBTRACT extrude of a Circle) so the shaft passes THROUGH it,
  * A KEY sits in a keyway and must be SMALL — its cross-section fits INSIDE the shaft radius
    (e.g. a 2x2 mm key on a 6 mm-radius shaft), not a block straddling the gear. Do not let a
    key overlap the gear body; place it in the shaft's keyseat under the gear.
  * Space coaxial parts along the axis with real gaps: front bearing | gear | pinion | rear
    bearing, each at a DISTINCT axial station, none sharing the same z-run. Leave gaps EMPTY —
    do not fill them with a spacer (see KEEP IT SIMPLE below).
  Aim for zero interpenetration — the pipeline rejects a subassembly whose parts overlap.
- KEEP IT SIMPLE — BUILD ONLY THE MINIMAL FUNCTIONAL PART SET: shaft + gears + bearings. Do NOT
  add non-functional filler: NO spacer, NO collar, NO washer, NO key/keyway, NO set-screw, NO
  dowel, NO snap-ring/retaining-ring, NO shim. These parts do NOTHING the physics uses (motion is
  by TOOTH CONTACT, not through a key or a spacer) and the pipeline never checks for them — but each
  one is one MORE chance to interpenetrate a neighbor and get the whole subassembly rejected. A
  spacer that fills a gap and a collar that retains a bearing are exactly the low-value parts that
  most often collide the housing or the gear. Leave the gap empty; let the bearing sit at its seat.
  Model such a part ONLY if it is the literal functional point of the machine (rare). Fewer parts =
  fewer overlaps = a subassembly that passes. When in doubt, DROP the part.

Respond in TWO parts: first NOTES (a short plaintext plan of the parts + their placements),
then the single ```python block. If asked to CONTINUE, output only the ```python block.

""" + _MODELING_METHOD


def build_manager_json_from_notes(notes: str, manager_ir: bool = True,
                                  manager_py: bool = False) -> str:
    """Regeneration message: the manager already wrote its decomposition as NOTES
    (saved when its payload overran the output cap); hand the notes back and ask for
    ONLY the payload now, so the whole output budget goes to it — no dropped parts."""
    if manager_py:
        return f"""\
Here is the subassembly plan you already worked out (your NOTES):

{notes}

Now output ONLY the single ```python code block for this subassembly, in full: `import params`,
per-part build123d builder helpers, and `build_subassembly()` returning a cadpy AssemblyHelper
that adds EVERY part from the notes with its name and cadpy rigid_frame mating datums (mesh_id on meshing gears, driver on the input). Do NOT repeat the notes — output
only the ```python block."""
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
                       manager_ir: bool = True, manager_py: bool = False) -> str:
    """The manager's user message: the product + (except in 方案B py mode) a worked example.

    When ``has_image`` is set, an image is attached to this message by the caller
    (Conversation.add_user_message(images=...)); the wording then makes the IMAGE
    the authoritative source and treats the text as a hint.

    In ``manager_py`` mode NO JSON few-shot is shown: MANAGER_SYSTEM_PY already carries the
    parametric-CadQuery example, and injecting IR_FEWSHOT_JSON here would prime the manager to
    emit connection-graph JSON (a `true`/`false` NameError when exec'd as Python).
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
    if manager_py:
        return f"""\
Author this subassembly as a parametric CadQuery ```python module, following the system
rules exactly (import params; derive every coordinate from params; output ONE ```python
block defining build_subassembly()).

{task}
Output:"""
    example = IR_FEWSHOT_JSON if manager_ir else FEWSHOT_JSON
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


def build_manager_critique(root_cause: str, fix_instruction: str, *,
                           prior_source: str = "", sibling_context: str = "",
                           machine_context: str = "") -> str:
    """方案B: deliver a DIAGNOSTICIAN's critique to a manager that is re-running to fix ONE
    localized fault. Unlike a blind re-decompose, this shows the manager (1) exactly what the
    diagnostician judged wrong, (2) its OWN previous code so it can see what to change, and
    (3) the neighbouring subassemblies' realized geometry (so a cross-sub clash — a gear that
    hit the housing — is understandable: the manager can SEE the other part's size/position).
    The manager must first RESTATE its understanding of the fault (the learning signal), then
    emit the corrected build_subassembly()."""
    parts = [
        "A DIAGNOSTICIAN inspected the assembled machine after a geometry pre-check FAILED and "
        "attributed ONE fault to THIS subassembly. This is not a full redesign — change the "
        "minimum needed to clear this specific fault, keep everything else.",
        f"\nROOT CAUSE (what your previous version did wrong):\n{root_cause}",
        f"\nREQUIRED FIX (do exactly this):\n{fix_instruction}",
    ]
    if prior_source:
        parts.append("\nYOUR PREVIOUS build_subassembly() (this is the code that produced the "
                     "fault — find the line the fix refers to and change it):\n```python\n"
                     + prior_source + "\n```")
    if sibling_context:
        parts.append("\nNEIGHBOURING SUBASSEMBLIES' realized geometry (so you can SEE what your "
                     "part collides with / must fit — e.g. the housing cavity your gear must clear, "
                     "the mating part your tenon must reach). Positions are GLOBAL mm:\n"
                     + sibling_context)
    if machine_context:
        parts.append("\nWHOLE-MACHINE geometry self-check (the numeric fault report):\n"
                     + machine_context)
    parts.append(
        "\nRespond in TWO parts:\n"
        "1. UNDERSTANDING — one or two sentences restating, in your own words, exactly what was "
        "wrong with your previous version and what you will change (this proves you understood the "
        "critique, not just regenerated).\n"
        "2. The corrected ```python build_subassembly()``` block, applying the fix and keeping the "
        "params-anchored global placement contract.")
    return "\n".join(parts)




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


def build_manager_subassembly(frame_contract, manager_ir: bool = True,
                              manager_py: bool = False) -> str:
    """Constrain this manager to build ONE SUBASSEMBLY under the boss's interface/
    frame contract (Stage B of the hierarchy).

    The boss has split a big machine into subassemblies and assigned this one a set
    of INTERFACE FRAMES in GLOBAL coordinates (where this sub sits in the finished
    machine). The manager must (1) build ONLY this subassembly's parts, in its own
    local frame, (2) place a real part at each interface frame, and (3) report which
    part realizes each frame — so the assembler can weld this sub to its neighbors.
    """
    if manager_py:
        return _build_manager_subassembly_py(frame_contract)
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
    through=getattr(fc,"through_mounts",[]) or []
    through_txt=("\nTHROUGH-SHAFT DATUM PAIRS (realize both on physical front/rear bores or bearings; "
                 "the rear point is validation-only, not a second weld):\n"+
                 "\n".join(f'  - seam {x["seam_id"]}: front "{x["front_frame"]}" -> rear "{x["rear_frame"]}"'
                            for x in through)+"\n") if through else ""
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
{through_txt}
RULES
- Build this subassembly in ITS OWN local frame (mm geometry, each part's attach point
  at its local origin). You choose where this subassembly's own root/origin sits. Round/axial
  geometry (shafts, bearings, gears, pinions, cylinders) uses LOCAL +Z as its physical axis;
  its inferred bore/teeth ports and local `spin_axis` are therefore [0,0,1]. The GLOBAL axis
  shown on an interface frame is for the assembler to orient the whole subassembly, not a
  value to paste into a part's local spin_axis.
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
- For every THROUGH-SHAFT pair above, realize the front and rear frames at distinct physical
  datums on the SAME shaft line: housing frames on the front/rear bore planes, stage frames on
  front/rear bearing or shaft points. Round parts remain LOCAL +Z; therefore make the realized
  front->rear vector parallel to the realizing front frame's LOCAL +Z axis. The assembler later
  rotates that local axis onto the boss's GLOBAL contract axis.
- The interface frames are HARD POINTS fixed by the boss — treat them as immovable.
  Where a frame gives a shaft/gear diameter, size YOUR mating shaft, bore, or gear to
  EXACTLY that diameter and put it on the frame's axis, so the part meets its
  neighbor across the seam. Do NOT invent a different position, axis, or diameter for
  an interface; only the boss changes a hard point.
- COAXIAL STATIONS ON ONE SHAFT: when several interface frames share the same axis and
  differ only along it (a front bearing, one or more gear/mesh centers, a rear bearing all
  on ONE shaft), they are ORDERED STATIONS at DISTINCT axial positions — read the differing
  coordinate along the axis and place EACH part (bearing, gear, pinion, spacer) at ITS
  station's axial offset on the shaft. Do NOT collapse them to a common origin: parts stacked
  at the same axial point grossly overlap. Space the shaft's parts so each sits at its
  frame's axial station and fills the gaps between stations (a spacer occupies the run
  between two gears; bearings sit at the ends).
- EXACT FRAME NAMES — realize EVERY interface frame under the EXACT name it is given above,
  character for character. The assembler and the frame gate match your realized frames to the
  contract BY NAME; a frame you realize under a different name (e.g. contract asks for
  'intermediate_stage_stage_1_mesh' and you report 'inter_pinion_mesh') counts as the
  contract frame NOT realized — your subassembly is rejected even though you built the part.
  You may name your PARTS/ports whatever you like, but each `frames_realized` entry MUST use
  the contract's frame name verbatim. Realize every listed frame; do not add, rename, drop,
  or merge them.
- A `mesh` frame MUST be realized ON A GEAR part — a part with real teeth (module + teeth,
  or a pitch diameter), whose center sits at the mesh frame. Do NOT put a mesh frame on the
  shaft, a bearing, or a plain cylinder: the assembler resolves the gear pair by finding a
  gear link at each mesh frame, and a mesh frame on a non-gear part makes the mesh unbuildable.
  Realize the mesh frame under its EXACT contract name (see above) on that gear link.
  CONCRETELY: for EVERY `mesh`-role interface frame in the list, add an entry to your
  connection graph's `frames` section: {{"frame": "<exact contract mesh-frame name>",
  "part": "<your gear part>", "port": "center"}}. If a shaft carries TWO gears (e.g. an
  intermediate stage with a stage-1 gear AND a stage-2 pinion), each of the two mesh frames
  binds to a DIFFERENT gear — never point both at the same part. A gear sub whose mesh frames
  are missing or bound to a non-gear part is REJECTED (ERR_MESH_FRAME_NOT_ON_GEAR), so do this
  every time.
{move_rule}

{frames_decl}"""


def _build_manager_subassembly_py(frame_contract) -> str:
    """方案B-v3: the subassembly instruction for the PARAMETRIC CADQUERY manager, authored as
    a TRADITIONAL PROGRAM. The boss wrote a shared `params` module (constants + relation
    functions + one zero-arg function per interface frame returning its GLOBAL coordinate).
    This manager `import params` and places every interface part by CALLING those functions —
    it never re-types a coordinate. Coordinates are therefore derived, not copied, and every
    subassembly that shares `params` agrees by construction."""
    fc = frame_contract
    lines = []
    for fr in getattr(fc, "frames", []):
        x, y, z = fr.xyz_m
        ax, ay, az = fr.axis
        dia = getattr(fr, "shaft_dia_mm", 0.0) or 0.0
        dia_txt = (f", diameter {dia:.2f} mm (build the mating shaft/bore/gear to EXACTLY "
                   f"this diameter)") if dia > 0 else ""
        role = getattr(fr, "role", "mount")
        # v3: name the params FUNCTION the manager must call for this frame's coordinate.
        lines.append(
            f'  - "{fr.name}" (role: {role}): call `params.{fr.name}()` for its GLOBAL mm '
            f'coordinate [~{x*1000:.1f}, {y*1000:.1f}, {z*1000:.1f}] and '
            f'`params.{fr.name}_axis()` for its spin-axis unit vector '
            f'[{ax:.2f}, {ay:.2f}, {az:.2f}]; realize a real part here and expose a cadpy '
            f'`rigid_frame` named "{fr.name}" so the assembler can mate onto '
            f'it{dia_txt}')
    frames_txt = "\n".join(lines) if lines else "  (none)"
    sub_id = getattr(fc, "sub_id", "?")
    origin = getattr(fc, "global_origin_note", "") or "(the machine's shared origin)"
    params = getattr(fc, "params_text", "") or ""
    params_block = (f"\nSHARED PARAMETER MODULE (already saved next to your code as `params.py`; "
                    f"`import params` and CALL its functions — this is the machine's single "
                    f"source of truth, identical for every subassembly):\n"
                    f"```python\n{params}\n```\n"
                    if params.strip() else "")
    return f"""\
BUILD THIS SUBASSEMBLY: {sub_id}
GLOBAL ORIGIN: {origin}
{params_block}
INTERFACE FRAMES this subassembly must expose. Each has a same-named function in `params` that
RETURNS its GLOBAL coordinate — place a real part so its relevant feature sits at that returned
point:
{frames_txt}

THINK OF THIS AS A NORMAL PROGRAM, NOT A DRAWING. The boss already wrote `params.py` (shown
above) deriving the FUNCTIONAL-CONNECTION layer from the hard inputs with `y = f(x)`. Your job:
CALL those functions for the functional/interface parts, and DERIVE the subordinate parts
locally yourself.

HARD RULES ON PLACEMENT (these make every subassembly line up automatically):
- `import params` at the top. READ the `params.py` shown above. For FUNCTIONAL-CONNECTION parts
  use ONLY the names it ACTUALLY defines — the boss chose those names for THIS machine; a guessed
  functional name will not exist (AttributeError lists what params does define). This applies to
  EVERY name including the "obvious" constants: the module may be `M` not `MODULE_MM`, the tooth
  count `Z_PINION` not `z_pinion()` — scan the params.py above for the real name every time; do NOT
  assume a conventional name. params covers only the functional-connection layer, so for subordinate
  parts you will NOT find a params name — that is expected, derive them locally.
- BUILD every part at the ORIGIN with its axis of revolution along LOCAL +Z, origin at the -Z end
  face (`align=(Align.CENTER, Align.CENTER, Align.MIN)`). Do NOT bake world position into the part.
- ASSEMBLE with the injected `AssemblyHelper` (already in scope — do not import it). `a.add(part,
  name)` registers each part under its link name; declare part-local mating datums with
  `a.rigid_frame(part, "<name>", Location((x,y,z)))`; then `a.connect((fixedpart,"frameA"), (movingpart,"frameB"))` welds B onto
  A. You NEVER type a world coordinate or a rotation — mating by named frames makes coaxial parts
  line up by construction. Return the AssemblyHelper.
- For each INTERFACE frame above, expose a cadpy frame with that EXACT name on the part that
  realizes it, and place the part so that frame coincides with `params.<frame>()` (mate the sub's
  root onto an anchor at the params coordinate). Meshing gears across subs land one center-distance
  apart because both managers read the SAME params center-distance.
- SUBORDINATE parts (spacer, shim, collar, shaft body): `a.add` them and `connect` onto a nearby
  part's frame at a station YOU compute from params anchors. Keep coaxial parts at DISTINCT axial
  stations so they never overlap.
- A GEAR RIDING ON A CARRIER/PIN STILL ANCHORS ITS CENTER TO ITS OWN params MESH FRAME (the #1
  planetary mistake). A planet gear rides on a carrier pin, but it MESHES with the sun and ring, so
  it is FUNCTIONAL: its center MUST sit at `params.planet_mesh_<i>()` (the gear plane the sun and
  ring anchor to), NOT at whatever axial height the pin/plate mate stack happens to produce. If you
  only stack the planet on the pin (pin base -> seat -> gear.base), it lands at plate_thickness+fw/2
  and misses the shared gear plane — the three gears no longer mesh. Use the pin for the RADIAL slot
  and spin only; take the AXIAL station from params. Connect the planet so its center reaches
  `params.planet_mesh_<i>()`, or size the pin so the gear mid-plane coincides with the params gear
  plane. Same for any gear that rides on a moving carrier/arm yet must mesh across the machine.
- A `mesh`-role frame MUST be realized on a real toothed `make_gear(...)` gear, tagged so it pairs
  with the meshing gear the neighbor subassembly builds at the SAME params point.
- Rotating parts are marked via the LABEL, in the EXACT pipe format `name|key=value|...` where the
  FIRST segment is the CLEAN part name (identical to your `a.add(part, name)` name) and the rest are
  metadata: e.g. `gear.label = "output_gear|dof=spin|spin_axis=z|mesh_id=stage2"`. The first segment
  becomes the link name AND the STL filename — if it is not the clean add-name, the part silently
  vanishes from the assembly. Keys: `dof` (spin|fixed|free), `spin_axis` (x|y|z), `driver` (True on
  the ONE input part), `mesh_id` (SAME id on the two meshing gears). No revolute joint needed.

Because you and every other manager take coordinates and dimensions from the SAME `params`
functions, your mating frames coincide by construction — there is no separate solving step. Do NOT
write a `_frame(name, default)` / `_p(name, default)` wrapper or any hasattr/getattr/default
fallback (it hides a wrong name): call params directly for functional parts, plain local variables
for subordinate ones.

Write `build_subassembly()` returning a cadpy AssemblyHelper. Output NOTES then ONE ```python block.

{_MODELING_METHOD}"""


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
