# PhysCAD Comfort v1 — External Harness Autonomous Run Protocol

This file is the complete run protocol for evaluating a coding harness with an installed
text-to-CAD skill. It is intended to be given unchanged to Claude Code and Codex.

The harness must run all ten frozen tasks autonomously, perform its own generation,
mechanical verification, physics evaluation, and refinement loop, then export final evidence
for an independent scorer. The harness must **not** calculate or claim the final benchmark
score itself.

---

## 1. Required launch parameters

Before starting, set these values explicitly:

```text
METHOD_NAME=<claude-code or codex>
MODEL_ID=<exact model identifier>
CLI_VERSION=<exact CLI/harness version>
TEXT_TO_CAD_SKILL_VERSION=<version, commit, or SHA-256>
OUTPUT_ROOT=<new absolute output directory>
MAX_ITERS=3
TASK_TIMEOUT_SECONDS=1800
```

Example output roots:

```text
C:/benchmark-runs/claude-code
C:/benchmark-runs/codex
```

`OUTPUT_ROOT` must be empty or nonexistent at suite start. Never write one method's outputs
inside another method's root.

---

## 2. Non-negotiable isolation rules

For every task:

1. Start a fresh CLI process/session and fresh task working directory.
2. Use the same `MODEL_ID`, thinking/reasoning configuration, permissions, tool access,
   `MAX_ITERS`, and timeout for all ten tasks.
3. Use only this file, the installed text-to-CAD skill, standard programming/CAD tools,
   documentation, and the task's own newly created files.
4. Do **not** search for, inspect, copy, or reference:
   - AutoMech/PhysCAD benchmark outputs or scores;
   - another Claude Code or Codex run;
   - any prior solution to these prompts;
   - accepted MJCF compilers, trajectories, meshes, videos, or evaluator results from another run.
5. Do not manually modify an artifact after the autonomous task process ends.
6. Do not rerun only a failed evaluator stage and present it as a fresh end-to-end result.
7. A task failure must not terminate the suite. Preserve its best/final artifacts and continue.
8. The score-bearing final attempt is the state selected by the harness at task termination.
   Do not replace it with a more favorable earlier, corrected, manual, or nonstandard attempt.
9. Record provider cache use honestly. Project task artifacts and accepted compilers must not be
   reused. Provider prompt-cache reads may be disclosed rather than artificially inferred away.
10. Do not use collision exclusions, equality constraints, hidden welds, direct output actuation,
    or support patches to conceal invalid geometry or force the requested answer.

---

## 3. Autonomous task loop

For each task, execute this loop with at most three CAD candidates/iterations:

1. Invoke the installed text-to-CAD skill using the exact frozen prompt below.
2. Generate an executable source program defining `MECHANISM` and `build_machine()`.
3. Execute the source in a clean process and export named parts and meshes.
4. Build explicit assembly/kinematic semantics.
5. Generate an MJCF model from those semantics.
6. Check source execution, part inventory, references, relation realization, non-exempt solid
   intersections, MJCF load/initialization, finite state, motion propagation, output, and task
   invariants.
7. If the candidate fails and budget remains, provide the measured failure evidence to the same
   harness and produce one revised candidate.
8. Retain every iteration under `raw/iterations/<index>/`.
9. At termination, export the selected final state using Section 4.

An iteration includes all CAD revision, mechanism-semantic revision, MJCF/compiler revision,
scenario/evaluator repair, and rollback work caused by that candidate. Internal tool calls are
not free and must be recorded where the harness exposes them.

Operational Pass@1 is true only when the first complete CAD candidate and its first complete
simulation lowering pass without replacement or repair. If this event is not explicitly
recorded, write `null`, not a guessed value.

---

## 4. Required output tree

Create exactly one directory per frozen task:

```text
OUTPUT_ROOT/
├── suite_config.json
├── 01_single_stage_4to1/
│   ├── source/machine.py
│   ├── assembly.json
│   ├── task_bindings.json
│   ├── meshes/*.stl
│   ├── models/model.mjcf
│   ├── evidence/trajectory.json
│   ├── evidence/contacts.json
│   ├── media/model.mp4                 # optional if video export is unavailable
│   └── raw/
│       ├── run.json
│       ├── evaluator_result.json
│       ├── stdout.log
│       ├── stderr.log
│       └── iterations/0/, 1/, 2/
├── 02_two_stage_9to1/
└── ... through 10_wind_rotor_pump/
```

Do not omit a task directory when it fails. Write the failure and all available evidence.

### `suite_config.json`

```json
{
  "suite_id": "physcad-comfort-v1",
  "method": "claude-code-or-codex",
  "model": "exact-model-id",
  "cli_version": "exact-version",
  "text_to_cad_skill_version": "version-or-hash",
  "max_iters": 3,
  "task_timeout_seconds": 1800,
  "task_count": 10,
  "cache_policy": "fresh project task directories; provider cache disclosed",
  "started_at": "ISO-8601",
  "finished_at": "ISO-8601"
}
```

### `raw/run.json`

```json
{
  "task_id": "01_single_stage_4to1",
  "prompt_sha256": "registered hash below",
  "method": "claude-code-or-codex",
  "model": "exact-model-id",
  "started_at": "ISO-8601",
  "finished_at": "ISO-8601",
  "wall_clock_seconds": 0.0,
  "iterations_used": 1,
  "operational_pass_at_1": true,
  "selected_iteration": 0,
  "selected_physics_attempt": "final",
  "llm_usage": {
    "requests": null,
    "input_tokens": null,
    "output_tokens": null,
    "cache_read_tokens": null,
    "reasoning_tokens": null
  },
  "tool_calls": null,
  "cache": {
    "project_artifact_hits": 0,
    "provider_cache_disclosed": true
  },
  "final_status_claimed_by_harness": "PASS-or-FAIL",
  "failure_summary": null
}
```

Use `null` for unavailable usage data; never estimate tokens, request counts, cost, or Pass@1.
The independent scorer ignores `final_status_claimed_by_harness` as a verdict.

---

## 5. Source and assembly contracts

### `source/machine.py`

It must be self-contained for the installed text-to-CAD runtime and define:

```python
MECHANISM = {
    # Complete links/ports/relations/motion/transmission/driver/output/watch semantics.
}

def build_machine():
    # Return a cadpy AssemblyHelper or build123d Compound with stable named parts.
    ...
```

Requirements:

- CAD length unit: millimetres.
- Every part has a stable, unique semantic name.
- Parts that must move independently remain separate solids.
- `build_machine()` is deterministic and produces a nonempty part set in a clean process.
- Do not read generated meshes or source from another task/method.
- All numerical gear ratios are explicit, finite, nonzero, and use the documented
  driven/driving convention.
- Press fits, running bearings, pins, revolutes, slides, closures, gear meshes, planetary
  stages, driver, output, and watched links are explicit in `MECHANISM`.

### `assembly.json`

Use this top-level schema:

```json
{
  "name": "mechanism-name",
  "root_link": "base",
  "links": [],
  "poses": [],
  "ports_by_link": {},
  "relations": [],
  "motion_joints": [],
  "transmissions": [],
  "planetary_stages": [],
  "mesh_pairs": [],
  "output_link": "output-link-name",
  "watch_links": []
}
```

Each link must identify its name, mesh file, degree of freedom, axes, driver status, and
rigid mount where applicable. Poses must use metres and radians. Mesh geometry remains in
millimetres. Every reference must resolve to an existing named entity.

A simulation collision exclusion is not a geometric exemption. A geometry overlap is exempt
only for its exact pair when an explicit authored physical relation supports that overlap,
such as a press fit, running bearing, dedicated pin fit, or declared ideal gear mesh.

---

## 6. `task_bindings.json`

Bind the exact required role names for the current task to final joint/body names. Values are
arrays because some roles have cardinality greater than one.

```json
{
  "roles": {
    "input_shaft": ["input_shaft_hinge"],
    "output_shaft": ["output_shaft_hinge"],
    "gear": ["input_pinion", "output_gear"],
    "hand_crank": ["hand_crank"]
  }
}
```

Do not bind a role to a nonexistent entity, duplicate one entity to satisfy a required count,
or alias input and output. The exact required roles appear with each task in Section 10.

---

## 7. MJCF and anti-cheating requirements

`models/model.mjcf` must represent the same named parts and final poses as `assembly.json`.

1. Every equality must be traceable to an explicit authored transmission/relation.
2. Equality ratio and direction must exactly match the authored transmission; do not infer or
   repair missing/zero ratios from geometry.
3. Ideal gear/planetary equality is permitted only when disclosed in
   `raw/evaluator_result.json`; exclude only the exact represented mesh pair.
4. Do not broad-exclude all nearby, parent-child, coaxial, fixed, or moving pairs.
5. Do not exclude a rod from a whole crank/web/carrier body when that would disable meaningful
   rod-vs-solid collision.
6. A pair with positive AABB overlap on all axes and unavailable/positive exact solid overlap
   remains collidable unless its exact authored relation is a valid geometric interface.
7. Sampled positive surface distance does not prove separation for overlapping AABBs.
8. Support/debug derivatives must be stored separately and must not replace `models/model.mjcf`.
9. Do not directly drive the registered output.
10. Do not add hidden welds, extra actuators, keyframes, mocap bodies, or constraints solely to
    manufacture the target metric.

---

## 8. Trajectory and contact evidence

### `evidence/trajectory.json`

Minimum schema:

```json
{
  "t": [0.0, 0.01, 0.02],
  "driver": "input_joint_name",
  "units": {"joints": "rad_or_m", "bodies": "mm", "time": "s"},
  "joints": {
    "input_joint_name": [0.0, 0.1, 0.2],
    "output_joint_name": [0.0, -0.025, -0.05]
  },
  "joint_meta": {
    "input_joint_name": {"kind": "spin", "unit": "rad"},
    "output_joint_name": {"kind": "spin", "unit": "rad"}
  },
  "bodies": {
    "input_part": [[0.0, 0.0, 10.0], [0.0, 0.0, 10.0], [0.0, 0.0, 10.0]]
  },
  "finite_health": {"all_finite": true}
}
```

Requirements:

- At least 200 aligned samples when simulation reaches the requested horizon.
- Include every registered input/output joint, expected moving coordinate, watched link, and
  body needed to verify carrying, axis drift, orbit, circularity, or closure.
- Record body positions in millimetres and joint hinge coordinates in radians; slide joints
  may be metres when declared in `joint_meta`.
- Preserve actual sample timestamps.
- Never serialize NaN or Infinity.
- Record equality/closure residual series when available.

### `evidence/contacts.json`

```json
{
  "contacts": [],
  "n_pairs": 0
}
```

If contacts are sampled over time, include pair names, sample index/time, distance,
position, and force/impulse where available. Empty contacts must mean measured none, not
measurement omitted.

---

## 9. Evaluator behavior

The harness may use its own evaluator for refinement, but the final independent score does
not trust the harness verdict. Preserve raw evaluator output under `raw/`.

Evidence priority:

1. joint/body trajectories and constraint residuals;
2. finite state, contacts, forces, and stability;
3. video;
4. VLM or natural-language summaries.

For ratio tasks, use unwrapped net angles and stable-window regression. For reciprocating
outputs, use span and hysteretic reversal count, never endpoint displacement alone.

Tasks 7 and 9 require finite-effort input. Direct-qpos may not replace them. Ideal/direct
kinematic tests for other tasks must be disclosed and cannot claim real tooth-contact or
load capacity.

---

## 10. Frozen tasks

The prompt text between each `PROMPT BEGIN/END` marker must be passed verbatim, preserving
newlines and punctuation. Its UTF-8 SHA-256 is provided for verification.

### 01_single_stage_4to1

- Prompt SHA-256: `cbeab4782cbb54e33cf06573f6bc79d9868a43792f0878e6234ba79a262f3f47`
- Roles: `input_shaft=1`, `output_shaft=1`, `gear=2`, `hand_crank=1`
- Input minimum: `6 rad`
- Output: `3.8 <= |theta_in/theta_out| <= 4.2`, opposite direction
- Invariants: fixed shaft axes, rigid gear carrying, one live mesh

**PROMPT BEGIN**

Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera. Build an open-frame hand-cranked single-stage spur gear reducer with an exact 4:1 reduction. Use one input shaft with a visible hand crank and one parallel output shaft. Support both shafts in clearly visible running bearings, rigidly attach each gear to its own shaft, expose the complete tooth mesh, and include a stable bench-mounted base. The input must be the hand crank only; the output shaft must not be independently driven. Author complete mechanism semantics for the shaft hinges, press fits, running fits, gear mesh, driver, output, and watched links.

**PROMPT END**

### 02_two_stage_9to1

- Prompt SHA-256: `983dcc69a758553550e055d7eb8dd1455dd7513c86f202b145ee29048e8776e9`
- Roles: `input_shaft=1`, `compound_intermediate_shaft=1`, `output_shaft=1`, `gear=4`, `hand_crank=1`
- Input minimum: `9 rad`
- Output: `8.5 <= |theta_in/theta_out| <= 9.5`, same direction
- Invariants: three fixed parallel axes, rigid compound pair, two live meshes

**PROMPT BEGIN**

Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera. Build an open-frame hand-cranked two-stage spur gear reducer with an exact overall 9:1 reduction, using two 3:1 stages. Use three parallel shafts: an input shaft with one visible hand crank, a compound intermediate shaft carrying both its driven gear and the second-stage pinion, and one output shaft. Support every shaft in visible running bearings, expose both tooth meshes, and mount the machine on a stable base. Only the input crank is driven. Author complete mechanism semantics, including the compound rigid carrying, both transmissions, driver, output, and watched links.

**PROMPT END**

### 03_idler_reverser_1to1

- Prompt SHA-256: `5014e249562b5ca4220b5a27bad4eccfa2eec39c3e212efff5f2bcaead1cd4c4`
- Roles: `input_shaft=1`, `idler_shaft=1`, `output_shaft=1`, `gear=3`, `hand_crank=1`
- Input minimum: `6 rad`
- Output: `0.95 <= |theta_out/theta_in| <= 1.05`, same direction after two external meshes
- Invariants: fixed axes, independent idler hinge, two live meshes

**PROMPT BEGIN**

Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera. Build an open-frame three-shaft spur gear reversing train with one input gear, one freely rotating idler gear, and one output gear. Use equal tooth counts for the input and output so the magnitude of the overall ratio is exactly 1:1. All three shaft axes must be parallel and fixed in the world, all two gear meshes must be fully visible, and only the input shaft has a hand crank. The idler must have its own independent hinge and must not be welded to either neighboring gear. Include a stable bench-mounted base and complete mechanism semantics for both meshes, all bearings, driver, output, and watched links.

**PROMPT END**

### 04_openwork_clock_12to1

- Prompt SHA-256: `7c5ff808279f6aefe1a1dfd1cc1623c54fbb36c57e2fae73a67dd99ce17c6abb`
- Roles: `minute_input=1`, `hour_output=1`, `coaxial_hand=2`
- Input minimum: `12 rad`
- Output: `11.4 <= |theta_minute/theta_hour| <= 12.6`
- Invariants: coaxial independent hands, both hands remain carried

**PROMPT BEGIN**

Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera. Build an openwork mechanical clock display with two clearly visible coaxial hands whose angular speed ratio is exactly 12:1. Use a visible spur gear train, independent coaxial shafts or sleeves with running clearances, rigidly attach each hand to its intended shaft, and expose the gears and both hands from the camera side. Mount the frame rigidly on a stable base. The minute-side input is the only driver; the hour hand is the final output. Coaxial members with different speeds must remain independent and must not be welded or forced to 1:1. Author complete mechanism semantics for every transmission, bearing, press fit, driver, output, and watched link.

**PROMPT END**

### 05_three_planet_4to1

- Prompt SHA-256: `c07c0576b0bb8de5860f094fe734feb28f9beefd324b902881518aaacf6c1af3`
- Roles: `fixed_ring=1`, `sun_input=1`, `carrier_output=1`, `planet_gear=3`, `planet_pin_hinge=3`, `hand_crank=1`
- Input minimum: `12 rad`
- Output: `0.2375 <= |theta_carrier/theta_sun| <= 0.2625`
- Invariants: ring fixed, three equally spaced planets, orbit, local spin, constant gear-pin distance

**PROMPT BEGIN**

Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera. Build an open-frame hand-driven planetary reducer with a fixed ring gear, a sun gear input, exactly three equally spaced planet gears on a rigid carrier, and the carrier as output. Choose tooth counts that give an exact 4:1 sun-to-carrier reduction with the ring fixed. Every planet gear must be carried around the sun by the carrier while also spinning on its own dedicated carrier pin hinge. Expose the sun, all three planets, the ring, and the carrier; do not hide them behind a full cover. Include a stable base, a visible input crank, and complete planetary-stage semantics, meshes, bearings, driver, output, and watched links.

**PROMPT END**

### 06_four_planet_4to1

- Prompt SHA-256: `1000daaca368fd525fbae6dd63616133ef0a2e4046d74cef82b59d77ab8b321d`
- Roles: `fixed_ring=1`, `sun_input=1`, `carrier_output=1`, `planet_gear=4`, `planet_pin_hinge=4`, `hand_crank=1`
- Input minimum: `12 rad`
- Output: `0.2375 <= |theta_carrier/theta_sun| <= 0.2625`
- Invariants: ring fixed, four planets spaced about 90 degrees, orbit, local spin, constant gear-pin distance

**PROMPT BEGIN**

Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera. Build an open-frame hand-driven planetary reducer with a fixed ring gear, a sun gear input, exactly four equally spaced planet gears on a rigid carrier, and the carrier as output. Choose tooth counts that give an exact 4:1 sun-to-carrier reduction with the ring fixed. Every planet gear must be carried by the carrier and must also spin on its own dedicated carrier pin hinge. Keep all four planets and their pins visibly exposed, use a stable base and visible input crank, and author complete planetary-stage semantics, meshes, bearings, driver, output, and watched links.

**PROMPT END**

### 07_horizontal_slider_crank

- Prompt SHA-256: `f4cbbbd5113248136792f571b5274e449402f0cae3642588cc96d7c9f77716d2`
- Roles: `crankshaft_input=1`, `crank_pin=1`, `connecting_rod=1`, `horizontal_slider=1`, `horizontal_guide=1`
- Input minimum: `2*pi rad`
- Output: horizontal stroke at least `20 mm`, at least two reversals
- Invariants: fixed crank axis, lateral drift at most 2% of stroke, closure residual below 2% of the relevant pin/port scale
- Required mode: finite-effort input; direct-qpos is forbidden

**PROMPT BEGIN**

Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera. Build an open-frame horizontal hand-cranked slider-crank mechanism. Fix the crankshaft axis rigidly to the base, attach one crank disk or crank web and one dedicated eccentric crank pin, connect it through a single rigid connecting rod to a slider constrained to one horizontal linear guide, and expose the complete crank, both rod ends, and slider. Use only one crankshaft input and do not directly actuate the slider. Author explicit revolute, slide, pin, closure, driver, output, and watched-link semantics. Keep rod collisions against the main shaft, web, frame, and guide physically meaningful; do not broadly exclude the whole rod from the crank body.

**PROMPT END**

### 08_vertical_piston_pump

- Prompt SHA-256: `aee082acf487794db4dcbc8fbd51a7168ddb9a51d2cea6be9b7941e3971eb4e4`
- Roles: `crankshaft_input=1`, `eccentric_pin=1`, `connecting_rod=1`, `vertical_crosshead=1`, `vertical_guide=1`, `pump_rod=1`, `piston_output=1`
- Input minimum: `2*pi rad`
- Output: vertical span at least `15 mm`, at least two reversals
- Invariants: fixed crank axis, rod-crosshead closure, rigid output carrying, no false body-ground collision

**PROMPT BEGIN**

Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera. Build an open-frame bench-mounted vertical reciprocating piston-pump mechanism driven by a horizontal hand crank. Use one fixed crankshaft, one eccentric crank pin, one connecting rod, one guided vertical crosshead or piston slider, and one visible pump rod/piston moving only vertically inside an open cylinder frame. Keep the mechanism above the ground plane, expose the crank and rod linkage, and use only the crankshaft as input. This is a mechanical motion benchmark; do not claim or simulate fluid pressure. Author complete revolute, slide, closure, rigid-carrying, driver, output, and watched-link semantics.

**PROMPT END**

### 09_open_pumpjack

- Prompt SHA-256: `23e1e29a5175ec1eb4b698f49aaea0f8fa1c9f0d6501843565a485b140f4b221`
- Roles: `crankshaft_input=1`, `hand_crank=1`, `crank_disk=1`, `crank_pin=1`, `pitman_rod=1`, `walking_beam=1`, `beam_pivot=1`, `polished_rod_output=1`, `vertical_guide=1`
- Input minimum: `2*pi rad`
- Output: vertical span at least `15 mm`, at least two reversals
- Invariants: fixed crank axis and beam pivot, beam pivots, all closures remain connected, output lateral drift at most 5% of stroke
- Required mode: finite-effort input

**PROMPT BEGIN**

Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera. Build an open-frame hand-cranked pumpjack mechanism on a stable base. Use one fixed horizontal crankshaft with a hand crank, one rotating crank disk and dedicated crank pin, one pitman connecting rod, one pivoted walking beam on a fixed central support, and one vertical polished-rod output guided to reciprocate. Keep the crank, pitman, beam pivot, and output rod fully visible. Only the crankshaft is driven. Author explicit hinge, pin, closure, slide or guided-output, driver, output, and watched-link semantics. This benchmark tests mechanical motion only, not underground fluid extraction.

**PROMPT END**

### 10_wind_rotor_pump

- Prompt SHA-256: `433b8ecd2d77f39e0b289f685c100c9a95e6f29cce382845be7f600472d2a10a`
- Roles: `rotor_shaft_input=1`, `wind_rotor=1`, `crank_disk=1`, `crank_pin=1`, `connecting_rod=1`, `vertical_crosshead=1`, `vertical_guide=1`, `pump_rod=1`, `piston_output=1`
- Input minimum: `2*pi rad`
- Output: vertical span at least `15 mm`, at least two reversals
- Invariants: fixed rotor/crank axis, circular crank-pin path, vertical output, rigid output carrying, closure residual below 2% scale

**PROMPT BEGIN**

Design this as an open demonstration mechanism: expose the complete mechanical structure and motion path, and avoid covers, housings, bridges, or plates that obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output from the simulation camera. Build an open-frame wind-rotor-driven reciprocating pump on a stable tower or bench frame. Use one fixed horizontal rotor shaft, a clearly visible wind rotor rigidly attached to it, one crank disk with a dedicated eccentric pin, one connecting rod, one guided vertical crosshead, and a visible pump rod/piston output that moves only vertically. Keep the entire rotor-to-crank-to-piston motion path exposed and use the wind rotor shaft as the only input. Author explicit world-frame shaft hinge, crank, pin, closure, slide, rigid-carrying, driver, output, and watched-link semantics. The benchmark tests imposed rotor-driven mechanical transmission, not aerodynamic power generation or fluid pressure.

**PROMPT END**

---

## 11. Suite completion and handoff

After all ten task processes exit:

1. Verify that all ten task directories exist.
2. Verify that `suite_config.json` and each `raw/run.json` identify the actual method/model.
3. Do not run or edit an independent benchmark score.
4. Print only a concise completion summary and the absolute `OUTPUT_ROOT`.
5. The user will provide that root to the independent scorer.

The independent scoring phase will:

- reexecute `source/machine.py`;
- validate the nonempty named part set and assembly references;
- perform scorer-owned exact-solid intersection checks;
- accept only exact authored geometry exemptions;
- independently load/replay MJCF;
- compute ratio, stroke, reversals, motion coverage, axis/carrying/closure residuals;
- produce cumulative Execution/Assembly/Geometry/Physics/Functional results;
- report all remaining missing evidence as `UNKNOWN`, never silently as PASS.
