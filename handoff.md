# HANDOFF — reducer pipeline hardening (branch: premanager-geometry-compiler)

State of the maker2 gear-reducer pipeline after a debugging session that took it from
"fails in iteration 0" to "reaches the authoritative assembly solve, one geometry blocker
left before precheck/render". This documents what was fixed, the ONE root cause behind most
of it, what still blocks a full run, and how to continue.

## The one root cause behind most failures

**Two frame-name vocabularies coexisted for the same interfaces**, and every by-name gate
silently no-opped when they disagreed:

- Boss plan authored names like `seat_input_front`, `in_front`, `inter_pinion`.
- The geometry compiler (authoritative) + the manager realized `housing_input_stage_front_bore`,
  `input_stage_front_bearing`, `intermediate_stage_stage_1_mesh`.

By-name gates (`frame_drift_errors`, seam agreement) and the assembler mount/mesh lookups
compared boss names against realized compiler names, missed on every frame, and either
errored late (at the libslvs solve) or passed a broken sub (collapsed housing seats slipped
through to the final solve).

**The durable fix** (this session's last change): `boss.unify_plan_frame_names(plan, contract)`,
called in `orchestrator_boss.py` right after the hardpoint contract is frozen. It rewrites the
boss plan's `sub.frames[].name` and ALL seam frame refs to the compiler-contract names,
matched deterministically by role + positional order (NOT nearest-distance — boss coords are
rough and not to scale, so order-matching within each role is what's correct). After this,
one vocabulary flows everywhere and the by-name gates work again.

## What was fixed this session (all on branch `premanager-geometry-compiler`)

Commit `19238a8` (already pushed) + the frame-name unification (this commit):

1. **bridge.py** — recognize reducer topology by role KEYWORDS (`sub_input`→`input_stage`),
   not exact ids, so boss's natural naming binds to the compiler.
2. **gear_reducer.py** — attach pitch diameter to mesh hardpoints. **gates.py** — cross-sub
   axial mesh-plane consistency check.
3. **manager.py gate + manager_prompt.py** — a `mesh` frame MUST be realized on a gear link
   (`ERR_MESH_FRAME_NOT_ON_GEAR`); COAXIAL STATIONS rule (space shaft parts by axial station).
4. **assembler.py** — `_gear_link_disambiguated`: when name/frame lookup fails, pick the sub's
   remaining unclaimed gear (fixes two-gears-on-one-shaft, e.g. intermediate stage).
5. **precheck.py** — general machine-agnostic weld-frame coincidence gate (`weld_frame_coincidence`).
6. **cq_worker.py** — carry shared helpers/constants into split per-part scripts (fixes
   `NameError: _make_gear` on gear rebuild).
7. **convex_decomp.py** — drop sub-floor convex slivers MuJoCo rejects ("mesh volume too small").
   Cache version bumped v1→v2.
8. **orchestrator_boss.py** — reuse baseline computed from `frame_contract_for` (compiled subs
   stopped force-rebuilding every iteration).
9. **slvs_adapter.py** — `_resolve_frame`: tolerant mount-frame lookup (falls back to
   role/side + stage-hint against realized compiler names when boss renamed a seam frame,
   e.g. `in_front`→`input_stage_front_bearing`, `seat_input_rear`→`housing_input_stage_rear_bore`).
   NOTE: after frame-name unification (#0) this fallback should rarely trigger, but it's kept
   as defense-in-depth.
10. **boss_prompt.py** — layered COMPUTE ORDER (budget→topology→hardpoints→flesh) + DO/AVOID.
11. **tools.py / run.py** — solver-tool research prompt; `--solver` CLI flag; research round
    cap raised 4→12.
12. **boss.py** — `unify_plan_frame_names` (the root-cause fix above).

## Verified green

Design/precheck/slvs/mate/mjcf golden tests all PASS after every change. `golden_two_gears`
FAILS but is PRE-EXISTING and unrelated — it's a pure MuJoCo physics-coupling test that
touches none of the changed code (no assembler/gate/prompt/frame path).

## How far a real run got (before the unification fix)

Run `output/a_two_stage_gear_reducer_boss_20260719_232445` reached the **authoritative
libslvs assembly solve** — all 4 subs built, assembler merged 23 links, mesh pairs resolved
(the old death-loop is gone). It then failed on: *"housing mount frames 'seat_input_front'
and 'seat_inter_front' realize 0.000 mm apart despite 80mm contract separation"* — the
housing manager collapsed all 6 bearing-seat frames onto `base_plate` origin `[0,0,0]`, and
the collapse gate (`frame_drift_errors`) that should have caught it no-opped due to the
name mismatch. **The unification fix targets exactly this** (gate can now match names), but
it has NOT yet been validated by a fresh end-to-end run.

## What to do next (in order)

1. **Run it fresh and watch for the housing collapse.** With names unified, the per-sub
   `frame_drift_errors` gate should now REJECT a collapsed housing at manager time (before
   assembly), forcing the manager to re-realize seats at distinct positions.
   ```
   FREECAD_AI_MODEL=claude-opus-4.8 PYTHONIOENCODING=utf-8 PYTHONUTF8=1 \
     /c/Users/jzhij/anaconda3/envs/myenv/python.exe -m maker2.run \
     "a two stage gear reducer" --hierarchy --solver --physics --kb --json
   ```
   Watch `output/<newest>/run.log`. Milestones: `progress 4/4` → `assembler merged` →
   `authoritative solve OK` → `precheck` → `render`.

2. **If the housing STILL collapses** (manager ignores the gate after retries), the deeper
   fix is DETERMINISTIC seat placement: since the compiler contract already holds each
   `housing_*_bore` world position, place housing mount frames from the contract instead of
   trusting the manager's realized `[0,0,0]`. Site: `orchestrator_boss.py:_sub_frames_to_dict`
   (the root-body fallback path ~lines 140-149) — re-seat root-body mount frames from
   `fr.xyz_m` (minus sub origin). This was investigated but NOT implemented; it's the
   belt-and-suspenders option if the gate alone doesn't force the manager to comply.

3. **Patch-path mesh frames**: the mesh-frame gate runs on the full decompose path; the
   manager PATCH/rebuild path (`manager skip-check: REBUILD` → `patched`) was seen to emit
   `did not realize interface frame ['..._stage_1_mesh']` without the gate blocking. If runs
   stall there, ensure the patch path re-runs the same frame gate.

## Planned next: "Plan-乙" — the LLM geometry-authority audit (NOT yet started)

This is the strategic follow-up agreed at the end of the session, and the recommended thing
to do AFTER a run reaches precheck. The thesis, proven repeatedly this session: **almost every
blocker was one bug class — an LLM (boss or manager) holding authority over geometry/naming
that the deterministic compiler should own, and a by-name check silently failing to catch the
divergence.** Each fix so far patched one instance. Plan-乙 is to stop playing whack-a-mole and
systematically CLOSE the class.

Concrete task for tomorrow:

- **Produce an audit**: enumerate every place in the pipeline where an LLM output can still
  OVERRIDE or DIVERGE FROM the compiler's authoritative geometry/naming. Known members of the
  class already found (use as seeds): boss authoring seat/mesh coordinates and frame names
  (now unified — but verify no other path reintroduces boss names); manager choosing where to
  realize a frame (mesh-on-gear, seat spread); manager PATCH path bypassing the frame gate
  (#3 above); any seam field the boss can rename on re-plan; the `_lock_interface_frame_names`
  path in boss.py (position/count-gated, defeated before — audit whether unification makes it
  redundant or still needed).
- **For each audited point, classify**: (i) already deterministic (compiler owns it), (ii)
  LLM proposes + a by-name gate must catch divergence — verify the names line up post-unify
  so the gate actually fires, or (iii) LLM still holds raw authority with no deterministic
  backstop — these are the remaining holes to close.
- **The closing principle** (make it a design rule): *anything geometric/positional/naming is
  owned by the `HardpointContract` (the compiler); the LLM may only REFERENCE it, never
  rewrite it.* `unify_plan_frame_names` is the first systematic application of this rule;
  Plan-乙 finds and fixes the rest.
- **Deliverable**: a short audit doc (holes ranked by how load-bearing they are) + fixes for
  the top holes. The deterministic seat-placement fix (Next-step #2) is likely one of them.

Rationale for doing 乙 (not 丙, "let LLMs write geometry code"): the 丙 probe
(`maker2/experiments/llm_writes_geometry.py`) showed LLM-written geometry is self-consistent
but NOT reproducible or inter-agent-aligned — expanding LLM geometry authority makes the class
WORSE. 乙 shrinks it. See the Design note below.

## Local-only changes NOT pushed (keep local)

- `worker/**` route wiring (`PYTHON_BIN` fallback + auto-adding `--solver`) — for running the
  web UI on this Windows box.
- `.gitignore`, `.vscode/`, debug logs, `CLAUDE.md`, `supabase/`, `.env` (ENVIRONMENT=local
  for billing bypass), `maker2/experiments/llm_writes_geometry.py` (a probe, optional to push).

## Design note (parked decision)

There's an open question whether to have boss/manager WRITE geometry-computing Python (vs the
current "compiler computes, agents reference"). A probe `maker2/experiments/llm_writes_geometry.py`
tested gpt-5.6-sol writing geometry code for reducer/tourbillon/torsen: all ran and were
self-consistent, but self-consistent ≠ reproducible ≠ inter-agent-aligned. Conclusion leaned
toward: keep the compiler authoritative, REDUCE the LLM's geometry authority (the exact thing
`unify_plan_frame_names` does), rather than expand it. Re-run the probe with N=5 on one task to
see variance if revisiting.
