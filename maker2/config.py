"""Settings for the workflow4freecad orchestrator.

Per-field resolution order: explicit override (CLI) > environment variable >
optional JSON config file > built-in default. The Settings object is built once
in main.py and threaded through manager/worker/orchestrator by dependency
injection — package modules never import this module, which keeps them
decoupled from where configuration actually comes from.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, fields
from typing import Any


# Verified install location of the bundled headless FreeCAD CLI on this machine.
# freecad_runner.find_freecadcmd() falls back through several candidates if this
# path is wrong on another machine.
_DEFAULT_FREECADCMD = r"C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe"


@dataclass
class Settings:
    """All knobs for one orchestrator run."""

    # ── LLM gateway ──────────────────────────────────────────────
    provider_name: str = "local_gateway"
    base_url: str = "http://127.0.0.1:8313/v1"   # the /v1 suffix is required
    api_key: str = "sk-xxx"                       # placeholder; override via env
    model: str = "claude-opus-4.8"
    temperature: float = 0.2
    manager_max_tokens: int = 32000               # claude-opus-4.8 supports 64K
                                                   # output (per the Copilot proxy's
                                                   # model limits); 32K gives large
                                                   # decompositions room with
                                                   # headroom under the real cap.
    worker_max_tokens: int = 128000                # one subassembly's whole-batch
                                                   # SCAD. 128K (under the 128K model
                                                   # cap) so a detailed batch fits;
                                                   # the worker also streams with
                                                   # completion-continuation if a
                                                   # reply still overruns.
    judger_max_tokens: int = 16000                 # the evaluator's verdict JSON is
                                                   # small (pass/reasons/
                                                   # suggestions); well under the cap.
    boss_max_tokens: int = 32000                  # the boss's SubassemblyPlan is
                                                   # usually small, but 32K leaves
                                                   # room for a many-subassembly
                                                   # machine without truncating.
    llm_timeout: int = 600                         # s per LLM request; a non-
                                                   # streaming send must finish the
                                                   # whole completion within this

    # ── Geometry worker backend ──────────────────────────────────
    worker_backend: str = "cadquery"              # "cadquery" (default; curved
                                                   # geometry via OCCT) | "openscad"
                                                   # (legacy fallback). The hierarchy
                                                   # build dispatches on this.
    cross_sub_solver: str = "slvs"               # authoritative cross-sub placement:
                                                   # "slvs" (required) | "closed_form" (legacy debug)
    deep_think: bool = True                       # the single speed/power toggle
                                                   # (maker2-mujoco-contact Phase 6):
                                                   # True  -> CadQuery worker + FULL
                                                   #          debugger (whole-sub context,
                                                   #          extended thinking).
                                                   # False -> OpenSCAD worker + SLIM
                                                   #          debugger (2 conflicting parts,
                                                   #          thinking off, 1 try).
                                                   # run_boss derives worker_backend +
                                                   # debugger_mode from this when set.

    # ── Physics engine (maker2-mujoco-contact) ───────────────────
    engine: str = "mujoco"                        # "mujoco" (default on this branch:
                                                   # pure contact under gravity, transmission
                                                   # by tooth contact, no motors) | "pybullet"
                                                   # (legacy joint-motor sim). The MuJoCo path
                                                   # (mjcf_builder, convex_decomp,
                                                   # run_scenario_mujoco) builds its MJCF from
                                                   # the model JSON; the visual URDF is kept
                                                   # only for the appearance-judge render.
    allow_gear_constraint: bool = False           # escape hatch: if a specific gear pair
                                                   # won't transmit by pure tooth contact,
                                                   # add a MuJoCo <equality> gear-ratio
                                                   # constraint for THAT pair. OFF by
                                                   # default (defeats "literal contact").
    base_rests_on_plane: bool = True              # MuJoCo: the assembly settles on the
                                                   # ground plane under gravity (literal).
                                                   # False pins the base to remove settle
                                                   # noise when only transmission matters.
    score_weights: dict = None                    # override the keep-best score weights
                                                   # (transmission/stability/overlap/judge);
                                                   # None -> score.py's calibration draft
                                                   # 0.45/0.25/0.15/0.15. Exposed so a
                                                   # labeled-run sweep can retune it.
    score_target: float = 0.9                     # boss stops when the design score
                                                   # reaches this (score-gated iteration).
    score_plateau: int = 3                        # stop after this many iterations with
                                                   # no accepted improvement.
    max_total_iters: int = 40                     # SAFETY: absolute ceiling even in
                                                   # "infinite" mode, so an unbuildable
                                                   # plan can't loop for hours.
    max_no_progress_iters: int = 8                # SAFETY: give up after this many
                                                   # consecutive iterations that never
                                                   # reach a physics score (stuck loop).

    # ── FreeCAD subprocess ───────────────────────────────────────
    freecadcmd_path: str = _DEFAULT_FREECADCMD
    worker_timeout: int = 120                     # seconds per freecadcmd run

    # ── Retry / parallelism ──────────────────────────────────────
    manager_retries: int = 2                      # JSON-repair attempts
    worker_retries: int = 3                       # rebuild attempts per part
    judger_retries: int = 2                       # verdict parse/repair attempts

    # ── Monotonic-improvement loops (Part C.bis: badness keep-best + escalation) ──
    loop_plateau_k: int = 2                       # a loop ESCALATES (changes approach:
                                                   # coarsen / split / carve differently)
                                                   # after this many attempts with no
                                                   # badness improvement, instead of a
                                                   # blind repeat. Used by manager
                                                   # _decompose_loop, boss _plan_loop, and
                                                   # the boss assembled keep-best.
    sub_best_of: int = 2                          # C7: generate this many manager
                                                   # decompositions per subassembly and
                                                   # keep the LOWEST pre-render badness one
                                                   # (pure-Python gates; no render). 1
                                                   # disables best-of-N.
    enable_sub_split: bool = True                 # C8: when a sub's realized link count /
                                                   # est_link_budget exceeds
                                                   # sub_split_threshold, halve it (run the
                                                   # manager on two named halves and merge)
                                                   # rather than overloading one manager.
    sub_split_threshold: int = 12                 # C8: the link count above which a sub is
                                                   # carved in two.
    judge_max_iterations: int = 3                 # generate->judge->refine passes
                                                   # before the --judge loop stops
                                                   # regardless of the verdict
    max_workers: int = 9                          # builds kept in flight at once
                                                   # by the continuous worker pool.
                                                   # Each worker's LLM send retries
                                                   # with backoff, so the gateway
                                                   # absorbs this sustained load.
                                                   # Raise via --max-workers if
                                                   # your gateway copes.

    # ── Output / behavior ────────────────────────────────────────
    do_viz: bool = True                           # show assembled URDF at the end
    allow_partial: bool = False                   # succeed even if some links fail

    # ── Hierarchy (boss -> managers -> assembler); off by default so the
    #    single-manager pipeline is unchanged until --hierarchy flips it on. ──
    enable_hierarchy: bool = False                # boss splits into subassemblies
    enable_hierarchy_judge: bool = True           # VLM appearance judge on the assembled
                                                   # machine before physics (catches parts
                                                   # floating/disconnected — precheck and
                                                   # physics do not)
    enable_solver_failure_analyzer: bool = True   # self-directed read-only investigation before Boss
    analyzer_max_tokens: int = 16000
    solver_analyzer_max_rounds: int = 12
    solver_local_repair_max_attempts: int = 2
    enable_solver_pose_repair: bool = True
    enable_solver_module_repair: bool = False
    enable_solver_seat_geometry_repair: bool = True
    enable_sub_conflict_gate: bool = True         # check each subassembly for rigid
                                                   # part interpenetration right after it
                                                   # builds, and run a debugger loop to
                                                   # fix it before the sub is accepted
    sub_conflict_max_tries: int = 3               # debugger passes before failing the
                                                   # sub up to the boss for a re-plan
    subassembly_max_managers: int = 9             # parallel per-sub manager builds
    enable_reference_tools: bool = False          # web/RAG reference lookup (Stage G)
    enable_kb: bool = False                        # local offline retrieval (maker2/kb):
                                                   # a curated per-agent knowledge base +
                                                   # a growing memory of passing runs,
                                                   # offered as a kb_search tool in the
                                                   # boss/manager/worker research pre-step.
                                                   # Independent of enable_reference_tools
                                                   # (web); either/both may be on.
    enable_appearance_proxy: bool = True          # boss-side coarse CadQuery proxy of
                                                   # the whole machine (per-sub bounding
                                                   # primitives at global pose) rendered
                                                   # BEFORE the managers build detail, and
                                                   # handed to them as proportion context
                                                   # (1c). On by default.
    manager_ir: bool = True                        # manager authors a CONNECTION GRAPH
                                                   # (parts + mates) that mate_solver solves
                                                   # into poses, instead of authoring an MJCF
                                                   # skeleton (pos/quat). ON by default;
                                                   # --no-manager-ir falls back to the MJCF
                                                   # skeleton path (mjcf_skeleton.py). See
                                                   # Part A of the plan.

    # ── Construction helpers ─────────────────────────────────────

    @classmethod
    def load(cls, config_path: str | None = None, **overrides) -> "Settings":
        """Build Settings from defaults < JSON file < env vars < explicit overrides."""
        data: dict[str, Any] = {}

        path = config_path or os.environ.get("WORKFLOW4FREECAD_CONFIG")
        if path and os.path.isfile(path):
            with open(path, "r", encoding="utf-8") as f:
                data.update(json.load(f))

        env_map = {
            "base_url": "FREECAD_AI_BASE_URL",
            "api_key": "FREECAD_AI_API_KEY",
            "model": "FREECAD_AI_MODEL",
            "freecadcmd_path": "FREECADCMD",
        }
        for field_name, env_name in env_map.items():
            val = os.environ.get(env_name)
            if val:
                data[field_name] = val

        data.update({k: v for k, v in overrides.items() if v is not None})

        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in known})

    def make_client(self, max_tokens: int, thinking: str = "off"):
        """Construct an LLMClient bound to these settings (lazy import).

        Imported lazily so config.py stays importable without the package on
        the path (e.g. when only inspecting settings).

        ``thinking`` maps to the gateway's reasoning budget ("off" | "on" |
        "extended" -> reasoning_effort none | medium | high). For claude-opus-4.8
        on the Copilot proxy this is LOAD-BEARING on a big prompt: with NO effort
        set, the model runs an UNBOUNDED hidden thinking phase that consumes the
        entire max_tokens before any content streams (empty, cap-truncated
        response); setting effort BOUNDS the thinking so visible output actually
        flows. Verified: a "create a car" boss plan returns 0 content at 32K with
        effort unset, but a complete 8-subassembly plan (finish=stop) with
        "extended". So the boss/manager (large structured outputs) run with
        thinking on; the worker/judger stay off.
        """
        from maker2.llm.client import LLMClient
        return LLMClient(
            provider_name=self.provider_name,
            base_url=self.base_url,
            api_key=self.api_key,
            model=self.model,
            max_tokens=max_tokens,
            temperature=self.temperature,
            thinking=thinking,
            timeout=self.llm_timeout,
        )

    def manager_client(self):
        """An LLMClient sized for the manager's decomposition output."""
        return self.make_client(self.manager_max_tokens, thinking="extended")

    def worker_client(self):
        """An LLMClient sized for a worker's code output."""
        return self.make_client(self.worker_max_tokens)

    def judger_client(self):
        """An LLMClient sized for the evaluator's verdict JSON."""
        return self.make_client(self.judger_max_tokens)

    def boss_client(self):
        """An LLMClient sized for the boss's SubassemblyPlan output."""
        return self.make_client(self.boss_max_tokens, thinking="extended")

    # ── Deep-think toggle derivations (Phase 6) ──────────────────
    def effective_worker_backend(self) -> str:
        """The geometry backend to use: deep_think picks CadQuery, else OpenSCAD.
        An explicit worker_backend other than the deep-think default still wins if the
        caller set it deliberately — but the toggle is the intended control."""
        return "cadquery" if self.deep_think else "openscad"

    def debugger_mode(self) -> str:
        """The subassembly debugger depth: FULL (whole-sub, extended thinking) when
        deep_think, else SLIM (two conflicting parts, thinking off, one try)."""
        return "full" if self.deep_think else "slim"

    def debugger_max_tries(self) -> int:
        """How many debugger passes per conflict: the full sub_conflict_max_tries when
        deep_think, else 1 (slim is a single shallow pass)."""
        return self.sub_conflict_max_tries if self.deep_think else 1
