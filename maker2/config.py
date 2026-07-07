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

    # ── Physics engine (maker2-mujoco-contact) ───────────────────
    engine: str = "pybullet"                      # "pybullet" (default, legacy joint-
                                                   # motor sim) | "mujoco" (pure contact
                                                   # under gravity). The MuJoCo path
                                                   # (mjcf_builder, convex_decomp,
                                                   # run_scenario_mujoco) is ONLY imported
                                                   # when this is "mujoco", so the default
                                                   # run is unaffected until it flips.
    allow_gear_constraint: bool = False           # escape hatch: if a specific gear pair
                                                   # won't transmit by pure tooth contact,
                                                   # add a MuJoCo <equality> gear-ratio
                                                   # constraint for THAT pair. OFF by
                                                   # default (defeats "literal contact").
    base_rests_on_plane: bool = True              # MuJoCo: the assembly settles on the
                                                   # ground plane under gravity (literal).
                                                   # False pins the base to remove settle
                                                   # noise when only transmission matters.

    # ── FreeCAD subprocess ───────────────────────────────────────
    freecadcmd_path: str = _DEFAULT_FREECADCMD
    worker_timeout: int = 120                     # seconds per freecadcmd run

    # ── Retry / parallelism ──────────────────────────────────────
    manager_retries: int = 2                      # JSON-repair attempts
    worker_retries: int = 3                       # rebuild attempts per part
    judger_retries: int = 2                       # verdict parse/repair attempts
    judge_max_iterations: int = 3                 # generate->judge->refine passes
                                                   # before the --judge loop stops
                                                   # regardless of the verdict
    max_workers: int = 6                          # builds kept in flight at once
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
    enable_sub_conflict_gate: bool = True         # check each subassembly for rigid
                                                   # part interpenetration right after it
                                                   # builds, and run a debugger loop to
                                                   # fix it before the sub is accepted
    sub_conflict_max_tries: int = 3               # debugger passes before failing the
                                                   # sub up to the boss for a re-plan
    subassembly_max_managers: int = 4             # parallel per-sub manager builds
    enable_reference_tools: bool = False          # web/RAG reference lookup (Stage G)
    enable_appearance_proxy: bool = True          # boss-side coarse CadQuery proxy of
                                                   # the whole machine (per-sub bounding
                                                   # primitives at global pose) rendered
                                                   # BEFORE the managers build detail, and
                                                   # handed to them as proportion context
                                                   # (1c). On by default.

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
