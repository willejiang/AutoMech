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
    manager_max_tokens: int = 16000               # the gateway HARD-CAPS output
                                                   # at 16000 tokens regardless of
                                                   # what we ask; requesting more
                                                   # is futile, so size to the cap
    worker_max_tokens: int = 16000                # one part's FreeCAD code. Sized
                                                   # to the gateway's hard cap so a
                                                   # compound part has room; if a
                                                   # reply still overruns, the
                                                   # worker asks it to shrink and
                                                   # retries (see worker.py).
    judger_max_tokens: int = 8000                 # the evaluator's verdict JSON is
                                                   # small (pass/reasons/
                                                   # suggestions); well under the
                                                   # gateway cap.
    llm_timeout: int = 600                         # s per LLM request; a non-
                                                   # streaming send must finish the
                                                   # whole completion within this

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

    def make_client(self, max_tokens: int):
        """Construct an LLMClient bound to these settings (lazy import).

        Imported lazily so config.py stays importable without the package on
        the path (e.g. when only inspecting settings).
        """
        from workflow4freecad.llm.client import LLMClient
        return LLMClient(
            provider_name=self.provider_name,
            base_url=self.base_url,
            api_key=self.api_key,
            model=self.model,
            max_tokens=max_tokens,
            temperature=self.temperature,
            thinking="off",
            timeout=self.llm_timeout,
        )

    def manager_client(self):
        """An LLMClient sized for the manager's decomposition output."""
        return self.make_client(self.manager_max_tokens)

    def worker_client(self):
        """An LLMClient sized for a worker's code output."""
        return self.make_client(self.worker_max_tokens)

    def judger_client(self):
        """An LLMClient sized for the evaluator's verdict JSON."""
        return self.make_client(self.judger_max_tokens)
