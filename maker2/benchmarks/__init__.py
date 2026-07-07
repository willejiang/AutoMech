"""Deterministic benchmark GATES for the boss->manager->worker pipeline.

Each gate is a pure-Python function that inspects one agent's output and returns a list
of GateError (empty = pass). Gates NEVER call an LLM: they catch a fault with math/graph
checks and hand the responsible agent a specific error code, BEFORE the expensive next
stage runs. This replaces routing routine geometry/interface/schema faults through the
slow LLM debugger.

Gate order (cheapest first): schema_gate (JSON validity, no geometry) -> boss_gate
(plan support + mesh distance, no build) -> manager_gate (per-sub overlap + connectivity,
no render) -> worker_gate (per-part dims + manifold) -> assembled_gate (weld chain +
grounding, post-stitch).

See .claude/plans/precious-humming-wand.md.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GateError:
    """One deterministic benchmark failure, routed back to the agent that owns it.

    ``layer`` names the responsible agent ("boss" | "manager" | "worker") so the
    orchestrator re-runs only that stage. ``code`` is a stable machine token (e.g.
    ``ERR_OVL``, ``ERR_SCHEMA_DRIVER_FIXED``) the LLM reacts to. ``detail`` is a short
    human/LLM-readable explanation. ``culprit`` is the specific offender — a sub id, a
    link name, a seam id, or "linkA~linkB" for a pair — so a fix is localized.
    """

    layer: str
    code: str
    detail: str
    culprit: str = ""

    def __str__(self) -> str:
        loc = f" [{self.culprit}]" if self.culprit else ""
        return f"{self.code}{loc}: {self.detail}"


def format_errors(errors: list["GateError"]) -> str:
    """Join gate errors into one feedback string for an agent re-run prompt."""
    return "\n".join(f"- {e}" for e in errors)
