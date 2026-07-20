"""Run a machine's subassembly managers as an AgentTeamRunner team.

This replaces the old one-way boss→manager fan-out (``build_all_subassemblies``). Each
manager is wrapped as a ``ProposalSource`` (:class:`ManagerAgent`) that builds its
subassembly against one shared, revisioned :class:`~maker2.team.TeamState` snapshot and
publishes its realized interface frames back to that state. Across rounds a manager reads
the frames its SIBLINGS realized in the previous round and re-authors to match — the
collaboration the string-only ``feedback_by_sub`` channel could not express.

Determinism and parallelism are preserved: the existing ``ThreadPoolExecutor`` (bounded
by ``settings.subassembly_max_managers``) is injected as the runner's ``collect_fn``, and
``TeamState.commit`` re-sorts proposals into a stable batch regardless of completion order.

``run_subassembly_team`` keeps the exact signature and ``{sub_id: SubResult}`` return of
``build_all_subassemblies``, so both call sites swap with no downstream change.
"""
from __future__ import annotations

import json
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed

from .model import SubResult
from .team import AgentTeamRunner, TeamState, TypedProposal
from .team_schema import (MANAGER_REALIZED_KIND, MANAGER_RESULT_KIND, ManagerRealized,
                          ManagerResult, RealizedFrame, build_team_state)


def _digest_of(model) -> str:
    """A short structural digest of a KinematicModel for the audit-only result payload."""
    try:
        links = tuple(sorted(getattr(l, "name", "") for l in getattr(model, "links", []) or []))
        from .team import stable_digest
        return stable_digest({"root": getattr(model, "root_link", ""), "links": links})[:16]
    except Exception:
        return ""


def _format_sibling_realized(state: dict, own_sub_id: str) -> str:
    """Render the OTHER subs' already-realized interface frames as a feedback block a
    manager can act on, or "" when no sibling has realized anything yet."""
    realized = state.get("realized", {}) or {}
    lines = []
    for sid in sorted(realized):
        if sid == own_sub_id:
            continue
        frames = realized.get(sid) or []
        if not frames:
            continue
        placed = ", ".join(
            f"{fr.get('frame', '?')}@{fr.get('link', '?')}"
            f"{tuple(round(v, 4) for v in (fr.get('local_xyz_m') or ()))}"
            for fr in frames)
        lines.append(f"  - subassembly '{sid}' realized: {placed}")
    if not lines:
        return ""
    return ("A sibling subassembly has already placed its shared interface frames. Match "
            "your mating frames to these so the assembly seams line up:\n" + "\n".join(lines))


class ManagerAgent:
    """A subassembly manager as a team ``ProposalSource``.

    ``propose`` builds this sub (reusing the unchanged ``build_subassembly`` pipeline, so
    every gate and keep-best is preserved), stores the real (mutable) ``SubResult`` in the
    shared ``results_sink`` side dict, and emits frozen ``manager.realized`` /
    ``manager.result`` proposals for the snapshot.
    """

    def __init__(self, spec, plan, settings, session_root, *, results_sink,
                 base_feedback=None, reuse=False, user_prompt="", log_fn=print):
        self.agent_id = spec.id
        self.spec = spec
        self.plan = plan
        self.settings = settings
        self.session_root = session_root
        self._sink = results_sink
        self._base_feedback = base_feedback
        self._reuse = reuse
        self._user_prompt = user_prompt
        self._log = log_fn

    def _build(self, snapshot):
        from .orchestrator_boss import build_subassembly, _load_sub_from_disk

        if self._reuse:
            r = _load_sub_from_disk(self.spec.id, self.session_root,
                                    log_fn=self._log, plan=self.plan, settings=self.settings)
            if r.ok:
                return r
            # A reused sub whose prior build is unusable is rebuilt this round.
            self._reuse = False
            self._base_feedback = ((r.error or "prior build unusable")
                                   + " — rebuild this subassembly.")

        # Sibling-realized frames (this round's shared state) augment any base feedback.
        sibling = _format_sibling_realized(dict(snapshot.state), self.spec.id)
        fb = "\n\n".join(x for x in (self._base_feedback, sibling) if x) or None
        return build_subassembly(self.spec, self.plan, self.settings, self.session_root,
                                 feedback=fb, user_prompt=self._user_prompt, log_fn=self._log)

    def propose(self, snapshot):
        try:
            result = self._build(snapshot)
        except Exception as e:  # build_subassembly shouldn't raise, but be safe
            result = SubResult(id=self.spec.id, ok=False,
                               error=f"raised: {type(e).__name__}: {e}")
        self._sink[self.spec.id] = result

        rev = snapshot.revision
        frames = tuple(RealizedFrame.from_sub_frame(d)
                       for d in (result.sub_frames or []))
        realized = TypedProposal.create(
            author=self.spec.id, base_revision=rev, kind=MANAGER_REALIZED_KIND,
            payload=ManagerRealized(self.spec.id, frames), key=str(rev))
        outcome = TypedProposal.create(
            author=self.spec.id, base_revision=rev, kind=MANAGER_RESULT_KIND,
            payload=ManagerResult(self.spec.id, bool(result.ok),
                                  _digest_of(result.model), result.error or ""),
            key=str(rev))
        return [realized, outcome]


def run_subassembly_team(plan, settings, session_root, *,
                         feedback_by_sub: dict | None = None,
                         reuse: set = frozenset(), user_prompt: str = "",
                         log_fn=print) -> dict:
    """Build every subassembly of `plan` as an AgentTeamRunner team. Returns
    {sub_id: SubResult}. Drop-in replacement for ``build_all_subassemblies``.

    Managers run in parallel each round via the injected ThreadPoolExecutor; over
    ``settings.team_max_rounds`` rounds they react to each other's realized interface
    frames. Reused subs are loaded (and promoted to rebuild if unusable) exactly as before.
    """
    feedback_by_sub = feedback_by_sub or {}
    lock = threading.Lock()

    def log(msg: str) -> None:
        with lock:
            log_fn(msg)

    subs = list(plan.subassemblies)
    reuse_ids = set(reuse)
    n = max(1, min(len(subs) or 1, getattr(settings, "subassembly_max_managers", 4)))
    rounds = max(1, int(getattr(settings, "team_max_rounds", 2)))
    log(f"[boss] team round: {len(subs)} subassembly(ies), up to {n} in parallel, "
        f"reusing {len(reuse_ids)} from disk, {rounds} max round(s)")

    results_sink: dict = {}
    state: TeamState = build_team_state(plan)
    agents = [
        ManagerAgent(s, plan, settings, session_root, results_sink=results_sink,
                     base_feedback=feedback_by_sub.get(s.id), reuse=s.id in reuse_ids,
                     user_prompt=user_prompt, log_fn=log)
        for s in subs
    ]

    def collect_fn(base_snapshot, sources):
        # The ONLY concurrency hook: run each source's propose() in the bounded pool,
        # returning AgentRoundResults. Determinism is restored by the runner + commit sort.
        from .team import AgentRoundResult
        out: list = []
        with ThreadPoolExecutor(max_workers=n) as pool:
            futures = {pool.submit(a.propose, base_snapshot): a for a in sources}
            done = 0
            for fut in as_completed(futures):
                agent = futures[fut]
                proposals = fut.result()  # propose() swallows build errors itself
                out.append(AgentRoundResult(agent.agent_id, tuple(proposals)))
                done += 1
                r = results_sink.get(agent.agent_id)
                render_dir = r.ctx.run_dir if (r and r.ctx) else ""
                log("ARTIFACT_JSON:" + json.dumps({
                    "kind": "subassembly", "sub_id": agent.agent_id,
                    "run_dir": render_dir, "render_dir": render_dir,
                    "ok": bool(r and r.ok)}))
                log(f"[boss] subassembly progress {done}/{len(sources)} "
                    f"({agent.agent_id}: {'OK' if (r and r.ok) else 'FAIL'})")
        return out

    runner = AgentTeamRunner(state, agents, collect_fn=collect_fn)
    runner.run(max_rounds=rounds, stop_when_idle=True)

    # After the last round, agents that reused an unusable prior build have cleared their
    # reuse flag and rebuilt in a later round; results_sink holds the freshest per sub.
    return {s.id: results_sink[s.id] for s in subs if s.id in results_sink}
