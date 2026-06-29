"""The 3-phase driver that turns a product prompt into an assembled URDF.

  Phase 1 (manager)  decompose the prompt into a KinematicModel, write model.urdf
                     up front, and scaffold one empty meshes/<link>.stl per link.
                     The URDF is the integration contract: it exists before any
                     geometry is built.
  Phase 2 (workers)  fan out one worker per link across a thread pool; each worker
                     builds in its own freecadcmd subprocess and FILLS its STL.
  Phase 3 (assemble) re-validate the URDF with meshes loaded, print a status
                     table, and render a preview of the assembled product.

Success is governed by the authoritative per-link check_stl gate (carried on
each WorkerResult), NOT by validate_urdf — yourdfpy only warns on a missing or
empty mesh. A run succeeds when every link built, or when --allow-partial is set.
"""

from __future__ import annotations

import os
import re
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field

from .freecad_runner import find_freecadcmd
from .manager import decompose
from .model import (JudgeVerdict, KinematicModel, RunContext, WorkerResult,
                    WorkerTask)
from .urdf_builder import build_urdf, scaffold_meshes, validate_urdf
from .worker import run_worker


class OrchestratorError(RuntimeError):
    """A run-fatal problem (bad URDF topology, no freecadcmd, manager failure)."""


@dataclass
class RunSummary:
    success: bool
    model: KinematicModel
    ctx: RunContext
    results: list
    urdf_ok: bool
    urdf_error: str = ""
    png_path: str = ""
    view_pngs: dict = field(default_factory=dict)
    judge: "JudgeVerdict | None" = None

    @property
    def built(self) -> int:
        return sum(1 for r in self.results if r.success)

    @property
    def total(self) -> int:
        return len(self.results)


def _slug(text: str, maxlen: int = 32) -> str:
    s = re.sub(r"[^a-z0-9]+", "_", text.strip().lower()).strip("_")
    return s[:maxlen].strip("_") or "product"


def make_run_context(product_prompt: str, out_dir: str,
                     run_dir: str | None = None) -> RunContext:
    """Build a RunContext with native absolute paths for one run.

    By default the run lands in out_dir/<slug>_<ts>/. When ``run_dir`` is given
    (the judge loop hands one per iteration) it is used verbatim instead, so the
    caller controls the folder layout and nothing is overwritten across attempts.
    """
    slug = _slug(product_prompt)
    if run_dir is None:
        ts = time.strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(out_dir, f"{slug}_{ts}")
    run_dir = os.path.abspath(run_dir)
    return RunContext(
        project_slug=slug,
        run_dir=run_dir,
        urdf_path=os.path.join(run_dir, "model.urdf"),
        meshes_dir=os.path.join(run_dir, "meshes"),
        logs_dir=os.path.join(run_dir, "logs"),
        model_json_path=os.path.join(run_dir, "kinematic_model.json"),
    )


class Orchestrator:
    """Drives one product prompt end-to-end into an assembled URDF deliverable."""

    def __init__(self, settings):
        self.settings = settings
        self._log_lock = threading.Lock()

    def run(self, product_prompt: str, *, out_dir: str = "output",
            render: bool = True, image_path: str | None = None,
            log_fn=print, run_dir: str | None = None,
            evaluator_feedback: str | None = None) -> RunSummary:
        def log(msg: str) -> None:
            with self._log_lock:
                log_fn(msg)

        ctx = make_run_context(product_prompt, out_dir, run_dir=run_dir)
        os.makedirs(ctx.logs_dir, exist_ok=True)
        log(f"[run] output dir: {ctx.run_dir}")

        # --- Phase 1: manager decomposition + the URDF contract ---------------
        log("[phase 1/3] manager: decomposing the product into parts...")
        model = decompose(product_prompt, self.settings, image_path=image_path,
                          model_json_path=ctx.model_json_path,
                          evaluator_feedback=evaluator_feedback, log_fn=log)

        build_urdf(model, ctx)
        topo_ok, topo_err = validate_urdf(ctx.urdf_path, require_meshes=False)
        if not topo_ok:
            raise OrchestratorError(f"URDF topology invalid after build: {topo_err}")
        mesh_paths = scaffold_meshes(model, ctx)
        log(f"[phase 1/3] wrote model.urdf + scaffolded {len(mesh_paths)} "
            f"empty mesh(es)")

        # --- Phase 2: workers fill the meshes (parallel) ----------------------
        try:
            freecadcmd = find_freecadcmd(self.settings.freecadcmd_path)
        except FileNotFoundError as e:
            raise OrchestratorError(str(e))

        tasks = [WorkerTask(link, mesh_paths[link.name]) for link in model.links]
        n_workers = max(1, min(len(tasks), self.settings.max_workers))
        log(f"[phase 2/3] building {len(tasks)} part(s) with up to "
            f"{n_workers} worker(s) in flight...")
        results = self._run_workers(tasks, freecadcmd, ctx, log, n_workers)

        # --- Phase 3: assemble, validate, report, visualize -------------------
        log("[phase 3/3] re-validating the assembled URDF (meshes loaded)...")
        urdf_ok, urdf_err = validate_urdf(ctx.urdf_path, require_meshes=True)

        all_built = all(r.success for r in results)
        success = all_built or self.settings.allow_partial
        summary = RunSummary(success=success, model=model, ctx=ctx,
                             results=results, urdf_ok=urdf_ok, urdf_error=urdf_err)

        log("\n" + self._status_table(model, results, urdf_ok))

        if render and self.settings.do_viz:
            summary.png_path = self._render(ctx, log)

        verdict = "SUCCESS" if summary.success else "INCOMPLETE"
        log(f"\n[done] {verdict}: {summary.built}/{summary.total} parts built. "
            f"Deliverable: {ctx.urdf_path}")
        return summary

    def run_with_judge(self, product_prompt: str, *, out_dir: str = "output",
                       image_path: str | None = None,
                       max_iterations: int | None = None,
                       log_fn=print) -> RunSummary:
        """Generate -> judge -> refine: build, evaluate, and on failure retry.

        Each iteration gets its own folder under one session root so nothing is
        overwritten. After a build, six orthographic views are rendered (best-
        effort, regardless of --no-viz, because the verdict needs them) and the
        judger emits judge.json. On PASS the loop stops; on FAIL the evaluator's
        suggestions become the manager's feedback for the next iteration, up to
        ``max_iterations`` total attempts. Returns the final iteration's summary
        with ``view_pngs`` and ``judge`` attached.
        """
        from .judger import JudgeError, judge
        from .viz import render_six_views

        def log(msg: str) -> None:
            with self._log_lock:
                log_fn(msg)

        n = max_iterations or self.settings.judge_max_iterations
        slug = _slug(product_prompt)
        ts = time.strftime("%Y%m%d_%H%M%S")
        session_root = os.path.abspath(os.path.join(out_dir, f"{slug}_{ts}"))
        log(f"[judge-loop] session root: {session_root} "
            f"(up to {n} iteration(s))")

        feedback = None
        summary = None
        for i in range(1, n + 1):
            iter_dir = os.path.join(session_root, f"iter_{i:02d}")
            log(f"\n========== iteration {i}/{n} ==========")
            summary = self.run(product_prompt, out_dir=out_dir, render=False,
                               image_path=image_path, log_fn=log_fn,
                               run_dir=iter_dir, evaluator_feedback=feedback)

            # Render the six views for the judge -- always attempted (the verdict
            # needs them), best-effort so a headless failure degrades to text.
            view_pngs: dict = {}
            try:
                view_pngs = render_six_views(summary.ctx.urdf_path,
                                             os.path.join(iter_dir, "views"))
                log(f"[judge-loop] rendered {len(view_pngs)} view(s) for the judge")
            except Exception as e:
                log(f"[judge-loop] view rendering unavailable "
                    f"({type(e).__name__}: {str(e)[:120]}); judging text-only")
            summary.view_pngs = view_pngs

            # Evaluate; a judge that cannot produce a verdict stops the loop
            # (there is no feedback to refine with) but keeps the built CAD.
            try:
                verdict = judge(product_prompt, summary.model, summary.results,
                                view_pngs, self.settings,
                                reference_image_path=image_path,
                                out_json_path=os.path.join(iter_dir, "judge.json"),
                                log_fn=log)
            except JudgeError as e:
                log(f"[judge-loop] evaluation failed ({e}); stopping the loop")
                break
            summary.judge = verdict

            if verdict.passed:
                log(f"[judge-loop] PASS on iteration {i}; stopping.")
                break
            if i < n:
                feedback = self._compose_feedback(verdict)
                log(f"[judge-loop] FAIL on iteration {i}; regenerating with the "
                    f"evaluator's feedback.")
            else:
                log(f"[judge-loop] FAIL on iteration {i}; iteration budget "
                    f"exhausted.")
        return summary

    @staticmethod
    def _compose_feedback(verdict) -> str:
        """Combine the verdict's suggestions + reasons into the manager's brief."""
        parts: list[str] = []
        if verdict.suggestions:
            parts.append(verdict.suggestions)
        if verdict.reasons:
            parts.append(f"(Evaluator's reasoning: {verdict.reasons})")
        return "\n\n".join(parts) if parts else \
            "The model did not pass evaluation; improve its fidelity to the request."

    # --- internals -----------------------------------------------------------

    def _run_workers(self, tasks, freecadcmd, ctx, log, n_workers) -> list:
        """Build all parts through a continuous pool of up to n_workers.

        One ThreadPoolExecutor over ALL tasks keeps n_workers builds in flight
        at all times: as soon as one finishes, the next queued part starts, so
        the pool never idles waiting on the slowest sibling of a batch. Threads
        are fine: each worker blocks on its own freecadcmd subprocess, so the
        GIL is moot. The gateway tolerates this sustained load because each
        worker's LLM send already retries with backoff on a dropped/empty
        response, so a momentary overload self-heals instead of failing a part.
        """
        results: dict[str, WorkerResult] = {}

        def work(task: WorkerTask) -> WorkerResult:
            return run_worker(task, self.settings, freecadcmd,
                              logs_dir=ctx.logs_dir, log_fn=log)

        total = len(tasks)
        done = 0
        with ThreadPoolExecutor(max_workers=n_workers) as pool:
            futures = {pool.submit(work, t): t for t in tasks}
            for fut in as_completed(futures):
                task = futures[fut]
                try:
                    results[task.link.name] = fut.result()
                except Exception as e:  # a worker should never raise, but be safe
                    results[task.link.name] = WorkerResult(
                        link_name=task.link.name, success=False,
                        abs_mesh_path=task.abs_mesh_path,
                        error=f"worker raised: {type(e).__name__}: {e}")
                done += 1
                log(f"[phase 2/3] progress {done}/{total} "
                    f"(finished: {task.link.name})")
        return [results[t.link.name] for t in tasks]

    @staticmethod
    def _status_table(model, results, urdf_ok) -> str:
        """ASCII status table (ASCII-only — the Windows console is cp1252)."""
        by_name = {r.link_name: r for r in results}
        header = (f"{'LINK':<18} {'STATUS':<6} {'ATT':<3} {'FACES':<7} "
                  f"{'BBOX mm (x,y,z)':<22} NOTE")
        sep = "-" * len(header)
        lines = [header, sep]
        for link in model.links:
            r = by_name.get(link.name)
            rep = r.stl_report if r else None
            if rep and rep.bbox_mm != (0.0, 0.0, 0.0):
                bx, by, bz = rep.bbox_mm
                bbox = f"{bx:.1f}x{by:.1f}x{bz:.1f}"
                faces = str(rep.num_faces)
            else:
                bbox, faces = "", ""
            status = "OK" if (r and r.success) else "FAIL"
            att = str(r.attempts) if r else "-"
            if r and r.success:
                note = "watertight" if (rep and rep.watertight) else "ok"
            else:
                detail = (r.error if (r and r.error) else
                          (rep.summary() if rep else "no result"))
                note = detail.splitlines()[0][:30] if detail else "failed"
            lines.append(f"{link.name:<18.18} {status:<6} {att:<3} {faces:<7} "
                         f"{bbox:<22} {note}")
        lines.append(sep)
        built = sum(1 for r in results if r.success)
        lines.append(f"{built}/{len(results)} parts built   "
                     f"URDF assemble (advisory): {'OK' if urdf_ok else 'WARN'}")
        return "\n".join(lines)

    @staticmethod
    def _render(ctx, log) -> str:
        """Best-effort PNG preview of the assembled product (never run-fatal)."""
        try:
            from .viz import render_png
            png = render_png(ctx.urdf_path,
                             os.path.join(ctx.run_dir, "preview.png"))
            log(f"[viz] preview rendered: {png}")
            return png
        except Exception as e:
            log(f"[viz] preview skipped ({type(e).__name__}: {str(e)[:120]})")
            return ""
