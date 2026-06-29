"""Worker agent: one LinkSpec -> a filled, validated STL.

A worker sees only its own link (never its siblings). It asks the LLM for a
FreeCAD Python *body* that builds the part in millimeters at the local origin,
runs that body in an isolated freecadcmd subprocess (so an OpenCASCADE segfault
is contained, not fatal to the orchestrator), and validates the exported STL
with the authoritative check_stl gate. On any failure it feeds the captured
error + STL report back to the model and retries, bounded by
Settings.worker_retries.

The worker NEVER positions its part relative to other parts — all spatial
relationships live in the manager's joints. This module only fills one mesh.
"""

from __future__ import annotations

import os
import re
import time

from .freecad_runner import RunResult, run_body
from .llm.client import LLMError, LLMTruncationError
from .llm.conversation import Conversation
from .model import WorkerResult, WorkerTask
from .prompts.worker_prompt import (build_worker_retry, build_worker_shrink,
                                    build_worker_system, build_worker_user)
from .validation import check_stl


# Grab the first fenced code block (any language tag) from an LLM response.
_FENCE = re.compile(r"```[ \t]*[a-zA-Z]*[ \t]*\r?\n(.*?)```", re.DOTALL)


def _extract_python(text: str) -> str:
    """Return the FreeCAD body: the first fenced block, else the whole text.

    The worker prompt asks for exactly one ```python block, but models sometimes
    drop the fence or add prose; falling back to the stripped text keeps a
    well-formed bare-code response usable.
    """
    m = _FENCE.search(text)
    if m:
        return m.group(1).strip("\n")
    return text.strip()


def _format_run_error(run: RunResult) -> str:
    """Condense a failed RunResult into actionable feedback for the model."""
    parts: list[str] = []
    if run.timed_out:
        parts.append("TIMED OUT -- likely an infinite loop or a FreeCAD hang/crash.")
    if run.returncode and run.returncode != 0:
        sign = " (negative => segfault/crash in OpenCASCADE)" if run.returncode < 0 else ""
        parts.append(f"freecadcmd exit code {run.returncode}{sign}.")
    if run.error:
        parts.append(run.error.strip())
    for ce in run.console_errors[:6]:
        parts.append(f"FreeCAD console error: {ce}")
    if not parts and run.stderr:
        tail = run.stderr.strip().splitlines()[-6:]
        if tail:
            parts.append("stderr tail:\n" + "\n".join(tail))
    return "\n".join(parts) if parts else "no specific error captured"


def run_worker(task: WorkerTask, settings, freecadcmd: str, *,
               logs_dir: str, log_fn=None) -> WorkerResult:
    """Build, validate, and (on failure) retry the STL for one link.

    Returns a WorkerResult with ``success`` reflecting the authoritative
    check_stl gate on the final attempt. Never raises for a build failure — a
    failed worker is data the orchestrator reports, not an exception.
    """
    link = task.link
    client = settings.worker_client()
    system = build_worker_system(task.units_convention)
    conv = Conversation()
    conv.add_user_message(build_worker_user(link))

    os.makedirs(logs_dir, exist_ok=True)
    result_path = os.path.join(logs_dir, f"{link.name}.result.json")
    script_path = os.path.join(logs_dir, f"{link.name}.script.py")

    attempts = settings.worker_retries + 1
    last_code = ""
    last_error = ""
    last_report = None

    for attempt in range(1, attempts + 1):
        messages = conv.get_messages_for_api(api_style=client.api_style)
        try:
            text = client.send(messages, system=system)
        except LLMTruncationError as e:
            # The reply exceeded the output cap and the gateway returned nothing
            # usable. Re-sending the identical prompt would just truncate again
            # (see client.LLMTruncationError), so append a nudge to SHRINK the
            # next reply -- minimal code, primitives, no prose -- then retry
            # within the attempt budget. Mirrors the manager's coarser retry.
            last_error = f"LLM truncated/empty: {str(e).splitlines()[0]}"
            if log_fn:
                log_fn(f"[worker:{link.name}] attempt {attempt}/{attempts} "
                       f"LLM reply over output cap; asking for a smaller reply "
                       f"and retrying")
            conv.add_user_message(build_worker_shrink())
            time.sleep(min(2 * attempt, 10))
            continue
        except LLMError as e:
            # A non-truncation send failure (connection/HTTP, or empty with no
            # generated output) is usually transient under concurrent load:
            # re-sending the identical prompt after a short backoff usually
            # succeeds, so retry within the attempt budget instead of letting
            # the exception escape and abort the worker.
            last_error = f"LLM request failed: {str(e).splitlines()[0]}"
            if log_fn:
                log_fn(f"[worker:{link.name}] attempt {attempt}/{attempts} "
                       f"LLM request failed; backing off and retrying")
            time.sleep(min(2 * attempt, 10))
            continue
        conv.add_assistant_message(text)
        body = _extract_python(text)
        last_code = body

        run = run_body(freecadcmd, body, stl_path=task.abs_mesh_path,
                       result_path=result_path, script_path=script_path,
                       timeout=settings.worker_timeout)
        report = check_stl(task.abs_mesh_path)
        last_report = report

        if run.ok and report.ok:
            if log_fn:
                log_fn(f"[worker:{link.name}] OK on attempt {attempt}/{attempts}: "
                       f"{report.summary()}")
            return WorkerResult(
                link_name=link.name, success=True, attempts=attempt,
                abs_mesh_path=task.abs_mesh_path, stl_report=report, code=body)

        last_error = _format_run_error(run)
        if log_fn:
            first = last_error.splitlines()[0] if last_error else ""
            log_fn(f"[worker:{link.name}] attempt {attempt}/{attempts} failed: "
                   f"{first} | {report.summary()}")
        conv.add_user_message(build_worker_retry(body, last_error, report.summary()))

    return WorkerResult(
        link_name=link.name, success=False, attempts=attempts,
        abs_mesh_path=task.abs_mesh_path, error=last_error,
        stl_report=last_report, code=last_code)
