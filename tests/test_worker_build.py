"""Test #5: worker build — check_stl + the worker build/validate/retry loop.

Deterministic parts use a FakeClient that returns hardcoded FreeCAD bodies, so
the full worker wiring (LLM response -> extract body -> freecadcmd -> check_stl
-> retry) is exercised end-to-end WITHOUT a network, but still through a real
freecadcmd subprocess. A final tolerant LIVE step asks the real model to build
the leg. freecadcmd-dependent steps SKIP cleanly if freecadcmd isn't found.

Run:  py -3.14 tests/test_worker_build.py
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from workflow4freecad.freecad_runner import find_freecadcmd
from workflow4freecad.model import LinkSpec, WorkerTask
from workflow4freecad.validation import check_stl
from workflow4freecad.worker import _extract_python, run_worker


_GOOD_CYL = (
    "```python\n"
    'cyl = doc.addObject("Part::Cylinder", "Leg")\n'
    "cyl.Radius = 20.0\n"
    "cyl.Height = 500.0\n"
    "doc.recompute()\n"
    "__result_obj__ = cyl\n"
    "```"
)

_NO_SOLID = (
    "Sure, here is the part:\n```python\n"
    "# oops -- this builds no geometry at all\n"
    "_unused = 1 + 1\n"
    "```"
)


class _FakeClient:
    api_style = "openai"

    def __init__(self, responses):
        self._responses = list(responses)
        self.calls = 0

    def send(self, messages, system=""):
        self.calls += 1
        # Repeat the last response if we run past the scripted list.
        idx = min(self.calls - 1, len(self._responses) - 1)
        return self._responses[idx]


class _FakeSettings:
    worker_retries = 3
    worker_timeout = 120

    def __init__(self, responses):
        self.client = _FakeClient(responses)

    def worker_client(self):
        return self.client


def _bbox_close(bbox, exp, tol_xy=3.0, tol_z=2.0) -> bool:
    bx, by, bz = bbox
    ex, ey, ez = exp
    return (abs(bx - ex) <= tol_xy and abs(by - ey) <= tol_xy
            and abs(bz - ez) <= tol_z)


def main() -> int:
    ok = True
    tmp = tempfile.mkdtemp(prefix="w4fc_worker_")
    logs_dir = os.path.join(tmp, "logs")

    # 1. _extract_python pulls the body out of a fenced response.
    body = _extract_python(_GOOD_CYL)
    extract_ok = ("Part::Cylinder" in body and "```" not in body
                  and "__result_obj__" in body)
    print("1. extract python body: %s" % extract_ok)
    ok = ok and extract_ok

    # 2. check_stl rejects a missing file and a 0-byte file.
    miss = check_stl(os.path.join(tmp, "nope.stl"))
    empty_path = os.path.join(tmp, "empty.stl")
    open(empty_path, "wb").close()
    empty = check_stl(empty_path)
    cs_ok = (not miss.ok and not miss.exists
             and not empty.ok and empty.exists and empty.size_bytes == 0)
    print("2. check_stl missing=%s empty=%s -> %s"
          % (miss.summary(), empty.summary(), cs_ok))
    ok = ok and cs_ok

    # Locate freecadcmd; the remaining build steps need it.
    try:
        freecadcmd = find_freecadcmd()
    except FileNotFoundError as e:
        print("3-5. SKIP build steps -- freecadcmd not found: %s" % e)
        print("\n%s: worker build (partial, no freecadcmd)"
              % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1

    # 3. Full worker loop with a good body -> success on attempt 1, bbox right.
    leg = LinkSpec("leg", "central cylindrical leg", shape_hint="cylinder",
                   size_mm={"radius": 20, "height": 500})
    task = WorkerTask(leg, abs_mesh_path=os.path.join(tmp, "leg.stl"))
    s_good = _FakeSettings([_GOOD_CYL])
    r_good = run_worker(task, s_good, freecadcmd, logs_dir=logs_dir,
                        log_fn=lambda m: None)
    bbox = r_good.stl_report.bbox_mm if r_good.stl_report else (0, 0, 0)
    good_ok = (r_good.success and r_good.attempts == 1
               and _bbox_close(bbox, (40, 40, 500)))
    print("3. worker good body: success=%s attempts=%d bbox=%s -> %s"
          % (r_good.success, r_good.attempts,
             tuple(round(v, 1) for v in bbox), good_ok))
    ok = ok and good_ok

    # 4. Retry loop: first response builds no solid, second is the good body.
    task2 = WorkerTask(leg, abs_mesh_path=os.path.join(tmp, "leg_retry.stl"))
    s_retry = _FakeSettings([_NO_SOLID, _GOOD_CYL])
    r_retry = run_worker(task2, s_retry, freecadcmd, logs_dir=logs_dir,
                         log_fn=lambda m: None)
    retry_ok = (r_retry.success and r_retry.attempts == 2
                and s_retry.client.calls == 2)
    print("4. worker retry (bad->good): success=%s attempts=%d calls=%d -> %s"
          % (r_retry.success, r_retry.attempts, s_retry.client.calls, retry_ok))
    ok = ok and retry_ok

    # 5. LIVE: real model builds the leg (tolerant -- env-dependent).
    try:
        from config import Settings
        settings = Settings.load()
        task3 = WorkerTask(leg, abs_mesh_path=os.path.join(tmp, "leg_live.stl"))
        logs = []
        r_live = run_worker(task3, settings, freecadcmd,
                            logs_dir=os.path.join(tmp, "logs_live"),
                            log_fn=logs.append)
        rep = r_live.stl_report
        live_ok = r_live.success and rep is not None and rep.ok
        print("5. LIVE worker: success=%s attempts=%d %s -> %s"
              % (r_live.success, r_live.attempts,
                 rep.summary() if rep else "(no report)", live_ok))
        ok = ok and live_ok
    except Exception as e:
        print("5. LIVE worker SKIP (%s: %s)"
              % (type(e).__name__, str(e).replace("\n", " ")[:160]))

    print("\n%s: worker build" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
