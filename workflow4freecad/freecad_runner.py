"""Run FreeCAD Python in a headless ``freecadcmd`` subprocess (Windows).

A worker hands us a *body* — a snippet of FreeCAD Python that builds one part
and assigns the final solid to ``__result_obj__``. We wrap it in a harness that:

  * imports FreeCAD + common workbench modules,
  * attaches a Console observer (FreeCAD geometry ops report failures to the
    Console instead of raising),
  * runs the body, tessellates the resulting solid, and exports an STL,
  * verifies the STL actually landed (``Mesh.export`` does NOT raise on a bad
    path — it logs a Console error and writes nothing), and
  * writes a small JSON result, then hard-exits with ``os._exit(0)`` to dodge a
    headless Qt event-loop hang.

Output/result paths are passed through the *environment* (``W4FC_STL_PATH`` /
``W4FC_RESULT_PATH``) rather than baked into the source, so native Windows paths
with backslashes never need escaping inside generated code.

This is the Windows-native reimplementation of freecad-ai's Linux
``executor._sandbox_test`` pattern — no ``signal.SIGALRM`` (not on Windows);
``subprocess.run(timeout=)`` is the backstop and a segfault surfaces as
``returncode < 0``.
"""

from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
import textwrap
from dataclasses import dataclass, field


# The body snippet is spliced in where this sentinel sits (at the try-block's
# 4-space indent). We use str.replace, not str.format, so the harness can
# contain literal { } (JSON dict) without clashing with the body's own braces.
_BODY_SENTINEL = "    # __W4FC_BODY__"

_HARNESS_TEMPLATE = '''\
import os, sys, json, traceback

_STL_PATH = os.environ["W4FC_STL_PATH"]
_RESULT_PATH = os.environ["W4FC_RESULT_PATH"]

_OK = True
_ERRTXT = ""
_CONSOLE_ERRORS = []

import FreeCAD as App
import Part
import Mesh
import math

# Optional workbench modules: present in a full install, imported defensively
# so a missing one never aborts the build.
try:
    import Draft
except Exception:
    Draft = None
try:
    import Sketcher
except Exception:
    Sketcher = None
try:
    import PartDesign
except Exception:
    PartDesign = None


class _ConsoleObserver:
    """Capture FreeCAD Console errors — failed booleans/lofts log here."""
    def OnMessage(self, msg):
        pass
    def OnWarning(self, msg):
        pass
    def OnError(self, msg):
        try:
            _CONSOLE_ERRORS.append(str(msg).strip())
        except Exception:
            pass


_obs = _ConsoleObserver()
try:
    App.Console.AttachObserver(_obs)
except Exception:
    pass

doc = App.newDocument("part")
__result_obj__ = None

try:
    # ===================== begin generated body =====================
    # __W4FC_BODY__
    # ====================== end generated body ======================
    doc.recompute()

    if __result_obj__ is not None:
        _objs = [__result_obj__]
    else:
        _objs = [o for o in doc.Objects
                 if getattr(o, "Shape", None) is not None
                 and not o.Shape.isNull()
                 and len(o.Shape.Solids) > 0]

    if not _objs:
        _OK = False
        _ERRTXT = ("no solid object produced; assign the final solid to "
                   "__result_obj__")
    else:
        Mesh.export(_objs, _STL_PATH)
        if not (os.path.isfile(_STL_PATH) and os.path.getsize(_STL_PATH) > 0):
            _OK = False
            _ERRTXT = "Mesh.export wrote no file (bad geometry or path)"
except Exception:
    _OK = False
    _ERRTXT = traceback.format_exc()
finally:
    if _CONSOLE_ERRORS and not _OK:
        _ERRTXT = (_ERRTXT + "\\n[FreeCAD Console]\\n"
                   + "\\n".join(_CONSOLE_ERRORS)).strip()
    try:
        with open(_RESULT_PATH, "w") as _f:
            json.dump({"ok": _OK, "error": _ERRTXT,
                       "console_errors": _CONSOLE_ERRORS}, _f)
    except Exception:
        pass
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass
    # Headless freecadcmd can hang on a lingering Qt event loop; hard-exit.
    os._exit(0)
'''


@dataclass
class RunResult:
    """Outcome of one freecadcmd subprocess invocation."""

    ok: bool
    error: str = ""
    console_errors: list = field(default_factory=list)
    returncode: int = 0
    timed_out: bool = False
    stdout: str = ""
    stderr: str = ""
    result_json_found: bool = False
    script_path: str = ""


def find_freecadcmd(explicit_path: str | None = None) -> str:
    """Locate ``freecadcmd.exe``, raising a clear error listing where we looked.

    Search order: ``$FREECADCMD`` env > explicit (settings) path > the verified
    default install > version glob > ``PATH``.
    """
    candidates: list[str] = []
    env = os.environ.get("FREECADCMD")
    if env:
        candidates.append(env)
    if explicit_path:
        candidates.append(explicit_path)
    candidates.append(r"C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe")
    candidates.extend(sorted(
        glob.glob(r"C:\Program Files\FreeCAD *\bin\freecadcmd.exe"),
        reverse=True,
    ))
    which = shutil.which("freecadcmd") or shutil.which("freecadcmd.exe")
    if which:
        candidates.append(which)

    seen = set()
    for c in candidates:
        if not c or c in seen:
            continue
        seen.add(c)
        if os.path.isfile(c):
            return c

    looked = "\n  ".join(dict.fromkeys(candidates)) or "(no candidates)"
    raise FileNotFoundError(
        "Could not find freecadcmd.exe. Looked in:\n  " + looked
        + "\nSet the FREECADCMD environment variable or "
        "Settings.freecadcmd_path to the correct location."
    )


def build_harness(body: str) -> str:
    """Splice a FreeCAD body snippet into the runnable harness script."""
    indented = textwrap.indent(body.rstrip("\n"), "    ")
    return _HARNESS_TEMPLATE.replace(_BODY_SENTINEL, indented, 1)


def run_body(freecadcmd: str, body: str, *, stl_path: str, result_path: str,
             script_path: str, timeout: int = 120) -> RunResult:
    """Wrap ``body`` in the harness and execute it in headless freecadcmd.

    All three paths must be native Windows absolute paths. Stale STL/result
    files are removed first so a leftover from a prior attempt can't be
    mistaken for success.
    """
    script_text = build_harness(body)
    os.makedirs(os.path.dirname(script_path), exist_ok=True)
    os.makedirs(os.path.dirname(stl_path), exist_ok=True)
    with open(script_path, "w", encoding="utf-8") as f:
        f.write(script_text)

    for stale in (stl_path, result_path):
        try:
            if os.path.isfile(stale):
                os.remove(stale)
        except OSError:
            pass

    env = {
        **os.environ,
        "QT_QPA_PLATFORM": "offscreen",
        "W4FC_STL_PATH": stl_path,
        "W4FC_RESULT_PATH": result_path,
    }

    try:
        proc = subprocess.run(
            [freecadcmd, script_path],
            capture_output=True, text=True, timeout=timeout, env=env,
        )
    except subprocess.TimeoutExpired as e:
        return RunResult(
            ok=False, timed_out=True,
            error=f"freecadcmd timed out after {timeout}s",
            stdout=(e.stdout or "") if isinstance(e.stdout, str) else "",
            stderr=(e.stderr or "") if isinstance(e.stderr, str) else "",
            script_path=script_path,
        )

    result = RunResult(
        ok=False, returncode=proc.returncode,
        stdout=proc.stdout or "", stderr=proc.stderr or "",
        script_path=script_path,
    )

    if os.path.isfile(result_path):
        result.result_json_found = True
        try:
            with open(result_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            result.ok = bool(data.get("ok"))
            result.error = data.get("error", "")
            result.console_errors = data.get("console_errors", []) or []
        except (OSError, json.JSONDecodeError) as e:
            result.error = f"result JSON unreadable: {e}"
    else:
        # No result file → harness never reached its finally (segfault, import
        # crash). returncode < 0 means killed by signal on POSIX; on Windows a
        # crash shows as a large/!=0 code.
        if proc.returncode != 0:
            result.error = (f"freecadcmd crashed (returncode={proc.returncode}); "
                            f"no result written. stderr tail: "
                            f"{(proc.stderr or '')[-500:]}")
        else:
            result.error = ("freecadcmd produced no result file and no error "
                            f"code. stderr tail: {(proc.stderr or '')[-500:]}")

    return result
