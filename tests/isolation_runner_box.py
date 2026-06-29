"""Isolation test #2: prove the freecadcmd subprocess seam with NO LLM.

Builds a hardcoded 40x40x500 mm box via the harness and asserts the STL lands
and the harness reports ok. This is the riskiest Windows/FreeCAD seam, so it is
proven before any LLM-generated code enters the picture.

Run:  py -3.14 tests/isolation_runner_box.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from workflow4freecad.freecad_runner import find_freecadcmd, run_body


BOX_BODY = """\
box = doc.addObject("Part::Box", "Box")
box.Length = 40.0
box.Width = 40.0
box.Height = 500.0
doc.recompute()
__result_obj__ = box
"""


def main() -> int:
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "_isolation_out")
    os.makedirs(out_dir, exist_ok=True)
    stl_path = os.path.join(out_dir, "box.stl")
    result_path = os.path.join(out_dir, "box.result.json")
    script_path = os.path.join(out_dir, "box.script.py")

    try:
        freecadcmd = find_freecadcmd()
    except FileNotFoundError as e:
        print("FAIL: " + str(e))
        return 1
    print("freecadcmd:", freecadcmd)

    res = run_body(freecadcmd, BOX_BODY, stl_path=stl_path,
                   result_path=result_path, script_path=script_path,
                   timeout=120)

    print("ok=%s returncode=%s timed_out=%s result_json=%s"
          % (res.ok, res.returncode, res.timed_out, res.result_json_found))
    if res.error:
        print("error:", res.error[:500])
    if res.console_errors:
        print("console:", res.console_errors[:5])

    size = os.path.getsize(stl_path) if os.path.isfile(stl_path) else 0
    print("stl exists=%s size=%d bytes" % (os.path.isfile(stl_path), size))

    ok = res.ok and os.path.isfile(stl_path) and size > 0
    print("\n%s: freecadcmd box build" % ("PASS" if ok else "FAIL"))
    if not ok:
        print("--- stdout tail ---\n", (res.stdout or "")[-800:])
        print("--- stderr tail ---\n", (res.stderr or "")[-800:])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
