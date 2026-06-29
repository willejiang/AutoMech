"""Test #4: manager decomposition — offline validation + mocked repair + live.

The offline checks (no network) are the hard gate: JSON extraction, parsing the
few-shot, tree validation (good + every failure mode), name normalization, and
save/load round-trip. A mocked client exercises decompose()'s repair loop without
a network. The live decompose() against the local gateway is tolerant — an env
without a real key/gateway prints LIVE SKIP rather than failing the suite.

Run:  py -3.14 tests/test_manager_decompose.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from workflow4freecad.manager import (ManagerError, _extract_json_object,
                                      _validate_model, decompose, load_model,
                                      model_to_dict, parse_model, save_model)
from workflow4freecad.model import JointSpec, KinematicModel, LinkSpec
from workflow4freecad.prompts.schema import FEWSHOT_JSON


def _expect_error(fn, needle: str) -> tuple[bool, str]:
    """Run fn; pass iff it raises with `needle` in the message."""
    try:
        fn()
    except (ManagerError, ValueError) as e:
        msg = str(e)
        return (needle.lower() in msg.lower()), msg.replace("\n", " ")[:140]
    return False, "no exception raised"


def _two_link(joint: JointSpec, *, l1="base", l2="arm") -> KinematicModel:
    return KinematicModel(
        name="m", root_link="",
        links=[LinkSpec(l1, "a"), LinkSpec(l2, "b")],
        joints=[joint],
    )


def main() -> int:
    ok = True

    # 1. JSON extraction tolerates ```json fences + trailing prose.
    fenced = "Here you go:\n```json\n" + FEWSHOT_JSON + "\n```\nDone."
    extracted = _extract_json_object(fenced)
    extract_ok = extracted.startswith("{") and extracted.rstrip().endswith("}")
    print("1. extract from fences: %s" % extract_ok)
    ok = ok and extract_ok

    # 2. The few-shot parses and validates cleanly.
    model = parse_model(FEWSHOT_JSON)
    _validate_model(model)
    fewshot_ok = (len(model.links) == 2 and len(model.joints) == 1
                  and model.root_link == "tabletop")
    print("2. few-shot: links=%d joints=%d root=%s -> %s"
          % (len(model.links), len(model.joints), model.root_link, fewshot_ok))
    ok = ok and fewshot_ok

    # 3. Validation assigns mesh_filename to every link.
    mesh_ok = all(l.mesh_filename == f"meshes/{l.name}.stl" for l in model.links)
    print("3. mesh_filename assigned: %s" % mesh_ok)
    ok = ok and mesh_ok

    # 4. Unknown joint endpoint is rejected.
    bad_ep, msg = _expect_error(
        lambda: _validate_model(_two_link(
            JointSpec("j", "fixed", "base", "ghost"))), "unknown child")
    print("4. unknown endpoint rejected: %s | %s" % (bad_ep, msg))
    ok = ok and bad_ep

    # 5. A forest (two roots / disconnected) is rejected.
    two_roots, msg = _expect_error(
        lambda: _validate_model(KinematicModel(
            name="m", root_link="",
            links=[LinkSpec("a", ""), LinkSpec("b", ""), LinkSpec("c", "")],
            joints=[JointSpec("j", "fixed", "a", "b")])),  # c is a 2nd root
        "multiple root")
    print("5. forest rejected: %s | %s" % (two_roots, msg))
    ok = ok and two_roots

    # 6. A cycle (no root) is rejected.
    cycle, msg = _expect_error(
        lambda: _validate_model(KinematicModel(
            name="m", root_link="",
            links=[LinkSpec("a", ""), LinkSpec("b", "")],
            joints=[JointSpec("j1", "fixed", "a", "b"),
                    JointSpec("j2", "fixed", "b", "a")])),
        "no root")
    print("6. cycle rejected: %s | %s" % (cycle, msg))
    ok = ok and cycle

    # 7. revolute without limits is rejected; with lower<upper it passes.
    rev_bad, msg = _expect_error(
        lambda: _validate_model(_two_link(
            JointSpec("j", "revolute", "base", "arm", axis=(0, 0, 1)))),
        "lower")
    rev_model = _two_link(
        JointSpec("j", "revolute", "base", "arm", axis=(0, 0, 1),
                  lower=-1.0, upper=1.0))
    try:
        _validate_model(rev_model)
        rev_good = True
    except (ManagerError, ValueError):
        rev_good = False
    print("7. revolute limits enforced: bad=%s good=%s" % (rev_bad, rev_good))
    ok = ok and rev_bad and rev_good

    # 8. Names are slugified/deduped and propagated into joint endpoints.
    msgy = KinematicModel(
        name="m", root_link="",
        links=[LinkSpec("Base Link", ""), LinkSpec("Base Link", "")],  # dup+spaces
        joints=[JointSpec("J 1", "fixed", "Base Link", "Base Link")])
    # After slugify+dedupe the two links become base_link / base_link_2; the
    # joint's parent/child both came from "Base Link" -> both map to base_link,
    # which is a self-loop -> rejected. We assert the names normalized.
    self_loop, _ = _expect_error(lambda: _validate_model(msgy), "itself")
    names_ok = [l.name for l in msgy.links] == ["base_link", "base_link_2"]
    print("8. slugify+dedupe: names=%s self_loop_caught=%s"
          % ([l.name for l in msgy.links], self_loop))
    ok = ok and names_ok and self_loop

    # 9. save/load round-trip preserves the validated model.
    import tempfile
    tmp = os.path.join(tempfile.gettempdir(), "w4fc_model_roundtrip.json")
    good = parse_model(FEWSHOT_JSON)
    _validate_model(good)
    save_model(good, tmp)
    back = load_model(tmp)
    _validate_model(back)
    roundtrip_ok = (model_to_dict(good)["links"] == model_to_dict(back)["links"]
                    and back.root_link == "tabletop")
    print("9. save/load round-trip: %s" % roundtrip_ok)
    ok = ok and roundtrip_ok

    # 10. decompose() repair loop with a mocked client (no network):
    #     attempt 1 returns invalid JSON, attempt 2 returns the good model.
    class _FakeClient:
        api_style = "openai"

        def __init__(self):
            self.calls = 0

        def send(self, messages, system=""):
            self.calls += 1
            if self.calls == 1:
                return "{ not valid json at all"
            return FEWSHOT_JSON

    class _FakeSettings:
        manager_retries = 2

        def __init__(self):
            self.client = _FakeClient()

        def manager_client(self):
            return self.client

    fs = _FakeSettings()
    logs = []
    repaired = decompose("a simple table", fs, log_fn=logs.append)
    repair_ok = (fs.client.calls == 2 and repaired.root_link == "tabletop")
    print("10. repair loop: calls=%d root=%s -> %s"
          % (fs.client.calls, repaired.root_link, repair_ok))
    ok = ok and repair_ok

    # 11. LIVE: real gateway decomposition (tolerant — env-dependent).
    try:
        from config import Settings
        settings = Settings.load()
        live_logs = []
        live = decompose(
            "a simple table: a flat square top on a single central leg",
            settings, log_fn=live_logs.append)
        live_ok = (len(live.links) >= 2 and live.root_link
                   and all(l.mesh_filename for l in live.links))
        print("11. LIVE decompose: name=%s links=%d joints=%d root=%s -> %s"
              % (live.name, len(live.links), len(live.joints),
                 live.root_link, live_ok))
        ok = ok and live_ok
    except Exception as e:
        print("11. LIVE decompose SKIP (%s: %s)"
              % (type(e).__name__, str(e).replace("\n", " ")[:160]))

    print("\n%s: manager decomposition" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
