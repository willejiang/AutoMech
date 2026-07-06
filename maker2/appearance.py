#!/usr/bin/env python3
"""Coarse appearance base (Session B item 1c): a low-poly proxy of the WHOLE machine,
built from the boss's plan BEFORE the managers build detail.

Each subassembly is approximated by one bounding primitive (a box) placed at its
GLOBAL pose — derived from the spread of that sub's interface frames plus its link
budget — and all the proxies are emitted as ONE coarse CadQuery script + a single
proxy STL. That STL is the machine's "base look" (the UI can show it immediately),
and a compact text summary of every sub's proxy size/pose is handed to each manager
so their parts keep consistent proportions with the rest of the machine.

Cheap by construction (one box per sub, no per-part detail) and OFF unless
settings.enable_appearance_proxy is set, so it never slows small runs. Marker lines
are '[boss] appearance'. Uses the same CadQuery kernel + subprocess-free in-process
export as a trivial proxy (boxes only, so no arbitrary generated code here).
"""
from __future__ import annotations

import json
import os
from pathlib import Path


def _sub_proxy_box(spec) -> dict:
    """A crude world-space box for one subassembly: centroid = mean of its frame
    global positions (fallback origin), half-extent from the frame spread with a floor
    scaled by the link budget (a bigger sub gets a bigger default box). Returns
    {id, center_m:[3], size_m:[3]}."""
    frames = list(getattr(spec, "frames", []) or [])
    pts = [tuple(f.xyz_m) for f in frames]
    if pts:
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        cz = sum(p[2] for p in pts) / len(pts)
        span = [max(1e-3, (max(p[i] for p in pts) - min(p[i] for p in pts)))
                for i in range(3)]
    else:
        cx = cy = cz = 0.0
        span = [0.05, 0.05, 0.05]
    # A floor size so a sub with coincident frames still has a visible box; grows a
    # little with the link budget (more parts -> physically bigger).
    budget = max(1, int(getattr(spec, "est_link_budget", 6)))
    floor = 0.02 + 0.004 * min(budget, 30)
    size = [max(span[i], floor) for i in range(3)]
    return {"id": spec.id, "center_m": [cx, cy, cz], "size_m": size}


def build_appearance_proxy(plan, out_dir: str, *, log_fn=print) -> dict:
    """Emit a coarse whole-machine proxy for `plan`. Writes appearance_proxy.py (a
    CadQuery script), appearance_proxy.stl (one union of per-sub boxes), and
    appearance_proxy.json (the per-sub proxy boxes). Returns {boxes, stl, script,
    summary_text} (summary_text is the manager-facing proportion context). Best-effort:
    on any failure it still returns the boxes + summary so managers get context even if
    the STL can't be rendered."""
    def log(m):
        log_fn(f"[boss] appearance: {m}")

    boxes = [_sub_proxy_box(s) for s in plan.subassemblies]
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    with open(os.path.join(out_dir, "appearance_proxy.json"), "w", encoding="utf-8") as f:
        json.dump({"name": plan.name, "boxes": boxes}, f, indent=2)

    # A single coarse CadQuery script: one box per sub at its world center (mm).
    lines = ["import cadquery as cq", "", "def build_appearance():", "    parts = []"]
    for b in boxes:
        cx, cy, cz = (v * 1000.0 for v in b["center_m"])
        sx, sy, sz = (v * 1000.0 for v in b["size_m"])
        lines.append(f"    parts.append(cq.Workplane('XY').box({sx:.2f}, {sy:.2f}, "
                     f"{sz:.2f}).translate(({cx:.2f}, {cy:.2f}, {cz:.2f})))  # {b['id']}")
    lines += ["    solid = parts[0]",
              "    for p in parts[1:]:",
              "        solid = solid.union(p)",
              "    return solid", ""]
    script = "\n".join(lines)
    script_path = os.path.join(out_dir, "appearance_proxy.py")
    Path(script_path).write_text(script, encoding="utf-8")

    stl_path = os.path.join(out_dir, "appearance_proxy.stl")
    try:
        import cadquery as cq  # noqa: F401
        ns: dict = {}
        exec(compile(script, script_path, "exec"), ns)
        solid = ns["build_appearance"]()
        (solid.val() if hasattr(solid, "val") else solid).exportStl(stl_path)
        log(f"rendered {len(boxes)}-box proxy -> {stl_path}")
        print("ARTIFACT_JSON:" + json.dumps({
            "kind": "appearance_proxy", "run_dir": out_dir,
            "stl": "appearance_proxy.stl", "boxes": boxes}), flush=True)
    except Exception as e:
        log(f"proxy STL render skipped ({e}); handing managers the numeric summary only")
        stl_path = ""

    return {"boxes": boxes, "stl": stl_path, "script": script_path,
            "summary_text": summarize_proxy(plan.name, boxes)}


def summarize_proxy(machine_name: str, boxes: list) -> str:
    """A compact, manager-facing description of the whole machine's coarse layout, so
    each manager keeps its subassembly's proportions consistent with the others."""
    rows = []
    for b in boxes:
        cx, cy, cz = (v * 1000.0 for v in b["center_m"])
        sx, sy, sz = (v * 1000.0 for v in b["size_m"])
        rows.append(f"  - {b['id']}: ~{sx:.0f}x{sy:.0f}x{sz:.0f} mm box "
                    f"centered at [{cx:.0f}, {cy:.0f}, {cz:.0f}] mm")
    body = "\n".join(rows) if rows else "  (no subassemblies)"
    return (f"COARSE MACHINE LAYOUT ({machine_name}) — approximate size + global "
            f"position of every subassembly, so your parts stay in proportion with "
            f"the rest of the machine:\n{body}")
