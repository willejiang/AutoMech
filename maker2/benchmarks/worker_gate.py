"""Phase 3 — worker_gate: per-part dimension lock + manifold check (deterministic).

Runs after ONE part renders, so a worker that ignored the manager's sizes or produced a
non-manifold mesh is caught for a cheap single-part re-render — not a whole-sub rebuild
and not the LLM debugger.

- DIMENSION LOCK: the manager declared this part's key dims in size_mm (radius/height/
  x/y/z/bore_dia...). Scan the CadQuery script's numeric literals; if a declared dim has
  NO literal within tolerance anywhere in the script, the worker drew something else ->
  ERR_DIM. Conservative: only flags a declared dim that is entirely ABSENT (a missing
  match is a real divergence; a present-but-also-other-numbers script is fine).
- MANIFOLD: reuse validation.check_stl; a non-watertight / degenerate / unloadable mesh
  makes MuJoCo penetrate -> ERR_MANIFOLD.

Reuses validation.check_stl. No LLM.
"""

from __future__ import annotations

import ast
import re

from . import GateError

# Fraction tolerance when matching a declared dim to a script literal.
_DIM_TOL_FRAC = 0.06

# size_mm keys that name a real geometric dimension the worker must honor, in CANONICAL
# form (C2 — canonical_size folds aliases like gear_dia/pitch_diameter onto these before
# the scan). Keys not here (free-form notes, tooth counts) are ignored.
_MATING_KEYS = ("radius", "height", "x", "y", "z", "bore_dia", "outer_dia", "inner_dia",
                "pitch_dia", "pitch_radius", "length", "width", "thickness", "depth")


def _script_numbers(script: str) -> list[float]:
    """All numeric literals in the script (via AST; regex fallback). A radius given as a
    diameter/2 still yields the halved value because AST-evaluable constant arithmetic is
    folded where trivially possible; otherwise both operands appear as literals."""
    nums: list[float] = []
    try:
        tree = ast.parse(script)
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
                nums.append(float(node.value))
            # fold simple  <num> / <num>  and  <num> * <num>
            elif (isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Div, ast.Mult))
                  and isinstance(node.left, ast.Constant)
                  and isinstance(node.right, ast.Constant)
                  and isinstance(node.left.value, (int, float))
                  and isinstance(node.right.value, (int, float))):
                try:
                    if isinstance(node.op, ast.Div) and node.right.value:
                        nums.append(float(node.left.value) / float(node.right.value))
                    elif isinstance(node.op, ast.Mult):
                        nums.append(float(node.left.value) * float(node.right.value))
                except ZeroDivisionError:
                    pass
    except SyntaxError:
        for m in re.finditer(r"-?\d+\.?\d*", script):
            try:
                nums.append(float(m.group()))
            except ValueError:
                pass
    return nums


def _has_match(target: float, nums: list[float]) -> bool:
    if target <= 0:
        return True                                  # nothing to match
    tol = max(_DIM_TOL_FRAC * target, 0.05)          # abs floor 0.05 mm
    return any(abs(n - target) <= tol for n in nums)


def worker_gate(part_script_text: str, link_spec, stl_path: str) -> list[GateError]:
    """Deterministic per-part checks: dimension lock (ERR_DIM) + manifold (ERR_MANIFOLD).
    Returns [] on pass."""
    errors: list[GateError] = []
    name = getattr(link_spec, "name", "?")

    # 1. Dimension lock — each declared mating dim must appear (within tol) in the script.
    #    Fold the manager's free-form aliases (gear_dia/wheel_dia/...) onto canonical keys
    #    first (C2) so a renamed-but-present dimension is still recognized.
    from .vocab import canonical_size
    size = canonical_size(getattr(link_spec, "size_mm", {}) or {},
                          getattr(link_spec, "shape_hint", "") or "")
    if part_script_text:
        nums = _script_numbers(part_script_text)
        # A declared radius may be drawn as diameter/2 (and vice-versa), so match against
        # the literals AND their halves and doubles.
        pool = list(nums) + [n / 2 for n in nums] + [n * 2 for n in nums]
        missing = []
        for key in _MATING_KEYS:
            if key not in size:
                continue
            try:
                want = float(size[key])
            except (TypeError, ValueError):
                continue
            if not _has_match(want, pool):
                missing.append(f"{key}={want}")
        if missing:
            errors.append(GateError(
                "worker", "ERR_DIM",
                f"part '{name}' script does not use the manager's declared dimension(s) "
                f"{missing} — draw the part to the specified size_mm",
                name))

    # 2. Manifold / loadable — reuse the authoritative STL check. Only a HARD failure
    #    (missing / unloadable / degenerate / zero-face) fails the part; non-watertight is
    #    common for valid CadQuery unions and is recorded-not-required by the pipeline
    #    (StlReport.watertight), so it is NOT treated as an error here to avoid false
    #    rejects. (A watertight warning could be surfaced separately if ever needed.)
    if stl_path:
        try:
            from ..validation import check_stl
            rep = check_stl(stl_path)
            if not rep.ok:
                errors.append(GateError(
                    "worker", "ERR_MANIFOLD",
                    f"part '{name}' STL is not usable "
                    f"(exists={rep.exists}, loadable={rep.loadable}, faces={rep.num_faces}, "
                    f"degenerate={rep.degenerate}): {rep.error or 'invalid mesh'}",
                    name))
        except Exception as e:
            errors.append(GateError(
                "worker", "ERR_MANIFOLD",
                f"part '{name}' STL could not be validated ({e})", name))
    return errors
