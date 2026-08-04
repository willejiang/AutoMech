"""Golden gear-RATIO test: a reduction pair's constraint must match the TOOTH ratio.

`golden_two_gears` proves teeth transmit; it cannot catch a wrong ratio, because it uses
two identical gears. At 1:1 every radius error cancels — tip, pitch, root all give 1.000 —
so a builder that measured entirely the wrong radius still passed it. This test uses
UNEQUAL gears, where the error does not cancel.

The bug it locks down: speed ratio is the ratio of PITCH radii, but a bounding box only
sees the TIP radius, which overshoots pitch by one module. On 15T/45T at m=0.6 that is
17/47 = 0.362 instead of 15/45 = 0.333, and the small gear is hurt most (+2 teeth on 15
is +13%, on 45 only +4.4%), so the measured ratio is dragged toward 1:1. Cascaded over a
watch's two stages it turned a designed 12:1 into 9.9:1, and because the gears themselves
were correct, the agent was told "ratio wrong" and re-cut correct teeth for 7 iterations.

`_pitch_radii_m` recovers the module from the pair itself (two meshing spur gears share
one, so centre = r_a + r_b with r_i = tip_i - m gives m = (tip_a + tip_b - centre)/2),
needing no tooth count, no module, and no metadata — the designer may author gears with
any API.

Run:  python -m maker2.tests.golden_gear_ratio
Exit 0 = every stage within tolerance of its tooth ratio.
"""
from __future__ import annotations

import math
import os
import sys
import tempfile

import trimesh

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _ROOT)

from maker2.mjcf_builder import _pitch_radii_m, _tip_radius_m   # noqa: E402

MODULE_MM = 0.6           # a watch-scale module: this is where the tip error bites
FACE_MM = 1.8


def make_gear_stl(path: str, module: float, teeth: int, face: float, bore: float) -> None:
    """An involute-ish spur gear as a polygon extrusion: root disc + trapezoidal teeth out
    to the tip circle. Mirrors what the injected `make_gear` builds, so the STL a
    designer's script produces and the STL this test measures are the same shape."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    pitch_r = module * teeth / 2.0
    tip_r = pitch_r + module
    root_r = max(pitch_r - 1.25 * module, bore / 2.0 + 0.5)
    half = (math.pi / teeth) / 2.0
    tip_half = half * 0.65

    def disc(r, n=256):
        return Polygon([(r * math.cos(2 * math.pi * k / n),
                         r * math.sin(2 * math.pi * k / n)) for k in range(n)])

    # Teeth start just inside the root disc so each overlaps it rather than touching it
    # tangentially — a shared boundary point is what makes the union topologically invalid.
    shapes = [disc(root_r)]
    for i in range(teeth):
        a = 2.0 * math.pi * i / teeth
        shapes.append(Polygon([
            (r * math.cos(a + h), r * math.sin(a + h))
            for r, h in ((root_r - 0.02, half), (pitch_r, half), (tip_r, tip_half),
                         (tip_r, -tip_half), (pitch_r, -half), (root_r - 0.02, -half))]))
    poly = unary_union(shapes)
    if bore > 0:
        poly = poly.difference(disc(bore / 2.0, 64))
    trimesh.creation.extrude_polygon(poly, height=face).export(path)


def main() -> int:
    # (drive teeth, driven teeth). All real reductions: near-1:1 pairs are deliberately
    # excluded because the tip error cancels there, which is exactly why the existing
    # equal-gear golden test could not catch this bug.
    STAGES = [(15, 45), (12, 48), (11, 37), (16, 64)]
    TOL = 0.01           # 1% of the tooth ratio

    d = tempfile.mkdtemp(prefix="golden_ratio_")
    failures = []
    print(f"module {MODULE_MM}mm, tolerance {TOL*100:.0f}% of the tooth ratio\n")
    print(f"{'pair':>10}{'teeth ratio':>13}{'from pitch':>12}{'err':>9}"
          f"{'from tip':>11}{'tip err':>9}")

    for za, zb in STAGES:
        a, b = f"g{za}", f"g{zb}"
        make_gear_stl(os.path.join(d, f"{a}.stl"), MODULE_MM, za, FACE_MM, 2.0)
        make_gear_stl(os.path.join(d, f"{b}.stl"), MODULE_MM, zb, FACE_MM, 2.0)

        # Mesh them at the true centre distance, in meters (what the builder passes).
        centre_m = MODULE_MM * (za + zb) / 2.0 / 1000.0
        truth = za / zb

        pitch = _pitch_radii_m(d, a, b, centre_m)
        if pitch is None:
            failures.append(f"{za}/{zb}: module solve returned None")
            print(f"{za:4}/{zb:<5}{truth:13.4f}{'FAILED':>12}")
            continue
        got = pitch[0] / pitch[1]
        err = abs(got - truth) / truth

        ta, tb = _tip_radius_m(d, a), _tip_radius_m(d, b)
        tip_ratio = ta / tb
        tip_err = abs(tip_ratio - truth) / truth

        print(f"{za:4}/{zb:<5}{truth:13.4f}{got:12.4f}{err*100:8.2f}%"
              f"{tip_ratio:11.4f}{tip_err*100:8.2f}%")
        if err > TOL:
            failures.append(f"{za}/{zb}: got {got:.4f}, want {truth:.4f} ({err*100:.1f}%)")
        # The test is only meaningful where the old method was actually wrong.
        if tip_err <= TOL:
            failures.append(f"{za}/{zb}: tip radii are within tolerance too — this pair "
                            f"cannot detect the regression, pick a smaller module")

    # A cascade multiplies each stage's error, which is how 12:1 became 9.9:1.
    print()
    for chain, want in (([(15, 45), (12, 48)], 12.0),):
        prod_pitch, prod_tip = 1.0, 1.0
        for za, zb in chain:
            a, b = f"g{za}", f"g{zb}"
            centre_m = MODULE_MM * (za + zb) / 2.0 / 1000.0
            pr = _pitch_radii_m(d, a, b, centre_m)
            prod_pitch *= pr[0] / pr[1]
            prod_tip *= _tip_radius_m(d, a) / _tip_radius_m(d, b)
        got, old = 1 / prod_pitch, 1 / prod_tip
        print(f"cascade {chain}: {got:.4f}:1 (want {want}:1, tip method gives {old:.4f}:1)")
        if abs(got - want) / want > TOL:
            failures.append(f"cascade: got {got:.4f}:1, want {want}:1")

    print()
    if failures:
        for f in failures:
            print("FAIL:", f)
        return 1
    print("golden gear ratio: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
