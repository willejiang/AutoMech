"""Executable golden tests for deterministic trajectory metrics."""
from __future__ import annotations

import math

from benchmark_scorer.metrics import (
    all_finite,
    axis_drift,
    carrying_distance_std,
    circularity_residual,
    closure_residual,
    finite_fraction,
    lateral_drift,
    motion_coverage,
    net_angular_travel,
    orbit_radius_variation,
    pair_distance_variation,
    reversal_count,
    span,
    stable_ratio_regression,
    unwrap_angles,
)


def close(actual: float, expected: float, tolerance: float = 1e-9) -> None:
    assert abs(actual - expected) <= tolerance, (actual, expected)


def main() -> int:
    wrapped = [6.0, 6.2, 0.1, 0.4]
    unwrapped = unwrap_angles(wrapped)
    assert all(b > a for a, b in zip(unwrapped, unwrapped[1:]))
    close(net_angular_travel(wrapped), unwrapped[-1] - unwrapped[0])
    assert all_finite([[1.0, 2.0], [3.0, 4.0]])
    close(finite_fraction([1.0, math.nan, 3.0, math.inf]), 0.5)
    assert not all_finite([1.0, math.nan])

    x = [0.2 * i for i in range(61)]
    y = [-value / 4.0 + 0.03 for value in x]
    regression = stable_ratio_regression(x, y, min_input_travel=6.0)
    close(regression.slope, -0.25)
    close(regression.r_squared, 1.0)
    try:
        stable_ratio_regression([0.0, 0.01, 0.02], [0.0, 1.0, 2.0], min_input_travel=0.1)
    except ValueError as error:
        assert "below minimum" in str(error)
    else:
        raise AssertionError("min-travel guard did not reject a static denominator")

    close(span([-2.0, 4.0, 1.0]), 6.0)
    assert reversal_count([0, 1, 2, 1.96, 2.03, 1, 0, 1, 2], hysteresis=0.2) == 2
    close(motion_coverage(([0, 2], [5, 5.05], [0, -3]), min_span=1.0), 2 / 3)

    close(axis_drift([(1, 2, 3), (1.3, 2.4, 3)]), 0.5)
    close(lateral_drift([(0, 0, 0), (1, 0.1, 0), (2, -0.2, 0)], (1, 0, 0)), 0.3)
    first = [(0, 0), (1, 0), (2, 0)]
    rigid = [(0, 3), (1, 3), (2, 3)]
    close(carrying_distance_std(first, rigid), 0.0)
    close(pair_distance_variation(first, rigid), 0.0)
    close(closure_residual(first, [(0, 0.1), (1, 0.2), (2, 0.05)]), 0.2)

    circle = [(math.cos(i * math.pi / 8), math.sin(i * math.pi / 8)) for i in range(16)]
    close(circularity_residual(circle, center=(0, 0)), 0.0, 1e-12)
    close(orbit_radius_variation(circle, (0, 0)), 0.0, 1e-12)

    for invalid_call in (
        lambda: span([0.0, math.nan]),
        lambda: axis_drift([(0, 0), (math.inf, 0)]),
        lambda: axis_drift([(0, 0), (1.7e308, 1.7e308)]),
        lambda: circularity_residual([]),
    ):
        try:
            invalid_call()
        except ValueError:
            pass
        else:
            raise AssertionError("non-finite/empty input was not rejected")

    print("golden benchmark scorer metrics: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
