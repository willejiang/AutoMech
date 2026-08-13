"""Deterministic trajectory metrics for mechanism benchmarks.

All distances use the caller's units.  Helpers reject non-finite samples rather
than allowing NaN/Inf to turn a comparison into an accidental pass.
"""
from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

_EPS = 1e-12


@dataclass(frozen=True)
class RatioRegression:
    """Least-squares output/input slope over a stable trajectory interval."""

    slope: float
    intercept: float
    r_squared: float
    input_travel: float
    samples: int


Point = Sequence[float]


def require_finite(values: Iterable[float], *, name: str = "values") -> tuple[float, ...]:
    """Return finite floats, raising ``ValueError`` for empty or invalid input."""
    raw = tuple(values)
    if not raw:
        raise ValueError(f"{name} must not be empty")
    if any(isinstance(value, (bool, str, bytes)) or not isinstance(value, (int, float))
           for value in raw):
        raise ValueError(f"{name} must contain only JSON numbers")
    result = tuple(float(value) for value in raw)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} contains a non-finite value")
    return result


def finite_fraction(values: Iterable[object]) -> float:
    """Fraction of scalar/vector samples whose numeric components are finite."""
    samples = tuple(values)
    if not samples:
        return 0.0

    def sample_is_finite(sample: object) -> bool:
        if isinstance(sample, (bool, str, bytes)):
            return False
        if isinstance(sample, Iterable):
            components = tuple(sample)
            return bool(components) and all(sample_is_finite(item) for item in components)
        try:
            return math.isfinite(float(sample))
        except (TypeError, ValueError):
            return False

    return sum(sample_is_finite(sample) for sample in samples) / len(samples)


def all_finite(values: Iterable[object]) -> bool:
    """Whether a non-empty scalar/vector sample series is entirely finite."""
    samples = tuple(values)
    return bool(samples) and finite_fraction(samples) == 1.0


def unwrap_angles(angles: Iterable[float], *, period: float = 2.0 * math.pi) -> tuple[float, ...]:
    """Unwrap a wrapped angular series using shortest adjacent increments."""
    values = require_finite(angles, name="angles")
    if not math.isfinite(period) or period <= 0.0:
        raise ValueError("period must be finite and positive")
    half = period / 2.0
    unwrapped = [values[0]]
    for previous, current in zip(values, values[1:]):
        raw_delta = current - previous
        if not math.isfinite(raw_delta):
            raise ValueError("angle delta overflowed the finite range")
        delta = (raw_delta + half) % period - half
        if not math.isfinite(delta):
            raise ValueError("angle unwrap produced a non-finite value")
        # Preserve the sign of an exact half-period jump.
        if math.isclose(delta, -half) and current - previous > 0.0:
            delta = half
        unwrapped.append(unwrapped[-1] + delta)
    return tuple(unwrapped)


def net_angular_travel(angles: Iterable[float], *, period: float = 2.0 * math.pi) -> float:
    """Signed endpoint displacement after unwrap, not accumulated path length."""
    values = unwrap_angles(angles, period=period)
    return values[-1] - values[0]


def stable_ratio_regression(
    input_angles: Iterable[float],
    output_angles: Iterable[float],
    *,
    min_input_travel: float,
    trim_fraction: float = 0.1,
    period: float = 2.0 * math.pi,
) -> RatioRegression:
    """Regress unwrapped output angle against input after trimming transients.

    ``min_input_travel`` guards against meaningless ratios from a nearly static
    denominator.  ``trim_fraction`` removes that fraction from each end.
    """
    inputs = unwrap_angles(input_angles, period=period)
    outputs = unwrap_angles(output_angles, period=period)
    if len(inputs) != len(outputs):
        raise ValueError("input_angles and output_angles must have equal length")
    if len(inputs) < 2:
        raise ValueError("ratio regression needs at least two samples")
    if not math.isfinite(min_input_travel) or min_input_travel <= 0.0:
        raise ValueError("min_input_travel must be finite and positive")
    if not math.isfinite(trim_fraction) or not 0.0 <= trim_fraction < 0.5:
        raise ValueError("trim_fraction must be in [0, 0.5)")

    trim = int(len(inputs) * trim_fraction)
    start, stop = trim, len(inputs) - trim
    if stop - start < 2:
        raise ValueError("stable interval has fewer than two samples")
    x, y = inputs[start:stop], outputs[start:stop]
    travel = abs(x[-1] - x[0])
    if travel < min_input_travel:
        raise ValueError(
            f"input travel {travel:.6g} is below minimum {min_input_travel:.6g}"
        )

    x_mean = sum(x) / len(x)
    y_mean = sum(y) / len(y)
    sxx = sum((value - x_mean) ** 2 for value in x)
    if sxx <= _EPS:
        raise ValueError("input angle has no usable variance")
    sxy = sum((xv - x_mean) * (yv - y_mean) for xv, yv in zip(x, y))
    slope = sxy / sxx
    intercept = y_mean - slope * x_mean
    residual = sum((yv - (slope * xv + intercept)) ** 2 for xv, yv in zip(x, y))
    total = sum((yv - y_mean) ** 2 for yv in y)
    r_squared = 1.0 if total <= _EPS and residual <= _EPS else 1.0 - residual / total
    return RatioRegression(slope, intercept, r_squared, travel, len(x))


def span(values: Iterable[float]) -> float:
    """Peak-to-peak scalar displacement."""
    samples = require_finite(values)
    return max(samples) - min(samples)


def reversal_count(values: Iterable[float], *, hysteresis: float) -> int:
    """Count direction changes after a Schmitt-trigger-style hysteresis filter."""
    samples = require_finite(values)
    if not math.isfinite(hysteresis) or hysteresis < 0.0:
        raise ValueError("hysteresis must be finite and non-negative")
    anchor = samples[0]
    direction = 0
    reversals = 0
    for value in samples[1:]:
        delta = value - anchor
        if direction == 0:
            if abs(delta) >= hysteresis and (hysteresis > 0.0 or delta != 0.0):
                direction = 1 if delta > 0.0 else -1
                anchor = value
        elif direction > 0:
            if value > anchor:
                anchor = value
            elif anchor - value >= hysteresis and (hysteresis > 0.0 or value != anchor):
                direction = -1
                reversals += 1
                anchor = value
        else:
            if value < anchor:
                anchor = value
            elif value - anchor >= hysteresis and (hysteresis > 0.0 or value != anchor):
                direction = 1
                reversals += 1
                anchor = value
    return reversals


def motion_coverage(series: Iterable[Iterable[float]], *, min_span: float) -> float:
    """Fraction of required scalar coordinates that move by at least ``min_span``."""
    coordinates = tuple(series)
    if not coordinates:
        raise ValueError("series must contain at least one coordinate")
    if not math.isfinite(min_span) or min_span < 0.0:
        raise ValueError("min_span must be finite and non-negative")
    return sum(span(values) >= min_span for values in coordinates) / len(coordinates)


def _points(points: Iterable[Point], *, name: str = "points") -> tuple[tuple[float, ...], ...]:
    result = tuple(require_finite(point, name=name) for point in points)
    if not result:
        raise ValueError(f"{name} must not be empty")
    dimension = len(result[0])
    if dimension == 0 or any(len(point) != dimension for point in result):
        raise ValueError(f"{name} must have a consistent positive dimension")
    return result


def _distance(a: Point, b: Point) -> float:
    value = math.hypot(*(av - bv for av, bv in zip(a, b)))
    if not math.isfinite(value):
        raise ValueError("point distance overflowed the finite range")
    return value


def axis_drift(centers: Iterable[Point]) -> float:
    """Maximum world-space displacement of an axis center from its first sample."""
    points = _points(centers, name="centers")
    return max(_distance(points[0], point) for point in points)


def lateral_drift(positions: Iterable[Point], guide_axis: Point) -> float:
    """Peak-to-peak motion perpendicular to a guide axis."""
    points = _points(positions, name="positions")
    axis = require_finite(guide_axis, name="guide_axis")
    if len(axis) != len(points[0]):
        raise ValueError("guide_axis dimension does not match positions")
    norm = math.sqrt(sum(value * value for value in axis))
    if norm <= _EPS:
        raise ValueError("guide_axis must be non-zero")
    unit = tuple(value / norm for value in axis)
    origin = points[0]
    perpendicular = []
    for point in points:
        delta = tuple(value - base for value, base in zip(point, origin))
        axial = sum(value * direction for value, direction in zip(delta, unit))
        residual = tuple(value - axial * direction
                         for value, direction in zip(delta, unit))
        perpendicular.append(residual)
    dimension_spans = [max(point[i] for point in perpendicular)
                       - min(point[i] for point in perpendicular)
                       for i in range(len(unit))]
    value = math.hypot(*dimension_spans)
    if not math.isfinite(value):
        raise ValueError("lateral drift overflowed the finite range")
    return value


def pair_distances(first: Iterable[Point], second: Iterable[Point]) -> tuple[float, ...]:
    """Per-sample Euclidean distance between paired trajectories."""
    a, b = _points(first, name="first"), _points(second, name="second")
    if len(a) != len(b) or len(a[0]) != len(b[0]):
        raise ValueError("paired trajectories must have matching shape")
    return tuple(_distance(av, bv) for av, bv in zip(a, b))


def carrying_distance_std(first: Iterable[Point], second: Iterable[Point]) -> float:
    """Population standard deviation of a rigid-carried pair's separation."""
    distances = pair_distances(first, second)
    mean = sum(distances) / len(distances)
    return math.sqrt(sum((value - mean) ** 2 for value in distances) / len(distances))


def pair_distance_variation(first: Iterable[Point], second: Iterable[Point]) -> float:
    """Peak-to-peak separation variation (e.g. planet gear to carrier pin)."""
    return span(pair_distances(first, second))


def orbit_radius_variation(orbit: Iterable[Point], center: Iterable[Point] | Point) -> float:
    """Peak-to-peak orbit radius variation around a fixed or moving center."""
    orbit_points = _points(orbit, name="orbit")
    center_values = tuple(center)
    if not center_values:
        raise ValueError("center must not be empty")
    if isinstance(center_values[0], Iterable) and not isinstance(center_values[0], (str, bytes)):
        centers = _points(center_values, name="center")
    else:
        fixed = require_finite(center_values, name="center")
        centers = tuple(fixed for _ in orbit_points)
    return span(pair_distances(orbit_points, centers))


def circularity_residual(points: Iterable[Point], center: Point | None = None) -> float:
    """RMS radial residual from a circle with a fixed or centroid-estimated center."""
    samples = _points(points)
    if center is None:
        center_value = tuple(sum(point[i] for point in samples) / len(samples) for i in range(len(samples[0])))
    else:
        center_value = require_finite(center, name="center")
        if len(center_value) != len(samples[0]):
            raise ValueError("center dimension does not match points")
    radii = tuple(_distance(point, center_value) for point in samples)
    mean = sum(radii) / len(radii)
    return math.sqrt(sum((radius - mean) ** 2 for radius in radii) / len(radii))


def closure_residual(first: Iterable[Point], second: Iterable[Point]) -> float:
    """Maximum separation of corresponding closure ports."""
    return max(pair_distances(first, second))


# Descriptive aliases keep call sites readable without changing the canonical
# definitions above.
motion_span = span
count_reversals = reversal_count
ratio_regression = stable_ratio_regression
carrying_distance_variation = carrying_distance_std
orbit_pair_distance_variation = pair_distance_variation
circularity_error = circularity_residual
closure_error = closure_residual


__all__ = [
    "RatioRegression", "all_finite", "axis_drift", "carrying_distance_std",
    "carrying_distance_variation", "circularity_error", "circularity_residual",
    "closure_error", "closure_residual", "count_reversals", "finite_fraction",
    "lateral_drift", "motion_coverage", "motion_span", "net_angular_travel",
    "orbit_pair_distance_variation", "orbit_radius_variation", "pair_distances",
    "pair_distance_variation", "ratio_regression", "require_finite",
    "reversal_count", "span", "stable_ratio_regression", "unwrap_angles",
]
