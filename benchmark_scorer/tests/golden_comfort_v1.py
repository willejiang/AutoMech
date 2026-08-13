"""Executable golden tests for the frozen Comfort v1 registry."""
from __future__ import annotations

import math

from benchmark_scorer.tasks.comfort_v1 import (
    CLOSURE_SCALE_FRACTION,
    FUNCTIONAL_POINTS,
    RECIPROCATING_FINITE_FRACTION_MIN,
    ROLE_CARDINALITIES,
    TASKS,
    TASK_REGISTRY,
    axis_drift_limit_mm,
    carrying_std_limit_mm,
    closure_limit,
    get_task,
    planet_pair_variation_limit_mm,
)


EXPECTED_IDS = (
    "01_single_stage_4to1",
    "02_two_stage_9to1",
    "03_idler_reverser_1to1",
    "04_openwork_clock_12to1",
    "05_three_planet_4to1",
    "06_four_planet_4to1",
    "07_horizontal_slider_crank",
    "08_vertical_piston_pump",
    "09_open_pumpjack",
    "10_wind_rotor_pump",
)


def main() -> int:
    assert tuple(task.task_id for task in TASKS) == EXPECTED_IDS
    assert tuple(TASK_REGISTRY) == EXPECTED_IDS
    assert len(set(EXPECTED_IDS)) == 10
    assert all(task.prompt.startswith("Design this as an open demonstration mechanism:") for task in TASKS)
    assert get_task("01_single_stage_4to1").ratio_min == 3.8
    assert get_task("01_single_stage_4to1").ratio_max == 4.2
    assert get_task("02_two_stage_9to1").input_min_rad == 9.0
    assert get_task("03_idler_reverser_1to1").direction == "same"
    assert get_task("04_openwork_clock_12to1").ratio_min == 11.4
    assert get_task("05_three_planet_4to1").ratio_min == 0.2375
    assert get_task("06_four_planet_4to1").ratio_max == 0.2625
    assert get_task("07_horizontal_slider_crank").output_span_min_mm == 20.0
    assert get_task("08_vertical_piston_pump").output_span_min_mm == 15.0
    assert get_task("09_open_pumpjack").finite_effort_required
    assert not get_task("10_wind_rotor_pump").finite_effort_required
    assert math.isclose(get_task("10_wind_rotor_pump").input_min_rad, 2 * math.pi)

    assert ROLE_CARDINALITIES["01_single_stage_4to1"]["gear"] == 2
    assert ROLE_CARDINALITIES["02_two_stage_9to1"]["compound_intermediate_shaft"] == 1
    assert ROLE_CARDINALITIES["03_idler_reverser_1to1"]["idler_shaft"] == 1
    assert ROLE_CARDINALITIES["04_openwork_clock_12to1"]["coaxial_hand"] == 2
    assert ROLE_CARDINALITIES["05_three_planet_4to1"]["planet_gear"] == 3
    assert ROLE_CARDINALITIES["05_three_planet_4to1"]["planet_pin_hinge"] == 3
    assert ROLE_CARDINALITIES["06_four_planet_4to1"]["planet_gear"] == 4
    assert ROLE_CARDINALITIES["07_horizontal_slider_crank"]["connecting_rod"] == 1
    assert ROLE_CARDINALITIES["08_vertical_piston_pump"]["piston_output"] == 1
    assert ROLE_CARDINALITIES["09_open_pumpjack"]["walking_beam"] == 1
    assert ROLE_CARDINALITIES["10_wind_rotor_pump"]["wind_rotor"] == 1

    assert dict(FUNCTIONAL_POINTS) == {"input": 5, "propagation": 10, "output": 15, "invariants": 10}
    assert sum(FUNCTIONAL_POINTS.values()) == 40
    assert RECIPROCATING_FINITE_FRACTION_MIN == 0.80
    assert CLOSURE_SCALE_FRACTION == 0.02
    assert axis_drift_limit_mm(50.0) == 1.0
    assert axis_drift_limit_mm(200.0) == 2.0
    assert carrying_std_limit_mm(50.0) == 0.5
    assert carrying_std_limit_mm(200.0) == 1.0
    assert planet_pair_variation_limit_mm(50.0) == 0.5
    assert planet_pair_variation_limit_mm(200.0) == 1.0
    assert closure_limit(10.0) == 0.2

    try:
        TASK_REGISTRY["new"] = TASKS[0]  # type: ignore[index]
    except TypeError:
        pass
    else:
        raise AssertionError("registry is not immutable")

    print("golden Comfort v1 task registry: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
