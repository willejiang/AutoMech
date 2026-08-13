"""Frozen Comfort / In-Distribution Suite v1 task registry.

The constants mirror ``benchmark.md`` and the prompts used by the 2026-08-11
suite runner.  They are data only: scoring remains deterministic and uses no
LLM judgment.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from types import MappingProxyType
from typing import Mapping


@dataclass(frozen=True)
class RoleCardinality:
    role: str
    count: int


@dataclass(frozen=True)
class ComfortTask:
    task_id: str
    prompt: str
    roles: tuple[RoleCardinality, ...]
    input_min_rad: float
    ratio_numerator: str | None = None
    ratio_denominator: str | None = None
    ratio_min: float | None = None
    ratio_max: float | None = None
    direction: str | None = None
    output_axis: str | None = None
    output_span_min_mm: float | None = None
    reversals_min: int | None = None
    finite_effort_required: bool = False
    invariants: tuple[str, ...] = ()

    @property
    def role_cardinalities(self) -> Mapping[str, int]:
        return MappingProxyType({item.role: item.count for item in self.roles})

    @property
    def prompt_sha256(self) -> str:
        return hashlib.sha256(self.prompt.encode("utf-8")).hexdigest()


EXPOSE = (
    "Design this as an open demonstration mechanism: expose the complete mechanical "
    "structure and motion path, and avoid covers, housings, bridges, or plates that "
    "obstruct the gears, shafts, bearings, joints, pins, linkages, carrier, or output "
    "from the simulation camera. "
)

_PROMPTS = {
    "01_single_stage_4to1": "Build an open-frame hand-cranked single-stage spur gear reducer with an exact 4:1 reduction. Use one input shaft with a visible hand crank and one parallel output shaft. Support both shafts in clearly visible running bearings, rigidly attach each gear to its own shaft, expose the complete tooth mesh, and include a stable bench-mounted base. The input must be the hand crank only; the output shaft must not be independently driven. Author complete mechanism semantics for the shaft hinges, press fits, running fits, gear mesh, driver, output, and watched links.",
    "02_two_stage_9to1": "Build an open-frame hand-cranked two-stage spur gear reducer with an exact overall 9:1 reduction, using two 3:1 stages. Use three parallel shafts: an input shaft with one visible hand crank, a compound intermediate shaft carrying both its driven gear and the second-stage pinion, and one output shaft. Support every shaft in visible running bearings, expose both tooth meshes, and mount the machine on a stable base. Only the input crank is driven. Author complete mechanism semantics, including the compound rigid carrying, both transmissions, driver, output, and watched links.",
    "03_idler_reverser_1to1": "Build an open-frame three-shaft spur gear reversing train with one input gear, one freely rotating idler gear, and one output gear. Use equal tooth counts for the input and output so the magnitude of the overall ratio is exactly 1:1. All three shaft axes must be parallel and fixed in the world, all two gear meshes must be fully visible, and only the input shaft has a hand crank. The idler must have its own independent hinge and must not be welded to either neighboring gear. Include a stable bench-mounted base and complete mechanism semantics for both meshes, all bearings, driver, output, and watched links.",
    "04_openwork_clock_12to1": "Build an openwork mechanical clock display with two clearly visible coaxial hands whose angular speed ratio is exactly 12:1. Use a visible spur gear train, independent coaxial shafts or sleeves with running clearances, rigidly attach each hand to its intended shaft, and expose the gears and both hands from the camera side. Mount the frame rigidly on a stable base. The minute-side input is the only driver; the hour hand is the final output. Coaxial members with different speeds must remain independent and must not be welded or forced to 1:1. Author complete mechanism semantics for every transmission, bearing, press fit, driver, output, and watched link.",
    "05_three_planet_4to1": "Build an open-frame hand-driven planetary reducer with a fixed ring gear, a sun gear input, exactly three equally spaced planet gears on a rigid carrier, and the carrier as output. Choose tooth counts that give an exact 4:1 sun-to-carrier reduction with the ring fixed. Every planet gear must be carried around the sun by the carrier while also spinning on its own dedicated carrier pin hinge. Expose the sun, all three planets, the ring, and the carrier; do not hide them behind a full cover. Include a stable base, a visible input crank, and complete planetary-stage semantics, meshes, bearings, driver, output, and watched links.",
    "06_four_planet_4to1": "Build an open-frame hand-driven planetary reducer with a fixed ring gear, a sun gear input, exactly four equally spaced planet gears on a rigid carrier, and the carrier as output. Choose tooth counts that give an exact 4:1 sun-to-carrier reduction with the ring fixed. Every planet gear must be carried by the carrier and must also spin on its own dedicated carrier pin hinge. Keep all four planets and their pins visibly exposed, use a stable base and visible input crank, and author complete planetary-stage semantics, meshes, bearings, driver, output, and watched links.",
    "07_horizontal_slider_crank": "Build an open-frame horizontal hand-cranked slider-crank mechanism. Fix the crankshaft axis rigidly to the base, attach one crank disk or crank web and one dedicated eccentric crank pin, connect it through a single rigid connecting rod to a slider constrained to one horizontal linear guide, and expose the complete crank, both rod ends, and slider. Use only one crankshaft input and do not directly actuate the slider. Author explicit revolute, slide, pin, closure, driver, output, and watched-link semantics. Keep rod collisions against the main shaft, web, frame, and guide physically meaningful; do not broadly exclude the whole rod from the crank body.",
    "08_vertical_piston_pump": "Build an open-frame bench-mounted vertical reciprocating piston-pump mechanism driven by a horizontal hand crank. Use one fixed crankshaft, one eccentric crank pin, one connecting rod, one guided vertical crosshead or piston slider, and one visible pump rod/piston moving only vertically inside an open cylinder frame. Keep the mechanism above the ground plane, expose the crank and rod linkage, and use only the crankshaft as input. This is a mechanical motion benchmark; do not claim or simulate fluid pressure. Author complete revolute, slide, closure, rigid-carrying, driver, output, and watched-link semantics.",
    "09_open_pumpjack": "Build an open-frame hand-cranked pumpjack mechanism on a stable base. Use one fixed horizontal crankshaft with a hand crank, one rotating crank disk and dedicated crank pin, one pitman connecting rod, one pivoted walking beam on a fixed central support, and one vertical polished-rod output guided to reciprocate. Keep the crank, pitman, beam pivot, and output rod fully visible. Only the crankshaft is driven. Author explicit hinge, pin, closure, slide or guided-output, driver, output, and watched-link semantics. This benchmark tests mechanical motion only, not underground fluid extraction.",
    "10_wind_rotor_pump": "Build an open-frame wind-rotor-driven reciprocating pump on a stable tower or bench frame. Use one fixed horizontal rotor shaft, a clearly visible wind rotor rigidly attached to it, one crank disk with a dedicated eccentric pin, one connecting rod, one guided vertical crosshead, and a visible pump rod/piston output that moves only vertically. Keep the entire rotor-to-crank-to-piston motion path exposed and use the wind rotor shaft as the only input. Author explicit world-frame shaft hinge, crank, pin, closure, slide, rigid-carrying, driver, output, and watched-link semantics. The benchmark tests imposed rotor-driven mechanical transmission, not aerodynamic power generation or fluid pressure.",
}


def _roles(**items: int) -> tuple[RoleCardinality, ...]:
    return tuple(RoleCardinality(role, count) for role, count in items.items())


TASKS = (
    ComfortTask("01_single_stage_4to1", EXPOSE + _PROMPTS["01_single_stage_4to1"], _roles(input_shaft=1, output_shaft=1, gear=2, hand_crank=1), 6.0, "input_shaft", "output_shaft", 3.8, 4.2, "opposite", invariants=("fixed_shaft_axes", "rigid_gear_carrying", "single_mesh")),
    ComfortTask("02_two_stage_9to1", EXPOSE + _PROMPTS["02_two_stage_9to1"], _roles(input_shaft=1, compound_intermediate_shaft=1, output_shaft=1, gear=4, hand_crank=1), 9.0, "input_shaft", "output_shaft", 8.5, 9.5, "same", invariants=("fixed_shaft_axes", "rigid_compound_pair", "two_live_meshes")),
    ComfortTask("03_idler_reverser_1to1", EXPOSE + _PROMPTS["03_idler_reverser_1to1"], _roles(input_shaft=1, idler_shaft=1, output_shaft=1, gear=3, hand_crank=1), 6.0, "output_shaft", "input_shaft", 0.95, 1.05, "same", invariants=("fixed_shaft_axes", "independent_idler_hinge", "two_live_meshes")),
    ComfortTask("04_openwork_clock_12to1", EXPOSE + _PROMPTS["04_openwork_clock_12to1"], _roles(minute_input=1, hour_output=1, coaxial_hand=2), 12.0, "minute_input", "hour_output", 11.4, 12.6, invariants=("coaxial_independent_hands", "hands_remain_carried")),
    ComfortTask("05_three_planet_4to1", EXPOSE + _PROMPTS["05_three_planet_4to1"], _roles(fixed_ring=1, sun_input=1, carrier_output=1, planet_gear=3, planet_pin_hinge=3, hand_crank=1), 12.0, "carrier_output", "sun_input", 0.2375, 0.2625, invariants=("ring_fixed", "planet_orbit", "planet_local_spin", "planet_pin_distance_constant", "equally_spaced")),
    ComfortTask("06_four_planet_4to1", EXPOSE + _PROMPTS["06_four_planet_4to1"], _roles(fixed_ring=1, sun_input=1, carrier_output=1, planet_gear=4, planet_pin_hinge=4, hand_crank=1), 12.0, "carrier_output", "sun_input", 0.2375, 0.2625, invariants=("ring_fixed", "planet_orbit", "planet_local_spin", "planet_pin_distance_constant", "spacing_90_deg")),
    ComfortTask("07_horizontal_slider_crank", EXPOSE + _PROMPTS["07_horizontal_slider_crank"], _roles(crankshaft_input=1, crank_pin=1, connecting_rod=1, horizontal_slider=1, horizontal_guide=1), 2.0 * 3.141592653589793, output_axis="horizontal", output_span_min_mm=20.0, reversals_min=2, finite_effort_required=True, invariants=("fixed_crank_axis", "lateral_drift_le_2pct_span", "closures_below_2pct_scale")),
    ComfortTask("08_vertical_piston_pump", EXPOSE + _PROMPTS["08_vertical_piston_pump"], _roles(crankshaft_input=1, eccentric_pin=1, connecting_rod=1, vertical_crosshead=1, vertical_guide=1, pump_rod=1, piston_output=1), 2.0 * 3.141592653589793, output_axis="vertical", output_span_min_mm=15.0, reversals_min=2, invariants=("fixed_crank_axis", "rod_crosshead_closure", "rigid_output_carrying", "no_ground_collision")),
    ComfortTask("09_open_pumpjack", EXPOSE + _PROMPTS["09_open_pumpjack"], _roles(crankshaft_input=1, hand_crank=1, crank_disk=1, crank_pin=1, pitman_rod=1, walking_beam=1, beam_pivot=1, polished_rod_output=1, vertical_guide=1), 2.0 * 3.141592653589793, output_axis="vertical", output_span_min_mm=15.0, reversals_min=2, finite_effort_required=True, invariants=("fixed_crank_axis", "fixed_beam_pivot", "beam_pivots", "closures_below_2pct_scale", "lateral_drift_le_5pct_span")),
    ComfortTask("10_wind_rotor_pump", EXPOSE + _PROMPTS["10_wind_rotor_pump"], _roles(rotor_shaft_input=1, wind_rotor=1, crank_disk=1, crank_pin=1, connecting_rod=1, vertical_crosshead=1, vertical_guide=1, pump_rod=1, piston_output=1), 2.0 * 3.141592653589793, output_axis="vertical", output_span_min_mm=15.0, reversals_min=2, invariants=("fixed_rotor_axis", "circular_crank_pin_path", "vertical_output", "rigid_output_carrying", "closures_below_2pct_scale")),
)

TASK_REGISTRY: Mapping[str, ComfortTask] = MappingProxyType({task.task_id: task for task in TASKS})

AXIS_DRIFT_ABS_MM = 1.0
AXIS_DRIFT_SCALE_FRACTION = 0.01
CARRYING_STD_ABS_MM = 0.5
CARRYING_STD_SCALE_FRACTION = 0.005
PLANET_PAIR_VARIATION_ABS_MM = 0.5
PLANET_PAIR_VARIATION_SCALE_FRACTION = 0.005
CLOSURE_SCALE_FRACTION = 0.02
RECIPROCATING_FINITE_FRACTION_MIN = 0.80
FUNCTIONAL_POINTS = MappingProxyType({"input": 5, "propagation": 10, "output": 15, "invariants": 10})
ROLE_CARDINALITIES: Mapping[str, Mapping[str, int]] = MappingProxyType({task.task_id: task.role_cardinalities for task in TASKS})


def get_task(task_id: str) -> ComfortTask:
    """Return a frozen task specification, raising ``KeyError`` if unknown."""
    return TASK_REGISTRY[task_id]


def _positive_scale(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def axis_drift_limit_mm(machine_diagonal_mm: float) -> float:
    scale = _positive_scale(machine_diagonal_mm, "machine_diagonal_mm")
    return max(AXIS_DRIFT_ABS_MM, AXIS_DRIFT_SCALE_FRACTION * scale)


def carrying_std_limit_mm(machine_diagonal_mm: float) -> float:
    scale = _positive_scale(machine_diagonal_mm, "machine_diagonal_mm")
    return max(CARRYING_STD_ABS_MM, CARRYING_STD_SCALE_FRACTION * scale)


def planet_pair_variation_limit_mm(machine_diagonal_mm: float) -> float:
    scale = _positive_scale(machine_diagonal_mm, "machine_diagonal_mm")
    return max(PLANET_PAIR_VARIATION_ABS_MM, PLANET_PAIR_VARIATION_SCALE_FRACTION * scale)


def closure_limit(scale: float) -> float:
    return CLOSURE_SCALE_FRACTION * _positive_scale(scale, "closure scale")
