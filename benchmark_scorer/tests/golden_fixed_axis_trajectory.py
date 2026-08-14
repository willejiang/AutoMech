"""Golden regression: fixed-axis invariants must use trajectory, not declarations."""
from __future__ import annotations

from benchmark_scorer.archive_evidence import _derived_invariants


def assembly() -> dict:
    return {
        "links": [
            {"name": "base", "dof": "fixed", "mount": ""},
            {"name": "input_shaft", "dof": "spin", "mount": "base"},
            {"name": "output_shaft", "dof": "spin", "mount": "base"},
            {"name": "input_pinion", "dof": "fixed", "mount": "input_shaft"},
            {"name": "output_gear", "dof": "fixed", "mount": "output_shaft"},
        ],
        "motion_joints": [
            {"name": "input_hinge", "child": "input_shaft"},
            {"name": "output_hinge", "child": "output_shaft"},
        ],
        "relations": [{
            "name": "mesh", "mate_type": "gear_spur_external",
            "base_part": "input_pinion", "incoming_part": "output_gear",
        }],
    }


def trajectory(input_positions, output_positions) -> dict:
    return {
        "bodies": {
            "base": [[0.0, 0.0, 0.0]] * len(input_positions),
            "input_shaft": input_positions,
            "output_shaft": output_positions,
        },
        "invariants": {},
    }


def main() -> None:
    manifest = {"topology_plan": {"coordinate_map": {
        "input_shaft": "input_hinge", "output_shaft": "output_hinge",
    }}}
    bindings = {"gear": ["input_pinion", "output_gear"]}

    stable = trajectory(
        [[-22.5, 0.0, 7.0], [-22.5, 0.0, 7.0]],
        [[22.5, 0.0, 7.0], [22.5, 0.0, 7.0]],
    )
    stable_result = _derived_invariants(
        "01_single_stage_4to1", assembly(), manifest, stable, bindings)
    assert stable_result["fixed_shaft_axes"] is True, stable_result

    orbiting = trajectory(
        [[-22.5, 0.0, 7.0], [22.5, 0.0, 7.0]],
        [[22.5, 0.0, 7.0], [-22.5, 0.0, 7.0]],
    )
    orbiting_result = _derived_invariants(
        "01_single_stage_4to1", assembly(), manifest, orbiting, bindings)
    assert orbiting_result["fixed_shaft_axes"] is False, orbiting_result

    # An off-center visual mesh does not move the body/joint frame. The trajectory stores
    # body origins, so a stable origin remains a valid fixed axis regardless of mesh COM.
    off_center_visual = trajectory(
        [[-22.5, 0.0, 7.0], [-22.5, 0.0, 7.0]],
        [[22.5, 0.0, 7.0], [22.5, 0.0, 7.0]],
    )
    assert _derived_invariants(
        "01_single_stage_4to1", assembly(), manifest,
        off_center_visual, bindings)["fixed_shaft_axes"] is True

    print("golden fixed-axis trajectory invariant: PASS")


if __name__ == "__main__":
    main()
