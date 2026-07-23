"""Lower compiled design parameters into the shared solver-neutral constraint IR."""
from __future__ import annotations

from maker2.constraint_ir import (AssemblyConstraintProblem, ConstraintKind, ConstraintSpec,
                                  EntityKind, EntitySpec)


def build_parallel_shaft_problem(stage_points_m: dict[str, tuple[float, float, float]],
                                 center_distances_m: tuple[tuple[str, str, float], ...],
                                 *, layout: str) -> AssemblyConstraintProblem:
    """Build a fully anchored zero-DOF shaft-center skeleton independent of built CAD."""
    entities = []
    constraints = []
    for role in sorted(stage_points_m):
        point = tuple(float(v) for v in stage_points_m[role])
        entities.extend((
            EntitySpec(f"target:{role}", EntityKind.POINT_3D, point, fixed=True,
                       provenance={"role": role, "layout": layout, "authority": "template"}),
            EntitySpec(f"shaft:{role}", EntityKind.POINT_3D, point,
                       provenance={"role": role, "layout": layout}),
        ))
        constraints.append(ConstraintSpec(f"anchor:{role}", ConstraintKind.COINCIDENT,
                                          (f"shaft:{role}", f"target:{role}"),
                                          provenance={"role": role}))
    for index, (left, right, distance) in enumerate(center_distances_m):
        constraints.append(ConstraintSpec(f"mesh:{index}:center_distance", ConstraintKind.DISTANCE,
                                          (f"shaft:{left}", f"shaft:{right}"), float(distance),
                                          enforced_by_solver=False,
                                          provenance={"roles": [left, right]}))
    return AssemblyConstraintProblem(entities=entities, constraints=constraints,
                                     expected_dof=0, units="m", base_id="design_world",
                                     diagnostics={"phase": "design_compile", "layout": layout,
                                                  "stage_points_m": stage_points_m})
