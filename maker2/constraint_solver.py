"""Generic libslvs backend for solver-neutral constraint problems.

The constraint IR uses meters externally; py-slvs parameters use millimeters.
"""
from __future__ import annotations

from .constraint_ir import (AssemblyConstraintProblem, ConstraintKind,
                            ConstraintSolveResult, EntityKind)


class SlvsSolveError(RuntimeError):
    def __init__(self, message, *, problem=None, result=None, placements=None,
                 failure_report=None):
        super().__init__(message)
        self.problem = problem
        self.result = result
        self.placements = placements
        self.failure_report = failure_report


def slvs_available():
    """Return whether the py-slvs backend can be imported and its origin."""
    try:
        from py_slvs import slvs
        return True, getattr(slvs, "__file__", "py_slvs.slvs")
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"


def solve_problem(problem: AssemblyConstraintProblem) -> ConstraintSolveResult:
    """Solve a solver-neutral constraint problem with libslvs."""
    from py_slvs import slvs

    system = slvs.System()
    handles = {}
    constraint_handles = {}
    fixed_group, solve_group = 1, 2
    for entity in problem.entities:
        if entity.kind == EntityKind.POINT_3D:
            handles[entity.id] = system.addPoint3dV(
                *[value * 1000.0 for value in entity.initial_m],
                group=fixed_group if entity.fixed else solve_group)
    for entity in problem.entities:
        if entity.kind == EntityKind.LINE_3D:
            handles[entity.id] = system.addLineSegment(
                handles[entity.refs[0]], handles[entity.refs[1]], group=solve_group)
    for constraint in problem.constraints:
        if not constraint.enforced_by_solver:
            continue
        handle = None
        if constraint.kind == ConstraintKind.COINCIDENT:
            handle = system.addPointsCoincident(
                handles[constraint.entities[0]], handles[constraint.entities[1]],
                group=solve_group)
        elif constraint.kind == ConstraintKind.DISTANCE:
            handle = system.addPointsDistance(
                constraint.value_m * 1000.0,
                handles[constraint.entities[0]], handles[constraint.entities[1]],
                group=solve_group)
        elif constraint.kind == ConstraintKind.POINT_ON_LINE:
            handle = system.addPointOnLine(
                handles[constraint.entities[0]], handles[constraint.entities[1]],
                group=solve_group)
        elif constraint.kind == ConstraintKind.PROJECTED_DISTANCE:
            handle = system.addPointsProjectDistance(
                constraint.value_m * 1000.0,
                handles[constraint.entities[0]], handles[constraint.entities[1]],
                handles[constraint.entities[2]], group=solve_group)
        else:
            raise SlvsSolveError(f"unsupported IR constraint {constraint.kind}")
        constraint_handles[int(handle)] = constraint.id

    raw_status = int(system.solve(group=solve_group, reportFailed=True,
                                  findFreeParams=True))
    status = {
        0: "okay",
        1: "inconsistent",
        2: "didnt_converge",
        3: "too_many_unknowns",
        4: "init_error",
        5: "redundant",
    }.get(raw_status, f"unknown_{raw_status}")
    failed = [constraint_handles.get(int(handle), f"handle:{int(handle)}")
              for handle in system.Failed]
    points = {}
    for entity in problem.entities:
        if entity.kind != EntityKind.POINT_3D:
            continue
        handle = handles[entity.id]
        points[entity.id] = tuple(
            system.getParam(system.getEntityParam(handle, index)).val / 1000.0
            for index in range(3))
    return ConstraintSolveResult(
        status, raw_status, int(system.Dof), points, failed,
        {"entity_handles": len(handles),
         "constraint_handles": constraint_handles})
