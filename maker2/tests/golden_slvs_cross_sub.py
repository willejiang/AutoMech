"""Golden checks for the authoritative libslvs cross-sub backend.
Run: python -m maker2.tests.golden_slvs_cross_sub
"""
from __future__ import annotations

import numpy as np
from types import SimpleNamespace

from maker2.constraint_ir import (AssemblyConstraintProblem, ConstraintKind, ConstraintSpec,
                                  EntityKind, EntitySpec)
from maker2.slvs_adapter import (solve_problem, SlvsSolveError, _rot_a_to_b, _se3,
                                 build_cross_sub_problem, reconstruct_placements,
                                 validate_authoritative_solution)


def native_solve():
    # Three authoritative housing targets and three stage anchors/axes; two gear centers
    # with an exact 80 mm distance. This exercises the native API and zero-DOF contract.
    es=[]; cs=[]
    for i,y in enumerate((0., .08, .16)):
        sid=f"s{i}"; target=f"t{i}"; target_end=f"te{i}"; o=f"o{i}"; a=f"a{i}"; line=f"l{i}"
        es += [EntitySpec(target,EntityKind.POINT_3D,(0,y,0),fixed=True),
               EntitySpec(target_end,EntityKind.POINT_3D,(.1,y,0),fixed=True),
               EntitySpec(o,EntityKind.POINT_3D,(0,y+.001,0)),
               EntitySpec(a,EntityKind.POINT_3D,(.1,y+.001,0)),
               EntitySpec(line,EntityKind.LINE_3D,refs=(o,a))]
        cs += [ConstraintSpec(f"{sid}:o",ConstraintKind.COINCIDENT,(o,target)),
               ConstraintSpec(f"{sid}:a",ConstraintKind.COINCIDENT,(a,target_end))]
    es += [EntitySpec('g0',EntityKind.POINT_3D,(.05,0,0)),
           EntitySpec('g1',EntityKind.POINT_3D,(.05,.08,0))]
    cs += [ConstraintSpec('g0:on',ConstraintKind.POINT_ON_LINE,('g0','l0')),
           ConstraintSpec('g0:x',ConstraintKind.PROJECTED_DISTANCE,('o0','g0','l0'),-.05),
           ConstraintSpec('g1:on',ConstraintKind.POINT_ON_LINE,('g1','l1')),
           ConstraintSpec('g1:x',ConstraintKind.PROJECTED_DISTANCE,('o1','g1','l1'),-.05),
           ConstraintSpec('mesh',ConstraintKind.DISTANCE,('g0','g1'),.08,
                          enforced_by_solver=False)]
    r=solve_problem(AssemblyConstraintProblem(entities=es,constraints=cs,expected_dof=0))
    assert r.status=='okay' and r.dof==0 and not r.failed_constraint_ids
    assert abs(np.linalg.norm(np.array(r.points_m['g0'])-r.points_m['g1'])-.08)<1e-7


def inconsistent_solve():
    es=[EntitySpec('a',EntityKind.POINT_3D,(0,0,0),fixed=True),
        EntitySpec('b',EntityKind.POINT_3D,(.01,0,0),fixed=True),
        EntitySpec('x',EntityKind.POINT_3D,(0,0,0)),
        EntitySpec('y',EntityKind.POINT_3D,(.01,0,0))]
    cs=[ConstraintSpec('x=a',ConstraintKind.COINCIDENT,('x','a')),
        ConstraintSpec('y=b',ConstraintKind.COINCIDENT,('y','b')),
        ConstraintSpec('impossible',ConstraintKind.DISTANCE,('x','y'),.012)]
    r=solve_problem(AssemblyConstraintProblem(entities=es,constraints=cs,expected_dof=0))
    assert r.status=='inconsistent' and 'impossible' in r.failed_constraint_ids


def rotation_checks():
    R=_rot_a_to_b([0,0,1],[1,0,0]); T=np.eye(4);T[:3,:3]=R
    assert _se3(T) and np.allclose(R@[0,0,1],[1,0,0],atol=1e-7)


if __name__=='__main__':
    native_solve(); inconsistent_solve(); rotation_checks()
    print('golden authoritative libslvs cross-sub: PASS')
