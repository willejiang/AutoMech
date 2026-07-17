"""Golden checks for the authoritative libslvs cross-sub backend.
Run: python -m maker2.tests.golden_slvs_cross_sub
"""
from __future__ import annotations

import numpy as np
from types import SimpleNamespace

from maker2.constraint_ir import (AssemblyConstraintProblem, ConstraintKind,
                                  ConstraintSolveResult, ConstraintSpec, EntityKind, EntitySpec,
                                  RigidStageSpec)
from maker2.slvs_adapter import (solve_problem, SlvsSolveError, _rot_a_to_b, _se3,
                                 build_cross_sub_problem, failure_report_dict,
                                 reconstruct_placements, validate_authoritative_solution)
from maker2.assembler import _classify_subs,_gear_face_center_offset_mm


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


def face_origin_notes():
    front=SimpleNamespace(size_mm={'thickness':15},origin_note='front face (-Z) at local origin; gear body extends +Z 0..15mm')
    rear=SimpleNamespace(size_mm={'thickness':20},origin_note='rear face (+Z) at local origin; gear body extends -Z')
    centered=SimpleNamespace(size_mm={'thickness':10},origin_note='Local origin is at the gear pitch-plane center and bore center; spans Z=-5 to Z=+5')
    assert _gear_face_center_offset_mm(front)==7.5
    assert _gear_face_center_offset_mm(rear)==-10.0
    assert _gear_face_center_offset_mm(centered)==0.0


def face_overlap_checks():
    stages={
      'a':RigidStageSpec('a',np.eye(4).tolist(),(0,0,0),(1,0,0),(0,0,0),(1,0,0),
                         {'ga':(0,0,0)}),
      'b':RigidStageSpec('b',np.eye(4).tolist(),(0,0,0),(1,0,0),(0,.08,0),(1,0,0),
                         {'gb':(0,0,0)})}
    gp={'seam_id':'mesh','parent_sub':'a','child_sub':'b','parent_part':'ga','child_part':'gb',
        'parent_entity':'a:gear:ga','child_entity':'b:gear:gb','target_m':.08,
        'parent_face_width_mm':12.,'child_face_width_mm':15.,
        'parent_face_center_offset_mm':0.,'child_face_center_offset_mm':0.}
    problem=AssemblyConstraintProblem(stages=stages,gear_pairs=[gp],expected_dof=0)
    result=ConstraintSolveResult('okay',0,0)
    placements={'a':np.eye(4),'b':np.eye(4)}
    placements['b'][:3,3]=[0,.08,0]
    validate_authoritative_solution(problem,placements,result)
    placements['b'][:3,3]=[.02,.08,0]
    try:
        validate_authoritative_solution(problem,placements,result)
        raise AssertionError('non-overlapping gear faces accepted')
    except SlvsSolveError as e:
        assert 'gear_face_overlap:mesh' in str(e)
        report=failure_report_dict(problem,result,placements,e)
        pair=report['gear_pairs'][0]
        assert pair['face_overlap_mm']==0 and pair['axial_delta_mm']==20
        assert report['diagnostics']['failure_kind']=='gear_face_overlap'


def _mount_fixture(contract_positions,realized_positions,face_width=10.,mate_type='insert',shaft_dia_mm=8,
                   rear=False,rear_shift=0.0):
    frames=[]; seams=[]; subspecs=[]; results={}; seed={}
    housing_frames=[]
    for i,(contract,realized) in enumerate(zip(contract_positions,realized_positions)):
        sid=f's{i}'; hf=f'seat{i}'; sf=f'shaft{i}'
        housing_frames.append(SimpleNamespace(name=hf,role='mount',axis=(1,0,0),
                                              xyz_m=contract,shaft_dia_mm=shaft_dia_mm))
        shaft_frame=SimpleNamespace(name=sf,role='mount',axis=(1,0,0),xyz_m=(0,0,0),shaft_dia_mm=shaft_dia_mm)
        stage_frames=[shaft_frame];rh=f'{hf}_rear';rs=f'{sf}_rear'
        if rear:
            housing_frames.append(SimpleNamespace(name=rh,role='mount',axis=(1,0,0),
              xyz_m=(contract[0]+.05,contract[1],contract[2]),shaft_dia_mm=shaft_dia_mm))
            stage_frames.append(SimpleNamespace(name=rs,role='mount',axis=(1,0,0),xyz_m=(.05,0,0),shaft_dia_mm=shaft_dia_mm))
        subspecs.append(SimpleNamespace(id=sid,frames=stage_frames))
        seams.append(SimpleNamespace(id=f'mount{i}',kind='weld',mate_type=mate_type,parent_sub='housing',
                     child_sub=sid,parent_frame=hf,child_frame=sf,mesh_pair=(),
                     rear_parent_frame=(rh if rear else ''),rear_child_frame=(rs if rear else '')))
        links=[SimpleNamespace(name=f'shaft{i}',dof='spin',spin_axis=(1,0,0),size_mm={}),
               SimpleNamespace(name=f'g{i}',dof='fixed',spin_axis=(1,0,0),
                               size_mm={'module':2,'teeth':20,'thickness':face_width})]
        results[sid]=SimpleNamespace(id=sid,model=SimpleNamespace(links=links))
        seed[sid]=np.eye(4)
    subspecs.insert(0,SimpleNamespace(id='housing',frames=housing_frames))
    results['housing']=SimpleNamespace(id='housing',model=SimpleNamespace(links=[]))
    if len(contract_positions)>1:
        seams.append(SimpleNamespace(id='mesh',kind='power',mate_type='',parent_sub='s0',child_sub='s1',
                     parent_frame='m0',child_frame='m1',mesh_pair=('g0','g1')))
    plan=SimpleNamespace(root_sub='housing',subassemblies=subspecs,seams=seams)

    def frame_in_root(sub,name):
        T=np.eye(4);is_rear=name.endswith('_rear');base=name[:-5] if is_rear else name
        if sub.id=='housing':
            idx=int(base.replace('seat',''));T[:3,3]=realized_positions[idx]
            if is_rear:T[:3,3]+=[.05+rear_shift,0,0]
        elif is_rear:T[:3,3]=[.05,0,0]
        return name,T
    def link_in_root(sub,name):
        if name.startswith('g'):return np.array([1.,0,0]),np.array([0.,0,0])
        return np.array([1.,0,0]),np.array([0.,0,0])
    def gear_link(sub,seam,which):return next(l for l in sub.model.links if l.name==seam.mesh_pair[which])
    helpers={'frame_in_root':frame_in_root,'link_in_root':link_in_root,'gear_link':gear_link,
             'gear_radius':lambda l:l.size_mm['module']*l.size_mm['teeth']/2,
             'gear_face_width':lambda l:l.size_mm.get('thickness'),
             'gear_face_center_offset':lambda l:0.0}
    return plan,results,seed,helpers


def rear_datum_checks():
    plan,subs,seed,helpers=_mount_fixture([(0,0,0),(0,.04,0)],[(0,0,0),(0,.04,0)],rear=True)
    problem=build_cross_sub_problem(plan,subs,seed,{'s0','s1'},'housing',**helpers)
    rear=[c for c in problem.constraints if c.id.endswith(':rear_plane')]
    assert len(rear)==2 and all(not c.enforced_by_solver for c in rear)
    result=ConstraintSolveResult('okay',0,0);placements={'s0':np.eye(4),'s1':np.eye(4)};placements['s1'][:3,3]=[0,.04,0]
    validate_authoritative_solution(problem,placements,result)
    plan,subs,seed,helpers=_mount_fixture([(0,0,0),(0,.04,0)],[(0,0,0),(0,.04,0)],rear=True,rear_shift=.01)
    problem=build_cross_sub_problem(plan,subs,seed,{'s0','s1'},'housing',**helpers)
    try:
        validate_authoritative_solution(problem,placements,result);raise AssertionError('rear plane miss accepted')
    except SlvsSolveError as e:assert 'rear_mount_plane:' in str(e)


def seat_mount_topology():
    plan,subs,seed,helpers=_mount_fixture([(0,0,0),(0,.08,0)],[(0,0,0),(0,.08,0)],
                                          mate_type='seat',shaft_dia_mm=0)
    gear_ids,base_id=_classify_subs(plan,subs)
    assert gear_ids=={'s0','s1'} and base_id=='housing'
    problem=build_cross_sub_problem(plan,subs,seed,gear_ids,base_id,**helpers)
    assert set(problem.stages)==gear_ids


def mount_layout_checks():
    plan,subs,seed,helpers=_mount_fixture([(0,0,0),(0,.08,0)],[(0,0,0),(0,0,0)])
    try:
        build_cross_sub_problem(plan,subs,seed,{'s0','s1'},'housing',**helpers)
        raise AssertionError('collapsed housing mounts accepted')
    except SlvsSolveError as e:
        assert e.problem and e.problem.diagnostics['failure_kind']=='housing_mount_layout'
        pair=e.problem.diagnostics['mount_layout']['pairs'][-1]
        assert pair['collapsed'] and pair['contract_separation_mm']==80
    # Gross expansion is the same physical frame-binding fault as collapse.
    plan,subs,seed,helpers=_mount_fixture([(0,0,0),(0,.08,0)],[(0,0,0),(0,.124,0)])
    try:
        build_cross_sub_problem(plan,subs,seed,{'s0','s1'},'housing',**helpers)
        raise AssertionError('expanded housing mount layout accepted')
    except SlvsSolveError as e:
        pair=e.problem.diagnostics['mount_layout']['pairs'][-1]
        assert not pair['collapsed'] and pair['layout_mismatch']
        assert round(pair['layout_error_mm'])==44 and round(pair['layout_limit_mm'])==12
    # Contract-coincident mounts remain legal even when their realizations coincide.
    plan,subs,seed,helpers=_mount_fixture([(0,0,0),(0,0,0)],[(0,0,0),(0,0,0)])
    problem=build_cross_sub_problem(plan,subs,seed,{'s0','s1'},'housing',**helpers)
    assert not problem.diagnostics['mount_layout']['pairs'][0]['collapsed']
    plan,subs,seed,helpers=_mount_fixture([(0,0,0),(0,.08,0)],[(0,0,0),(0,.08,0)],face_width=None)
    try:
        build_cross_sub_problem(plan,subs,seed,{'s0','s1'},'housing',**helpers)
        raise AssertionError('missing gear face width accepted')
    except SlvsSolveError as e:
        assert e.problem.diagnostics['failure_kind']=='gear_face_geometry'


if __name__=='__main__':
    native_solve(); inconsistent_solve(); rotation_checks(); face_origin_notes(); face_overlap_checks(); rear_datum_checks(); seat_mount_topology(); mount_layout_checks()
    print('golden authoritative libslvs cross-sub: PASS')
