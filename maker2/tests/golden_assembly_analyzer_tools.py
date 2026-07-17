"""Focused safety and candidate checks for the self-directed assembly analyzer."""
from __future__ import annotations
import json,os,tempfile
from types import SimpleNamespace
from maker2.assembly_analyzer import _resolver,analyzer_trace_path
from maker2.assembly_repair import _parse_bores,generate_repair_candidates


def security():
 d=tempfile.mkdtemp();open(os.path.join(d,'ok.json'),'w').write('{}');r=_resolver(d)
 assert r('ok.json').endswith('ok.json')
 for p in ('../x','C:/Windows/system.ini','/etc/passwd','.env'):
  try:r(p);raise AssertionError(p)
  except ValueError:pass

def bore_parse():
 s='''def build():\n    bores = [(0.0,20.0),(-70.0,25.0),(-140.0,30.0)]\n    return bores\n'''
 p=_parse_bores(s);assert p and p['bores'][1]==[-70.0,25.0]

def radial_guard():
 pair={'seam_id':'mesh','parent_sub':'a','child_sub':'b','parent_part':'ga','child_part':'gb',
       'absolute_residual_mm':74.,'required_distance_mm':50.,'radial_distance_mm':124.,
       'axial_delta_mm':-19.,'face_overlap_mm':0.,'minimum_face_overlap_mm':.1}
 pose=SimpleNamespace(name='place_gb',child='gb',xyz_m=(0,0,.03),rpy_rad=(0,0,0))
 link=SimpleNamespace(name='gb',spin_axis=(0,0,1))
 subs={'b':SimpleNamespace(model=SimpleNamespace(poses=[pose],links=[link]))}
 settings=SimpleNamespace(enable_solver_pose_repair=True,enable_solver_seat_geometry_repair=False)
 report={'diagnostics':{'failure_kind':'gear_face_overlap'},'gear_pairs':[pair]}
 candidates=generate_repair_candidates(report,None,subs,settings)
 assert len(candidates)==1 and not candidates[0].allowed
 pair['radial_distance_mm']=51.5
 candidates=generate_repair_candidates(report,None,subs,settings)
 assert candidates[0].allowed

def traces():
 d=tempfile.mkdtemp();a=analyzer_trace_path(d,'slvs','slvs_x',1);b=analyzer_trace_path(d,'precheck','precheck_x',1)
 assert a!=b and a.endswith('slvs_slvs_x_attempt_1.json') and b.endswith('precheck_precheck_x_attempt_1.json')

if __name__=='__main__':security();bore_parse();radial_guard();traces();print('golden assembly analyzer tools: PASS')
