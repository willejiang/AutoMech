"""Golden candidate and transaction checks."""
from __future__ import annotations
import json,os,tempfile
from types import SimpleNamespace
from maker2.assembly_repair import (_parse_bores,RepairCandidate,_cid,LocalRepairTransaction,
                                    generate_repair_candidates,solver_repair_acceptance)
from maker2.model import KinematicModel,LinkSpec,PoseSpec

def candidates():
 d={'type':'housing_multibore_pattern','sub':'housing','link':'shell','centers':[0,-80,-200]}
 assert _cid(d)==_cid(dict(d))
 s='bores = [(0.0,20.0),(-70.0,25.0),(-140.0,30.0)]\n'
 assert _parse_bores(s)['bores'][2][0]==-140.0

def rear_candidate():
 m=KinematicModel('inter','shaft',[LinkSpec('shaft','shaft','cylinder',{'radius':10,'height':150},dof='spin'),
   LinkSpec('rear_bearing','rear bearing','cylinder',{'radius':16,'height':12})],
   [PoseSpec('place_rear','shaft','rear_bearing',(0,0,.138),(0,0,0))])
 m2=KinematicModel('input','shaft',[LinkSpec('shaft','shaft','cylinder',{'radius':8,'height':150},dof='spin'),
   LinkSpec('rear_bearing','rear bearing','cylinder',{'radius':14,'height':12})],
   [PoseSpec('place_rear','shaft','rear_bearing',(0,0,.144),(0,0,0))])
 report={'diagnostics':{'failure_kind':'rear_mount_plane','rear_mounts':[{
  'seam_id':'w','sub_id':'inter','shaft_frame':'rear_datum','signed_residual_mm':-12.,'passed':False},{
  'seam_id':'wi','sub_id':'input','shaft_frame':'rear_datum','signed_residual_mm':-6.,'passed':False}]},
  'stages':{'inter':{'local_axis':[0,0,1]},'input':{'local_axis':[0,0,1]}},'gear_pairs':[]}
 subs={'inter':SimpleNamespace(model=m,sub_frames=[{'frame':'rear_datum','link':'rear_bearing'}]),
       'input':SimpleNamespace(model=m2,sub_frames=[{'frame':'rear_datum','link':'rear_bearing'}])}
 cs=generate_repair_candidates(report,None,subs,SimpleNamespace(enable_solver_pose_repair=True,
  enable_solver_seat_geometry_repair=False))
 allowed=[c for c in cs if c.allowed];assert len(allowed)==2
 assert [next(iter(c.predicted_residuals_mm.values())) for c in allowed]==[12.,6.]
 assert abs(allowed[0].new_value[2]-.15)<1e-12
 assert abs(allowed[1].new_value[2]-.15)<1e-12

def acceptance():
 c=RepairCandidate('c','rear_mount_axial_alignment',{},None,None,{'w1':12.},'',{},True,'')
 before={'error':'rear_mount_plane:w1:12.000mm; rear_mount_plane:w2:6.000mm; gear_face_overlap:m:0.000mm'}
 improved={'error':'rear_mount_plane:w2:6.000mm; gear_face_overlap:m:0.000mm'}
 worse={'error':'rear_mount_plane:w2:6.000mm; gear_distance:new:3.000mm'}
 assert solver_repair_acceptance(before,improved,c)['accepted']
 assert not solver_repair_acceptance(before,worse,c)['accepted']

def rollback():
 root=tempfile.mkdtemp();open(os.path.join(root,'x'),'w').write('old')
 r=SimpleNamespace(ctx=SimpleNamespace(run_dir=root),model={'v':1},sub_frames=[1],results=[],ok=True,error='')
 try:
  with LocalRepairTransaction({'s':r},['s']):
   r.model={'v':2};open(os.path.join(root,'x'),'w').write('new');raise RuntimeError()
 except RuntimeError:pass
 assert r.model=={'v':1} and open(os.path.join(root,'x')).read()=='old'
if __name__=='__main__':candidates();rear_candidate();acceptance();rollback();print('golden assembly repair: PASS')
