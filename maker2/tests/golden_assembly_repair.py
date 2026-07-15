"""Golden candidate and transaction checks."""
from __future__ import annotations
import json,os,tempfile
from types import SimpleNamespace
from maker2.assembly_repair import _parse_bores,RepairCandidate,_cid,LocalRepairTransaction

def candidates():
 d={'type':'housing_multibore_pattern','sub':'housing','link':'shell','centers':[0,-80,-200]}
 assert _cid(d)==_cid(dict(d))
 s='bores = [(0.0,20.0),(-70.0,25.0),(-140.0,30.0)]\n'
 assert _parse_bores(s)['bores'][2][0]==-140.0

def rollback():
 root=tempfile.mkdtemp();open(os.path.join(root,'x'),'w').write('old')
 r=SimpleNamespace(ctx=SimpleNamespace(run_dir=root),model={'v':1},sub_frames=[1],results=[],ok=True,error='')
 try:
  with LocalRepairTransaction({'s':r},['s']):
   r.model={'v':2};open(os.path.join(root,'x'),'w').write('new');raise RuntimeError()
 except RuntimeError:pass
 assert r.model=={'v':1} and open(os.path.join(root,'x')).read()=='old'
if __name__=='__main__':candidates();rollback();print('golden assembly repair: PASS')
