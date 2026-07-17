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

def traces():
 d=tempfile.mkdtemp();a=analyzer_trace_path(d,'slvs','slvs_x',1);b=analyzer_trace_path(d,'precheck','precheck_x',1)
 assert a!=b and a.endswith('slvs_slvs_x_attempt_1.json') and b.endswith('precheck_precheck_x_attempt_1.json')

if __name__=='__main__':security();bore_parse();traces();print('golden assembly analyzer tools: PASS')
