"""Focused safety and candidate checks for the self-directed assembly analyzer."""
from __future__ import annotations
import json,os,tempfile
from types import SimpleNamespace
from maker2.assembly_analyzer import _resolver
from maker2.assembly_repair import _parse_bores,generate_repair_candidates


def security():
 d=tempfile.mkdtemp();open(os.path.join(d,'ok.json'),'w').write('{}');r=_resolver(d)
 assert r('ok.json').endswith('ok.json')
 for p in ('../x','C:/Windows/system.ini','/etc/passwd','.env'):
  try:r(p);raise AssertionError(p)
  except ValueError:pass

def bore_parse():
 s='''def build():\n    bores = [(0.0,20.0),(-70.0,25.0),(-140.0,30.0)]\n    return bores\n'''
 p=_parse_bores(s);assert p and p[2][1]==[-70.0,25.0]

if __name__=='__main__':security();bore_parse();print('golden assembly analyzer tools: PASS')
