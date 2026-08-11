"""Golden support MJCF applies only accepted agent-declared topology patches."""
from __future__ import annotations
import json,tempfile
from pathlib import Path
from build123d import Align,Box,export_stl
from maker2.mjcf_validation import execute_compiler,validate_candidate
from maker2.mjcf_support_derivative import derive_support_mjcf


def main():
 d=Path(tempfile.mkdtemp(prefix='golden_support_derivative_'));(d/'meshes').mkdir()
 for n in ('base','payload'):export_stl(Box(10,10,5,align=(Align.CENTER,Align.CENTER,Align.MIN)),d/f'meshes/{n}.stl')
 I=[[.01,0,0],[0,.01,0],[0,0,.01]];F=lambda z:{'xyz_m':[0,0,z],'quat_wxyz':[1,0,0,0],'matrix':[[1,0,0,0],[0,1,0,0],[0,0,1,z],[0,0,0,1]]}
 facts={'run_dir':str(d),'entity_ids':['link/base','link/payload'],'model':{'name':'x','root_link':'base','links':[{'name':'base'},{'name':'payload'}]},'links':{
  'base':{'world_frame':F(0),'mesh_path':'meshes/base.stl','mass_kg':1,'com_m':[0,0,0],'inertia_kg_m2':I,'friction':[1,.05,.005],'dof':'fixed'},
  'payload':{'world_frame':F(.01),'mesh_path':'meshes/payload.stl','mass_kg':1,'com_m':[0,0,0],'inertia_kg_m2':I,'friction':[1,.05,.005],'dof':'fixed'}},'simulation':{'gravity':[0,0,-9.81],'timestep':.001,'solver':'Newton','iterations':10}}
 (d/'mjcf_facts.json').write_text(json.dumps(facts))
 src='''
def compile_mjcf(facts,out):
 out.topology_plan({'support_ground':'base','coordinate_map':{},'tree_edges':[],'closure_edges':[],'rigid_carried':['payload'],'independent_coaxial':[],'transmissions':[],'contact_decisions':[],'support_strategy':['remove carry weld and free payload']})
 out.body('base');out.body('payload')
 out.weld('carry','payload','base','normal assembly carries payload',['link/payload'],['link/payload'])
 out.support_patch('remove_constraint','carry','test whether geometry supports payload')
 out.support_patch('free_body','payload','release carried payload after weld removal')
 out.decision('link/base','emitted',['base'],'support ground',['link/base'])
 out.decision('link/payload','emitted',['payload','carry'],'normally carried body',['link/payload'])
'''
 xml,manifest=execute_compiler(src,facts);assert validate_candidate(xml,manifest,facts,d/'model.mjcf',run_smoke=False)['ok']
 (d/'builder_manifest.json').write_text(json.dumps(manifest))
 path,ground=derive_support_mjcf(d/'model.mjcf',d/'builder_manifest.json',d/'mjcf_facts.json',d/'model_support.mjcf')
 text=Path(path).read_text();assert ground=='base' and 'name="carry"' not in text and 'name="support_free_payload"' in text
 print('golden mjcf support derivative: PASS')
if __name__=='__main__':main()
