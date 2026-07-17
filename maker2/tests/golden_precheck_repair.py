"""Focused goldens for structured precheck reports and physical housing repair."""
from __future__ import annotations
import copy,json,os,tempfile
from types import SimpleNamespace
import numpy as np
from maker2.precheck import PrecheckReport,Violation
from maker2.assembly_repair import (RepairCandidate,_parse_bores,
                                    generate_precheck_repair_candidates,
                                    precheck_repair_acceptance)
from maker2.model import KinematicModel,LinkSpec,PoseSpec,SubassemblyPlan


def structured_ids():
    kwargs=dict(kind='part_overlap',severity='interface',seam_id='weld_housing_input',
                involved_sub_ids=['housing','input'],parent_link='housing_front_plate',
                child_link='input_bearing',parent_local_link='front_plate',
                child_local_link='bearing',shaft_role='input',overlap_fraction=.4,
                threshold=.3)
    a=Violation(**kwargs);b=Violation(**kwargs)
    assert a.violation_id==b.violation_id
    ra=PrecheckReport(False,[a]);rb=PrecheckReport(False,[b])
    assert ra.failure_id==rb.failure_id and ra.failure_id.startswith('precheck_')
    d=ra.to_dict();assert d['violations'][0]['seam_id']=='weld_housing_input'
    assert abs(d['aggregate_overlap']-.4)<1e-9


def script_parse():
    two='''import cadquery as cq\ndef build():\n p=cq.Workplane("YZ").box(180,56,8)\n p=p.faces(">X").workplane().pushPoints([(-40,5),(40,-5)]).cboreHole(20,28,2)\n return p\n'''
    parsed=_parse_bores(two)
    assert parsed and parsed['bores']==[[-40.0,5.0],[40.0,-5.0]]


def _fixture(points=((0,-.06,0),(0,0,0),(0,.06,0)),half_y=90):
    root=tempfile.mkdtemp();subdir=os.path.join(root,'sub_housing');os.makedirs(os.path.join(subdir,'cq'))
    script='''import cadquery as cq\ndef build():\n p=cq.Workplane("YZ").box(180,56,8)\n p=p.faces(">X").workplane().pushPoints([(-40,0),(40,0)]).cboreHole(20,28,2)\n return p\n'''
    for name in ('front_plate','rear_plate'):
        open(os.path.join(subdir,'cq',name+'.py'),'w').write(script)
    links=[LinkSpec('front_plate','front plate','box',{'x':8,'y':half_y*2,'z':56}),
           LinkSpec('rear_plate','rear plate','box',{'x':8,'y':half_y*2,'z':56})]
    model=KinematicModel('housing','front_plate',links,
        [PoseSpec('rear','front_plate','rear_plate',(0.05,0,0),(0,0,0))])
    frames=[]
    for side,link in (('front','front_plate'),('rear','rear_plate')):
        for role in ('input','inter','output'):
            frames.append({'frame':f'seat_{role}_{side}','link':link,
                           'local_xyz_m':[0,0,0],'local_rpy_rad':[0,0,0]})
    res=SimpleNamespace(id='housing',model=model,sub_frames=frames,results=[],ok=True,error='',
                        ctx=SimpleNamespace(run_dir=subdir,model_json_path=os.path.join(subdir,'kinematic_model.json'),
                          urdf_path=os.path.join(subdir,'model.urdf'),meshes_dir=os.path.join(subdir,'meshes')))
    plan=SubassemblyPlan('reducer','housing',subassemblies=[SimpleNamespace(id='housing',frames=[])])
    asm=os.path.join(root,'assembly_iter_0');os.makedirs(asm)
    mounts=[];placements={}
    for role,stage,point in zip(('input','inter','output'),('input_stage','inter_stage','output_stage'),points):
        T=np.eye(4);T[:3,3]=point;placements[stage]=T.tolist()
        mounts.append({'stage':stage,'seam':f'weld_{role}','housing_frame':f'seat_{role}_front',
                       'realized_position_m':list(point),'contract_axis':[1,0,0]})
    json.dump({'status':'okay','placements':placements,
               'diagnostics':{'mount_layout':{'mounts':mounts}}},
              open(os.path.join(asm,'assembly_constraint_report.json'),'w'),indent=2)
    violations=[]
    for role,stage in zip(('input','inter','output'),('input_stage','inter_stage','output_stage')):
        violations.append({'kind':'part_overlap','severity':'interface','shaft_role':role,
          'violation_id':'v_'+role,'seam_id':'weld_'+role,
          'involved_sub_ids':['housing',stage],'parent_link':'housing_front_plate',
          'child_link':stage+'_bearing','overlap_fraction':.5})
    report={'failure_id':'precheck_fixture','aggregate_overlap':1.5,'violations':violations}
    ctx=SimpleNamespace(run_dir=asm)
    return report,plan,{'housing':res},ctx


def candidate_and_blocks():
    report,plan,subs,ctx=_fixture()
    candidates=generate_precheck_repair_candidates(report,plan,subs,ctx,
        SimpleNamespace(enable_precheck_housing_geometry_repair=True))
    assert len(candidates)==1
    c=candidates[0]
    assert c.allowed and len(c.target['frame_bore_mapping'])==6
    assert c.rebuild_scope=={'subassemblies':['housing'],'links':['front_plate','rear_plate']}
    assert c.target['centers_by_link']['front_plate']['input']==[-60.0,0.0]
    assert c.target['cutter_properties']['rear_plate'][0]['values_mm']==[20.0,28.0,2.0]
    ambiguous=copy.deepcopy(report);ambiguous['violations'][2]['shaft_role']='inter'
    assert not generate_precheck_repair_candidates(ambiguous,plan,subs,ctx,SimpleNamespace(enable_precheck_housing_geometry_repair=True))
    edge_report,edge_plan,edge_subs,edge_ctx=_fixture(points=((0,-.08,0),(0,0,0),(0,.08,0)),half_y=90)
    assert not generate_precheck_repair_candidates(edge_report,edge_plan,edge_subs,edge_ctx,SimpleNamespace(enable_precheck_housing_geometry_repair=True))
    web_report,web_plan,web_subs,web_ctx=_fixture(points=((0,-.01,0),(0,0,0),(0,.01,0)))
    assert not generate_precheck_repair_candidates(web_report,web_plan,web_subs,web_ctx,SimpleNamespace(enable_precheck_housing_geometry_repair=True))


def accessory_candidate():
    report,plan,subs,ctx=_fixture()
    res=subs['housing'];res.model.links.append(LinkSpec('fill_plug','plug','cylinder',{'radius':8,'height':18}))
    res.model.poses.append(PoseSpec('plug_pose','front_plate','fill_plug',(0,.015,.02),(0,0,0)))
    report['violations']=[{'kind':'part_overlap','severity':'interface','shaft_role':'output',
      'violation_id':'v_plug','seam_id':'weld_output','involved_sub_ids':['housing','output_stage'],
      'parent_local_link':'fill_plug','child_local_link':'output_gear','overlap_fraction':1.0}]
    candidates=generate_precheck_repair_candidates(report,plan,subs,ctx,
      SimpleNamespace(enable_precheck_housing_geometry_repair=True))
    assert len(candidates)==1 and candidates[0].candidate_type=='housing_accessory_relocation'
    assert candidates[0].new_value['fill_plug'][1] < 0


def acceptance():
    candidate=RepairCandidate('c','housing_multibore_pattern',{'target_violation_ids':['a']},None,None,{},'',{},True,'')
    before={'ok':False,'aggregate_overlap':.5,'violations':[{'violation_id':'a','kind':'part_overlap','severity':'interface','seam_id':'s','involved_sub_ids':['h','i'],'parent_link':'p','child_link':'b'}]}
    clean={'ok':True,'aggregate_overlap':0.0,'violations':[]}
    assert precheck_repair_acceptance(before,clean,candidate)['accepted']
    remaining={'ok':False,'aggregate_overlap':.2,'violations':before['violations']}
    assert not precheck_repair_acceptance(before,remaining,candidate)['accepted']
    new={'ok':False,'aggregate_overlap':.1,'violations':[{'violation_id':'n','kind':'load_error','severity':'interface','seam_id':'','involved_sub_ids':[],'parent_link':'','child_link':''}]}
    assert not precheck_repair_acceptance(before,new,candidate)['accepted']


if __name__=='__main__':
    structured_ids();script_parse();candidate_and_blocks();accessory_candidate();acceptance()
    print('golden precheck repair: PASS')
