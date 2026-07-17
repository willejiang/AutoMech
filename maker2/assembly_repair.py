"""Deterministic candidates and bounded local repair for authoritative-solver failures."""
from __future__ import annotations
import ast, copy, glob, hashlib, json, math, os, re, shutil, tempfile
from dataclasses import dataclass, asdict
import numpy as np
from .manager import save_model

@dataclass(frozen=True)
class RepairCandidate:
    candidate_id:str; candidate_type:str; target:dict; old_value:object; new_value:object
    predicted_residuals_mm:dict; dispatcher:str; rebuild_scope:dict; allowed:bool; rationale:str
    def to_dict(self): return asdict(self)

def _cid(x):
    raw=json.dumps(x,sort_keys=True,separators=(',',':')).encode()
    return hashlib.sha256(raw).hexdigest()[:16]

def _pose_candidate(pair, subs, allowed=False):
    sid, part = pair['child_sub'], pair['child_part']; res=subs.get(sid)
    poses=[p for p in (res.model.poses if res and res.model else []) if p.child==part]
    if len(poses)!=1: return None
    links=[l for l in res.model.links if l.name==part]
    if len(links)!=1:return None
    p=poses[0];old=np.asarray(p.xyz_m,float);axis=np.asarray(links[0].spin_axis,float)
    try:
        from .assembler import _mat
        axis=_mat((0,0,0),p.rpy_rad)[:3,:3]@axis
    except Exception:return None
    norm=float(np.linalg.norm(axis));axial=float(pair.get('axial_delta_mm',0))/1000.0
    if not np.all(np.isfinite(old)) or not math.isfinite(axial) or norm<1e-12:return None
    # axial_delta is projected onto the parent stage axis. Account for both stage-axis
    # direction and the built gear's +/- local-axis orientation when mapping into the
    # pose parent's coordinates.
    direction=(float(pair.get('axis_direction_dot',1.0)) *
               float(pair.get('child_gear_axis_sign',1.0)))
    axis=axis/norm;new=old-axial*direction*axis
    if np.linalg.norm(new-old)<1e-9:return None
    predicted=pair['absolute_residual_mm']
    data={'type':'gear_pose_axial_alignment','sub':sid,'part':part,'pose':p.name,'new':new.tolist()}
    return RepairCandidate('pose_'+_cid(data),'gear_pose_axial_alignment',
      {'sub_id':sid,'part':part,'pose':p.name},old.tolist(),new.tolist(),
      {pair['seam_id']:predicted},'model_pose_patch',{'subassemblies':[sid],'links':[]},
      bool(allowed),"Align this gear's face center axially; preserve teeth, module and radial pitch distance")

def _role_for_name(name):
    n=(name or '').upper()
    if 'INPUT' in n: return 'input'
    if 'INTER' in n or 'MIDDLE' in n: return 'inter'
    if 'OUTPUT' in n: return 'output'
    return None


def _literal_float(node):
    try:
        return float(ast.literal_eval(node))
    except Exception:
        return None


def _parse_bores(script):
    """Find a physical multi-bore pattern in literals or named constants, including
    constants local to a build function. Returns semantic input/inter/output roles."""
    tree=ast.parse(script); constants={}; lines={}
    for n in ast.walk(tree):
        if isinstance(n,ast.Assign) and len(n.targets)==1 and isinstance(n.targets[0],ast.Name):
            try: constants[n.targets[0].id]=float(ast.literal_eval(n.value));lines[n.targets[0].id]=(n.lineno,n.end_lineno)
            except Exception: pass
    for n in ast.walk(tree):
        if isinstance(n,(ast.Assign,ast.AnnAssign)):
            targets=n.targets if isinstance(n,ast.Assign) else [n.target]; value=n.value
            if any(isinstance(t,ast.Name) and t.id=='bores' for t in targets):
                try:v=ast.literal_eval(value)
                except Exception:return None
                if isinstance(v,list) and all(isinstance(x,(tuple,list)) and len(x)>=2 for x in v):
                    roles=['input','inter','output'][:len(v)]
                    return {'mode':'literal','line_start':n.lineno,'line_end':n.end_lineno,
                            'bores':[list(x) for x in v],'roles':roles}
    # Vectorized CadQuery idiom: pushPoints([(Y_INPUT,0), ...]) or literal 2-D points.
    for n in ast.walk(tree):
        if not isinstance(n,ast.Call) or not isinstance(n.func,ast.Attribute) or n.func.attr!='pushPoints' or not n.args: continue
        seq=n.args[0]
        if not isinstance(seq,(ast.List,ast.Tuple)): continue
        points=[];names=[];roles=[];components=[];ok=True
        for item in seq.elts:
            if not isinstance(item,(ast.Tuple,ast.List)) or len(item.elts)<2: ok=False;break
            point=[]; point_names=[]
            for e0 in item.elts[:2]:
                e=e0; sign=1.0
                if isinstance(e,ast.UnaryOp) and isinstance(e.op,ast.USub): sign=-1.0;e=e.operand
                if isinstance(e,ast.Name) and e.id in constants:
                    point.append(sign*constants[e.id]); point_names.append(e.id)
                else:
                    value=_literal_float(e0)
                    if value is None: ok=False;break
                    point.append(value); point_names.append(None)
            if not ok:break
            role=next((_role_for_name(x) for x in point_names if x and _role_for_name(x)),None)
            comp=next((i for i,x in enumerate(point_names) if x and _role_for_name(x)),0)
            points.append(point);names.append(next((x for x in point_names if x),None));roles.append(role);components.append(comp)
        if ok and len(points)>=2:
            if len(points)==3 and not any(roles): roles=['input','inter','output']
            return {'mode':'pushpoints','names':names,'roles':roles,'components':components,'lines':lines,
                    'push_line_start':n.lineno,'push_line_end':n.end_lineno,'bores':points,
                    'diameter_mm':next((v for k,v in constants.items() if 'BORE_D' in k),0.0)}
    # Aggregate repeated `.center(x, Y_ROLE)` bore cutters.
    cent=[]
    for n in ast.walk(tree):
        if not isinstance(n,ast.Call) or not isinstance(n.func,ast.Attribute) or n.func.attr!='center' or len(n.args)<2: continue
        for comp,e in enumerate(n.args[:2]):
            if isinstance(e,ast.Name) and e.id in constants and _role_for_name(e.id):
                cent.append((n,constants[e.id],e.id,_role_for_name(e.id),comp)); break
    if len(cent)>=2:
        return {'mode':'centers_multi','names':[x[2] for x in cent],'roles':[x[3] for x in cent],
                'components':[x[4] for x in cent],'lines':lines,
                'calls':[(x[0].lineno,x[0].end_lineno) for x in cent],
                'bores':[[x[1]] for x in cent],
                'diameter_mm':next((v for k,v in constants.items() if 'BORE_D' in k),0.0)}
    # Aggregate repeated single-point pushPoints calls (common back-wall style). Literal
    # points are supported because the precheck repair replaces a wrong two-point pattern
    # with one explicit Python-owned three-point pattern.
    agg=[]
    for n in ast.walk(tree):
        if not isinstance(n,ast.Call) or not isinstance(n.func,ast.Attribute) or n.func.attr!='pushPoints' or not n.args: continue
        seq=n.args[0]
        if not isinstance(seq,(ast.List,ast.Tuple)) or len(seq.elts)!=1: continue
        item=seq.elts[0]
        if not isinstance(item,(ast.Tuple,ast.List)) or len(item.elts)<2: continue
        point=[]; names=[]
        for e0 in item.elts[:2]:
            e=e0; sign=1.0
            if isinstance(e,ast.UnaryOp) and isinstance(e.op,ast.USub): sign=-1.0;e=e.operand
            if isinstance(e,ast.Name) and e.id in constants:
                point.append(sign*constants[e.id]); names.append(e.id)
            else:
                value=_literal_float(e0)
                if value is None: point=[]; break
                point.append(value); names.append(None)
        if point: agg.append((n,point,names))
    if len(agg)>=2:
        roles=[]
        for _n,_point,names in agg:
            roles.append(next((_role_for_name(x) for x in names if x and _role_for_name(x)),None))
        return {'mode':'pushpoints_multi','names':[x[2] for x in agg],'roles':roles,
                'lines':lines,'calls':[(x[0].lineno,x[0].end_lineno) for x in agg],
                'bores':[x[1] for x in agg],
                'diameter_mm':next((v for k,v in constants.items() if 'BORE_D' in k),0.0)}
    for n in ast.walk(tree):
        if not isinstance(n,ast.For) or not isinstance(n.iter,(ast.Tuple,ast.List)): continue
        vals=[]; names=[]; roles=[]; ok=True
        for i,e in enumerate(n.iter.elts):
            if isinstance(e,ast.Name) and e.id in constants:
                vals.append(constants[e.id]); names.append(e.id); roles.append(_role_for_name(e.id))
            else:
                try: vals.append(float(ast.literal_eval(e))); names.append(None); roles.append('input' if i==0 else None)
                except Exception: ok=False;break
        body=ast.get_source_segment(script,n) or ''
        if ok and len(vals)>=2 and ('.hole(' in body or '.circle(' in body):
            return {'mode':'constants','names':names,'roles':roles,'lines':lines,
                    'loop_line_start':n.lineno,'loop_line_end':n.end_lineno,
                    'bores':[[v] for v in vals],
                    'diameter_mm':next((v for k,v in constants.items() if 'BORE_D' in k),0.0)}
    return None


def _multibore_candidate(report, subs):
    base=report.get('base_id'); res=subs.get(base)
    if not res or not res.ctx:return None
    pairs=report.get('gear_pairs',[])
    if len(pairs)!=2:return None
    run=res.ctx.run_dir; patterns=[]
    for path in [os.path.join(run,'_batch_0.py'),os.path.join(run,'model.py')]+glob.glob(os.path.join(run,'cq','*.py')):
        if os.path.isfile(path):
            parsed=_parse_bores(open(path,encoding='utf-8').read())
            if parsed:patterns.append((path,parsed))
    # Prefer the script of the physical link carrying realized seat frames.
    frame_links=[e.get('link') for e in res.sub_frames or [] if str(e.get('frame','')).startswith('seat_')]
    preferred=os.path.join(run,'cq',f'{frame_links[0]}.py') if frame_links else ''
    primary=next(((p,x) for p,x in patterns if p==preferred and set(x.get('roles',())) >= {'input','inter','output'}),None)
    if primary is None: primary=next(((p,x) for p,x in patterns if set(x.get('roles',())) >= {'input','inter','output'}),None)
    if primary is None:return None
    _,parsed=primary; role_old={r:float(x[0]) for r,x in zip(parsed['roles'],parsed['bores']) if r}
    if set(role_old)<{'input','inter','output'}:return None
    # Use Python's coherent closed-form seed only to generate a candidate mount pattern;
    # libslvs remains the authority that must accept/reject it. Map stage IDs to semantic roles.
    stages=report.get('stages',{})
    role_sid={}
    for sid in stages:
        role=_role_for_name(sid)
        if role: role_sid[role]=sid
    if set(role_sid)<{'input','inter','output'}:return None
    suggested={r:stages[sid].get('suggested_target_anchor_m') for r,sid in role_sid.items()}
    if any(v is None for v in suggested.values()):return None
    # Detect the housing pattern coordinate (largest varying component) and preserve its sign/gauge.
    arr=np.asarray([suggested[r] for r in ('input','inter','output')],float)
    spans=np.ptp(arr,axis=0); active=np.flatnonzero(spans>1e-4)
    # Current literal patcher represents a one-dimensional bore pattern only.
    if len(active)!=1:return None
    dim=int(active[0]); raw={r:float(suggested[r][dim])*1000.0 for r in suggested}
    sign=1.0 if role_old['inter']>=role_old['input'] else -1.0
    scale_sign=1.0 if raw['inter']>=raw['input'] else -1.0
    centers={r:role_old['input']+sign*scale_sign*(raw[r]-raw['input']) for r in raw}
    physical=[]; scripts=[]
    for path,x in patterns:
        if (path.startswith(os.path.join(run,'cq')) and len(x.get('bores',[]))>=2
                and ('.hole(' in open(path,encoding='utf-8').read() or '.circle(' in open(path,encoding='utf-8').read())):
            physical.append(os.path.splitext(os.path.basename(path))[0]); scripts.append(path)
    if not physical:return None
    # Also keep shared batch/model sources synchronized when they carry the same full pattern.
    scripts += [p for p,x in patterns if not p.startswith(os.path.join(run,'cq'))
                and set(x.get('roles',())) >= {'input','inter','output'}]
    data={'type':'housing_multibore_pattern','sub':base,'centers':centers,'links':physical}
    target={'sub_id':base,'scripts':scripts,'centers_by_role':centers,
            'physical_links':physical,
            'frames':[e['frame'] for e in res.sub_frames or [] if str(e.get('frame','')).startswith('seat_')]}
    return RepairCandidate('bores_'+_cid(data),'housing_multibore_pattern',target,role_old,centers,
      {p['seam_id']:0.0 for p in pairs},'housing_multibore_patch',
      {'subassemblies':[base],'links':physical},True,
      'Pin input bore; recompute physical multi-bore constants from locked gear center distances')

def _frame_spec(plan, sub_id, frame_name):
    spec=plan.sub_by_id(sub_id) if hasattr(plan,'sub_by_id') else next((s for s in plan.subassemblies if s.id==sub_id),None)
    return next((f for f in (spec.frames if spec else []) if f.name==frame_name),None)


def _plate_half_extents_mm(link):
    size=getattr(link,'size_mm',{}) or {}
    vals=[]
    for key in ('y','z'):
        try: vals.append(float(size[key])/2.0)
        except Exception: vals.append(0.0)
    return vals


def _cutter_properties(script):
    tree=ast.parse(script); props=[]
    for n in ast.walk(tree):
        if not isinstance(n,ast.Call) or not isinstance(n.func,ast.Attribute):continue
        if n.func.attr not in ('hole','cboreHole','cskHole','circle'):continue
        values=[]
        for a in n.args[:3]:
            try:values.append(float(ast.literal_eval(a)))
            except Exception:values.append(None)
        props.append({'kind':n.func.attr,'values_mm':values})
    return props


def _line_plane_intersection_local(T_plate_world, point_world, axis_world):
    inv=np.linalg.inv(T_plate_world);p=(inv@np.append(point_world,1.0))[:3];a=inv[:3,:3]@axis_world
    if abs(float(a[0]))<1e-8:return None
    q=p-a*(p[0]/a[0])
    return [float(q[1]*1000.0),float(q[2]*1000.0)]


def generate_precheck_repair_candidates(report, plan, subs, assembly_ctx, settings):
    """Build one conservative physical housing-bore repair from a structured precheck report."""
    if not getattr(settings,'enable_precheck_housing_geometry_repair',True):return []
    violations=[v for v in report.get('violations',[]) if v.get('kind')=='part_overlap'
                and v.get('severity')=='interface' and v.get('shaft_role') in ('input','inter','output')]
    roles={v.get('shaft_role') for v in violations}
    if roles != {'input','inter','output'}:return []
    base_ids={v.get('involved_sub_ids',[None])[0] for v in violations}
    if len(base_ids)!=1:return []
    base=next(iter(base_ids));res=subs.get(base)
    if not res or not res.ctx or not res.model:return []
    report_path=os.path.join(assembly_ctx.run_dir,'assembly_constraint_report.json')
    try: solved=json.load(open(report_path,encoding='utf-8'))
    except Exception:return []
    placements=solved.get('placements',{});mounts=solved.get('diagnostics',{}).get('mount_layout',{}).get('mounts',[])
    if solved.get('status')!='okay' or not placements or len(mounts)!=3:return []
    by_role={}
    for m in mounts:
        role=_role_for_name(m.get('stage')) or _role_for_name(m.get('housing_frame'))
        if not role or m.get('stage') not in placements:return []
        T=np.asarray(placements[m['stage']],float);axis=np.asarray(m.get('contract_axis',(1,0,0)),float)
        n=float(np.linalg.norm(axis))
        if T.shape!=(4,4) or n<1e-9:return []
        point=np.asarray(m.get('realized_position_m'),float)
        # The solved shaft line passes through the housing mount target; placements prove
        # the stage orientation and uniqueness, while the target owns the physical center.
        by_role[role]={'point_world_m':point.tolist(),'axis_world':(axis/n).tolist(),
                       'stage':m['stage'],'seam_id':m.get('seam',''),'housing_frame':m.get('housing_frame','')}
    if set(by_role)!={'input','inter','output'}:return []
    pts=np.asarray([by_role[r]['point_world_m'] for r in ('input','inter','output')],float)
    if min(np.linalg.norm(pts[i]-pts[j]) for i in range(3) for j in range(i+1,3))<1e-4:return []
    from .assembler import _root_to_link
    r2l=_root_to_link(res.model);frame_entries=res.sub_frames or []
    frame_by_role_side={}
    for entry in frame_entries:
        frame=str(entry.get('frame',''));role=_role_for_name(frame)
        side=('rear' if ('rear' in frame.lower() or 'back' in frame.lower()) else
              'front' if 'front' in frame.lower() else None)
        if role and side:
            if (role,side) in frame_by_role_side:return []
            frame_by_role_side[(role,side)]=frame
    if set(frame_by_role_side)!={(r,s) for r in ('input','inter','output') for s in ('front','rear')}:return []
    plate_links=[]
    for link in res.model.links:
        path=os.path.join(res.ctx.run_dir,'cq',f'{link.name}.py')
        if ('plate' in link.name.lower() and os.path.isfile(path)
                and _parse_bores(open(path,encoding='utf-8').read())):
            plate_links.append(link.name)
    plate_links=sorted(plate_links)
    physical=[];scripts=[];centers_by_link={};old_by_link={};props={};mapping=[];margin_checks=[];web_checks=[]
    for link_name in plate_links:
        link=next((l for l in res.model.links if l.name==link_name),None);T=r2l.get(link_name)
        path=os.path.join(res.ctx.run_dir,'cq',f'{link_name}.py')
        if link is None or T is None or not os.path.isfile(path):continue
        text=open(path,encoding='utf-8').read();parsed=_parse_bores(text);cutters=_cutter_properties(text)
        if not parsed or not cutters:continue
        centers={}
        for role in ('input','inter','output'):
            q=_line_plane_intersection_local(T,np.asarray(by_role[role]['point_world_m']),np.asarray(by_role[role]['axis_world']))
            if q is None:return []
            centers[role]=q
            side='rear' if ('rear' in link_name.lower() or 'back' in link_name.lower()) else 'front'
            mapping.append({'role':role,'side':side,'frame':frame_by_role_side[(role,side)],
                            'link':link_name,'center_mm':q,'seam_id':by_role[role]['seam_id']})
        half=_plate_half_extents_mm(link);diameters=[x['values_mm'][0] for x in cutters if x['values_mm'] and x['values_mm'][0]]
        if not all(half) or not diameters:return []
        radius=max(diameters)/2.0; margin=2.0
        for role,q in centers.items():
            ok=(half[0]-abs(q[0])>=radius+margin and half[1]-abs(q[1])>=radius+margin)
            margin_checks.append({'link':link_name,'role':role,'margin_mm':margin,'passed':ok})
            if not ok:return []
        rows=list(centers.items())
        for i,(ra,a) in enumerate(rows):
            for rb,b in rows[i+1:]:
                web=float(np.linalg.norm(np.asarray(a)-b)-2*radius);ok=web>=2.0
                web_checks.append({'link':link_name,'roles':[ra,rb],'web_mm':web,'passed':ok})
                if not ok:return []
        physical.append(link_name);scripts.append(path);centers_by_link[link_name]=centers
        old_by_link[link_name]=parsed.get('bores',[]);props[link_name]=cutters
    if len(physical)!=2 or len(mapping)!=6:return []
    target_ids=[v['violation_id'] for v in violations]
    data={'type':'housing_multibore_pattern','sub':base,'centers':centers_by_link,'links':physical,
          'targets':sorted(target_ids)}
    frame_centers={x['frame']:x['center_mm'] for x in mapping}
    target={'sub_id':base,'scripts':scripts,'physical_links':physical,'centers_by_link':centers_by_link,
            'frame_centers':frame_centers,'frame_bore_mapping':mapping,'cutter_properties':props,'margin_checks':margin_checks,
            'web_checks':web_checks,'target_violation_ids':target_ids,'solved_report_path':report_path}
    return [RepairCandidate('bores_'+_cid(data),'housing_multibore_pattern',target,old_by_link,centers_by_link,
      {x:0.0 for x in target_ids},'housing_multibore_patch',{'subassemblies':[base],'links':physical},True,
      'Cut exactly six plate bores on the three authoritative solved shaft lines')]


def precheck_repair_acceptance(before, after, candidate):
    def signature(v):
        return (v.get('kind'),v.get('severity'),v.get('seam_id'),
                tuple(sorted(v.get('involved_sub_ids',[]))),
                tuple(sorted(x for x in (v.get('parent_link'),v.get('child_link')) if x)))
    before_ids={v['violation_id'] for v in before.get('violations',[])}
    after_ids={v['violation_id'] for v in after.get('violations',[])}
    before_signatures={signature(v) for v in before.get('violations',[])}
    after_signatures={signature(v) for v in after.get('violations',[])}
    targets=set(candidate.target.get('target_violation_ids',[]))
    new_ids=after_ids-before_ids;new_signatures=after_signatures-before_signatures
    before_overlap=float(before.get('aggregate_overlap',0.0));after_overlap=float(after.get('aggregate_overlap',0.0))
    accepted=(bool(targets) and not (targets & after_ids) and not new_signatures and
              after_overlap < before_overlap-1e-9 and after.get('ok',False))
    return {'accepted':accepted,'targets_cleared':sorted(targets-after_ids),
            'targets_remaining':sorted(targets&after_ids),'new_violation_ids':sorted(new_ids),
            'new_violation_signatures':[list(x) for x in sorted(new_signatures,key=str)],
            'before_overlap':before_overlap,'after_overlap':after_overlap,
            'strictly_improved':after_overlap < before_overlap-1e-9,
            'fully_clean':bool(after.get('ok',False))}


def generate_repair_candidates(report, plan, subs, settings):
    out=[]
    pose_allowed=(report.get('diagnostics',{}).get('failure_kind')=='gear_face_overlap'
                  and getattr(settings,'enable_solver_pose_repair',True))
    for p in report.get('gear_pairs',[]):
        c=_pose_candidate(p,subs,allowed=pose_allowed and
                          float(p.get('face_overlap_mm',1.0)) <=
                          float(p.get('minimum_face_overlap_mm',0.1)))
        if c:out.append(c)
    c=_multibore_candidate(report,subs)
    if c and getattr(settings,'enable_solver_seat_geometry_repair',True):out.append(c)
    return sorted(out,key=lambda c:(not c.allowed,max(c.predicted_residuals_mm.values() or [1e9]),c.candidate_id))

class LocalRepairTransaction:
    """Byte-for-byte disk + in-memory snapshot for the candidate's affected subs."""
    def __init__(self, subs, sub_ids):
        self.subs=subs; self.ids=set(sub_ids); self.mem={}; self.tmp=tempfile.mkdtemp(prefix='maker2_local_repair_')
    def __enter__(self):
        for sid in self.ids:
            r=self.subs[sid]; self.mem[sid]=(copy.deepcopy(r.model),copy.deepcopy(r.sub_frames),copy.deepcopy(r.results),r.ok,r.error)
            if r.ctx and os.path.isdir(r.ctx.run_dir): shutil.copytree(r.ctx.run_dir,os.path.join(self.tmp,sid))
        return self
    def rollback(self):
        for sid,(m,f,rs,ok,err) in self.mem.items():
            r=self.subs[sid];r.model,r.sub_frames,r.results,r.ok,r.error=m,f,rs,ok,err
            src=os.path.join(self.tmp,sid)
            if r.ctx and os.path.isdir(src):
                shutil.rmtree(r.ctx.run_dir,ignore_errors=True);shutil.copytree(src,r.ctx.run_dir)
    def __exit__(self,typ,val,tb):
        if typ:self.rollback()
        shutil.rmtree(self.tmp,ignore_errors=True)
        return False


def apply_candidate(candidate, subs, settings, log_fn=print):
    """Apply one Python-owned candidate and refresh only its declared local scope."""
    from .urdf_builder import build_urdf, validate_urdf
    if not candidate.allowed:
        raise ValueError('candidate is blocked')
    if candidate.candidate_type=='gear_pose_axial_alignment':
        res=subs[candidate.target['sub_id']]
        p=next(x for x in res.model.poses if x.name==candidate.target['pose'])
        p.xyz_m=tuple(candidate.new_value)
        save_model(res.model,res.ctx.model_json_path)
        build_urdf(res.model,res.ctx)
        ok,err=validate_urdf(res.ctx.urdf_path,require_meshes=True)
        if not ok: raise RuntimeError(f'pose patch URDF invalid: {err}')
        return {res.id}
    if candidate.candidate_type=='housing_multibore_pattern':
        from .cq_worker import rebuild_link
        res=subs[candidate.target['sub_id']]
        centers_by_link=candidate.target.get('centers_by_link') or {}
        # Replace the plate's complete bore operation chain with one explicit three-point
        # pattern, preserving the original cutter call and all cutter dimensions/depths.
        for path in candidate.target['scripts']:
            link_name=os.path.splitext(os.path.basename(path))[0]
            role_values=centers_by_link.get(link_name)
            text=open(path,encoding='utf-8').read(); parsed=_parse_bores(text)
            if not parsed or not role_values:raise RuntimeError(f'unsupported bore script {path}')
            tree=ast.parse(text); statements=[]
            for n in ast.walk(tree):
                if not isinstance(n,(ast.Assign,ast.AnnAssign,ast.Expr)):continue
                value=n.value if hasattr(n,'value') else None
                src=ast.get_source_segment(text,n) or ''
                if value is not None and '.pushPoints(' in src and any(
                        f'.{name}(' in src for name in ('hole','cboreHole','cskHole')):
                    statements.append((n,src))
            if not statements:raise RuntimeError(f'no machine-editable bore cutter in {path}')
            points=[tuple(role_values[r]) for r in ('input','inter','output')]
            statement,src=statements[0]
            push=src.find('.pushPoints(');open_i=src.find('(',push);depth=0;close_i=-1
            for i in range(open_i,len(src)):
                if src[i]=='(':depth+=1
                elif src[i]==')':
                    depth-=1
                    if depth==0:close_i=i;break
            if close_i<0:raise RuntimeError(f'unsupported pushPoints expression in {path}')
            replacement=src[:open_i+1]+repr(points)+src[close_i:]
            lines=text.splitlines();start=statements[0][0].lineno-1;end=statements[-1][0].end_lineno
            indent=lines[start][:len(lines[start])-len(lines[start].lstrip())]
            lines[start:end]=[indent+replacement.lstrip()]
            open(path,'w',encoding='utf-8').write('\n'.join(lines)+'\n')
        for name in candidate.target['physical_links']:
            link=next(l for l in res.model.links if l.name==name)
            qpath=os.path.join(res.ctx.run_dir,'cq',f'{name}.py');qtext=open(qpath,encoding='utf-8').read()
            rebuilt=rebuild_link(link,qtext,res.ctx,res.ctx.run_dir,log_fn=log_fn)
            if not rebuilt.success:raise RuntimeError(f'housing link {name} rebuild failed: {rebuilt.error}')
        # Synchronize all six realized frames from the same centers used by the cutters.
        frame_centers=candidate.target.get('frame_centers',{})
        frame_links={x['frame']:x['link'] for x in candidate.target.get('frame_bore_mapping',[])}
        changed_frames=set()
        for e in res.sub_frames:
            frame=e.get('frame')
            if frame in frame_centers:
                xyz=list(e.get('local_xyz_m',[0,0,0]));xyz[1:3]=[float(x)/1000. for x in frame_centers[frame]];e['local_xyz_m']=xyz
                e['link']=frame_links[frame]
                changed_frames.add(frame)
        if changed_frames != set(frame_centers):
            raise RuntimeError('not every mapped seat frame was synchronized')
        with open(os.path.join(res.ctx.run_dir,'sub_frames.json'),'w',encoding='utf-8') as f:json.dump(res.sub_frames,f,indent=2)
        build_urdf(res.model,res.ctx);ok,err=validate_urdf(res.ctx.urdf_path,require_meshes=True)
        if not ok:raise RuntimeError(f'multibore patch URDF invalid: {err}')
        return {res.id}
    raise ValueError('unsupported candidate')
