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

def _pose_candidate(pair, subs):
    sid, part = pair['child_sub'], pair['child_part']; res=subs.get(sid)
    poses=[p for p in (res.model.poses if res and res.model else []) if p.child==part]
    if len(poses)!=1: return None
    p=poses[0]; old=np.asarray(p.xyz_m,float); axis=np.asarray(res.model.links[[l.name for l in res.model.links].index(part)].spin_axis,float)
    axis=axis/(np.linalg.norm(axis) or 1); axial=pair['axial_delta_mm']/1000.0
    new=old-axial*axis
    predicted=pair['absolute_residual_mm']
    data={'type':'gear_pose_axial_alignment','sub':sid,'part':part,'pose':p.name,'new':new.tolist()}
    return RepairCandidate('pose_'+_cid(data),'gear_pose_axial_alignment',
      {'sub_id':sid,'part':part,'pose':p.name},old.tolist(),new.tolist(),
      {pair['seam_id']:predicted},'model_pose_patch',{'subassemblies':[sid],'links':[]},
      False,"Axial face skew does not alter spur pitch-center distance; diagnostic only")

def _role_for_name(name):
    n=(name or '').upper()
    if 'INPUT' in n: return 'input'
    if 'INTER' in n or 'MIDDLE' in n: return 'inter'
    if 'OUTPUT' in n: return 'output'
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
    # Vectorized CadQuery idiom: pushPoints([(Y_INPUT,0), ...]) or [(0,Y_INPUT), ...].
    for n in ast.walk(tree):
        if not isinstance(n,ast.Call) or not isinstance(n.func,ast.Attribute) or n.func.attr!='pushPoints' or not n.args: continue
        seq=n.args[0]
        if not isinstance(seq,(ast.List,ast.Tuple)): continue
        vals=[];names=[];roles=[];components=[];ok=True
        for item in seq.elts:
            if not isinstance(item,(ast.Tuple,ast.List)): ok=False;break
            found=None
            for comp,e0 in enumerate(item.elts):
                e=e0; sign=1.0
                if isinstance(e,ast.UnaryOp) and isinstance(e.op,ast.USub): sign=-1.0;e=e.operand
                if isinstance(e,ast.Name) and e.id in constants:
                    found=(sign*constants[e.id],e.id,_role_for_name(e.id),comp);break
            if found is None: ok=False;break
            v,name,role,comp=found;vals.append(v);names.append(name);roles.append(role);components.append(comp)
        if ok and len(vals)>=2:
            return {'mode':'pushpoints','names':names,'roles':roles,'components':components,'lines':lines,
                    'push_line_start':n.lineno,'push_line_end':n.end_lineno,'bores':[[v] for v in vals],
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
    # Aggregate repeated single-point pushPoints calls (common back-wall style).
    agg=[]
    for n in ast.walk(tree):
        if not isinstance(n,ast.Call) or not isinstance(n.func,ast.Attribute) or n.func.attr!='pushPoints' or not n.args: continue
        seq=n.args[0]
        if not isinstance(seq,(ast.List,ast.Tuple)) or len(seq.elts)!=1: continue
        item=seq.elts[0]
        if not isinstance(item,(ast.Tuple,ast.List)) or not item.elts: continue
        e=item.elts[0]; sign=1.0
        if isinstance(e,ast.UnaryOp) and isinstance(e.op,ast.USub): sign=-1.0;e=e.operand
        if isinstance(e,ast.Name) and e.id in constants:
            agg.append((n,sign*constants[e.id],e.id,_role_for_name(e.id)))
    if len(agg)>=2:
        return {'mode':'pushpoints_multi','names':[x[2] for x in agg],'roles':[x[3] for x in agg],
                'lines':lines,'calls':[(x[0].lineno,x[0].end_lineno) for x in agg],
                'bores':[[x[1]] for x in agg],
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

def generate_repair_candidates(report, plan, subs, settings):
    out=[]
    for p in report.get('gear_pairs',[]):
        c=_pose_candidate(p,subs)
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
        centers=candidate.new_value
        # Rewrite semantic seat constants in each complete physical-bore script.
        for path in candidate.target['scripts']:
            text=open(path,encoding='utf-8').read(); parsed=_parse_bores(text)
            if not parsed:continue
            lines=text.splitlines()
            role_values=centers
            if parsed['mode'] in ('constants','pushpoints','pushpoints_multi','centers_multi'):
                roles=list(parsed['roles'])
                if len(roles)==2 and 'inter' not in roles: roles=['input','output']
                for role,name in zip(roles,parsed['names']):
                    if name and role in role_values:
                        i=parsed['lines'][name][0]-1
                        indent=lines[i][:len(lines[i])-len(lines[i].lstrip())]
                        lines[i]=f'{indent}{name} = {float(role_values[role])!r}'
                if parsed['mode']=='constants':
                    loop_i=parsed['loop_line_start']-1;loop=lines[loop_i]
                    for role,name in zip(roles,parsed['names']):
                        if name is None and role in role_values:
                            loop=re.sub(r'\b[-+]?\d+(?:\.\d+)?\b',str(float(role_values[role])),loop,count=1)
                    lines[loop_i]=loop
            elif parsed['mode']=='literal':
                rows=[]
                for role,row in zip(parsed['roles'],parsed['bores']): rows.append(tuple([role_values[role]]+row[1:]))
                a,b=parsed['line_start']-1,parsed['line_end'];lines[a:b]=['bores = '+repr(rows)]
            open(path,'w',encoding='utf-8').write('\n'.join(lines)+'\n')
        for name in candidate.target['physical_links']:
            link=next(l for l in res.model.links if l.name==name)
            qpath=os.path.join(res.ctx.run_dir,'cq',f'{name}.py');qtext=open(qpath,encoding='utf-8').read()
            rebuilt=rebuild_link(link,qtext,res.ctx,res.ctx.run_dir,log_fn=log_fn)
            if not rebuilt.success:raise RuntimeError(f'housing link {name} rebuild failed: {rebuilt.error}')
        # Synchronize every top/bottom frame by semantic role; preserve its other coordinates.
        for e in res.sub_frames:
            role=_role_for_name(e.get('frame'))
            if role in centers:
                xyz=list(e.get('local_xyz_m',[0,0,0]));xyz[1]=float(centers[role])/1000.;e['local_xyz_m']=xyz
        with open(os.path.join(res.ctx.run_dir,'sub_frames.json'),'w',encoding='utf-8') as f:json.dump(res.sub_frames,f,indent=2)
        build_urdf(res.model,res.ctx);ok,err=validate_urdf(res.ctx.urdf_path,require_meshes=True)
        if not ok:raise RuntimeError(f'multibore patch URDF invalid: {err}')
        return {res.id}
    raise ValueError('unsupported candidate')
