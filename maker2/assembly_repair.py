"""Deterministic candidates and bounded local repair for authoritative-solver failures."""
from __future__ import annotations
import ast, copy, hashlib, json, math, os, re, shutil, tempfile
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

def _parse_bores(script):
    tree=ast.parse(script)
    for n in ast.walk(tree):
        if isinstance(n,(ast.Assign,ast.AnnAssign)):
            targets=n.targets if isinstance(n,ast.Assign) else [n.target]; value=n.value
            if any(isinstance(t,ast.Name) and t.id=='bores' for t in targets):
                try:v=ast.literal_eval(value)
                except Exception:return None
                if isinstance(v,list) and all(isinstance(x,(tuple,list)) and len(x)>=2 for x in v):
                    return n.lineno,n.end_lineno,[list(x) for x in v]
    return None

def _multibore_candidate(report, subs):
    base=report.get('base_id'); res=subs.get(base)
    if not res or not res.ctx:return None
    frame_entries=[e for e in res.sub_frames or [] if str(e.get('frame','')).startswith('seat_')]
    if len(frame_entries)<2:return None
    links={e.get('link') for e in frame_entries}
    if len(links)!=1:return None
    link=next(iter(links)); path=os.path.join(res.ctx.run_dir,'cq',f'{link}.py')
    if not os.path.isfile(path):return None
    text=open(path,encoding='utf-8').read(); parsed=_parse_bores(text)
    if not parsed:return None
    l0,l1,bores=parsed
    pairs=report.get('gear_pairs',[])
    if len(bores)!=len(pairs)+1:return None
    centers=[float(bores[0][0])]
    for pair in pairs:
        # Parallel shaft-line spacing equals the pitch-center distance. Gear face
        # axial offsets affect overlap, not the radial distance between shafts.
        centers.append(centers[-1]-pair['required_distance_mm'])
    new=[[centers[i]]+bores[i][1:] for i in range(len(bores))]
    pred={p['seam_id']:0.0 for p in pairs}
    data={'type':'housing_multibore_pattern','sub':base,'link':link,'centers':centers}
    return RepairCandidate('bores_'+_cid(data),'housing_multibore_pattern',
      {'sub_id':base,'part':link,'script':path,'line_start':l0,'line_end':l1,
       'frames':[e['frame'] for e in frame_entries]},bores,new,pred,'housing_multibore_patch',
      {'subassemblies':[base],'links':[link]},True,
      'Pin first physical bore; recompute remaining centers from locked gear distances and axial offsets')

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
        path=candidate.target['script']
        lines=open(path,encoding='utf-8').read().splitlines()
        repl='bores = '+repr([tuple(x) for x in candidate.new_value])
        a,b=candidate.target['line_start']-1,candidate.target['line_end']
        lines[a:b]=[repl]
        link=next(l for l in res.model.links if l.name==candidate.target['part'])
        rebuilt=rebuild_link(link,'\n'.join(lines)+'\n',res.ctx,res.ctx.run_dir,log_fn=log_fn)
        if not rebuilt.success: raise RuntimeError(f'housing link rebuild failed: {rebuilt.error}')
        ordered=[e for name in candidate.target['frames'] for e in res.sub_frames if e.get('frame')==name]
        for e,x in zip(ordered,candidate.new_value):
            xyz=list(e.get('local_xyz_m',[0,0,0]));xyz[1]=float(x[0])/1000.;e['local_xyz_m']=xyz
        with open(os.path.join(res.ctx.run_dir,'sub_frames.json'),'w',encoding='utf-8') as f:
            json.dump(res.sub_frames,f,indent=2)
        build_urdf(res.model,res.ctx)
        ok,err=validate_urdf(res.ctx.urdf_path,require_meshes=True)
        if not ok: raise RuntimeError(f'multibore patch URDF invalid: {err}')
        return {res.id}
    raise ValueError('unsupported candidate')
