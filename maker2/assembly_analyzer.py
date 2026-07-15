"""Self-directed, read-only assembly failure analyzer with sandboxed artifact/source tools."""
from __future__ import annotations
import glob, json, os, re
from pathlib import Path
from .jsonutil import extract_json_object
from .llm.conversation import Conversation
from .prompts.assembly_analyzer_prompt import ANALYZER_SYSTEM
from .tools import KB_SEARCH_TOOL, _kb_bound, run_tool_loop

MAX_FILE=512*1024; MAX_LINES=400; MAX_RESULT=32*1024; MAX_MATCHES=60; MAX_GLOB=200
_DENY=re.compile(r'(^|[\\/])(\.git|__pycache__)([\\/]|$)|(^|[_.-])(env|token|secret|credential)|\.(pem|key|pyc)$',re.I)

def _tool(name,desc,props,required=()):
 return {'type':'function','function':{'name':name,'description':desc,'parameters':{'type':'object','properties':props,'required':list(required),'additionalProperties':False}}}
def _cap(s):
 s=str(s); return s if len(s)<=MAX_RESULT else s[:MAX_RESULT]+f'\n...(truncated {len(s)-MAX_RESULT} chars)'
def _resolver(root,source=False):
 root=os.path.realpath(root)
 def r(rel):
  if not isinstance(rel,str) or os.path.isabs(rel) or '..' in Path(rel).parts or rel.startswith(('~','\\','/')) or re.match(r'^[A-Za-z]:',rel):raise ValueError('unsafe path')
  p=os.path.realpath(os.path.join(root,rel));
  if os.path.commonpath([p,root])!=root or _DENY.search(rel):raise ValueError('path outside allowed root or denied')
  if source and not p.lower().endswith(('.py','.md')):raise ValueError('source reads limited to .py/.md')
  return p
 return r

def analyze_failure(session_root,report,candidates,settings,log_fn=print,report_path=""):
 ar=_resolver(session_root); sr=_resolver(os.path.dirname(__file__),True); calls=[]; seen=[]; cmap={c.candidate_id:c for c in candidates}
 def rec(name,fn):
  def w(**kw):
   try: out=_cap(fn(**kw)); err=None
   except Exception as e: out=f'(error: {type(e).__name__}: {e})';err=str(e)
   calls.append({'tool':name,'args':kw,'result':out,'error':err}); return out
  return w
 def list_artifacts(pattern='**/*',limit=100):
  files=[]
  for p in glob.glob(os.path.join(session_root,pattern),recursive=True):
   if os.path.isfile(p):
    rel=os.path.relpath(p,session_root); ar(rel);files.append(rel)
   if len(files)>=min(limit,MAX_GLOB):break
  return json.dumps(files)
 def read_text(path,offset=0,limit=200,source=False):
  p=(sr if source else ar)(path); size=os.path.getsize(p)
  if size>MAX_FILE:raise ValueError('file too large')
  ls=open(p,encoding='utf-8',errors='replace').read().splitlines(); chunk=ls[max(0,offset):max(0,offset)+min(limit,MAX_LINES)]
  seen.append((source,path,max(1,offset+1),offset+len(chunk))); return '\n'.join(f'{offset+i+1}\t{x}' for i,x in enumerate(chunk))
 def read_json(path,pointer=''):
  p=ar(path); obj=json.load(open(p,encoding='utf-8'))
  if pointer:
   for part in pointer.strip('/').split('/'): obj=obj[int(part)] if isinstance(obj,list) else obj[part.replace('~1','/').replace('~0','~')]
  seen.append((False,path,1,10**9));return json.dumps(obj,indent=2)
 def search_log(regex,context=2,max_matches=30):
  if len(regex)>200:raise ValueError('regex too long')
  rx=re.compile(regex);ls=open(ar('run.log'),encoding='utf-8',errors='replace').read().splitlines();out=[]
  for i,x in enumerate(ls):
   if rx.search(x):
    a=max(0,i-min(context,10));b=min(len(ls),i+min(context,10)+1);out.extend(f'{j+1}\t{ls[j]}' for j in range(a,b));seen.append((False,'run.log',a+1,b))
    if len(out)>=min(max_matches,MAX_MATCHES):break
  return '\n'.join(out)
 def search_files(scope,regex,glob_pattern='**/*.py',max_matches=30):
  root=os.path.dirname(__file__) if scope=='source' else session_root; resolve=sr if scope=='source' else ar;rx=re.compile(regex);out=[]
  for p in glob.glob(os.path.join(root,glob_pattern),recursive=True):
   if not os.path.isfile(p) or os.path.getsize(p)>MAX_FILE:continue
   rel=os.path.relpath(p,root);resolve(rel)
   for i,x in enumerate(open(p,encoding='utf-8',errors='replace')):
    if rx.search(x):out.append(f'{rel}:{i+1}: {x.rstrip()}');seen.append((scope=='source',rel,i+1,i+1))
    if len(out)>=min(max_matches,MAX_MATCHES):return '\n'.join(out)
  return '\n'.join(out)
 def list_candidates():return json.dumps([c.to_dict() for c in candidates],indent=2)
 def simulate_candidate(candidate_id):
  c=cmap.get(candidate_id);return json.dumps(c.to_dict() if c else {'error':'unknown candidate'},indent=2)
 tools=[_tool('list_artifacts','List current run artifacts',{'pattern':{'type':'string'},'limit':{'type':'integer'}}),
 _tool('read_json','Read JSON artifact or pointer',{'path':{'type':'string'},'pointer':{'type':'string'}},['path']),
 _tool('read_text','Read artifact text lines',{'path':{'type':'string'},'offset':{'type':'integer'},'limit':{'type':'integer'}},['path']),
 _tool('search_log','Regex search run.log',{'regex':{'type':'string'},'context':{'type':'integer'},'max_matches':{'type':'integer'}},['regex']),
 _tool('search_files','Search artifact/source files',{'scope':{'type':'string','enum':['artifacts','source']},'regex':{'type':'string'},'glob_pattern':{'type':'string'},'max_matches':{'type':'integer'}},['scope','regex']),
 _tool('read_source','Read maker2 source lines',{'path':{'type':'string'},'offset':{'type':'integer'},'limit':{'type':'integer'}},['path']),
 _tool('list_repair_candidates','List Python-generated candidates',{}),_tool('simulate_candidate','Inspect a candidate',{'candidate_id':{'type':'string','enum':list(cmap)}},['candidate_id']),KB_SEARCH_TOOL]
 ex={'list_artifacts':rec('list_artifacts',list_artifacts),'read_json':rec('read_json',read_json),'read_text':rec('read_text',read_text),'search_log':rec('search_log',search_log),'search_files':rec('search_files',search_files),'read_source':rec('read_source',lambda path,offset=0,limit=200:read_text(path,offset,limit,True)),'list_repair_candidates':rec('list_repair_candidates',lambda:list_candidates()),'simulate_candidate':rec('simulate_candidate',simulate_candidate),'kb_search':rec('kb_search',_kb_bound('analyzer'))}
 rel_report=(os.path.relpath(report_path,session_root) if report_path else 'assembly_constraint_report.json')
 conv=Conversation();conv.add_user_message(f"Failure {report.get('failure_id')}. Report path: {rel_report}. Investigate current session with tools before deciding.")
 client=settings.make_client(getattr(settings,'analyzer_max_tokens',16000),thinking='extended');text=run_tool_loop(client,conv,ANALYZER_SYSTEM,tools,ex,max_rounds=getattr(settings,'solver_analyzer_max_rounds',12),log_fn=log_fn,text_only_nudge='Call a read/search tool now; do not answer before investigating.')
 # Tool budget may end on a preamble/tool call. Force one final no-tools synthesis turn.
 try:
  conv.add_user_message('Investigation is over. Return the required final JSON object now; no prose and no more tools.')
  final,_=client.send_collect(conv.get_messages_for_api(api_style=client.api_style),system=ANALYZER_SYSTEM)
  text=final or text
 except Exception: pass
 try:decision=json.loads(extract_json_object(text))
 except Exception:decision={'failure_id':report.get('failure_id'),'decision':'escalate','classification':'topology','root_cause':'Analyzer returned invalid JSON','layer':'boss_topology','culprits':[],'evidence':[],'selected_candidate_id':None,'confidence':'low','explanation':text,'escalation_reason':'invalid_output'}
 cid=decision.get('selected_candidate_id');
 if decision.get('decision')=='repair' and (cid not in cmap or not cmap[cid].allowed):decision['decision']='escalate';decision['selected_candidate_id']=None;decision['escalation_reason']='invalid_or_blocked_candidate'
 trace={'failure_id':report.get('failure_id'),'calls':calls,'seen':seen,'decision':decision};json.dump(trace,open(os.path.join(session_root,'analyzer_trace.json'),'w',encoding='utf-8'),indent=2)
 return decision
