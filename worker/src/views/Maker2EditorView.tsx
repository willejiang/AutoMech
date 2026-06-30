import { useEffect, useMemo, useRef, useState } from 'react';
import { useNavigate } from '@tanstack/react-router';
import { ArrowLeft } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { ScrollArea } from '@/components/ui/scroll-area';
import { StreamingCodeBlock } from '@/components/chat/StreamingCodeBlock';
import { Maker2ModelCanvas } from '@/components/viewer/Maker2ModelCanvas';
import { apiUrl } from '@/services/api';
import { cn } from '@/lib/utils';

// One maker2 run, driven entirely by the SSE stream from /api/run-maker2-stream.
// Left = live stage progress + the generated .scad; right = the assembled colored
// model (orbitable). Ephemeral: no Supabase row, no history — the run lives only
// as long as this page is open (navigating away aborts the Python process).

interface Maker2EditorViewProps {
  prompt: string;
  model: string;
  iters: number;
}

type Maker2Result = {
  ok: boolean;
  run_dir?: string;
  links?: number;
  joints?: number;
  movable_joints?: number;
  built?: number;
  iterations?: number;
  judge?: { passed: boolean | null; reasons?: string } | null;
  physics?: { passed: boolean | null; summary?: string } | null;
  error?: string;
};

// The four pipeline stages, in order. We light each one up by matching the
// stdout marker maker2 prints (see run-maker2-stream STAGE_PREFIXES).
const STAGES = [
  { key: 'manager', label: 'Manager — decompose into links + joints',
    match: (l: string) => l.startsWith('[1/3]') },
  { key: 'worker', label: 'CAD worker — generate SCAD + render STLs',
    match: (l: string) => l.startsWith('[2/3]') },
  { key: 'judge', label: 'Judge — review the assembled model',
    match: (l: string) => l.startsWith('[judge]') || l.startsWith('[3/3]') },
  { key: 'physics', label: 'Physics — PyBullet stability test',
    match: (l: string) => l.startsWith('[physics]') },
] as const;

type StageState = 'pending' | 'active' | 'done';

export function Maker2EditorView({ prompt, model, iters }: Maker2EditorViewProps) {
  const navigate = useNavigate();
  const [lines, setLines] = useState<string[]>([]);
  const [result, setResult] = useState<Maker2Result | null>(null);
  const [errorMsg, setErrorMsg] = useState<string | null>(null);
  const [glbBlob, setGlbBlob] = useState<Blob | undefined>(undefined);
  const [scad, setScad] = useState<string>('');
  const [iteration, setIteration] = useState(1);
  const logEndRef = useRef<HTMLDivElement>(null);
  const startedRef = useRef(false);

  // Open the SSE stream exactly once for this run.
  useEffect(() => {
    if (startedRef.current) return;
    startedRef.current = true;

    const qs = new URLSearchParams({ prompt, iters: String(iters) });
    if (model) qs.set('model', model);
    const es = new EventSource(`${apiUrl('run-maker2-stream')}?${qs.toString()}`);

    const pushLine = (raw: string) => {
      setLines((prev) => [...prev, raw]);
      const m = raw.match(/ITERATION\s+(\d+)/);
      if (m) setIteration(Number(m[1]));
    };

    es.addEventListener('stage', (e) =>
      pushLine(JSON.parse((e as MessageEvent).data).raw));
    es.addEventListener('log', (e) =>
      pushLine(JSON.parse((e as MessageEvent).data).raw));

    es.addEventListener('result', (e) => {
      const r = JSON.parse((e as MessageEvent).data) as Maker2Result;
      setResult(r);
      if (r.run_dir) {
        const dir = encodeURIComponent(r.run_dir);
        fetch(`${apiUrl('run-maker2-glb')}?dir=${dir}`)
          .then((res) => (res.ok ? res.blob() : Promise.reject(res.statusText)))
          .then(setGlbBlob)
          .catch(() => {/* canvas keeps its spinner; verdict still shows */});
        fetch(`${apiUrl('run-maker2-glb')}?dir=${dir}&file=scad`)
          .then((res) => (res.ok ? res.text() : ''))
          .then(setScad)
          .catch(() => {});
      }
    });

    es.addEventListener('error', (e) => {
      // Our server-sent {error} payload (distinct from a transport error, which
      // has no data).
      const data = (e as MessageEvent).data;
      if (data) {
        try {
          const d = JSON.parse(data);
          setErrorMsg(d.error + (d.tail ? `\n${d.tail}` : ''));
        } catch { setErrorMsg('stream error'); }
      }
    });

    es.addEventListener('end', () => es.close());

    return () => es.close();
  }, [prompt, model, iters]);

  // Keep the log pinned to the newest line.
  useEffect(() => {
    logEndRef.current?.scrollIntoView({ block: 'end' });
  }, [lines]);

  // Derive each stage's state from the running log (and the final result).
  const stageStates = useMemo<Record<string, StageState>>(() => {
    const seen = new Set<string>();
    for (const line of lines) {
      for (const s of STAGES) if (s.match(line)) seen.add(s.key);
    }
    const states: Record<string, StageState> = {};
    const lastSeenIdx = Math.max(
      -1,
      ...STAGES.map((s, i) => (seen.has(s.key) ? i : -1)),
    );
    STAGES.forEach((s, i) => {
      if (result) states[s.key] = seen.has(s.key) ? 'done' : 'pending';
      else if (i < lastSeenIdx) states[s.key] = 'done';
      else if (i === lastSeenIdx) states[s.key] = 'active';
      else states[s.key] = 'pending';
    });
    return states;
  }, [lines, result]);

  const running = !result && !errorMsg;
  const verdictPass = result?.ok && result?.judge?.passed !== false;

  return (
    <div className="flex h-full min-h-full w-full flex-col bg-adam-bg-secondary-dark text-adam-text-primary">
      {/* Header */}
      <div className="flex items-center gap-3 border-b border-adam-neutral-800 px-4 py-3">
        <Button variant="ghost" size="sm" onClick={() => navigate({ to: '/' })}>
          <ArrowLeft className="mr-1 h-4 w-4" /> Back
        </Button>
        <div className="min-w-0 flex-1">
          <div className="truncate text-sm font-medium">{prompt}</div>
          <div className="text-xs text-adam-neutral-400">
            articulated (maker2){model ? ` · ${model}` : ''} · max {iters} iter
          </div>
        </div>
        {result && (
          <span
            className={cn(
              'rounded-full px-3 py-1 text-xs font-semibold',
              verdictPass ? 'bg-green-900/40 text-green-300'
                : 'bg-red-900/40 text-red-300',
            )}
          >
            {verdictPass ? 'PASS' : 'FAIL'}
          </span>
        )}
      </div>

      {/* Split body */}
      <div className="flex min-h-0 flex-1">
        {/* LEFT: stages + generated SCAD */}
        <div className="flex w-2/5 min-w-[360px] flex-col border-r border-adam-neutral-800">
          <div className="border-b border-adam-neutral-800 p-4">
            <div className="mb-2 text-xs font-semibold uppercase tracking-wide text-adam-neutral-400">
              Pipeline {running && `· iteration ${iteration}`}
            </div>
            <ul className="space-y-1.5">
              {STAGES.map((s) => {
                const st = stageStates[s.key];
                return (
                  <li key={s.key} className="flex items-center gap-2 text-sm">
                    <StageDot state={st} />
                    <span
                      className={cn(
                        st === 'pending' && 'text-adam-neutral-500',
                        st === 'active' && 'text-adam-text-primary',
                        st === 'done' && 'text-adam-neutral-300',
                      )}
                    >
                      {s.label}
                    </span>
                  </li>
                );
              })}
            </ul>
            {result && (
              <div className="mt-3 space-y-1 text-xs text-adam-neutral-300">
                <div>
                  Built {result.built}/{result.links} links ·{' '}
                  {result.movable_joints} movable joints ·{' '}
                  {result.iterations} iteration
                  {(result.iterations ?? 1) > 1 ? 's' : ''}
                </div>
                {result.judge && (
                  <div>
                    <span className="font-semibold">
                      Judge: {result.judge.passed === false ? 'FAIL' : 'PASS'}
                    </span>
                    {result.judge.reasons ? ` — ${result.judge.reasons}` : ''}
                  </div>
                )}
                {result.physics && (
                  <div>
                    <span className="font-semibold">
                      Physics: {result.physics.passed ? 'PASS' : 'FAIL'}
                    </span>
                    {result.physics.summary ? ` — ${result.physics.summary}` : ''}
                  </div>
                )}
              </div>
            )}
            {errorMsg && (
              <pre className="mt-3 whitespace-pre-wrap rounded border border-red-800 bg-red-900/20 p-2 text-xs text-red-300">
                {errorMsg}
              </pre>
            )}
          </div>

          {/* Generated SCAD (typewriter) once available; raw log while running */}
          <div className="min-h-0 flex-1">
            {scad ? (
              <StreamingCodeBlock code={scad} isStreaming={false}
                filename="model.scad" />
            ) : (
              <ScrollArea className="h-full">
                <pre className="p-4 font-mono text-xs leading-relaxed text-adam-neutral-400">
                  {lines.join('\n')}
                  <div ref={logEndRef} />
                </pre>
              </ScrollArea>
            )}
          </div>
        </div>

        {/* RIGHT: orbitable colored solid */}
        <div className="min-w-0 flex-1 bg-adam-neutral-950/30">
          <Maker2ModelCanvas glbBlob={glbBlob} />
        </div>
      </div>
    </div>
  );
}

function StageDot({ state }: { state: StageState }) {
  if (state === 'active')
    return (
      <span className="h-3 w-3 shrink-0 animate-spin rounded-full border-2 border-adam-blue border-t-transparent" />
    );
  if (state === 'done')
    return <span className="h-3 w-3 shrink-0 rounded-full bg-green-500" />;
  return <span className="h-3 w-3 shrink-0 rounded-full border border-adam-neutral-600" />;
}
