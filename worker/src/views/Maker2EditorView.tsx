import { useCallback, useEffect, useRef, useState } from 'react';
import { useNavigate } from '@tanstack/react-router';
import { ArrowLeft, Send } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { ScrollArea } from '@/components/ui/scroll-area';
import { StreamingCodeBlock } from '@/components/chat/StreamingCodeBlock';
import { Maker2ModelCanvas } from '@/components/viewer/Maker2ModelCanvas';
import { PhysicsPanel, type PhysicsResult } from '@/components/viewer/PhysicsPanel';
import { apiUrl } from '@/services/api';
import { cn } from '@/lib/utils';

// A maker2 CONVERSATION: an ordered list of turns, each turn = one full pipeline
// run (manager -> worker -> judge -> physics). Turn 1 is the original prompt;
// later turns are refine requests ("make the gears bigger") that re-run with the
// prior model as context. The right canvas always shows the latest model that
// actually rendered; a turn that only judge-FAILs keeps its model, and a hard
// crash falls back to the previous good render. Threads persist on disk.

interface Maker2EditorViewProps {
  prompt: string;
  model: string;
  iters: number;
  threadId: string;             // conversation id (= route $runId)
  // When set, reopen an existing thread from the sidebar (load past turns).
  reopenThread?: boolean;
  // Legacy (pre-thread) run: read-only single view of one run_dir.
  viewDir?: string;
}

type Maker2Result = {
  ok: boolean;
  run_dir?: string;
  render_dir?: string;
  hard_failed?: boolean;
  links?: number;
  movable_joints?: number;
  built?: number;
  iterations?: number;
  judge?: { passed: boolean | null; reasons?: string } | null;
  physics?: PhysicsResult | null;
  error?: string;
};

type SubArtifact = { sub_id: string; run_dir: string; ok: boolean };
type PrecheckArtifact = {
  ok: boolean;
  violations: { kind: string; severity: string; sub_id?: string; detail?: string }[];
};

type Turn = {
  message: string;              // user's prompt / refine request for this turn
  lines: string[];             // live stdout while streaming
  result: Maker2Result | null;
  scad: string;
  glbBlob?: Blob;
  runDir: string;              // this turn's run_dir (for the NEXT refine's prior model)
  streaming: boolean;
  error?: string;
  // Hierarchy (boss mode): per-subassembly builds + the geometric pre-check.
  subs?: SubArtifact[];
  precheck?: PrecheckArtifact;
};

const STAGES = [
  { key: 'boss', label: 'Boss — split into subassemblies',
    match: (l: string) => l.startsWith('[boss]') },
  { key: 'manager', label: 'Manager — decompose into links + joints',
    match: (l: string) => l.startsWith('[1/3]') || l.startsWith('[sub:') },
  { key: 'worker', label: 'CAD worker — generate SCAD + render STLs',
    match: (l: string) => l.startsWith('[2/3]') || l.includes('[worker]') },
  { key: 'assembler', label: 'Assembler — stitch subassemblies',
    match: (l: string) => l.startsWith('[assembler]') },
  { key: 'precheck', label: 'Pre-check — verify geometry fits',
    match: (l: string) => l.startsWith('[precheck]') },
  { key: 'judge', label: 'Judge — review the assembled model',
    match: (l: string) => l.startsWith('[judge]') || l.startsWith('[3/3]') },
  { key: 'physics', label: 'Physics — drive + evaluate',
    match: (l: string) => l.startsWith('[physics]') },
] as const;

type StageState = 'pending' | 'active' | 'done';

const emptyTurn = (message: string): Turn => ({
  message, lines: [], result: null, scad: '', runDir: '', streaming: true,
});

export function Maker2EditorView(props: Maker2EditorViewProps) {
  const { prompt, model, iters, threadId, reopenThread, viewDir } = props;
  const navigate = useNavigate();
  const [turns, setTurns] = useState<Turn[]>([]);
  const [chatInput, setChatInput] = useState('');
  const [loadingThread, setLoadingThread] = useState(!!reopenThread);
  const startedRef = useRef(false);
  const esRef = useRef<EventSource | null>(null);
  const endRef = useRef<HTMLDivElement>(null);

  // Update a turn by index (immutably).
  const patchTurn = useCallback((idx: number, patch: Partial<Turn>) => {
    setTurns((prev) => prev.map((t, i) => (i === idx ? { ...t, ...patch } : t)));
  }, []);

  const loadArtifacts = useCallback((idx: number, dir: string) => {
    const d = encodeURIComponent(dir);
    fetch(`${apiUrl('run-maker2-glb')}?dir=${d}`)
      .then((r) => (r.ok ? r.blob() : Promise.reject(r.statusText)))
      .then((blob) => patchTurn(idx, { glbBlob: blob }))
      .catch(() => {/* leave glb undefined; canvas falls back or shows status */});
    fetch(`${apiUrl('run-maker2-glb')}?dir=${d}&file=scad`)
      .then((r) => (r.ok ? r.text() : ''))
      .then((s) => patchTurn(idx, { scad: s }))
      .catch(() => {});
  }, [patchTurn]);

  // Stream one turn (fresh or refine). `idx` is its position in `turns`.
  const streamTurn = useCallback((idx: number, params: URLSearchParams) => {
    const es = new EventSource(`${apiUrl('run-maker2-stream')}?${params.toString()}`);
    esRef.current = es;

    es.addEventListener('stage', (e) => {
      const raw = JSON.parse((e as MessageEvent).data).raw as string;
      setTurns((prev) => prev.map((t, i) =>
        i === idx ? { ...t, lines: [...t.lines, raw] } : t));
    });
    es.addEventListener('log', (e) => {
      const raw = JSON.parse((e as MessageEvent).data).raw as string;
      setTurns((prev) => prev.map((t, i) =>
        i === idx ? { ...t, lines: [...t.lines, raw] } : t));
    });
    es.addEventListener('result', (e) => {
      const r = JSON.parse((e as MessageEvent).data) as Maker2Result;
      patchTurn(idx, { result: r, runDir: r.run_dir ?? '', streaming: false });
      // Prefer the last-good render dir; fall back to the run dir.
      const dir = r.render_dir || r.run_dir;
      if (dir) loadArtifacts(idx, dir);
    });
    // Per-iteration artifacts: the loop emits these as soon as a model is built
    // (after the judge) or a recording is made (after physics), pass OR fail. Load
    // them into THIS turn right away so the canvas + video always show the newest
    // attempt instead of waiting for the loop to finish.
    es.addEventListener('artifact', (e) => {
      const a = JSON.parse((e as MessageEvent).data) as {
        kind: 'model' | 'physics' | 'subassembly' | 'assembled_model' | 'precheck';
        run_dir?: string; render_dir?: string;
        physics?: PhysicsResult | null;
        sub_id?: string; ok?: boolean;
        violations?: PrecheckArtifact['violations'];
      };
      const dir = a.render_dir || a.run_dir;
      if ((a.kind === 'model' || a.kind === 'assembled_model') && dir) {
        // A renderable model (single-machine iteration OR the assembled hierarchy)
        // -> pull its GLB + SCAD; the canvas jumps to it.
        patchTurn(idx, { runDir: a.run_dir ?? dir });
        loadArtifacts(idx, dir);
      } else if (a.kind === 'subassembly') {
        // Hierarchy: accumulate each subassembly build (clickable in the card).
        setTurns((prev) => prev.map((t, i) => {
          if (i !== idx) return t;
          const sub: SubArtifact = { sub_id: a.sub_id ?? '?',
            run_dir: a.run_dir ?? '', ok: !!a.ok };
          const rest = (t.subs ?? []).filter((s) => s.sub_id !== sub.sub_id);
          return { ...t, subs: [...rest, sub] };
        }));
      } else if (a.kind === 'precheck') {
        patchTurn(idx, { precheck: { ok: !!a.ok, violations: a.violations ?? [] } });
      } else if (a.kind === 'physics' && a.physics) {
        // Fresh recording -> merge physics into the turn's (possibly partial) result
        // and point its render_dir at this iteration so the panel plays this video.
        setTurns((prev) => prev.map((t, i) => {
          if (i !== idx) return t;
          const merged: Maker2Result = {
            ...(t.result ?? { ok: true }),
            physics: a.physics ?? null,
            render_dir: a.render_dir || t.result?.render_dir,
            run_dir: a.run_dir || t.result?.run_dir,
          };
          return { ...t, result: merged };
        }));
      }
    });
    es.addEventListener('error', (e) => {
      const data = (e as MessageEvent).data;
      if (data) {
        try {
          const d = JSON.parse(data);
          patchTurn(idx, { error: d.error + (d.tail ? `\n${d.tail}` : ''), streaming: false });
        } catch { patchTurn(idx, { error: 'stream error', streaming: false }); }
      }
    });
    es.addEventListener('end', () => es.close());
  }, [patchTurn, loadArtifacts]);

  // Reopen an existing thread: load past turns read-only from disk.
  useEffect(() => {
    if (!reopenThread || startedRef.current) return;
    startedRef.current = true;
    fetch(`${apiUrl('maker2-thread')}?id=${encodeURIComponent(threadId)}`)
      .then((r) => (r.ok ? r.json() : Promise.reject(r.statusText)))
      .then((doc: { turns?: Array<{ message: string; run_dir: string; render_dir: string }> }) => {
        const loaded: Turn[] = (doc.turns ?? []).map((t) => ({
          message: t.message, lines: [], result: null, scad: '',
          runDir: t.run_dir, streaming: false,
        }));
        setTurns(loaded);
        setLoadingThread(false);
        // Load each turn's saved result + artifacts (latest last so canvas ends on it).
        (doc.turns ?? []).forEach((t, i) => {
          const rd = t.render_dir || t.run_dir;
          if (t.run_dir) {
            fetch(`${apiUrl('run-maker2-glb')}?dir=${encodeURIComponent(t.run_dir)}&file=result`)
              .then((r) => (r.ok ? r.json() : null))
              .then((res: Maker2Result | null) => { if (res) patchTurn(i, { result: res }); })
              .catch(() => {});
          }
          if (rd) loadArtifacts(i, rd);
        });
      })
      .catch(() => { setLoadingThread(false); });
  }, [reopenThread, threadId, loadArtifacts, patchTurn]);

  // Legacy read-only single run (pre-thread): load one turn from disk.
  useEffect(() => {
    if (!viewDir || reopenThread || startedRef.current) return;
    startedRef.current = true;
    const one: Turn = {
      message: prompt || 'Articulated run', lines: [], result: null,
      scad: '', runDir: viewDir, streaming: false,
    };
    setTurns([one]);
    fetch(`${apiUrl('run-maker2-glb')}?dir=${encodeURIComponent(viewDir)}&file=result`)
      .then((r) => (r.ok ? r.json() : null))
      .then((res: Maker2Result | null) => { if (res) patchTurn(0, { result: res }); })
      .catch(() => {});
    loadArtifacts(0, viewDir);
  }, [viewDir, reopenThread, prompt, loadArtifacts, patchTurn]);

  // Fresh conversation: start turn 1 live from the prompt.
  useEffect(() => {
    if (reopenThread || viewDir || startedRef.current || !prompt) return;
    startedRef.current = true;
    setTurns([emptyTurn(prompt)]);
    const qs = new URLSearchParams({ prompt, iters: String(iters), thread: threadId });
    if (model) qs.set('model', model);
    streamTurn(0, qs);
  }, [reopenThread, viewDir, prompt, model, iters, threadId, streamTurn]);

  // On unmount, close the stream AND reset the start guard so a StrictMode
  // remount re-opens a fresh EventSource (otherwise the double-mount closes the
  // first stream and the guard blocks re-opening -> static, never-updating UI).
  useEffect(() => () => {
    esRef.current?.close();
    startedRef.current = false;
  }, []);

  // Auto-scroll the conversation to the newest content.
  useEffect(() => { endRef.current?.scrollIntoView({ block: 'end' }); }, [turns]);

  // Send a refine turn: rebuild from the previous turn's model + the message.
  const sendRefine = () => {
    const msg = chatInput.trim();
    const prev = turns[turns.length - 1];
    // Legacy read-only runs have no thread to append to -> no refine.
    if (viewDir || !msg || !prev || prev.streaming || !prev.runDir) return;
    setChatInput('');
    const idx = turns.length;
    setTurns((prev2) => [...prev2, emptyTurn(msg)]);
    const qs = new URLSearchParams({
      prompt,                    // keep the original product as base context
      iters: String(iters),
      thread: threadId,
      refine: msg,
      prior: `${prev.runDir}/kinematic_model.json`,
    });
    if (model) qs.set('model', model);
    streamTurn(idx, qs);
  };

  // The model to show in the canvas: the latest turn that has a rendered GLB.
  const latestWithGlb = [...turns].reverse().find((t) => t.glbBlob);
  const lastTurn = turns[turns.length - 1];
  const anyStreaming = turns.some((t) => t.streaming);
  // Hard-failed only if the last turn crashed AND no earlier turn ever rendered.
  const canvasFailed = !anyStreaming && !latestWithGlb && !!lastTurn &&
    (lastTurn.result?.hard_failed || !!lastTurn.error);
  const canvasFailReason = lastTurn?.error
    || lastTurn?.result?.error
    || (lastTurn?.result ? 'The manager or worker did not produce a model.' : undefined);

  // The physics to show: the latest turn that produced a physics result.
  const physicsTurn = [...turns].reverse().find((t) => t.result?.physics);
  const physics = physicsTurn?.result?.physics ?? null;
  const physicsRunDir = physicsTurn?.result?.render_dir || physicsTurn?.runDir || '';

  // Hierarchy: a clicked subassembly overrides the canvas so the user can inspect
  // one sub in isolation before it's assembled. Cleared when a new turn streams.
  const [pickedSub, setPickedSub] = useState<{ id: string; blob: Blob } | null>(null);
  const pickSub = useCallback((sub: SubArtifact) => {
    if (!sub.run_dir) return;
    fetch(`${apiUrl('run-maker2-glb')}?dir=${encodeURIComponent(sub.run_dir)}`)
      .then((r) => (r.ok ? r.blob() : Promise.reject(r.statusText)))
      .then((blob) => setPickedSub({ id: sub.sub_id, blob }))
      .catch(() => {/* leave prior canvas */});
  }, []);

  return (
    <div className="flex h-full min-h-full w-full flex-col bg-adam-bg-secondary-dark text-adam-text-primary">
      {/* Header */}
      <div className="flex items-center gap-3 border-b border-adam-neutral-800 px-4 py-3">
        <Button variant="ghost" size="sm" onClick={() => navigate({ to: '/' })}>
          <ArrowLeft className="mr-1 h-4 w-4" /> Back
        </Button>
        <div className="min-w-0 flex-1">
          <div className="truncate text-sm font-medium">{prompt || turns[0]?.message}</div>
          <div className="text-xs text-adam-neutral-400">
            articulated (maker2){model ? ` · ${model}` : ''} ·{' '}
            {iters > 0 ? `max ${iters} iter` : 'loops until it works'}
            {turns.length > 1 ? ` · ${turns.length} turns` : ''}
          </div>
        </div>
      </div>

      {/* Split body */}
      <div className="flex min-h-0 flex-1">
        {/* LEFT: the conversation (turns) + a refine input */}
        <div className="flex w-2/5 min-w-[380px] flex-col border-r border-adam-neutral-800">
          <ScrollArea className="min-h-0 flex-1">
            <div className="flex flex-col gap-4 p-4">
              {loadingThread && (
                <div className="text-sm text-adam-neutral-400">Loading conversation…</div>
              )}
              {turns.map((t, i) => (
                <TurnCard key={i} turn={t} index={i} onPickSub={pickSub} />
              ))}
              <div ref={endRef} />
            </div>
          </ScrollArea>

          {/* Refine input */}
          <div className="border-t border-adam-neutral-800 p-3">
            <div className="flex items-end gap-2">
              <textarea
                value={chatInput}
                onChange={(e) => setChatInput(e.target.value)}
                onKeyDown={(e) => {
                  if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); sendRefine(); }
                }}
                rows={2}
                placeholder={
                  viewDir ? 'Read-only run (start a new one to refine)'
                    : anyStreaming ? 'Building…'
                      : 'Refine this model, e.g. "make the gears bigger"'
                }
                disabled={!!viewDir || anyStreaming || !lastTurn?.runDir}
                className="min-h-0 flex-1 resize-none rounded border border-adam-neutral-700 bg-adam-neutral-900/40 p-2 text-sm text-adam-text-primary placeholder:text-adam-neutral-500 disabled:opacity-50"
              />
              <Button
                size="sm"
                onClick={sendRefine}
                disabled={!!viewDir || anyStreaming || !chatInput.trim() || !lastTurn?.runDir}
              >
                <Send className="h-4 w-4" />
              </Button>
            </div>
          </div>
        </div>

        {/* CENTER: orbitable colored solid (latest good render) */}
        <div className="relative min-w-0 flex-1 bg-adam-neutral-950/30">
          <Maker2ModelCanvas
            glbBlob={pickedSub?.blob ?? latestWithGlb?.glbBlob}
            status={canvasFailed && !pickedSub ? 'failed' : 'loading'}
            failedReason={canvasFailed && !pickedSub ? canvasFailReason : undefined}
          />
          {pickedSub && (
            <div className="absolute left-3 top-3 flex items-center gap-2 rounded-full bg-adam-neutral-900/80 px-3 py-1 text-xs text-adam-text-primary shadow">
              <span>subassembly: <span className="font-mono">{pickedSub.id}</span></span>
              <button className="text-adam-blue hover:underline"
                onClick={() => setPickedSub(null)}>
                back to machine
              </button>
            </div>
          )}
        </div>

        {/* RIGHT: physics evaluation (breakdown + recorded video), when present */}
        {physics && (
          <div className="w-[340px] shrink-0 border-l border-adam-neutral-800 bg-adam-bg-secondary-dark">
            <PhysicsPanel
              physics={physics}
              runDir={physicsRunDir}
              running={anyStreaming}
            />
          </div>
        )}
      </div>
    </div>
  );
}

// One conversation turn: the user message, its pipeline progress, verdict, .scad.
function TurnCard({ turn, index, onPickSub }: {
  turn: Turn; index: number; onPickSub?: (sub: SubArtifact) => void;
}) {
  const groups = splitIterations(turn.lines);
  const verdictPass = turn.result?.ok && turn.result?.judge?.passed !== false;
  return (
    <div className="rounded-lg border border-adam-neutral-800 bg-adam-neutral-900/30">
      {/* user message */}
      <div className="flex items-start justify-between gap-2 border-b border-adam-neutral-800 px-3 py-2">
        <div className="text-sm">
          <span className="mr-2 text-xs text-adam-neutral-500">
            {index === 0 ? 'Build' : 'Refine'}
          </span>
          {turn.message}
        </div>
        {turn.result && (
          <span className={cn('shrink-0 rounded-full px-2 py-0.5 text-[10px] font-semibold',
            verdictPass ? 'bg-green-900/40 text-green-300' : 'bg-red-900/40 text-red-300')}>
            {verdictPass ? 'PASS' : 'FAIL'}
          </span>
        )}
      </div>

      <div className="px-3 py-2">
        {/* Per-iteration pipeline: the closed loop can rebuild many times inside one
            turn; show EACH manager->worker->judge->physics pass with its verdict so a
            rebuild is visible, not collapsed into one row. */}
        {groups.length > 0 ? (
          <div className="space-y-2">
            {groups.map((g, gi) => {
              // An iteration is settled once a later one started or the run ended.
              const settled = gi < groups.length - 1 || !turn.streaming;
              const stageStates = deriveIterStages(g, settled);
              return (
                <div key={gi} className={cn(groups.length > 1 &&
                  'rounded border border-adam-neutral-800/60 bg-adam-neutral-950/30 p-2')}>
                  {groups.length > 1 && (
                    <div className="mb-1 flex items-center gap-2 text-[11px] font-semibold text-adam-neutral-300">
                      <span>Iteration {g.n + 1}</span>
                      {g.judge && (
                        <span className={g.judge === 'PASS' ? 'text-green-400' : 'text-red-400'}>
                          judge {g.judge}
                        </span>
                      )}
                      {g.physics && (
                        <span className={g.physics === 'PASS' ? 'text-green-400' : 'text-red-400'}>
                          · physics {g.physics}{g.physicsCause ? ` (${g.physicsCause})` : ''}
                        </span>
                      )}
                    </div>
                  )}
                  <ul className="space-y-1">
                    {STAGES.map((s) => {
                      const st = stageStates[s.key];
                      return (
                        <li key={s.key} className="flex items-center gap-2 text-xs">
                          <StageDot state={st} />
                          <span className={cn(
                            st === 'pending' && 'text-adam-neutral-500',
                            st === 'active' && 'text-adam-text-primary',
                            st === 'done' && 'text-adam-neutral-300')}>
                            {s.label}
                          </span>
                        </li>
                      );
                    })}
                  </ul>
                </div>
              );
            })}
          </div>
        ) : (
          // Reopened from disk (no live lines): a result means every stage ran.
          <ul className="space-y-1">
            {STAGES.map((s) => (
              <li key={s.key} className="flex items-center gap-2 text-xs">
                <StageDot state={turn.result ? 'done' : 'pending'} />
                <span className="text-adam-neutral-300">{s.label}</span>
              </li>
            ))}
          </ul>
        )}

        {/* Hierarchy: the subassemblies this turn built (click to inspect one). */}
        {turn.subs && turn.subs.length > 0 && (
          <div className="mt-2">
            <div className="mb-1 text-[11px] font-semibold text-adam-neutral-400">
              Subassemblies ({turn.subs.length})
            </div>
            <div className="flex flex-wrap gap-1">
              {[...turn.subs].sort((a, b) => a.sub_id.localeCompare(b.sub_id)).map((s) => (
                <button
                  key={s.sub_id}
                  onClick={() => onPickSub?.(s)}
                  title="View this subassembly on the canvas"
                  className={cn(
                    'rounded border px-2 py-0.5 font-mono text-[10px] transition-colors',
                    s.ok
                      ? 'border-adam-neutral-700 bg-adam-neutral-900/40 text-adam-neutral-200 hover:border-adam-blue hover:text-adam-blue'
                      : 'border-red-800 bg-red-900/20 text-red-300 hover:border-red-500')}
                >
                  {s.ok ? '' : '✗ '}{s.sub_id}
                </button>
              ))}
            </div>
          </div>
        )}

        {/* Hierarchy: geometric pre-check result before physics. */}
        {turn.precheck && (
          <div className={cn('mt-2 rounded border px-2 py-1 text-[11px]',
            turn.precheck.ok
              ? 'border-green-800 bg-green-900/20 text-green-300'
              : 'border-red-800 bg-red-900/20 text-red-300')}>
            <span className="font-semibold">
              Geometry pre-check: {turn.precheck.ok ? 'OK' : `${turn.precheck.violations.length} issue(s)`}
            </span>
            {!turn.precheck.ok && turn.precheck.violations.slice(0, 3).map((v, i) => (
              <div key={i} className="mt-0.5 text-red-300/90">
                · {v.kind}{v.sub_id ? ` (${v.sub_id})` : ''}: {v.detail}
              </div>
            ))}
          </div>
        )}

        {turn.result && (
          <div className="mt-2 space-y-1 text-[11px] text-adam-neutral-300">
            <div>
              Built {turn.result.built}/{turn.result.links} links ·{' '}
              {turn.result.movable_joints} movable ·{' '}
              {turn.result.iterations} iter
            </div>
            {turn.result.judge && (
              <div>
                <span className="font-semibold">
                  Judge: {turn.result.judge.passed === false ? 'FAIL' : 'PASS'}
                </span>
                {turn.result.judge.reasons ? ` — ${turn.result.judge.reasons}` : ''}
              </div>
            )}
            {turn.result.physics && (
              <div>
                <span className="font-semibold">
                  Physics: {turn.result.physics.passed ? 'PASS' : 'FAIL'}
                </span>
                {turn.result.physics.summary ? ` — ${turn.result.physics.summary}` : ''}
              </div>
            )}
          </div>
        )}
        {turn.error && (
          <pre className="mt-2 whitespace-pre-wrap rounded border border-red-800 bg-red-900/20 p-2 text-[11px] text-red-300">
            {turn.error}
          </pre>
        )}

        {/* live log while streaming; generated SCAD once available */}
        {turn.streaming && turn.lines.length > 0 && !turn.scad && (
          <pre className="mt-2 max-h-40 overflow-auto rounded bg-adam-neutral-950/50 p-2 font-mono text-[10px] leading-relaxed text-adam-neutral-400">
            {turn.lines.slice(-40).join('\n')}
          </pre>
        )}
        {turn.scad && (
          <div className="mt-2 max-h-56 overflow-hidden rounded border border-adam-neutral-800">
            <StreamingCodeBlock code={turn.scad} isStreaming={false} filename="model.scad" />
          </div>
        )}
      </div>
    </div>
  );
}

// Split a turn's stdout into per-iteration groups. The closed loop prints
// "===== ITERATION N (feedback: ...) =====" before each pass; everything before
// the first such marker (the [run] preamble) is attached to iteration 0. Each
// group also carries the judge/physics verdict parsed from that iteration's own
// lines, so the sidebar can show "Iteration 2: judge FAIL -> rebuilt".
type IterGroup = {
  n: number;                   // iteration number as printed (0-based)
  lines: string[];
  judge?: 'PASS' | 'FAIL';
  physics?: 'PASS' | 'FAIL' | 'ERROR';
  physicsCause?: string;       // structure | scenario | framing (on FAIL)
};

const ITER_RE = /=====\s*ITERATION\s+(\d+)/i;

function splitIterations(lines: string[]): IterGroup[] {
  const groups: IterGroup[] = [];
  let cur: IterGroup | null = null;
  for (const line of lines) {
    const m = line.match(ITER_RE);
    if (m) {
      cur = { n: Number(m[1]), lines: [] };
      groups.push(cur);
      continue;
    }
    if (!cur) { cur = { n: 0, lines: [] }; groups.push(cur); }
    cur.lines.push(line);
    // Verdicts printed within this iteration.
    const jm = line.match(/\[judge\]\s+verdict:\s*(PASS|FAIL)/i);
    if (jm) cur.judge = jm[1].toUpperCase() as 'PASS' | 'FAIL';
    if (/\[loop\].*physics PASS/i.test(line)) cur.physics = 'PASS';
    const pf = line.match(/\[loop\]\s*physics FAIL(?:\s*\(cause=([a-z]+)\))?/i);
    if (pf) { cur.physics = 'FAIL'; if (pf[1]) cur.physicsCause = pf[1].toLowerCase(); }
    if (/\[physics\]\s+failed:/i.test(line)) cur.physics = 'ERROR';
  }
  return groups;
}

// Stage state for ONE iteration's line group. `settled` means this iteration is
// finished (a later iteration started, or the whole run ended) -> all-done.
function deriveIterStages(g: IterGroup, settled: boolean): Record<string, StageState> {
  const out: Record<string, StageState> = {};
  if (settled) {
    STAGES.forEach((s) => { out[s.key] = 'done'; });
    return out;
  }
  const seen = new Set<string>();
  for (const line of g.lines) {
    for (const s of STAGES) if (s.match(line)) seen.add(s.key);
  }
  const lastSeenIdx = Math.max(-1, ...STAGES.map((s, i) => (seen.has(s.key) ? i : -1)));
  STAGES.forEach((s, i) => {
    if (i < lastSeenIdx) out[s.key] = 'done';
    else if (i === lastSeenIdx) out[s.key] = 'active';
    else out[s.key] = 'pending';
  });
  return out;
}

function StageDot({ state }: { state: StageState }) {
  if (state === 'active')
    return <span className="h-2.5 w-2.5 shrink-0 animate-spin rounded-full border-2 border-adam-blue border-t-transparent" />;
  if (state === 'done')
    return <span className="h-2.5 w-2.5 shrink-0 rounded-full bg-green-500" />;
  return <span className="h-2.5 w-2.5 shrink-0 rounded-full border border-adam-neutral-600" />;
}
