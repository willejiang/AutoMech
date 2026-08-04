import { useCallback, useEffect, useReducer, useRef, useState } from 'react';
import { useNavigate } from '@tanstack/react-router';
import {
  ArrowLeft,
  ChevronLeft,
  ChevronRight,
  Brain,
  ChevronDown,
  ChevronUp,
  Square,
} from 'lucide-react';
import { Button } from '@/components/ui/button';
import { ScrollArea } from '@/components/ui/scroll-area';
import { Maker2ModelCanvas } from '@/components/viewer/Maker2ModelCanvas';
import { PhysicsPanel, type PhysicsResult } from '@/components/viewer/PhysicsPanel';
import { apiUrl } from '@/services/api';
import { cn } from '@/lib/utils';
import { PipelineTimeline } from '@/components/workbench/PipelineTimeline';
import { PartsGhostPanel } from '@/components/workbench/PartsGhostPanel';
import { ScoreGraph } from '@/components/workbench/ScoreGraph';
import { PartsInspector } from '@/components/workbench/PartsInspector';
import type {
  ArtifactEvent,
  AssembledInfo,
  GateResult,
  IterationInfo,
  SubInfo,
} from '@/components/workbench/types';

// WorkbenchView — a fresh, cleaner maker2 frontend. It streams ONE hierarchy run
// over the same /api/run-maker2-stream SSE + /api/run-maker2-glb backend the
// Maker2EditorView uses (unchanged), and lays the result out as a 3-column
// workbench: LEFT = pipeline timeline + benchmark gates; CENTER = the assembled
// 3D model with version scrubbing + a deep-think toggle; RIGHT = a score/iteration
// graph, a parts/subassembly inspector, and a collapsible raw-log tail.

interface WorkbenchViewProps {
  prompt: string;
  model: string;
  iters: number;
  threadId: string;
  deep?: boolean;
  // Research + pipeline switches, passed through to the run. `web` used to be
  // hard-coded to '1' here and `mode` was never sent at all, so the launcher's
  // toggles could not reach Python no matter what the user picked.
  web?: boolean;
  mode?: 'single-agent' | 'hierarchy';
  // Upload id of a reference image; the run route turns it back into a path for --image.
  image?: string;
}

type Maker2Result = {
  ok: boolean;
  run_dir?: string;
  render_dir?: string;
  hard_failed?: boolean;
  iterations?: number;
  error?: string;
};

// ---- Reducer state: everything the stream accumulates for this one run. ----

type State = {
  stageLines: string[]; // raw stage/log lines, for the log tail
  gates: GateResult[]; // latest gate per (layer, code, culprit)
  iterations: IterationInfo[]; // one entry per iter (score / passed / breakdown)
  subs: SubInfo[]; // subassembly builds
  assembled: AssembledInfo[]; // rendered machines per iter (canvas scrubbing)
  selectedIdx: number; // index into `assembled` shown on the canvas
  pinned: boolean; // user scrubbed off the newest -> don't auto-follow
  error: string | null;
  done: boolean;
};

const initialState: State = {
  stageLines: [],
  gates: [],
  iterations: [],
  subs: [],
  assembled: [],
  selectedIdx: -1,
  pinned: false,
  error: null,
  done: false,
};

type Action =
  | { type: 'line'; raw: string }
  | { type: 'gate'; gate: GateResult }
  | { type: 'sub'; sub: SubInfo }
  | { type: 'assembled'; iter: number; run_dir: string }
  | { type: 'glb'; iter: number; blob: Blob }
  | { type: 'physics'; info: IterationInfo }
  | { type: 'select'; idx: number }
  | { type: 'error'; error: string }
  | { type: 'done' };

// Merge one iteration's partial info (from a physics or assembled event) into the
// iterations array, keyed by iter.
function upsertIter(iters: IterationInfo[], info: IterationInfo): IterationInfo[] {
  const at = iters.findIndex((i) => i.iter === info.iter);
  if (at >= 0) {
    const next = [...iters];
    next[at] = { ...next[at], ...info };
    return next;
  }
  return [...iters, info].sort((a, b) => a.iter - b.iter);
}

function reducer(state: State, action: Action): State {
  switch (action.type) {
    case 'line':
      return { ...state, stageLines: [...state.stageLines, action.raw] };

    case 'gate': {
      const g = action.gate;
      // Keep the LATEST gate per (layer, code, culprit) so a re-run's fix
      // replaces the stale failure instead of stacking a duplicate.
      const rest = state.gates.filter(
        (x) => !(x.layer === g.layer && x.code === g.code && x.culprit === g.culprit),
      );
      return { ...state, gates: [...rest, g] };
    }

    case 'sub': {
      const rest = state.subs.filter((s) => s.sub_id !== action.sub.sub_id);
      return { ...state, subs: [...rest, action.sub] };
    }

    case 'assembled': {
      // A new stitched machine for this iter. Upsert into `assembled` (dedupe by
      // iter) and, unless the user pinned an earlier version, select the newest.
      const at = state.assembled.findIndex((a) => a.iter === action.iter);
      let assembled: AssembledInfo[];
      if (at >= 0) {
        assembled = [...state.assembled];
        assembled[at] = { ...assembled[at], run_dir: action.run_dir };
      } else {
        assembled = [...state.assembled, { iter: action.iter, run_dir: action.run_dir }].sort(
          (a, b) => a.iter - b.iter,
        );
      }
      const newestIdx = assembled.length - 1;
      const selectedIdx = state.pinned ? state.selectedIdx : newestIdx;
      return {
        ...state,
        assembled,
        selectedIdx: selectedIdx < 0 ? newestIdx : selectedIdx,
        iterations: upsertIter(state.iterations, {
          iter: action.iter,
          runDir: action.run_dir,
        }),
      };
    }

    case 'glb': {
      const assembled = state.assembled.map((a) =>
        a.iter === action.iter ? { ...a, glbBlob: action.blob } : a,
      );
      return { ...state, assembled };
    }

    case 'physics':
      return { ...state, iterations: upsertIter(state.iterations, action.info) };

    case 'select': {
      const clamped = Math.max(0, Math.min(state.assembled.length - 1, action.idx));
      return {
        ...state,
        selectedIdx: clamped,
        pinned: clamped !== state.assembled.length - 1,
      };
    }

    case 'error':
      return { ...state, error: action.error, done: true };

    case 'done':
      return { ...state, done: true };

    default:
      return state;
  }
}

// Stable identity so <Maker2ModelCanvas>'s ghosting effect doesn't re-run every render
// on an iteration nobody has ghosted anything in.
const EMPTY_GHOSTED: Set<string> = new Set();

export function WorkbenchView(props: WorkbenchViewProps) {
  const { prompt, model, iters, threadId, deep } = props;
  const web = props.web ?? true;
  const mode = props.mode ?? 'single-agent';
  const image = props.image;
  const navigate = useNavigate();
  const [state, dispatch] = useReducer(reducer, initialState);
  const startedRef = useRef(false);
  const esRef = useRef<EventSource | null>(null);

  // Deep-think toggle: controlled boolean, sticky across reloads (localStorage),
  // seeded from the route's `deep` prop the FIRST time. Flipping it mid-run only
  // affects the NEXT run (the SSE query is fixed at open time).
  const [deepThink, setDeepThinkState] = useState<boolean>(() => {
    if (typeof localStorage !== 'undefined') {
      const stored = localStorage.getItem('maker2.deepThink');
      if (stored === '1') return true;
      if (stored === '0') return false;
    }
    return !!deep;
  });
  const setDeepThink = (v: boolean) => {
    setDeepThinkState(v);
    try {
      localStorage.setItem('maker2.deepThink', v ? '1' : '0');
    } catch {
      /* ignore */
    }
  };
  // The deep value that was actually used for the live run (fixed at open time),
  // so the header can note a pending change if the user flips the toggle mid-run.
  const runDeepRef = useRef<boolean>(deepThink);
  const [collapsedLog, setCollapsedLog] = useState(true);
  // True when this view rendered a RECORDED run rather than starting a live one.
  const [replayed, setReplayed] = useState(false);
  // True when the user STOPPED this run, so the header can say so rather than
  // showing it as a run that finished on its own.
  const [stopped, setStopped] = useState(false);

  // STOP the live run. Closing the EventSource drops the HTTP connection, which
  // fires `request.signal`'s abort in run-maker2-stream and kills the python
  // child — the server side already handles that. We must also mark the run done
  // ourselves: no `end` event will ever arrive now, so without this the UI spins
  // forever on a run that is already dead.
  const stop = useCallback(() => {
    esRef.current?.close();
    esRef.current = null;
    setStopped(true);
    dispatch({ type: 'done' });
  }, []);

  // Per-part ghosting for the model on the canvas. Keyed by iteration: each iteration
  // is a different machine, so a part hidden in iter 2 must not silently stay hidden
  // when you scrub to iter 5 where that name may mean something else (or not exist).
  const [partNames, setPartNames] = useState<string[]>([]);
  const [ghostedByIter, setGhostedByIter] = useState<Record<number, Set<string>>>({});

  // Fetch the assembled GLB for one iteration and drop the blob into that entry.
  const loadGlb = useCallback((iter: number, dir: string) => {
    fetch(`${apiUrl('run-maker2-glb')}?dir=${encodeURIComponent(dir)}`)
      .then((r) => (r.ok ? r.blob() : Promise.reject(r.statusText)))
      .then((blob) => dispatch({ type: 'glb', iter, blob }))
      .catch(() => {
        /* a partial/failed export just leaves the prior canvas */
      });
  }, []);

  // Apply ONE pipeline event to the view state. Shared by the live SSE and by
  // replay, so a recorded run renders through exactly the same path it did live.
  const applyEvent = useCallback(
    (event: string, data: unknown) => {
      switch (event) {
        case 'stage':
        case 'log':
          dispatch({ type: 'line', raw: (data as { raw: string }).raw });
          break;
        case 'artifact': {
          const a = data as ArtifactEvent;
          switch (a.kind) {
            case 'gate':
              dispatch({
                type: 'gate',
                gate: {
                  layer: a.layer,
                  code: a.code,
                  detail: a.detail,
                  culprit: a.culprit,
                  ok: a.ok,
                  sub_id: a.sub_id,
                  iter: a.iter,
                },
              });
              break;
            case 'subassembly':
              dispatch({
                type: 'sub',
                sub: { sub_id: a.sub_id, ok: a.ok, run_dir: a.run_dir },
              });
              break;
            case 'assembled_model': {
              const dir = a.render_dir || a.run_dir;
              dispatch({ type: 'assembled', iter: a.iter, run_dir: dir });
              loadGlb(a.iter, dir);
              break;
            }
            case 'physics':
              dispatch({
                type: 'physics',
                info: {
                  iter: a.iter,
                  score: a.score,
                  passed: a.passed,
                  breakdown: a.score_breakdown,
                  runDir: a.render_dir || a.run_dir,
                  physics: a.physics,
                },
              });
              break;
            // precheck / mesh_progress are surfaced elsewhere (gates + canvas); no
            // dedicated state needed here for the first cut.
            default:
              break;
          }
          break;
        }
        case 'result': {
          const r = data as Maker2Result;
          if (r.hard_failed || r.error) {
            dispatch({
              type: 'error',
              error: r.error || 'The pipeline did not produce a model.',
            });
          }
          break;
        }
        case 'error': {
          const d = data as { error?: string; tail?: string };
          dispatch({
            type: 'error',
            error: (d?.error ?? 'stream error') + (d?.tail ? `\n${d.tail}` : ''),
          });
          break;
        }
        case 'end':
          dispatch({ type: 'done' });
          break;
        default:
          break;
      }
    },
    [loadGlb],
  );

  // Open this run. REPLAY-FIRST: a run that already happened is recorded as an
  // event log (run-maker2-stream tees every event to the thread), so opening it
  // from history REPLAYS that recording. Only a thread with NO recording starts a
  // live run — otherwise clicking a past run re-ran the whole pipeline and
  // overwrote the very result the user clicked on.
  useEffect(() => {
    if (startedRef.current) return;
    startedRef.current = true;
    let cancelled = false;

    const live = () => {
      if (cancelled || !prompt) {
        // No recording AND no prompt to run: nothing to show. Don't leave the UI
        // spinning forever on a thread that has neither.
        if (!prompt) dispatch({ type: 'done' });
        return;
      }
      runDeepRef.current = deepThink;
      setReplayed(false);

      const qs = new URLSearchParams({
        prompt,
        iters: String(iters),
        thread: threadId,
        web: web ? '1' : '0',
        deep: deepThink ? '1' : '0',
        mode,
      });
      if (model) qs.set('model', model);
      // The image id ends in .png/.jpg, and a request URL ENDING in an image extension is
      // served as a static asset before it ever reaches the API route ("Cannot GET
      // /api/run-maker2-stream"). A trailing marker keeps the extension off the end of the
      // URL. Verified against the dev server: same id last = 404, not last = run starts.
      if (image) {
        qs.set('image', image);
        qs.set('_', '1');
      }

      const es = new EventSource(`${apiUrl('run-maker2-stream')}?${qs.toString()}`);
      esRef.current = es;
      const on = (name: string) => (e: Event) => {
        const raw = (e as MessageEvent).data;
        if (!raw && name !== 'error') return;
        try {
          applyEvent(name, raw ? JSON.parse(raw) : {});
        } catch {
          /* a malformed frame is not worth tearing the run down */
        }
        if (name === 'end') es.close();
      };
      for (const name of ['stage', 'log', 'artifact', 'result', 'error', 'end']) {
        es.addEventListener(name, on(name));
      }
    };

    // Ask for a recording first. A network/route failure just falls through to a
    // live run, so this can never make the page worse than it was.
    fetch(`${apiUrl('maker2-replay')}?id=${encodeURIComponent(threadId)}`)
      .then((r) => (r.ok ? r.json() : Promise.reject(r.statusText)))
      .then((doc: { recorded?: boolean; events?: { event: string; data: unknown }[] }) => {
        if (cancelled) return;
        const events = doc?.events ?? [];
        if (!doc?.recorded || events.length === 0) {
          live();
          return;
        }
        setReplayed(true);
        for (const ev of events) applyEvent(ev.event, ev.data);
        dispatch({ type: 'done' });
      })
      .catch(() => {
        if (!cancelled) live();
      });

    return () => {
      cancelled = true;
    };
    // deepThink is intentionally read at open time only (a mid-run flip affects the
    // next run); startedRef guards against re-open on its change.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [prompt, model, iters, threadId, applyEvent]);

  // On unmount, close the stream AND reset the guard so a StrictMode remount opens
  // a fresh EventSource (else the double-mount closes the first and the guard
  // blocks re-open -> a static, never-updating UI).
  useEffect(
    () => () => {
      esRef.current?.close();
      startedRef.current = false;
    },
    [],
  );

  // ---- Derived view state. ----
  const selected = state.selectedIdx >= 0 ? state.assembled[state.selectedIdx] : undefined;
  const hasVersions = state.assembled.length > 1;
  const canvasGlb = selected?.glbBlob;
  const canvasFailed = state.done && !state.assembled.some((a) => a.glbBlob) && !!state.error;

  const selectedIter = selected?.iter;
  const selectedIterInfo =
    selectedIter !== undefined
      ? state.iterations.find((i) => i.iter === selectedIter)
      : undefined;
  const scorePoints = state.iterations
    .filter((i): i is IterationInfo & { score: number } => typeof i.score === 'number')
    .map((i) => ({ iter: i.iter, score: i.score }));

  // Ghosting is per-iteration; -1 is the "nothing selected yet" bucket.
  const ghostKey = selectedIter ?? -1;
  const ghosted = ghostedByIter[ghostKey] ?? EMPTY_GHOSTED;

  const running = !state.done;
  const deepPending = runDeepRef.current !== deepThink;

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
            workbench (maker2){model ? ` · ${model}` : ''} ·{' '}
            {iters > 0 ? `max ${iters} iter` : 'loops until it works'}
            {replayed
              ? ' · recorded run'
              : stopped
                ? ' · stopped'
                : running
                  ? ' · running…'
                  : ' · done'}
          </div>
        </div>
        {/* Stop: only for a LIVE run. Closing the SSE aborts the request, which the
            server turns into a kill of the python child. A replay has nothing to stop. */}
        {running && !replayed && (
          <button
            onClick={stop}
            className="flex shrink-0 items-center gap-1.5 whitespace-nowrap rounded-full border border-red-500/60 bg-red-500/10 px-3 py-1 text-xs font-medium text-red-400 transition-colors hover:bg-red-500/20"
            title="Stop this run now — kills the generation process."
          >
            <Square className="h-3.5 w-3.5" />
            Stop
          </button>
        )}
        {/* Deep-think toggle: CadQuery + full debugger when on, OpenSCAD + slim
            when off. A mid-run flip only takes effect on the next run. */}
        <div className="flex shrink-0 flex-col items-end gap-0.5">
          <button
            onClick={() => setDeepThink(!deepThink)}
            className={cn(
              'flex items-center gap-1.5 whitespace-nowrap rounded-full border px-3 py-1 text-xs font-medium transition-colors',
              deepThink
                ? 'border-adam-blue bg-adam-blue/15 text-adam-blue'
                : 'border-adam-neutral-700 bg-adam-neutral-900/40 text-adam-neutral-400 hover:text-adam-text-primary',
            )}
            title="Deep-think: CadQuery + full debugger (slower, sturdier). Off: OpenSCAD + slim."
          >
            <Brain className="h-3.5 w-3.5" />
            Deep-think {deepThink ? 'on' : 'off'}
          </button>
          {deepPending && (
            <span className="text-[10px] text-amber-400">applies next run</span>
          )}
        </div>
      </div>

      {/* 3-column workbench */}
      <div className="flex min-h-0 flex-1">
        {/* LEFT: pipeline timeline + benchmark gates */}
        <div className="flex w-[320px] shrink-0 flex-col border-r border-adam-neutral-800">
          <ScrollArea className="min-h-0 flex-1">
            <PipelineTimeline lines={state.stageLines} gates={state.gates} done={state.done} />
            <div className="border-t border-adam-neutral-800" />
            <PartsGhostPanel
              parts={partNames}
              ghosted={ghosted}
              onChange={(next) =>
                setGhostedByIter((prev) => ({ ...prev, [ghostKey]: next }))
              }
              iter={selectedIter}
            />
          </ScrollArea>
        </div>

        {/* CENTER: the assembled 3D model + version scrubber */}
        <div className="relative flex min-w-0 flex-1 flex-col bg-adam-neutral-950/30">
          <div className="flex items-center justify-between border-b border-adam-neutral-800 px-4 py-2 text-xs">
            <div className="text-adam-neutral-400">
              {selected ? (
                <>
                  iteration{' '}
                  <span className="font-mono text-adam-text-primary">{selected.iter}</span>
                  {selectedIterInfo && typeof selectedIterInfo.score === 'number' && (
                    <>
                      {' · '}score{' '}
                      <span
                        className={cn(
                          'font-mono font-semibold',
                          selectedIterInfo.passed ? 'text-green-400' : 'text-amber-300',
                        )}
                      >
                        {selectedIterInfo.score.toFixed(3)}
                      </span>
                    </>
                  )}
                </>
              ) : (
                <span>Assembling…</span>
              )}
            </div>
            {hasVersions && (
              <div className="text-adam-neutral-400">
                version {state.selectedIdx + 1} / {state.assembled.length}
              </div>
            )}
          </div>

          <div className="relative min-h-0 flex-1">
            <Maker2ModelCanvas
              glbBlob={canvasGlb}
            ghosted={ghosted}
            onParts={setPartNames}
              status={canvasFailed ? 'failed' : 'loading'}
              failedReason={canvasFailed ? state.error ?? undefined : undefined}
            />
            {/* Version scrubber: step through earlier/later design iterations. */}
            {hasVersions && (
              <>
                <button
                  aria-label="previous version"
                  disabled={state.selectedIdx <= 0}
                  onClick={() => dispatch({ type: 'select', idx: state.selectedIdx - 1 })}
                  className="absolute left-3 top-1/2 -translate-y-1/2 rounded-full bg-adam-neutral-900/80 p-2 text-adam-text-primary shadow hover:bg-adam-neutral-800 disabled:opacity-30"
                >
                  <ChevronLeft className="h-5 w-5" />
                </button>
                <button
                  aria-label="next version"
                  disabled={state.selectedIdx >= state.assembled.length - 1}
                  onClick={() => dispatch({ type: 'select', idx: state.selectedIdx + 1 })}
                  className="absolute right-3 top-1/2 -translate-y-1/2 rounded-full bg-adam-neutral-900/80 p-2 text-adam-text-primary shadow hover:bg-adam-neutral-800 disabled:opacity-30"
                >
                  <ChevronRight className="h-5 w-5" />
                </button>
                {state.pinned && (
                  <button
                    onClick={() =>
                      dispatch({ type: 'select', idx: state.assembled.length - 1 })
                    }
                    className="absolute bottom-3 left-1/2 -translate-x-1/2 rounded-full bg-adam-neutral-900/80 px-3 py-1 text-xs text-adam-blue shadow hover:underline"
                  >
                    jump to latest
                  </button>
                )}
              </>
            )}
          </div>
        </div>

        {/* RIGHT: score graph + parts inspector + collapsible log tail */}
        <div className="flex w-[340px] shrink-0 flex-col border-l border-adam-neutral-800 bg-adam-bg-secondary-dark">
          <ScrollArea className="min-h-0 flex-1">
            <div className="pt-2">
              <ScoreGraph points={scorePoints} />
            </div>
            {selectedIterInfo?.physics && selectedIterInfo.runDir && (
              <div className="border-t border-adam-neutral-800 pt-2">
                <PhysicsPanel
                  physics={selectedIterInfo.physics as unknown as PhysicsResult}
                  runDir={selectedIterInfo.runDir}
                  running={running}
                />
              </div>
            )}
            <div className="border-t border-adam-neutral-800 pt-2">
              <PartsInspector subs={state.subs} />
            </div>

            {/* Collapsible raw-log tail. */}
            <div className="border-t border-adam-neutral-800">
              <button
                onClick={() => setCollapsedLog((c) => !c)}
                className="flex w-full items-center justify-between px-3 py-2 text-[11px] font-semibold uppercase tracking-wide text-adam-neutral-400 hover:text-adam-text-primary"
              >
                <span>Raw log ({state.stageLines.length})</span>
                {collapsedLog ? (
                  <ChevronDown className="h-3.5 w-3.5" />
                ) : (
                  <ChevronUp className="h-3.5 w-3.5" />
                )}
              </button>
              {!collapsedLog && state.stageLines.length > 0 && (
                <pre className="mx-3 mb-3 max-h-60 overflow-auto rounded bg-adam-neutral-950/50 p-2 font-mono text-[10px] leading-relaxed text-adam-neutral-400">
                  {state.stageLines.slice(-120).join('\n')}
                </pre>
              )}
            </div>

            {state.error && (
              <pre className="mx-3 mb-3 whitespace-pre-wrap rounded border border-red-800 bg-red-900/20 p-2 text-[11px] text-red-300">
                {state.error}
              </pre>
            )}
          </ScrollArea>
        </div>
      </div>
    </div>
  );
}
