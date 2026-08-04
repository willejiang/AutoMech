import { Check, X, AlertTriangle } from 'lucide-react';
import { cn } from '@/lib/utils';
import type { GateLayer, GateResult } from './types';

// PipelineTimeline — the 8 pipeline stages as a vertical list. Each row shows a
// status dot (pending / active / done, derived from which stage-log prefixes have
// appeared in the stream) AND a small gate badge summarising that stage's layer:
// green check if all its gates are ok, red with a count if any are BLOCKING
// (ok:false), amber if only warnings (ok:true).

// The 8 stages, in order, with the log-line prefix(es) that mark each one as
// reached. Prefixes come from the maker2 loop's stdout (see Maker2EditorView).
// The HIERARCHY pipeline: a boss splits the machine into subassemblies, managers lay
// out parts, workers author CAD, and an assembler stitches them back together.
const HIERARCHY_STAGES = [
  {
    key: 'boss',
    label: 'Boss — split into subassemblies',
    match: (l: string) => l.startsWith('[boss]'),
  },
  {
    key: 'manager',
    label: 'Manager — parts + relative poses',
    match: (l: string) => l.startsWith('[1/3]') || l.startsWith('[sub:'),
  },
  {
    key: 'worker',
    label: 'Worker — generate CAD + render',
    match: (l: string) => l.startsWith('[2/3]') || l.includes('[worker]'),
  },
  {
    key: 'assembler',
    label: 'Assembler — stitch subassemblies',
    match: (l: string) => l.startsWith('[assembler]') || l.startsWith('[assembled]'),
  },
  {
    key: 'precheck',
    label: 'Pre-check — verify geometry fits',
    match: (l: string) => l.startsWith('[precheck]'),
  },
  {
    key: 'support',
    label: 'Support — welds + grounding',
    match: (l: string) => l.includes('[support]') || l.includes('[conflict]') || l.includes('[debugger]'),
  },
  {
    key: 'judge',
    label: 'Judge — review the model',
    match: (l: string) => l.startsWith('[judge]') || l.startsWith('[3/3]'),
  },
  {
    key: 'physics',
    label: 'Physics — drive + evaluate',
    match: (l: string) => l.startsWith('[physics]'),
  },
] as const;

// The SINGLE-AGENT pipeline, which is what this app runs by default. Its log prefixes
// are entirely different — one agent authors the whole machine, the geometry is compiled
// ([convex]/[mjcf]), gravity answers what holds it up ([support]), physics drives it, and
// a diagnosis feeds the next iteration. Matching it against the hierarchy stages above
// left every dot stuck on 'pending' for a run that was in fact progressing fine.
const SINGLE_AGENT_STAGES = [
  {
    key: 'author',
    label: 'Author — write build_machine()',
    match: (l: string) => l.includes('authoring build_machine'),
  },
  {
    key: 'compile',
    label: 'Compile — solids + collision geometry',
    match: (l: string) =>
      l.includes('part(s),') || l.startsWith('[convex]') || l.startsWith('[mjcf]'),
  },
  {
    key: 'support',
    label: 'Support — what actually holds it up',
    match: (l: string) => l.includes('[support]'),
  },
  {
    key: 'physics',
    label: 'Physics — drive + evaluate',
    match: (l: string) => l.startsWith('[physics]') || l.includes('simulating physics'),
  },
  {
    key: 'diagnose',
    label: 'Diagnose — score + re-author',
    match: (l: string) =>
      l.includes('score=') || l.includes('-> diagnose') || l.includes('PASS on iteration'),
  },
] as const;

type Stage = {
  key: string;
  label: string;
  match: (l: string) => boolean;
};

type StageKey = string;
type StageState = 'pending' | 'active' | 'done';

// Which gate layer's results are surfaced under each stage's badge. The judge and
// physics stages have no deterministic gate layer of their own.
const STAGE_GATE_LAYER: Record<string, GateLayer> = {
  boss: 'boss',
  manager: 'manager',
  worker: 'worker',
  assembler: 'assembled',
  precheck: 'assembled',
  support: 'assembled',
};

// Which pipeline produced these lines? Read it off the log rather than plumbing the
// mode down: a replayed thread carries its own lines and no route state.
function pickStages(lines: string[]): readonly Stage[] {
  const hierarchy = lines.some(
    (l) => l.startsWith('[boss]') || l.startsWith('[sub:') || l.startsWith('[assembler]'),
  );
  return hierarchy ? HIERARCHY_STAGES : SINGLE_AGENT_STAGES;
}

function deriveStageStates(
  lines: string[],
  done: boolean,
  stages: readonly Stage[],
): Record<StageKey, StageState> {
  const out = {} as Record<StageKey, StageState>;
  const seen = new Set<StageKey>();
  for (const line of lines) {
    for (const s of stages) if (s.match(line)) seen.add(s.key);
  }
  const lastSeenIdx = Math.max(-1, ...stages.map((s, i) => (seen.has(s.key) ? i : -1)));
  stages.forEach((s, i) => {
    if (done) {
      // Whole run finished: everything that was ever seen is done; the rest stays
      // pending (a run can legitimately end before physics if it hard-failed).
      out[s.key] = seen.has(s.key) ? 'done' : 'pending';
    } else if (i < lastSeenIdx) out[s.key] = 'done';
    else if (i === lastSeenIdx) out[s.key] = 'active';
    else out[s.key] = 'pending';
  });
  return out;
}

// Roll up the gates for one layer into a badge verdict.
type Badge = { kind: 'none' } | { kind: 'ok' } | { kind: 'warn' } | { kind: 'fail'; count: number };

function badgeForLayer(gates: GateResult[], layer: GateLayer | undefined): Badge {
  if (!layer) return { kind: 'none' };
  const forLayer = gates.filter((g) => g.layer === layer);
  if (forLayer.length === 0) return { kind: 'none' };
  const blocking = forLayer.filter((g) => !g.ok);
  if (blocking.length > 0) return { kind: 'fail', count: blocking.length };
  const warnings = forLayer.filter((g) => g.ok);
  if (warnings.length > 0) return { kind: 'warn' };
  return { kind: 'ok' };
}

function StageDot({ state }: { state: StageState }) {
  if (state === 'active')
    return (
      <span className="h-2.5 w-2.5 shrink-0 animate-spin rounded-full border-2 border-adam-blue border-t-transparent" />
    );
  if (state === 'done')
    return <span className="h-2.5 w-2.5 shrink-0 rounded-full bg-green-500" />;
  return <span className="h-2.5 w-2.5 shrink-0 rounded-full border border-adam-neutral-600" />;
}

function GateBadge({ badge }: { badge: Badge }) {
  if (badge.kind === 'none') return null;
  if (badge.kind === 'ok')
    return (
      <span className="flex items-center gap-0.5 rounded-full bg-green-900/40 px-1.5 py-0.5 text-[9px] font-semibold text-green-300">
        <Check className="h-2.5 w-2.5" />
      </span>
    );
  if (badge.kind === 'warn')
    return (
      <span className="flex items-center gap-0.5 rounded-full bg-amber-900/40 px-1.5 py-0.5 text-[9px] font-semibold text-amber-300">
        <AlertTriangle className="h-2.5 w-2.5" />
      </span>
    );
  return (
    <span className="flex items-center gap-0.5 rounded-full bg-red-900/40 px-1.5 py-0.5 text-[9px] font-semibold text-red-300">
      <X className="h-2.5 w-2.5" />
      {badge.count}
    </span>
  );
}

export function PipelineTimeline({
  lines,
  gates,
  done,
}: {
  lines: string[];
  gates: GateResult[];
  done: boolean;
}) {
  const stages = pickStages(lines);
  const states = deriveStageStates(lines, done, stages);
  return (
    <div className="px-3 py-2">
      <div className="mb-2 text-[11px] font-semibold uppercase tracking-wide text-adam-neutral-400">
        Pipeline
      </div>
      <ul className="space-y-1.5">
        {stages.map((s) => {
          const st = states[s.key];
          const badge = badgeForLayer(gates, STAGE_GATE_LAYER[s.key]);
          return (
            <li key={s.key} className="flex items-center gap-2 text-xs">
              <StageDot state={st} />
              <span
                className={cn(
                  'flex-1',
                  st === 'pending' && 'text-adam-neutral-500',
                  st === 'active' && 'text-adam-text-primary',
                  st === 'done' && 'text-adam-neutral-300',
                )}
              >
                {s.label}
              </span>
              <GateBadge badge={badge} />
            </li>
          );
        })}
      </ul>
    </div>
  );
}
