import { cn } from '@/lib/utils';
import type { GateLayer, GateResult } from './types';

// BenchmarkPanel — lists the deterministic gate results the pipeline emitted.
// A BLOCKING failure (ok:false) gets a red left border; a non-blocking WARNING
// (ok:true, e.g. ERR_DIM / ERR_SUP_GROUND) gets an amber one. Rows are grouped by
// layer then sorted blocking-first so the failures that stopped a build float up.

const LAYER_ORDER: Record<GateLayer, number> = {
  boss: 0,
  manager: 1,
  worker: 2,
  assembled: 3,
};

const LAYER_LABEL: Record<GateLayer, string> = {
  boss: 'Boss',
  manager: 'Manager',
  worker: 'Worker',
  assembled: 'Assembled',
};

function sortGates(gates: GateResult[]): GateResult[] {
  return [...gates].sort((a, b) => {
    const la = LAYER_ORDER[a.layer] ?? 9;
    const lb = LAYER_ORDER[b.layer] ?? 9;
    if (la !== lb) return la - lb;
    // Blocking (ok:false) before warnings (ok:true).
    if (a.ok !== b.ok) return a.ok ? 1 : -1;
    return a.code.localeCompare(b.code);
  });
}

export function BenchmarkPanel({ gates }: { gates: GateResult[] }) {
  const sorted = sortGates(gates);
  const blocking = gates.filter((g) => !g.ok).length;
  const warnings = gates.filter((g) => g.ok).length;

  return (
    <div className="flex min-h-0 flex-col">
      <div className="flex items-center justify-between px-3 py-2">
        <div className="text-[11px] font-semibold uppercase tracking-wide text-adam-neutral-400">
          Benchmark gates
        </div>
        {gates.length > 0 && (
          <div className="flex items-center gap-2 text-[10px]">
            {blocking > 0 && (
              <span className="rounded-full bg-red-900/40 px-2 py-0.5 font-semibold text-red-300">
                {blocking} blocking
              </span>
            )}
            {warnings > 0 && (
              <span className="rounded-full bg-amber-900/40 px-2 py-0.5 font-semibold text-amber-300">
                {warnings} warning{warnings === 1 ? '' : 's'}
              </span>
            )}
          </div>
        )}
      </div>

      {sorted.length === 0 ? (
        <div className="px-3 py-4 text-xs text-adam-neutral-500">
          No gate issues — all deterministic checks passed.
        </div>
      ) : (
        <div className="flex flex-col gap-1.5 px-3 pb-3">
          {sorted.map((g, i) => (
            <div
              key={`${g.layer}-${g.code}-${g.culprit}-${i}`}
              className={cn(
                'rounded border-l-2 bg-adam-neutral-900/40 px-2 py-1.5',
                g.ok
                  ? 'border-amber-500'
                  : 'border-red-500',
              )}
            >
              <div className="flex items-center gap-2">
                <span className="text-[9px] uppercase tracking-wide text-adam-neutral-500">
                  {LAYER_LABEL[g.layer] ?? g.layer}
                </span>
                <span
                  className={cn(
                    'font-mono text-[11px] font-semibold',
                    g.ok ? 'text-amber-300' : 'text-red-300',
                  )}
                >
                  {g.code}
                </span>
                {typeof g.iter === 'number' && (
                  <span className="ml-auto text-[9px] text-adam-neutral-500">
                    iter {g.iter}
                  </span>
                )}
              </div>
              {g.culprit && (
                <div className="mt-1">
                  <span className="rounded bg-adam-neutral-950/60 px-1.5 py-0.5 font-mono text-[10px] text-adam-neutral-300">
                    {g.culprit}
                  </span>
                </div>
              )}
              {g.detail && (
                <div className="mt-1 text-[11px] leading-snug text-adam-neutral-400">
                  {g.detail}
                </div>
              )}
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
