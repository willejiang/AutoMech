import { cn } from '@/lib/utils';
import type { SubInfo } from './types';

// PartsInspector — the parts/material inspector.
//
// LIMITATION: the maker2 SSE stream does not emit a per-link 'material' artifact,
// and the assembled model.json (which carries dof/material/size per part) is not
// exposed to the client through /api/run-maker2-glb. So as a first cut we inspect
// at the SUBASSEMBLY granularity: each sub the boss split out, with its build
// pass/fail. If a future backend exposes per-part data, pass it via `parts` and it
// renders below each row (dof / material / size) without touching the callers.

export type PartInfo = {
  name: string;
  sub_id?: string;
  dof?: number;
  material?: string;
  size?: string;
};

export function PartsInspector({
  subs,
  parts,
}: {
  subs: SubInfo[];
  parts?: PartInfo[];
}) {
  const sorted = [...subs].sort((a, b) => a.sub_id.localeCompare(b.sub_id));
  const partsBySub = new Map<string, PartInfo[]>();
  for (const p of parts ?? []) {
    if (!p.sub_id) continue;
    const list = partsBySub.get(p.sub_id) ?? [];
    list.push(p);
    partsBySub.set(p.sub_id, list);
  }

  return (
    <div className="px-3 pb-3">
      <div className="mb-1 text-[11px] font-semibold uppercase tracking-wide text-adam-neutral-400">
        Parts inspector
      </div>

      {sorted.length === 0 ? (
        <div className="rounded border border-adam-neutral-800 bg-adam-neutral-950/30 px-2 py-3 text-xs text-adam-neutral-500">
          No subassemblies built yet.
        </div>
      ) : (
        <div className="flex flex-col gap-1">
          {sorted.map((s) => {
            const subParts = partsBySub.get(s.sub_id) ?? [];
            return (
              <div
                key={s.sub_id}
                className="rounded border border-adam-neutral-800 bg-adam-neutral-900/40 px-2 py-1.5"
              >
                <div className="flex items-center gap-2">
                  <span
                    className={cn(
                      'h-2 w-2 shrink-0 rounded-full',
                      s.ok ? 'bg-green-500' : 'bg-red-500',
                    )}
                  />
                  <span className="font-mono text-[11px] text-adam-neutral-200">
                    {s.sub_id}
                  </span>
                  <span
                    className={cn(
                      'ml-auto text-[9px] font-semibold uppercase',
                      s.ok ? 'text-green-400' : 'text-red-400',
                    )}
                  >
                    {s.ok ? 'ok' : 'fail'}
                  </span>
                </div>

                {/* Per-part detail, only when a backend exposes it. */}
                {subParts.length > 0 && (
                  <div className="mt-1 flex flex-col gap-0.5 pl-4">
                    {subParts.map((p, i) => (
                      <div
                        key={`${p.name}-${i}`}
                        className="flex items-center gap-2 font-mono text-[10px] text-adam-neutral-400"
                      >
                        <span className="text-adam-neutral-300">{p.name}</span>
                        {typeof p.dof === 'number' && (
                          <span>dof {p.dof}</span>
                        )}
                        {p.material && <span>· {p.material}</span>}
                        {p.size && <span>· {p.size}</span>}
                      </div>
                    ))}
                  </div>
                )}
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
}
