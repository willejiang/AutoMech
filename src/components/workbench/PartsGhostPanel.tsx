import { useMemo, useState } from 'react';
import { cn } from '@/lib/utils';

/**
 * PartsGhostPanel — the part list for the iteration currently on the canvas, with a
 * checkbox per part that controls whether it renders SOLID or GHOSTED.
 *
 * This replaces the gate list in the left column because seeing inside the machine is
 * the thing you actually do here all day. Hover-to-fade only dims the one front-most
 * part under the cursor, and category ghosting is too coarse: a watch has several
 * nested plates, bridges and pipes, and you usually want to keep two specific parts
 * solid and push everything else back. That needs a persistent, per-part control.
 *
 * Unchecked = ghosted, not hidden. A hidden part tells you nothing about where it sat;
 * a ghosted one still shows its envelope, so you can see the gear train THROUGH the
 * bridge that covers it and still know the bridge is there.
 */

/** Coarse grouping for the list, matched against the URDF link name. Purely to keep a
 *  40-part list navigable — the checkboxes act on individual parts either way. */
const GROUPS: { key: string; label: string; test: RegExp }[] = [
  { key: 'gears', label: 'Gears & wheels', test: /gear|pinion|wheel|sun|planet|ring|cog/ },
  { key: 'shafts', label: 'Shafts & arbors', test: /shaft|pin|arbor|axle|spindle|pipe|staff/ },
  { key: 'bearings', label: 'Bearings & spacers', test: /bearing|bush|journal|collar|spacer|jewel|thrust|washer/ },
  {
    key: 'structure',
    label: 'Plates & structure',
    test: /housing|shell|wall|plate|cover|case|frame|carrier|base|body|bracket|mount|seat|flange|bridge|standoff|pillar/,
  },
  { key: 'other', label: 'Other', test: /.*/ },
];

function groupFor(name: string): string {
  const n = name.toLowerCase();
  for (const g of GROUPS) if (g.test.test(n)) return g.key;
  return 'other';
}

interface PartsGhostPanelProps {
  /** Part names in the GLB currently on the canvas, in scene order. */
  parts: string[];
  /** Names rendered ghosted. Everything not in here is solid. */
  ghosted: Set<string>;
  onChange: (next: Set<string>) => void;
  /** Iteration the canvas is showing, so the header says what this list belongs to. */
  iter?: number;
}

export function PartsGhostPanel({ parts, ghosted, onChange, iter }: PartsGhostPanelProps) {
  const [collapsed, setCollapsed] = useState<Set<string>>(new Set());

  const groups = useMemo(() => {
    const byKey = new Map<string, string[]>();
    for (const p of parts) {
      const k = groupFor(p);
      const list = byKey.get(k);
      if (list) list.push(p);
      else byKey.set(k, [p]);
    }
    return GROUPS.map((g) => ({ ...g, items: byKey.get(g.key) ?? [] })).filter(
      (g) => g.items.length > 0,
    );
  }, [parts]);

  const setMany = (names: string[], solid: boolean) => {
    const next = new Set(ghosted);
    for (const n of names) {
      if (solid) next.delete(n);
      else next.add(n);
    }
    onChange(next);
  };

  const groupState = (items: string[]) => {
    const solid = items.filter((n) => !ghosted.has(n)).length;
    return solid === 0 ? 'none' : solid === items.length ? 'all' : 'some';
  };

  if (parts.length === 0) {
    return (
      <div className="px-3 py-3 text-xs text-adam-neutral-500">
        No model on the canvas yet — the part list appears once an iteration is assembled.
      </div>
    );
  }

  const ghostedCount = parts.filter((p) => ghosted.has(p)).length;

  return (
    <div className="px-3 py-3">
      <div className="mb-2 flex items-center justify-between">
        <div className="text-xs font-semibold uppercase tracking-wide text-adam-neutral-400">
          Parts{typeof iter === 'number' ? ` · iter ${iter}` : ''}
        </div>
        <div className="flex items-center gap-2 text-[10px]">
          {ghostedCount > 0 && (
            <span className="text-adam-neutral-500">{ghostedCount} ghosted</span>
          )}
          <button
            type="button"
            className="text-adam-neutral-400 hover:text-adam-text-primary"
            onClick={() => setMany(parts, true)}
          >
            Show all
          </button>
          <button
            type="button"
            className="text-adam-neutral-400 hover:text-adam-text-primary"
            onClick={() => setMany(parts, false)}
          >
            Ghost all
          </button>
        </div>
      </div>
      <div className="mb-2 text-[10px] leading-snug text-adam-neutral-500">
        Uncheck a part to ghost it — it stays visible as an outline so you can see the
        train through it.
      </div>

      {groups.map((g) => {
        const st = groupState(g.items);
        const isCollapsed = collapsed.has(g.key);
        return (
          <div key={g.key} className="mb-2">
            <div className="flex items-center gap-2">
              <input
                type="checkbox"
                className="h-3.5 w-3.5 accent-adam-blue"
                checked={st !== 'none'}
                ref={(el) => {
                  if (el) el.indeterminate = st === 'some';
                }}
                onChange={(e) => setMany(g.items, e.target.checked)}
              />
              <button
                type="button"
                className="flex-1 text-left text-xs font-medium text-adam-neutral-300 hover:text-adam-text-primary"
                onClick={() =>
                  setCollapsed((prev) => {
                    const next = new Set(prev);
                    if (next.has(g.key)) next.delete(g.key);
                    else next.add(g.key);
                    return next;
                  })
                }
              >
                {g.label}{' '}
                <span className="text-adam-neutral-500">({g.items.length})</span>
              </button>
            </div>
            {!isCollapsed && (
              <div className="ml-5 mt-1 flex flex-col gap-0.5">
                {g.items.map((name) => {
                  const solid = !ghosted.has(name);
                  return (
                    <label
                      key={name}
                      className={cn(
                        'flex cursor-pointer items-center gap-2 text-xs',
                        solid ? 'text-adam-neutral-300' : 'text-adam-neutral-600',
                      )}
                    >
                      <input
                        type="checkbox"
                        className="h-3 w-3 accent-adam-blue"
                        checked={solid}
                        onChange={(e) => setMany([name], e.target.checked)}
                      />
                      <span className="truncate" title={name}>
                        {name}
                      </span>
                    </label>
                  );
                })}
              </div>
            )}
          </div>
        );
      })}
    </div>
  );
}
