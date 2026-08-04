import { useEffect, useState } from 'react';
import { Link } from '@tanstack/react-router';
import { History, Plus } from 'lucide-react';
import { apiJson } from '@/services/api';
import { cn } from '@/lib/utils';

// One row per past run, read straight off disk (output/threads/<id>/thread.json plus
// legacy single-turn runs under output/). No database: reopening a run replays its
// recorded events.ndjson.
type RunEntry = {
  threadId: string;
  run_dir: string;
  title?: string;
  prompt?: string;
  created_at?: string;
  ok?: boolean;
  turns?: number;
  judgePassed?: boolean;
};

export function RunSidebar({ activeId }: { activeId?: string }) {
  const [runs, setRuns] = useState<RunEntry[]>([]);
  const [failed, setFailed] = useState(false);

  useEffect(() => {
    let alive = true;
    apiJson<{ runs?: RunEntry[] } | RunEntry[]>('list-maker2-runs')
      .then((data) => {
        if (!alive) return;
        setRuns(Array.isArray(data) ? data : (data.runs ?? []));
      })
      .catch(() => alive && setFailed(true));
    return () => {
      alive = false;
    };
  }, []);

  return (
    <div className="flex h-full w-64 shrink-0 flex-col border-r border-border bg-card/40">
      <Link to="/" className="block px-4 pb-3 pt-4 hover:opacity-80">
        {/* 1451x320 source, height-constrained so it stays crisp on HiDPI */}
        <img src="/automech-logo.png" alt="AutoMech" className="h-6 w-auto" />
      </Link>

      <Link
        to="/"
        className="mx-3 mb-1 flex items-center gap-2 rounded border border-border px-3 py-2 text-sm hover:bg-accent/40"
      >
        <Plus size={14} />
        New machine
      </Link>

      <div className="flex items-center gap-2 px-4 pb-1 pt-3 text-xs text-muted-foreground">
        <History size={12} />
        Past runs
      </div>

      <div className="min-h-0 flex-1 overflow-y-auto px-2 pb-2">
        {failed && (
          <div className="px-2 py-3 text-xs text-muted-foreground">
            could not read output/threads
          </div>
        )}
        {!failed && runs.length === 0 && (
          <div className="px-2 py-3 text-xs text-muted-foreground">no runs yet</div>
        )}
        {runs.map((r) => (
          <Link
            key={r.threadId}
            to="/workbench/$runId"
            params={{ runId: r.threadId }}
            search={{ prompt: '', model: '', iters: 0 }}
            className={cn(
              'block rounded px-2 py-2 text-xs hover:bg-accent/50',
              r.threadId === activeId && 'bg-accent/60',
            )}
            title={r.prompt}
          >
            <div className="truncate text-foreground">
              {r.title || r.prompt || r.threadId.slice(0, 8)}
            </div>
            <div className="mt-0.5 flex items-center gap-1.5 text-muted-foreground">
              <span
                className={cn(
                  'inline-block h-1.5 w-1.5 rounded-full',
                  r.judgePassed || r.ok ? 'bg-emerald-500' : 'bg-muted-foreground/40',
                )}
              />
              {r.turns ? `${r.turns} turn${r.turns > 1 ? 's' : ''}` : 'run'}
              {r.created_at && <span>· {r.created_at.slice(0, 10)}</span>}
            </div>
          </Link>
        ))}
      </div>
    </div>
  );
}
