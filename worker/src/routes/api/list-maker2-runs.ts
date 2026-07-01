import { createFileRoute } from '@tanstack/react-router';
import { json, methodNotAllowed, preflight } from '@/server/api';
import { readFileSync, readdirSync, existsSync, statSync } from 'node:fs';
import { resolve, join, basename } from 'node:path';

// DEV bridge: list maker2 CONVERSATIONS for the sidebar. A conversation is a
// thread (output/threads/<id>/thread.json) with one entry per thread showing its
// latest turn. Legacy runs that predate threads are listed as single-turn
// entries (reopened read-only by run_dir). No DB.
const REPO_ROOT = resolve(process.cwd(), '..');
const OUTPUT_ROOT = resolve(REPO_ROOT, 'output');
const THREADS_ROOT = resolve(OUTPUT_ROOT, 'threads');

type Entry = {
  // How the sidebar reopens it: a threadId (continue the conversation) OR, for
  // legacy runs, an empty threadId + a run_dir (read-only view).
  threadId: string;
  run_dir: string;      // the latest turn's run_dir (for legacy: the run itself)
  title: string;
  prompt: string;
  model: string;
  maxIters: number;
  created_at: string;
  ok: boolean;
  turns: number;
  judgePassed: boolean | null;
};

function slugToTitle(dirName: string): string {
  return dirName.replace(/_\d{8}_\d{6}$/, '').replace(/_/g, ' ').trim() || dirName;
}

function readJson(path: string): Record<string, unknown> | null {
  try { return JSON.parse(readFileSync(path, 'utf-8')); } catch { return null; }
}

export const Route = createFileRoute('/api/list-maker2-runs')({
  server: {
    handlers: {
      POST: methodNotAllowed,
      OPTIONS: preflight,
      GET: async () => {
        if (!existsSync(OUTPUT_ROOT)) return json({ runs: [] });

        const entries: Entry[] = [];
        const claimedRunDirs = new Set<string>();   // runs already shown via a thread

        // 1) Threads = the real conversations. One entry per thread, its latest turn.
        if (existsSync(THREADS_ROOT)) {
          for (const id of readdirSync(THREADS_ROOT)) {
            const tpath = join(THREADS_ROOT, id, 'thread.json');
            const doc = readJson(tpath);
            const turns = (doc?.turns as Array<Record<string, unknown>>) ?? [];
            if (!turns.length) continue;
            const first = turns[0];
            const last = turns[turns.length - 1];
            for (const t of turns) {
              if (typeof t.run_dir === 'string') claimedRunDirs.add(t.run_dir);
            }
            entries.push({
              threadId: id,
              run_dir: typeof last.run_dir === 'string' ? last.run_dir : '',
              title: (typeof first.message === 'string' && first.message) || 'Articulated run',
              prompt: typeof first.message === 'string' ? first.message : '',
              model: typeof doc?.model === 'string' ? doc.model : '',
              maxIters: 2,
              created_at: typeof last.ts === 'string' ? last.ts
                : (typeof doc?.created_at === 'string' ? doc.created_at : ''),
              ok: last.ok === true,
              turns: turns.length,
              judgePassed: typeof last.judge_passed === 'boolean' ? last.judge_passed : null,
            });
          }
        }

        // 2) Legacy single runs (no thread) -> one-turn entries, reopened by dir.
        for (const name of readdirSync(OUTPUT_ROOT)) {
          if (name.startsWith('iter_') || name === 'threads') continue;
          const dir = join(OUTPUT_ROOT, name);
          if (claimedRunDirs.has(dir)) continue;   // already shown under a thread
          let st;
          try { st = statSync(dir); } catch { continue; }
          if (!st.isDirectory()) continue;

          const result = readJson(join(dir, 'result.json'));
          if (!result) continue;
          const meta = readJson(join(dir, 'run.json')) ?? {};
          // Skip runs that already belong to a thread (thread is set in run.json).
          if (typeof meta.thread === 'string' && meta.thread) continue;
          const prompt = typeof meta.prompt === 'string' ? meta.prompt : '';
          const judge = (result.judge as Record<string, unknown> | null) ?? null;

          entries.push({
            threadId: '',
            run_dir: dir,
            title: prompt || slugToTitle(basename(dir)),
            prompt,
            model: typeof meta.model === 'string' ? meta.model : '',
            maxIters: typeof meta.max_iters === 'number' ? meta.max_iters : 2,
            created_at: typeof meta.created_at === 'string'
              ? meta.created_at : st.mtime.toISOString(),
            ok: result.ok === true,
            turns: 1,
            judgePassed: judge && typeof judge.passed === 'boolean' ? judge.passed : null,
          });
        }

        entries.sort((a, b) => (b.created_at || '').localeCompare(a.created_at || ''));
        return json({ runs: entries.slice(0, 20) }, 200);
      },
    },
  },
});
