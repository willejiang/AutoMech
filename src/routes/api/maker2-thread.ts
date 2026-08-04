import { createFileRoute } from '@tanstack/react-router';
import { corsHeaders, json, methodNotAllowed, preflight } from '@/server/api';
import { readFileSync, existsSync } from 'node:fs';
import { resolve, join, sep } from 'node:path';

// DEV bridge: read one maker2 conversation thread from disk so the editor can
// render all past turns and know which model to show + which prior model to feed
// the next refine. thread.json is written by maker2/run.py (--thread). NO auth.
const REPO_ROOT = process.cwd();
const OUTPUT_ROOT = resolve(REPO_ROOT, 'output');
const THREADS_ROOT = resolve(OUTPUT_ROOT, 'threads');

type Turn = {
  message: string;
  run_dir: string;
  render_dir: string;
  ok: boolean;
  hard_failed: boolean;
  judge_passed: boolean | null;
  ts: string;
  // Resolved server-side for the client so refine can feed the prior model back:
  priorModelPath: string;   // <run_dir>/kinematic_model.json (may not exist on a crash)
};

// A thread id is a plain uuid segment; reject anything that would escape.
function safeThreadDir(id: string | null): string | null {
  if (!id) return null;
  const abs = resolve(THREADS_ROOT, id);
  if (abs !== THREADS_ROOT && !abs.startsWith(THREADS_ROOT + sep)) return null;
  return abs;
}

export const Route = createFileRoute('/api/maker2-thread')({
  server: {
    handlers: {
      POST: methodNotAllowed,
      OPTIONS: preflight,
      GET: async ({ request }) => {
        const url = new URL(request.url);
        const dir = safeThreadDir(url.searchParams.get('id'));
        if (!dir) return json({ error: 'invalid thread id' }, 400);

        const tpath = join(dir, 'thread.json');
        if (!existsSync(tpath)) return json({ error: 'no such thread', turns: [] }, 404);

        let doc: { id?: string; model?: string; turns?: Turn[] };
        try { doc = JSON.parse(readFileSync(tpath, 'utf-8')); }
        catch { return json({ error: 'thread.json unreadable', turns: [] }, 500); }

        const turns = (doc.turns ?? []).map((t) => ({
          ...t,
          priorModelPath: t.run_dir ? join(t.run_dir, 'kinematic_model.json') : '',
        }));

        return json({ id: doc.id, model: doc.model ?? '', turns });
      },
    },
  },
});
