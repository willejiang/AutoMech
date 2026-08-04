import { createFileRoute } from '@tanstack/react-router';
import { corsHeaders, json, methodNotAllowed, preflight } from '@/server/api';
import { existsSync, readFileSync } from 'node:fs';
import { resolve, sep } from 'node:path';

// DEV bridge: REPLAY one past maker2 run. run-maker2-stream tees every SSE event it
// sends to output/threads/<id>/events.ndjson; this hands that log back so opening a
// historical run RENDERS it instead of spawning python and re-running the whole
// pipeline (which was destroying the very run the user clicked on). NO auth.
const REPO_ROOT = process.cwd();
const THREADS_ROOT = resolve(REPO_ROOT, 'output', 'threads');

function safeThreadDir(id: string | null): string | null {
  if (!id) return null;
  const abs = resolve(THREADS_ROOT, id);
  if (abs !== THREADS_ROOT && !abs.startsWith(THREADS_ROOT + sep)) return null;
  return abs;
}

export const Route = createFileRoute('/api/maker2-replay')({
  server: {
    handlers: {
      POST: methodNotAllowed,
      OPTIONS: preflight,
      GET: async ({ request }) => {
        const url = new URL(request.url);
        const dir = safeThreadDir(url.searchParams.get('id'));
        if (!dir) return json({ error: 'invalid thread id' }, 400);

        const path = resolve(dir, 'events.ndjson');
        // A missing log is NOT an error: runs from before this existed, or a run
        // that never started, simply have nothing to replay. The client treats an
        // empty `events` as "no recording" and shows the thread summary instead.
        if (!existsSync(path)) return json({ events: [], recorded: false });

        let events: unknown[] = [];
        try {
          events = readFileSync(path, 'utf-8')
            .split('\n')
            .filter((l) => l.trim())
            .map((l) => {
              try { return JSON.parse(l); } catch { return null; }
            })
            .filter(Boolean);
        } catch {
          return json({ error: 'events.ndjson unreadable', events: [], recorded: false }, 500);
        }

        return new Response(JSON.stringify({ events, recorded: true }), {
          headers: { ...corsHeaders, 'Content-Type': 'application/json' },
        });
      },
    },
  },
});
