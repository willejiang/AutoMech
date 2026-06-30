import { createFileRoute } from '@tanstack/react-router';
import { json, methodNotAllowed, preflight } from '@/server/api';
import { spawn } from 'node:child_process';
import { resolve } from 'node:path';

// DEV bridge: frontend posts a PROMPT + selected model -> shell to maker2
// (manager -> cadam SCAD worker -> articulated URDF -> appearance judge ->
// optional PyBullet physics). NO auth (dev/test). maker2 produces the URDF, so
// there is NO separate urdf_author step. maker2 is a package at the repo root;
// the worker dev server runs from worker/, so cwd is one level up.
const REPO_ROOT = resolve(process.cwd(), '..');

type Body = { prompt?: string; model?: string; physics?: boolean };

export const Route = createFileRoute('/api/run-maker2')({
  server: {
    handlers: {
      GET: methodNotAllowed,
      OPTIONS: preflight,
      POST: async ({ request }) => {
        const body = (await request.json()) as Body;
        if (!body.prompt) return json({ error: 'need prompt' }, 400);

        const args = ['-m', 'maker2.run', body.prompt, '--json', '--allow-partial'];
        if (body.model) args.push('--model', body.model);
        if (body.physics) args.push('--physics');

        const result = await new Promise((res) => {
          const p = spawn('python3', args, { cwd: REPO_ROOT });
          let out = '';
          p.stdout.on('data', (d) => (out += d));
          p.stderr.on('data', (d) => (out += d));
          p.on('close', () => {
            // run.py prints exactly one `RESULT_JSON:{...}` line at the end.
            const m = out.match(/RESULT_JSON:(\{.*\})\s*$/m);
            if (m) {
              try {
                res(JSON.parse(m[1]));
                return;
              } catch {
                /* fall through to error */
              }
            }
            res({ ok: false,
                  error: 'maker2 produced no parseable result: '
                    + (out.trim().slice(-400) || '(no output — python3 not found?)'),
                  log: out.slice(-2000) });
          });
          p.on('error', (e) =>
            res({ ok: false, error: `spawn failed: ${String(e)}` }));
        });
        return json(result);
      },
    },
  },
});
