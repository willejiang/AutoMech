import { createFileRoute } from '@tanstack/react-router';
import { corsHeaders, json, methodNotAllowed, preflight } from '@/server/api';
import { spawn } from 'node:child_process';
import { mkdtempSync, writeFileSync, readFileSync, existsSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';

// DEV bridge: frontend posts the generated .scad -> shell to the orchestrator
// (render -> visual gate -> URDF author -> PyBullet test set) -> verdict.
// NO auth on purpose (dev/test, no Supabase). PyBullet needs no GPU.
const ORCH = resolve(process.cwd(), '../orchestrator');

export const Route = createFileRoute('/api/run-physics-test')({
  server: {
    handlers: {
      GET: methodNotAllowed,
      OPTIONS: preflight,
      POST: async ({ request }) => {
        const body = (await request.json()) as { scad?: string; task?: string };
        if (!body.scad || !body.task) return json({ error: 'need scad+task' }, 400);

        const dir = mkdtempSync(join(tmpdir(), 'phys-'));
        const scadPath = join(dir, 'model.scad');
        writeFileSync(scadPath, body.scad, 'utf-8');
        const runId = 'fe-' + Date.now();

        const verdict = await new Promise((res) => {
          const p = spawn('python3', ['automech_loop.py', '--scad', scadPath,
            '--task', body.task!, '--max-iters', '1', '--run-id', runId],
            { cwd: ORCH });
          let log = '';
          p.stdout.on('data', (d) => (log += d));
          p.stderr.on('data', (d) => (log += d));
          p.on('close', () => {
            const rj = join(ORCH, 'runs', runId, 'result.json');
            if (existsSync(rj)) res(JSON.parse(readFileSync(rj, 'utf-8')));
            else res({ passed: false, summary: 'loop produced no result', failures: [], log: log.slice(-2000) });
          });
        });
        return json(verdict);
      },
    },
  },
});
