import { createFileRoute } from '@tanstack/react-router';
import { corsHeaders, json, methodNotAllowed, preflight } from '@/server/api';
import { spawn } from 'node:child_process';
import { readFileSync, existsSync } from 'node:fs';
import { resolve, join, sep } from 'node:path';

// DEV bridge: serve a finished maker2 run's artifacts to the editor canvas.
//   GET ?dir=<run_dir>              -> the assembled, colored URDF as ONE .glb
//   GET ?dir=<run_dir>&file=scad    -> the generated model.scad as text/plain
// The .glb is what the orbitable solid canvas renders; the .scad is shown on the
// left. NO auth (dev/test). maker2 lives at the repo root; worker cwd is one up.
const REPO_ROOT = resolve(process.cwd(), '..');
const OUTPUT_ROOT = resolve(REPO_ROOT, 'output');

// Only ever read inside REPO_ROOT/output — `dir` comes from a query param, so
// reject anything that escapes (path traversal / absolute elsewhere).
function safeRunDir(dir: string | null): string | null {
  if (!dir) return null;
  const abs = resolve(dir);
  if (abs !== OUTPUT_ROOT && !abs.startsWith(OUTPUT_ROOT + sep))
    return null;
  return abs;
}

export const Route = createFileRoute('/api/run-maker2-glb')({
  server: {
    handlers: {
      POST: methodNotAllowed,
      OPTIONS: preflight,
      GET: async ({ request }) => {
        const url = new URL(request.url);
        const runDir = safeRunDir(url.searchParams.get('dir'));
        if (!runDir) return json({ error: 'invalid dir' }, 400);

        // Text branch: hand back the generated SCAD for the left panel.
        if (url.searchParams.get('file') === 'scad') {
          const scadPath = join(runDir, 'model.scad');
          if (!existsSync(scadPath)) return json({ error: 'no model.scad' }, 404);
          return new Response(readFileSync(scadPath, 'utf-8'), {
            headers: { ...corsHeaders, 'Content-Type': 'text/plain; charset=utf-8' },
          });
        }

        // GLB branch: assemble the URDF -> binary glTF via the maker2 CLI.
        const urdfPath = join(runDir, 'model.urdf');
        if (!existsSync(urdfPath)) return json({ error: 'no model.urdf' }, 404);

        const result = await new Promise<
          { ok: true; data: Buffer } | { ok: false; err: string }
        >((res) => {
          const p = spawn('python3',
            ['-m', 'maker2.export_glb', urdfPath, '--stdout'],
            { cwd: REPO_ROOT,
              env: { ...process.env, PYTHONIOENCODING: 'utf-8', PYTHONUTF8: '1' } });
          const chunks: Buffer[] = [];
          let err = '';
          p.stdout.on('data', (d: Buffer) => chunks.push(d));
          p.stderr.on('data', (d: Buffer) => (err += d.toString()));
          p.on('error', (e) => res({ ok: false, err: `spawn failed: ${String(e)}` }));
          p.on('close', (code) => {
            const data = Buffer.concat(chunks);
            // glTF binary magic — guards against a stderr-only failure slipping by.
            if (code === 0 && data.length > 4 && data.subarray(0, 4).toString() === 'glTF')
              res({ ok: true, data });
            else res({ ok: false, err: err.slice(-400) || `exit ${code}` });
          });
        });

        if (!result.ok) return json({ error: `glb export failed: ${result.err}` }, 500);
        return new Response(result.data, {
          headers: {
            ...corsHeaders,
            'Content-Type': 'model/gltf-binary',
            'Cache-Control': 'no-cache',
          },
        });
      },
    },
  },
});
