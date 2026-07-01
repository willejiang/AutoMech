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

        // JSON branch: the saved verdict, for reopening a finished run read-only.
        if (url.searchParams.get('file') === 'result') {
          const resultPath = join(runDir, 'result.json');
          if (!existsSync(resultPath)) return json({ error: 'no result.json' }, 404);
          return new Response(readFileSync(resultPath, 'utf-8'), {
            headers: { ...corsHeaders, 'Content-Type': 'application/json' },
          });
        }

        // VIDEO branch: a physics test's recorded MP4, with HTTP Range so the
        // <video> element can seek/scrub. ?test=<i> picks the test (default 0).
        // Read into a Buffer and slice (the clips are tiny, ~30KB): a Node stream
        // cast to a web ReadableStream doesn't play reliably in the browser.
        if (url.searchParams.get('file') === 'video') {
          const t = Number(url.searchParams.get('test') ?? '0');
          const idx = Number.isFinite(t) && t >= 0 ? Math.floor(t) : 0;
          const mp4 = join(runDir, 'physics', `test_${idx}`, 'model.mp4');
          if (!existsSync(mp4)) return json({ error: 'no recording' }, 404);
          const buf = readFileSync(mp4);
          const size = buf.length;
          const range = request.headers.get('range');
          const baseHeaders: Record<string, string> = {
            ...corsHeaders,
            'Content-Type': 'video/mp4',
            'Accept-Ranges': 'bytes',
            'Cache-Control': 'no-cache',
          };
          if (range) {
            const m = /bytes=(\d*)-(\d*)/.exec(range);
            const start = m && m[1] ? parseInt(m[1], 10) : 0;
            const end = m && m[2] ? parseInt(m[2], 10) : size - 1;
            if (start >= size || end >= size || start > end) {
              return new Response(null, {
                status: 416,
                headers: { ...baseHeaders, 'Content-Range': `bytes */${size}` },
              });
            }
            const slice = buf.subarray(start, end + 1);
            return new Response(slice, {
              status: 206,
              headers: {
                ...baseHeaders,
                'Content-Range': `bytes ${start}-${end}/${size}`,
                'Content-Length': String(slice.length),
              },
            });
          }
          return new Response(buf, {
            headers: { ...baseHeaders, 'Content-Length': String(size) },
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
