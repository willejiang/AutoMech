import { createFileRoute } from '@tanstack/react-router';
import { corsHeaders, methodNotAllowed, preflight } from '@/server/api';
import { uploadPath } from './upload-image';
import { spawn } from 'node:child_process';
import { createWriteStream, existsSync, mkdirSync, type WriteStream } from 'node:fs';
import { resolve, sep } from 'node:path';

// DEV bridge (SSE): the editor opens an EventSource here and we stream the
// Python run's stdout line-by-line so the UI can show real per-stage progress
// (agent -> build -> judge -> physics) instead of a dead
// spinner. The buffered sibling (run-maker2.ts) stays for the parameter-panel
// button. NO auth (dev/test). EventSource can only GET + can't set headers, so the
// prompt/model/iters ride in the query string. maker2 is a package at the repo
// root, which is also where this app runs from.
const REPO_ROOT = process.cwd();

// maker2/run.py prints these stage markers; the client classifies a line into a
// stage group by these prefixes. Anything else is treated as verbose `log`.
// Hierarchy (boss) mode adds [boss]/[sub:<id>]/[assembler]/[precheck]/[aggregate]
// and reference tools add [tool]. `[sub:` (no closing bracket) catches [sub:crank].
const STAGE_PREFIXES = [
  '[run]', '[1/3]', '[2/3]', '[3/3]', '[judge]', '[loop]', '[physics]',
  '[done]', '=====', 'RESULT:',
  '[boss]', '[sub:', '[assembler]', '[precheck]', '[assembled]', '[aggregate]', '[tool]',
  '[single-agent]', '[step->model]',
];

function sse(event: string, data: unknown): string {
  return `event: ${event}\ndata: ${JSON.stringify(data)}\n\n`;
}

// REPLAY LOG. Every event this endpoint sends is also appended to
// output/threads/<thread>/events.ndjson, so opening the run later can REPLAY it
// instead of spawning python again. Without this the only record of a run is
// thread.json's one-line summary + the run_dir artifacts, and the frontend has
// nothing to render from — which is exactly why opening a past run re-ran it.
const OUTPUT_ROOT = resolve(REPO_ROOT, 'output');
const THREADS_ROOT = resolve(OUTPUT_ROOT, 'threads');

function openEventLog(threadId: string | undefined): WriteStream | null {
  if (!threadId) return null;
  const dir = resolve(THREADS_ROOT, threadId);
  // A thread id is one uuid-ish segment; reject anything that escapes the root.
  if (dir !== THREADS_ROOT && !dir.startsWith(THREADS_ROOT + sep)) return null;
  try {
    mkdirSync(dir, { recursive: true });
    return createWriteStream(resolve(dir, 'events.ndjson'), { flags: 'w' });
  } catch {
    return null;   // a log we can't write must never break the live run
  }
}

export const Route = createFileRoute('/api/run-maker2-stream')({
  server: {
    handlers: {
      POST: methodNotAllowed,
      OPTIONS: preflight,
      GET: async ({ request }) => {
        const url = new URL(request.url);
        const prompt = url.searchParams.get('prompt');
        const model = url.searchParams.get('model') ?? undefined;
        const iters = url.searchParams.get('iters') ?? undefined;
        const threadId = url.searchParams.get('thread') ?? undefined;
        const refineMessage = url.searchParams.get('refine') ?? undefined;
        const deep = url.searchParams.get('deep');   // "1" | "0" | null (unset)
        if (!prompt) {
          return new Response(sse('error', { error: 'need prompt' }), {
            headers: { ...corsHeaders, 'Content-Type': 'text/event-stream' },
          });
        }

        const args = ['-u', '-m', 'maker2.run', prompt, '--json', '--allow-partial',
          '--physics'];
        if (model) args.push('--model', model);
        if (iters && Number(iters) > 0) args.push('--max-iters', iters);
        if (threadId) args.push('--thread', threadId);
        if (refineMessage) args.push('--refine-message', refineMessage);
        // Pipeline mode. On this experiment branch the DEFAULT is the single-agent
        // text-to-cad path: ONE agent authors the WHOLE machine as one build123d script
        // (no boss / no per-sub managers / no assembler), refines it against a rigid-conflict
        // self-check, then runs physics. Pass ?mode=hierarchy to use the older boss+manager_py
        // pipeline instead.
        const mode = url.searchParams.get('mode') ?? 'single-agent';
        if (mode === 'hierarchy') {
          args.push('--hierarchy');
          args.push('--manager-py');
          args.push('--debugger-read-tools');
          args.push('--precheck-warn-only');
          args.push('--kb');
          args.push('--solver');
          if (url.searchParams.get('web') === '1') args.push('--web');
          if (deep === '1') args.push('--deep-think');
          else if (deep === '0') args.push('--no-deep-think');
        } else {
          // Single-agent text-to-cad (default on this branch). It still gets the same
          // research tools as the pipeline path: --kb (local KB) always, --web when the
          // UI toggle set web=1, and the deep-think flag — otherwise the agent authors
          // the whole drivetrain from memory and guesses gear math / horology it never
          // looked up.
          args.push('--single-agent');
          args.push('--kb');
          if (url.searchParams.get('web') === '1') args.push('--web');
          if (deep === '1') args.push('--deep-think');
          else if (deep === '0') args.push('--no-deep-think');
          // A reference image, uploaded separately (EventSource cannot POST a body). Only
          // the single-agent path reads it — the hierarchy path's boss takes an image too,
          // but nothing here has ever passed one, so wiring that is a separate change.
          const img = url.searchParams.get('image');
          if (img) {
            const p = uploadPath(img);
            if (p) args.push('--image', p);
            else return new Response(sse('error', { error: `unknown image id: ${img}` }), {
              headers: { ...corsHeaders, 'Content-Type': 'text/event-stream' },
            });
          }
        }

        const stream = new ReadableStream<Uint8Array>({
          start(controller) {
            const enc = new TextEncoder();
            let closed = false;
            const log = openEventLog(threadId);
            const raw = (s: string) => {
              if (closed) return;
              try { controller.enqueue(enc.encode(s)); }
              catch { closed = true; }   // controller closed under us
            };
            // Send an event AND record it for replay. `open`/`end` are recorded
            // too so a replay reproduces the same event sequence the live client saw.
            const send = (event: string, data: unknown) => {
              raw(sse(event, data));
              try { log?.write(JSON.stringify({ event, data }) + '\n'); }
              catch { /* a broken log must not kill the run */ }
            };
            const close = () => {
              if (closed) return;
              closed = true;
              try { log?.end(); } catch { /* ignore */ }
              try { controller.close(); } catch { /* already closed */ }
            };

            // LLM credentials saved through /api/llm-settings. maker2/config.py
            // resolves defaults < this JSON < env vars, so pointing it at the file is
            // the whole integration — nothing about the key travels in the query
            // string, and an unset file just leaves the env/default gateway in place.
            const llmConfig = resolve(REPO_ROOT, '.automech', 'llm.json');
            const p = spawn(process.env.PYTHON_BIN || 'python3', args, {
              cwd: REPO_ROOT,
              env: { ...process.env, PYTHONIOENCODING: 'utf-8', PYTHONUTF8: '1',
                PYTHONUNBUFFERED: '1',
                ...(existsSync(llmConfig)
                  ? { WORKFLOW4FREECAD_CONFIG: llmConfig }
                  : {}) },
            });

            // Flush an immediate comment + open event so the client (and any
            // buffering proxy) sees bytes before python finishes booting.
            raw(`: open\n\n`);
            send('open', { prompt });

            let buf = '';        // partial-line carryover across chunks
            let tail = '';       // last bytes, surfaced if the run dies w/o a result
            let gotResult = false;

            const onData = (chunk: Buffer) => {
              buf += chunk.toString();
              const lines = buf.split('\n');
              buf = lines.pop() ?? '';   // keep the unterminated remainder
              for (const line of lines) {
                tail = line;
                const m = line.match(/^RESULT_JSON:(\{.*\})\s*$/);
                if (m) {
                  try {
                    send('result', JSON.parse(m[1]));
                    gotResult = true;
                  } catch {
                    send('log', { raw: line });
                  }
                  continue;
                }
                // Per-iteration artifacts: a renderable model (after judge) or a sim
                // recording (after physics) is ready NOW. The UI loads it immediately
                // so the canvas/video track the loop instead of waiting for the end.
                const am = line.match(/^ARTIFACT_JSON:(\{.*\})\s*$/);
                if (am) {
                  try {
                    send('artifact', JSON.parse(am[1]));
                  } catch {
                    send('log', { raw: line });
                  }
                  continue;
                }
                const isStage = STAGE_PREFIXES.some((pre) => line.startsWith(pre));
                send(isStage ? 'stage' : 'log', { raw: line });
              }
            };

            p.stdout.on('data', onData);
            p.stderr.on('data', onData);
            p.on('error', (e) => {
              send('error', { error: `spawn failed: ${String(e)}` });
              close();
            });
            p.on('close', () => {
              if (buf.trim()) send('log', { raw: buf });
              if (!gotResult) {
                send('error', {
                  error: 'maker2 produced no result',
                  tail: tail.slice(-400),
                });
              }
              send('end', {});
              close();
            });

            // If the browser navigates away, kill the Python run.
            request.signal.addEventListener('abort', () => {
              try { p.kill(); } catch { /* already gone */ }
              close();
            });
          },
        });

        return new Response(stream, {
          headers: {
            ...corsHeaders,
            'Content-Type': 'text/event-stream',
            'Cache-Control': 'no-cache, no-transform',
            'X-Accel-Buffering': 'no',
            Connection: 'keep-alive',
          },
        });
      },
    },
  },
});
