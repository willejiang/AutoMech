import { createFileRoute } from '@tanstack/react-router';
import { corsHeaders, methodNotAllowed, preflight } from '@/server/api';
import { spawn } from 'node:child_process';
import { resolve } from 'node:path';

// DEV bridge (SSE): the maker2 editor opens an EventSource here and we stream the
// Python run's stdout line-by-line so the UI can show real per-stage progress
// (manager -> cadam SCAD worker -> URDF -> judge -> physics) instead of a dead
// spinner. The buffered sibling (run-maker2.ts) stays for the parameter-panel
// button. NO auth (dev/test). EventSource can only GET + can't set headers, so the
// prompt/model/iters ride in the query string. maker2 is a package at the repo
// root; the worker dev server runs from worker/, so cwd is one level up.
const REPO_ROOT = resolve(process.cwd(), '..');

// maker2/run.py prints these stage markers; the client classifies a line into a
// stage group by these prefixes. Anything else is treated as verbose `log`.
// Hierarchy (boss) mode adds [boss]/[sub:<id>]/[assembler]/[precheck]/[aggregate]
// and reference tools add [tool]. `[sub:` (no closing bracket) catches [sub:crank].
const STAGE_PREFIXES = [
  '[run]', '[1/3]', '[2/3]', '[3/3]', '[judge]', '[loop]', '[physics]',
  '[done]', '=====', 'RESULT:',
  '[boss]', '[sub:', '[assembler]', '[precheck]', '[assembled]', '[aggregate]', '[tool]',
];

function sse(event: string, data: unknown): string {
  return `event: ${event}\ndata: ${JSON.stringify(data)}\n\n`;
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
        // Both fresh runs AND refines go through the BOSS hierarchy. A refine re-plans
        // the SAME machine with the change (run_boss loads the prior plan from the
        // session and reuses unchanged subassemblies). The old single-manager
        // --prior-model path is not used for hierarchy refines.
        args.push('--hierarchy');
        // 方案B: the manager authors a parametric CadQuery module that derives every
        // coordinate from a boss-authored `params` module (no LLM-typed coordinates), and
        // the libslvs solver stitches subs. Both flags are required for this pipeline.
        args.push('--manager-py');
        args.push('--solver');
        // Local offline knowledge base (maker2/kb): the boss/manager/worker retrieve
        // this project's output-format conventions + worked examples + prior passing
        // designs via a kb_search tool. Always on — it is offline and cheap, and a
        // missing/empty index degrades to a no-op (never fails the run). Requires a
        // one-time `python -m maker2.kb.ingest` + the MiniLM cache on a fresh machine.
        args.push('--kb');
        // Web-search reference lookup (keyless), when the client asks for it.
        if (url.searchParams.get('web') === '1') args.push('--web');
        // Deep-think toggle: ON -> CadQuery worker + full debugger; OFF -> OpenSCAD +
        // slim debugger. Only forwarded when the client set it explicitly.
        if (deep === '1') args.push('--deep-think');
        else if (deep === '0') args.push('--no-deep-think');

        const stream = new ReadableStream<Uint8Array>({
          start(controller) {
            const enc = new TextEncoder();
            let closed = false;
            const send = (s: string) => {
              if (closed) return;
              try { controller.enqueue(enc.encode(s)); }
              catch { closed = true; }   // controller closed under us
            };
            const close = () => {
              if (closed) return;
              closed = true;
              try { controller.close(); } catch { /* already closed */ }
            };

            const p = spawn('python3', args, {
              cwd: REPO_ROOT,
              env: { ...process.env, PYTHONIOENCODING: 'utf-8', PYTHONUTF8: '1',
                PYTHONUNBUFFERED: '1' },
            });

            // Flush an immediate comment + open event so the client (and any
            // buffering proxy) sees bytes before python finishes booting.
            send(`: open\n\n`);
            send(sse('open', { prompt }));

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
                    send(sse('result', JSON.parse(m[1])));
                    gotResult = true;
                  } catch {
                    send(sse('log', { raw: line }));
                  }
                  continue;
                }
                // Per-iteration artifacts: a renderable model (after judge) or a sim
                // recording (after physics) is ready NOW. The UI loads it immediately
                // so the canvas/video track the loop instead of waiting for the end.
                const am = line.match(/^ARTIFACT_JSON:(\{.*\})\s*$/);
                if (am) {
                  try {
                    send(sse('artifact', JSON.parse(am[1])));
                  } catch {
                    send(sse('log', { raw: line }));
                  }
                  continue;
                }
                const isStage = STAGE_PREFIXES.some((pre) => line.startsWith(pre));
                send(sse(isStage ? 'stage' : 'log', { raw: line }));
              }
            };

            p.stdout.on('data', onData);
            p.stderr.on('data', onData);
            p.on('error', (e) => {
              send(sse('error', { error: `spawn failed: ${String(e)}` }));
              close();
            });
            p.on('close', () => {
              if (buf.trim()) send(sse('log', { raw: buf }));
              if (!gotResult) {
                send(sse('error', {
                  error: 'maker2 produced no result',
                  tail: tail.slice(-400),
                }));
              }
              send(sse('end', {}));
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
