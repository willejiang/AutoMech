import { createFileRoute } from '@tanstack/react-router';
import { spawn } from 'node:child_process';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { isRecord, json, methodNotAllowed, preflight } from '@/server/api';

// WHERE THE LLM CREDENTIALS LIVE. maker2/config.py already resolves settings as
// defaults < JSON file < env vars < explicit overrides, and reads that JSON file from
// $WORKFLOW4FREECAD_CONFIG — so the UI does not need a new mechanism, it just needs to
// write that file. Keeping the key server-side (rather than in localStorage) means it
// never has to travel in a query string to the SSE route, and it is gitignored.
const REPO_ROOT = process.cwd();
const CONFIG_PATH = resolve(REPO_ROOT, '.automech', 'llm.json');

type LlmConfig = {
  base_url?: string;
  api_key?: string;
  model?: string;
};

function readConfig(): LlmConfig {
  try {
    if (!existsSync(CONFIG_PATH)) return {};
    const parsed: unknown = JSON.parse(readFileSync(CONFIG_PATH, 'utf8'));
    return isRecord(parsed) ? (parsed as LlmConfig) : {};
  } catch {
    return {};
  }
}

// The key is write-only over the wire: the UI shows whether one is set and its last
// four characters, never the secret itself.
function redact(cfg: LlmConfig) {
  const key = cfg.api_key ?? '';
  return {
    base_url: cfg.base_url ?? '',
    model: cfg.model ?? '',
    has_key: Boolean(key),
    key_hint: key ? `…${key.slice(-4)}` : '',
  };
}

// Does the gateway actually answer? A settings page that only stores strings is a
// guess; this is the difference between "saved" and "works". Asked through Python so
// it exercises the same client the run will use — including whatever proxy/TLS
// environment that process has.
async function probe(cfg: LlmConfig): Promise<{ ok: boolean; detail: string }> {
  const code = `
import json, os, sys
sys.path.insert(0, os.getcwd())
from maker2.config import Settings
s = Settings.load()
try:
    c = s.make_client(64)
    txt = str(c.test_connection())[:60]
    print(json.dumps({"ok": True, "detail": s.model + " replied: " + txt}))
except Exception as e:
    print(json.dumps({"ok": False, "detail": f"{type(e).__name__}: {e}"[:300]}))
`;
  return new Promise((done) => {
    const py = spawn(process.env.PYTHON_BIN || 'python3', ['-c', code], {
      cwd: REPO_ROOT,
      env: {
        ...process.env,
        WORKFLOW4FREECAD_CONFIG: existsSync(CONFIG_PATH) ? CONFIG_PATH : '',
      },
    });
    let out = '';
    let err = '';
    py.stdout.on('data', (d) => (out += String(d)));
    py.stderr.on('data', (d) => (err += String(d)));
    const timer = setTimeout(() => py.kill(), 30_000);
    py.on('close', () => {
      clearTimeout(timer);
      const line = out.trim().split(/\r?\n/).pop() ?? '';
      try {
        const parsed: unknown = JSON.parse(line);
        if (isRecord(parsed)) {
          return done({
            ok: Boolean(parsed.ok),
            detail: String(parsed.detail ?? ''),
          });
        }
      } catch {
        /* fall through to the raw stderr, which says more than "bad json" */
      }
      done({ ok: false, detail: (err || out || 'no output').slice(-300) });
    });
  });
}

export const Route = createFileRoute('/api/llm-settings')({
  server: {
    handlers: {
      OPTIONS: () => preflight(),

      // Current settings, redacted, plus where they are stored.
      GET: () => json({ ...redact(readConfig()), path: CONFIG_PATH }),

      POST: async ({ request }) => {
        const body: unknown = await request.json().catch(() => null);
        if (!isRecord(body)) return json({ error: 'bad_request' }, 400);

        // `probe: true` tests the CURRENT saved settings without writing anything.
        if (body.probe === true && body.base_url === undefined) {
          return json(await probe(readConfig()));
        }

        const current = readConfig();
        const next: LlmConfig = {
          base_url:
            typeof body.base_url === 'string' ? body.base_url.trim() : current.base_url,
          model: typeof body.model === 'string' ? body.model.trim() : current.model,
          // An empty string means "leave the stored key alone" — the UI never sends
          // the existing secret back, so a blank field must not erase it. Send null
          // to clear it deliberately.
          api_key:
            body.api_key === null
              ? ''
              : typeof body.api_key === 'string' && body.api_key.trim()
                ? body.api_key.trim()
                : current.api_key,
        };

        try {
          mkdirSync(dirname(CONFIG_PATH), { recursive: true });
          writeFileSync(CONFIG_PATH, JSON.stringify(next, null, 2), {
            encoding: 'utf8',
            mode: 0o600,
          });
        } catch (e) {
          return json({ error: `could not write ${CONFIG_PATH}: ${String(e)}` }, 500);
        }

        const result = { ...redact(next), path: CONFIG_PATH };
        if (body.probe === true) {
          return json({ ...result, ...(await probe(next)) });
        }
        return json(result);
      },

      PUT: () => methodNotAllowed(),
      DELETE: () => methodNotAllowed(),
    },
  },
});
