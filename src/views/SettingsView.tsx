import { useEffect, useState } from 'react';
import { Check, Loader2, X } from 'lucide-react';
import { apiJson } from '@/services/api';
import { cn } from '@/lib/utils';

type SettingsState = {
  base_url: string;
  model: string;
  has_key: boolean;
  key_hint: string;
  path?: string;
};

type ProbeResult = { ok: boolean; detail: string };

export function SettingsView() {
  const [cfg, setCfg] = useState<SettingsState | null>(null);
  const [baseUrl, setBaseUrl] = useState('');
  const [model, setModel] = useState('');
  const [apiKey, setApiKey] = useState('');
  const [busy, setBusy] = useState<'save' | 'probe' | null>(null);
  const [probeResult, setProbeResult] = useState<ProbeResult | null>(null);

  useEffect(() => {
    apiJson<SettingsState>('llm-settings')
      .then((s) => {
        setCfg(s);
        setBaseUrl(s.base_url);
        setModel(s.model);
      })
      .catch(() => setCfg(null));
  }, []);

  const post = async (body: Record<string, unknown>, kind: 'save' | 'probe') => {
    setBusy(kind);
    setProbeResult(null);
    try {
      const r = await apiJson<SettingsState & Partial<ProbeResult>>('llm-settings', {
        method: 'POST',
        body: JSON.stringify(body),
      });
      if (r.base_url !== undefined) setCfg(r);
      // The key is never echoed back, so clear the field once it is stored.
      if (typeof body.api_key === 'string' && body.api_key) setApiKey('');
      if (typeof r.ok === 'boolean') setProbeResult({ ok: r.ok, detail: r.detail ?? '' });
    } catch (e) {
      setProbeResult({ ok: false, detail: String(e) });
    } finally {
      setBusy(null);
    }
  };

  return (
    <div className="mx-auto max-w-2xl px-6 py-10">
      <h1 className="text-xl font-semibold tracking-tight">Settings</h1>
      <p className="mb-8 mt-1 text-sm text-muted-foreground">
        Which model runs the pipeline. Stored on this machine, never in the browser.
      </p>

      <div className="space-y-5 rounded-lg border border-border bg-card p-5">
        <Field
          label="Gateway URL"
          hint="OpenAI-compatible endpoint; the /v1 suffix is required."
        >
          <input
            value={baseUrl}
            onChange={(e) => setBaseUrl(e.target.value)}
            placeholder="http://127.0.0.1:8313/v1"
            className="w-full rounded border border-border bg-transparent px-2.5 py-1.5 text-sm outline-none focus:border-adam-blue"
          />
        </Field>

        <Field label="Model">
          <input
            value={model}
            onChange={(e) => setModel(e.target.value)}
            placeholder="claude-opus-4.8"
            className="w-full rounded border border-border bg-transparent px-2.5 py-1.5 text-sm outline-none focus:border-adam-blue"
          />
        </Field>

        <Field
          label="API key"
          hint={
            cfg?.has_key
              ? `A key is stored (${cfg.key_hint}). Leave blank to keep it.`
              : 'No key stored yet.'
          }
        >
          <input
            type="password"
            value={apiKey}
            onChange={(e) => setApiKey(e.target.value)}
            placeholder={cfg?.has_key ? '••••••••' : 'sk-…'}
            autoComplete="off"
            className="w-full rounded border border-border bg-transparent px-2.5 py-1.5 text-sm outline-none focus:border-adam-blue"
          />
        </Field>

        <div className="flex items-center gap-2 pt-1">
          <button
            onClick={() =>
              post({ base_url: baseUrl, model, api_key: apiKey || undefined }, 'save')
            }
            disabled={busy !== null}
            className="rounded bg-primary px-3 py-1.5 text-sm text-primary-foreground disabled:opacity-40"
          >
            {busy === 'save' ? 'Saving…' : 'Save'}
          </button>

          <button
            onClick={() =>
              post(
                {
                  base_url: baseUrl,
                  model,
                  api_key: apiKey || undefined,
                  probe: true,
                },
                'probe',
              )
            }
            disabled={busy !== null}
            className="flex items-center gap-1.5 rounded border border-border px-3 py-1.5 text-sm hover:bg-accent/40 disabled:opacity-40"
          >
            {busy === 'probe' && <Loader2 size={13} className="animate-spin" />}
            Save &amp; test
          </button>

          {cfg?.has_key && (
            <button
              onClick={() => post({ api_key: null }, 'save')}
              disabled={busy !== null}
              className="ml-auto text-xs text-muted-foreground hover:text-destructive disabled:opacity-40"
            >
              Clear key
            </button>
          )}
        </div>

        {probeResult && (
          <div
            className={cn(
              'flex items-start gap-2 rounded border px-3 py-2 text-xs',
              probeResult.ok
                ? 'border-green-600/40 bg-green-500/10 text-green-700'
                : 'border-red-500/40 bg-red-500/10 text-red-700',
            )}
          >
            {probeResult.ok ? (
              <Check size={14} className="mt-px shrink-0" />
            ) : (
              <X size={14} className="mt-px shrink-0" />
            )}
            <span className="break-all">{probeResult.detail}</span>
          </div>
        )}
      </div>

      {cfg?.path && (
        <p className="mt-3 text-[11px] text-muted-foreground">
          Written to <code className="break-all">{cfg.path}</code>, which is
          gitignored. Environment variables (FREECAD_AI_BASE_URL and friends) still
          win over this file.
        </p>
      )}
    </div>
  );
}

function Field({
  label,
  hint,
  children,
}: {
  label: string;
  hint?: string;
  children: React.ReactNode;
}) {
  return (
    <label className="block">
      <div className="mb-1 text-xs font-medium">{label}</div>
      {children}
      {hint && <div className="mt-1 text-[11px] text-muted-foreground">{hint}</div>}
    </label>
  );
}
