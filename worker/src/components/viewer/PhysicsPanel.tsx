import { useEffect, useState } from 'react';
import { ChevronLeft, ChevronRight } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { apiUrl } from '@/services/api';
import { cn } from '@/lib/utils';

// The physics evaluation for the latest turn: what the driven test did (which
// input joint was turned, what downstream moved) + a playable recording of each
// test. Data comes straight from result.json's physics field — no extra fetch.

type PhysicsMetrics = {
  verdict?: string;
  test_kind?: string;
  input_joint?: string;
  input_travel?: number;
  watched?: Record<string, number>;
  moved_count?: number;
  watched_count?: number;
  exploded?: boolean;
  end_z?: number;
  max_tilt_deg?: number;
  max_drift?: number;
  min_base_z?: number;
};

type PhysicsTest = {
  name?: string;
  strategy?: string;
  verdict?: string;
  summary?: string;
  metrics?: PhysicsMetrics;
  video?: string | null;
};

export type PhysicsResult = {
  passed: boolean | null;
  verdict?: string;
  summary?: string;
  metrics?: PhysicsMetrics;
  video?: string | null;
  tests?: PhysicsTest[];
};

interface PhysicsPanelProps {
  physics: PhysicsResult;
  runDir: string;          // absolute run_dir, for the video route
  running?: boolean;       // the run is still building
}

export function PhysicsPanel({ physics, runDir, running }: PhysicsPanelProps) {
  const tests = physics.tests ?? [];
  const [active, setActive] = useState(0);
  const [videoErr, setVideoErr] = useState(false);
  const [blobUrl, setBlobUrl] = useState<string | null>(null);
  const idx = Math.min(active, Math.max(0, tests.length - 1));
  const test = tests[idx];
  const m = test?.metrics ?? physics.metrics ?? {};
  const driven = m.test_kind === 'driven_mechanism';
  const pass = (test?.verdict ?? physics.verdict) !== 'FAIL' && physics.passed !== false;
  const videoUrl = `${apiUrl('run-maker2-glb')}?dir=${encodeURIComponent(runDir)}&file=video&test=${idx}`;

  // Load the MP4 via fetch() -> blob URL rather than a direct <video src>. A
  // <video>'s own request carries Sec-Fetch-Dest: video, which the TanStack/Nitro
  // dev server routes to its static-asset resolver (bypassing this API handler ->
  // 404). A fetch() sends Sec-Fetch-Dest: empty and hits the handler correctly.
  useEffect(() => {
    setVideoErr(false);
    setBlobUrl(null);
    if (!test?.video || !runDir) return;
    let revoked = false;
    let objUrl: string | null = null;
    fetch(videoUrl)
      .then((r) => (r.ok ? r.blob() : Promise.reject(r.status)))
      .then((b) => {
        if (revoked) return;
        objUrl = URL.createObjectURL(b);
        setBlobUrl(objUrl);
      })
      .catch(() => { if (!revoked) setVideoErr(true); });
    return () => {
      revoked = true;
      if (objUrl) URL.revokeObjectURL(objUrl);
    };
  }, [idx, runDir, test?.video, videoUrl]);

  return (
    <div className="flex h-full flex-col overflow-hidden">
      <div className="flex items-center justify-between border-b border-adam-neutral-800 px-3 py-2">
        <div className="text-xs font-semibold uppercase tracking-wide text-adam-neutral-400">
          Physics
        </div>
        <span className={cn('rounded-full px-2 py-0.5 text-[10px] font-semibold',
          pass ? 'bg-green-900/40 text-green-300' : 'bg-red-900/40 text-red-300')}>
          {pass ? 'PASS' : 'FAIL'}
        </span>
      </div>

      <div className="flex min-h-0 flex-1 flex-col gap-3 overflow-auto p-3">
        {/* active test label + prev/next */}
        {tests.length > 0 && (
          <div className="flex items-center justify-between gap-2">
            <div className="min-w-0 text-xs text-adam-neutral-300">
              {tests.length > 1 && (
                <span className="text-adam-neutral-500">
                  Test {idx + 1}/{tests.length}:{' '}
                </span>
              )}
              <span className="font-medium">{test?.name ?? test?.strategy}</span>
            </div>
            {tests.length > 1 && (
              <div className="flex shrink-0 gap-1">
                <Button variant="ghost" size="icon" className="h-6 w-6"
                  disabled={idx === 0} onClick={() => setActive(idx - 1)}>
                  <ChevronLeft className="h-4 w-4" />
                </Button>
                <Button variant="ghost" size="icon" className="h-6 w-6"
                  disabled={idx >= tests.length - 1} onClick={() => setActive(idx + 1)}>
                  <ChevronRight className="h-4 w-4" />
                </Button>
              </div>
            )}
          </div>
        )}

        {/* recording */}
        <div className="overflow-hidden rounded border border-adam-neutral-800 bg-black">
          {test?.video ? (
            <div>
              {blobUrl ? (
                <video
                  key={blobUrl}
                  className="h-auto w-full"
                  controls
                  loop
                  playsInline
                  src={blobUrl}
                />
              ) : videoErr ? (
                <div className="p-3 text-center text-[11px] text-red-400">
                  Couldn’t load the recording.
                </div>
              ) : (
                <div className="flex aspect-video items-center justify-center text-xs text-adam-neutral-500">
                  Loading recording…
                </div>
              )}
            </div>
          ) : (
            <div className="flex aspect-video items-center justify-center p-4 text-center text-xs text-adam-neutral-500">
              {running
                ? 'Simulation recording will appear after the physics test.'
                : 'No recording for this test.'}
            </div>
          )}
        </div>

        {/* breakdown */}
        {driven ? (
          <div className="space-y-2 text-xs text-adam-neutral-300">
            <div>
              Drove <span className="font-mono text-adam-text-primary">{m.input_joint}</span>{' '}
              <span className="text-adam-text-primary">{fmt(m.input_travel)}</span> rad →{' '}
              <span className="text-adam-text-primary">
                {m.moved_count ?? 0}/{m.watched_count ?? 0}
              </span>{' '}
              downstream joints moved
            </div>
            {m.exploded && (
              <div className="rounded border border-red-800 bg-red-900/20 px-2 py-1 text-red-300">
                Jammed / exploded — the mechanism did not run cleanly.
              </div>
            )}
            {m.watched && Object.keys(m.watched).length > 0 && (
              <table className="w-full border-collapse text-[11px]">
                <tbody>
                  {Object.entries(m.watched).map(([j, v]) => (
                    <tr key={j} className="border-t border-adam-neutral-800/60">
                      <td className="truncate py-0.5 pr-2 font-mono text-adam-neutral-400">{j}</td>
                      <td className={cn('py-0.5 text-right tabular-nums',
                        v >= 0.05 ? 'text-green-400' : 'text-adam-neutral-500')}>
                        {fmt(v)} rad
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            )}
          </div>
        ) : (
          <div className="space-y-1 text-xs text-adam-neutral-300">
            <div className="text-adam-neutral-400">Stability test</div>
            <div>settled z = {fmt(m.end_z)} · tilt {fmt(m.max_tilt_deg)}° · drift {fmt(m.max_drift)} m</div>
          </div>
        )}

        {test?.summary && (
          <div className="border-t border-adam-neutral-800 pt-2 text-[11px] text-adam-neutral-400">
            {test.summary}
          </div>
        )}
      </div>
    </div>
  );
}

function fmt(n?: number): string {
  return typeof n === 'number' ? (Math.round(n * 1000) / 1000).toString() : '—';
}
