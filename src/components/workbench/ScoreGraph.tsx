// ScoreGraph — a hand-rolled SVG line chart of the physics score (y, 0..1) vs
// iteration (x). No chart dependency: just a polyline + a dot per iteration, with
// the best iteration highlighted. Responsive width, ~200px tall, readable on the
// dark workbench background.

type Point = { iter: number; score: number };

const H = 200;
const PAD_L = 34; // room for the y axis labels
const PAD_R = 12;
const PAD_T = 12;
const PAD_B = 24; // room for the x axis labels
// A fixed viewBox width; the SVG scales to its container via width="100%".
const VB_W = 480;

export function ScoreGraph({ points }: { points: Point[] }) {
  // De-dupe by iter (keep the last score seen for an iter) and sort ascending.
  const byIter = new Map<number, number>();
  for (const p of points) byIter.set(p.iter, p.score);
  const pts: Point[] = [...byIter.entries()]
    .map(([iter, score]) => ({ iter, score }))
    .sort((a, b) => a.iter - b.iter);

  const plotW = VB_W - PAD_L - PAD_R;
  const plotH = H - PAD_T - PAD_B;

  const minIter = pts.length ? pts[0].iter : 0;
  const maxIter = pts.length ? pts[pts.length - 1].iter : 0;
  const iterSpan = Math.max(1, maxIter - minIter);

  const x = (iter: number) =>
    PAD_L + ((iter - minIter) / iterSpan) * plotW;
  // Score is clamped to 0..1; y is inverted (1 at the top).
  const y = (score: number) => {
    const s = Math.max(0, Math.min(1, score));
    return PAD_T + (1 - s) * plotH;
  };

  const best = pts.reduce<Point | null>(
    (b, p) => (b === null || p.score > b.score ? p : b),
    null,
  );

  const linePath = pts.map((p) => `${x(p.iter)},${y(p.score)}`).join(' ');

  return (
    <div className="px-3 pb-3">
      <div className="mb-1 flex items-center justify-between">
        <div className="text-[11px] font-semibold uppercase tracking-wide text-adam-neutral-400">
          Score / iteration
        </div>
        {best && (
          <div className="text-[10px] text-adam-neutral-400">
            best{' '}
            <span className="font-mono font-semibold text-green-400">
              {best.score.toFixed(3)}
            </span>{' '}
            @ iter {best.iter}
          </div>
        )}
      </div>

      {pts.length === 0 ? (
        <div className="flex h-[120px] items-center justify-center rounded border border-adam-neutral-800 bg-adam-neutral-950/30 text-xs text-adam-neutral-500">
          No physics scores yet
        </div>
      ) : (
        <svg
          viewBox={`0 0 ${VB_W} ${H}`}
          width="100%"
          height={H}
          preserveAspectRatio="none"
          className="rounded border border-adam-neutral-800 bg-adam-neutral-950/30"
        >
          {/* y gridlines + labels at 0, 0.5, 1 */}
          {[0, 0.5, 1].map((t) => (
            <g key={t}>
              <line
                x1={PAD_L}
                y1={y(t)}
                x2={VB_W - PAD_R}
                y2={y(t)}
                stroke="#3a3a3a"
                strokeWidth={1}
                strokeDasharray={t === 0 || t === 1 ? undefined : '3 3'}
              />
              <text
                x={PAD_L - 6}
                y={y(t) + 3}
                textAnchor="end"
                fontSize={10}
                fill="#9a9a9a"
              >
                {t.toFixed(1)}
              </text>
            </g>
          ))}

          {/* x axis labels: first + last iteration */}
          <text
            x={x(minIter)}
            y={H - 8}
            textAnchor="middle"
            fontSize={10}
            fill="#9a9a9a"
          >
            {minIter}
          </text>
          {maxIter !== minIter && (
            <text
              x={x(maxIter)}
              y={H - 8}
              textAnchor="middle"
              fontSize={10}
              fill="#9a9a9a"
            >
              {maxIter}
            </text>
          )}

          {/* the score line */}
          {pts.length > 1 && (
            <polyline
              points={linePath}
              fill="none"
              stroke="#0078D4"
              strokeWidth={2}
              vectorEffect="non-scaling-stroke"
            />
          )}

          {/* a dot per iteration; the best one is highlighted green */}
          {pts.map((p) => {
            const isBest = best !== null && p.iter === best.iter;
            return (
              <circle
                key={p.iter}
                cx={x(p.iter)}
                cy={y(p.score)}
                r={isBest ? 5 : 3}
                fill={isBest ? '#22c55e' : '#0078D4'}
                stroke={isBest ? '#052e16' : 'none'}
                strokeWidth={isBest ? 1.5 : 0}
              />
            );
          })}
        </svg>
      )}
    </div>
  );
}
