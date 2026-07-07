import { createFileRoute } from '@tanstack/react-router';
import { WorkbenchView } from '@/views/WorkbenchView';

// Standalone (no _layout, no _auth) WORKBENCH for one maker2 run. The $runId IS
// the thread id. A fresh run streams from prompt/model/iters over the same SSE the
// maker2 editor uses. `deep=1` turns on deep-think (CadQuery + full debugger).
// This is ADDITIVE: /maker2/$runId (Maker2EditorView) stays as the fallback view.
type WorkbenchSearch = {
  prompt: string;
  model: string;
  iters: number;
  thread?: string;
  deep?: number;
};

export const Route = createFileRoute('/workbench/$runId')({
  validateSearch: (search: Record<string, unknown>): WorkbenchSearch => ({
    prompt: typeof search.prompt === 'string' ? search.prompt : '',
    model: typeof search.model === 'string' ? search.model : '',
    // 0 = infinite loop (default); a positive value caps the loop (from /iters N).
    iters:
      Number.isFinite(Number(search.iters)) && Number(search.iters) > 0
        ? Number(search.iters)
        : 0,
    thread: typeof search.thread === 'string' ? search.thread : undefined,
    deep: Number(search.deep) === 1 ? 1 : undefined,
  }),
  component: WorkbenchPage,
});

function WorkbenchPage() {
  const { runId } = Route.useParams();
  const { prompt, model, iters, thread, deep } = Route.useSearch();
  return (
    <div className="h-screen w-screen">
      <WorkbenchView
        prompt={prompt}
        model={model}
        iters={iters}
        threadId={thread || runId}
        deep={deep === 1}
      />
    </div>
  );
}
