import { createFileRoute } from '@tanstack/react-router';
import { WorkbenchView } from '@/views/WorkbenchView';

// The workbench for ONE maker2 run. $runId IS the thread id, so reopening a past run
// from the sidebar replays it; arriving with a `prompt` starts a fresh one over SSE.
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
    // 0 = run until it converges (default); a positive value caps the loop.
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
