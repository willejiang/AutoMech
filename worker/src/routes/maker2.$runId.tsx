import { createFileRoute } from '@tanstack/react-router';
import { Maker2EditorView } from '@/views/Maker2EditorView';

// Standalone (no _layout, no _auth) editor for one maker2 CONVERSATION. The
// $runId IS the thread id. A FRESH run streams turn 1 from prompt/model/iters.
// Reopening from the sidebar passes ?thread=<id> to load the saved thread and
// continue chatting; legacy runs (pre-thread) pass ?dir=<run_dir> for a
// read-only single view.
type Maker2Search = {
  prompt: string; model: string; iters: number;
  thread?: string; dir?: string;
};

export const Route = createFileRoute('/maker2/$runId')({
  validateSearch: (search: Record<string, unknown>): Maker2Search => ({
    prompt: typeof search.prompt === 'string' ? search.prompt : '',
    model: typeof search.model === 'string' ? search.model : '',
    iters: Number(search.iters) > 0 ? Number(search.iters) : 2,
    thread: typeof search.thread === 'string' ? search.thread : undefined,
    dir: typeof search.dir === 'string' ? search.dir : undefined,
  }),
  component: Maker2Page,
});

function Maker2Page() {
  const { runId } = Route.useParams();
  const { prompt, model, iters, thread, dir } = Route.useSearch();
  return (
    <div className="h-screen w-screen">
      <Maker2EditorView
        prompt={prompt}
        model={model}
        iters={iters}
        threadId={thread || runId}
        reopenThread={!!thread}
        viewDir={dir}
      />
    </div>
  );
}
