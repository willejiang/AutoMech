import { createFileRoute } from '@tanstack/react-router';
import { Maker2EditorView } from '@/views/Maker2EditorView';

// Standalone (no _layout, no _auth) editor for one ephemeral maker2 run. The
// $runId only keys the page; the actual run is parameterized by the search params
// (prompt/model/iters), which the view streams via /api/run-maker2-stream.
type Maker2Search = { prompt: string; model: string; iters: number };

export const Route = createFileRoute('/maker2/$runId')({
  validateSearch: (search: Record<string, unknown>): Maker2Search => ({
    prompt: typeof search.prompt === 'string' ? search.prompt : '',
    model: typeof search.model === 'string' ? search.model : '',
    iters: Number(search.iters) > 0 ? Number(search.iters) : 2,
  }),
  component: Maker2Page,
});

function Maker2Page() {
  const { prompt, model, iters } = Route.useSearch();
  return (
    <div className="h-screen w-screen">
      <Maker2EditorView prompt={prompt} model={model} iters={iters} />
    </div>
  );
}
