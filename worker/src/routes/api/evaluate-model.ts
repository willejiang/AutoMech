import { createFileRoute } from '@tanstack/react-router';
import { evaluateModelStream } from '@/server/evaluateModel';
import {
  corsHeaders,
  isRecord,
  isUnauthorizedError,
  json,
  methodNotAllowed,
  preflight,
  requireUser,
} from '@/server/api';

// Cap how many images we forward — the 6-view export is the expected caller.
const MAX_IMAGES = 6;

export const Route = createFileRoute('/api/evaluate-model')({
  server: {
    handlers: {
      GET: methodNotAllowed,
      OPTIONS: preflight,
      POST: async ({ request }) => {
        try {
          await requireUser(request);
        } catch (err) {
          if (isUnauthorizedError(err)) {
            return json({ error: 'Unauthorized' }, 401);
          }
          throw err;
        }

        try {
          const body: unknown = await request.json();
          if (!isRecord(body)) {
            return json({ error: 'invalid_request' }, 400);
          }
          const prompt = typeof body.prompt === 'string' ? body.prompt : '';
          const images = Array.isArray(body.images)
            ? body.images
                .filter((item): item is string => typeof item === 'string')
                .slice(0, MAX_IMAGES)
            : [];

          const stream = evaluateModelStream({ prompt, images });
          return new Response(stream, {
            status: 200,
            headers: {
              ...corsHeaders,
              'Content-Type': 'text/plain; charset=utf-8',
              'Cache-Control': 'no-cache, no-transform',
            },
          });
        } catch {
          // Evaluation is advisory — never hard-error. An empty body makes the
          // client fall back to "no verdict".
          return new Response('', {
            status: 200,
            headers: {
              ...corsHeaders,
              'Content-Type': 'text/plain; charset=utf-8',
            },
          });
        }
      },
    },
  },
});
