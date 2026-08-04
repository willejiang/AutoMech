// Shared API-route helpers. The cadam version of this file also exported requireUser(),
// which builds a Supabase client from the Authorization header — the maker2 routes never
// called it (they spawn a local Python and read this repo's output/ directory, so there is
// no user to authorise), and dropping it is what keeps Supabase out of this app entirely.

export const corsHeaders = {
  'Access-Control-Allow-Origin': '*',
  'Access-Control-Allow-Headers': 'content-type',
  'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
};

export function json(data: unknown, status = 200) {
  return Response.json(data, { status, headers: corsHeaders });
}

export function preflight() {
  return new Response('ok', { headers: corsHeaders });
}

export function methodNotAllowed() {
  return json({ error: 'method_not_allowed' }, 405);
}

export function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null;
}
