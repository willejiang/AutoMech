// Served from the root, so BASE_URL is '/'. (The cadam app this was split out of ran
// under /cadam, which is why its apiUrl had to prefix every call.) No auth header:
// the maker2 routes spawn a local Python and read this repo's output/ directory —
// there is no user, no session, and nothing to authorise against.
export function apiUrl(path: string) {
  const basePath = import.meta.env.BASE_URL.replace(/\/$/, '');
  return `${basePath}/api/${path}`;
}

export async function apiJson<T = unknown>(
  path: string,
  init: RequestInit = {},
): Promise<T> {
  const response = await fetch(apiUrl(path), {
    ...init,
    headers: { 'Content-Type': 'application/json', ...init.headers },
  });
  if (!response.ok) {
    throw new Error(`${path} failed: ${response.status}`);
  }
  return (await response.json()) as T;
}
