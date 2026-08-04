import { createFileRoute } from '@tanstack/react-router';
import { existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { randomUUID } from 'node:crypto';
import { resolve } from 'node:path';
import { json, methodNotAllowed, preflight } from '@/server/api';

// IMAGE UPLOAD for the "build what you see" path. The run route is an EventSource (SSE),
// which can only GET and cannot carry a body, so the image cannot ride along with the
// prompt. It is POSTed here first, written under output/uploads/, and the run is started
// with the returned id — which run-maker2-stream resolves back to a path for `--image`.
//
// Returning an ID rather than the path keeps the client from choosing where Python reads
// from: the id is validated against a fixed directory, so a crafted value cannot point the
// spawn at an arbitrary file.
const REPO_ROOT = process.cwd();
const UPLOAD_DIR = resolve(REPO_ROOT, 'output', 'uploads');

// What maker2/imageutil.py accepts. Rejecting anything else HERE means the user is told
// at upload time, instead of the run dying several seconds in on ImageLoadError.
const EXT_BY_TYPE: Record<string, string> = {
  'image/png': '.png',
  'image/jpeg': '.jpg',
  'image/gif': '.gif',
  'image/webp': '.webp',
};

const MAX_BYTES = 12 * 1024 * 1024;

/** Resolve an upload id to its path, or null. Shared with the run route so the id->path
 *  rule lives in one place. Rejects anything that is not one of our own generated names. */
export function uploadPath(id: string): string | null {
  if (!/^[0-9a-f-]{36}\.(png|jpg|gif|webp)$/i.test(id)) return null;
  const p = resolve(UPLOAD_DIR, id);
  if (!p.startsWith(UPLOAD_DIR)) return null;      // defence in depth vs traversal
  return existsSync(p) ? p : null;
}

export const Route = createFileRoute('/api/upload-image')({
  server: {
    handlers: {
      OPTIONS: () => preflight(),
      GET: () => methodNotAllowed(),

      POST: async ({ request }) => {
        const form = await request.formData().catch(() => null);
        const file = form?.get('image');
        if (!(file instanceof File)) return json({ error: 'need an image file' }, 400);

        const ext = EXT_BY_TYPE[file.type];
        if (!ext) {
          return json({
            error: `unsupported type ${file.type || 'unknown'}; use PNG, JPEG, GIF or WebP`,
          }, 415);
        }
        if (file.size <= 0) return json({ error: 'image is empty' }, 400);
        if (file.size > MAX_BYTES) {
          return json({ error: `image is ${(file.size / 1e6).toFixed(1)}MB; max 12MB` }, 413);
        }

        const id = `${randomUUID()}${ext}`;
        try {
          mkdirSync(UPLOAD_DIR, { recursive: true });
          writeFileSync(resolve(UPLOAD_DIR, id),
                        Buffer.from(await file.arrayBuffer()));
        } catch (e) {
          return json({ error: `could not save upload: ${String(e)}` }, 500);
        }
        return json({ id, name: file.name, bytes: file.size });
      },
    },
  },
});
