import { generate3DModelFilename } from '@/utils/file-utils';
import { Message } from '@shared/types';
import { ZipWriter, BlobWriter, BlobReader } from '@zip.js/zip.js';

// On-demand DXF generator. The OpenSCAD worker produces DXF output by recompiling
// the source through a top-down projection, so consumers receive a callback rather
// than a ready blob.
export type DxfExporter = () => Promise<Blob>;

interface DownloadOptions {
  content: Blob | string;
  filename: string;
  mimeType?: string;
}

interface GenerateDownloadFilenameOptions {
  currentMessage?: Message | null;
  fallback?: string;
  extension: string;
}

/**
 * Downloads a file by creating a temporary download link
 */
export function downloadFile({
  content,
  filename,
  mimeType = 'application/octet-stream',
}: DownloadOptions): void {
  let blob: Blob;

  if (typeof content === 'string') {
    blob = new Blob([content], { type: mimeType });
  } else {
    blob = content;
  }

  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

/**
 * Generates a filename for downloads using the 3D model filename utility
 */
export function generateDownloadFilename({
  currentMessage,
  fallback = 'parametric-model',
  extension,
}: GenerateDownloadFilenameOptions): string {
  const baseName = generate3DModelFilename({
    conversationTitle: undefined,
    assistantMessage: currentMessage || undefined,
    modelName: undefined,
    fallback,
  });

  return `${baseName}.${extension}`;
}

/**
 * Downloads STL file from blob
 */
export function downloadSTLFile(
  output: Blob,
  currentMessage?: Message | null,
): void {
  const filename = generateDownloadFilename({
    currentMessage,
    extension: 'stl',
  });

  downloadFile({
    content: output,
    filename,
    mimeType: 'application/octet-stream',
  });
}

/**
 * Downloads OpenSCAD code as .scad file
 */
export function downloadOpenSCADFile(
  code: string,
  currentMessage?: Message | null,
): void {
  const filename = generateDownloadFilename({
    currentMessage,
    extension: 'scad',
  });

  downloadFile({
    content: code,
    filename,
    mimeType: 'text/plain',
  });
}

/**
 * Downloads the model evaluation verdict as a .json file.
 */
export function downloadEvaluationJson(
  evaluation: { passed: boolean; reason: string; suggestions: string },
  currentMessage?: Message | null,
): void {
  const filename = generateDownloadFilename({
    currentMessage,
    extension: 'json',
  });

  downloadFile({
    content: JSON.stringify(evaluation, null, 2),
    filename,
    mimeType: 'application/json;charset=utf-8',
  });
}

/**
 * Downloads DXF file from blob
 */
export function downloadDXFFile(
  output: Blob,
  currentMessage?: Message | null,
): void {
  const filename = generateDownloadFilename({
    currentMessage,
    extension: 'dxf',
  });

  downloadFile({
    content: output,
    filename,
    mimeType: 'application/dxf',
  });
}

/**
 * Zips the six orthographic-view PNGs into a single archive and downloads it.
 * One archive (not six separate downloads) avoids the browser's
 * multiple-download prompt. Entries are numbered `<base>-1.png` … `-6.png`
 * in view order (front, back, left, right, top, bottom).
 */
export async function downloadSixViewZip(
  views: { name: string; dataUrl: string }[],
  currentMessage?: Message | null,
): Promise<void> {
  const baseName = generateDownloadFilename({
    currentMessage,
    extension: 'views',
  }).replace(/\.views$/, '');

  const blobWriter = new BlobWriter();
  const zipWriter = new ZipWriter(blobWriter, {
    level: 6,
    bufferedWrite: true,
  });
  for (const [index, view] of views.entries()) {
    const pngBlob = await fetch(view.dataUrl).then((response) =>
      response.blob(),
    );
    await zipWriter.add(
      `${baseName}-${index + 1}.png`,
      new BlobReader(pngBlob),
    );
  }
  await zipWriter.close();

  downloadFile({
    content: await blobWriter.getData(),
    filename: `${baseName}-6views.zip`,
    mimeType: 'application/zip',
  });
}
