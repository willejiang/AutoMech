import { useRef, useState } from 'react';
import { useNavigate } from '@tanstack/react-router';
import { ArrowUp, BookOpen, Brain, Globe, ImagePlus, Layers, Loader2, X } from 'lucide-react';
import { apiUrl } from '@/services/api';
import { cn } from '@/lib/utils';

// The launcher. There is no "articulated" toggle here — in cadam that switch chose
// between its OpenSCAD generator and maker2; this app only runs maker2, so the choice
// is gone and the pipeline is simply what happens.
const MODELS = [
  { id: 'gpt-5.6-sol', label: 'GPT-5.6 Sol' },
  { id: 'gpt-5.5', label: 'GPT-5.5' },
  { id: 'claude-opus-4.8', label: 'Claude Opus 4.8' },
  { id: 'gemini-3.1-pro', label: 'Gemini 3.1 Pro' },
];

export function LauncherView() {
  const navigate = useNavigate();
  const [prompt, setPrompt] = useState('');
  const [model, setModel] = useState(MODELS[0].id);
  const [web, setWeb] = useState(true);
  const [deep, setDeep] = useState(true);
  const [hierarchy, setHierarchy] = useState(false);

  // A reference image is uploaded BEFORE the run starts: the run stream is an EventSource,
  // which can only GET, so the file cannot ride along with the prompt. We keep the id the
  // upload returns and pass that instead.
  const fileRef = useRef<HTMLInputElement | null>(null);
  const [image, setImage] = useState<{ id: string; name: string } | null>(null);
  const [uploading, setUploading] = useState(false);
  const [imageError, setImageError] = useState('');

  const pickImage = async (file: File) => {
    setUploading(true);
    setImageError('');
    try {
      const body = new FormData();
      body.append('image', file);
      const r = await fetch(apiUrl('upload-image'), { method: 'POST', body });
      const data = (await r.json()) as { id?: string; name?: string; error?: string };
      if (!r.ok || !data.id) throw new Error(data.error || `upload failed (${r.status})`);
      setImage({ id: data.id, name: data.name || file.name });
    } catch (e) {
      setImageError(e instanceof Error ? e.message : String(e));
    } finally {
      setUploading(false);
    }
  };

  const start = () => {
    const text = prompt.trim();
    // With a picture attached the text is only a hint, so it need not carry the machine.
    if (!text && !image) return;
    navigate({
      to: '/workbench/$runId',
      params: { runId: crypto.randomUUID() },
      search: {
        prompt: text || 'the machine in the attached image',
        model,
        iters: 0,
        deep: deep ? 1 : undefined,
        web: web ? 1 : 0,
        mode: hierarchy ? 'hierarchy' : 'single-agent',
        image: image?.id,
      },
    });
  };

  return (
    <div className="flex h-full flex-col items-center justify-center px-6">
      <div className="w-full max-w-2xl">
        <img
          src="/automech-logo.png"
          alt="AutoMech"
          className="mx-auto mb-3 h-9 w-auto"
        />
        <p className="mb-8 text-center text-sm text-muted-foreground">
          Describe a machine. It gets built, simulated, and rebuilt until it works.
        </p>

        <div className="rounded-lg border border-border bg-card p-3">
          <textarea
            value={prompt}
            onChange={(e) => setPrompt(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === 'Enter' && (e.metaKey || e.ctrlKey)) start();
            }}
            rows={3}
            autoFocus
            placeholder="a hand-cranked gear reducer with a 20:1 ratio"
            className="w-full resize-none bg-transparent px-1 text-sm outline-none placeholder:text-muted-foreground"
          />

          <div className="mt-3 flex flex-wrap items-center gap-2">
            <input
              ref={fileRef}
              type="file"
              accept="image/png,image/jpeg,image/gif,image/webp"
              className="hidden"
              onChange={(e) => {
                const f = e.target.files?.[0];
                if (f) void pickImage(f);
                e.target.value = '';   // same file twice in a row must still fire
              }}
            />
            <Toggle
              on={!!image}
              onClick={() => (image ? setImage(null) : fileRef.current?.click())}
              icon={
                uploading ? <Loader2 size={13} className="animate-spin" />
                  : image ? <X size={13} /> : <ImagePlus size={13} />
              }
              title={image ? `${image.name} — click to remove` : 'Build what is in a photo'}
            >
              {image ? truncate(image.name) : 'Image'}
            </Toggle>

            <Toggle on={web} onClick={() => setWeb(!web)} icon={<Globe size={13} />}>
              Web
            </Toggle>
            <Toggle on icon={<BookOpen size={13} />} title="Local knowledge base is always on">
              KB
            </Toggle>
            <Toggle on={deep} onClick={() => setDeep(!deep)} icon={<Brain size={13} />}>
              Deep think
            </Toggle>
            <Toggle
              on={hierarchy}
              onClick={() => setHierarchy(!hierarchy)}
              icon={<Layers size={13} />}
              title="Boss + per-subassembly managers instead of one agent"
            >
              Hierarchy
            </Toggle>

            <select
              value={model}
              onChange={(e) => setModel(e.target.value)}
              className="ml-auto rounded border border-border bg-transparent px-2 py-1 text-xs text-muted-foreground outline-none"
            >
              {MODELS.map((m) => (
                <option key={m.id} value={m.id} className="bg-card">
                  {m.label}
                </option>
              ))}
            </select>

            <button
              onClick={start}
              disabled={!prompt.trim() && !image}
              className="rounded bg-primary p-1.5 text-primary-foreground disabled:opacity-30"
              title="Build it (Cmd/Ctrl + Enter)"
            >
              <ArrowUp size={15} />
            </button>
          </div>
        </div>

        {imageError && (
          <p className="mt-2 text-xs text-destructive">{imageError}</p>
        )}
      </div>
    </div>
  );
}

/** Keep a long filename from pushing the toggle row onto a second line. */
function truncate(name: string, max = 18) {
  return name.length <= max ? name : `${name.slice(0, max - 1)}…`;
}

function Toggle({
  on,
  onClick,
  icon,
  title,
  children,
}: {
  on: boolean;
  onClick?: () => void;
  icon: React.ReactNode;
  title?: string;
  children: React.ReactNode;
}) {
  return (
    <button
      onClick={onClick}
      title={title}
      disabled={!onClick}
      className={cn(
        'flex items-center gap-1.5 rounded-full border px-2.5 py-1 text-xs transition-colors',
        on
          ? 'border-primary/40 bg-primary/10 text-foreground'
          : 'border-border text-muted-foreground hover:text-foreground',
        !onClick && 'cursor-default opacity-70',
      )}
    >
      {icon}
      {children}
    </button>
  );
}
