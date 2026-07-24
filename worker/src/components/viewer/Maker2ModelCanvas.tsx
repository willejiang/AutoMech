import { OrbitControls, Stage } from '@react-three/drei';
import { Canvas, type ThreeEvent } from '@react-three/fiber';
import { Suspense, useCallback, useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { GLTFLoader } from 'three-stdlib';

/**
 * Maker2ModelCanvas — renders the assembled maker2 URDF (exported to a single GLB
 * by /api/run-maker2-glb) as a SOLID, colored model the user can orbit/zoom, the
 * way the cadam canvas shows a parametric result.
 *
 * Deliberately NOT GlbPreview: that one decimates the mesh to a ~2k-point Adam-logo
 * particle cloud and drops the per-link colors. Here we render the real gltf.scene
 * so the colors authored in the URDF <material> blocks show through.
 *
 * Hover-to-fade: pointing at a part fades ONLY that part so you can see the internal
 * structure behind it (gears/shafts inside a housing). The GLB is a multi-part scene
 * graph (one node per URDF link), so each hit mesh fades independently. Materials in a
 * GLB can be SHARED instances, so on load we clone each mesh's material — otherwise
 * fading one part would fade every part that happens to share the material.
 */
interface Maker2ModelCanvasProps {
  /** The assembled GLB. While undefined, the run is still in progress. */
  glbBlob?: Blob;
  // 'failed' -> the run produced no model at all (hard crash): show a message
  // instead of spinning forever. Omitted/'loading' keeps the normal spinner.
  status?: 'loading' | 'failed';
  failedReason?: string;
}

const FADED_OPACITY = 0.12;

/** Coarse part categories, matched against the URDF link name (case-insensitive), in
 *  priority order — the first pattern that hits wins. "Structure" (housings, walls,
 *  plates, covers) is what you usually want to hide to see the gear train inside. */
const PART_CATEGORIES: { key: string; label: string; test: RegExp }[] = [
  { key: 'gears', label: 'Gears', test: /gear|pinion|sun|planet|ring/ },
  { key: 'shafts', label: 'Shafts & pins', test: /shaft|pin|arbor|axle|spindle/ },
  { key: 'bearings', label: 'Bearings & bushings', test: /bearing|bush|journal|collar|spacer/ },
  {
    key: 'structure',
    label: 'Housing & structure',
    test: /housing|shell|wall|plate|cover|case|frame|carrier|base|body|bracket|mount|seat|flange/,
  },
];
const OTHER_CATEGORY = { key: 'other', label: 'Other', test: /.*/ };

function categoryFor(name: string): string {
  const n = name.toLowerCase();
  for (const c of PART_CATEGORIES) if (c.test.test(n)) return c.key;
  return OTHER_CATEGORY.key;
}

interface PartEntry {
  mesh: THREE.Mesh;
  name: string;
  category: string;
}

/** Per-mesh: clone the material (so a shared material isn't faded globally) and stash the
 *  original opacity/transparent so we can restore it on pointer-out. Also collect a flat
 *  list of the named part meshes for the Parts Inspector. */
function prepareScene(root: THREE.Object3D): PartEntry[] {
  const parts: PartEntry[] = [];
  root.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (!mesh.isMesh || !mesh.material) return;
    const mat = Array.isArray(mesh.material)
      ? mesh.material.map((m) => m.clone())
      : mesh.material.clone();
    mesh.material = mat;
    const mats = Array.isArray(mat) ? mat : [mat];
    for (const m of mats) {
      m.userData.__origOpacity = m.opacity;
      m.userData.__origTransparent = m.transparent;
    }
    // The GLB names each part node after its URDF link; prefer the nearest named ancestor.
    let named: THREE.Object3D = mesh;
    while (named && !named.name && named.parent) named = named.parent;
    const name = named?.name || mesh.name || 'part';
    parts.push({ mesh, name, category: categoryFor(name) });
  });
  return parts;
}

function setMeshFaded(mesh: THREE.Mesh, faded: boolean) {
  const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
  for (const m of mats) {
    if (faded) {
      m.transparent = true;
      m.opacity = FADED_OPACITY;
      m.depthWrite = false;
    } else {
      m.opacity = (m.userData.__origOpacity as number) ?? 1;
      m.transparent = (m.userData.__origTransparent as boolean) ?? false;
      m.depthWrite = true;
    }
    m.needsUpdate = true;
  }
}

function Model({ scene }: { scene: THREE.Object3D }) {
  const hoveredRef = useRef<THREE.Mesh | null>(null);

  const clearHover = useCallback(() => {
    if (hoveredRef.current) {
      setMeshFaded(hoveredRef.current, false);
      hoveredRef.current = null;
    }
  }, []);

  // Fade the single closest hit mesh (event.object). stopPropagation so only the
  // front-most part fades, not everything the ray passes through.
  const handleMove = useCallback(
    (e: ThreeEvent<PointerEvent>) => {
      e.stopPropagation();
      const mesh = e.object as THREE.Mesh;
      if (!mesh?.isMesh || mesh === hoveredRef.current) return;
      if (hoveredRef.current) setMeshFaded(hoveredRef.current, false);
      setMeshFaded(mesh, true);
      hoveredRef.current = mesh;
    },
    [],
  );

  // Restore whatever we faded when the pointer leaves the model entirely.
  useEffect(() => () => clearHover(), [clearHover]);

  // Use <Stage> ONLY to auto-center/frame the model (intensity 0 -> it adds no
  // lights of its own). The light rig lives at the Canvas level so we control the
  // directions — critically an OVERHEAD key so top-down / bird's-eye views are lit
  // as well as the front (the watch is mostly flat top faces from above).
  return (
    <Stage intensity={0} environment={null} adjustCamera={1.1} shadows={false}>
      <primitive
        object={scene}
        onPointerMove={handleMove}
        onPointerOut={clearHover}
      />
    </Stage>
  );
}

/** Parts Inspector — a floating panel that toggles the VISIBILITY of whole part
 *  categories (and individual parts) so nested housings/plates can be hidden to reveal
 *  the gear train inside. This is the real "see inside" control; hover-fade only dims a
 *  single front-most part and can't cut through several nested shells at once. */
function PartsInspector({ parts }: { parts: PartEntry[] }) {
  const [open, setOpen] = useState(true);
  // Force a re-render after mutating mesh.visible (three state isn't React state).
  const [, bump] = useState(0);
  const rerender = () => bump((n) => n + 1);
  const [expanded, setExpanded] = useState<Set<string>>(new Set());

  // Group parts by category, in the declared order, dropping empty categories.
  const order = [...PART_CATEGORIES, OTHER_CATEGORY];
  const groups = order
    .map((c) => ({ ...c, items: parts.filter((p) => p.category === c.key) }))
    .filter((g) => g.items.length > 0);

  const setGroupVisible = (items: PartEntry[], visible: boolean) => {
    for (const p of items) p.mesh.visible = visible;
    rerender();
  };
  const groupState = (items: PartEntry[]) => {
    const vis = items.filter((p) => p.mesh.visible).length;
    return vis === 0 ? 'none' : vis === items.length ? 'all' : 'some';
  };

  if (!open) {
    return (
      <button
        type="button"
        onClick={() => setOpen(true)}
        className="absolute right-3 top-3 rounded-md bg-adam-neutral-900/85 px-2.5 py-1.5 text-xs font-medium text-adam-neutral-200 shadow-lg backdrop-blur hover:bg-adam-neutral-800"
      >
        Parts
      </button>
    );
  }

  return (
    <div className="absolute right-3 top-3 max-h-[80%] w-56 overflow-auto rounded-lg bg-adam-neutral-900/90 p-3 text-xs text-adam-neutral-200 shadow-xl backdrop-blur">
      <div className="mb-2 flex items-center justify-between">
        <span className="font-semibold">Parts</span>
        <div className="flex gap-2">
          <button
            type="button"
            className="text-adam-neutral-400 hover:text-adam-neutral-100"
            onClick={() => setGroupVisible(parts, true)}
          >
            Show all
          </button>
          <button
            type="button"
            className="text-adam-neutral-500 hover:text-adam-neutral-100"
            onClick={() => setOpen(false)}
          >
            ✕
          </button>
        </div>
      </div>
      {groups.map((g) => {
        const st = groupState(g.items);
        const isExpanded = expanded.has(g.key);
        return (
          <div key={g.key} className="mb-1.5">
            <div className="flex items-center gap-2">
              <input
                type="checkbox"
                className="h-3.5 w-3.5 accent-adam-blue"
                checked={st !== 'none'}
                ref={(el) => {
                  if (el) el.indeterminate = st === 'some';
                }}
                onChange={(e) => setGroupVisible(g.items, e.target.checked)}
              />
              <button
                type="button"
                className="flex-1 text-left font-medium hover:text-white"
                onClick={() =>
                  setExpanded((prev) => {
                    const next = new Set(prev);
                    next.has(g.key) ? next.delete(g.key) : next.add(g.key);
                    return next;
                  })
                }
              >
                {g.label}{' '}
                <span className="text-adam-neutral-500">({g.items.length})</span>
              </button>
            </div>
            {isExpanded && (
              <div className="ml-5 mt-1 flex flex-col gap-0.5">
                {g.items.map((p, i) => (
                  <label key={`${p.name}-${i}`} className="flex items-center gap-2 text-adam-neutral-400">
                    <input
                      type="checkbox"
                      className="h-3 w-3 accent-adam-blue"
                      checked={p.mesh.visible}
                      onChange={(e) => {
                        p.mesh.visible = e.target.checked;
                        rerender();
                      }}
                    />
                    <span className="truncate" title={p.name}>
                      {p.name}
                    </span>
                  </label>
                ))}
              </div>
            )}
          </div>
        );
      })}
    </div>
  );
}

export function Maker2ModelCanvas({ glbBlob, status, failedReason }: Maker2ModelCanvasProps) {
  const [scene, setScene] = useState<THREE.Object3D | null>(null);
  const [parts, setParts] = useState<PartEntry[]>([]);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    setScene(null);
    setParts([]);
    setError(null);
    if (!glbBlob) return;

    let revoked = false;
    const url = URL.createObjectURL(glbBlob);
    const loader = new GLTFLoader();
    loader.load(
      url,
      (gltf) => {
        if (!revoked) {
          setParts(prepareScene(gltf.scene));
          setScene(gltf.scene);
        }
      },
      undefined,
      (e) => {
        if (!revoked) setError(String(e));
      },
    );
    return () => {
      revoked = true;
      URL.revokeObjectURL(url);
    };
  }, [glbBlob]);

  if (error) {
    return (
      <div className="flex h-full w-full items-center justify-center p-6 text-center text-sm text-red-400">
        Failed to load model: {error}
      </div>
    );
  }

  // A hard-failed run has no model to ever show; don't spin forever.
  if (!scene && status === 'failed') {
    return (
      <div className="flex h-full w-full flex-col items-center justify-center gap-2 p-6 text-center text-adam-neutral-400">
        <div className="text-sm font-medium text-red-400">
          Build failed — no model was produced
        </div>
        {failedReason && (
          <div className="max-w-md text-xs text-adam-neutral-500">{failedReason}</div>
        )}
      </div>
    );
  }

  if (!scene) {
    return (
      <div className="flex h-full w-full flex-col items-center justify-center gap-3 text-adam-neutral-400">
        <div className="h-8 w-8 animate-spin rounded-full border-2 border-adam-blue border-t-transparent" />
        <div className="text-sm">{glbBlob ? 'Loading model…' : 'Assembling…'}</div>
      </div>
    );
  }

  return (
    <div className="relative h-full w-full">
      <Canvas
        className="h-full w-full"
        camera={{ position: [0, 0, 5], fov: 45 }}
        dpr={[1, 2]}
        gl={{ toneMapping: THREE.ACESFilmicToneMapping, toneMappingExposure: 1.0 }}
      >
        {/* Light-gray background. */}
        <color attach="background" args={['#c8ccd2']} />
        {/* Light rig covering ALL orbit angles. The DOMINANT key is OVERHEAD so a
            top-down / bird's-eye view (mostly flat top faces) is well lit, not just
            the front; front/back/side fills keep every other angle clear too. */}
        <ambientLight intensity={0.55} />
        <hemisphereLight args={[0xffffff, 0x30333c, 0.6]} />
        <directionalLight position={[0, 10, 0.5]} intensity={1.5} />{/* overhead key */}
        <directionalLight position={[0, -6, 2]} intensity={0.4} />{/* under-fill */}
        <directionalLight position={[6, 3, 6]} intensity={0.7} />{/* front-right */}
        <directionalLight position={[-6, 3, -6]} intensity={0.5} />{/* back-left */}
        <Suspense fallback={null}>
          <Model scene={scene} />
        </Suspense>
        <OrbitControls makeDefault enablePan enableZoom enableRotate />
      </Canvas>
      {parts.length > 0 && <PartsInspector parts={parts} />}
    </div>
  );
}
