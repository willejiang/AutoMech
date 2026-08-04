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
  /** Part names to render GHOSTED. Driven by the left-column part list, so the choice
   *  persists while you orbit instead of vanishing the moment the cursor moves off. */
  ghosted?: Set<string>;
  /** Reports this GLB's part names (scene order) so the panel can list them. */
  onParts?: (names: string[]) => void;
  // 'failed' -> the run produced no model at all (hard crash): show a message
  // instead of spinning forever. Omitted/'loading' keeps the normal spinner.
  status?: 'loading' | 'failed';
  failedReason?: string;
}

const FADED_OPACITY = 0.12;


interface PartEntry {
  mesh: THREE.Mesh;
  name: string;
}

/** Per-mesh: clone the material (so a shared material isn't faded globally) and stash the
 *  original opacity/transparent so we can restore it on pointer-out. Also collect a flat
 *  list of the named part meshes so the caller can drive per-part ghosting. */
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
    parts.push({ mesh, name });
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

function Model({ scene, parts, ghosted }: {
  scene: THREE.Object3D;
  parts: PartEntry[];
  ghosted?: Set<string>;
}) {
  const hoveredRef = useRef<THREE.Mesh | null>(null);

  // Apply the panel's selection. This is the AUTHORITATIVE fade state; hover is only a
  // transient nudge on top of it, so re-running this also repairs anything hover left
  // faded. Ghosted parts stay VISIBLE — an outline still tells you where the part sits;
  // it simply stops hiding what is behind it.
  useEffect(() => {
    for (const p of parts) setMeshFaded(p.mesh, !!ghosted?.has(p.name));
  }, [parts, ghosted]);

  // Un-hovering restores the mesh to what the PANEL says, not unconditionally to solid
  // — otherwise brushing past a ghosted part would silently un-ghost it.
  const restore = useCallback(
    (mesh: THREE.Mesh) => {
      const entry = parts.find((p) => p.mesh === mesh);
      setMeshFaded(mesh, !!(entry && ghosted?.has(entry.name)));
    },
    [parts, ghosted],
  );

  const clearHover = useCallback(() => {
    if (hoveredRef.current) {
      restore(hoveredRef.current);
      hoveredRef.current = null;
    }
  }, [restore]);

  // Fade the single closest hit mesh (event.object). stopPropagation so only the
  // front-most part fades, not everything the ray passes through.
  const handleMove = useCallback(
    (e: ThreeEvent<PointerEvent>) => {
      e.stopPropagation();
      const mesh = e.object as THREE.Mesh;
      if (!mesh?.isMesh || mesh === hoveredRef.current) return;
      if (hoveredRef.current) restore(hoveredRef.current);
      setMeshFaded(mesh, true);
      hoveredRef.current = mesh;
    },
    [restore],
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

export function Maker2ModelCanvas({
  glbBlob,
  status,
  failedReason,
  ghosted,
  onParts,
}: Maker2ModelCanvasProps) {
  const [scene, setScene] = useState<THREE.Object3D | null>(null);
  const [parts, setParts] = useState<PartEntry[]>([]);
  const [error, setError] = useState<string | null>(null);
  // Held in a ref so a caller passing an inline arrow doesn't re-run the GLB load.
  const onPartsRef = useRef(onParts);
  onPartsRef.current = onParts;

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
          const found = prepareScene(gltf.scene);
          setParts(found);
          setScene(gltf.scene);
          onPartsRef.current?.([...new Set(found.map((p) => p.name))]);
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
          <Model scene={scene} parts={parts} ghosted={ghosted} />
        </Suspense>
        <OrbitControls makeDefault enablePan enableZoom enableRotate />
      </Canvas>
    </div>
  );
}
