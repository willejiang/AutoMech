import { OrbitControls, Stage } from '@react-three/drei';
import { Canvas } from '@react-three/fiber';
import { Suspense, useEffect, useState } from 'react';
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
 */
interface Maker2ModelCanvasProps {
  /** The assembled GLB. While undefined, the run is still in progress. */
  glbBlob?: Blob;
  // 'failed' -> the run produced no model at all (hard crash): show a message
  // instead of spinning forever. Omitted/'loading' keeps the normal spinner.
  status?: 'loading' | 'failed';
  failedReason?: string;
}

function Model({ scene }: { scene: THREE.Object3D }) {
  // Use <Stage> ONLY to auto-center/frame the model (intensity 0 -> it adds no
  // lights of its own). The light rig lives at the Canvas level so we control the
  // directions — critically an OVERHEAD key so top-down / bird's-eye views are lit
  // as well as the front (the watch is mostly flat top faces from above).
  return (
    <Stage intensity={0} environment={null} adjustCamera={1.1} shadows={false}>
      <primitive object={scene} />
    </Stage>
  );
}

export function Maker2ModelCanvas({ glbBlob, status, failedReason }: Maker2ModelCanvasProps) {
  const [scene, setScene] = useState<THREE.Object3D | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    setScene(null);
    setError(null);
    if (!glbBlob) return;

    let revoked = false;
    const url = URL.createObjectURL(glbBlob);
    const loader = new GLTFLoader();
    loader.load(
      url,
      (gltf) => {
        if (!revoked) setScene(gltf.scene);
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
  );
}
