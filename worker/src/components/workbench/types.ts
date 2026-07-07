// Shared types for the Workbench view — a fresh maker2 frontend that streams a
// hierarchy run over the SAME SSE + GLB backend the Maker2EditorView uses.
// These mirror the ARTIFACT kinds the /api/run-maker2-stream route emits.

// Which layer of the hierarchy a deterministic gate ran against.
export type GateLayer = 'boss' | 'manager' | 'worker' | 'assembled';

// One deterministic benchmark result. ok=false is a BLOCKING failure; ok=true is
// a non-blocking WARNING (e.g. ERR_DIM, ERR_SUP_GROUND).
export type GateResult = {
  layer: GateLayer;
  code: string;        // e.g. ERR_OVL, ERR_CONNECT, ERR_MANIFOLD, ERR_SCHEMA_*
  detail: string;
  culprit: string;
  ok: boolean;
  sub_id?: string;
  iter?: number;
};

// One geometric pre-check violation (kind:'precheck').
export type PrecheckViolation = {
  kind: string;
  severity: string;
  sub_id?: string;
  detail?: string;
};

// The per-iteration physics score breakdown (kind:'physics').
export type ScoreBreakdown = {
  score: number;
  terms: Record<string, number>;
};

// The discriminated union of every artifact object the SSE 'artifact' event
// carries (the `kind` field is the discriminant).
export type ArtifactEvent =
  | GateResult & { kind: 'gate' }
  | {
      kind: 'subassembly';
      sub_id: string;
      run_dir: string;
      render_dir?: string;
      ok: boolean;
    }
  | { kind: 'assembled_model'; iter: number; run_dir: string; render_dir?: string }
  | { kind: 'precheck'; iter?: number; ok: boolean; violations: PrecheckViolation[] }
  | {
      kind: 'physics';
      iter: number;
      run_dir: string;
      render_dir?: string;
      passed: boolean;
      score: number;
      score_breakdown?: ScoreBreakdown;
      physics?: Record<string, unknown>;
    }
  | {
      kind: 'mesh_progress';
      run_dir?: string;
      built?: number;
      total?: number;
      iter?: number;
    };

// A subassembly build, accumulated from 'subassembly' artifacts.
export type SubInfo = { sub_id: string; ok: boolean; run_dir: string };

// One iteration's rolled-up state, indexed by `iter`.
export type IterationInfo = {
  iter: number;
  score?: number;
  passed?: boolean;
  breakdown?: ScoreBreakdown;
  runDir?: string;
};

// A rendered assembled machine for one iteration (canvas scrubbing).
export type AssembledInfo = {
  iter: number;
  run_dir: string;
  glbBlob?: Blob;
};
