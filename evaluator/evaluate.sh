#!/usr/bin/env bash
# PhysCAD evaluator — single entry point the maker loop calls.
#   ./evaluate.sh <design_dir>
# where <design_dir> contains manifest.json + the .scad/.stl it references.
#
# Runs the SIM half inside the isaac-sim container, then the VLM half on the host.
# Emits <design_dir>/out/result.json (the maker handoff).
set -euo pipefail

DESIGN_DIR="$(cd "$1" && pwd)"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="nvcr.io/nvidia/isaac-sim:6.0.1"
ISAAC_CACHE="/data/isaac-cache"
OUT="${DESIGN_DIR}/out"
mkdir -p "$OUT" "$ISAAC_CACHE"

echo "[evaluate] design dir: $DESIGN_DIR"

# ---- 1. SIM half: run_eval.py inside the container ----
# Mount: the code (ro), the design dir (rw), and Isaac caches (so shaders persist).
docker run --rm --runtime=nvidia --gpus all \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
  -v "${ROOT}:/code:ro" \
  -v "${DESIGN_DIR}:/work" \
  -v "${ISAAC_CACHE}/kit:/root/.local/share/ov/data/Kit" \
  -v "${ISAAC_CACHE}/cache:/root/.cache" \
  "$IMAGE" \
  ./python.sh /code/run_eval.py \
    --manifest /work/manifest.json \
    --out /work/out \
    --headless 1

# ---- 2. VLM half: analyze.py on the host (Azure key stays out of the image) ----
# shellcheck disable=SC1091
set -a; source "${ROOT}/.env"; set +a
python3 "${ROOT}/analyze.py" \
  --sim-result "${OUT}/sim_result.json" \
  --manifest "${DESIGN_DIR}/manifest.json" \
  --out "${OUT}/result.json"

echo "[evaluate] DONE -> ${OUT}/result.json"
cat "${OUT}/result.json" | python3 -c "import json,sys; r=json.load(sys.stdin); print(f\"  passed={r['passed']}  failures={len(r['failures'])}\")"
