#!/usr/bin/env bash
# Build MP4s for every loop iteration that has frames, on the box. Idempotent.
#   build_loop_mp4s.sh <loop_workdir>
set -uo pipefail
WD="${1:-/data/physcad/loop_cassie}"
MK="/data/physcad/physcad_evaluator/make_mp4.sh"
mkdir -p /data/physcad/videos
for it in "$WD"/iter_*; do
  [ -d "$it/frames" ] || continue
  n=$(ls "$it/frames"/*.png 2>/dev/null | wc -l)
  [ "$n" -gt 0 ] || continue
  out="/data/physcad/videos/$(basename "$WD")_$(basename "$it").mp4"
  bash "$MK" "$it/frames" "$out" 30 2>/dev/null && echo "built $out ($n frames)"
done
