#!/usr/bin/env bash
# Stitch an iteration's frames into an MP4 on the box.
#   make_mp4.sh <frames_dir> <out.mp4> [fps]
# Frames are rgb_XXXX.png (BasicWriter). Slow-ish sims -> low fps reads well.
set -euo pipefail
FRAMES="$1"; OUT="$2"; FPS="${3:-12}"
N=$(ls "$FRAMES"/rgb_*.png 2>/dev/null | wc -l)
if [ "$N" -eq 0 ]; then echo "no frames in $FRAMES"; exit 1; fi
# BasicWriter names are zero-padded rgb_0000.png — use glob pattern in order
ffmpeg -y -framerate "$FPS" -pattern_type glob -i "$FRAMES/rgb_*.png" \
  -c:v libx264 -pix_fmt yuv420p -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" "$OUT" 2>/dev/null
echo "wrote $OUT ($N frames @ ${FPS}fps -> $(stat -c%s "$OUT" 2>/dev/null) bytes)"
