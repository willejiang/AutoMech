#!/bin/bash
cd /work/IsaacLab
ln -sfn /isaac-sim _isaac_sim 2>/dev/null
echo "=== render trained Cassie policy (play.py) with video capture ==="
echo "start: $(date)"
# play the Play variant, few envs, record video. --enable_cameras needed for video headless.
/isaac-sim/python.sh scripts/reinforcement_learning/rsl_rl/play.py \
  --task Isaac-Velocity-Flat-Cassie-Play-v0 \
  --headless --enable_cameras \
  --num_envs 16 \
  --video --video_length 200 \
  2>&1 | grep -iE "loading|checkpoint|video|saved|error|traceback|recording|\.mp4|step" | tail -30
echo "=== play rc=${PIPESTATUS[0]} end: $(date) ==="
echo "=== videos produced? ==="
find /work/IsaacLab/logs -name "*.mp4" 2>/dev/null | tail -5
