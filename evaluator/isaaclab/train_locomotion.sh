#!/bin/bash
cd /work/IsaacLab
ln -sfn /isaac-sim _isaac_sim 2>/dev/null
echo "=== train Cassie flat-velocity, headless, SHORT run for feasibility ==="
echo "start: $(date)"
# small env count + few iterations just to confirm it RUNS + measure speed
/isaac-sim/python.sh scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Flat-Cassie-v0 \
  --headless \
  --num_envs 1024 \
  --max_iterations 50 \
  2>&1 | grep -iE "iteration|reward|fps|step|error|traceback|saved|Mean|Episode|learning|exact|^\[" | tail -60
echo "=== train rc=${PIPESTATUS[0]} end: $(date) ==="
echo "=== checkpoint produced? ==="
find /work/IsaacLab/logs -name "*.pt" 2>/dev/null | tail -3
