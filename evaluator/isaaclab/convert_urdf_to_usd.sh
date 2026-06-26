#!/bin/bash
cd /work/IsaacLab
ln -sfn /isaac-sim _isaac_sim 2>/dev/null
echo "=== convert robot_dog URDF -> USD ==="
/isaac-sim/python.sh scripts/tools/convert_urdf.py \
  /work/robot_dog/robot_dog.urdf \
  /work/robot_dog/robot_dog.usd \
  --joint-stiffness 40 --joint-damping 2 --joint-target-type position \
  --headless 2>&1 | grep -iE "error|traceback|success|wrote|usd|joint|link|fail|saved|converting" | tail -25
echo "=== convert rc=${PIPESTATUS[0]} ==="
ls -la /work/robot_dog/robot_dog.usd 2>/dev/null || find /work/robot_dog -name "*.usd*" 2>/dev/null
