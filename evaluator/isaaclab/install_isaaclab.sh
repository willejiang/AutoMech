#!/bin/bash
set -e
cd /work/IsaacLab
ln -sfn /isaac-sim _isaac_sim
if ! command -v sudo >/dev/null 2>&1; then
  printf '#!/bin/bash\nexec "$@"\n' > /usr/local/bin/sudo && chmod +x /usr/local/bin/sudo
fi
export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
export DEBIAN_FRONTEND=noninteractive
echo "=== ./isaaclab.sh --install rl (correct token: pulls rsl_rl/rl_games) ==="
./isaaclab.sh --install rl
echo "=== INSTALL DONE rc=$? ==="
# quick sanity: can we import isaaclab + an RL lib?
/isaac-sim/python.sh -c "import isaaclab; print('isaaclab import OK')" 2>&1 | tail -2
/isaac-sim/python.sh -c "import rsl_rl; print('rsl_rl import OK')" 2>&1 | tail -2 || echo "rsl_rl not importable"
