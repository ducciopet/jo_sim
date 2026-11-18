#!/bin/bash
# Night queue (18-19/11): wait for s5m_chain.sh, then sins p5b-heading build check, p5b_chain.sh, then the motion-threshold
# sensitivity rescore (rescore_dyn.sh, DYN 0.2 / 0.5) of R0 x3 (Y0) + Dynablox / M-detector. Stops on a build failure.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
while pgrep -f s5m_chain.sh > /dev/null; do sleep 60; done
echo "$(date +%T) s5m done"
git -C sins checkout -q p5b-heading || { echo "CHECKOUT FAILED"; exit 1; }
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > bags/build_p5b.log 2>&1 || { echo "BUILD FAILED (bags/build_p5b.log)"; exit 1; }
echo "$(date +%T) build ok"
bash eval/scripts/analysis/p5b_chain.sh
B=/mnt/nas-imperatore/datasets_SINS/derived/baselines_out
for d in 0.2 0.5; do DYN=$d TAGS="Y0 Y0r2 Y0r3" BASE="dynablox:$B/dynabloxF_coda{c}.bag:dynablox mdetector:$B/mdetectorF_coda{c}.bag:mdetector" \
  bash eval/scripts/analysis/rescore_dyn.sh; done
echo "$(date +%T) QUEUE DONE"
