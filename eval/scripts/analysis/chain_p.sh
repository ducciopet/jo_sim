#!/bin/bash
# Laptop, 1 rep: point-to-surface motion evidence (P1-P3) and fresh-frame class decay (B1) on CODa 0/10/16_w1,
# after the camera-pose / clusters-cloud fixes (reference = DBF, the same code with only the debug CSV on).
cd /home/duccio/Desktop/jo_sim_sins
while pgrep -f "chain_dbf.sh" > /dev/null; do sleep 20; done
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > bags/coda/build_p.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
W="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1"
for p in "KP1 coda_p1" "KB1 coda_b1" "KP3 coda_p3" "KP2 coda_p2"; do set -- $p
  [ "$(df --output=avail -BG . | tail -1 | tr -dc 0-9)" -lt 1 ] && { echo "DISK LOW"; exit 1; }
  echo "$(date +%T) $1 ($2)"; ENV=$2 STEREO=1 bash eval/scripts/coda_ab.sh $1 $W
done
echo "$(date +%T) P DONE"
