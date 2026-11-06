#!/bin/bash
# Laptop (10 Hz real time): WNC vs WNCE (= WNC + lidar_eps_far 0.45 -> 0.7), CODa tuning seqs x3, then Marina di Pisa street x3, then scoring.
cd /home/duccio/Desktop/jo_sim_sins
free_gb() { df --output=avail -BG . | tail -1 | tr -dc 0-9; }
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > bags/coda/build_e.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
echo "$(date +%T) built"
W="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1"
for r in "" r2 r3; do
  for p in "KWNC coda_wnc" "KWNCE coda_wnce"; do set -- $p
    [ "$(free_gb)" -lt 1 ] && { echo "DISK LOW, stop"; exit 1; }
    echo "$(date +%T) $1$r ($2)"; ENV=$2 STEREO=1 bash eval/scripts/coda_ab.sh $1$r $W
  done
done
echo "$(date +%T) CODA DONE"
docker exec -u ros jo_sim_sins bash -c 'PROFILES="mdp_street_wnc mdp_street_wnce" bash /home/ros/bags/mdp/run_street.sh' > bags/mdp/run_street_e.log 2>&1
PROFILES="mdp_street_wnc mdp_street_wnce" bash eval/scripts/analysis/score_street.sh r1 r2 r3
echo "$(date +%T) E DONE"
