#!/bin/bash
# Laptop, 1 rep, after chain_p: point-shift VETO (V), outside-FOV association gate = inside (A), jitter allowance alone (J)
# on CODa 0/10/16_w1, then the street (V, A, J). Reference = DBF (CODa) / mdp_street_dbf_r1 (street).
cd /home/duccio/Desktop/jo_sim_sins
while pgrep -f "chain_p.sh" > /dev/null; do sleep 20; done
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > bags/coda/build_q.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
W="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1"
for p in "KV coda_v" "KA coda_a" "KJ coda_j"; do set -- $p
  [ "$(df --output=avail -BG . | tail -1 | tr -dc 0-9)" -lt 1 ] && { echo "DISK LOW"; exit 1; }
  echo "$(date +%T) $1 ($2)"; ENV=$2 STEREO=1 bash eval/scripts/coda_ab.sh $1 $W
done
echo "$(date +%T) CODA DONE"
for p in mdp_street_v mdp_street_a mdp_street_j; do touch bags/mdp/runs/${p}_r2.done bags/mdp/runs/${p}_r3.done; done
docker exec -u ros jo_sim_sins bash -c 'PROFILES="mdp_street_v mdp_street_a mdp_street_j" bash /home/ros/bags/mdp/run_street.sh' > bags/mdp/run_street_q.log 2>&1
echo "$(date +%T) Q DONE"
