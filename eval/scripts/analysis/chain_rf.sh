#!/bin/bash
# Laptop: R (motion proven by the path) and R+F (object-scale fragment merge) — street x1, then CODa tuning x1
cd /home/duccio/Desktop/jo_sim_sins
for p in mdp_street_vjsyer mdp_street_vjsyerf; do
  touch bags/mdp/runs/${p}_r2.done bags/mdp/runs/${p}_r3.done
  docker exec -u ros jo_sim_sins bash -c "PROFILES=$p bash /home/ros/bags/mdp/run_street.sh" > bags/mdp/run_street_$p.log 2>&1
  echo "$(date +%T) $p done"
done
ENV=coda_vjsyer STEREO=1 bash eval/scripts/coda_ab.sh VR w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) VR done"
ENV=coda_vjsyerf STEREO=1 bash eval/scripts/coda_ab.sh VRF w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) RF DONE"
