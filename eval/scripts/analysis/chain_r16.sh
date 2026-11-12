#!/bin/bash
# Laptop: R12P = R12 + person veto on the motion groups: CODa tuning x1 first (seq 0 pedestrians), then street x1
cd /home/duccio/Desktop/jo_sim_sins
ENV=coda_vjsyer12p STEREO=1 bash eval/scripts/coda_ab.sh VR12P w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) VR12P done"
p=mdp_street_vjsyer12p
touch bags/mdp/runs/${p}_r2.done bags/mdp/runs/${p}_r3.done
docker exec -u ros jo_sim_sins bash -c "PROFILES=$p bash /home/ros/bags/mdp/run_street.sh" > bags/mdp/run_street_$p.log 2>&1
echo "$(date +%T) $p done"
echo "R16 DONE"
