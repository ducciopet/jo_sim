#!/bin/bash
# Laptop: R5 = R (permissive proof) + occlusion-edge veto: does the occlusion veto alone keep the street vehicle gain of R
# and remove its CODa false positives (seq 0 rows of static objects, seq 10 wall patches)? CODa tuning x1, street x1
cd /home/duccio/Desktop/jo_sim_sins
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda 2>&1' | grep -E "error|Summary" | head
ENV=coda_vjsyer5 STEREO=1 bash eval/scripts/coda_ab.sh VR5 w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) VR5 done"
p=mdp_street_vjsyer5
touch bags/mdp/runs/${p}_r2.done bags/mdp/runs/${p}_r3.done
docker exec -u ros jo_sim_sins bash -c "PROFILES=$p bash /home/ros/bags/mdp/run_street.sh" > bags/mdp/run_street_$p.log 2>&1
echo "$(date +%T) $p done"
echo "R6 DONE"
