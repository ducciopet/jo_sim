#!/bin/bash
# Laptop, after chain_rf.sh: R without disocclusion — street x1, CODa tuning x1 (tag VRD)
cd /home/duccio/Desktop/jo_sim_sins
while pgrep -f "chain_[r]f.sh" > /dev/null; do sleep 60; done
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' | tail -1
p=mdp_street_vjsyerd
touch bags/mdp/runs/${p}_r2.done bags/mdp/runs/${p}_r3.done
docker exec -u ros jo_sim_sins bash -c "PROFILES=$p bash /home/ros/bags/mdp/run_street.sh" > bags/mdp/run_street_$p.log 2>&1
echo "$(date +%T) $p done"
ENV=coda_vjsyerd STEREO=1 bash eval/scripts/coda_ab.sh VRD w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) RD DONE"
