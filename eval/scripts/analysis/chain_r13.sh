#!/bin/bash
# Laptop: diagnostic street run of R10 with the path-proof failure counters
cd /home/duccio/Desktop/jo_sim_sins
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda 2>&1' | grep -E "error|Summary" | head
p=mdp_street_vjsyer10dbg
touch bags/mdp/runs/${p}_r2.done bags/mdp/runs/${p}_r3.done
docker exec -u ros jo_sim_sins bash -c "PROFILES=$p bash /home/ros/bags/mdp/run_street.sh" > bags/mdp/run_street_$p.log 2>&1
echo "R13 DONE"
