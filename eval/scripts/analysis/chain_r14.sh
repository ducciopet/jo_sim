#!/bin/bash
# Laptop, after chain_r13 (diag): R11 = R9 + anchored fragment merge: street x1 first, then CODa tuning x1
cd /home/duccio/Desktop/jo_sim_sins
while pgrep -f "chain_[r]13.sh" > /dev/null; do sleep 20; done
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda 2>&1' | grep -E "error|Summary" | head
p=mdp_street_vjsyer11
touch bags/mdp/runs/${p}_r2.done bags/mdp/runs/${p}_r3.done
docker exec -u ros jo_sim_sins bash -c "PROFILES=$p bash /home/ros/bags/mdp/run_street.sh" > bags/mdp/run_street_$p.log 2>&1
echo "$(date +%T) $p done"
ENV=coda_vjsyer11 STEREO=1 bash eval/scripts/coda_ab.sh VR11 w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) VR11 done"
echo "R14 DONE"
