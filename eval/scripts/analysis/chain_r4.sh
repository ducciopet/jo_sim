#!/bin/bash
# Laptop, after chain_r3.sh: R4 (occlusion-edge veto) and R3 + F at gap ratio 0.3 — CODa tuning x1, then street x1
cd /home/duccio/Desktop/jo_sim_sins
while pgrep -f "chain_[r]3.sh" > /dev/null; do sleep 20; done
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda 2>&1' | grep -E "error|Summary" | head
ENV=coda_vjsyer4 STEREO=1 bash eval/scripts/coda_ab.sh VR4 w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) VR4 done"
ENV=coda_vjsyer3g STEREO=1 bash eval/scripts/coda_ab.sh VR3G w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) VR3G done"
for p in mdp_street_vjsyer4 mdp_street_vjsyer3g; do
  touch bags/mdp/runs/${p}_r2.done bags/mdp/runs/${p}_r3.done
  docker exec -u ros jo_sim_sins bash -c "PROFILES=$p bash /home/ros/bags/mdp/run_street.sh" > bags/mdp/run_street_$p.log 2>&1
  echo "$(date +%T) $p done"
done
echo "R4 DONE"
