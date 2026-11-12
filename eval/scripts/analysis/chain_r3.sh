#!/bin/bash
# Laptop, after the VR2F CODa runs: R3 (= R2 + point-shift veto respected) and R3+F — CODa tuning x1, street x1
cd /home/duccio/Desktop/jo_sim_sins
while pgrep -f "coda_[a]b.sh" > /dev/null; do sleep 20; done
echo "$(date +%T) VR2F done"
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' | tail -1
ENV=coda_vjsyer3 STEREO=1 bash eval/scripts/coda_ab.sh VR3 w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) VR3 done"
ENV=coda_vjsyer3f STEREO=1 bash eval/scripts/coda_ab.sh VR3F w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) VR3F done"
for p in mdp_street_vjsyer3f mdp_street_vjsyer3; do
  touch bags/mdp/runs/${p}_r2.done bags/mdp/runs/${p}_r3.done
  docker exec -u ros jo_sim_sins bash -c "PROFILES=$p bash /home/ros/bags/mdp/run_street.sh" > bags/mdp/run_street_$p.log 2>&1
  echo "$(date +%T) $p done"
done
echo "R3 DONE"
