#!/bin/bash
# Laptop: after the in-flight CODa run (R3 + occlusion, built with the point veto hard-wired -> renamed VR3O),
# rebuild with track_travel_proof_point_veto (default off); R4 = R2 + occlusion edges, R3G = R2 + F(0.3): CODa, street
cd /home/duccio/Desktop/jo_sim_sins
while pgrep -f "coda_[a]b.sh" > /dev/null; do sleep 20; done
for f in eval/results/coda/VR4_*.json; do mv $f ${f/VR4_/VR3O_}; done
echo "$(date +%T) VR3O done"
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda 2>&1' | grep -E "error|Summary" | head
rm -rf bags/coda/runs/VR4_v2_*
ENV=coda_vjsyer4 STEREO=1 bash eval/scripts/coda_ab.sh VR4 w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) VR4 done"
ENV=coda_vjsyer3g STEREO=1 bash eval/scripts/coda_ab.sh VR3G w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
echo "$(date +%T) VR3G done"
for p in mdp_street_vjsyer4 mdp_street_vjsyer3g; do
  touch bags/mdp/runs/${p}_r2.done bags/mdp/runs/${p}_r3.done
  docker exec -u ros jo_sim_sins bash -c "PROFILES=$p bash /home/ros/bags/mdp/run_street.sh" > bags/mdp/run_street_$p.log 2>&1
  echo "$(date +%T) $p done"
done
echo "R5 DONE"
