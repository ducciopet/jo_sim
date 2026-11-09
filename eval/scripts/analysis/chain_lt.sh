#!/bin/bash
# Laptop, 1 rep: latency work on CODa (adapter cloud_stride 2 = RealSense-style decimation of the simulated camera driver)
# LT2 = depth_input_max_points 150k + adaptive fusion wait; LT3 = LT2 + fusion timeout 50 ms; then street lt2.
cd /home/duccio/Desktop/jo_sim_sins
W="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1"
for p in "LT2 coda_lt2" "LT3 coda_lt3"; do set -- $p
  echo "$(date +%T) $1"; ADAPTER_ARGS="-p cloud_stride:=2" ENV=$2 STEREO=1 bash eval/scripts/coda_ab.sh $1 $W
done
touch bags/mdp/runs/mdp_street_lt2_r2.done bags/mdp/runs/mdp_street_lt2_r3.done
docker exec -u ros jo_sim_sins bash -c 'PROFILES="mdp_street_lt2" bash /home/ros/bags/mdp/run_street.sh' > bags/mdp/run_street_lt2.log 2>&1
echo "$(date +%T) LT DONE"
