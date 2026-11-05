#!/bin/bash
# N rule (track_motion_min_net_footprints 0.5): CODa tuning 0+10+16_w1 x3 (KW1N = coda_w1n, KWN = coda_wn), then
# Marina di Pisa street x3 (mdp_street_wn, mdp_street_w1n), then scoring.
cd /home/duccio/Desktop/jo_sim_sins
W="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1"
for r in "" r2 r3; do
  for p in "KW1N coda_w1n" "KWN coda_wn"; do set -- $p
    echo "$(date +%T) $1$r ($2)"; ENV=$2 STEREO=1 bash eval/scripts/coda_ab.sh $1$r $W
  done
done
echo "CODA DONE"
docker exec -u ros jo_sim_sins bash -c 'PROFILES="mdp_street_wn mdp_street_w1n" bash /home/ros/bags/mdp/run_street.sh' > bags/mdp/run_street_n.log 2>&1
PROFILES="mdp_street_wn mdp_street_w1n" bash eval/scripts/analysis/score_street.sh r1 r2 r3
echo "N DONE"
