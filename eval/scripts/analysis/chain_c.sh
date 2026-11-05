#!/bin/bash
# After the N chain + KWNr3 rerun: build (C, J, M rules), then CODa tuning x3 (KWNC = coda_wnc, KWNCJM = coda_wncjm),
# then Marina di Pisa street x3 (mdp_street_wnc, mdp_street_wncjm), then scoring.
cd /home/duccio/Desktop/jo_sim_sins
until grep -q "RERUN DONE" bags/coda/rerun_kwnr3.log 2>/dev/null; do sleep 60; done
rm -rf bags/coda/runs/KW1Nr2_* bags/coda/runs/KW1Nr3_* bags/coda/runs/KWNr2_*
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > bags/coda/build_c.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
echo "$(date +%T) built"
W="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1"
for r in "" r2 r3; do
  for p in "KWNC coda_wnc" "KWNCJM coda_wncjm"; do set -- $p
    echo "$(date +%T) $1$r ($2)"; ENV=$2 STEREO=1 bash eval/scripts/coda_ab.sh $1$r $W
  done
done
echo "CODA DONE"
docker exec -u ros jo_sim_sins bash -c 'PROFILES="mdp_street_wnc mdp_street_wncjm" bash /home/ros/bags/mdp/run_street.sh' > bags/mdp/run_street_c.log 2>&1
PROFILES="mdp_street_wnc mdp_street_wncjm" bash eval/scripts/analysis/score_street.sh r1 r2 r3
echo "C DONE"
