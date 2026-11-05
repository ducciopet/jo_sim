#!/bin/bash
# Experiments still pending at the laptop -> lab PC move (2026-11-05), in order. Run on the host from the repo root:
#   nohup bash eval/scripts/analysis/lab_resume.sh > bags/coda/lab_resume.log 2>&1 &
# 1) KWNr3 (coda_wn) seq 10 w4/w5: lost to DDS UDP write failures on the laptop
# 2) CODa tuning 0+10+16_w1 x3: KWNC (coda_wnc) and KWNCJM (coda_wncjm)
# 3) Marina di Pisa street x3: mdp_street_wnc, mdp_street_wncjm
# 4) street rep 1 of mdp_street_wn / mdp_street_w1n (also lost to DDS failures; reps 2-3 are scored)
# The container must have the sins build with the C/J/M rules (sins >= c807344): colcon build first (step 0).
cd $(dirname $0)/../../..
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > bags/coda/build_lab.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
for n in coda_wnc coda_wncjm mdp_street_wnc mdp_street_wncjm; do python3 eval/configs/sins/make_cfg.py $n > /dev/null; done
echo "$(date +%T) KWNr3 rerun"; ENV=coda_wn STEREO=1 bash eval/scripts/coda_ab.sh KWNr3 10_w4 10_w5
W="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1"
for r in "" r2 r3; do
  for p in "KWNC coda_wnc" "KWNCJM coda_wncjm"; do set -- $p
    echo "$(date +%T) $1$r ($2)"; ENV=$2 STEREO=1 bash eval/scripts/coda_ab.sh $1$r $W
  done
done
echo "CODA DONE"
mkdir -p bags/mdp/runs; for P in mdp_street_wn mdp_street_w1n; do for r in r2 r3; do touch bags/mdp/runs/${P}_$r.done; done; done   # scored on the laptop
docker exec -u ros jo_sim_sins bash -c 'PROFILES="mdp_street_wnc mdp_street_wncjm mdp_street_wn mdp_street_w1n" bash /home/ros/bags/mdp/run_street.sh' > bags/mdp/run_street_lab.log 2>&1
PROFILES="mdp_street_wnc mdp_street_wncjm mdp_street_wn mdp_street_w1n" bash eval/scripts/analysis/score_street.sh r1 r2 r3
echo "ALL DONE"
