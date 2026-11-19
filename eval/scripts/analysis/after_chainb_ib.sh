#!/bin/bash
# 20/11, after chainb.sh: second evaluation e3 + --ignore-box (rescore_e3ib.sh) on chain B (CB*), LEAFC (Q*), their street
# runs, Dynablox / M-detector on CODa (all 15 windows, NAS baselines_out) and on the street. Analysis only, no runs.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
while pgrep -f chainb.sh > /dev/null; do sleep 60; done
T=""; for t in CB0 CBW CBN CBT Q0 QC QB QS; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in pcar1 pcar1wb pcar1n pcar1t pcaq pcalc pcalcb pcalcbs; do for r in 1 2 3; do S="$S mdp_street_vjsye2i2${p}_r$r"; done; done
TAGS="$T" STREET="$S" FRAMES=testing bash eval/scripts/analysis/rescore_e3ib.sh
B=/mnt/nas-imperatore/datasets_SINS/derived/baselines_out
BASE="dynablox:$B/dynabloxF_coda{c}.bag:dynablox mdetector:$B/mdetectorF_coda{c}.bag:mdetector" FRAMES=testing \
  WINDOWS="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 11_w1 11_w2 11_w3 12_w1 12_w2 12_w3" bash eval/scripts/analysis/rescore_e3ib.sh
G=bags/mdp/mdp_street
for m in dynablox mdetector; do X=""; [ $m = mdetector ] && X="--points-min-cluster 5"
  A="--method $m --run $B/${m}F_street.bag --odom ${G}_odom --range 15 --ignore-undefined --ignore-box $X"
  python3 eval/coda_eval.py $A --gt ${G}_gt.csv --out eval/results/mdp_e3ib/${m}_street.json > /dev/null || echo "FAIL $m street"
  python3 eval/coda_eval.py $A --gt ${G}_vehicle_gt.csv --ignore-gt ${G}_person_gt.csv --out eval/results/mdp_e3ib/${m}_street_vehicle.json > /dev/null
  python3 eval/coda_eval.py $A --gt ${G}_person_gt.csv --ignore-gt ${G}_vehicle_gt.csv --out eval/results/mdp_e3ib/${m}_street_person.json > /dev/null
done
echo "$(date +%T) IB DONE"
