#!/bin/bash
# Leaf/size/speed chain (19/11, agreed with the laptop, after after_p6street.sh): sins branch leaf-connected (aebd06c, on
# cec8e5e = R0 code). Cumulative arms x3 alternated on CODa 9 tuning windows + street:
#   Q0 = R0 (coda_vjsye2i2pca / street pcaq) / QC = + connected mask leaf (pcalc) / QB = + cluster size limit = bus (pcalcb) /
#   QS = + association speed gate 15 m/s (pcalcbs); lab x3: LQ0 lab_uca / LQC lab_ucalc / LQS lab_ucalcs.
# Then e3 all + testing + vehicles. lab_local.sh builds the checked-out branch (stops on BUILD FAILED).
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
while pgrep -f after_p6street.sh > /dev/null; do sleep 60; done
echo "$(date +%T) queue before done"
git -C sins fetch -q origin; git -C sins checkout -q leaf-connected && git -C sins merge -q --ff-only origin/leaf-connected || { echo "CHECKOUT FAILED"; exit 1; }
git -C sins log --oneline -1
for p in lab_uca lab_ucalc lab_ucalcs; do python3 eval/configs/sins/make_cfg.py $p > /dev/null || exit 1; done   # installed by lab_local's build
CODA="Q0 coda_vjsye2i2pca;QC coda_vjsye2i2pcalc;QB coda_vjsye2i2pcalcb;QS coda_vjsye2i2pcalcbs" \
STREET="mdp_street_vjsye2i2pcaq mdp_street_vjsye2i2pcalc mdp_street_vjsye2i2pcalcb mdp_street_vjsye2i2pcalcbs" \
  bash eval/scripts/analysis/lab_local.sh || exit 1
ARMS="LQ0 lab_uca;LQC lab_ucalc;LQS lab_ucalcs" OUT=lab_leafc bash eval/scripts/analysis/lab_ab.sh
T=""; for t in Q0 QC QB QS; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in pcaq pcalc pcalcb pcalcbs; do for r in 1 2 3; do S="$S mdp_street_vjsye2i2${p}_r$r"; done; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
echo "$(date +%T) ALL DONE"
