#!/bin/bash
# P6/P7 chain (19/11), sins branch p5b-heading (build of 18/11 21:19). Neither P5 nor P5b won -> no extent prior arm.
#   CODa 9 tuning windows + street x3 alternated: X0 = R0 / XL = R0 + P6 (track_vehicle_lshape) /
#   XS = R0 + P6 + P7 (track_shape_memory); lab x3 (last variant only): LX0 = lab_uca / LXS = lab_ucals.
# Then e3 all + testing + vehicles.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
git -C sins checkout -q p5b-heading || { echo "CHECKOUT FAILED"; exit 1; }
for p in lab_uca lab_ucals; do python3 eval/configs/sins/make_cfg.py $p > /dev/null || exit 1; done   # installed by lab_local's build
CODA="X0 coda_vjsye2i2pca;XL coda_vjsye2i2pcal;XS coda_vjsye2i2pcals" \
STREET="mdp_street_vjsye2i2pcax mdp_street_vjsye2i2pcal mdp_street_vjsye2i2pcals" bash eval/scripts/analysis/lab_local.sh
ARMS="LX0 lab_uca;LXS lab_ucals" OUT=lab_p67 bash eval/scripts/analysis/lab_ab.sh
T=""; for t in X0 XL XS; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in pcax pcal pcals; do for r in 1 2 3; do S="$S mdp_street_vjsye2i2${p}_r$r"; done; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
echo "$(date +%T) ALL DONE"
