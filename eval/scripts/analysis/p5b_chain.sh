#!/bin/bash
# After s5m_chain.sh (18-19/11): sins branch p5b-heading (yolo-assoc + track_vehicle_extent_prior_heading, default off).
#   CODa 9 tuning windows + street x3: Z0 = R0 / ZH = R0 + P5b (heading-gated vehicle extent prior);
#   lab x3: LZ0 = lab_uca (ground planes) / LZG = lab_ucag (grid, the ground model of CODa and the street).
# Then e3 all + testing + vehicles.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
git -C sins checkout -q p5b-heading || { echo "CHECKOUT FAILED"; exit 1; }
for p in lab_uca lab_ucag; do python3 eval/configs/sins/make_cfg.py $p > /dev/null || exit 1; done   # installed by lab_local's build
CODA="Z0 coda_vjsye2i2pca;ZH coda_vjsye2i2pcah" \
STREET="mdp_street_vjsye2i2pcaz mdp_street_vjsye2i2pcah" bash eval/scripts/analysis/lab_local.sh
ARMS="LZ0 lab_uca;LZG lab_ucag" OUT=lab_ground bash eval/scripts/analysis/lab_ab.sh
T=""; for t in Z0 ZH; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in pcaz pcah; do for r in 1 2 3; do S="$S mdp_street_vjsye2i2${p}_r$r"; done; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
echo "$(date +%T) ALL DONE"
