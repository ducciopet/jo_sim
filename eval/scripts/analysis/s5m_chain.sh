#!/bin/bash
# S5 of eval/PIPELINE.md + M confirmation (18/11), one chain, arms alternated:
#   Y0 = R0 (coda_vjsye2i2pca / street pcay), YM = R0 + YOLO identity per camera (M: pcm / street pcmy),
#   Y6 = R0 with the YOLO input at 640 px (pca6).
# Part A: Y0 / YM / Y6 x3 on the 9 CODa tuning windows + street x3.
# Part B: Y0 / YM reps 4-5 on CODa 10 + 16 (seq 0 too noisy, x3 only) + street reps 4-5 -> M is x5 on 10, 16, street.
# Then e3 all + testing + vehicles. sins yolo-assoc build (new switches off unless the profile sets them).
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
CODA="Y0 coda_vjsye2i2pca;YM coda_vjsye2i2pcm;Y6 coda_vjsye2i2pca6" \
STREET="mdp_street_vjsye2i2pcay mdp_street_vjsye2i2pcmy mdp_street_vjsye2i2pca6" bash eval/scripts/analysis/lab_local.sh
CODA="Y0 coda_vjsye2i2pca;YM coda_vjsye2i2pcm" WINDOWS="10_w1 10_w2 10_w3 10_w4 10_w5 16_w1" REPS="r4 r5" \
STREET="mdp_street_vjsye2i2pcay mdp_street_vjsye2i2pcmy" SREPS="4 5" bash eval/scripts/analysis/lab_local.sh
T=""; for t in Y0 YM Y6; do T="$T $t ${t}r2 ${t}r3"; done; T="$T Y0r4 Y0r5 YMr4 YMr5"
S=""; for p in pcay pcmy; do for r in 1 2 3 4 5; do S="$S mdp_street_vjsye2i2${p}_r$r"; done; done
for r in 1 2 3; do S="$S mdp_street_vjsye2i2pca6_r$r"; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
echo "$(date +%T) ALL DONE"
