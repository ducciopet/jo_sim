#!/bin/bash
# P6 street re-run (19/11): in p67_chain.sh the street arm "pcal" collided with an older R0 re-run of the same name
# (its .done files were still there), so the street P6 runs were skipped and the old runs scored. Street x3 alternated:
# pcaw = R0 / pcp6 = R0 + P6 (track_vehicle_lshape). CODa skipped (results of w1 exist). Then e3 street rescore.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
git -C sins checkout -q p5b-heading || { echo "CHECKOUT FAILED"; exit 1; }
CODA="X0 coda_vjsye2i2pca" WINDOWS="w1" STREET="mdp_street_vjsye2i2pcaw mdp_street_vjsye2i2pcp6" bash eval/scripts/analysis/lab_local.sh
S=""; for p in pcaw pcp6; do for r in 1 2 3; do S="$S mdp_street_vjsye2i2${p}_r$r"; done; done
TAGS="" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
echo "$(date +%T) ALL DONE"
