#!/bin/bash
# S4 of eval/PIPELINE.md (19/11): ground model, 3 arms x3 alternated, CODa 9 tuning windows + street, e3 all + testing
# + vehicles: G0 = R0 (grid) / GL = planes, floor from the LiDAR (floor_source lidar) / GC = planes, floor from the
# cameras' depth (floor_source camera). sins yolo-assoc build (new switches off).
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
CODA="G0 coda_vjsye2i2pcag;GL coda_vjsye2i2pcapl;GC coda_vjsye2i2pcapc" \
STREET="mdp_street_vjsye2i2pcag mdp_street_vjsye2i2pcapl mdp_street_vjsye2i2pcapc" bash eval/scripts/analysis/lab_local.sh
T=""; for t in G0 GL GC; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in pcag pcapl pcapc; do S="$S mdp_street_vjsye2i2${p}_r1 mdp_street_vjsye2i2${p}_r2 mdp_street_vjsye2i2${p}_r3"; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
echo "$(date +%T) ALL DONE"
