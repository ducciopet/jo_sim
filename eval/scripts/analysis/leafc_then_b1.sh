#!/bin/bash
# 19/11 order changed by the user (via the laptop): leaf/size/speed chain now, then B1 (Dynablox + M-detector on the street).
# p6street_chain stopped (2 of 3 pairs done, not rerun), P6 FP diagnosis skipped.
cd $(dirname $0)/../../..
bash eval/scripts/analysis/leafc_chain.sh
echo "$(date +%T) leafc rc $?"
bash eval/baselines/street_baselines.sh
echo "$(date +%T) QUEUE DONE"
