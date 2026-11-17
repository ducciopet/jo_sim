#!/bin/bash
# Lab PC (17/11): final held-out check of the chosen candidate I2P (vjsye2i2p: inheritance 0.6 m / 0.5 s + probation
# 0.5 s / 0.5; sins branch inherit-guard d3400bc, local) vs the reference vjsye2 run in the SAME chain (the 13/11
# held-out reference LHVE2 ran under different timing). x3, CODa held-out 11/12, alternated per repetition.
cd $(dirname $0)/../../..
export ADAPTER_ARGS="-p cloud_stride:=2"
git -C sins log --oneline -1
W="11_w1 11_w2 11_w3 12_w1 12_w2 12_w3"
CODA="LHE2L coda_vjsye2;LHI2P coda_vjsye2i2p" WINDOWS="$W" STREET=none bash eval/scripts/analysis/lab_local.sh
T=""; for t in LHE2L LHI2P; do T="$T $t ${t}r2 ${t}r3"; done
TAGS="$T" WINDOWS="$W" bash eval/scripts/analysis/rescore_e2.sh
echo "$(date +%T) LAB HELDOUT I2P DONE"
