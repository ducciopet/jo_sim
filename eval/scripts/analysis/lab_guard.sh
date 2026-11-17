#!/bin/bash
# Lab PC (17/11): I2 + inheritance guards (sins branch inherit-guard d3400bc, local; no git pull / submodule update
# here, it would reset sins). I2P = probation 0.5 s / 0.5, I2SP = + donor size check; x3 CODa tuning + street, then
# one reference repetition (LVE2M) on CODa as a check of the session drift (LVE2L x3 = 17/11 reference).
cd $(dirname $0)/../../..
export ADAPTER_ARGS="-p cloud_stride:=2"
git -C sins log --oneline -1
CODA="LVI2P coda_vjsye2i2p;LVI2SP coda_vjsye2i2sp" STREET="mdp_street_vjsye2i2p mdp_street_vjsye2i2sp" bash eval/scripts/analysis/lab_local.sh
CODA="LVE2M coda_vjsye2" STREET=none REPS="_" bash eval/scripts/analysis/lab_local.sh
T="LVE2M"; for t in LVI2P LVI2SP; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in mdp_street_vjsye2i2p mdp_street_vjsye2i2sp; do S="$S ${p}_r1 ${p}_r2 ${p}_r3"; done
TAGS="$T" STREET="$S" bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" bash eval/scripts/analysis/coda_vehicle_e2.sh
echo "$(date +%T) LAB GUARD DONE"
