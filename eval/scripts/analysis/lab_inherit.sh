#!/bin/bash
# Lab PC (16/11): reference vjsye2 vs inherited motion state at track rebirth, x3, CODa tuning 0/10/16 + street.
#   I1 = track_inherit_motion_dist 1.0 m / age 1.0 s (C2 of 04/11, now on the current reference and evaluation)
#   I2 = 0.6 m / 0.5 s (stricter)
# The reference runs (LVE2*, mdp_street_vjsye2_r*) already exist on the lab. Then everything is re-scored with the
# 16/11 evaluation (rescore_e2.sh). Held-out 11/12 only afterwards, for the chosen candidate.
cd $(dirname $0)/../../..
export ADAPTER_ARGS="-p cloud_stride:=2"
git pull -q && git submodule update -q sins || { echo "PULL FAILED"; exit 1; }
git -C sins log --oneline -1
CODA="LVI1 coda_vjsye2i1;LVI2 coda_vjsye2i2" STREET="mdp_street_vjsye2i1 mdp_street_vjsye2i2" bash eval/scripts/analysis/lab_local.sh
T=""; for t in LVI1 LVI2; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in mdp_street_vjsye2i1 mdp_street_vjsye2i2; do S="$S ${p}_r1 ${p}_r2 ${p}_r3"; done
TAGS="$T" STREET="$S" bash eval/scripts/analysis/rescore_e2.sh
echo "$(date +%T) LAB INHERIT DONE"
