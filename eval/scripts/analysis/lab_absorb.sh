#!/bin/bash
# Lab PC (17/11): reference vjsye2 vs orphan-piece absorption (A2 = vjsye2a, track_absorb_pieces), x3, CODa tuning
# 0/10/16 + street, then one more reference repetition (LVE2L) on CODa in the same session for a fair latency
# comparison. Re-scored with the e2 evaluation (+ CODa vehicle view). Held-out 11/12 only afterwards, if chosen.
cd $(dirname $0)/../../..
export ADAPTER_ARGS="-p cloud_stride:=2"
git -C sins log --oneline -1
CODA="LVA2 coda_vjsye2a" STREET="mdp_street_vjsye2a" bash eval/scripts/analysis/lab_local.sh
CODA="LVE2L coda_vjsye2" STREET=none REPS="_" bash eval/scripts/analysis/lab_local.sh
TAGS="LVA2 LVA2r2 LVA2r3 LVE2L" STREET="mdp_street_vjsye2a_r1 mdp_street_vjsye2a_r2 mdp_street_vjsye2a_r3" bash eval/scripts/analysis/rescore_e2.sh
TAGS="LVA2 LVA2r2 LVA2r3 LVE2L" bash eval/scripts/analysis/coda_vehicle_e2.sh
echo "$(date +%T) LAB ABSORB DONE"
