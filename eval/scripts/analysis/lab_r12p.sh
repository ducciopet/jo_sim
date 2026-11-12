#!/bin/bash
# Lab PC (12/11): reference vjsye2 vs R5 (path proof + occlusion edges) vs R12P (R5 + inherited path history +
# motion-coherent groups with person veto), x3, all on this machine and this build:
# CODa tuning 0/10/16 (LVE2/LVR5/LVR12P), held-out 11/12 (LHVE2/LHVR5/LHVR12P), Marina di Pisa street.
# Code: sins b190b06 / jo_sim c0e1f9e (pulled below). Resumable like lab_local.sh (the street call has no CODa windows, its CODA only names an installed profile).
cd $(dirname $0)/../../..
git pull -q && git submodule update -q sins || { echo "PULL FAILED"; exit 1; }
git -C sins log --oneline -1
CODA="LVE2 coda_vjsye2;LVR5 coda_vjsyer5;LVR12P coda_vjsyer12p" STREET=none bash eval/scripts/analysis/lab_local.sh
CODA="LHVE2 coda_vjsye2;LHVR5 coda_vjsyer5;LHVR12P coda_vjsyer12p" WINDOWS="11_w1 11_w2 11_w3 12_w1 12_w2 12_w3" STREET=none bash eval/scripts/analysis/lab_local.sh
CODA="LVR12P coda_vjsyer12p" STREET="mdp_street_vjsye2 mdp_street_vjsyer5 mdp_street_vjsyer12p" WINDOWS=" " bash eval/scripts/analysis/lab_local.sh
echo "$(date +%T) LAB R12P DONE"
