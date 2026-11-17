#!/bin/bash
# Lab PC (17/11): street reference (mdp_street_vjsye2l = mdp_street_vjsye2, identical parameters) x3 in the current
# session, next to vjsye2a/vjsye2i1/vjsye2i2 (the 13/11 reference ran under different timing, see lab_ref3.sh).
cd $(dirname $0)/../../..
export ADAPTER_ARGS="-p cloud_stride:=2"
CODA="X coda_vjsye2" WINDOWS=" " STREET="mdp_street_vjsye2l" bash eval/scripts/analysis/lab_local.sh
STREET="mdp_street_vjsye2l_r1 mdp_street_vjsye2l_r2 mdp_street_vjsye2l_r3" bash eval/scripts/analysis/rescore_e2.sh
echo "$(date +%T) LAB STREETREF DONE"
