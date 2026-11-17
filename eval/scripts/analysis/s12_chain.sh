#!/bin/bash
# S1+S2 of eval/PIPELINE.md in ONE chain (18/11), x3 alternated, all on the lab PC, sins branch yolo-assoc (43768e1):
#   CODa 9 tuning windows + street, 6 arms: R0 = I2P+carrier+A (pcal) / I2P+A without carrier (pa) / R0+M (pcm) /
#   R0+M+P3+P4 (pcam) / +P2 (pcamp) / +P2+P5 (pcamv); scored e3 all + testing + vehicles;
#   then lab regression x3: lab_uca vs lab_ucamv.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
CODA="S0 coda_vjsye2i2pcal;SA coda_vjsye2i2pa;SM coda_vjsye2i2pcm;SMP coda_vjsye2i2pcam;SP2 coda_vjsye2i2pcamp;SV coda_vjsye2i2pcamv" \
STREET="mdp_street_vjsye2i2pcal mdp_street_vjsye2i2pa mdp_street_vjsye2i2pcm mdp_street_vjsye2i2pcam mdp_street_vjsye2i2pcamp mdp_street_vjsye2i2pcamv" \
  bash eval/scripts/analysis/lab_local.sh
T=""; for t in S0 SA SM SMP SP2 SV; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in pcal pa pcm pcam pcamp pcamv; do S="$S mdp_street_vjsye2i2${p}_r1 mdp_street_vjsye2i2${p}_r2 mdp_street_vjsye2i2${p}_r3"; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
echo "$(date +%T) CODA+STREET DONE"
ARMS="LUCA2 lab_uca;LUMV lab_ucamv" OUT=lab_s12 bash eval/scripts/analysis/lab_ab.sh
echo "$(date +%T) ALL DONE"
