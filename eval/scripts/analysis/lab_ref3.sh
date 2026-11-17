#!/bin/bash
# Lab PC (17/11): reference vjsye2 re-run (LVE2L r1 done by lab_absorb.sh, here r2/r3) in the current session: LVE2 x3
# (16/11) ran at 71 ms median latency, the candidates I1/I2/A2 and LVE2L at ~52 ms, and LVE2L r1 differs from LVE2 by
# -8.6 MOTA all on seq 0 and -13 vehicle recall. The comparison needs the reference under the same conditions.
cd $(dirname $0)/../../..
export ADAPTER_ARGS="-p cloud_stride:=2"
CODA="LVE2L coda_vjsye2" STREET=none REPS="r2 r3" bash eval/scripts/analysis/lab_local.sh
TAGS="LVE2Lr2 LVE2Lr3" bash eval/scripts/analysis/rescore_e2.sh
TAGS="LVE2Lr2 LVE2Lr3" bash eval/scripts/analysis/coda_vehicle_e2.sh
echo "$(date +%T) LAB REF3 DONE"
