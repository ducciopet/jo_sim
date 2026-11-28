#!/bin/bash
# FINAL RUNS paper-v2 (freeze 28/11, user ok): same order, harness, gate, reps and requeue as final_chain.sh (paper-v1).
# Only the profiles (<v1 profile>_v2), the tag (S2) and the output folders (sins_final_v2, results/final_v2) change.
cd /home/punk-opc/Desktop/jo_sim_sins
N=/mnt/nas-imperatore/datasets_SINS/derived/sins_final_v2
[ "$(git -C sins describe --tags --exact-match 2>/dev/null)" = paper-v2 ] || { echo "sins is not at paper-v2: stop"; exit 1; }
F="bash eval/scripts/final/run_final.sh"; export DRY=0 TAG=S2
L=bags/final_scratch/logs; echo "$(date '+%d/%m %T') FINAL V2 START"
R() { local ds=$1 p=$2; shift 2; PROF=$p NO=$N/$ds RES=eval/results/final_v2/$ds $F $ds "$@"; }
REPS="1 2 3" R coda_new coda_vjsye2i2pcar1lccbma_v2 2_w1 2_w2 2_w3 2_w4 3_w1 3_w2 18_w1 18_w2 18_w3
echo "$(date '+%d/%m %T') CODA_NEW V2 DONE" > $L/v2_coda_new.done
REPS="1 2 3" R kitti kitti_tracking_v2 $(seq -f '%04g' 0 20)
echo "$(date '+%d/%m %T') KITTI V2 DONE" > $L/v2_kitti.done
REPS="1 2 3" R doals doals_v2 hauptgebaeude_1 hauptgebaeude_2 niederdorf_1 niederdorf_2 station_1 station_2 shopville_1 shopville_2
REPS="1 2 3 4 5" R coda coda_vjsye2i2pcar1lccbma_v2 w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
REPS="1 2 3" R coda coda_vjsye2i2pcar1lccbma_v2 11_w1 11_w2 11_w3 12_w1 12_w2 12_w3
REPS="1 2 3" R jrdb jrdb_v2 forbes huangb packard2 tressider2 clark0 clark1
REPS="1 2 3 4 5 6" R lab lab_ucar1lccbma_v2 20260519_1942 20260519_1945 20260519_1947 20260519_1949 20260519_1950 20260519_1952
echo "$(date '+%d/%m %T') FINAL V2 ALL DONE" | tee $L/v2.done
