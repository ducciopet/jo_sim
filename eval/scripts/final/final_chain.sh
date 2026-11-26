#!/bin/bash
# FINAL RUNS (freeze paper-v1, 26/11 evening, user): agreed order, load gate + requeue + scoring inside run_final.sh.
cd /home/punk-opc/Desktop/jo_sim_sins
while kill -0 3228985 2>/dev/null || kill -0 3226708 2>/dev/null; do sleep 60; done   # wait: input conversion + dry run
F="bash eval/scripts/final/run_final.sh"; export DRY=0 TAG=SF
echo "$(date '+%d/%m %T') FINAL START"
REPS="1 2 3" $F coda_new 2_w1 2_w2 2_w3 2_w4 3_w1 3_w2 18_w1 18_w2 18_w3
echo "$(date '+%d/%m %T') CODA_NEW DONE" > bags/final_scratch/logs/coda_new.done
REPS="1 2 3" $F kitti $(seq -f '%04g' 0 20)
echo "$(date '+%d/%m %T') KITTI DONE" > bags/final_scratch/logs/kitti.done
REPS="1 2 3" $F doals hauptgebaeude_1 hauptgebaeude_2 niederdorf_1 niederdorf_2 station_1 station_2 shopville_1 shopville_2
REPS="1 2 3 4 5" $F coda w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1
REPS="1 2 3" $F coda 11_w1 11_w2 11_w3 12_w1 12_w2 12_w3
REPS="1 2 3" $F jrdb forbes huangb packard2 tressider2 clark0 clark1
REPS="1 2 3 4 5 6" $F lab 20260519_1942 20260519_1945 20260519_1947 20260519_1949 20260519_1950 20260519_1952
echo "$(date '+%d/%m %T') FINAL ALL DONE"
