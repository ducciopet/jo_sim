#!/bin/bash
# CODa tuning (seqs 0, 10, 16_w1), x3 interleaved: K0 = profile coda (baseline, current GT), KW1 = coda_w1 (two-point init),
# KW = coda_w (two-point init + size gate after one body length of straight travel).
cd /home/duccio/Desktop/jo_sim_sins
W="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1"
for r in "" r2 r3; do
  for p in "K0 coda" "KW1 coda_w1" "KW coda_w"; do set -- $p
    echo "$(date +%T) $1$r ($2)"; ENV=$2 STEREO=1 bash eval/scripts/coda_ab.sh $1$r $W
  done
done
echo "W DONE"
