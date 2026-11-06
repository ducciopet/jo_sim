#!/bin/bash
# Lab PC: CODa HELD-OUT seqs 11 and 12, evaluation only (no parameter changes). One sequence at a time for the disk:
# convert from the raw CODa on the NAS, copy the windows to the NAS (derived/coda_windows), then frozen profile WNC
# (LHW = coda_wnc) and reference K0 (LHK = coda) x3 with STEREO=1 via lab_local.sh (which drops the local window copies).
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
N=/mnt/nas-imperatore/datasets_SINS/derived/coda_windows
while pgrep -f "lab_local.s[h]" > /dev/null; do sleep 60; done   # wait for the run in progress
for S in 11 12; do
  bash eval/datasets/convert_coda_heldout.sh $S >> bags/coda/convert$S.log 2>&1
  for w in w1 w2 w3; do [ -e bags/coda/coda${S}_$w.done ] || { echo "$(date +%T) conversion of ${S}_$w failed"; exit 1; }
    for d in coda${S}_$w coda${S}_${w}_stereo coda${S}_${w}_odom; do rsync -a bags/coda/$d/ $N/$d/; done
    cp bags/coda/coda${S}_${w}_gt.csv bags/coda/coda${S}_${w}_gt.json $N/
  done
  echo "$(date +%T) seq $S converted and copied to the NAS, $(df -h --output=avail / | tail -1) free"
  WINDOWS="${S}_w1 ${S}_w2 ${S}_w3" CODA="LHK coda;LHW coda_wnc" STREET=none bash eval/scripts/analysis/lab_local.sh
done
echo "$(date +%T) HELDOUT DONE"
