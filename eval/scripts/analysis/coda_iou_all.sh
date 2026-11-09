#!/bin/bash
# Point-wise IoU (eval/coda_point_iou.py) for every method whose output is on this PC, on every CODa window:
# SINS WNC x3 (LWNC tuning, no cameras on 06/11; LWNCP with cameras / LHW held-out), SINS K0 x3 on the held-out (LHK), SINS LiDAR-only (LLO, where kept),
# Dynablox and M-detector (NAS derived/baselines_out). env TUNE / HELD = SINS tags on tuning / held-out windows, EXTRA (default LLO). Scans read once per window from the NAS copy. Resumable
# (one json per run in eval/results/coda_iou/). Runs at nice 19, one process. log: bags/coda/coda_iou.log
cd $(dirname $0)/../../..
source eval/scripts/analysis/lab_gate.sh   # lab PC reservations (bags/lab_blackout)
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
N=/mnt/nas-imperatore/datasets_SINS/derived; B=$N/baselines_out; O=eval/results/coda_iou
for w in ${WINDOWS:-0_w1 0_w2 0_w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 11_w1 11_w2 11_w3 12_w1 12_w2 12_w3}; do
  case $w in 11_*|12_*) S=${HELD:-"LHW LHK"} ;; *) S=${TUNE:-"LWNC LWNCP"} ;; esac
  runs=""
  for r in "" r2 r3; do
    for t in $S ${EXTRA-LLO}; do
      d=bags/coda/runs/$t${r}_v2_coda$w; [ -e $d/metadata.yaml ] && [ ! -e $O/$t${r}_$w.json ] && runs="$runs $t${r}_$w=sins:$d"
    done
  done
  for m in dynablox mdetector; do
    f=$B/${m}F_coda$w.bag; [ -e $f ] && [ ! -e $O/${m}_$w.json ] && runs="$runs ${m}_$w=$m:$f"
  done
  [ -z "$runs" ] && continue
  lab_gate 900; echo "$(date +%T) $w:$runs"
  nice -n 19 python3 eval/coda_point_iou.py --window $N/coda_windows/coda$w --out-dir $O --runs $runs \
    || echo "$(date +%T) FAIL $w"
done
echo "$(date +%T) IOU DONE"
