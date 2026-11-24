#!/bin/bash
# 24/11 DIAGNOSTIC (laptop request, repo bf89d22): base EB (r1lccbma) + per-track debug CSV, CODa 9 windows x1, tag HDBG.
# The CSV path is fixed (/home/ros/bags/coda/runs/dbg_tracks.csv): renamed to HDBG_trk_<c>.csv after each window.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
git -C sins fetch -q origin; git -C sins checkout -q c5-semantic-birth && git -C sins merge -q --ff-only origin/c5-semantic-birth || { echo "CHECKOUT FAILED"; exit 1; }
git -C sins log --oneline -1
for w in w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1; do
  case $w in *_*) c=$w ;; *) c=0_$w ;; esac
  [ -e bags/coda/runs/HDBG_trk_$c.csv ] && continue
  rm -f bags/coda/runs/dbg_tracks.csv
  WINDOWS=$w REPS=_ CODA="HDBG coda_vjsye2i2pcar1lccbmadbg" STREET=none bash eval/scripts/analysis/lab_local.sh > /dev/null 2>&1
  [ -s bags/coda/runs/dbg_tracks.csv ] && mv bags/coda/runs/dbg_tracks.csv bags/coda/runs/HDBG_trk_$c.csv || echo "NO CSV $w"
  echo "$(date +%T) $w done $(ls -la bags/coda/runs/HDBG_trk_$c.csv 2>/dev/null | awk '{print $5}') bytes"
done
echo "$(date +%T) HDBG DONE"
