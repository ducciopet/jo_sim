#!/bin/bash
# Queue 19/11: wait for p6street_chain.sh, then the P6 FP diagnosis on the existing CODa runs (seq 0 and 10, X0 vs XL),
# then B1 (Dynablox + M-detector on the street).
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
while pgrep -f p6street_chain.sh > /dev/null; do sleep 60; done
echo "$(date +%T) p6street done"
python3 eval/diagnostics/p6_fp/p6_fp.py X0 XL w1 w2 w3 > eval/diagnostics/p6_fp/seq0.txt 2>&1; echo "$(date +%T) fp seq0 rc $?"
python3 eval/diagnostics/p6_fp/p6_fp.py X0 XL 10_w1 10_w2 10_w3 10_w4 10_w5 > eval/diagnostics/p6_fp/seq10.txt 2>&1; echo "$(date +%T) fp seq10 rc $?"
bash eval/baselines/street_baselines.sh
echo "$(date +%T) QUEUE DONE"
