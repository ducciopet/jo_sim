#!/bin/bash
# 20/11 morning: after leafc_then_b1.sh (LEAFC + B1 street), the error decomposition (eval/diagnostics/err_decomp.py, laptop)
# on the LEAFC CODa runs, all frames and testing frames, r1-r3 pooled, 9 tuning windows. EMMC (call 3) skipped: its CSVs
# are not on the lab PC. Output -> eval/diagnostics/err_decomp_leafc.txt (appended to lab_queue by hand).
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
while pgrep -f leafc_then_b1.sh > /dev/null; do sleep 60; done
O=eval/diagnostics/err_decomp_leafc.txt; : > $O
W="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1"
for F in all testing; do for T in Q0 QC QB QS; do
  echo "===== $T r1-r3, frames $F" >> $O
  python3 eval/diagnostics/err_decomp.py --frames $F --runs "bags/coda/runs/${T}_v2_coda{c}" "bags/coda/runs/${T}r2_v2_coda{c}" \
    "bags/coda/runs/${T}r3_v2_coda{c}" --windows $W >> $O 2>&1 || echo "FAIL $T $F" >> $O
done; done
echo "$(date +%T) DECOMP DONE"
