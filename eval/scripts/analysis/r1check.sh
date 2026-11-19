#!/bin/bash
# R1 check (20/11, agreed with the laptop): sins c2-shape-assoc (leaf-connected + C2 switches + random_seed, all default off),
# 1 run of R1 (coda_vjsye2i2pcar1) on CODa w1, e3 MOTA all, all frames. Pass: inside [min-2, max+2] of Q0 r1-r3 on w1
# (61.6 / 61.6 / 61.0 -> [59.0, 63.6]). Waits for the err_decomp queue.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
while pgrep -f after_leafc_decomp.sh > /dev/null; do sleep 60; done
git -C sins fetch -q origin; git -C sins checkout -q c2-shape-assoc && git -C sins merge -q --ff-only origin/c2-shape-assoc || { echo "CHECKOUT FAILED"; exit 1; }
git -C sins log --oneline -1
CODA="R1C coda_vjsye2i2pcar1" WINDOWS="w1" REPS="_" STREET=none bash eval/scripts/analysis/lab_local.sh || exit 1
TAGS="R1C" WINDOWS="w1" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
python3 -c "
import json; m=json.load(open('eval/results/coda_e3/R1C_w1.json'))['metrics']['merged']['mota']*100
print(f'R1 w1 e3 MOTA all {m:.1f}: ' + ('PASS' if 59.0 <= m <= 63.6 else 'FAIL') + ' (range 59.0-63.6)')"
grep -h "random_seed\|seed" bags/coda/runs/R1C_v2_coda0_w1_logs/det.log | head -3
echo "$(date +%T) R1 CHECK DONE"
