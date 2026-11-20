#!/bin/bash
# R1 + yolo_leaf_connected (20/11, user requirement: no distance window on the YOLO mask points), sins c2-shape-assoc.
# x3 alternated in the same chain with an R1 control under new tags: CODa RB0 = R1 (pcar1) / RC0 = R1 + connected (pcar1lc);
# street pcar1k (R1 copy) / pcar1lc; lab LRB0 lab_ucar1 / LRC0 lab_ucar1lc. Then e3 + e3ib (all + testing), vehicles, err_decomp.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
git -C sins fetch -q origin; git -C sins checkout -q c2-shape-assoc && git -C sins merge -q --ff-only origin/c2-shape-assoc || { echo "CHECKOUT FAILED"; exit 1; }
git -C sins log --oneline -1
for p in lab_ucar1 lab_ucar1lc; do python3 eval/configs/sins/make_cfg.py $p > /dev/null || exit 1; done
CODA="RB0 coda_vjsye2i2pcar1;RC0 coda_vjsye2i2pcar1lc" STREET="mdp_street_vjsye2i2pcar1k mdp_street_vjsye2i2pcar1lc" \
  bash eval/scripts/analysis/lab_local.sh || exit 1
ARMS="LRB0 lab_ucar1;LRC0 lab_ucar1lc" OUT=lab_r1lc bash eval/scripts/analysis/lab_ab.sh
T=""; for t in RB0 RC0; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in pcar1k pcar1lc; do for r in 1 2 3; do S="$S mdp_street_vjsye2i2${p}_r$r"; done; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" STREET="$S" FRAMES=testing bash eval/scripts/analysis/rescore_e3ib.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
O=eval/diagnostics/err_decomp_r1lc.txt; : > $O
for F in all testing; do for t in RB0 RC0; do
  echo "===== $t r1-r3, frames $F" >> $O
  python3 eval/diagnostics/err_decomp.py --frames $F --runs "bags/coda/runs/${t}_v2_coda{c}" "bags/coda/runs/${t}r2_v2_coda{c}" \
    "bags/coda/runs/${t}r3_v2_coda{c}" --windows w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 >> $O 2>&1 || echo "FAIL $t $F" >> $O
  echo "===== $t r1-r3, frames $F, seq 0 only" >> $O
  python3 eval/diagnostics/err_decomp.py --frames $F --runs "bags/coda/runs/${t}_v2_coda{c}" "bags/coda/runs/${t}r2_v2_coda{c}" \
    "bags/coda/runs/${t}r3_v2_coda{c}" --windows w1 w2 w3 >> $O 2>&1 || echo "FAIL $t $F seq0" >> $O
done; done
echo "$(date +%T) ALL DONE"
