#!/bin/bash
# 20/11, after chainA.sh (replaces r1lc_nf_chain.sh): remove every distance window on YOLO points (user). sins c5-semantic-birth
# (= c2-shape-assoc + 2 default-off commits). x3 alternated, all on R1 + connected: RC1 = r1lc (control) / RN0 = nearest_cluster off
# (lcn) / RF0 = mask->box fallback off (lcf) / RCC = fallback by 3D connectivity (lcc, no window anywhere). Street: lcr (r1lc copy),
# lcn, lcf, lcc; lab LRC1/LRN0/LRF0/LRCC. Then e3, e3ib, vehicles, err_decomp -> eval/diagnostics/err_decomp_r1lc_nf.txt,
# and the yolo_leaf_fallback_connected counter lines from the RCC logs -> eval/diagnostics/fallback_connected_counts.txt.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
while pgrep -f chainA.sh > /dev/null; do sleep 60; done
git -C sins fetch -q origin; git -C sins checkout -q c5-semantic-birth && git -C sins merge -q --ff-only origin/c5-semantic-birth || { echo "CHECKOUT FAILED"; exit 1; }
git -C sins log --oneline -1
[ $(df -B1G --output=avail / | tail -1) -lt 10 ] && { echo "DISK LOW ($(df -h --output=avail / | tail -1)), not starting"; exit 1; }
for p in lab_ucar1lc lab_ucar1lcn lab_ucar1lcf lab_ucar1lcc; do python3 eval/configs/sins/make_cfg.py $p > /dev/null || exit 1; done
guard() { while kill -0 $1 2>/dev/null; do
  [ $(df -B1G --output=avail / | tail -1) -lt 3 ] && { echo "$(date +%T) DISK_ABORT"; pkill -P $1; kill $1; exit 1; }
  sleep 20; done; }
P=mdp_street_vjsye2i2pcar1
CODA="RC1 coda_vjsye2i2pcar1lc;RN0 coda_vjsye2i2pcar1lcn;RF0 coda_vjsye2i2pcar1lcf;RCC coda_vjsye2i2pcar1lcc" \
STREET="${P}lcr ${P}lcn ${P}lcf ${P}lcc" bash eval/scripts/analysis/lab_local.sh & L=$!
guard $L & G=$!; wait $L; R=$?; kill $G 2>/dev/null; [ $R = 0 ] || { echo "LAB_LOCAL FAILED $R"; exit 1; }
ARMS="LRC1 lab_ucar1lc;LRN0 lab_ucar1lcn;LRF0 lab_ucar1lcf;LRCC lab_ucar1lcc" OUT=lab_r1lc_nf bash eval/scripts/analysis/lab_ab.sh
T=""; for t in RC1 RN0 RF0 RCC; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in lcr lcn lcf lcc; do for r in 1 2 3; do S="$S ${P}${p}_r$r"; done; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" STREET="$S" FRAMES=testing bash eval/scripts/analysis/rescore_e3ib.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
O=eval/diagnostics/err_decomp_r1lc_nf.txt; : > $O
for F in all testing; do for t in RC1 RN0 RF0 RCC; do
  echo "===== $t r1-r3, frames $F" >> $O
  python3 eval/diagnostics/err_decomp.py --frames $F --runs "bags/coda/runs/${t}_v2_coda{c}" "bags/coda/runs/${t}r2_v2_coda{c}" \
    "bags/coda/runs/${t}r3_v2_coda{c}" --windows w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 >> $O 2>&1 || echo "FAIL $t $F" >> $O
done; done
grep -H "yolo_leaf_fallback_connected:" bags/coda/runs/RCC*_logs/* bags/mdp/runs/${P}lcc_r*_logs/* > eval/diagnostics/fallback_connected_counts.txt 2>/dev/null
grep -rH "yolo_leaf_fallback_connected:" eval/results/lab_r1lc_nf 2>/dev/null | grep LRCC >> eval/diagnostics/fallback_connected_counts.txt
echo "$(date +%T) ALL DONE"
