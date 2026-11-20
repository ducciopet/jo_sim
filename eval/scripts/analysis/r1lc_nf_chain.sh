#!/bin/bash
# 20/11, queued after r1lc_chain.sh: what nearest_cluster and the mask->box fallback change on R1 + connected, x3 alternated
# with a fresh R1 + connected control (RC1 = pcar1lc): RN0 = nearest_cluster off (pcar1lcn), RF0 = box fallback off (pcar1lcf,
# no depth window anywhere). Street pcar1lc control copy is not re-run here (compare to r1lc_chain's pcar1lc, same build);
# lab LRC1 / LRN0 / LRF0. Then e3, e3ib, vehicles, err_decomp -> eval/diagnostics/err_decomp_r1lc_nf.txt.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
while pgrep -f r1lc_chain.sh > /dev/null; do sleep 60; done
git -C sins log --oneline -1
for p in lab_ucar1lc lab_ucar1lcn lab_ucar1lcf; do python3 eval/configs/sins/make_cfg.py $p > /dev/null || exit 1; done
CODA="RC1 coda_vjsye2i2pcar1lc;RN0 coda_vjsye2i2pcar1lcn;RF0 coda_vjsye2i2pcar1lcf" \
STREET="mdp_street_vjsye2i2pcar1lcn mdp_street_vjsye2i2pcar1lcf" bash eval/scripts/analysis/lab_local.sh || exit 1
ARMS="LRC1 lab_ucar1lc;LRN0 lab_ucar1lcn;LRF0 lab_ucar1lcf" OUT=lab_r1lc_nf bash eval/scripts/analysis/lab_ab.sh
T=""; for t in RC1 RN0 RF0; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in pcar1lcn pcar1lcf; do for r in 1 2 3; do S="$S mdp_street_vjsye2i2${p}_r$r"; done; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" STREET="$S" FRAMES=testing bash eval/scripts/analysis/rescore_e3ib.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
O=eval/diagnostics/err_decomp_r1lc_nf.txt; : > $O
for F in all testing; do for t in RC1 RN0 RF0; do
  echo "===== $t r1-r3, frames $F" >> $O
  python3 eval/diagnostics/err_decomp.py --frames $F --runs "bags/coda/runs/${t}_v2_coda{c}" "bags/coda/runs/${t}r2_v2_coda{c}" \
    "bags/coda/runs/${t}r3_v2_coda{c}" --windows w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 >> $O 2>&1 || echo "FAIL $t $F" >> $O
done; done
echo "$(date +%T) ALL DONE"
