#!/bin/bash
# Chain B (20/11, agreed with the laptop), sins c2-shape-assoc. Overlays of R1 (R0 + ray offset 0 + random_seed 1), x3 alternated
# on CODa 9 tuning windows + street: CB0 = R1 (pcar1) / CBW = walls off + size cap bus (pcar1wb) / CBN = splitter off (pcar1n) /
# CBT = roof 4.5 m outdoors (pcar1t); lab x3: LCB0 lab_ucar1 / LCBN lab_ucar1n / LCBB lab_ucar1b (size cap only, indoor walls on).
# Then e3 all + testing + vehicles, then err_decomp (all + testing) on the four CODa arms.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
git -C sins fetch -q origin; git -C sins checkout -q c2-shape-assoc && git -C sins merge -q --ff-only origin/c2-shape-assoc || { echo "CHECKOUT FAILED"; exit 1; }
git -C sins log --oneline -1
for p in lab_ucar1 lab_ucar1n lab_ucar1b; do python3 eval/configs/sins/make_cfg.py $p > /dev/null || exit 1; done   # installed by lab_local's build
CODA="CB0 coda_vjsye2i2pcar1;CBW coda_vjsye2i2pcar1wb;CBN coda_vjsye2i2pcar1n;CBT coda_vjsye2i2pcar1t" \
STREET="mdp_street_vjsye2i2pcar1 mdp_street_vjsye2i2pcar1wb mdp_street_vjsye2i2pcar1n mdp_street_vjsye2i2pcar1t" \
  bash eval/scripts/analysis/lab_local.sh || exit 1
ARMS="LCB0 lab_ucar1;LCBN lab_ucar1n;LCBB lab_ucar1b" OUT=lab_chainb bash eval/scripts/analysis/lab_ab.sh
T=""; for t in CB0 CBW CBN CBT; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in pcar1 pcar1wb pcar1n pcar1t; do for r in 1 2 3; do S="$S mdp_street_vjsye2i2${p}_r$r"; done; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
O=eval/diagnostics/err_decomp_chainb.txt; : > $O
for F in all testing; do for t in CB0 CBW CBN CBT; do
  echo "===== $t r1-r3, frames $F" >> $O
  python3 eval/diagnostics/err_decomp.py --frames $F --runs "bags/coda/runs/${t}_v2_coda{c}" "bags/coda/runs/${t}r2_v2_coda{c}" \
    "bags/coda/runs/${t}r3_v2_coda{c}" --windows w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 >> $O 2>&1 || echo "FAIL $t $F" >> $O
done; done
echo "$(date +%T) ALL DONE"
