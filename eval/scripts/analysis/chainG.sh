#!/bin/bash
# Chain G (24/11, laptop request, repo 7bf8498), sins c5-semantic-birth e82f9ea: N0 = r1lccbma (base; FT rejected in chain F) vs NE = r1lccbmae
# (+ track_extent_assoc_margin 0.0: pieces inside a moving track's predicted box join it, no new track). x3 alternated
# CODa 9 windows + street (N0 street = copy profile lccbmak); lab 6 reps per arm (two lab_ab passes).
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
git -C sins fetch -q origin; git -C sins checkout -q c5-semantic-birth && git -C sins merge -q --ff-only origin/c5-semantic-birth || { echo "CHECKOUT FAILED"; exit 1; }
git -C sins log --oneline -1
[ $(df -B1G --output=avail / | tail -1) -lt 8 ] && { echo "DISK LOW ($(df -h --output=avail / | tail -1)), not starting"; exit 1; }
for p in r1 r1lc r1lcc r1lccbm r1lccbma r1lccbmae; do for s in coda_vjsye2i2pca mdp_street_vjsye2i2pca lab_uca; do
  python3 eval/configs/sins/make_cfg.py $s$p > /dev/null || { echo "make_cfg $s$p FAILED"; exit 1; }; done; done
python3 eval/configs/sins/make_cfg.py mdp_street_vjsye2i2pcar1lccbmak > /dev/null || { echo "make_cfg lccbmak FAILED"; exit 1; }
guard() { while kill -0 $1 2>/dev/null; do
  [ $(df -B1G --output=avail / | tail -1) -lt 3 ] && { echo "$(date +%T) DISK_ABORT"; pkill -P $1; kill $1; exit 1; }
  sleep 20; done; }
P=mdp_street_vjsye2i2pcar1
CODA="N0 coda_vjsye2i2pcar1lccbma;NE coda_vjsye2i2pcar1lccbmae" \
STREET="${P}lccbmak ${P}lccbmae" bash eval/scripts/analysis/lab_local.sh & L=$!
guard $L & G=$!; wait $L; R=$?; kill $G 2>/dev/null; [ $R = 0 ] || { echo "LAB_LOCAL FAILED $R"; exit 1; }
[ $(df -B1G --output=avail / | tail -1) -lt 3 ] && { echo "DISK LOW, stop before lab"; exit 1; }
ARMS="LN0 lab_ucar1lccbma;LNE lab_ucar1lccbmae" OUT=lab_chaing bash eval/scripts/analysis/lab_ab.sh; ARMS="LN1 lab_ucar1lccbma;LNE1 lab_ucar1lccbmae" OUT=lab_chaing2 bash eval/scripts/analysis/lab_ab.sh
T=""; for t in N0 NE; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in lccbmak lccbmae; do for r in 1 2 3; do S="$S ${P}${p}_r$r"; done; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" STREET="$S" FRAMES=testing bash eval/scripts/analysis/rescore_e3ib.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
O=eval/diagnostics/err_decomp_chainG.txt; : > $O
for F in all testing; do for t in N0 NE; do
  echo "===== $t r1-r3, frames $F" >> $O
  python3 eval/diagnostics/err_decomp.py --frames $F --runs "bags/coda/runs/${t}_v2_coda{c}" "bags/coda/runs/${t}r2_v2_coda{c}" \
    "bags/coda/runs/${t}r3_v2_coda{c}" --windows w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 >> $O 2>&1 || echo "FAIL $t $F" >> $O
done; done
true
T3="N0 N0r2 N0r3 NE NEr2 NEr3"; (python3 eval/diagnostics/idsw_class.py --tags $T3; python3 eval/diagnostics/idsw_class.py --frames testing --tags $T3) > eval/diagnostics/idsw_class_chainG.txt 2>&1
python3 eval/diagnostics/coda_takeover.py --tags $T3 > eval/diagnostics/takeover_chainG.txt 2>&1
(python3 eval/diagnostics/coda_person_funnel.py --tags $T3) > eval/diagnostics/person_funnel_chainG.txt 2>&1
python3 eval/diagnostics/coda_vehicle_funnel.py --tags $T3 --windows 10_w1 10_w4 10_w5 > eval/diagnostics/vehicle_funnel_chainG.txt 2>&1
echo "$(date +%T) ALL DONE"
