#!/bin/bash
# Chain H (25/11), sins c5-semantic-birth 7cb2245: H0 = r1lccbma (base; NE rejected in chain G) vs HV = r1lccbmav
# (+ fragment_merge_vehicle_anchor: unclassified pieces join a YOLO car/truck/bus piece, gap <= 0.8 m, union <= 2.8 m wide).
# x3 alternated CODa 9 windows + street (H0 street = copy profile lccbmah); lab 6 reps per arm (two lab_ab passes).
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
git -C sins fetch -q origin; git -C sins checkout -q c5-semantic-birth && git -C sins merge -q --ff-only origin/c5-semantic-birth || { echo "CHECKOUT FAILED"; exit 1; }
git -C sins log --oneline -1
[ $(df -B1G --output=avail / | tail -1) -lt 8 ] && { echo "DISK LOW ($(df -h --output=avail / | tail -1)), not starting"; exit 1; }
for p in r1 r1lc r1lcc r1lccbm r1lccbma r1lccbmav; do for s in coda_vjsye2i2pca mdp_street_vjsye2i2pca lab_uca; do
  python3 eval/configs/sins/make_cfg.py $s$p > /dev/null || { echo "make_cfg $s$p FAILED"; exit 1; }; done; done
python3 eval/configs/sins/make_cfg.py mdp_street_vjsye2i2pcar1lccbmah > /dev/null || { echo "make_cfg lccbmah FAILED"; exit 1; }
guard() { while kill -0 $1 2>/dev/null; do
  [ $(df -B1G --output=avail / | tail -1) -lt 3 ] && { echo "$(date +%T) DISK_ABORT"; pkill -P $1; kill $1; exit 1; }
  sleep 20; done; }
P=mdp_street_vjsye2i2pcar1
CODA="H0 coda_vjsye2i2pcar1lccbma;HV coda_vjsye2i2pcar1lccbmav" \
STREET="${P}lccbmah ${P}lccbmav" bash eval/scripts/analysis/lab_local.sh & L=$!
guard $L & G=$!; wait $L; R=$?; kill $G 2>/dev/null; [ $R = 0 ] || { echo "LAB_LOCAL FAILED $R"; exit 1; }
[ $(df -B1G --output=avail / | tail -1) -lt 3 ] && { echo "DISK LOW, stop before lab"; exit 1; }
ARMS="LJ0 lab_ucar1lccbma;LJV lab_ucar1lccbmav" OUT=lab_chainh bash eval/scripts/analysis/lab_ab.sh; ARMS="LJ1 lab_ucar1lccbma;LJV1 lab_ucar1lccbmav" OUT=lab_chainh2 bash eval/scripts/analysis/lab_ab.sh
T=""; for t in H0 HV; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in lccbmah lccbmav; do for r in 1 2 3; do S="$S ${P}${p}_r$r"; done; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" STREET="$S" FRAMES=testing bash eval/scripts/analysis/rescore_e3ib.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
O=eval/diagnostics/err_decomp_chainH.txt; : > $O
for F in all testing; do for t in H0 HV; do
  echo "===== $t r1-r3, frames $F" >> $O
  python3 eval/diagnostics/err_decomp.py --frames $F --runs "bags/coda/runs/${t}_v2_coda{c}" "bags/coda/runs/${t}r2_v2_coda{c}" \
    "bags/coda/runs/${t}r3_v2_coda{c}" --windows w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 >> $O 2>&1 || echo "FAIL $t $F" >> $O
done; done
true
T3="H0 H0r2 H0r3 HV HVr2 HVr3"; (python3 eval/diagnostics/idsw_class.py --tags $T3; python3 eval/diagnostics/idsw_class.py --frames testing --tags $T3) > eval/diagnostics/idsw_class_chainH.txt 2>&1
python3 eval/diagnostics/coda_takeover.py --tags $T3 > eval/diagnostics/takeover_chainH.txt 2>&1
(python3 eval/diagnostics/coda_person_funnel.py --tags $T3) > eval/diagnostics/person_funnel_chainH.txt 2>&1
python3 eval/diagnostics/coda_vehicle_funnel.py --tags $T3 --windows 10_w1 10_w4 10_w5 > eval/diagnostics/vehicle_funnel_chainH.txt 2>&1
for t in H0 HV; do python3 eval/diagnostics/veh_leaf_cover.py $t 10_w1 10_w4 10_w5; done > eval/diagnostics/veh_leaf_cover_chainH.txt 2>&1
echo "$(date +%T) ALL DONE"
