#!/bin/bash
# Chain A (20/11, agreed with the laptop, commits d55489e/7dd42f8), sins c2-shape-assoc, all on R1 + yolo_leaf_connected, cumulative:
# AL0 = r1lc (reference) / AB0 = r1lcb (+ bus size cap 12x6x4, 120 m3) / AN0 = r1lcbn (+ splitter off) / AM0 = r1lcbm (+ refine merge
# fragments) / AK0 = r1lcbk (+ shape association). x3 alternated on CODa 9 tuning windows + street; lab x3 LAL0/LAB0/LAN0/LAM0/LAK0.
# Disk guard: the lab PC disk filled on 20/11 morning (another user's job) and failed runs were marked .done; if free space drops
# below 3 GB the chain is killed and DISK_ABORT is printed (the interrupted run must then be cleaned up with the user's OK).
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
git -C sins fetch -q origin; git -C sins checkout -q c2-shape-assoc && git -C sins merge -q --ff-only origin/c2-shape-assoc || { echo "CHECKOUT FAILED"; exit 1; }
git -C sins log --oneline -1
for p in lab_ucar1lc lab_ucar1lcb lab_ucar1lcbn lab_ucar1lcbm lab_ucar1lcbk; do python3 eval/configs/sins/make_cfg.py $p > /dev/null || exit 1; done
guard() { while kill -0 $1 2>/dev/null; do
  [ $(df -B1G --output=avail / | tail -1) -lt 3 ] && { echo "$(date +%T) DISK_ABORT"; pkill -P $1; kill $1; docker exec jo_sim_sins pkill -f "ros2 bag record" ; exit 1; }
  sleep 20; done; }
P=mdp_street_vjsye2i2pcar1
CODA="AL0 coda_vjsye2i2pcar1lc;AB0 coda_vjsye2i2pcar1lcb;AN0 coda_vjsye2i2pcar1lcbn;AM0 coda_vjsye2i2pcar1lcbm;AK0 coda_vjsye2i2pcar1lcbk" \
STREET="${P}lc ${P}lcb ${P}lcbn ${P}lcbm ${P}lcbk" bash eval/scripts/analysis/lab_local.sh & L=$!
guard $L & G=$!; wait $L; R=$?; kill $G 2>/dev/null; [ $R = 0 ] || { echo "LAB_LOCAL FAILED $R"; exit 1; }
[ $(df -B1G --output=avail / | tail -1) -lt 3 ] && { echo "DISK LOW, stop before lab"; exit 1; }
ARMS="LAL0 lab_ucar1lc;LAB0 lab_ucar1lcb;LAN0 lab_ucar1lcbn;LAM0 lab_ucar1lcbm;LAK0 lab_ucar1lcbk" OUT=lab_chaina bash eval/scripts/analysis/lab_ab.sh
T=""; for t in AL0 AB0 AN0 AM0 AK0; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in lc lcb lcbn lcbm lcbk; do for r in 1 2 3; do S="$S ${P}${p}_r$r"; done; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" STREET="$S" FRAMES=testing bash eval/scripts/analysis/rescore_e3ib.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
O=eval/diagnostics/err_decomp_chainA.txt; : > $O
for F in all testing; do for t in AL0 AB0 AN0 AM0 AK0; do
  echo "===== $t r1-r3, frames $F" >> $O
  python3 eval/diagnostics/err_decomp.py --frames $F --runs "bags/coda/runs/${t}_v2_coda{c}" "bags/coda/runs/${t}r2_v2_coda{c}" \
    "bags/coda/runs/${t}r3_v2_coda{c}" --windows w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 >> $O 2>&1 || echo "FAIL $t $F" >> $O
done; done
echo "$(date +%T) ALL DONE"
