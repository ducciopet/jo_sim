#!/bin/bash
# Chain D (21/11, laptop names, repo e258750), sins c5-semantic-birth cd01b90, on X = R1 + connected + fallback connected (lcc, no
# depth window on YOLO points anywhere). DC0 = r1lcc (reference, = RCC) / DM0 = r1lccbm (+ bus cap + merge fragments) / DS0 =
# r1lccbms (+ split_masked) / DQ0 = r1lccbmq (+ person-size confirm 3). x3 alternated CODa 9 windows + street; lab LDC0..LDQ0.
# DC0 street/lab also cover the nf2 street/lab for lcc. Disk guard as chain A (abort below 3 GB; refuse to start below 15 GB).
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
git -C sins fetch -q origin; git -C sins checkout -q c5-semantic-birth && git -C sins merge -q --ff-only origin/c5-semantic-birth || { echo "CHECKOUT FAILED"; exit 1; }
git -C sins log --oneline -1
[ $(df -B1G --output=avail / | tail -1) -lt 15 ] && { echo "DISK LOW ($(df -h --output=avail / | tail -1)), not starting"; exit 1; }
for p in r1 r1lc r1lcc r1lccbm r1lccbms r1lccbmq; do for s in coda_vjsye2i2pca mdp_street_vjsye2i2pca lab_uca; do
  python3 eval/configs/sins/make_cfg.py $s$p > /dev/null || { echo "make_cfg $s$p FAILED"; exit 1; }; done; done
guard() { while kill -0 $1 2>/dev/null; do
  [ $(df -B1G --output=avail / | tail -1) -lt 3 ] && { echo "$(date +%T) DISK_ABORT"; pkill -P $1; kill $1; exit 1; }
  sleep 20; done; }
P=mdp_street_vjsye2i2pcar1
CODA="DC0 coda_vjsye2i2pcar1lcc;DM0 coda_vjsye2i2pcar1lccbm;DS0 coda_vjsye2i2pcar1lccbms;DQ0 coda_vjsye2i2pcar1lccbmq" \
STREET="${P}lcc ${P}lccbm ${P}lccbms ${P}lccbmq" bash eval/scripts/analysis/lab_local.sh & L=$!
guard $L & G=$!; wait $L; R=$?; kill $G 2>/dev/null; [ $R = 0 ] || { echo "LAB_LOCAL FAILED $R"; exit 1; }
[ $(df -B1G --output=avail / | tail -1) -lt 3 ] && { echo "DISK LOW, stop before lab"; exit 1; }
ARMS="LDC0 lab_ucar1lcc;LDM0 lab_ucar1lccbm;LDS0 lab_ucar1lccbms;LDQ0 lab_ucar1lccbmq" OUT=lab_chaind bash eval/scripts/analysis/lab_ab.sh
T=""; for t in DC0 DM0 DS0 DQ0; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in lcc lccbm lccbms lccbmq; do for r in 1 2 3; do S="$S ${P}${p}_r$r"; done; done
TAGS="$T" STREET="$S" E=e3 FRAMES=testing bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" STREET="$S" FRAMES=testing bash eval/scripts/analysis/rescore_e3ib.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
O=eval/diagnostics/err_decomp_chainD.txt; : > $O
for F in all testing; do for t in DC0 DM0 DS0 DQ0; do
  echo "===== $t r1-r3, frames $F" >> $O
  python3 eval/diagnostics/err_decomp.py --frames $F --runs "bags/coda/runs/${t}_v2_coda{c}" "bags/coda/runs/${t}r2_v2_coda{c}" \
    "bags/coda/runs/${t}r3_v2_coda{c}" --windows w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 >> $O 2>&1 || echo "FAIL $t $F" >> $O
done; done
for f in bags/coda/runs/DS0*_logs/det.log; do echo "$(basename $(dirname $f)) $(grep -h 'split_masked:' $f | tail -1 | sed 's/.*split_masked:/split_masked:/')"; done > eval/diagnostics/split_masked_counts.txt
echo "$(date +%T) ALL DONE"
