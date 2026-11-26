#!/bin/bash
# CODa held-out seqs 10/11/12 -> bags/coda/coda<seq>_w<k> (+ _stereo): the annotated windows with 200 frames (20 s)
# of lead-in, as for seq 0. Only these windows were downloaded (download_coda_heldout.sh). Resumable (<window>.done).
# usage: [ONLY=w1] setsid nohup bash eval/datasets/convert_coda_heldout.sh <seq> > bags/coda/convert<seq>.log 2>&1 < /dev/null &
set -o pipefail
cd $(dirname $0)/../..
R=${CODA:-/mnt/nas-imperatore/datasets_SINS/CODa}
S=$1
CLASSES=(Pedestrian Bike Scooter Skateboard "Service Vehicle" Horse Car Motorcycle "Golf Cart" "Pickup Truck" "Delivery Truck" "Utility Vehicle" Bus Cart Wheelchair)   # CODa has no plain "Truck" class
case $S in
  10) W="1750 2149 w1|5615 6014 w2|8120 8449 w3|9000 9399 w4|11300 11599 w5" ;;
  11) W="1520 2219 w1|4950 5249 w2|5400 5799 w3" ;;
  12) W="2430 2929 w1|10200 10599 w2|11280 11749 w3" ;;
  # seq 16 (TUNING, 04/11): the annotated stretches with moving vehicles; no ZED archive -> stereo pair only (--no-camera)
  16) W="2900 3799 w1|8450 9249 w2|10320 11559 w3|14700 15849 w4"; NOCAM=--no-camera ;;
  # UNSEEN seqs 2/3/18 (26/11, user OK to convert before the freeze; nothing run or scored until after the tag): one window per
  # annotated span (span limits from the external PC), same 200-frame lead-in; no ZED archive -> stereo pair only, as seq 16
  2)  W="2200 2599 w1|4840 5339 w2|5275 5659 w3|6650 7149 w4"; NOCAM=--no-camera ;;
  3)  W="1600 2099 w1|11750 12149 w2"; NOCAM=--no-camera ;;
  18) W="1450 1749 w1|1830 2129 w2|6300 6699 w3"; NOCAM=--no-camera ;;
  *) echo "usage: $0 10|11|12|16|2|3|18"; exit 1 ;;
esac
IFS='|' read -ra WS <<< "$W"
for w in "${WS[@]}"; do
  set -- $w; out=${OUT:-bags/coda}/coda${S}_$3   # OUT=bags/final_scratch/coda_new: unseen windows streamed to the NAS
  [ -e $out.done ] && continue
  [ -n "$ONLY" ] && [ "$ONLY" != "$3" ] && continue   # ONLY=w2: just that window (tight disk)
  rm -rf $out ${out}_odom ${out}_stereo
  echo "$(date +%T) start $S $3 ($1-$2)"
  python3 eval/datasets/coda_to_bag.py $R $S $1 $2 $out $NOCAM --classes "${CLASSES[@]}" 2>&1 | tail -2 &&
  python3 eval/datasets/coda_stereo_bag.py $R ${out}_gt.json ${out}_stereo 2>&1 | tail -1 && touch $out.done
done
echo "$(date +%T) all done $S"
