#!/bin/bash
# CODa held-out seqs 10/11/12 -> bags/coda/coda<seq>_w<k> (+ _stereo): the annotated windows with 200 frames (20 s)
# of lead-in, as for seq 0. Only these windows were downloaded (download_coda_heldout.sh). Resumable (<window>.done).
# usage: setsid nohup bash eval/datasets/convert_coda_heldout.sh <seq> > bags/coda/convert<seq>.log 2>&1 < /dev/null &
set -o pipefail
cd $(dirname $0)/../..
R=${CODA:-/mnt/nas-imperatore/datasets_SINS/CODa}
S=$1
CLASSES=(Pedestrian Bike Scooter Skateboard "Service Vehicle" Horse Car Motorcycle "Golf Cart" Truck Wheelchair)
case $S in
  10) W="1750 2149 w1|5615 6014 w2|8120 8449 w3|9000 9399 w4|11300 11599 w5" ;;
  11) W="1520 2219 w1|4950 5249 w2|5400 5799 w3" ;;
  12) W="2430 2929 w1|10200 10599 w2|11280 11749 w3" ;;
  *) echo "usage: $0 10|11|12"; exit 1 ;;
esac
IFS='|' read -ra WS <<< "$W"
for w in "${WS[@]}"; do
  set -- $w; out=bags/coda/coda${S}_$3
  [ -e $out.done ] && continue
  rm -rf $out ${out}_odom ${out}_stereo
  echo "$(date +%T) start $S $3 ($1-$2)"
  python3 eval/datasets/coda_to_bag.py $R $S $1 $2 $out --classes "${CLASSES[@]}" 2>&1 | tail -2 &&
  python3 eval/datasets/coda_stereo_bag.py $R ${out}_gt.json ${out}_stereo 2>&1 | tail -1 && touch $out.done
done
echo "$(date +%T) all done $S"
