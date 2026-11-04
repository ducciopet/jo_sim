#!/bin/bash
# CODa seq 0 -> bags/coda: the three annotated windows (400-799, 4750-4999, 5600-5799), each with 20 s of lead-in
# so the tracker and the wall/floor estimates are settled when scoring starts. Resumable (<window>.done).
# usage: setsid nohup bash eval/datasets/convert_coda0.sh > bags/coda/convert.log 2>&1 < /dev/null &
set -o pipefail
cd $(dirname $0)/../..
R=${CODA:-/mnt/kloster-reichenau/datasets_SINS/CODa}
CLASSES=(Pedestrian Bike Scooter Skateboard "Service Vehicle" Horse Car Motorcycle "Golf Cart" "Pickup Truck" "Delivery Truck" "Utility Vehicle" Bus Cart Wheelchair)   # CODa has no plain "Truck" class
for w in "200 799 w1" "4550 4999 w2" "5400 5799 w3"; do
  set -- $w; out=bags/coda/coda0_$3
  [ -e $out.done ] && continue
  rm -rf $out ${out}_odom
  echo "$(date +%T) start $3"
  python3 eval/datasets/coda_to_bag.py $R 0 $1 $2 $out --classes "${CLASSES[@]}" 2>&1 | tail -1 && touch $out.done
done
echo "$(date +%T) all done"
