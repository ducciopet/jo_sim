#!/bin/bash
# KITTI tracking -> bags/kitti/<seq>{,_odom,_gt.csv,_gt.json,_cam.json} with kitti_to_bag.py (one sequence at a time).
# usage: convert_kitti.sh <KITTI_tracking root on the NAS> [split=training|testing] [seq ...]
#   smoke test (unlabelled): convert_kitti.sh $NAS/KITTI_tracking testing 0000   -> bags/kitti/test_0000
ROOT=$1; SPLIT=${2:-training}; shift 2
cd $(dirname $0)/../../..
PY=${PY:-eval/datasets/.venv/bin/python}; [ -x $PY ] || PY=$HOME/.venvs/sins_eval/bin/python;  # lab PC fallback
 mkdir -p bags/kitti
for s in ${@:-$(seq -f '%04g' 0 20)}; do
  name=$s; [ $SPLIT = testing ] && name=test_$s
  [ -e bags/kitti/$name ] && continue
  echo "$(date +%T) convert $SPLIT/$s"
  $PY eval/datasets/kitti/kitti_to_bag.py $ROOT $s bags/kitti/$name --split $SPLIT || echo "$(date +%T) FAILED $s"
done
