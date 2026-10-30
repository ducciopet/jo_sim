#!/bin/bash
# Host-side helper for CODa tuning: regenerate cfg/coda from cfg/outdoor + overrides, run SINS on the windows in the
# container under TAG, evaluate each window. usage: coda_ab.sh <TAG> [windows]   (env V2_EXTRA: launch args)
cd $(dirname $0)/../..
TAG=$1; shift; W=${@:-w1 w2 w3}
V2_EXTRA=${V2_EXTRA-yolo_leaf_method:=mask}   # outdoor: pixel-exact YOLO leaves (cfg/outdoor/detection.yaml); V2_EXTRA= for boxes
python3 eval/configs/sins/make_cfg.py coda > /dev/null
rm -f bags/coda/runs/${TAG}_v2_coda0_*.done
docker exec -u ros jo_sim_sins bash -c "TAG=$TAG V2_EXTRA='$V2_EXTRA' MORE_TOPICS='$MORE_TOPICS' bash /home/ros/bags/replay/run_coda.sh $W" > /dev/null 2>&1
for w in $W; do
  python3 eval/coda_eval.py --run bags/coda/runs/${TAG}_v2_coda0_$w --gt bags/coda/coda0_${w}_gt.csv \
    --odom bags/coda/coda0_${w}_odom --out eval/results/coda/${TAG}_$w.json 2>&1 | tail -1 | cut -c1-250
done
