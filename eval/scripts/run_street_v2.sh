#!/bin/bash
# SINS on Marina di Pisa "street": profile mdp_street_v2 (V1 + large-object association gates), 3 repetitions.
# usage (container): bash /home/ros/bags/mdp/run_street_v2.sh
D=/home/ros/bags/mdp; R=/home/ros/bags/replay
mkdir -p $D/runs
for rep in 1 2 3; do
  for P in mdp_street_v2; do
    out=$D/runs/${P}_r${rep}
    [ -e $out.done ] && continue
    echo "$(date +%T) start $P rep $rep"
    EXTRA_TOPICS="/sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /yolo_seg_track/front_camera/detections /tf_static" \
    V2_ARGS="env:=$P camera_names:=front_camera" \
      bash $R/run_method.sh v2 $D/mdp_street $D/mdp_street_odom $out && touch $out.done
  done
done
echo "$(date +%T) ALL DONE v2"
