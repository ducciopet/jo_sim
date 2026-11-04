#!/bin/bash
# SINS on Marina di Pisa "street" (robot still on the kerb), inside the container: profile mdp_street (A) and
# mdp_street_v1 (B = A + large-mover size-gate bypass V1), alternated, 3 repetitions. Real-time replay, 18.4 min each.
# usage (container): bash /home/ros/bags/mdp/run_street.sh
D=/home/ros/bags/mdp; R=/home/ros/bags/replay
mkdir -p $D/runs
for rep in 1 2 3; do
  for P in mdp_street mdp_street_v1; do
    out=$D/runs/${P}_r${rep}
    [ -e $out.done ] && continue
    echo "$(date +%T) start $P rep $rep"
    EXTRA_TOPICS="/sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /yolo_seg_track/front_camera/detections /tf_static" \
    V2_ARGS="env:=$P camera_names:=front_camera" \
      bash $R/run_method.sh v2 $D/mdp_street $D/mdp_street_odom $out && touch $out.done
  done
done
echo "$(date +%T) ALL DONE"
