#!/bin/bash
# SINS on Marina di Pisa "street" (robot still on the kerb), inside the container: profile mdp_street (A) and
# mdp_street_v1 (B = A + large-mover size-gate bypass V1), alternated, 3 repetitions. Real-time replay, 18.4 min each.
# usage (container): [PROFILES="p1 p2"] bash /home/ros/bags/mdp/run_street.sh
D=/home/ros/bags/mdp; R=/home/ros/bags/replay
mkdir -p $D/runs
# lab PC reservations: same rule as eval/scripts/analysis/lab_gate.sh, file bags/lab_blackout (epochs)
lab_gate(){ local f=/home/ros/bags/lab_blackout s e; [ -f $f ] || return 0
  while read s e < $f; n=$(date +%s); [ $((n + $1)) -gt $s ] && [ $n -lt $e ]; do sleep 60; done; }
for rep in 1 2 3; do
  for P in ${PROFILES:-mdp_street mdp_street_v1}; do
    out=$D/runs/${P}_r${rep}
    [ -e $out.done ] && continue
    lab_gate 1320; echo "$(date +%T) start $P rep $rep"
    EXTRA_TOPICS="/sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /yolo_seg_track/front_camera/detections /tf_static" \
    V2_ARGS="env:=$P camera_names:=front_camera" \
      bash $R/run_method.sh v2 $D/mdp_street $D/mdp_street_odom $out && touch $out.done
  done
done
echo "$(date +%T) ALL DONE"
