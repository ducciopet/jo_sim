#!/bin/bash
# SINS on DOALS (bags/doals, from eval/datasets/doals/convert_doals.sh), inside the container, LiDAR-only by configuration
# (no camera in DOALS): same switches as run_coda_lidaronly.sh. Profile doals (sensor overrides only, FINAL_RUN_SPEC.md).
# Written 24/11 on the laptop; first used by the lab smoke test.
# usage (container): TAG=t0 bash /home/ros/bags/replay/run_doals.sh [hauptgebaeude_1 ...]
D=/home/ros/bags/doals; R=/home/ros/bags/replay
for s in ${@:-hauptgebaeude_1 hauptgebaeude_2 niederdorf_1 niederdorf_2 station_1 station_2 shopville_1 shopville_2}; do
  out=$D/runs/${TAG:-t0}_v2_doals_$s
  [ -e $out.done ] && continue
  [ -e $D/$s ] || { echo "missing $D/$s"; continue; }
  mkdir -p $D/runs
  EXTRA_TOPICS="${MORE_TOPICS} /sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /tf /tf_static" \
  V2_ARGS="env:=${DOALS_ENV:-doals} camera_names:=lidar_only_none enable_yolo11:=false enable_yolo26:=false yolo_leaf_method:=box enable_semantic_segmentation:=false enable_calibration:=false raw_depth_transport:=raw $V2_EXTRA" \
    bash $R/run_method.sh v2 $D/$s $D/${s}_odom $out && touch $out.done
done
