#!/bin/bash
# SINS on the JRDB held-out sequences (bags/jrdb, from eval/datasets/jrdb/convert_jrdb.sh), inside the container.
# Same protocol as run_coda.sh: detector before the bag, shared odometry (KISS-ICP, from the converter), adapter for the
# RealSense-style streams, depth of the five color cameras projected online from the scan (lidar_depth_node.py).
# Profile per sequence: jrdb_indoor / jrdb_outdoor (eval/configs/sins/jrdb_*/overrides.yaml).
# usage (container): TAG=t0 bash /home/ros/bags/replay/run_jrdb.sh [forbes huangb ...]     env V2_EXTRA: more launch args
D=/home/ros/bags/jrdb; R=/home/ros/bags/replay
CAMS=cam0,cam2,cam4,cam6,cam8; AC="[cam0, cam2, cam4, cam6, cam8]"
for s in ${@:-forbes huangb packard2 tressider2 clark0 clark1}; do
  case $s in clark*|tressider1|memorial|hewlett) ENV=jrdb_outdoor ;; *) ENV=jrdb_indoor ;; esac
  ENV=${JRDB_ENV:-$ENV}   # JRDB_ENV=jrdb_outdoor: one profile for every sequence
  out=$D/runs/${TAG:-t0}_v2_jrdb_$s
  [ -e $out.done ] && continue
  mkdir -p $D/runs
  EXTRA_TOPICS="${MORE_TOPICS} /sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /tf /tf_static" \
  ADAPTER=1 ADAPTER_CAMS="$AC" LIDAR_DEPTH="$AC" \
  V2_ARGS="env:=$ENV camera_names:=$CAMS raw_depth_transport:=raw enable_calibration:=false $V2_EXTRA" \
    bash $R/run_method.sh v2 $D/$s $D/${s}_odom $out && touch $out.done
done
