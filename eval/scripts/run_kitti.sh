#!/bin/bash
# SINS on KITTI tracking (bags/kitti, from eval/datasets/kitti/convert_kitti.sh), inside the container. Same protocol as
# run_jrdb.sh: detector before the bag, shared odometry (OXTS, from the converter), adapter for the RealSense-style
# streams, depth of cam2 projected online from the scan (lidar_depth_node.py). Profile: kitti_tracking (sensor overrides
# only, FINAL_RUN_SPEC.md). Written 24/11 on the laptop, first used by the lab smoke test (testing 0000).
# usage (container): TAG=t0 bash /home/ros/bags/replay/run_kitti.sh [0000 0002 ...]   (names = bag names in bags/kitti)
D=/home/ros/bags/kitti; R=/home/ros/bags/replay
CAMS=cam2; AC="[cam2]"
for s in ${@:-$(seq -f '%04g' 0 20)}; do
  out=$D/runs/${TAG:-t0}_v2_kitti_$s
  [ -e $out.done ] && continue
  [ -e $D/$s ] || { echo "missing $D/$s"; continue; }
  mkdir -p $D/runs
  EXTRA_TOPICS="${MORE_TOPICS} /sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /tf /tf_static" \
  ADAPTER=1 ADAPTER_CAMS="$AC" LIDAR_DEPTH="$AC" \
  V2_ARGS="env:=${KITTI_ENV:-kitti_tracking} camera_names:=$CAMS raw_depth_transport:=raw enable_calibration:=false $V2_EXTRA" \
    bash $R/run_method.sh v2 $D/$s $D/${s}_odom $out && touch $out.done
done
