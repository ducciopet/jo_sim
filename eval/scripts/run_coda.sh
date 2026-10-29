#!/bin/bash
# SINS on the CODa windows (bags/coda, from eval/datasets/convert_coda0.sh), inside the container.
# Same protocol as the lab runs (run_method.sh: detector started before the bag, shared odometry, adapter for the
# RealSense-style streams), with the dataset's own profile env:=coda and the factory calibration (no online ICP).
# usage (container): TAG=t0 bash /home/ros/bags/replay/run_coda.sh [w1 w2 w3]     env V2_EXTRA: more launch args
D=/home/ros/bags/coda; R=/home/ros/bags/replay
for w in ${@:-w1 w2 w3}; do
  out=$D/runs/${TAG:-t0}_v2_coda0_$w
  [ -e $out.done ] && continue
  mkdir -p $D/runs
  EXTRA_TOPICS="${MORE_TOPICS} /sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /yolo_semantic_seg/front_camera/detections /tf /tf_static" \
  ADAPTER=1 V2_ARGS="env:=coda camera_names:=front_camera raw_depth_transport:=raw enable_calibration:=false $V2_EXTRA" \
    bash $R/run_method.sh v2 $D/coda0_$w $D/coda0_${w}_odom $out && touch $out.done
done
