#!/bin/bash
# full pipeline, v2 (prefix r3, with tracker inputs recorded) and v1 (prefix v1p), on every Qualisys sequence
D=$(dirname $0)
for b in /home/ros/bags/bags_benchmark/duccio_test_*; do
  seq=$(basename $b | sed 's/duccio_test_//; s/_0$//')
  EXTRA_TOPICS="/jo_detector_v2/tracker/tracks /jo_detector_v2/dbscan_detector/fused/detections3d /jo_detector_v2/dbscan_detector/fused/clusters /yolo_semantic_seg/front_camera/detections /front_camera/camera/color/camera_info /tf /tf_static" \
    ADAPTER=1 V2_ARGS="camera_names:=front_camera raw_depth_transport:=raw" bash $D/run_method.sh v2 $b $D/odom_$seq $D/r3_v2_$seq
  echo "v2 $seq"
  V1_ARGS="depth_transport:=raw color_transport:=raw calibration_camera_info_topic:=/front_camera/camera/depth/camera_info" bash $D/run_method.sh v1 $b $D/odom_$seq $D/v1p_$seq
  echo "v1 $seq"
done
