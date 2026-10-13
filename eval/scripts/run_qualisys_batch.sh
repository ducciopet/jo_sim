#!/bin/bash
# Lab sequences with Qualisys GT: shared odometry once per sequence, then every method on it.
D=$(dirname $0)
for b in /home/ros/bags/bags_benchmark/duccio_test_*; do
  seq=$(basename $b | sed 's/duccio_test_//; s/_0$//')
  [ -f $D/odom_$seq/metadata.yaml ] || bash $D/make_odom.sh $b $D/odom_$seq
  echo "odom $seq"
  ADAPTER=1 V2_ARGS="camera_names:=front_camera raw_depth_transport:=raw" bash $D/run_method.sh v2 $b $D/odom_$seq $D/v2_$seq
  echo "v2 $seq"
  V1_ARGS="depth_transport:=raw color_transport:=raw calibration_camera_info_topic:=/front_camera/camera/depth/camera_info" bash $D/run_method.sh v1 $b $D/odom_$seq $D/v1_$seq
  echo "v1 $seq"
done
