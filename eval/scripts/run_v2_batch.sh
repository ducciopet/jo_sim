#!/bin/bash
# v2 only, on every Qualisys sequence (shared odometry must already exist: run_qualisys_batch.sh / make_odom.sh)
D=$(dirname $0)
for b in /home/ros/bags/bags_benchmark/duccio_test_*; do
  seq=$(basename $b | sed 's/duccio_test_//; s/_0$//')
  ADAPTER=1 V2_ARGS="camera_names:=front_camera raw_depth_transport:=raw" bash $D/run_method.sh v2 $b $D/odom_$seq $D/v2_$seq
  echo "v2 $seq"
done
