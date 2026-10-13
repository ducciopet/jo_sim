#!/bin/bash
# v1 full pipeline on every Qualisys sequence (prefix ${OUTP:-v1i}), same environment config as v2 (indoor)
D=$(dirname $0)
for seq in ${SEQS:-20260519_1942 20260519_1945 20260519_1947 20260519_1949 20260519_1950 20260519_1952}; do
  V1_ARGS="env:=${V1_ENV:-indoor} depth_transport:=raw color_transport:=raw calibration_camera_info_topic:=/front_camera/camera/depth/camera_info" \
    bash $D/run_method.sh v1 /home/ros/bags/bags_benchmark/duccio_test_${seq}_0 $D/odom_$seq $D/${OUTP:-v1i}_$seq
  echo "v1 $seq"
done
