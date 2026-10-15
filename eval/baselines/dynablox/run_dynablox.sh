#!/bin/bash
# Run upstream Dynablox (ROS1 Noetic container, output-only patch) on one benchmark sequence, same protocol as v1/v2:
# shared odometry as TF odom -> base_link (bags/replay/odom_<seq>), URDF base_link -> velodyne, detector started 12 s
# before the bag, playback at 1x. LiDAR only, so the converted bag carries no images.
# usage: run_dynablox.sh <seq e.g. 20260519_1950> <out name e.g. dynabloxa_20260519_1950> [--keep-input]
set -e
SEQ=$1; OUT=$2; KEEP=$3
R=$(cd $(dirname $0)/../../.. && pwd)
B=$R/bags; IN=$B/dynablox_tmp/in_$SEQ.bag
mkdir -p $B/dynablox_tmp $B/dynablox_out
if [ ! -e $IN ]; then
  rosbags-convert --src $B/bags_benchmark/duccio_test_${SEQ}_0 $B/replay/odom_$SEQ --dst $IN --dst-typestore ros1_noetic \
    --include-topic /velodyne_points /odometry/filtered /tf
fi
rm -f $B/dynablox_out/$OUT.bag; mkdir -p $B/dynablox_out/${OUT}_logs
docker run --rm --name dynablox_run -v $B:/bags -v $(cd $(dirname $0) && pwd):/jo dynablox-noetic:upstream bash -c "
  source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash
  roscore > /bags/dynablox_out/${OUT}_logs/roscore.log 2>&1 &
  sleep 4; rosparam set use_sim_time true
  roslaunch /jo/run_dynablox_jo.launch > /bags/dynablox_out/${OUT}_logs/detector.log 2>&1 &
  sleep 12
  rosbag record -O /bags/dynablox_out/$OUT.bag /motion_detector/visualization/clusters /odometry/filtered __name:=rec > /bags/dynablox_out/${OUT}_logs/rec.log 2>&1 &
  sleep 2
  rosbag play --clock -q /bags/dynablox_tmp/in_$SEQ.bag > /bags/dynablox_out/${OUT}_logs/play.log 2>&1
  sleep 3; rosnode kill /rec; sleep 2
"
[ "$KEEP" = "--keep-input" ] || rm -f $IN
echo "done $OUT"
