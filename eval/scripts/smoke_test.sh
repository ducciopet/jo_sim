#!/bin/bash
# Phase-0 smoke test: run localization + detector (v1 or v2) on a lab bag, headless, and record outputs.
DET=${1:-v2}
BAG=${2:-/home/ros/bags/20260910_124939_validation_lab}
DUR=${3:-80}
OUT=/home/ros/bags/smoke/$DET
export ROS_DOMAIN_ID=77
source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash
rm -rf $OUT; mkdir -p $(dirname $OUT)
LOG=/home/ros/bags/smoke/${DET}_logs; rm -rf $LOG; mkdir -p $LOG
ros2 launch jo_navigation localization_detector.launch.py rviz:=false \
  glim_config:=$(ros2 pkg prefix jo_sim)/share/jo_sim/config/glim/glim_config_bunker > $LOG/loc.log 2>&1 &
sleep 15
ros2 bag record -o $OUT -s mcap /perception/tracked_dynamic_obstacles /tracked_dynamic_obstacles /odometry/filtered > $LOG/rec.log 2>&1 &
sleep 2
timeout $DUR ros2 bag play $BAG --clock > $LOG/play.log 2>&1 &
PLAY=$!
sleep 2
if [ "$DET" = v2 ]; then
  ros2 launch jo_detector_v2 run_detector.launch.py rviz:=false > $LOG/det.log 2>&1 &
else
  DISPLAY= ros2 launch jo_detector run_detector.launch.py use_sim_time:=true > $LOG/det.log 2>&1 &
fi
wait $PLAY
sleep 3
pkill -TERM -f "ros2 bag record"; sleep 3
pkill -TERM -f "ros2 launch"; sleep 5
pkill -9 -f "glim_rosnode|_node|ekf|rgbd_odometry|ros2 launch|yolo|static_transform" ; sleep 1
ros2 bag info $OUT 2>&1 | tail -6
