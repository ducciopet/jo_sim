#!/bin/bash
# Phase-2 protocol, step A: generate the odometry ONCE per sequence (GLIM + EKF,
# jo_navigation/localization_detector.launch.py) and record it, so that every
# method later replays the identical /odometry/filtered + odom->base_link TF.
# usage: make_odom.sh <sensor_bag> <out_bag>
BAG=$1; OUT=$2
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash
DIR=$(dirname $0); rm -rf $OUT; LOG=${OUT}_logs; rm -rf $LOG; mkdir -p $LOG
DUR=$(python3 -c "import yaml;print(int(yaml.safe_load(open('$BAG/metadata.yaml'))['rosbag2_bagfile_information']['duration']['nanoseconds']/1e9)+3)")
ros2 launch jo_navigation localization_detector.launch.py rviz:=false \
  glim_config:=$(ros2 pkg prefix jo_sim)/share/jo_sim/config/glim/glim_config_bunker > $LOG/loc.log 2>&1 &
sleep 15
ros2 bag record --use-sim-time -o $OUT -s mcap /odometry/filtered /tf > $LOG/rec.log 2>&1 &
sleep 2
timeout -k 5 $DUR ros2 bag play $BAG --clock > $LOG/play.log 2>&1
sleep 3
bash $DIR/cleanup.sh > /dev/null
