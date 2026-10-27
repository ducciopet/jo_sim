#!/bin/bash
# Phase-2 protocol, step B: replay sensor bag + the shared odometry bag (make_odom.sh)
# and run ONE method on it, without any localization of its own. Records the
# method's ObstacleArray output and the (shared) odometry used for alignment.
# usage: run_method.sh <v1|v2> <sensor_bag> <odom_bag> <out_bag>   (env EXTRA_TOPICS: more topics to record)
# env:   ADAPTER=1 (derive missing RealSense streams, eval/adapters/realsense_bag_adapter.py), V1_ARGS, V2_ARGS
METHOD=$1; BAG=$2; ODOM=$3; OUT=$4
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
# DDS on loopback only: an interface vanishing mid-run (e.g. a hotspot) stalled playback once (r8c v1 1942)
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash
DIR=$(dirname $0); rm -rf $OUT; LOG=${OUT}_logs; rm -rf $LOG; mkdir -p $LOG
DUR=$(python3 -c "import yaml;print(int(yaml.safe_load(open('$BAG/metadata.yaml'))['rosbag2_bagfile_information']['duration']['nanoseconds']/1e9)+3)")
if [ -n "$ADAPTER" ]; then
  python3 $DIR/realsense_bag_adapter.py --ros-args -p use_sim_time:=true > $LOG/adapter.log 2>&1 &
fi
# start the detector BEFORE the bag and give it time to load (YOLO model, ~4-6 s), so no method loses
# the first seconds of the sequence to start-up
if [ "$METHOD" = v2 ]; then
  ros2 launch sins run_detector.launch.py rviz:=false $V2_ARGS > $LOG/det.log 2>&1 &
else
  ros2 launch jo_detector run_detector.launch.py use_sim_time:=true rviz:=false $V1_ARGS > $LOG/det.log 2>&1 &
fi
sleep ${WARMUP:-12}
ros2 bag record --use-sim-time -o $OUT -s mcap /perception/tracked_dynamic_obstacles /jo_detector/tracked_dynamic_obstacles /odometry/filtered $EXTRA_TOPICS > $LOG/rec.log 2>&1 &
sleep 3
timeout -k 5 $DUR ros2 bag play -i $BAG -i $ODOM --clock > $LOG/play.log 2>&1 &
PLAY=$!
wait $PLAY
sleep 3
bash $DIR/cleanup.sh > /dev/null
