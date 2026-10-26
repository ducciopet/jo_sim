#!/bin/bash
# Map experiment: GLIM alone on one benchmark sequence, with one dynamic-rejection variant, recording its map.
# Records every scan as it enters odometry/mapping (/glim_ros/aligned_points: AFTER the rejection, world frame) and
# GLIM's own pose (/glim_ros/odom), whose union over the sequence is the map content.
# Sensor data (LiDAR, IMU, static TF) from the benchmark bag; the tracked obstacles that drive the rejection from a
# recorded detector run (its /perception/tracked_dynamic_obstacles, as published live), on the same clock.
# usage: run_glim_map.sh <seq e.g. 20260519_1950> <variant> <obstacles_bag|-> <out_bag>
#   variant: none     no rejection (GLIM as used for the shared odometry)
#            d        BBOX rejection, DYNAMIC tracks only
#            dpd      BBOX rejection, DYNAMIC + POTENTIALLY_DYNAMIC (track age > 5, the deployed config)
# OBS_TOPIC=<topic>: obstacle topic in the obstacles bag (v1: /jo_detector/tracked_dynamic_obstacles), remapped
SEQ=$1; VAR=$2; OBS=$3; OUT=$4
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash
BAG=/home/ros/bags/bags_benchmark/duccio_test_${SEQ}_0
CFG=/tmp/glimcfg_${VAR}_$$; rm -rf $CFG
cp -r $(ros2 pkg prefix jo_sim)/share/jo_sim/config/glim/glim_config_bunker $CFG
if [ "$VAR" != none ]; then
  sed -i 's|"enable_global_mapping": true,|"enable_global_mapping": true,\n    "dynamic_rejection_type": "BBOX",\n    "bbox_topic": "/perception/tracked_dynamic_obstacles",|' $CFG/config_ros.json
fi
[ "$VAR" = d ] && sed -i 's|"bbox_potentially_dynamic_min_track_age": [0-9]*|"bbox_potentially_dynamic_min_track_age": 100000000|' $CFG/config_bbox_rejection.json
rm -rf $OUT; LOG=${OUT}_logs; rm -rf $LOG; mkdir -p $LOG; cp -r $CFG $LOG/config
INPUTS="-i $BAG"; TOPICS="/velodyne_points /imu/data /tf_static"
OBS_TOPIC=${OBS_TOPIC:-/perception/tracked_dynamic_obstacles}; REMAP=""
[ "$OBS" != - ] && { INPUTS="$INPUTS -i $OBS"; TOPICS="$TOPICS $OBS_TOPIC"; }
[ "$OBS" != - ] && [ "$OBS_TOPIC" != /perception/tracked_dynamic_obstacles ] && \
  REMAP="--remap $OBS_TOPIC:=/perception/tracked_dynamic_obstacles"
DUR=$(python3 -c "import yaml;print(int(yaml.safe_load(open('$BAG/metadata.yaml'))['rosbag2_bagfile_information']['duration']['nanoseconds']/1e9)+3)")
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$CFG -p use_sim_time:=true > $LOG/glim.log 2>&1 &
sleep 8
ros2 bag record --use-sim-time -o $OUT -s mcap /glim_ros/aligned_points /glim_ros/odom > $LOG/rec.log 2>&1 &
sleep 2
timeout -k 5 $DUR ros2 bag play $INPUTS --clock --topics $TOPICS $REMAP > $LOG/play.log 2>&1
sleep 3
bash $(dirname $0)/cleanup.sh > /dev/null
rm -rf $CFG
