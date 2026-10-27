#!/bin/bash
# Replay a recorded run's tracker INPUTS and run tracker_node alone with parameter overrides.
# Input runs recorded before the package rename (topics /jo_detector_v2/...) are remapped onto /sins/....
# usage: run_tracker_only.sh <input_run_bag> <out_bag> [rate] -- [-p name:=value ...]
IN=$1; OUT=$2; RATE=${3:-1.0}; shift 3; [ "$1" = "--" ] && shift
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
# DDS on loopback only: an interface vanishing mid-run (e.g. a hotspot) stalled playback once (r8c v1 1942)
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash
CFG=$(ros2 pkg prefix sins)/share/sins/cfg/indoor/tracker.yaml
rm -rf $OUT; LOG=${OUT}_logs; rm -rf $LOG; mkdir -p $LOG
DUR=$(python3 -c "import yaml;print(int(yaml.safe_load(open('$IN/metadata.yaml'))['rosbag2_bagfile_information']['duration']['nanoseconds']/1e9/$RATE)+4)")
ros2 run sins tracker_node --ros-args --params-file $CFG -p use_sim_time:=true \
  -p "camera_names:=['front_camera']" "$@" > $LOG/tracker.log 2>&1 &
ros2 bag record --use-sim-time -o $OUT -s mcap /perception/tracked_dynamic_obstacles /sins/tracker/tracks /odometry/filtered > $LOG/rec.log 2>&1 &
sleep 3
timeout -k 5 $DUR ros2 bag play $IN --clock --rate $RATE --topics /sins/dbscan_detector/fused/detections3d /jo_detector_v2/dbscan_detector/fused/detections3d \
  /sins/dbscan_detector/fused/clusters /jo_detector_v2/dbscan_detector/fused/clusters /yolo_semantic_seg/front_camera/detections \
  /front_camera/camera/color/camera_info /tf /tf_static /odometry/filtered \
  --remap /jo_detector_v2/dbscan_detector/fused/detections3d:=/sins/dbscan_detector/fused/detections3d \
  /jo_detector_v2/dbscan_detector/fused/clusters:=/sins/dbscan_detector/fused/clusters > $LOG/play.log 2>&1
sleep 2
bash $(dirname $0)/cleanup.sh > /dev/null
