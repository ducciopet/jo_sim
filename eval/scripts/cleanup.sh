#!/bin/bash
# Kill every ROS replay/detector process in the container by PID (never driver scripts or itself).
pids=$(ps -eo pid,cmd | grep -E "ros2 (bag|launch|run)|_node|glim_rosnode|ekf_node|rgbd_odometry|yolo|realsense_bag_adapter|static_transform|rviz2|ros2-daemon|odometry_tf" | grep -v -E "grep|cleanup.sh" | awk '{print $1}')
[ -n "$pids" ] && kill -TERM $pids 2>/dev/null; sleep 3
pids=$(ps -eo pid,cmd | grep -E "ros2 (bag|launch|run)|_node|glim_rosnode|ekf_node|rgbd_odometry|yolo|realsense_bag_adapter|static_transform|rviz2|ros2-daemon|odometry_tf" | grep -v -E "grep|cleanup.sh" | awk '{print $1}')
[ -n "$pids" ] && kill -9 $pids 2>/dev/null
ps -eo cmd | grep -E "ros2|_node|glim|yolo" | grep -v -E "grep|cleanup.sh" | wc -l
