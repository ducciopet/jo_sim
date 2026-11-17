#!/bin/bash
# Input audit (17/11): start the SINS launch exactly as a run does (no bag played) and dump the EFFECTIVE parameters of
# every node (ros2 param dump), then stop it. Run inside the container:
#   bash param_dump.sh <out dir> <launch args...>     e.g. ... /home/ros/bags/audit/coda env:=coda_vjsye2i2pca camera_names:=front_camera
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash
OUT=$1; shift; mkdir -p $OUT
ros2 launch sins run_detector.launch.py rviz:=false "$@" > $OUT/launch.log 2>&1 &
sleep ${WAIT:-30}
for n in $(ros2 node list 2>/dev/null); do f=$(echo $n | tr '/' '_'); timeout 20 ros2 param dump $n > $OUT/${f#_}.yaml 2>/dev/null; done
ros2 node list > $OUT/nodes.txt 2>/dev/null
bash /home/ros/bags/replay/cleanup.sh > /dev/null 2>&1
echo "dumped $(ls $OUT/*.yaml | wc -l) nodes to $OUT"
