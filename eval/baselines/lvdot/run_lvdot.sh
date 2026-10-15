#!/bin/bash
# Run upstream LV-DOT (ROS1 Noetic container) on one benchmark sequence, same protocol as v1/v2:
# shared odometry (bags/replay/odom_<seq>), detector started 12 s before the bag, playback at 1x.
# usage: run_lvdot.sh <seq e.g. 20260519_1950> <out name e.g. lvdota_20260519_1950> [--keep-input]
set -e
SEQ=$1; OUT=$2; KEEP=$3
R=$(cd $(dirname $0)/../../.. && pwd)
B=$R/bags; IN=$B/lvdot_tmp/in_$SEQ.bag
mkdir -p $B/lvdot_tmp $B/lvdot_out
if [ ! -e $IN ]; then
  rosbags-convert --src $B/bags_benchmark/duccio_test_${SEQ}_0 $B/replay/odom_$SEQ --dst $IN --dst-typestore ros1_noetic \
    --include-topic /velodyne_points /front_camera/camera/depth/image_rect_raw /front_camera/camera/color/image_raw /odometry/filtered
fi
rm -f $B/lvdot_out/$OUT.bag $B/lvdot_out/$OUT.bag.active; mkdir -p $B/lvdot_out/${OUT}_logs
docker run --rm --gpus all --name lvdot_run -v $B:/bags -v $(cd $(dirname $0) && pwd):/jo lvdot-noetic:upstream bash -c "
  source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash
  roscore > /bags/lvdot_out/${OUT}_logs/roscore.log 2>&1 &
  sleep 4; rosparam set use_sim_time true
  roslaunch /jo/run_detector_jo.launch > /bags/lvdot_out/${OUT}_logs/detector.log 2>&1 &
  sleep 12
  rosbag record -O /bags/lvdot_out/$OUT.bag /onboard_detector/dynamic_bboxes /onboard_detector/tracked_bboxes /odometry/filtered __name:=rec > /bags/lvdot_out/${OUT}_logs/rec.log 2>&1 &
  sleep 2
  rosbag play --clock -q /bags/lvdot_tmp/in_$SEQ.bag > /bags/lvdot_out/${OUT}_logs/play.log 2>&1
  sleep 2; rosnode kill /rec; sleep 2
  cp ~/.ros/log/latest/yolov11_detector_node*.log /bags/lvdot_out/${OUT}_logs/ 2>/dev/null || true
"
[ "$KEEP" = "--keep-input" ] || rm -f $IN
echo "done $OUT"
