#!/bin/bash
# Run upstream Xu et al. 2023 WITH its YOLO module (RGB-D + color) (ROS1 Noetic container) on one benchmark sequence, same protocol as v1/v2:
# shared odometry (bags/replay/odom_<seq>), detector started 12 s before the bag, playback at 1x.
# usage: run_xu_rgbd_yolo.sh <seq e.g. 20260519_1950> <out name e.g. xuyoloa_20260519_1950> [--keep-input]
set -e
SEQ=$1; OUT=$2; KEEP=$3
R=$(cd $(dirname $0)/../../.. && pwd)
B=$R/bags; IN=$B/xu_rgbd_tmp/in_yolo_$SEQ.bag
mkdir -p $B/xu_rgbd_tmp $B/xu_rgbd_out
[ -e $IN ] || python3 $(dirname $0)/make_input_yolo.py $SEQ $IN
rm -f $B/xu_rgbd_out/$OUT.bag $B/xu_rgbd_out/$OUT.bag.active; mkdir -p $B/xu_rgbd_out/${OUT}_logs
docker run --rm --name xu_rgbd_run -v $B:/bags -v $(cd $(dirname $0) && pwd):/jo xu-rgbd-noetic:upstream bash -c "
  source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash
  roscore > /bags/xu_rgbd_out/${OUT}_logs/roscore.log 2>&1 &
  sleep 4; rosparam set use_sim_time true
  ln -sf /usr/bin/python3 /usr/bin/python  # JO: the YOLO node shebang is "python" (Noetic has only python3)
  roslaunch /jo/run_detector_jo_yolo.launch > /bags/xu_rgbd_out/${OUT}_logs/detector.log 2>&1 &
  sleep 12
  rosbag record -O /bags/xu_rgbd_out/$OUT.bag /onboard_detector/dynamic_bboxes /onboard_detector/tracked_bboxes /yolo_detector/detected_bounding_boxes /odometry/filtered __name:=rec > /bags/xu_rgbd_out/${OUT}_logs/rec.log 2>&1 &
  sleep 2
  rosbag play --clock -q /bags/xu_rgbd_tmp/in_yolo_$SEQ.bag > /bags/xu_rgbd_out/${OUT}_logs/play.log 2>&1
  sleep 2; rosnode kill /rec; sleep 2

"
[ "$KEEP" = "--keep-input" ] || rm -f $IN
echo "done $OUT"
