#!/bin/bash
# Upstream LV-DOT (same container as the lab runs) on one CODa window: the window's odometry, sensor I/O from
# make_param_coda.py, detector started 12 s before the bag, playback at 1x.
# usage: run_lvdot_coda.sh <window e.g. coda0_w1> <out name e.g. lvdota_coda0_w1>
set -e
W=$1; OUT=$2
D=$(cd $(dirname $0) && pwd); R=$(cd $D/../../.. && pwd); B=$R/bags; IN=$B/lvdot_tmp/in_$W.bag
CODA=${CODA:-/mnt/nas-imperatore/datasets_SINS/CODa}
mkdir -p $B/lvdot_tmp $B/lvdot_out
[ -e $IN ] || rosbags-convert --src $B/coda/$W $B/coda/${W}_odom --dst $IN --dst-typestore ros1_noetic \
  --include-topic /velodyne_points /front_camera/camera/depth/image_rect_raw /front_camera/camera/color/image_raw /odometry/filtered
python3 $D/make_param_coda.py $B/coda/$W $CODA $B/lvdot_tmp/param_$W.yaml
sed "s|/jo/detector_param_jo.yaml|/bags/lvdot_tmp/param_$W.yaml|" $D/run_detector_jo.launch > $B/lvdot_tmp/launch_$W.launch
rm -f $B/lvdot_out/$OUT.bag $B/lvdot_out/$OUT.bag.active; mkdir -p $B/lvdot_out/${OUT}_logs
docker run --rm --gpus all --name lvdot_run -v $B:/bags -v $D:/jo lvdot-noetic:upstream bash -c "
  source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash
  roscore > /bags/lvdot_out/${OUT}_logs/roscore.log 2>&1 &
  sleep 4; rosparam set use_sim_time true
  roslaunch /bags/lvdot_tmp/launch_$W.launch > /bags/lvdot_out/${OUT}_logs/detector.log 2>&1 &
  sleep 12
  rosbag record -O /bags/lvdot_out/$OUT.bag /onboard_detector/dynamic_bboxes /onboard_detector/tracked_bboxes /odometry/filtered __name:=rec > /bags/lvdot_out/${OUT}_logs/rec.log 2>&1 &
  sleep 2
  rosbag play --clock -q /bags/lvdot_tmp/in_$W.bag > /bags/lvdot_out/${OUT}_logs/play.log 2>&1
  sleep 2; rosnode kill /rec; sleep 2
"
touch $B/lvdot_out/$OUT.done
echo "done $OUT"
