#!/bin/bash
# Upstream Dynablox (same container and output-only patch as the lab runs) on one CODa window (bags/coda, from
# eval/datasets/convert_coda0.sh): window's odometry as TF odom -> base_link, base_link -> velodyne from the window's
# ground fit (<window>_gt.json), OS1-128 sensor model in motion_detector_coda.yaml, detector started 12 s before the bag.
# usage: run_dynablox_coda.sh <window e.g. coda0_w1> <out name e.g. dynabloxa_coda0_w1>
set -e
W=$1; OUT=$2
R=$(cd $(dirname $0)/../../.. && pwd)
B=$R/bags; IN=$B/dynablox_tmp/in_$W.bag; D=$(cd $(dirname $0) && pwd)
mkdir -p $B/dynablox_tmp $B/dynablox_out
[ -e $IN ] || rosbags-convert --src $B/coda/$W $B/coda/${W}_odom --dst $IN --dst-typestore ros1_noetic \
  --include-topic /velodyne_points /odometry/filtered /tf
TF=$(python3 -c "
import json, numpy as np
from scipy.spatial.transform import Rotation
T = np.array(json.load(open('$B/coda/${W}_gt.json'))['T_base_velodyne'])
print(*T[:3, 3], *Rotation.from_matrix(T[:3, :3]).as_quat())")
sed "s|args=\"0.03 0.029 0.6 0 0 0 base_link velodyne\"|args=\"$TF base_link velodyne\"|; s|/jo/motion_detector_jo.yaml|/jo/motion_detector_coda.yaml|" \
  $D/run_dynablox_jo.launch > $B/dynablox_tmp/launch_$W.launch
rm -f $B/dynablox_out/$OUT.bag; mkdir -p $B/dynablox_out/${OUT}_logs
# leftover container from a hung run (ours, fixed name); watchdog: a dead master hangs play/record forever
docker rm -f dynablox_run > /dev/null 2>&1 || true
set +e
timeout -k 30 ${RUN_TIMEOUT:-1800} docker run --rm --name dynablox_run -v $B:/bags -v $D:/jo dynablox-noetic:upstream bash -c "
  source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash
  roscore > /bags/dynablox_out/${OUT}_logs/roscore.log 2>&1 &
  for i in \$(seq 60); do rosparam list > /dev/null 2>&1 && break; sleep 1; done
  rosparam set use_sim_time true || exit 3
  roslaunch /bags/dynablox_tmp/launch_$W.launch > /bags/dynablox_out/${OUT}_logs/detector.log 2>&1 &
  sleep 12
  rosbag record -O /bags/dynablox_out/$OUT.bag /motion_detector/visualization/clusters /odometry/filtered __name:=rec > /bags/dynablox_out/${OUT}_logs/rec.log 2>&1 &
  sleep 2
  rosbag play --clock -q /bags/dynablox_tmp/in_$W.bag > /bags/dynablox_out/${OUT}_logs/play.log 2>&1
  sleep 3; rosnode kill /rec; sleep 2
"
RC=$?; docker rm -f dynablox_run > /dev/null 2>&1 || true; [ $RC = 0 ] || { echo "run failed rc=$RC"; exit $RC; }
touch $B/dynablox_out/$OUT.done
echo "done $OUT"
