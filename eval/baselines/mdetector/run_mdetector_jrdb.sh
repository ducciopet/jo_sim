#!/bin/bash
# Run upstream M-detector (ROS1 Noetic container) on one benchmark sequence, same protocol as v1/v2: shared odometry
# (expressed for the LiDAR frame by make_input.py), detector started 12 s before the bag, playback at 1x.
# JRDB variant: same input adapter (make_input_coda.py reads the same layout), sensor model jrdb.yaml.
# env RATE: playback rate (M-detector drops OS1-128 scans at 1x and then pairs clouds with the wrong odometry)
# usage: run_mdetector_jrdb.sh <seq e.g. clark0> <out name e.g. mdetectora_jrdb_clark0>
set -e
SEQ=$1; OUT=$2; KEEP=$3
D=$(cd $(dirname $0) && pwd); R=$(cd $D/../../.. && pwd); B=$R/bags; IN=$B/mdetector_tmp/in_$SEQ.bag
mkdir -p $B/mdetector_tmp $B/mdetector_out
[ -e $IN ] || python3 $D/make_input_coda.py $B/jrdb/$SEQ $IN
rm -f $B/mdetector_out/$OUT.bag; mkdir -p $B/mdetector_out/${OUT}_logs
docker run --rm --name mdetector_run -v $B:/bags -v $D:/jo mdetector-noetic:upstream bash -c "
  source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash
  roscore > /bags/mdetector_out/${OUT}_logs/roscore.log 2>&1 &
  sleep 4; rosparam set use_sim_time true
  roslaunch /jo/run_mdetector_jrdb.launch > /bags/mdetector_out/${OUT}_logs/detector.log 2>&1 &
  sleep 12
  rosbag record -O /bags/mdetector_out/$OUT.bag /m_detector/point_out /m_detector/frame_out /odom_velodyne __name:=rec > /bags/mdetector_out/${OUT}_logs/rec.log 2>&1 &
  sleep 2
  rosbag play --clock -q -r ${RATE:-1} /bags/mdetector_tmp/in_$SEQ.bag > /bags/mdetector_out/${OUT}_logs/play.log 2>&1
  sleep 3; rosnode kill /rec; sleep 2
"
[ "$KEEP" = "--keep-input" ] || rm -f $IN
touch $B/mdetector_out/$OUT.done
echo "done $OUT"
