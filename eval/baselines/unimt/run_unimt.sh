#!/bin/bash
# Run UniMT + SimpleTrack (upstream node, I/O adapted in infer_jo.py) on one benchmark sequence, same protocol as v1/v2:
# shared odometry (bags/replay/odom_<seq>), detector started 12 s before the bag (model loading), playback at 1x.
# usage: run_unimt.sh <seq e.g. 20260519_1950> <out name e.g. unimta_20260519_1950> [--keep-input]
set -e
SEQ=$1; OUT=$2; KEEP=$3
D=$(cd $(dirname $0) && pwd); R=$(cd $D/../../.. && pwd)
B=$R/bags; IN=$B/unimt_tmp/in_$SEQ.bag
mkdir -p $B/unimt_tmp $B/unimt_out
if [ ! -e $IN ]; then
  rosbags-convert --src $B/bags_benchmark/duccio_test_${SEQ}_0 $B/replay/odom_$SEQ --dst $IN --dst-typestore ros1_noetic \
    --include-topic /velodyne_points /front_camera/camera/color/image_raw /odometry/filtered
fi
rm -f $B/unimt_out/$OUT.bag $B/unimt_out/$OUT.bag.active; mkdir -p $B/unimt_out/${OUT}_logs
docker run --rm --gpus all --name unimt_run -v $B:/bags -v $D:/jo \
  -v $D/ckpts/wheelchair.pth:/ws/src/detection/ckpts/cmdt_wheelchair.pth:ro unimt-noetic:upstream bash -c "
  source /opt/ros/noetic/setup.bash; export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
  roscore > /bags/unimt_out/${OUT}_logs/roscore.log 2>&1 &
  sleep 4; rosparam set use_sim_time true
  cd /ws && python3.10 -u /jo/infer_jo.py > /bags/unimt_out/${OUT}_logs/detector.log 2>&1 &
  for i in \$(seq 300); do grep -q 'finish init' /bags/unimt_out/${OUT}_logs/detector.log && break; sleep 1; done; sleep 12  # JO: model load, then as v1/v2
  rosbag record -O /bags/unimt_out/$OUT.bag /unimt/tracks /odometry/filtered __name:=rec > /bags/unimt_out/${OUT}_logs/rec.log 2>&1 &
  sleep 2
  rosbag play --clock -q /bags/unimt_tmp/in_$SEQ.bag > /bags/unimt_out/${OUT}_logs/play.log 2>&1
  sleep 2; rosnode kill /rec; sleep 2
"
[ "$KEEP" = "--keep-input" ] || rm -f $IN
echo "done $OUT"
