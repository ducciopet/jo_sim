#!/bin/bash
# Phase B: reproduce Dynablox on DOALS exactly as the upstream README ("Running and Evaluating an Experiment"):
# run_experiment.launch with the upstream default.yaml, evaluate:=true, the sequence's indices.csv as ground truth,
# real-time playback (player_rate 1), Ouster deskew by lidar_undistortion. Then upstream evaluate_data.py over all
# outputs. Paper target (RA-L 2023, Table I): object IoU 86.0% within 20 m (default.yaml evaluates 0.5-20 m).
# DOALS layout (download_doals_data.sh): <root>/<scene>/sequence_<n>/bag.bag + indices.csv
# usage: run_doals.sh <DOALS root, host path> [scene/sequence_n ...]   log/output: bags/dynablox_doals/
set -u
ROOT=$1; shift
R=$(cd $(dirname $0)/../../../.. && pwd); OUT=$R/bags/dynablox_doals; mkdir -p $OUT
SEQS=${@:-"hauptgebaeude/sequence_1 hauptgebaeude/sequence_2 niederdorf/sequence_1 niederdorf/sequence_2 station/sequence_1 station/sequence_2 shopville/sequence_1 shopville/sequence_2"}
for S in $SEQS; do
  name=$(echo $S | sed 's#/sequence_#_#')
  [ -e $OUT/$name.done ] && continue
  [ -e $ROOT/$S/bag.bag ] || { echo "$(date +%T) MISSING $S"; continue; }
  # NAS replay drops data (as for CODa): copy the sequence locally, run, delete
  L=$R/bags/doals_tmp; rm -rf $L; mkdir -p $L/$S && rsync -a $ROOT/$S/bag.bag $ROOT/$S/indices.csv $L/$S/ || { echo "$(date +%T) COPY FAIL $S"; continue; }
  docker rm -f dynablox_doals > /dev/null 2>&1
  timeout -k 30 ${RUN_TIMEOUT:-1500} docker run --rm --name dynablox_doals -v $L:/doals:ro -v $OUT:/out dynablox-noetic:doals bash -c "
    source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash
    roslaunch dynablox_ros run_experiment.launch bag_file:=/doals/$S/bag.bag evaluate:=true visualize:=false \
      eval_output_path:=/out/$name ground_truth_file:=/doals/$S/indices.csv > /out/${name}.log 2>&1
  "
  RC=$?; docker rm -f dynablox_doals > /dev/null 2>&1
  rm -rf $L
  [ $RC = 0 ] && touch $OUT/$name.done
  echo "$(date +%T) $name rc=$RC"
done
docker run --rm -v $OUT:/out dynablox-noetic:doals bash -c "
  cd /ws/src/dynablox/dynablox_ros/src/evaluation && python3 evaluate_data.py /out" > $OUT/evaluate.txt 2>&1
cat $OUT/evaluate.txt
echo "$(date +%T) DOALS DONE"
