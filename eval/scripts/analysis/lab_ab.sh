#!/bin/bash
# Lab regression A/B (generic): ARMS="TAG env;TAG env;..." x3 (reps a b c), order rotated per rep, 6 Qualisys seqs, same launch
# as lab_carrier.sh; GT person only -> eval/results/$OUT. usage: ARMS="LUCA2 lab_uca;LUMV lab_ucamv" OUT=lab_s12 bash lab_ab.sh
cd $(dirname $0)/../../..
source eval/scripts/analysis/lab_gate.sh
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
N=/mnt/nas-imperatore/datasets_SINS/derived/lab_qualisys; L=bags/lab_qualisys; C=/home/ros/bags/lab_qualisys
LAT=eval/results/lab_latency.txt; O=eval/results/$OUT; mkdir -p $L/runs $O
SEQS=${SEQS:-20260519_1942 20260519_1945 20260519_1947 20260519_1949 20260519_1950 20260519_1952}
TOPICS="/sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /yolo_seg_track/front_camera/detections /tf /tf_static"
lat(){ echo "$1 $(docker exec jo_sim_sins bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash; python3 /home/ros/bags/replay/lat_tracks.py $2" 2>/dev/null | tail -1)" | tee -a $LAT; }
IFS=";" read -ra A <<< "$ARMS"
for a in "${A[@]}"; do set -- $a; python3 eval/configs/sins/make_cfg.py $2 > /dev/null || exit 1; done
powerprofilesctl set performance
run(){ local out=$L/runs/${1}_$2; [ -e $out.done ] && return; lab_gate 300; echo "$(date +%T) $1 $2 ($3)"
  docker exec -u ros jo_sim_sins bash -c "EXTRA_TOPICS='$TOPICS' ADAPTER=1 V2_ARGS='env:=$3 camera_names:=front_camera raw_depth_transport:=raw' \
    bash /home/ros/bags/replay/run_method.sh v2 $C/duccio_test_${2}_0 $C/odom_$2 $C/runs/${1}_$2" > /dev/null 2>&1
  [ -f $out/metadata.yaml ] || { echo "FAIL $1 $2"; return; }; touch $out.done; lat "$1 $2" $C/runs/${1}_$2; }
for s in $SEQS; do
  rsync -a $N/bags_benchmark/duccio_test_${s}_0/ $L/duccio_test_${s}_0/
  n=${#A[@]}; i=0
  for r in a b c; do for j in $(seq 0 $((n-1))); do set -- ${A[$(( (j+i) % n ))]}; run $1$r $s $2; done; i=$((i+1)); done
  rm -rf $L/duccio_test_${s}_0
done
powerprofilesctl set balanced
M=""; for a in "${A[@]}"; do set -- $a; M="$M ${1}a ${1}b ${1}c"; done
python3 eval/run_own_eval.py --runs $L/runs --mocap $L/qualysis --methods $M --annotated-classes person --out $O > $O/eval.log 2>&1 || echo "EVAL lab FAILED"
echo "$(date +%T) LAB AB DONE"
