#!/bin/bash
# DIAGNOSTIC (17/11): lab_u with the class anchor off (env lab_u_na), x1 on 1947 and 1950, to verify the mechanism of
# the lab_u false positives. Same launch as lab_unified.sh. -> bags/lab_qualisys/runs/LUNAa_<seq>, eval/results/lab_unified_diag
cd $(dirname $0)/../../..
source eval/scripts/analysis/lab_gate.sh
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
N=/mnt/nas-imperatore/datasets_SINS/derived/lab_qualisys; L=bags/lab_qualisys; C=/home/ros/bags/lab_qualisys
TOPICS="/sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /yolo_seg_track/front_camera/detections /tf /tf_static"
powerprofilesctl set performance
for s in 20260519_1947 20260519_1950; do
  rsync -a $N/bags_benchmark/duccio_test_${s}_0/ $L/duccio_test_${s}_0/
  out=$L/runs/LUNAa_$s; [ -e $out.done ] || { lab_gate 300; echo "$(date +%T) LUNAa $s"
    docker exec -u ros jo_sim_sins bash -c "EXTRA_TOPICS='$TOPICS' ADAPTER=1 V2_ARGS='env:=lab_u_na camera_names:=front_camera raw_depth_transport:=raw' \
      bash /home/ros/bags/replay/run_method.sh v2 $C/duccio_test_${s}_0 $C/odom_$s $C/runs/LUNAa_$s" > /dev/null 2>&1
    [ -f $out/metadata.yaml ] && touch $out.done || echo "FAIL $s"; }
  rm -rf $L/duccio_test_${s}_0
done
powerprofilesctl set balanced
python3 eval/run_own_eval.py --runs $L/runs --mocap $L/qualysis --methods LINDa LUNIa LUNIb LUNIc LUNAa --annotated-classes person --out eval/results/lab_unified_diag > eval/results/lab_unified_diag.log 2>&1 || echo "EVAL FAILED"
echo "$(date +%T) DIAG DONE"
