#!/bin/bash
# Resumable benchmark campaign: REPS repetitions of the full pipeline (v2 and v1) on every Qualisys sequence,
# then the greedy_kuhn ablation (tracker-only replay on each v2 run's recorded inputs).
# Each finished job leaves <out>.done; a rerun skips finished jobs and redoes an interrupted one from scratch.
# Stop between jobs: touch <dir>/${CAMPAIGN}.stop (or kill this script + cleanup.sh); resume: run it again.
# usage: CAMPAIGN=r8 REPS="a b c" bash run_campaign.sh
#   V1=0 skips the v1 runs (e.g. v1 unchanged: link the previous campaign's runs instead), GREEDY=0 skips the ablation.
D=$(cd $(dirname $0) && pwd)
C=${CAMPAIGN:-r8}; REPS=${REPS:-a b c}; V1=${V1:-1}; GREEDY=${GREEDY:-1}
SEQS=${SEQS:-20260519_1942 20260519_1945 20260519_1947 20260519_1949 20260519_1950 20260519_1952}
STATE=$D/${C}_state.log
V2_TOPICS="/sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /sins/dbscan_detector/fused/clusters /yolo_semantic_seg/front_camera/detections /front_camera/camera/color/camera_info /tf /tf_static"

job() {  # job <name> <out> <command...>
  local name=$1 out=$2; shift 2
  [ -e $D/${C}.stop ] && { echo "$(date +%T) STOP requested, exiting before $name" | tee -a $STATE; exit 0; }
  [ -e $out.done ] && return
  echo "$(date +%T) start $name" | tee -a $STATE
  "$@" && touch $out.done && echo "$(date +%T) done  $name" | tee -a $STATE
}

for r in $REPS; do
  for seq in $SEQS; do
    BAG=/home/ros/bags/bags_benchmark/duccio_test_${seq}_0
    EXTRA_TOPICS="$V2_TOPICS" ADAPTER=1 V2_ARGS="camera_names:=front_camera raw_depth_transport:=raw" \
      job "v2 rep $r $seq" $D/${C}${r}_v2_$seq bash $D/run_method.sh v2 $BAG $D/odom_$seq $D/${C}${r}_v2_$seq
    [ "$V1" = 1 ] && V1_ARGS="env:=indoor depth_transport:=raw color_transport:=raw calibration_camera_info_topic:=/front_camera/camera/depth/camera_info" \
      job "v1 rep $r $seq" $D/${C}${r}_v1_$seq bash $D/run_method.sh v1 $BAG $D/odom_$seq $D/${C}${r}_v1_$seq
  done
done
[ "$GREEDY" = 1 ] && for r in $REPS; do
  for seq in $SEQS; do
    job "greedy_kuhn rep $r $seq" $D/${C}${r}g_v2_$seq bash $D/run_tracker_only.sh $D/${C}${r}_v2_$seq $D/${C}${r}g_v2_$seq 1.0 -- \
      -p track_assignment_method:=greedy_kuhn
  done
done
echo "$(date +%T) campaign $C complete" | tee -a $STATE
