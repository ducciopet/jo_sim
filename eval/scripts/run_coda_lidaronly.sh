#!/bin/bash
# SINS LiDAR-only on the CODa windows (sensor-modality comparison, eval/baselines/FIDELITY.md section 3), inside the
# container. Same protocol as run_coda.sh, but no camera reaches SINS, by configuration only (no code change):
# - camera_names:=lidar_only_none -> preprocessing/static_structures subscribe to topics that no bag publishes;
# - YOLO (enable_yolo11/26) and semantic segmentation off, yolo_leaf_method box (no mask detector forced on);
# - no RealSense adapter, no stereo depth node, no stereo bag.
# The run records /yolo_* topics anyway, so the check "no camera data" is done on the output bag.
# usage (container): TAG=LLO ENV=coda_wnc bash /home/ros/bags/replay/run_coda_lidaronly.sh [w1 | 10_w1 ...]
D=/home/ros/bags/coda; R=/home/ros/bags/replay
for w in ${@:-w1 w2 w3}; do
  case $w in *_*) ;; *) w=0_$w ;; esac
  out=$D/runs/${TAG:-LLO}_v2_coda$w
  [ -e $out.done ] && continue
  mkdir -p $D/runs
  EXTRA_TOPICS="/sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /yolo_seg_track/front_camera/detections /yolo_semantic_seg/front_camera/detections /tf /tf_static" \
  V2_ARGS="env:=${ENV:-coda_wnc} camera_names:=lidar_only_none enable_yolo11:=false enable_yolo26:=false yolo_leaf_method:=box enable_semantic_segmentation:=false enable_calibration:=false raw_depth_transport:=raw $V2_EXTRA" \
    bash $R/run_method.sh v2 $D/coda$w $D/coda${w}_odom $out && touch $out.done
done
