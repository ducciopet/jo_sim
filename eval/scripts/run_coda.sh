#!/bin/bash
# SINS on the CODa windows (bags/coda, from eval/datasets/convert_coda0.sh), inside the container.
# Same protocol as the lab runs (run_method.sh: detector started before the bag, shared odometry, adapter for the
# RealSense-style streams), with the dataset's own profile env:=coda and the factory calibration (no online ICP).
# STEREO=1: second camera 'stereo_camera' = CODa cam0/cam1 (one frame per scan, eval/datasets/coda_stereo_bag.py ->
# coda0_<w>_stereo), depth computed online by stereo_depth_node.py.
# usage (container): TAG=t0 [STEREO=1] bash /home/ros/bags/replay/run_coda.sh [w1 w2 w3 | 10_w1 11_w2 ...]     env V2_EXTRA: more launch args
D=/home/ros/bags/coda; R=/home/ros/bags/replay
for w in ${@:-w1 w2 w3}; do
  case $w in *_*) ;; *) w=0_$w ;; esac   # w1 = seq 0 window w1; 10_w1 = seq 10 window w1
  out=$D/runs/${TAG:-t0}_v2_coda$w
  [ -e $out.done ] && continue
  mkdir -p $D/runs
  CAMS=front_camera; XB=""; XS=""; AC=""
  if [ -n "$STEREO" ]; then
    CAMS=front_camera,stereo_camera; XB=$D/coda${w}_stereo; XS=stereo_camera; AC="[front_camera, stereo_camera]"
  fi
  EXTRA_TOPICS="${MORE_TOPICS} /sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /yolo_semantic_seg/front_camera/detections /yolo_semantic_seg/stereo_camera/detections /yolo_seg_track/front_camera/detections /yolo_seg_track/stereo_camera/detections /tf /tf_static" \
  ADAPTER=1 ADAPTER_CAMS="$AC" STEREO="$XS" EXTRA_BAGS="$XB" \
  V2_ARGS="env:=coda camera_names:=$CAMS raw_depth_transport:=raw enable_calibration:=false $V2_EXTRA" \
    bash $R/run_method.sh v2 $D/coda$w $D/coda${w}_odom $out && touch $out.done
done
