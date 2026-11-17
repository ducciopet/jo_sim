#!/bin/bash
# Lab PC (17/11): unified profile (env lab_u = coda_vjsye2i2p behaviour + indoor scale/sensor values + ground planes,
# eval/configs/sins/lab_u) x3 vs the current indoor profile (env indoor) x3, ALTERNATED in the same chain, on the 6 Qualisys
# sequences of 19/05. Same launch as the lab campaigns (run_campaign.sh: ADAPTER=1, camera_names front_camera, raw depth,
# online calibration on). Data from the NAS (derived/lab_qualisys), each sensor bag copied locally for its runs only.
# Order per sequence: rep a indoor,lab_u | rep b lab_u,indoor | rep c indoor,lab_u.
# Outputs: bags/lab_qualisys/runs/{LIND,LUNI}{a,b,c}_<seq> ; per-scan latency eval/results/lab_latency.txt;
# YOLO per-frame inference time eval/results/lab_unified/yolo_time.txt; metrics eval/results/lab_unified/{cls,all}
# (cls = --annotated-classes person: unmatched non-person classified predictions ignored; all = as before).
# Resumable (a run with .done is skipped). usage: bash eval/scripts/analysis/lab_unified.sh
cd $(dirname $0)/../../..
source eval/scripts/analysis/lab_gate.sh
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
N=/mnt/nas-imperatore/datasets_SINS/derived/lab_qualisys; L=bags/lab_qualisys; C=/home/ros/bags/lab_qualisys
LAT=eval/results/lab_latency.txt; O=eval/results/lab_unified; mkdir -p $L/runs $O
SEQS=${SEQS:-20260519_1942 20260519_1945 20260519_1947 20260519_1949 20260519_1950 20260519_1952}
TOPICS="/sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /sins/dbscan_detector/fused/clusters /yolo_semantic_seg/front_camera/detections /yolo_seg_track/front_camera/detections /yolo_seg_track/front_camera/inference_time /yolo_semantic_seg/front_camera/inference_time /yolo_semantic_seg/front_camera/detection_time /front_camera/camera/color/camera_info /tf /tf_static"
lat(){ echo "$1 $(docker exec jo_sim_sins bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash; python3 /home/ros/bags/replay/lat_tracks.py $2" 2>/dev/null | tail -1)" | tee -a $LAT
  docker exec jo_sim_sins bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash; python3 /home/ros/bags/replay/lat_topics.py $2" 2>/dev/null | sed "s|^|$1 |" >> ${LAT%.txt}_topics.txt; }
for s in $SEQS; do [ -d $N/bags_benchmark/duccio_test_${s}_0 ] && [ -d $N/replay/odom_$s ] || { echo "MISSING DATA $s on the NAS"; exit 1; }; done
rsync -a $N/qualysis/ $L/qualysis/ && for s in $SEQS; do rsync -a $N/replay/odom_$s/ $L/odom_$s/; done
python3 eval/configs/sins/make_cfg.py lab_u > /dev/null || exit 1
powerprofilesctl set performance
lab_gate 900; docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > /dev/null || { echo BUILD FAILED; exit 1; }
for e in indoor lab_u; do docker exec jo_sim_sins test -f /home/ros/local_install/sins/share/sins/cfg/$e/preprocessing.yaml || { echo "cfg $e NOT INSTALLED"; exit 1; }; done
run(){  # run <TAG> <env> <seq>
  local out=$L/runs/${1}_$2
  [ -e $out.done ] && return
  lab_gate 300; echo "$(date +%T) $1 $2 ($3)"
  docker exec -u ros jo_sim_sins bash -c "EXTRA_TOPICS='$TOPICS' ADAPTER=1 V2_ARGS='env:=$3 camera_names:=front_camera raw_depth_transport:=raw' \
    bash /home/ros/bags/replay/run_method.sh v2 $C/duccio_test_${2}_0 $C/odom_$2 $C/runs/${1}_$2" > /dev/null 2>&1
  [ -f $out/metadata.yaml ] || { echo "FAIL $1 $2"; return; }
  touch $out.done
  lat "$1 $2" $C/runs/${1}_$2
  python3 eval/scripts/analysis/yolo_time.py $out | sed "s|^|$1 $2 |" >> $O/yolo_time.txt
}
for s in $SEQS; do
  rsync -a $N/bags_benchmark/duccio_test_${s}_0/ $L/duccio_test_${s}_0/
  echo "$(date +%T) $s copied, $(df -h --output=avail / | tail -1) free"
  run LINDa $s indoor; run LUNIa $s lab_u
  run LUNIb $s lab_u;  run LINDb $s indoor
  run LINDc $s indoor; run LUNIc $s lab_u
  rm -rf $L/duccio_test_${s}_0
done
powerprofilesctl set balanced
M="LINDa LINDb LINDc LUNIa LUNIb LUNIc"
python3 eval/run_own_eval.py --runs $L/runs --mocap $L/qualysis --methods $M --out $O/all > $O/eval_all.log 2>&1 || echo "EVAL all FAILED"
python3 eval/run_own_eval.py --runs $L/runs --mocap $L/qualysis --methods $M --annotated-classes person --out $O/cls > $O/eval_cls.log 2>&1 || echo "EVAL cls FAILED"
echo "$(date +%T) ALL DONE"
