#!/bin/bash
# Lab PC (17/11): cumulative A/B x3, alternated, all in one chain:
#   lab (6 Qualisys seqs): indoor (LIN2) / lab_uc (LUC: lab_u + class anchor carrier) / lab_uca (LUCA: + point-shift veto
#     unclassified only); order rotated per rep. GT person only (--annotated-classes person) -> eval/results/lab_carrier
#   CODa 9 tuning windows + street via lab_local.sh: I2P (LXI, mdp_street_vjsye2i2pl) / I2P+carrier (LXC) / +A (LXA);
#     scored e3 (+ e3 vehicles); latency as usual.
#   then ONE untimed replay with the tracker lifecycle log (lab_uca, 1947) for the standing-person class clear.
# sins branch floor-carrier (local on the lab). Resumable.
cd $(dirname $0)/../../..
source eval/scripts/analysis/lab_gate.sh
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
N=/mnt/nas-imperatore/datasets_SINS/derived/lab_qualisys; L=bags/lab_qualisys; C=/home/ros/bags/lab_qualisys
LAT=eval/results/lab_latency.txt; O=eval/results/lab_carrier; mkdir -p $L/runs $O
SEQS=${SEQS:-20260519_1942 20260519_1945 20260519_1947 20260519_1949 20260519_1950 20260519_1952}
TOPICS="/sins/tracker/tracks /sins/dbscan_detector/fused/detections3d /yolo_seg_track/front_camera/detections /yolo_seg_track/front_camera/inference_time /yolo_semantic_seg/front_camera/detection_time /tf /tf_static"
lat(){ echo "$1 $(docker exec jo_sim_sins bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash; python3 /home/ros/bags/replay/lat_tracks.py $2" 2>/dev/null | tail -1)" | tee -a $LAT; }
for p in lab_uc lab_uca; do python3 eval/configs/sins/make_cfg.py $p > /dev/null || exit 1; done
powerprofilesctl set performance
lab_gate 900; docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > /dev/null || { echo BUILD FAILED; exit 1; }
for e in indoor lab_uc lab_uca; do docker exec jo_sim_sins test -f /home/ros/local_install/sins/share/sins/cfg/$e/tracker.yaml || { echo "cfg $e NOT INSTALLED"; exit 1; }; done
run(){  # run <TAG> <seq> <env> [extra launch args]
  local out=$L/runs/${1}_$2
  [ -e $out.done ] && return
  lab_gate 300; echo "$(date +%T) $1 $2 ($3)"
  docker exec -u ros jo_sim_sins bash -c "EXTRA_TOPICS='$TOPICS' ADAPTER=1 V2_ARGS='env:=$3 camera_names:=front_camera raw_depth_transport:=raw $4' \
    bash /home/ros/bags/replay/run_method.sh v2 $C/duccio_test_${2}_0 $C/odom_$2 $C/runs/${1}_$2" > /dev/null 2>&1
  [ -f $out/metadata.yaml ] || { echo "FAIL $1 $2"; return; }
  touch $out.done; [ -z "$4" ] && lat "$1 $2" $C/runs/${1}_$2
}
for s in $SEQS; do
  rsync -a $N/bags_benchmark/duccio_test_${s}_0/ $L/duccio_test_${s}_0/
  echo "$(date +%T) $s copied, $(df -h --output=avail / | tail -1) free"
  run LIN2a $s indoor; run LUCa $s lab_uc;  run LUCAa $s lab_uca
  run LUCb $s lab_uc;  run LUCAb $s lab_uca; run LIN2b $s indoor
  run LUCAc $s lab_uca; run LIN2c $s indoor; run LUCc $s lab_uc
  [ $s = 20260519_1947 ] && run LCUCA $s lab_uca "tracker_debug_lifecycle:=true"   # untimed, lifecycle log
  rm -rf $L/duccio_test_${s}_0
done
M="LIN2a LIN2b LIN2c LUCa LUCb LUCc LUCAa LUCAb LUCAc"
python3 eval/run_own_eval.py --runs $L/runs --mocap $L/qualysis --methods $M --annotated-classes person --out $O > $O/eval.log 2>&1 || echo "EVAL lab FAILED"
echo "$(date +%T) LAB DONE"
CODA="LXI coda_vjsye2i2p;LXC coda_vjsye2i2pc;LXA coda_vjsye2i2pca" STREET="mdp_street_vjsye2i2pl mdp_street_vjsye2i2pc mdp_street_vjsye2i2pca" \
  bash eval/scripts/analysis/lab_local.sh
T=""; for t in LXI LXC LXA; do T="$T $t ${t}r2 ${t}r3"; done
S=""; for p in mdp_street_vjsye2i2pl mdp_street_vjsye2i2pc mdp_street_vjsye2i2pca; do S="$S ${p}_r1 ${p}_r2 ${p}_r3"; done
TAGS="$T" STREET="$S" E=e3 bash eval/scripts/analysis/rescore_e2.sh
TAGS="$T" E=e3 bash eval/scripts/analysis/coda_vehicle_e2.sh
echo "$(date +%T) ALL DONE"
