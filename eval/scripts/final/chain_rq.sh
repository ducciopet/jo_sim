#!/bin/bash
# CHAIN RQ (27/11, post paper-v1, user OK in this session: "Confermo"). Build: sins c6-reacquire-tick 6b3563c
# (= paper-v1 + track_reacquire_* default off + tick_report_every default 50). Same harness as the finals (gate, manifest, requeue).
# 1) KITTI 0018 stall debug: frozen kitti_tracking + tick timing every tick (kitti_tracking_tick), 1 run.
# 2) RQ (coda_vjsye2i2pcar1lccbmarq) vs MF (coda_vjsye2i2pcar1lccbma, final method profile) on the CODa dev windows, x3,
#    arm order alternated per rep. Outputs on NAS sins_final/dev/, scores in eval/results/dev/. Then reinstall paper-v1.
cd /home/punk-opc/Desktop/jo_sim_sins
N=/mnt/nas-imperatore/datasets_SINS/derived/sins_final/dev; F="bash eval/scripts/final/run_final.sh"; export DRY=0
W="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1"
echo "$(date '+%d/%m %T') RQ CHAIN START"
PROF=kitti_tracking_tick TAG=DBGT REPS=1 NO=$N/kitti_tick RES=eval/results/dev/kitti_tick $F kitti 0018
echo "$(date '+%d/%m %T') KITTI TICK DONE" > bags/final_scratch/logs/kitti_tick.done
arm() { case $1 in RQ) P=coda_vjsye2i2pcar1lccbmarq ;; MF) P=coda_vjsye2i2pcar1lccbma ;; esac
  PROF=$P TAG=$1 REPS=$2 NO=$N/rq/$1 RES=eval/results/dev/rq/$1 $F coda $W; }
arm MF 1; arm RQ 1; arm RQ 2; arm MF 2; arm MF 3; arm RQ 3
echo "$(date '+%d/%m %T') RQ CHAIN DONE" > bags/final_scratch/logs/rq.done
# reinstall paper-v1
git -C sins checkout -q paper-v1 && docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && MAKEFLAGS=-j6 colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --parallel-workers 3 --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > bags/final_scratch/logs/build_paper_v1.log 2>&1 \
  && echo "$(date '+%d/%m %T') paper-v1 REINSTALLED ($(git -C sins rev-parse --short HEAD))" || echo "$(date '+%d/%m %T') REINSTALL FAILED"
