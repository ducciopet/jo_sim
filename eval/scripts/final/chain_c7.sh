#!/bin/bash
# CHAIN C7 (27/11, post paper-v1, user OK in this session: "Si"). Starts after chain RQ ends (it must not be touched).
# Build sins c7-assign-components 220a266 (track_assignment_components, default off = paper-v1).
# (1) KITTI 0018 tick timing with the flag on; (2) identity: CODa w1 and KITTI 0006, flag off vs on;
# (3) the 7 KITTI seqs with the most drops, flag on. Then reinstall paper-v1.
cd /home/punk-opc/Desktop/jo_sim_sins
while pgrep -f chain_rq.sh > /dev/null; do sleep 60; done
echo "$(date '+%d/%m %T') C7 START"
BUILD() { docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && MAKEFLAGS=-j6 colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --parallel-workers 3 --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda'; }
git -C sins checkout -q 220a266 || { echo "CHECKOUT FAILED"; exit 1; }
P=$HOME/.venvs/sins_eval/bin/python
for p in kitti_tracking_tick_ac kitti_tracking_ac coda_vjsye2i2pcar1lccbmaac; do $P eval/configs/sins/make_cfg.py $p || { echo "CFG FAILED $p"; exit 1; }; done
BUILD > bags/final_scratch/logs/build_c7.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
echo "$(date '+%d/%m %T') C7 BUILT $(git -C sins rev-parse --short HEAD)"
N=/mnt/nas-imperatore/datasets_SINS/derived/sins_final/dev/c7; F="bash eval/scripts/final/run_final.sh"; export DRY=0 REPS=1
PROF=kitti_tracking_tick_ac TAG=ACT NO=$N/kitti_tick RES=eval/results/dev/c7/kitti_tick $F kitti 0018
PROF=kitti_tracking TAG=IDOFF NO=$N/id_kitti_off RES=eval/results/dev/c7/id_kitti_off $F kitti 0006
PROF=kitti_tracking_ac TAG=IDON NO=$N/id_kitti_on RES=eval/results/dev/c7/id_kitti_on $F kitti 0006
PROF=coda_vjsye2i2pcar1lccbma TAG=IDOFF NO=$N/id_coda_off RES=eval/results/dev/c7/id_coda_off $F coda w1
PROF=coda_vjsye2i2pcar1lccbmaac TAG=IDON NO=$N/id_coda_on RES=eval/results/dev/c7/id_coda_on $F coda w1
echo "$(date '+%d/%m %T') C7 CHECKS DONE" > bags/final_scratch/logs/c7_checks.done
PROF=kitti_tracking_ac TAG=AC NO=$N/kitti_ac RES=eval/results/dev/c7/kitti_ac $F kitti 0018 0009 0008 0004 0007 0019 0020
echo "$(date '+%d/%m %T') C7 DONE" > bags/final_scratch/logs/c7.done
git -C sins checkout -q paper-v1 && BUILD > bags/final_scratch/logs/build_paper_v1_c7.log 2>&1 \
  && echo "$(date '+%d/%m %T') paper-v1 REINSTALLED ($(git -C sins rev-parse --short HEAD))" || echo "$(date '+%d/%m %T') REINSTALL FAILED"
