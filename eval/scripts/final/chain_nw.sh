#!/bin/bash
# CHAIN NW (27/11, post paper-v1, user OK in this session: "confermo", rigorous route). Starts after chain C7 ends.
# paper-v1 binary (sins tag paper-v1), config only: NW = final profile + fusion_timeout_sec 0.0 (no camera wait).
# MF and NW alternated in the same chain: CODa dev (0 w1-3, 10 w1-5, 16_w1) x3, then lab 6 seqs x3.
cd /home/punk-opc/Desktop/jo_sim_sins
while pgrep -f chain_c7.sh > /dev/null || pgrep -f chain_rq.sh > /dev/null; do sleep 60; done
echo "$(date '+%d/%m %T') NW START"
[ "$(git -C sins rev-parse HEAD)" = "$(git -C sins rev-parse paper-v1^{commit})" ] || git -C sins checkout -q paper-v1 || { echo "CHECKOUT FAILED"; exit 1; }
P=$HOME/.venvs/sins_eval/bin/python
for p in coda_vjsye2i2pcar1lccbmanw lab_ucar1lccbmanw; do $P eval/configs/sins/make_cfg.py $p || { echo "CFG FAILED $p"; exit 1; }; done
diff -r sins/cfg/coda_vjsye2i2pcar1lccbma sins/cfg/coda_vjsye2i2pcar1lccbmanw | grep '^[<>]' | grep -v GENERATED > bags/final_scratch/logs/nw_cfg_diff.txt
diff -r sins/cfg/lab_ucar1lccbma sins/cfg/lab_ucar1lccbmanw | grep '^[<>]' | grep -v GENERATED >> bags/final_scratch/logs/nw_cfg_diff.txt
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && MAKEFLAGS=-j6 colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --parallel-workers 3 --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > bags/final_scratch/logs/build_nw.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
echo "$(date '+%d/%m %T') NW BUILT $(git -C sins rev-parse --short HEAD) (paper-v1 + NW cfgs)"
N=/mnt/nas-imperatore/datasets_SINS/derived/sins_final/dev/nw; F="bash eval/scripts/final/run_final.sh"; export DRY=0
W="w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1"; L="20260519_1942 20260519_1945 20260519_1947 20260519_1949 20260519_1950 20260519_1952"
coda() { case $1 in MF) P=coda_vjsye2i2pcar1lccbma ;; NW) P=coda_vjsye2i2pcar1lccbmanw ;; esac
  PROF=$P TAG=$1 REPS=$2 NO=$N/coda/$1 RES=eval/results/dev/nw/coda/$1 $F coda $W; }
lab() { case $1 in MF) P=lab_ucar1lccbma ;; NW) P=lab_ucar1lccbmanw ;; esac
  PROF=$P TAG=$1 REPS=$2 NO=$N/lab/$1 RES=eval/results/dev/nw/lab/$1 $F lab $L; }
coda MF 1; coda NW 1; coda NW 2; coda MF 2; coda MF 3; coda NW 3
echo "$(date '+%d/%m %T') NW CODA DONE" > bags/final_scratch/logs/nw_coda.done
lab MF 1; lab NW 1; lab NW 2; lab MF 2; lab MF 3; lab NW 3
echo "$(date '+%d/%m %T') NW DONE" > bags/final_scratch/logs/nw.done
