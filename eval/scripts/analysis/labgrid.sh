#!/bin/bash
# Freeze check (25/11, paper session): lab ground model planes (LGP0 = lab_ucar1lccbma) vs grid (LGG = lab_ucar1lccbmag, the only
# change ground_model grid, grid parameters = CODa's). 6 reps per arm alternated (two lab_ab passes). Run after chain H.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
for p in lab_ucar1lccbma lab_ucar1lccbmag; do python3 eval/configs/sins/make_cfg.py $p > /dev/null || { echo "make_cfg $p FAILED"; exit 1; }; done
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > /dev/null || { echo BUILD FAILED; exit 1; }
docker exec jo_sim_sins test -f /home/ros/local_install/sins/share/sins/cfg/lab_ucar1lccbmag/preprocessing.yaml || { echo 'cfg lab_ucar1lccbmag NOT INSTALLED'; exit 1; }
ARMS="LGP0 lab_ucar1lccbma;LGG lab_ucar1lccbmag" OUT=lab_ground bash eval/scripts/analysis/lab_ab.sh
ARMS="LGP1 lab_ucar1lccbma;LGG1 lab_ucar1lccbmag" OUT=lab_ground2 bash eval/scripts/analysis/lab_ab.sh
python3 eval/scripts/analysis/agg_lab.py eval/results/lab_ground > eval/diagnostics/lab_ground.txt 2>&1
python3 eval/scripts/analysis/agg_lab.py eval/results/lab_ground2 >> eval/diagnostics/lab_ground.txt 2>&1
echo "$(date +%T) LABGRID DONE"
