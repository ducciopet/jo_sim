#!/bin/bash
# Laptop DIAGNOSTIC for moving vehicles labelled PD: coda_vjsye + per-track CSV (coda_vjsyedbg) on the tuning windows
# with annotated vehicles, then the street (mdp_street_vjsyecsv). Output: bags/coda/runs/VDB_v2_coda<w> + VDB_trk_<w>.csv;
# bags/mdp/runs/mdp_street_vjsyecsv_r1 + dbg_tracks_vjsye.csv
cd /home/duccio/Desktop/jo_sim_sins
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > bags/coda/build_vdb.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
echo "$(date +%T) built"
for w in w1 10_w1 10_w3 10_w4 10_w5 16_w1; do
  case $w in *_*) c=$w ;; *) c=0_$w ;; esac
  [ -e bags/coda/runs/VDB_trk_$c.csv ] && continue
  ENV=coda_vjsyedbg STEREO=1 bash eval/scripts/coda_ab.sh VDB $w
  mv bags/coda/runs/dbg_tracks.csv bags/coda/runs/VDB_trk_$c.csv
  echo "$(date +%T) $w done"
done
touch bags/mdp/runs/mdp_street_vjsyecsv_r2.done bags/mdp/runs/mdp_street_vjsyecsv_r3.done
docker exec -u ros jo_sim_sins bash -c 'PROFILES="mdp_street_vjsyecsv" bash /home/ros/bags/mdp/run_street.sh' > bags/mdp/run_street_vdb.log 2>&1
echo "$(date +%T) VDB DONE"
