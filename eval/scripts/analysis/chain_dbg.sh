#!/bin/bash
# Laptop DIAGNOSTIC: WNC + per-track CSV (profiles coda_dbg / mdp_street_dbg), CODa 0/10/16_w1 and the street, 1 rep.
# Output: bags/coda/runs/DBG_v2_coda<w> + DBG_trk_<w>.csv; bags/mdp/runs/mdp_street_dbg_r1 + dbg_tracks_wnc.csv
cd /home/duccio/Desktop/jo_sim_sins
docker exec jo_sim_sins bash -lc 'source /opt/ros/jazzy/setup.bash; cd /home/ros && colcon build --packages-select sins --symlink-install --install-base /home/ros/local_install --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda' > bags/coda/build_dbg.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
echo "$(date +%T) built"
for w in w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1; do
  case $w in *_*) c=$w ;; *) c=0_$w ;; esac
  [ -e bags/coda/runs/DBG_trk_$c.csv ] && continue
  ENV=coda_dbg STEREO=1 bash eval/scripts/coda_ab.sh DBG $w
  mv bags/coda/runs/dbg_tracks.csv bags/coda/runs/DBG_trk_$c.csv
  echo "$(date +%T) $w done"
done
touch bags/mdp/runs/mdp_street_dbg_r2.done bags/mdp/runs/mdp_street_dbg_r3.done
docker exec -u ros jo_sim_sins bash -c 'PROFILES="mdp_street_dbg" bash /home/ros/bags/mdp/run_street.sh' > bags/mdp/run_street_dbg.log 2>&1
echo "$(date +%T) DBG DONE"
