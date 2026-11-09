#!/bin/bash
# Lab PC (09/11): SINS LiDAR-only (eval/scripts/run_coda_lidaronly.sh: no camera, no YOLO, by configuration) with the
# frozen profile coda_wnc, x3, on all CODa windows (tuning 0/10/16 + held-out 11/12). Compare with LWNC / LHW
# (same profile, all sensors, same machine). One window copied locally at a time (NAS replay drops scans), no stereo.
# Resumable (result json). Results eval/results/coda/LLO[r2|r3]_<w>.json, latency eval/results/lab_latency.txt.
# log: bags/coda/lab_lidaronly.log
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
N=/mnt/nas-imperatore/datasets_SINS/derived; LAT=eval/results/lab_latency.txt; T=${TAG:-LLO}   # env TAG: run tag (LLF = fixed code 5d0c8ea)
lat(){ echo "$1 $(docker exec jo_sim_sins bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash; python3 /home/ros/bags/replay/lat_tracks.py $2" 2>/dev/null | tail -1)" | tee -a $LAT; }
cp eval/scripts/run_coda_lidaronly.sh bags/replay/
powerprofilesctl set performance
for w in ${WINDOWS:-w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 11_w1 11_w2 11_w3 12_w1 12_w2 12_w3}; do
  case $w in *_*) c=$w ;; *) c=0_$w ;; esac
  todo=0; for r in "" r2 r3; do [ -e eval/results/coda/${T}${r}_$w.json ] || todo=1; done; [ $todo = 0 ] && continue
  [ -L bags/coda/coda$c ] && rm bags/coda/coda$c
  rsync -a $N/coda_windows/coda$c/ bags/coda/coda$c/ && rsync -a $N/coda_windows/coda${c}_odom $N/coda_windows/coda${c}_gt.csv $N/coda_windows/coda${c}_gt.json bags/coda/
  echo "$(date +%T) window $w copied, $(df -h --output=avail / | tail -1) free"
  for r in "" r2 r3; do
    [ -e eval/results/coda/${T}${r}_$w.json ] && continue
    rm -rf bags/coda/runs/${T}${r}_v2_coda$c bags/coda/runs/${T}${r}_v2_coda$c.done
    docker exec -u ros jo_sim_sins bash -c "TAG=$T$r ENV=${ENV:-coda_wnc} bash /home/ros/bags/replay/run_coda_lidaronly.sh $c" > /dev/null 2>&1
    python3 eval/coda_eval.py --run bags/coda/runs/${T}${r}_v2_coda$c --gt bags/coda/coda${c}_gt.csv --odom bags/coda/coda${c}_odom \
      --out eval/results/coda/${T}${r}_$w.json 2>&1 | tail -1 | cut -c1-200 || echo "$(date +%T) SCORE FAIL $T$r $w"
    lat "$T$r $w" /home/ros/bags/coda/runs/${T}${r}_v2_coda$c
  done
  rm -rf bags/coda/coda$c; ln -sfn /mnt/nas/datasets_SINS/derived/coda_windows/coda$c bags/coda/coda$c
done
powerprofilesctl set balanced
echo "$(date +%T) LIDARONLY DONE"
