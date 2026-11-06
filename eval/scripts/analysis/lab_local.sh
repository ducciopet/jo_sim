#!/bin/bash
# Lab PC (06/11): WN / WNC / WNCJM x3 on CODa 0+10+16 and Marina di Pisa street, all on this machine.
# Disk is short and NAS replay drops scans, so each CODa window is copied locally, run 9 times, then removed.
# Resumable: a run whose result json exists is skipped. Local, not committed.
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
N=/mnt/nas-imperatore/datasets_SINS/derived; LAT=eval/results/lab_latency.txt
lat(){ echo "$1 $(docker exec jo_sim_sins bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ros/local_install/setup.bash; python3 /home/ros/bags/replay/lat_tracks.py $2" 2>/dev/null | tail -1)" | tee -a $LAT; }
for w in w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1; do
  case $w in *_*) c=$w ;; *) c=0_$w ;; esac
  for d in coda$c coda${c}_stereo; do [ -L bags/coda/$d ] && rm bags/coda/$d; rsync -a $N/coda_windows/$d/ bags/coda/$d/; done
  echo "$(date +%T) window $w copied, $(df -h --output=avail / | tail -1) free"
  for r in "" r2 r3; do for p in "LWN coda_wn" "LWNC coda_wnc" "LWNCJM coda_wncjm"; do set -- $p
    [ -e eval/results/coda/$1${r}_$w.json ] && continue
    echo "$(date +%T) $1$r $w"; ENV=$2 STEREO=1 bash eval/scripts/coda_ab.sh $1$r $w
    lat "$1$r $w" /home/ros/bags/coda/runs/$1${r}_v2_coda$c
  done; done
  for d in coda$c coda${c}_stereo; do rm -rf bags/coda/$d; ln -sfn /mnt/nas/datasets_SINS/derived/coda_windows/$d bags/coda/$d; done
done
echo "CODA DONE"
[ -L bags/mdp/mdp_street ] && { rm bags/mdp/mdp_street; rsync -a $N/mdp_street_work/mdp_street/ bags/mdp/mdp_street/; }
echo "$(date +%T) street copied, $(df -h --output=avail / | tail -1) free"
P="mdp_street_wn mdp_street_wnc mdp_street_wncjm"
docker exec -u ros jo_sim_sins bash -c "PROFILES='$P' bash /home/ros/bags/mdp/run_street.sh" > bags/mdp/run_street_lab.log 2>&1
mkdir -p eval/results/mdp_lab
for r in r1 r2 r3; do for p in $P; do [ -e bags/mdp/runs/${p}_$r.done ] || { echo "MISSING $p $r"; continue; }
  lat "$p $r" /home/ros/bags/mdp/runs/${p}_$r
  for G in "" _vehicle _person; do o=eval/results/mdp_lab/${p}${G}_$r.json
    [ -e $o ] || python3 eval/coda_eval.py --run bags/mdp/runs/${p}_$r --gt bags/mdp/mdp_street${G}_gt.csv --odom bags/mdp/mdp_street_odom --range 15 --out $o > /dev/null 2>&1 || echo "FAIL $p $G $r"
  done; done; done
rm -rf bags/mdp/mdp_street; ln -sfn /mnt/nas/datasets_SINS/derived/mdp_street_work/mdp_street bags/mdp/mdp_street
powerprofilesctl set balanced
echo "$(date +%T) ALL DONE"
