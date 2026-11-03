#!/bin/bash
# CODa HELD-OUT (sequences 10, 11, 12: the only ones besides 0/1 with the ZED RGB-D archive, i.e. the same sensors as the
# tuning sequence 0). Only the annotated windows (+20 s lead-in) are fetched, member by member over HTTP range requests
# (remote_zip.py), from the sequence and depthonly archives: OS1 scans, stereo cam0/cam1, ZED cam3 color + depth.
# Lists: eval/datasets/coda_select/sel_<seq>.txt (windows in eval/datasets/README / STATUS 03/11).
# usage: setsid nohup bash eval/datasets/download_coda_heldout.sh > ~/coda_heldout_dl.log 2>&1 < /dev/null &
cd $(dirname $0)/../..
D=${1:-/mnt/nas-imperatore/datasets_SINS/CODa}
U=https://web.corral.tacc.utexas.edu/texasrobotics/web_CODa
for s in 10 11 12; do
  for z in sequences depthonly; do
    echo "$(date +%T) $z/$s"
    for t in 1 2 3 4 5 6; do python3 eval/datasets/remote_zip.py $U/$z/$s.zip $D --names eval/datasets/coda_select/sel_$s.txt && break; sleep 120; done
  done
done
echo "$(date +%T) ALL DONE"
