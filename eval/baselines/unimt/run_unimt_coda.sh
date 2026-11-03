#!/bin/bash
# SimpleTrack (UniMT tracking) on CODa windows with the authors CODa CenterPoint, offline (infer_coda.py).
# Input: bags/unimt_tmp/coda0_<w>.npz from make_coda_manifest.py. Output: bags/unimt_out/unimt_coda0_<w>_{tracks,dets}.csv
# usage: run_unimt_coda.sh w1 w2 w3 | 10_w1 ...   (bare w<k> = seq 0)
D=$(cd $(dirname $0) && pwd); R=$(cd $D/../../.. && pwd); B=$R/bags
mkdir -p $B/unimt_out
for w in "$@"; do
  case $w in *_*) ;; *) w=0_$w ;; esac
  docker run --rm --gpus all --name unimt_coda -v $B:/bags -v $D:/jo -v /mnt/nas-imperatore:/mnt/nas-imperatore:ro \
    unimt-noetic:upstream bash -c "
    cd /ws && python3.10 -u /jo/infer_coda.py /bags/unimt_tmp/coda$w.npz /bags/unimt_out/unimt_coda${w}_tracks.csv \
      /bags/unimt_out/unimt_coda${w}_dets.csv" > $B/unimt_out/unimt_coda${w}.log 2>&1
  tail -1 $B/unimt_out/unimt_coda${w}.log
done
