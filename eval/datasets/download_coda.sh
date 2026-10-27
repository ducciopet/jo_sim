#!/bin/bash
# CODa (UT Campus Object Dataset, CC BY-NC-SA 4.0) sequences with LiDAR + stereo + 3D boxes AND the separate RGB-D
# (Azure Kinect / ZED) archive, from TACC (no registration). One archive at a time: download (16 parallel connections, resumable: pget.py), unzip,
# delete the zip, so the share never holds both. Default: sequences 0 and 1 (GDC, ~14 min each, ~120 GB).
# usage: download_coda.sh [dest] [seq ...]
DEST=${1:-/mnt/kloster-reichenau/datasets_SINS/CODa}; shift; SEQS=${@:-0 1}
URL=https://web.corral.tacc.utexas.edu/texasrobotics/web_CODa
mkdir -p $DEST/_zips
for s in $SEQS; do
  for part in sequences depthonly; do
    tag=${part}_$s
    [ -e $DEST/_zips/$tag.extracted ] && continue
    echo "$(date +%T) download $tag"
    python3 $(dirname $0)/pget.py $URL/$part/$s.zip $DEST/_zips/$tag.zip 16 || { echo "$(date +%T) FAILED download $tag"; exit 1; }
    echo "$(date +%T) unzip $tag"
    unzip -q -o $DEST/_zips/$tag.zip -d $DEST || { echo "$(date +%T) FAILED unzip $tag"; exit 1; }
    rm -f $DEST/_zips/$tag.zip; touch $DEST/_zips/$tag.extracted
    echo "$(date +%T) done $tag ($(df -h $DEST | awk 'NR==2{print $4}') free)"
  done
done
echo "$(date +%T) coda download complete"
