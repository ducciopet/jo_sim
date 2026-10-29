#!/bin/bash
# KITTI odometry velodyne scans + calibration for SemanticKITTI (labels already in DEST). The archive links come from the
# KITTI site after login (cvlibs.net, eval_odometry.php); they extract to dataset/sequences/XX/, next to the labels.
# Same scheme as download_coda.sh: one archive at a time, 16 parallel resumable connections (pget.py), unzip, delete the zip.
# usage: download_kitti.sh [dest] [archive ...]
DEST=${1:-/mnt/kloster-reichenau/datasets_SINS/SemanticKITTI}; shift; PARTS=${@:-data_odometry_calib data_odometry_velodyne}
URL=https://s3.eu-central-1.amazonaws.com/avg-kitti
mkdir -p $DEST/_zips
for tag in $PARTS; do
  [ -e $DEST/_zips/$tag.extracted ] && continue
  echo "$(date +%T) download $tag"
  python3 $(dirname $0)/pget.py $URL/$tag.zip $DEST/_zips/$tag.zip 16 || { echo "$(date +%T) FAILED download $tag"; exit 1; }
  echo "$(date +%T) unzip $tag"
  # the share can drop for a moment (Host is down): retry the whole archive, -o rewrites half-written files
  for t in 1 2 3; do unzip -q -o $DEST/_zips/$tag.zip -d $DEST && break; [ $t = 3 ] && { echo "$(date +%T) FAILED unzip $tag"; exit 1; }; echo "$(date +%T) retry unzip $tag"; sleep 300; done
  rm -f $DEST/_zips/$tag.zip; touch $DEST/_zips/$tag.extracted
  echo "$(date +%T) done $tag ($(df -h $DEST | awk 'NR==2{print $4}') free)"
done
echo "$(date +%T) kitti download complete"
