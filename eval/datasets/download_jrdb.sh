#!/bin/bash
# JRDB 2022/2024 full train set (JRDB account needed; the link is the one on jrdb.erc.monash.edu/#downloads after login).
# Same scheme as download_coda.sh: 16 parallel resumable connections (pget.py), unzip, delete the zip.
# usage: download_jrdb.sh [dest]
DEST=${1:-/mnt/kloster-reichenau/datasets_SINS/JRDB}
URL=https://jrdb.erc.monash.edu/static/downloads/JRDB2024/train_dataset_with_activity.zip
tag=train_dataset_with_activity
mkdir -p $DEST/_zips
[ -e $DEST/_zips/$tag.extracted ] && { echo "$tag already extracted"; exit 0; }
echo "$(date +%T) download $tag"
python3 $(dirname $0)/pget.py $URL $DEST/_zips/$tag.zip 16 || { echo "$(date +%T) FAILED download $tag"; exit 1; }
echo "$(date +%T) unzip $tag"
# the share can drop for a moment (Host is down): retry the whole archive, -o rewrites half-written files
for t in 1 2 3; do unzip -q -o $DEST/_zips/$tag.zip -d $DEST && break; [ $t = 3 ] && { echo "$(date +%T) FAILED unzip $tag"; exit 1; }; echo "$(date +%T) retry unzip $tag"; sleep 300; done
rm -f $DEST/_zips/$tag.zip; touch $DEST/_zips/$tag.extracted
echo "$(date +%T) jrdb download complete ($(df -h $DEST | awk 'NR==2{print $4}') free)"
