#!/bin/bash
# KITTI colour images for the labelled SemanticKITTI sequences 00-10: left (image_2) first, then right (image_3, stereo depth)
cd ~/Desktop/jo_sim_sins
U=https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_color.zip
D=/mnt/kloster-reichenau/datasets_SINS/SemanticKITTI
for cam in 2 3; do
  echo "=== $(date '+%F %T') START image_$cam"
  for t in 1 2 3 4 5; do python3 eval/datasets/remote_zip.py $U $D --match "sequences/(0[0-9]|10)/image_$cam/" --workers 16 && break; echo "retry $t"; sleep 60; done
  echo "=== $(date '+%F %T') END image_$cam exit=$? ($(df -h $D | awk 'NR==2{print $4}') free)"
done
