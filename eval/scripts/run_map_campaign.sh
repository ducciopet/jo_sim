#!/bin/bash
# Map experiment on every benchmark sequence (inside the container, from bags/replay): GLIM alone with each
# rejection variant; resumable (a finished map has its metadata.yaml). Obstacles from the r9a detector runs.
cd $(dirname $0)
for s in 1942 1945 1947 1949 1950 1952; do
  q=20260519_$s
  run() { [ -f map_$1_$q/metadata.yaml ] && return; echo "$(date +%T) $1 $q"; bash run_glim_map.sh $q $2 $3 map_$1_$q; }
  run none none -
  run none2 none -
  run v2d d r9a_v2_$q
  run v2dpd dpd r9a_v2_$q
  OBS_TOPIC=/jo_detector/tracked_dynamic_obstacles run v1dpd dpd r9a_v1_$q
done
echo "$(date +%T) map campaign complete"
