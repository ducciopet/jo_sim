#!/bin/bash
# Map experiment, literature baselines as the obstacle source (bags/replay/obs_<method>a_<seq>, written by
# eval/map_obstacles_from_baseline.py). Waits for run_map_campaign.sh to finish (never two GLIM runs at once).
cd $(dirname $0)
until grep -q "map campaign complete" map_campaign.log 2>/dev/null; do sleep 30; done
for s in 1942 1945 1947 1949 1950 1952; do
  q=20260519_$s
  for m in dynablox lvdot; do
    [ -f map_${m}_$q/metadata.yaml ] && continue
    echo "$(date +%T) $m $q"; bash run_glim_map.sh $q dpd obs_${m}a_$q map_${m}_$q
  done
done
echo "$(date +%T) map campaign 2 complete"
