#!/bin/bash
# tracker_node alone on every sequence's recorded inputs (<IN_PREFIX>_v2_<seq>), output <PREFIX>_v2_<seq>
D=$(dirname $0)
for s in 1942 1945 1947 1949 1950 1952; do
  bash $D/run_tracker_only.sh $D/${IN_PREFIX}_v2_20260519_$s $D/${PREFIX}_v2_20260519_$s ${RATE:-1.0} -- $TRACKER_ARGS
  echo "tracker $s"
done
