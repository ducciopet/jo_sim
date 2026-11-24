#!/bin/bash
# DOALS -> bags/doals/<scene>_<n>{,_odom,_doals_gt.json} with doals_to_bag.py (one sequence at a time).
# usage: convert_doals.sh <DOALS root on the NAS> [scene/sequence_n ...]
ROOT=$1; shift
cd $(dirname $0)/../../..
PY=${PY:-eval/datasets/.venv/bin/python}; mkdir -p bags/doals
for S in ${@:-hauptgebaeude/sequence_1 hauptgebaeude/sequence_2 niederdorf/sequence_1 niederdorf/sequence_2 station/sequence_1 station/sequence_2 shopville/sequence_1 shopville/sequence_2}; do
  name=$(echo $S | sed 's#/sequence_#_#')
  [ -e bags/doals/$name ] && continue
  echo "$(date +%T) convert $S"
  $PY eval/datasets/doals/doals_to_bag.py $ROOT $S bags/doals/$name || echo "$(date +%T) FAILED $S"
done
