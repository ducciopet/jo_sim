#!/bin/bash
# JRDB held-out sequences -> bags/jrdb/<short name>. Chosen BEFORE any SINS run, from JRDB's own action labels
# (most still people within 15 m, eval/datasets/jrdb/README.md); indoor/outdoor from the images.
# Resumable (<name>.done).  usage: setsid nohup bash eval/datasets/jrdb/convert_jrdb.sh > bags/jrdb/convert.log 2>&1 < /dev/null &
cd $(dirname $0)/../../..
R=${JRDB:-/mnt/kloster-reichenau/datasets_SINS/JRDB/train_dataset_with_activity}
PY=eval/datasets/.venv/bin/python
mkdir -p bags/jrdb
for s in "forbes-cafe-2019-01-22_0 forbes" "huang-basement-2019-01-25_0 huangb" "packard-poster-session-2019-03-20_2 packard2" \
         "tressider-2019-04-26_2 tressider2" "clark-center-2019-02-28_0 clark0" "clark-center-2019-02-28_1 clark1"; do
  set -- $s; out=bags/jrdb/$2
  [ -e $out.done ] && continue
  rm -rf $out ${out}_odom
  echo "$(date +%T) start $1"
  $PY eval/datasets/jrdb/jrdb_to_bag.py $R $1 $out 2>&1 | tail -1 && touch $out.done
done
echo "$(date +%T) all done"
