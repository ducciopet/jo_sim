#!/bin/bash
# JRDB TUNING split -> bags/jrdb/<short name>. Chosen 03/11/2026 BEFORE any SINS run on them: JRDB train sequences that are
# not held-out and not recorded at a held-out place (packard-poster-session_0/_1 excluded: same event as packard2), ranked by
# still people within 15 m (JRDB action labels); indoor/outdoor from the images. bytes-cafe_0 (debug, already converted) joins
# the indoor half. Profiles may be tuned ONLY on these; the six held-out sequences are scored once at the end.
# usage: setsid nohup bash eval/datasets/jrdb/convert_jrdb_tune.sh > bags/jrdb/convert_tune.log 2>&1 < /dev/null &
cd $(dirname $0)/../../..
R=${JRDB:-/mnt/nas-imperatore/datasets_SINS/JRDB/train_dataset_with_activity}
PY=eval/datasets/.venv/bin/python
for s in "gates-ai-lab-2019-02-08_0 gateslab" "nvidia-aud-2019-04-18_0 nvidia" "stlc-111-2019-04-19_0 stlc" \
         "tressider-2019-03-16_1 tressider1" "memorial-court-2019-03-16_0 memorial" "hewlett-packard-intersection-2019-01-24_0 hewlett"; do
  set -- $s; out=bags/jrdb/$2
  [ -e $out.done ] && continue
  rm -rf $out ${out}_odom
  echo "$(date +%T) start $1"
  $PY eval/datasets/jrdb/jrdb_to_bag.py $R $1 $out 2>&1 | tail -1 && touch $out.done
done
echo "$(date +%T) all done"
