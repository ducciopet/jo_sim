#!/bin/bash
# SiT: every labelled scene zip -> datasets_SINS/derived/sit/<scene>{,_odom,_gt.csv,_gt.json}. Resumable (<scene>.done).
# Reads /mnt/nas-imperatore/GT/SiT read-only, one scene at a time (nice/ionice: shared NAS).
# usage: setsid nohup bash eval/datasets/sit/convert_sit.sh > <log> 2>&1 < /dev/null &
cd $(dirname $0)/../../..
SRC=${SIT:-/mnt/nas-imperatore/GT/SiT}
D=${OUT:-/mnt/nas-imperatore/datasets_SINS/derived/sit}
PY=eval/datasets/.venv/bin/python
mkdir -p $D
cp $SRC/ImageSets/*.txt $D/ 2>/dev/null
for z in $(ls $SRC/*/*.zip | sort); do
  s=$(basename $z .zip)
  [ -e $D/$s.done ] && continue
  rm -rf $D/$s $D/${s}_odom
  echo "$(date +%T) $s"
  nice -n 10 ionice -c3 $PY eval/datasets/sit/sit_to_bag.py $z $D/$s 2>&1 | tail -1 && [ -e $D/${s}_gt.json ] && touch $D/$s.done
done
echo "$(date +%T) SIT CONVERT DONE"
