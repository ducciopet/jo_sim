#!/bin/bash
# Host-side: regenerate cfg/jrdb_*, run SINS on the JRDB sequences in the container under TAG, evaluate each.
# usage: jrdb_ab.sh <TAG> [sequences]   (env V2_EXTRA: launch args)
cd $(dirname $0)/../..
TAG=$1; shift; S=${@:-forbes huangb packard2 tressider2 clark0 clark1}
for e in jrdb_indoor jrdb_outdoor; do python3 eval/configs/sins/make_cfg.py $e > /dev/null; done
docker exec -u ros jo_sim_sins bash -c "TAG=$TAG V2_EXTRA='$V2_EXTRA' MORE_TOPICS='$MORE_TOPICS' bash /home/ros/bags/replay/run_jrdb.sh $S" > /dev/null 2>&1
for s in $S; do
  python3 eval/coda_eval.py --run bags/jrdb/runs/${TAG}_v2_jrdb_$s --gt bags/jrdb/${s}_gt.csv \
    --odom bags/jrdb/${s}_odom --range 15 --out eval/results/jrdb/${TAG}_$s.json 2>&1 | tail -1 | cut -c1-250
done
