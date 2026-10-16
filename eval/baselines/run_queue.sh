#!/bin/bash
# Baseline campaigns ONE AT A TIME (two at once share the disk: a 5 GB input conversion makes the other's replay fall
# behind real time). usage: run_queue.sh <campaign script> [...]   e.g. run_queue.sh xu_rgbd/run_xu_rgbd_yolo_campaign.sh unimt/run_unimt_campaign.sh
D=$(cd $(dirname $0) && pwd)
while ps -eo args | awk '$1=="bash" && NF==2 && $2 ~ /_campaign\.sh$/' | grep -q .; do sleep 30; done   # wait for any campaign already running
for c in "$@"; do echo "$(date +%T) queue: $c"; bash $D/$c; done
echo "$(date +%T) queue complete"
