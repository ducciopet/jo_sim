#!/bin/bash
# Xu et al. 2023 on every benchmark sequence, REPS repetitions, resumable (<out>.done in bags/xu_rgbd_out/).
# Stop between runs: touch bags/xu_rgbd_out/STOP. Resume: run again.
D=$(cd $(dirname $0) && pwd); B=$D/../../../bags; REPS=${REPS:-a b c}
for s in 1942 1945 1947 1949 1950 1952; do
  seq=20260519_$s
  for r in $REPS; do
    out=xuyolo${r}_$seq
    [ -e $B/xu_rgbd_out/$out.done ] && continue
    [ -e $B/xu_rgbd_out/STOP ] && { echo "$(date +%T) STOP"; exit 0; }
    echo "$(date +%T) start $out"
    bash $D/run_xu_rgbd_yolo.sh $seq $out --keep-input > $B/xu_rgbd_out/${out}_run.log 2>&1 && python3 $D/../check_run.py $B/xu_rgbd_out/$out.bag $seq >> $B/xu_rgbd_out/${out}_run.log 2>&1 && touch $B/xu_rgbd_out/$out.done
    echo "$(date +%T) done  $out"
  done
  rm -f $B/xu_rgbd_tmp/in_yolo_$seq.bag
done
echo "$(date +%T) campaign complete"
