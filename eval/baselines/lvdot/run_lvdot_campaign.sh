#!/bin/bash
# LV-DOT on every benchmark sequence, REPS repetitions, resumable (<out>.done in bags/lvdot_out/).
# Stop between runs: touch bags/lvdot_out/STOP. Resume: run again.
D=$(cd $(dirname $0) && pwd); B=$D/../../../bags; REPS=${REPS:-a b c}
for s in 1942 1945 1947 1949 1950 1952; do
  seq=20260519_$s
  for r in $REPS; do
    out=lvdot${r}_$seq
    [ -e $B/lvdot_out/$out.done ] && continue
    [ -e $B/lvdot_out/STOP ] && { echo "$(date +%T) STOP"; exit 0; }
    echo "$(date +%T) start $out"
    bash $D/run_lvdot.sh $seq $out --keep-input > $B/lvdot_out/${out}_run.log 2>&1 && python3 $D/../check_run.py $B/lvdot_out/$out.bag $seq >> $B/lvdot_out/${out}_run.log 2>&1 && touch $B/lvdot_out/$out.done
    echo "$(date +%T) done  $out"
  done
  rm -f $B/lvdot_tmp/in_$seq.bag
done
echo "$(date +%T) campaign complete"
