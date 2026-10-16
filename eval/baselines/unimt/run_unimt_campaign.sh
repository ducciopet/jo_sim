#!/bin/bash
# UniMT on every benchmark sequence, REPS repetitions, resumable (<out>.done in bags/unimt_out/).
# Stop between runs: touch bags/unimt_out/STOP. Resume: run again.
D=$(cd $(dirname $0) && pwd); B=$D/../../../bags; REPS=${REPS:-a b c}
for s in 1942 1945 1947 1949 1950 1952; do
  seq=20260519_$s
  for r in $REPS; do
    out=unimt${r}_$seq
    [ -e $B/unimt_out/$out.done ] && continue
    [ -e $B/unimt_out/STOP ] && { echo "$(date +%T) STOP"; exit 0; }
    echo "$(date +%T) start $out"
    bash $D/run_unimt.sh $seq $out --keep-input > $B/unimt_out/${out}_run.log 2>&1 && python3 $D/../check_run.py $B/unimt_out/$out.bag $seq >> $B/unimt_out/${out}_run.log 2>&1 && touch $B/unimt_out/$out.done
    echo "$(date +%T) done  $out"
  done
  rm -f $B/unimt_tmp/in_$seq.bag
done
echo "$(date +%T) campaign complete"
