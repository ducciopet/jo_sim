#!/bin/bash
# EagerMOT on every benchmark sequence and repetition: inputs from the CMDT detections of the UniMT node
# (PREFIX=unimtdet campaign of eval/baselines/unimt/run_unimt_campaign.sh, waited for) and the YOLO boxes of the
# SINS replay of the same repetition. Offline and deterministic given its inputs. Resumable (<out>.csv.done).
D=$(cd $(dirname $0) && pwd); R=$(cd $D/../../.. && pwd); B=$R/bags; REPS=${REPS:-a b c}
mkdir -p $B/eagermot_out
for s in 1942 1945 1947 1949 1950 1952; do
  seq=20260519_$s
  for r in $REPS; do
    out=$B/eagermot_out/eagermot${r}_$seq.csv
    [ -e $out.done ] && continue
    until [ -e $B/unimt_out/unimtdet${r}_$seq.done ]; do sleep 20; done
    python3 $D/export_inputs.py $seq $r && $D/.venv/bin/python $D/run_eagermot_jo.py $B/eagermot_out/in_${r}_$seq.json $out \
      && touch $out.done
  done
done
echo "$(date +%T) eagermot campaign complete"
