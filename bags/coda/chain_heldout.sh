#!/bin/bash
# CODa held-out (seqs 10, 11, 12), one sequence at a time for the disk: convert -> SINS H1 x3 (frozen heldout-v5,
# STEREO=1, as T1) -> Dynablox, M-detector, LV-DOT, CenterPoint+SimpleTrack, EagerMOT -> drop the window bags
# (regenerable from the NAS; GT csv/json and odometry kept). Resumable: every step skips what is done.
cd /home/duccio/Desktop/jo_sim_sins
B=bags; E=eval/baselines; CODA=/mnt/nas-imperatore/datasets_SINS/CODa
while pgrep -f "convert_coda_heldou[t].sh" > /dev/null; do sleep 30; done   # a conversion started by hand
for S in 10 11 12; do
  case $S in 10) W="10_w1 10_w2 10_w3 10_w4 10_w5" ;; *) W="${S}_w1 ${S}_w2 ${S}_w3" ;; esac
  [ -e $B/coda/heldout_$S.done ] && continue
  if [ $S != 10 ]; then until grep -q "ALL DONE" ~/coda_heldout_dl.log; do sleep 60; done; fi
  bash eval/datasets/convert_coda_heldout.sh $S >> $B/coda/convert$S.log 2>&1
  for w in $W; do [ -e $B/coda/coda$w.done ] || { echo "$(date +%T) conversion of $w failed"; exit 1; }; done
  echo "$(date +%T) seq $S converted"
  for T in H1 H1r2 H1r3; do
    todo=""; for w in $W; do [ -e $B/coda/runs/${T}_v2_coda$w.done ] || todo="$todo $w"; done
    [ -n "$todo" ] && STEREO=1 bash eval/scripts/coda_ab.sh $T $todo
  done
  echo "$(date +%T) seq $S SINS done"
  for w in $W; do
    [ -e $B/dynablox_out/dynabloxa_coda$w.done ] || bash $E/dynablox/run_dynablox_coda.sh coda$w dynabloxa_coda$w | tail -1
    rm -f $B/dynablox_tmp/in_coda$w.bag
    [ -e $B/mdetector_out/mdetectora_coda$w.done ] || bash $E/mdetector/run_mdetector_coda.sh coda$w mdetectora_coda$w | tail -1
    [ -e $B/lvdot_out/lvdota_coda$w.done ] || bash $E/lvdot/run_lvdot_coda.sh coda$w lvdota_coda$w | tail -1
    rm -f $B/lvdot_tmp/in_coda$w.bag
    mkdir -p $B/unimt_tmp
    if [ ! -s $B/unimt_out/unimt_coda${w}_tracks.csv ]; then
      python3 $E/unimt/make_coda_manifest.py $CODA $B/coda/coda${w}_gt.json $B/unimt_tmp/coda$w.npz | tail -1
      bash $E/unimt/run_unimt_coda.sh $w
    fi
    if [ ! -e $B/eagermot_out/eagermot_coda$w.csv.done ]; then
      python3 $E/eagermot/export_inputs_coda.py $w H1_v2_coda$w | tail -1 &&
      $E/eagermot/.venv/bin/python $E/eagermot/run_eagermot_jo.py $B/eagermot_out/in_coda$w.json $B/eagermot_out/eagermot_coda$w.csv | tail -1 &&
      touch $B/eagermot_out/eagermot_coda$w.csv.done
    fi
    echo "$(date +%T) $w baselines done"
  done
  for w in $W; do rm -rf $B/coda/coda$w $B/coda/coda${w}_stereo; done
  touch $B/coda/heldout_$S.done; echo "$(date +%T) seq $S complete"; df -h / | tail -1
done
python3 eval/coda_compare.py --heldout 2>&1 | grep -v "per label"
echo "$(date +%T) HELDOUT ALL DONE"
