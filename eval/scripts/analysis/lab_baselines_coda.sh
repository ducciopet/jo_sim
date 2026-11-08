#!/bin/bash
# Phase D on the lab PC: Dynablox (max_range 20, paper default) and M-detector (KITTI-based coda_kitti.yaml, 0.25x,
# declared "offline, slowed 4x") on every CODa window (tuning 0/10/16 + held-out 11/12), one window copied locally at a
# time (NAS replay drops scans). Each output is scored with coda_eval.py on all annotated frames and on the CODa
# 'testing' frames only (eval/baselines/FIDELITY.md section 1), then the output bag is moved to the NAS and the local
# window deleted. Resumable: a window is skipped when both result files exist.
# env METHODS="dynablox mdetector", WINDOWS="0_w1 ..."; log: bags/baselines_build/coda_chain.log
cd $(dirname $0)/../../..
export PATH=$HOME/.venvs/sins_eval/bin:$PATH
NAS=/mnt/nas-imperatore/datasets_SINS/derived; OUTNAS=$NAS/baselines_out; mkdir -p $OUTNAS
WINDOWS=${WINDOWS:-"0_w1 0_w2 0_w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 11_w1 11_w2 11_w3 12_w1 12_w2 12_w3"}
METHODS=${METHODS:-"dynablox mdetector"}
score() {  # method run tag window
  for fr in all testing; do
    res=eval/results/coda/cmpF${fr:0:1}_$3_$4.json
    [ -e $res ] && continue
    python3 eval/coda_eval.py --method $1 --run $2 --gt bags/coda/coda$4_gt.csv --odom bags/coda/coda$4_odom \
      --frames $fr --out $res > /dev/null || echo "$(date +%T) SCORE FAIL $3 $4 $fr"
    python3 -c "import json;d=json.load(open('$res'))['metrics']['dynamic_only'];print('$(date +%T) $3 $4 $fr moving rec %.1f prec %.1f MOTA %.1f'%(100*d['recall'],100*d['precision'],100*d['mota']))" 2>/dev/null
  done
}
powerprofilesctl set performance 2>/dev/null
for W in $WINDOWS; do
  todo=0
  for m in $METHODS; do for fr in a t; do [ -e eval/results/coda/cmpF${fr}_${m}_$W.json ] || todo=1; done; done
  [ $todo = 0 ] && continue
  rsync -a $NAS/coda_windows/coda${W} $NAS/coda_windows/coda${W}_odom $NAS/coda_windows/coda${W}_gt.csv \
    $NAS/coda_windows/coda${W}_gt.json bags/coda/ || { echo "$(date +%T) COPY FAIL $W"; continue; }
  echo "$(date +%T) $W copied, $(df -h --output=avail / | tail -1) free"
  if [[ " $METHODS " == *" dynablox "* ]]; then
    bash eval/baselines/dynablox/run_dynablox_coda.sh coda$W dynabloxF_coda$W > /dev/null 2>&1 || echo "$(date +%T) RUN FAIL dynablox $W"
    rm -f bags/dynablox_tmp/in_coda$W.bag
    score dynablox bags/dynablox_out/dynabloxF_coda$W.bag dynablox $W
    mv bags/dynablox_out/dynabloxF_coda$W.bag $OUTNAS/ 2>/dev/null
  fi
  if [[ " $METHODS " == *" mdetector "* ]]; then
    CFG=coda_kitti.yaml RATE=0.25 bash eval/baselines/mdetector/run_mdetector_coda.sh coda$W mdetectorF_coda$W > /dev/null 2>&1 \
      || echo "$(date +%T) RUN FAIL mdetector $W"
    score mdetector bags/mdetector_out/mdetectorF_coda$W.bag mdetector $W
    mv bags/mdetector_out/mdetectorF_coda$W.bag $OUTNAS/ 2>/dev/null
  fi
  rm -rf bags/coda/coda${W} bags/coda/coda${W}_odom
done
powerprofilesctl set balanced 2>/dev/null
echo "$(date +%T) CODA BASELINES DONE"
