#!/bin/bash
# Sensitivity of the derived motion label (user decision 17/11): re-score e3 (all + testing frames) with a different
# speed threshold for "moving" (coda_eval --dyn-speed; default 0.3 m/s over +-0.5 s). Same windows and baselines as
# rescore_e2.sh; results in eval/results/coda_e3_dyn<V>/ (+ test/). Separate script so that rescore_e2.sh, used by the
# lab chains, is never edited while a chain runs.
# usage: DYN=0.2 TAGS="LXI LXIr2 ..." BASE="method:template{c}:tag ..." [WINDOWS=...] [CODA_ROOT=...] bash rescore_dyn.sh
cd $(dirname $0)/../../..
[ -d $HOME/.venvs/sins_eval/bin ] && export PATH=$HOME/.venvs/sins_eval/bin:$PATH
: ${DYN:?set DYN}
CR=${CODA_ROOT:+--coda-root $CODA_ROOT}
cw(){ case $1 in *_*) echo $1 ;; *) echo 0_$1 ;; esac; }
for w in ${WINDOWS:-w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 11_w1 11_w2 11_w3 12_w1 12_w2 12_w3}; do c=$(cw $w)
  for F in all testing; do D=eval/results/coda_e3_dyn$DYN; [ $F = all ] || D=$D/test; mkdir -p $D
    for t in $TAGS; do r=bags/coda/runs/${t}_v2_coda$c; o=$D/${t}_$w.json
      [ -e $o ] || [ ! -e $r ] || python3 eval/coda_eval.py --ignore-undefined --dyn-speed $DYN $CR --frames $F --run $r --gt bags/coda/coda${c}_gt.csv --odom bags/coda/coda${c}_odom --out $o > /dev/null 2>&1 || echo "FAIL $t $w $F"; done
    for b in $BASE; do IFS=: read m tpl tag <<< "$b"; r=${tpl//\{c\}/$c}; o=$D/${tag}_$w.json
      [ -e $o ] || [ ! -e $r ] || python3 eval/coda_eval.py --ignore-undefined --dyn-speed $DYN $CR --frames $F --method $m --run $r --gt bags/coda/coda${c}_gt.csv --odom bags/coda/coda${c}_odom --out $o > /dev/null 2>&1 || echo "FAIL $tag $w $F"; done
  done
done
echo "RESCORE DYN $DYN DONE"
