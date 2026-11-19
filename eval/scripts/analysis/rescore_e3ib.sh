#!/bin/bash
# 20/11: copy of rescore_e2.sh for e3 + --ignore-box (ignored objects cover their whole box; laptop e23c22b) -> eval/results/{coda,mdp}_e3ib/.
# Second evaluation column next to e3 (e3 stays the main metric until the user decides). Same env TAGS / STREET / BASE / WINDOWS / FRAMES.
# (header of rescore_e2.sh:) Re-score existing runs with the evaluation of 16/11 ("e2": CODa range 20 m, operator ignored only on its box, street
# class views against the whole GT via --ignore-gt). Old results are kept; new ones go to eval/results/{coda,mdp}_e2/.
# Only reads run outputs (no replay, no build); skips results that exist.
#   CODa SINS runs:   TAGS="LVE2 LVE2r2 LVE2r3"  WINDOWS="w1 ... 16_w1"   -> coda_e2/<tag>_<w>.json
#   street SINS runs: STREET="mdp_street_vjsye2_r1 ..." (bags/mdp/runs/<run>)  -> mdp_e2/<run>{,_vehicle,_person}.json
#   CSV/bag baselines on CODa: BASE="eagermot:bags/eagermot_mc_out/eagermot_coda{c}.csv:EMMC ..." (method:template:tag)
# env E=e3: the 17/11 evaluation (e2 + coda_eval --ignore-undefined: annotated rows with undefined motion state become
# ignore regions) -> eval/results/{coda,mdp}_e3/; FRAMES=testing also scores on CODa's testing frames only (-> coda_e3/test/)
cd $(dirname $0)/../../..
E=e3ib; X="--ignore-undefined --ignore-box"
[ -d $HOME/.venvs/sins_eval/bin ] && export PATH=$HOME/.venvs/sins_eval/bin:$PATH  # lab venv (pandas)
mkdir -p eval/results/coda_$E eval/results/mdp_$E
cw(){ case $1 in *_*) echo $1 ;; *) echo 0_$1 ;; esac; }
for w in ${WINDOWS:-w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1}; do c=$(cw $w)
  for F in all $FRAMES; do D=eval/results/coda_$E; [ $F = all ] || D=$D/test; mkdir -p $D
  for t in $TAGS; do r=bags/coda/runs/${t}_v2_coda$c; o=$D/${t}_$w.json
    [ -e $o ] || [ ! -e $r ] || python3 eval/coda_eval.py $X --frames $F --run $r --gt bags/coda/coda${c}_gt.csv --odom bags/coda/coda${c}_odom --out $o > /dev/null 2>&1 || echo "FAIL $t $w $F"; done
  for b in $BASE; do IFS=: read m tpl tag <<< "$b"; r=${tpl//\{c\}/$c}; o=$D/${tag}_$w.json
    [ -e $o ] || [ ! -e $r ] || python3 eval/coda_eval.py $X --frames $F --method $m --run $r --gt bags/coda/coda${c}_gt.csv --odom bags/coda/coda${c}_odom --out $o > /dev/null 2>&1 || echo "FAIL $tag $w $F"; done
  done
done
for s in $STREET; do r=bags/mdp/runs/$s; [ -e $r ] || { echo "missing $r"; continue; }
  G=bags/mdp/mdp_street; A="--run $r --odom ${G}_odom --range 15"
  [ -e eval/results/mdp_$E/$s.json ] || python3 eval/coda_eval.py $X $A --gt ${G}_gt.csv --out eval/results/mdp_$E/$s.json > /dev/null 2>&1 || echo "FAIL $s"
  [ -e eval/results/mdp_$E/${s}_vehicle.json ] || python3 eval/coda_eval.py $X $A --gt ${G}_vehicle_gt.csv --ignore-gt ${G}_person_gt.csv --out eval/results/mdp_$E/${s}_vehicle.json > /dev/null 2>&1 || echo "FAIL $s vehicle"
  [ -e eval/results/mdp_$E/${s}_person.json ] || python3 eval/coda_eval.py $X $A --gt ${G}_person_gt.csv --ignore-gt ${G}_vehicle_gt.csv --out eval/results/mdp_$E/${s}_person.json > /dev/null 2>&1 || echo "FAIL $s person"
done
echo "RESCORE $E DONE"
