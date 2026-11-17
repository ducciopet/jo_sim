#!/bin/bash
# CODa moving-vehicle view with the 16/11 evaluation ("e2"), identical for every method:
#   GT  = the window's annotated vehicles (Car, Pickup Truck, Utility Vehicle, Service Vehicle, Delivery Truck) plus an
#         empty-frame marker on every annotated frame without one, so the frame grid is the window's own;
#   --ignore-gt = all the other annotated objects (pedestrians, bikes, scooters...): a prediction on them is neither
#         TP nor FP of the vehicle view (as the street class views, rescore_e2.sh);
#   coda_eval.py defaults otherwise (range 20 m, operator ignored on its own box). Read: dynamic_only recall/precision.
# Windows: the CODa windows with annotated vehicles; tuning 0_w1 10_w1 10_w3 10_w4 10_w5 16_w1 (default),
# held-out 11_w1 11_w2 12_w2 12_w3 (only for the final check of a chosen candidate).
# GT files: eval/results/coda_vehicle_e2/gt/coda<w>_{vgt,ogt}.csv (+ _vgt.json copied from the window), regenerated if missing.
# usage: TAGS="LVE2 LVE2r2 LVE2r3" [BASE="dynablox mdetector"] [VW="..."] bash eval/scripts/analysis/coda_vehicle_e2.sh
#   SINS runs bags/coda/runs/<tag>_v2_coda<w>; baselines NAS derived/baselines_out/<m>F_coda<w>.bag (read only)
#   -> eval/results/coda_vehicle_e2/<tag or method>_<w>.json. Resumable (existing json skipped).
cd $(dirname $0)/../../..
[ -d $HOME/.venvs/sins_eval/bin ] && export PATH=$HOME/.venvs/sins_eval/bin:$PATH  # lab venv (pandas)
E=${E:-e2}; X=""; [ $E = e3 ] && X="--ignore-undefined"   # E=e3: + --ignore-undefined -> coda_vehicle_e3
G=eval/results/coda_vehicle_e2/gt; O=eval/results/coda_vehicle_$E; mkdir -p $G $O
B=/mnt/nas-imperatore/datasets_SINS/derived/baselines_out
for c in ${VW:-0_w1 10_w1 10_w3 10_w4 10_w5 16_w1}; do
  [ -e $G/coda${c}_vgt.csv ] || python3 - "$c" "$G" <<'EOF'
import sys, pandas as pd
c, G = sys.argv[1], sys.argv[2]
V = ['Car', 'Pickup Truck', 'Utility Vehicle', 'Service Vehicle', 'Delivery Truck']
g = pd.read_csv(f'bags/coda/coda{c}_gt.csv', keep_default_na=False)
obj = g[g.track_id.astype(int) >= 0]
v = obj[obj['class'].isin(V)]
frames = sorted(set(g.t_ns.astype('int64')))
empty = [t for t in frames if t not in set(v.t_ns.astype('int64'))]
mark = pd.DataFrame({col: '' for col in g.columns}, index=range(len(empty)))
mark['t_ns'] = empty; mark['track_id'] = -1
pd.concat([v, mark]).sort_values('t_ns', kind='stable').to_csv(f'{G}/coda{c}_vgt.csv', index=False)
obj[~obj['class'].isin(V)].to_csv(f'{G}/coda{c}_ogt.csv', index=False)
print(c, 'vehicle ids', v.track_id.nunique(), 'rows', len(v), '| other rows', len(obj) - len(v))
EOF
  cp -n bags/coda/coda${c}_gt.json $G/coda${c}_vgt.json 2>/dev/null
  A="--gt $G/coda${c}_vgt.csv --ignore-gt $G/coda${c}_ogt.csv --odom bags/coda/coda${c}_odom"
  for t in $TAGS; do o=$O/${t}_$c.json; r=bags/coda/runs/${t}_v2_coda$c
    [ -e $o ] || [ ! -e $r ] || python3 eval/coda_eval.py $X --run $r $A --out $o > /dev/null 2>&1 || echo "FAIL $t $c"; done
  for m in $BASE; do o=$O/${m}_$c.json; r=$B/${m}F_coda$c.bag
    [ -e $o ] || [ ! -e $r ] || python3 eval/coda_eval.py $X --method $m --run $r $A --out $o > /dev/null 2>&1 || echo "FAIL $m $c"; done
  # CSV/bag baselines anywhere (laptop outputs), as rescore_e2.sh: LBASE="simpletrack:bags/unimt_out/unimt_coda{c}_tracks.csv:ST ..."
  for b in $LBASE; do IFS=: read m tpl tag <<< "$b"; r=${tpl//\{c\}/$c}; o=$O/${tag}_$c.json
    [ -e $o ] || [ ! -e $r ] || python3 eval/coda_eval.py $X --method $m --run $r $A --out $o > /dev/null 2>&1 || echo "FAIL $tag $c"; done
done
echo "CODA VEHICLE E2 DONE"
