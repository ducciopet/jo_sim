cd /home/duccio/Desktop/jo_sim_sins
for r in "$@"; do for P in ${PROFILES:-mdp_street mdp_street_v1}; do [ -e bags/mdp/runs/${P}_$r.done ] || continue; for G in "" _vehicle _person; do
  o=eval/results/mdp/${P}${G}_$r.json; [ -e $o ] || python3 eval/coda_eval.py --run bags/mdp/runs/${P}_$r --gt bags/mdp/mdp_street${G}_gt.csv --odom bags/mdp/mdp_street_odom --range 15 --out $o > /dev/null 2>&1 || echo FAIL $P $G
  python3 -c "
import json;d=json.load(open('$o'))['metrics']['dynamic_only']
print('%-14s %-3s %-8s rec %5.1f prec %5.1f MOTA %6.1f  tp %6d fp %6d fn %6d'%('$P','$r','${G:-all}',100*d['tp']/max(1,d['num_gt']),100*d['tp']/max(1,d['tp']+d['fp']),100*d['mota'],d['tp'],d['fp'],d['fn']))"
done; done; done
