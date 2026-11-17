#!/usr/bin/env python3
"""Pool run_own_eval summaries over the sequences, per method: MOTA, IDF1, FP, FN, IDSW, recall D/PD (main dyn 0.3).
usage: agg_lab.py <eval out dir> [methods...]"""
import json, sys
from pathlib import Path
D = Path(sys.argv[1])
for f in ('summary.json', 'summary_dynamic_only.json'):
    S = json.load(open(D / f)); print('==', f)
    for m, v in S['methods'].items():
        if len(sys.argv) > 2 and m not in sys.argv[2:]: continue
        a = dict(tp=0, fp=0, fn=0, idsw=0, num_gt=0, idtp=0, num_pred=0, ign=0); rc = {}
        for s, r in v['per_seq'].items():
            r = r['0.3']
            for k in ('tp', 'fp', 'fn', 'idsw', 'num_gt', 'idtp', 'num_pred'): a[k] += r[k]
            a['ign'] += r.get('num_pred_ignored_class', 0)
            for st, row in r['confusion'].items():
                n = sum(row.values()); c = rc.setdefault(st, [0, 0]); c[0] += row.get(st, 0); c[1] += n
        mota = 1 - (a['fn'] + a['fp'] + a['idsw']) / a['num_gt']; idf1 = 2 * a['idtp'] / (a['num_gt'] + a['num_pred'])
        rec = ' '.join(f"{k[:3]} {100*c[0]/c[1]:.1f}" for k, c in rc.items() if c[1])
        print(f"{m:7s} MOTA {100*mota:5.1f} IDF1 {100*idf1:5.1f} FP {a['fp']:4d} FN {a['fn']:4d} IDSW {a['idsw']:3d} ign {a['ign']:3d} | recall {rec}")
