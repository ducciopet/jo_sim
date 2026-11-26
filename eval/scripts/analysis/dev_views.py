#!/usr/bin/env python3
"""Summary of the 26/11 local rescoring of the current-base development runs (NAS offline): per arm (3 reps), pooled over the
9 CODa tuning windows (or the 6 lab sequences) per rep: MOTA, IDF1 (from summed counts), HOTA/DetA/AssA (hota_from_sums), and
the state recalls. Reports mean and [min-max] over reps. usage: dev_views.py coda|lab"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from harness.metrics import hota_from_sums   # noqa: E402
R = Path(__file__).resolve().parents[2] / 'results'
WS = ['w1', 'w2', 'w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']
ARMS = {'EB (chain E)': ['EB', 'EBr2', 'EBr3'], 'N0 (chain G)': ['N0', 'N0r2', 'N0r3'], 'H0 (chain H)': ['H0', 'H0r2', 'H0r3']}
LARMS = {'LN (chain G)': ['LN0a', 'LN0b', 'LN0c', 'LN1a', 'LN1b', 'LN1c'], 'LJ (chain H)': ['LJ0a', 'LJ0b', 'LJ0c', 'LJ1a', 'LJ1b', 'LJ1c'],
         'LGP (ground check)': ['LGP0a', 'LGP0b', 'LGP0c', 'LGP1a', 'LGP1b', 'LGP1c']}
def pool(ms):
    s = {k: sum(m[k] for m in ms) for k in ('num_gt', 'tp', 'fp', 'fn', 'idsw', 'idtp', 'num_pred')}
    h = {k: np.sum([m['hota_alpha'][k] for m in ms], 0) for k in ('tp', 'fn', 'fp', 'assa_num')}
    hota, deta, assa = hota_from_sums(h)
    rec = {}
    for st in ('dynamic', 'potentially_dynamic'):
        n = sum(sum(m['confusion'][st].values()) for m in ms); c = sum(m['confusion'][st][st] for m in ms)
        rec[st] = c / n if n else float('nan')
    return dict(mota=1 - (s['fp'] + s['fn'] + s['idsw']) / s['num_gt'], idf1=2 * s['idtp'] / (s['num_gt'] + s['num_pred']),
                hota=hota, deta=deta, assa=assa, recD=rec['dynamic'], recPD=rec['potentially_dynamic'])
def line(name, rows):
    f = lambda k: f"{100 * np.mean([r[k] for r in rows]):5.1f} [{100 * min(r[k] for r in rows):.1f}-{100 * max(r[k] for r in rows):.1f}]"
    return f"| {name} | " + ' | '.join(f(k) for k in ('mota', 'idf1', 'hota', 'deta', 'assa', 'recD', 'recPD')) + ' |'
HEAD = '| arm | MOTA | IDF1 | HOTA | DetA | AssA | recall D | recall PD |\n|---|---|---|---|---|---|---|---|'
if sys.argv[1] == 'coda':
    for v in ['e3', 'state', 'pdprev', 'dyn0.2', 'dyn0.4']:
        for fr in ['', 'test']:
            d = R / 'coda_dev_views' / v / fr
            print(f'\nCODa view {v}, frames {"testing" if fr else "all"} (pooled 9 windows per rep; mean [min-max] over 3 reps)\n' + HEAD)
            for name, tags in ARMS.items():
                try: rows = [pool([json.load(open(d / f'{t}_{w}.json'))['metrics']['full'] for w in WS]) for t in tags]
                except FileNotFoundError as e: print(f'| {name} | missing {e.filename} |'); continue
                print(line(name, rows))
else:
    d = json.load(open(R / 'lab_dev_views' / 'summary.json'))['methods']
    for th in ['0.3', '0.2', '0.4']:
        print(f'\nLab, GT dynamic threshold {th} m/s (pooled 6 sequences per rep; mean [min-max] over 6 reps)\n' + HEAD)
        for name, tags in LARMS.items():
            print(line(name, [pool([d[t]['per_seq'][s][th] for s in d[t]['per_seq']]) for t in tags]))
