#!/usr/bin/env python3
"""MOTA (full = all annotated agents, and dynamic_only = moving) pooled per CODa sequence from coda_e3 jsons, for SINS
tags (mean over the given reps) and baselines. usage: cmp_e3.py <dir: eval/results/coda_e3 or .../test> TAG[,r2,r3] ..."""
import json, sys
from pathlib import Path
D = Path(sys.argv[1]); W = {'0': ['w1', 'w2', 'w3'], '10': ['10_w1', '10_w2', '10_w3', '10_w4', '10_w5'], '16': ['16_w1'],
                            '11': ['11_w1', '11_w2', '11_w3'], '12': ['12_w1', '12_w2', '12_w3']}
def pool(tag, ws, mode):
    a = dict(tp=0, fp=0, fn=0, idsw=0, num_gt=0); n = 0
    for w in ws:
        f = D / f'{tag}_{w}.json'
        if not f.exists(): return None
        m = json.load(open(f))['metrics'][mode]; n += 1
        for k in a: a[k] += m[k]
    return 100 * (1 - (a['fn'] + a['fp'] + a['idsw']) / a['num_gt']) if a['num_gt'] else None
print(f'{D}:  MOTA all / moving per seq')
for spec in sys.argv[2:]:
    tags = spec.split(',')
    cells = []
    for s, ws in W.items():
        v = [pool(t, ws, 'full') for t in tags]; m = [pool(t, ws, 'dynamic_only') for t in tags]
        v = [x for x in v if x is not None]; m = [x for x in m if x is not None]
        cells.append(f'{s}: ' + (f'{sum(v)/len(v):5.1f} / {sum(m)/len(m):5.1f}' if v else '   -  /   -  '))
    print(f'{tags[0]:6s} ' + ' | '.join(cells))
