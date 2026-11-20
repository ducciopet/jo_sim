#!/usr/bin/env python3
"""Pooled MOTA-all over the 9 CODa tuning windows (counts summed: 1 - (FN+FP+IDSW)/GT), per rep, for the 20/11 decision rule.
usage: pooled9.py <dir: coda_e3[/test] or coda_e3ib[/test]> REF[,r2,r3] TAG[,r2,r3] ...
Per group: per-rep pooled MOTA, mean, range; for each arm the paired gain vs the reference (rep k vs rep k) and the reference
range (max-min of its 3 reps). Rule (PIPELINE, 'Decisions 20/11 afternoon'): gain > reference spread on both testing and all
frames; vetoes (a seq below the reference rep range, street) are checked with cmp_e3.py / agg_e3.py."""
import json, sys
from pathlib import Path
D = Path(sys.argv[1]); WS = ['w1', 'w2', 'w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']
def pool(tag, mode='full'):
    a = dict(fp=0, fn=0, idsw=0, num_gt=0)
    for w in WS:
        f = D / f'{tag}_{w}.json'
        if not f.exists(): return None
        m = json.load(open(f))['metrics'][mode]
        for k in a: a[k] += m[k]
    return 100 * (1 - (a['fn'] + a['fp'] + a['idsw']) / a['num_gt'])
groups = [s.split(',') for s in sys.argv[2:]]
vals = [[pool(t) for t in g] for g in groups]
ref = vals[0]; rr = [x for x in ref if x is not None]; spread = max(rr) - min(rr) if rr else float('nan')
print(f'{D}: pooled MOTA all, 9 windows | ref {groups[0][0]} spread {spread:.1f}')
for g, v in zip(groups, vals):
    ok = [x for x in v if x is not None]
    if not ok: print(f'{g[0]:6s} missing'); continue
    s = f'{g[0]:6s} ' + ' '.join(f'{x:5.1f}' if x is not None else '  -  ' for x in v) + f' | mean {sum(ok)/len(ok):5.1f} [{min(ok):.1f}-{max(ok):.1f}]'
    if v is not ref:
        d = [a - b for a, b in zip(v, ref) if a is not None and b is not None]
        if d: s += f' | paired gain {sum(d)/len(d):+5.1f} (' + ' '.join(f'{x:+.1f}' for x in d) + f') {"> spread" if sum(d)/len(d) > spread else "<= spread"}'
    print(s)
