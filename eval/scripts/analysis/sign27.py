#!/usr/bin/env python3
"""Sign test for adoption (PIPELINE_FINAL v2, section 2): 9 CODa windows x 3 reps = 27 paired error counts (FP+FN+IDSW),
arm rep k vs reference rep k on the same window. Adopt if the arm wins >= 19/27 (ties count as not won) on both testing and all.
usage: sign27.py <dir: coda_e3[/test] or coda_e3ib[/test]> REF,REFr2,REFr3 ARM,ARMr2,ARMr3"""
import json, sys
from math import comb
from pathlib import Path
D = Path(sys.argv[1]); WS = ['w1', 'w2', 'w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']
ref, arm = sys.argv[2].split(','), sys.argv[3].split(',')
def err(tag, w):
    m = json.load(open(D / f'{tag}_{w}.json'))['metrics']['full']
    return m['fp'] + m['fn'] + m['idsw']
win = loss = tie = 0; per = []
for w in WS:
    s = ''
    for r, a in zip(ref, arm):
        d = err(a, w) - err(r, w)
        win += d < 0; loss += d > 0; tie += d == 0; s += f' {d:+d}'
    per.append(f'{w}:{s}')
n = win + loss + tie
p = sum(comb(n, k) for k in range(win, n + 1)) / 2 ** n
print(f'{D}: {arm[0]} vs {ref[0]}: wins {win}/{n} (losses {loss}, ties {tie}), one-sided p {p:.3f} -> {"ADOPT" if win >= 19 else "no"}')
print('  error delta arm-ref per window (r1 r2 r3): ' + ' | '.join(per))
