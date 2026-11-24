#!/usr/bin/env python3
"""Held walking persons on young tracks (24/11): was there a dynamic donor the heir should have inherited from? (HDBG debug CSV)
Heirs: held rows (held_rows_<TAG>.csv from held_static_veto.py) with motion_ev = 1, dyn_ticks < 5. Per heir track (first such row):
born < 1 s before -> young. Donor candidates: other tracks born before the heir, ever published dynamic (status dynamic or
coasting_pub) or not, whose last matched tick is within 2 s before the heir's birth. The nearest one to the heir's birth position
(KF position of the donor at the heir's birth tick if it still exists, else its last row) is the donor. Reports: donor age at
birth (birth - donor's last matched time), distance to the donor's KF prediction and to its last observation (ox, oy), the
class: within 0.6 m / 0.5 s, within 1.5 m / 1.0 s, farther, none; donor ever published dynamic, donor dyn_ticks max.
usage: held_donors.py [TAG]"""
import sys, collections
from pathlib import Path
import numpy as np, pandas as pd
TAG = sys.argv[1] if len(sys.argv) > 1 else 'HDBG'
E = Path(__file__).resolve().parent; B = E.parents[1] / 'bags/coda'
H = pd.read_csv(E / f'held_rows_{TAG}.csv')
H = H[(H.motion_ev == 1) & (H.dyn_ticks < 5)]
print(f'{TAG}: held rows with motion_ev=1 and dyn_ticks<5: {len(H)}; on tracks younger than 1 s: {100 * (H.age < 1.0).mean():.0f}% '
      f'(< 2 s {100 * (H.age < 2.0).mean():.0f}%)')
C = collections.Counter(); ages, dk, dl, dmax = [], [], [], []
for w, Hw in H.groupby('w'):
    c = w if '_' in w else f'0_{w}'
    d = pd.read_csv(B / 'runs' / f'{TAG}_trk_{c}.csv'); d['tn'] = (d.t * 1e9).round().astype('int64')
    info = {}
    for tid, q in d.groupby('id'):
        m = q[q.matched == 1]
        info[tid] = dict(birth=q.tn.min(), q=q, lastm=m.tn.max() if len(m) else None, lastobs=(m.ox.iloc[-1], m.oy.iloc[-1]) if len(m) else None,
                         pubdyn=bool(q.status.isin(['dynamic', 'coasting_pub']).any()), dynmax=int(q.dyn_ticks.max()), end=q.tn.max())
    for tid in Hw[Hw.age < 1.0].tid.unique():
        h = info[tid]; b = h['birth']; q0 = h['q'].iloc[0]; bx, by = q0.x, q0.y
        C['young heir tracks'] += 1
        best = None
        for oid, o in info.items():
            if oid == tid or o['birth'] >= b or o['lastm'] is None or not (b - 2e9 <= o['lastm'] <= b): continue
            qo = o['q']; at = qo[qo.tn <= b]
            if not len(at): continue
            kx, ky = at.x.iloc[-1], at.y.iloc[-1]
            dist = np.hypot(kx - bx, ky - by)
            if dist > 3.0: continue
            if best is None or dist < best[0]: best = (dist, oid, (b - o['lastm']) / 1e9, np.hypot(o['lastobs'][0] - bx, o['lastobs'][1] - by))
        if best is None: C['no donor (none within 3 m, last matched within 2 s)'] += 1; continue
        dist, oid, age, dlo = best; o = info[oid]
        ages.append(age); dk.append(dist); dl.append(dlo); dmax.append(o['dynmax'])
        cat = 'donor <= 0.6 m & <= 0.5 s' if dist <= 0.6 and age <= 0.5 else 'donor <= 1.5 m & <= 1.0 s' if dist <= 1.5 and age <= 1.0 else 'donor farther'
        C[cat] += 1; C[cat + (' | donor ever published dynamic' if o['pubdyn'] else ' | donor NEVER published dynamic')] += 1
        if cat.startswith('donor <= 0.6'): C['  near donor still alive at birth (unmatched/coasting)' if o['end'] > b else '  near donor already deleted at birth'] += 1
for k in sorted(C): print(f'  {k}: {C[k]}')
if ages:
    p = lambda v: f'p50 {np.percentile(v, 50):.2f} / p90 {np.percentile(v, 90):.2f}'
    print(f'  donor age at heir birth (s since last match) {p(ages)}; distance to donor KF position {p(dk)} m; to donor last observation {p(dl)} m; '
          f'donor max dyn_ticks {p(dmax)}')
