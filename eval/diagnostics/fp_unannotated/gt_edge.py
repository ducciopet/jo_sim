#!/usr/bin/env python3
"""Split fp_funnel.py's moving FP rows by whether a RAW CODa annotation (any class, any state, the window's gt.csv) lies
within 1.5 m at the nearest annotated frame (<= 60 ms). build_gt -> gt_tracks drops every annotation whose speed is
undefined (first/last ~0.5 s of each annotated track, half_window 0.5 s, and gaps > 0.3 s) but keeps its frame on the
grid, so a prediction on that object is scored FP. usage: gt_edge.py fprows_TAG.csv  -> prints, writes fprows_TAG_edge.csv"""
import sys
import numpy as np, pandas as pd
from pathlib import Path
R = Path(__file__).resolve().parents[3]
F = pd.read_csv(sys.argv[1], keep_default_na=False)
F['raw_gt'] = False
for w, g in F.groupby('w'):
    raw = pd.read_csv(R / f'bags/coda/coda{w}_gt.csv', keep_default_na=False)
    raw = raw[raw.track_id.astype(int) >= 0]
    xy = {t: q[['x', 'y']].to_numpy(float) for t, q in raw.groupby(raw.t_ns.astype('int64'))}
    ts = np.array(sorted(xy))
    for i, r in g.iterrows():
        k = int(np.argmin(np.abs(ts - r.t * 1e9)))
        if abs(ts[k] - r.t * 1e9) < 0.06e9:
            p = xy[ts[k]]
            F.at[i, 'raw_gt'] = bool((np.hypot(p[:, 0] - r.x, p[:, 1] - r.y) < 1.5).any())
F['in20'] = F.rng.astype(float) <= 20
print('moving FP rows (3 reps):', len(F), '| within 20 m:', F.in20.sum())
print('share with a raw annotation within 1.5 m (dropped edge GT), by bucket, within 20 m:')
q = F[F.in20]
print(q.groupby('b').agg(rows=('raw_gt', 'size'), raw_gt=('raw_gt', 'sum')).assign(
    pct=lambda d: (100 * d.raw_gt / d.rows).round(1)).to_string())
print('overall within 20 m: %d / %d = %.1f%%' % (q.raw_gt.sum(), len(q), 100 * q.raw_gt.mean()))
F.to_csv(sys.argv[1].replace('.csv', '_edge.csv'), index=False)
