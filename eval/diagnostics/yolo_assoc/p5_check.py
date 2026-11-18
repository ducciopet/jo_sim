#!/usr/bin/env python3
"""P5 (track_vehicle_extent_prior) bug check, 18/11: for the GT vehicles matched to a published obstacle within 3 m,
compare two runs (reference vs P5 on) on the same input:
 - distance of the published centre from the GT centre;
 - radial error: (pred - GT) projected on the unit ray robot -> GT (negative = centre short, towards the sensor);
 - GT view: end-on (GT heading within 45 deg of the ray) or side-on; published extent along the ray vs GT extent along the ray.
P5 should move the centre AWAY from the sensor (radial error from negative towards 0) and set the extent along the ray to
the vehicle length when end-on, the width when side-on.
usage: p5_check.py <odom bag> <vehicle gt csv> <run A> <run B> [range m]"""
import sys
import numpy as np, pandas as pd
sys.path.insert(0, 'eval')
from harness.ros_io import load_obstacles, load_odometry

odom_bag, gt_csv, runs = sys.argv[1], sys.argv[2], sys.argv[3:5]
RNG = float(sys.argv[5]) if len(sys.argv) > 5 else 20.0
od = load_odometry(odom_bag)
g = pd.read_csv(gt_csv, keep_default_na=False)
g = g[g.track_id.astype(int) >= 0].copy()
for c in 'x y yaw l w'.split():
    g[c] = g[c].astype(float)
g['t'] = g.t_ns.astype('int64')
rx = np.interp(g.t / 1e9, od[:, 0], od[:, 1]); ry = np.interp(g.t / 1e9, od[:, 0], od[:, 2])
ray = np.stack([g.x - rx, g.y - ry], 1); rng = np.linalg.norm(ray, axis=1); ray /= rng[:, None]
g['rng'] = rng; g['rx'], g['ry'] = ray[:, 0], ray[:, 1]
hd = np.stack([np.cos(g['yaw']), np.sin(g['yaw'])], 1)
c = np.abs((hd * ray).sum(1))
g['endon'] = c >= np.sqrt(0.5)
g['gt_ray_ext'] = np.where(g.endon, g.l, g.w)
g = g[g.rng <= RNG]

def match(run):
    p, _, _ = load_obstacles(run)
    p = p[p.track_id >= 0]
    ts = np.unique(p.t_ns.to_numpy())
    out = []
    for t, gg in g.groupby('t'):
        k = np.searchsorted(ts, t); k = min(max(k, 1), len(ts) - 1)
        tp = ts[k] if abs(ts[k] - t) < abs(ts[k - 1] - t) else ts[k - 1]
        if abs(tp - t) > 0.06e9: continue
        pp = p[p.t_ns == tp]
        P = pp[['x', 'y']].to_numpy(float)
        for _, r in gg.iterrows():
            d = np.hypot(P[:, 0] - r.x, P[:, 1] - r.y)
            if not len(d) or d.min() > 3.0: continue
            j = d.argmin(); q = pp.iloc[j]
            e = np.array([q.x - r.x, q.y - r.y])
            ph = np.array([np.cos(q.yaw), np.sin(q.yaw)]); rr = np.array([r.rx, r.ry])
            xon = abs(ph @ rr) >= np.sqrt(0.5)
            out.append(dict(key=(t, r.track_id), d=d[j], rad=e @ rr, endon=r.endon, gt_ext=r.gt_ray_ext,
                            pr_ext=q.l if xon else q.w, yawdiff=np.degrees(abs(np.arctan2(np.sin(q.yaw - r.yaw), np.cos(q.yaw - r.yaw)))),
                            rng=r.rng))
    return pd.DataFrame(out)

A, B = match(runs[0]), match(runs[1])
print(f'GT vehicle rows within {RNG:.0f} m: {len(g)} (end-on {int(g.endon.sum())})')
for n, D in (('A ' + runs[0].split('/')[-1], A), ('B ' + runs[1].split('/')[-1], B)):
    within1 = (D.d <= 1.0).mean() * 100
    print(f'{n}: matched<3m {len(D)}, <1m {within1:.0f}%, dist med {D.d.median():.2f} | radial err med {D.rad.median():+.2f} '
          f'(end-on {D[D.endon].rad.median():+.2f}, side-on {D[~D.endon].rad.median():+.2f}) | ray extent pub/GT med '
          f'end-on {D[D.endon].pr_ext.median():.2f}/{D[D.endon].gt_ext.median():.2f} side-on {D[~D.endon].pr_ext.median():.2f}/'
          f'{D[~D.endon].gt_ext.median():.2f} | yaw diff med {D.yawdiff.median():.0f} deg')
M = A.merge(B, on='key', suffixes=('_a', '_b'))
if len(M):
    sh = M.rad_b - M.rad_a
    print(f'paired {len(M)}: radial shift B-A med {sh.median():+.2f} m (away from sensor {100 * (sh > 0.05).mean():.0f}%, '
          f'towards {100 * (sh < -0.05).mean():.0f}%) | dist B-A med {(M.d_b - M.d_a).median():+.2f}; B worse by >0.3 m on '
          f'{100 * (M.d_b - M.d_a > 0.3).mean():.0f}% | <1m A {100 * (M.d_a <= 1).mean():.0f}% B {100 * (M.d_b <= 1).mean():.0f}%')
    for lab, s in (('end-on', M.endon_a), ('side-on', ~M.endon_a)):
        X = M[s]
        if len(X):
            print(f'  {lab:7s} n {len(X)}: rad A {X.rad_a.median():+.2f} B {X.rad_b.median():+.2f} | ray ext A {X.pr_ext_a.median():.2f} '
                  f'B {X.pr_ext_b.median():.2f} GT {X.gt_ext_a.median():.2f} | <1m A {100 * (X.d_a <= 1).mean():.0f}% B {100 * (X.d_b <= 1).mean():.0f}%')
