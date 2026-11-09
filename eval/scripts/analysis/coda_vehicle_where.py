#!/usr/bin/env python3
"""CODa: where SINS loses the annotated vehicles (Car, Pickup Truck, Utility/Service Vehicle, Delivery Truck).
For every MOVING GT vehicle row (1.2-25 m), at the nearest output stamp: published obstacle within 1 m (and its status),
internal tracker track within 1 m but not published (and its state), published obstacle only 1-2.5 m off (box of a
fragment), or nothing. usage: coda_vehicle_where.py <TAG> <window>...   e.g. LFX 10_w4 10_w5 16_w1"""
import sys, collections, numpy as np, pandas as pd
from pathlib import Path
from rosbags.highlevel import AnyReader
VEH = ['Car', 'Pickup Truck', 'Utility Vehicle', 'Service Vehicle', 'Delivery Truck']
tag = sys.argv[1]


def moving(g):
    sp = pd.Series(np.nan, index=g.index)
    for _, d in g.sort_values('t_ns').groupby('track_id'):
        t = d.t_ns.to_numpy() / 1e9; x = d.x.to_numpy(); y = d.y.to_numpy()
        for i, ix in enumerate(d.index):
            j0 = np.searchsorted(t, t[i] - 1); j1 = np.searchsorted(t, t[i] + 1, 'right') - 1
            if t[j1] - t[j0] > 0.5:
                sp[ix] = np.hypot(x[j1] - x[j0], y[j1] - y[j0]) / (t[j1] - t[j0])
    return (sp > 0.3).to_numpy()

for w in sys.argv[2:]:
    for rep in ['', 'r2', 'r3']:
        run = Path(f'bags/coda/runs/{tag}{rep}_v2_coda{w}')
        if not run.exists():
            continue
        g = pd.read_csv(f'bags/coda/coda{w}_gt.csv'); g = g[g['class'].isin(VEH) & (g.range <= 25) & (g.range >= 1.2)]
        g = g[moving(g)]   # moving vehicles only (1 s centred speed > 0.3 m/s, as coda_eval)
        if g.empty:
            continue
        O, T = {}, {}
        with AnyReader([run]) as r:
            for cc, _, raw in r.messages([c for c in r.connections if c.topic == '/perception/tracked_dynamic_obstacles']):
                m = r.deserialize(raw, cc.msgtype); t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
                O[t] = [(o.pose.position.x, o.pose.position.y, o.status) for o in m.obstacles]
            for cc, _, raw in r.messages([c for c in r.connections if c.topic == '/sins/tracker/tracks']):
                m = r.deserialize(raw, cc.msgtype); t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
                T[t] = [(d.bbox.center.position.x, d.bbox.center.position.y, d.results[0].hypothesis.class_id if d.results else '?',
                         d.bbox.size.x, d.bbox.size.y) for d in m.detections]
        to, tt = np.array(sorted(O)), np.array(sorted(T)); C = collections.Counter(); sz = []
        for _, o in g.iterrows():
            Ov = O[to[np.argmin(abs(to - o.t_ns))]]; Tv = T[tt[np.argmin(abs(tt - o.t_ns))]]
            ob = [x for x in Ov if np.hypot(x[0] - o.x, x[1] - o.y) < 1.0]
            tk = [x for x in Tv if np.hypot(x[0] - o.x, x[1] - o.y) < 1.0]
            if ob: C[f'published status {ob[0][2]}'] += 1
            elif tk: C[f'internal only ({tk[0][2]})'] += 1; sz.append(tk[0][3:5])
            elif any(np.hypot(x[0] - o.x, x[1] - o.y) < 2.5 for x in Ov): C['published 1-2.5 m off'] += 1
            else: C['no track within 1 m'] += 1
        n = len(g)
        print(f'{w} {tag}{rep or ""}: {n} rows | ' + ', '.join(f'{k} {100*v/n:.0f}%' for k, v in C.most_common())
              + (f' | internal-only box {np.median(sz, axis=0).round(1)} m' if sz else ''))
