#!/usr/bin/env python3
"""Start-of-motion delay of the lab person (1947): for each GT PD->D transition, the person's track in the tracker
debug CSV (row nearest to the GT position, < 0.8 m, odom frame via the run's mocap alignment), ticks from the GT start
until the track is published 'dynamic', and per non-D tick the gates that were off
(speed_ok, kf_conf, vote, cons_ok, left_ok, motion_ev). usage: d_delay.py <csv> <alignment.json> <gt tracks.csv> <seq>"""
import sys, json
import numpy as np, pandas as pd
sys.path.insert(0, 'eval')
from harness.gt_mocap import compose, inverse
d = pd.read_csv(sys.argv[1]); al = json.load(open(sys.argv[2])); seq = sys.argv[4]
Ti = inverse(np.array(al['T_mocap_odom'][seq])); dl = al['delta'][seq]
g = pd.read_csv(sys.argv[3]); g = g[g.track_id >= 0].sort_values('t_ns'); t0 = g.t_ns.iloc[0]
st = g.state.to_numpy(); starts = [g.t_ns.iloc[i] for i in range(1, len(g)) if st[i] == 'dynamic' and st[i - 1] != 'dynamic']
G = ['speed_ok', 'kf_conf', 'vote', 'cons_ok', 'left_ok', 'motion_ev']
ticks = np.unique(d.t)
for s in starts:
    rows = []
    for t in ticks[(ticks * 1e9 + dl * 1e9 >= s) & (ticks * 1e9 + dl * 1e9 < s + 3e9)]:
        k = np.argmin(np.abs(g.t_ns.to_numpy() - (t + dl) * 1e9)); ox, oy, _ = compose(Ti, np.array([g.x.iloc[k], g.y.iloc[k], 0.0]))
        q = d[d.t == t]; dd = np.hypot(q.x - ox, q.y - oy)
        if not len(q) or dd.min() > 0.8: rows.append((round(t + dl - s / 1e9, 1), None, 'no track', [])); continue
        r = q.iloc[int(np.argmin(dd.to_numpy()))]
        rows.append((round(t + dl - s / 1e9, 1), int(r.id), r.status, [c for c in G if r[c] == 0]))
    first = next((i for i, r in enumerate(rows) if r[2] == 'dynamic'), None)
    blk = {}
    for r in rows[:first if first is not None else len(rows)]:
        for c in (r[3] or ['(status ' + r[2] + ')']): blk[c] = blk.get(c, 0) + 1
    print(f'start {(s - t0) / 1e9:.1f} s: ticks to D = {first if first is not None else ">30"}; gates off before D (ticks): {blk}')
# all GT-dynamic ticks: person's track status and the gates off when it is not published dynamic
tot = {}; n = 0; nd = 0
for t in ticks:
    k = np.argmin(np.abs(g.t_ns.to_numpy() - (t + dl) * 1e9))
    if abs(g.t_ns.iloc[k] - (t + dl) * 1e9) > 0.06e9 or g.state.iloc[k] != 'dynamic': continue
    n += 1
    ox, oy, _ = compose(Ti, np.array([g.x.iloc[k], g.y.iloc[k], 0.0]))
    q = d[d.t == t]; dd = np.hypot(q.x - ox, q.y - oy)
    if not len(q) or dd.min() > 0.8: tot['no track'] = tot.get('no track', 0) + 1; continue
    r = q.iloc[int(np.argmin(dd.to_numpy()))]
    if r.status == 'dynamic': nd += 1; continue
    key = r.status + ': ' + (','.join(c for c in G if r[c] == 0) or 'all gates on')
    tot[key] = tot.get(key, 0) + 1
print(f'GT dynamic ticks {n}, person track published D {nd}; the others: ' + str(dict(sorted(tot.items(), key=lambda x: -x[1]))))
