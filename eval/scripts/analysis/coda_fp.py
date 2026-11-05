"""CODa: dynamic-labelled outputs far (> 1.5 m) from every GT box at annotated frames, grouped by output id.
usage: coda_fp.py <tag> <window> ...   (window like 0_w3)"""
import sys, numpy as np, pandas as pd
from pathlib import Path
from rosbags.highlevel import AnyReader
tags, wins = sys.argv[1].split(','), sys.argv[2:]
for tag in tags:
  rows = []
  for w in wins:
    g = pd.read_csv(f'bags/coda/coda{w}_gt.csv'); ts = np.unique(g.t_ns)
    O, P = {}, {}
    with AnyReader([Path(f'bags/coda/runs/{tag}_v2_coda{w}')]) as r:
        for cc, t, raw in r.messages([c for c in r.connections if c.topic in ('/perception/tracked_dynamic_obstacles', '/odometry/filtered')]):
            m = r.deserialize(raw, cc.msgtype); st = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if cc.topic == '/odometry/filtered': P[st] = (m.pose.pose.position.x, m.pose.pose.position.y)
            else: O[st] = [(o.track_id, o.pose.position.x, o.pose.position.y, o.size.x, o.size.y, o.size.z, o.status, np.hypot(o.twist.linear.x, o.twist.linear.y), o.track_age) for o in m.obstacles]
    ot = np.array(sorted(O)); pt = np.array(sorted(P))
    for t in ts:
        k = np.argmin(np.abs(ot - t)); 
        if abs(ot[k] - t) > 60e6: continue
        rp = P[pt[np.argmin(np.abs(pt - t))]]
        G = g[g.t_ns == t][['x', 'y']].values
        for (i, x, y, sx, sy, sz, s, v, cl) in O[ot[k]]:
            if s not in (1, 3): continue
            rng = np.hypot(x - rp[0], y - rp[1])
            if rng > 25 or rng < 1.2: continue
            d = np.min(np.hypot(G[:, 0] - x, G[:, 1] - y)) if len(G) else 99
            if d > 1.5: rows.append(dict(w=w, id=i, t=t, x=x, y=y, fp=max(sx, sy), sz=sz, rng=rng, spd=v, age=cl))
  R = pd.DataFrame(rows)
  if R.empty: print(tag, 'no FP'); continue
  T = R.groupby(['w', 'id']).agg(n=('t', 'size'), fp=('fp', 'median'), h=('sz', 'median'), rng=('rng', 'median'), spd=('spd', 'median'), age=('age', 'min'),
                                  travel=('x', lambda s: 0)).reset_index()
  # net travel of each FP output over its FP frames
  tr = R.sort_values('t').groupby(['w', 'id']).apply(lambda d: np.hypot(d.x.iloc[-1] - d.x.iloc[0], d.y.iloc[-1] - d.y.iloc[0]), include_groups=False)
  T['travel'] = tr.values
  print(f'== {tag}: FP rows {len(R)}  outputs {len(T)}')
  print('  by footprint  :', R.groupby(pd.cut(R.fp, [0, .8, 1.5, 3, 99]), observed=True).size().to_dict())
  print('  by range      :', R.groupby(pd.cut(R.rng, [0, 5, 10, 15, 25]), observed=True).size().to_dict())
  print('  KF speed median %.2f m/s' % R.spd.median())
  print('  FP outputs with net travel < 0.5 m (static thing called dynamic): rows %d of %d' % (T[T.travel < .5].n.sum(), T.n.sum()))
  print(T.sort_values('n', ascending=False).head(8).round(2).to_string(index=False))
