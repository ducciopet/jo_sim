"""Cars on the street bag: what the clusters look like scan to scan (innovation a constant-velocity filter would see,
fragmentation, partial view), and whether the tracker gates would accept them. usage: car_clusters.py <run>"""
import sys, numpy as np, pandas as pd
from pathlib import Path
from rosbags.highlevel import AnyReader
run = sys.argv[1]
g = pd.read_csv('bags/mdp/mdp_street_gt.csv'); g = g[(g.track_id >= 0) & (g['class'] == 'vehicle') & (g.range <= 15) & (g.range >= 1.2)]
D, T = {}, {}
with AnyReader([Path(run)]) as r:
    for topic, S in (('/sins/dbscan_detector/fused/detections3d', D), ('/sins/tracker/tracks', T)):
        c = [c for c in r.connections if c.topic == topic]
        for cc, _, raw in r.messages(c):
            m = r.deserialize(raw, cc.msgtype); t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            S[t] = np.array([[d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size.x, d.bbox.size.y, int(d.id) if str(d.id).isdigit() else -1] for d in m.detections]).reshape(-1, 5)
ts = np.array(sorted(D)); dt = np.median(np.diff(ts)) / 1e9
rows = []
for tid, d in g.sort_values('t_ns').groupby('track_id'):
    prev = []
    for _, o in d.iterrows():
        cl = D.get(o.t_ns, np.zeros((0, 5)))
        # clusters whose centre lies inside the car box grown by 0.5 m
        c, s = np.cos(o.yaw), np.sin(o.yaw)
        if len(cl):
            lx = c * (cl[:, 0] - o.x) + s * (cl[:, 1] - o.y); ly = -s * (cl[:, 0] - o.x) + c * (cl[:, 1] - o.y)
            ins = (np.abs(lx) < o.l / 2 + .5) & (np.abs(ly) < o.w / 2 + .5)
        else: ins = np.zeros(0, bool)
        n = int(ins.sum())
        k = None
        if n: k = np.flatnonzero(ins)[np.argmax(cl[ins, 2] * cl[ins, 3])]   # biggest piece = what a track would follow
        tr = T.get(o.t_ns, np.zeros((0, 5)))
        trk = len(tr) and np.min(np.hypot(tr[:, 0] - o.x, tr[:, 1] - o.y)) < 2.5
        row = dict(car=tid, t=o.t_ns, fov=bool(o.in_fov), rng=o.range, L=o.l, n=n, trk=bool(trk))
        if k is not None:
            p = cl[k, :2]; row.update(off=np.hypot(*(p - [o.x, o.y])), ext=max(cl[k, 2], cl[k, 3]) / max(o.l, .1))
            if len(prev) >= 2 and prev[-1][0] == o.t_ns - 1 and prev[-2][0] == o.t_ns - 2:
                pass
            if len(prev) >= 2:
                (t1, p1), (t2, p2) = prev[-1], prev[-2]
                if (o.t_ns - t1) < 1.5 * dt * 1e9 and (t1 - t2) < 1.5 * dt * 1e9:
                    row['innov'] = np.hypot(*(p - (p1 + (p1 - p2) * (o.t_ns - t1) / (t1 - t2))))
                    row['step'] = np.hypot(*(p - p1)); row['spd'] = row['step'] / ((o.t_ns - t1) / 1e9)
            prev.append((o.t_ns, p))
        else:
            prev = []
        rows.append(row)
R = pd.DataFrame(rows)
R['rb'] = pd.cut(R.rng, [0, 5, 10, 15])
print('frames', len(R), ' cars', R.car.nunique(), ' scan dt %.3f s' % dt)
print('\nclusters per car per frame:', R.n.value_counts(normalize=True).sort_index().round(2).to_dict())
q = R.dropna(subset=['innov'])
print('\nCV innovation of the biggest piece (m): median %.2f  p75 %.2f  p90 %.2f' % tuple(np.percentile(q.innov, [50, 75, 90])))
for thr in (0.6, 1.0, 1.5, 2.0): print('  share > %.1f m: %.2f' % (thr, (q.innov > thr).mean()))
print('cluster speed (m/s) median %.1f p90 %.1f;  share > 8 m/s %.2f' % (np.median(q.spd), np.percentile(q.spd, 90), (q.spd > 8).mean()))
print('centroid offset from car centre: median %.2f m; cluster length / car length median %.2f' % (R.off.median(), R.ext.median()))
print('\nby range:'); print(q.groupby('rb', observed=True).agg(n=('innov', 'size'), innov_med=('innov', 'median'), gt06=('innov', lambda x: (x > .6).mean()), gt10=('innov', lambda x: (x > 1).mean()), trk=('trk', 'mean')).round(2).to_string())
print('\ntracked vs innovation bin:'); q = q.assign(ib=pd.cut(q.innov, [0, .3, .6, 1, 2, 50])); print(q.groupby('ib', observed=True).trk.agg(['size', 'mean']).round(2).to_string())
print('\ntracked vs pieces:'); print(R.groupby(R.n.clip(0, 3)).trk.agg(['size', 'mean']).round(2).to_string())
