"""Moving-pedestrian funnel on CODa windows: for each annotated pedestrian (visible, 1.2-25 m), is there within 1 m
(a) a detector cluster, (b) a tracker track (and its state), (c) an output obstacle. usage: <run> <prefix> [...]"""
import sys
sys.path.insert(0, '/home/duccio/Desktop/jo_sim_sins/eval')
import collections, numpy as np, pandas as pd
from pathlib import Path
from rosbags.highlevel import AnyReader
from harness.ros_io import typestore, load_obstacles, load_odometry
from harness.tracks import objects

def det3d(run, tp):
    out = collections.defaultdict(list)
    with AnyReader([Path(run)], default_typestore=typestore()) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == tp]):
            m = r.deserialize(raw, c.msgtype); t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            for d in m.detections:
                p, s = d.bbox.center.position, d.bbox.size
                st = d.results[0].hypothesis.class_id if d.results else ''
                out[t].append((p.x, p.y, max(s.x, s.y), s.z, st, d.id, d.results[0].hypothesis.score if d.results else 0, [h.hypothesis.class_id for h in d.results]))
    return out

def nearest(store, ts, t):
    k = ts[np.argmin(np.abs(ts - t))]
    return store[k] if abs(k - t) < 0.06e9 else []

spd0, spd3, info3 = [], [], []
stage = collections.Counter(); sts = collections.Counter(); dsz = []
for run, pre in zip(sys.argv[1::2], sys.argv[2::2]):
    odom = load_odometry(pre + '_odom')
    g = pd.read_csv(pre + '_gt.csv'); g = g[(g.track_id >= 0) & (g['class'] == 'Pedestrian')].sort_values('t_ns')
    sp = []
    for _, gg in g.groupby('track_id'):
        tt = gg.t_ns.to_numpy() / 1e9; x, y = gg.x.to_numpy(), gg.y.to_numpy()
        v = np.array([np.hypot(np.interp(a + .5, tt, x) - np.interp(a - .5, tt, x), np.interp(a + .5, tt, y) - np.interp(a - .5, tt, y)) / (min(a + .5, tt[-1]) - max(a - .5, tt[0]) + 1e-9) for a in tt])
        sp.append(pd.Series(v, index=gg.index))
    g['spd'] = pd.concat(sp); g = g[g.npts >= 20]
    D = det3d(run, '/sins/dbscan_detector/fused/detections3d'); T = det3d(run, '/sins/tracker/tracks')
    Dt, Tt = np.array(sorted(D)), np.array(sorted(T))
    O = objects(load_obstacles(run)[0]); Ot = np.unique(O.t_ns)
    for _, q in g.iterrows():
        t = q.t_ns; rx, ry = np.interp(t / 1e9, odom[:, 0], odom[:, 1]), np.interp(t / 1e9, odom[:, 0], odom[:, 2])
        rr = np.hypot(q.x - rx, q.y - ry)
        if not 1.2 < rr < 25: continue
        o = O[O.t_ns == Ot[np.argmin(np.abs(Ot - t))]]
        if len(o) and np.min(np.hypot(o.x - q.x, o.y - q.y)) < 1.0: stage['0 output ok'] += 1; spd0.append(q.spd); continue
        tr = [x for x in nearest(T, Tt, t) if np.hypot(x[0] - q.x, x[1] - q.y) < 1.0]
        if tr:
            stage['3 track exists, not output'] += 1; sts[tr[0][4]] += 1; spd3.append(q.spd)
            tid = tr[0][5]; hist = [(tt, x) for tt in Tt[(Tt <= t) & (Tt > t - 3e9)] for x in T[tt] if x[5] == tid]
            age = len(hist); v = np.nan
            if age >= 6:
                a, b = hist[0], hist[-1]; v = np.hypot(b[1][0] - a[1][0], b[1][1] - a[1][1]) / ((b[0] - a[0]) / 1e9)
            info3.append(dict(age=age, v=v, kf=tr[0][6], sz=tr[0][2], hz=tr[0][3], gv=q.spd, res=tuple(tr[0][7]), fov=q.in_fov, r=rr))
            continue
        de = [x for x in nearest(D, Dt, t) if np.hypot(x[0] - q.x, x[1] - q.y) < 1.0]
        if de: stage['2 cluster exists, no track'] += 1; dsz.append(de[0][2:4]); continue
        de2 = [x for x in nearest(D, Dt, t) if np.hypot(x[0] - q.x, x[1] - q.y) < 2.0]
        stage['1b cluster 1-2 m away (merged/offset)' if de2 else '1a no cluster within 2 m'] += 1
        if de2: dsz.append(de2[0][2:4])
n = sum(stage.values())
for k in sorted(stage): print(f'{k:42s} {stage[k] / n:6.1%}')
print('state of the unpublished tracks:', dict(sts.most_common()))
if dsz: a = np.array(dsz); print('size of the cluster near a missed person: footprint median %.2f m, height %.2f m' % tuple(np.median(a, 0)))
b = [0, .2, .5, 1, 1.5, 9]
print('GT speed of output-ok rows   :', pd.cut(pd.Series(spd0), b).value_counts(normalize=True).sort_index().round(2).to_dict())
print('GT speed of static-track rows:', pd.cut(pd.Series(spd3), b).value_counts(normalize=True).sort_index().round(2).to_dict())
I = pd.DataFrame(info3)
print('static tracks on walking people (GT > 0.5 m/s):'); J = I[I.gv > 0.5]
print('  ticks of the same track id in the last 3 s:', pd.cut(J.age, [0, 5, 10, 20, 31]).value_counts(normalize=True).sort_index().round(2).to_dict())
print('  track own speed (>= 6 ticks old):', pd.cut(J.v.dropna(), [0, .1, .25, .5, 1, 9]).value_counts(normalize=True).sort_index().round(2).to_dict())
print('  results field:', J.res.value_counts().head(5).to_dict(), ' in fov:', J.fov.mean().round(2))
print('  KF speed of these tracks:', pd.cut(J.kf, [-.01, .07, .25, .5, 1, 9]).value_counts(normalize=True).sort_index().round(2).to_dict())
print('  footprint:', pd.cut(J.sz, [0, .5, 1.3, 2, 9]).value_counts(normalize=True).sort_index().round(2).to_dict(), ' height:', pd.cut(J.hz, [0, .7, 1.2, 2.7, 9]).value_counts(normalize=True).sort_index().round(2).to_dict())
