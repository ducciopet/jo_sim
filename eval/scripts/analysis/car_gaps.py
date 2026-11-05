"""Cars split into several clusters: gap between the pieces (box to box, xy and z) vs the DBSCAN eps at that range."""
import sys, numpy as np, pandas as pd
from pathlib import Path
from rosbags.highlevel import AnyReader
run = sys.argv[1]
g = pd.read_csv('bags/mdp/mdp_street_gt.csv'); g = g[(g.track_id >= 0) & (g['class'] == 'vehicle') & (g.range <= 15) & (g.range >= 1.2)]
D = {}
with AnyReader([Path(run)]) as r:
    c = [c for c in r.connections if c.topic == '/sins/dbscan_detector/fused/detections3d']
    for cc, _, raw in r.messages(c):
        m = r.deserialize(raw, cc.msgtype); t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        D[t] = np.array([[b.center.position.x, b.center.position.y, b.center.position.z, b.size.x, b.size.y, b.size.z] for b in (d.bbox for d in m.detections)]).reshape(-1, 6)
rows = []
for _, o in g.iterrows():
    cl = D.get(o.t_ns)
    if cl is None or not len(cl): continue
    c, s = np.cos(o.yaw), np.sin(o.yaw)
    lx = c * (cl[:, 0] - o.x) + s * (cl[:, 1] - o.y); ly = -s * (cl[:, 0] - o.x) + c * (cl[:, 1] - o.y)
    P = cl[(np.abs(lx) < o.l / 2 + .5) & (np.abs(ly) < o.w / 2 + .5)]
    if len(P) < 2: continue
    P = P[np.argsort(-P[:, 3] * P[:, 4])]
    gaps = []
    for b in P[1:]:   # gap of each extra piece to the biggest one (axis-aligned box separation)
        a = P[0]
        gx = max(0, abs(a[0] - b[0]) - (a[3] + b[3]) / 2); gy = max(0, abs(a[1] - b[1]) - (a[4] + b[4]) / 2); gz = max(0, abs(a[2] - b[2]) - (a[5] + b[5]) / 2)
        gaps.append((np.hypot(gx, gy), gz, b[3] * b[4]))
    k = int(np.argmin([x[0] + x[1] for x in gaps]))
    eps = 0.18 + (0.45 - 0.18) * min(o.range, 15) / 15
    rows.append(dict(rng=o.range, n=len(P), gxy=gaps[k][0], gz=gaps[k][1], area=gaps[k][2], eps=eps))
R = pd.DataFrame(rows); R['rb'] = pd.cut(R.rng, [0, 5, 10, 15])
R['touch'] = (R.gxy < .01) & (R.gz < .01)
print('split frames', len(R))
print('closest piece to the main one: xy gap median %.2f m, z gap median %.2f m' % (R.gxy.median(), R.gz.median()))
print('share whose boxes overlap/touch (gap 0 in xy and z): %.2f' % R.touch.mean())
print('share xy gap < eps (boxes already within eps, split by sparse points inside): %.2f' % (R.gxy < R.eps).mean())
print('share xy gap in [eps, 2 eps): %.2f   >= 2 eps: %.2f' % (((R.gxy >= R.eps) & (R.gxy < 2 * R.eps)).mean(), (R.gxy >= 2 * R.eps).mean()))
print(R.groupby('rb', observed=True).agg(n=('gxy', 'size'), gxy_med=('gxy', 'median'), gz_med=('gz', 'median'), eps=('eps', 'mean'), touch=('touch', 'mean'), piece_area_med=('area', 'median')).round(2).to_string())
