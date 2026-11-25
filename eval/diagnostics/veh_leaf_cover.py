#!/usr/bin/env python3
"""Missed vehicles (25/11, chain H prep): inside the GT box (+0.3 m) of each vehicle GT row NOT matched at 1 m, the published
fused detections (/sins/dbscan_detector/fused/detections3d) and their class: does a YOLO-classified vehicle piece exist next to
unclassified pieces (results[1] = YOLO class; leaf present, fragments outside it) or are all pieces unclassified (no leaf)? Also the extent of the
classified piece vs the GT length. Same e3 GT/filters as coda_eval. usage: veh_leaf_cover.py TAG [windows ...]"""
import argparse, sys, collections
from pathlib import Path
import numpy as np, pandas as pd
from scipy.optimize import linear_sum_assignment
from rosbags.highlevel import AnyReader
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import coda_eval as ce   # noqa: E402
TAG = sys.argv[1]; WS = sys.argv[2:] or ['10_w1', '10_w4', '10_w5']
VEH = ['Car', 'Pickup Truck', 'Utility Vehicle', 'Service Vehicle', 'Delivery Truck', 'Bus', 'Truck', 'Van']
VC = {'car', 'truck', 'bus', 'motorbike'}
B = Path(__file__).resolve().parents[2] / 'bags/coda'
ns = argparse.Namespace(range=20.0, ignore_dist=1.0, min_range=1.2, max_height=2.5, min_pts=1, pd_everywhere=False, camera_only=False,
                        cam_json=None, ignore_undefined=True, dyn_speed=0.3, match_dist=1.0, points_min_cluster=20, extra_ignore=None,
                        extra_ignore_dist=0.3, extra_ignore_dt=0.5, frames='all', coda_root='', ignore_gt=None, topic=None,
                        method='sins', ignore_box=False)
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
rows = []
for w in WS:
    c = w if '_' in w else f'0_{w}'; run = B / 'runs' / f'{TAG}_v2_coda{c}'
    det = {}
    with AnyReader([run]) as r:
        for cn, _, raw in r.messages([x for x in r.connections if x.topic == '/sins/dbscan_detector/fused/detections3d']):
            m = r.deserialize(raw, cn.msgtype)
            det[st(m.header)] = [(d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size.x, d.bbox.size.y,
                                  d.results[1].hypothesis.class_id if len(d.results) > 1 else '') for d in m.detections]
    dt = np.array(sorted(det))
    odom = ce.load_odometry(B / f'coda{c}_odom'); gt, ign, frames, _ = ce.build_gt(str(B / f'coda{c}_gt.csv'), odom, ns)
    raw = pd.read_csv(B / f'coda{c}_gt.csv'); g1 = raw.drop_duplicates('track_id').set_index('track_id'); dims = raw.groupby('track_id')[['l', 'w']].median()
    gt = gt.assign(cls=gt.track_id.map(g1['class']), l=gt.track_id.map(dims.l).fillna(0.5), w=gt.track_id.map(dims.w).fillna(0.5), yaw=gt.yaw.fillna(0.0))
    pred, _, _ = ce.load_obstacles(str(run), None); pred, _, _ = ce.filter_pred(pred, ign, frames, odom, ce.odom_ground_z(B / f'coda{c}_odom'), ns)
    pred = pred[pred.track_id >= 0]; pt = np.unique(pred.t_ns.to_numpy())
    for t, G in gt[gt.track_id >= 0].groupby('t_ns'):
        if not G.cls.isin(VEH).any(): continue
        k = pt[np.argmin(np.abs(pt - t))]; P = pred[pred.t_ns == k] if abs(k - t) < 0.06e9 else pred.iloc[:0]
        G = G.reset_index(drop=True); m = set()
        if len(P):
            Dm = np.hypot(P.x.to_numpy()[:, None] - G.x.to_numpy()[None], P.y.to_numpy()[:, None] - G.y.to_numpy()[None])
            r_, c_ = linear_sum_assignment(np.where(Dm <= 1.0, Dm, 1e6)); m = {j for i, j in zip(r_, c_) if Dm[i, j] <= 1.0}
        D = det[dt[np.argmin(np.abs(dt - t))]]
        ti = np.argmin(np.abs(odom[:, 0] - t / 1e9)); rx, ry, ryaw = odom[ti, 1:4]
        for j, g in G.iterrows():
            if g.cls not in VEH or j in m: continue
            b = np.degrees(np.arctan2(g.y - ry, g.x - rx) - ryaw); b = (b + 180) % 360 - 180
            cs, sn = np.cos(g.yaw), np.sin(g.yaw)
            I = [d for d in D if abs((d[0] - g.x) * cs + (d[1] - g.y) * sn) <= g.l / 2 + 0.3 and abs(-(d[0] - g.x) * sn + (d[1] - g.y) * cs) <= g.w / 2 + 0.3]
            vc = [d for d in I if d[4] in VC]; oc = [d for d in I if d[4] and d[4] not in VC]
            kind = 'none' if not I else ('veh-classed + others' if vc and len(I) > len(vc) else 'veh-classed only' if vc else 'other-class' if oc else 'unclassified only')
            rows.append(dict(w=w, L='L>=6' if g.l >= 6 else 'L<6', fov='front' if abs(b) <= 35 else 'side/back', mov='moving' if g.state == 'dynamic' else 'still',
                             kind=kind, n=len(I), vlen=max([max(d[2], d[3]) for d in vc], default=np.nan), glen=g.l))
R = pd.DataFrame(rows)
print(f'{TAG} {WS}: missed vehicle GT rows {len(R)}')
for grp, S in [('all', R)] + [(f'{k}={v}', R[R[k] == v]) for k in ('fov', 'L', 'mov') for v in sorted(R[k].unique())]:
    print(f'{grp:16s} n {len(S):4d} | ' + ', '.join(f'{k} {100 * v / len(S):.0f}%' for k, v in S.kind.value_counts().items()) +
          f' | pieces med {S.n.median():.0f} | veh piece long side / GT length med {np.nanmedian(S.vlen / S.glen) if S.vlen.notna().any() else float("nan"):.2f}')
