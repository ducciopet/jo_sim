#!/usr/bin/env python3
"""Vehicle GT rows not found at 1 m (24/11, laptop request): which tracker gate holds them. Inputs: HDBG run (EB + track_debug_csv),
bags/coda/runs/<TAG>_trk_<c>.csv and the published output of <TAG>_v2_coda<c>. Same e3 GT/filtering as coda_eval.
Per annotated frame, Hungarian at 1 m between all GT objects and published obstacles. For each vehicle GT row NOT matched:
the debug-track rows at the same tick (<= 60 ms) whose centre lies inside the GT box (+0.3 m):
  none        no debug track inside the box
  else, per GT row the track nearest the GT centre: its status, and its flags (confirmed, hits, size_ok, cls, fov, left_ok,
  cons_ok, kf_conf, vote, disocc, speed_ok, large_travel, motion_ev, dyn_ticks, confirm_ticks, sx/sy/sz), plus the number of
  debug tracks inside the box (fragments).
Split: GT length >= 6 m vs < 6 m; bearing within 35 deg of the robot heading (front camera) or not; GT moving/still.
usage: veh_gates.py [TAG] [windows ...]"""
import argparse, sys, collections
from pathlib import Path
import numpy as np, pandas as pd
from scipy.optimize import linear_sum_assignment
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import coda_eval as ce   # noqa: E402

TAG = sys.argv[1] if len(sys.argv) > 1 else 'HDBG'
WS = sys.argv[2:] or ['10_w1', '10_w4', '10_w5']
VEH = ['Car', 'Pickup Truck', 'Utility Vehicle', 'Service Vehicle', 'Delivery Truck', 'Bus', 'Truck', 'Van']
B = Path(__file__).resolve().parents[2] / 'bags/coda'
ns = argparse.Namespace(range=20.0, ignore_dist=1.0, min_range=1.2, max_height=2.5, min_pts=1, pd_everywhere=False, camera_only=False,
                        cam_json=None, ignore_undefined=True, dyn_speed=0.3, match_dist=1.0, points_min_cluster=20, extra_ignore=None,
                        extra_ignore_dist=0.3, extra_ignore_dt=0.5, frames='all', coda_root='', ignore_gt=None, topic=None,
                        method='sins', ignore_box=False)
FL = ['confirmed', 'size_ok', 'fov', 'left_ok', 'cons_ok', 'kf_conf', 'vote', 'disocc', 'speed_ok', 'large_travel', 'motion_ev']
rows = []
for w in WS:
    c = w if '_' in w else f'0_{w}'
    d = pd.read_csv(B / 'runs' / f'{TAG}_trk_{c}.csv'); d['tn'] = (d.t * 1e9).round().astype('int64')
    dts = np.unique(d.tn.to_numpy())
    odom = ce.load_odometry(B / f'coda{c}_odom')
    gt, ign, frames, _ = ce.build_gt(str(B / f'coda{c}_gt.csv'), odom, ns)
    raw = pd.read_csv(B / f'coda{c}_gt.csv'); g1 = raw.drop_duplicates('track_id').set_index('track_id')
    dims = raw.groupby('track_id')[['l', 'w']].median()
    gt = gt.assign(cls=gt.track_id.map(g1['class']), l=gt.track_id.map(dims.l).fillna(0.5), w=gt.track_id.map(dims.w).fillna(0.5),
                   yaw=gt.yaw.fillna(0.0))
    pred, _, _ = ce.load_obstacles(str(B / 'runs' / f'{TAG}_v2_coda{c}'), None)
    pred, _, _ = ce.filter_pred(pred, ign, frames, odom, ce.odom_ground_z(B / f'coda{c}_odom'), ns)
    pred = pred[pred.track_id >= 0]; pt = np.unique(pred.t_ns.to_numpy())
    for t, G in gt[gt.track_id >= 0].groupby('t_ns'):
        if not G.cls.isin(VEH).any(): continue
        k = pt[np.argmin(np.abs(pt - t))]; P = pred[pred.t_ns == k] if abs(k - t) < 0.06e9 else pred.iloc[:0]
        G = G.reset_index(drop=True); m = set()
        if len(P):
            Dm = np.hypot(P.x.to_numpy()[:, None] - G.x.to_numpy()[None], P.y.to_numpy()[:, None] - G.y.to_numpy()[None])
            r_, c_ = linear_sum_assignment(np.where(Dm <= 1.0, Dm, 1e6)); m = {j for i, j in zip(r_, c_) if Dm[i, j] <= 1.0}
        kd = dts[np.argmin(np.abs(dts - t))]; Dd = d[d.tn == kd] if abs(kd - t) < 0.06e9 else d.iloc[:0]
        ti = np.argmin(np.abs(odom[:, 0] - t / 1e9)); rx, ry, ryaw = odom[ti, 1:4]
        for j, g in G.iterrows():
            if g.cls not in VEH: continue
            b = np.degrees(np.arctan2(g.y - ry, g.x - rx) - ryaw); b = (b + 180) % 360 - 180
            r = dict(w=w, cls=g.cls, L='L>=6' if g.l >= 6 else 'L<6', fovg='front' if abs(b) <= 35 else 'side/back',
                     mov='moving' if g.state == 'dynamic' else 'still', tp=j in m)
            if not r['tp'] and len(Dd):
                cs, sn = np.cos(g.yaw), np.sin(g.yaw); dx, dy = Dd.x - g.x, Dd.y - g.y
                ins = (np.abs(dx * cs + dy * sn) <= g.l / 2 + 0.3) & (np.abs(-dx * sn + dy * cs) <= g.w / 2 + 0.3)
                I = Dd[ins.to_numpy()]
                r['nfrag'] = len(I)
                if len(I):
                    q = I.iloc[int(np.argmin(np.hypot(I.x - g.x, I.y - g.y)))]
                    r.update(status=q.status, hits=q.hits, dyn_ticks=q.dyn_ticks, confirm_ticks=q.confirm_ticks, tcls=q.cls,
                             sx=q.sx, sy=q.sy, sz=q.sz, speed=q.speed, **{f: q[f] for f in FL})
                else: r['status'] = 'none'
            rows.append(r)
R = pd.DataFrame(rows)
print(f'{TAG} windows {WS}: vehicle GT rows {len(R)}, TP {100 * R.tp.mean():.1f}%')
U = R[~R.tp]
for grp, S in [('all', U)] + [(f'{k}={v}', U[U[k] == v]) for k in ('L', 'fovg', 'mov') for v in sorted(U[k].unique())]:
    if not len(S): continue
    print(f'\n== not found, {grp}: {len(S)} rows | status of the nearest debug track inside the box: ' +
          ', '.join(f'{s} {100 * n / len(S):.0f}%' for s, n in S.status.value_counts().items()) +
          f' | fragments inside box mean {S.nfrag.mean():.2f}')
    for st in S.status.value_counts().index:
        X = S[S.status == st]
        if st == 'none' or len(X) < 5: continue
        print(f'   {st:20s} n {len(X):4d} | ' + ', '.join(f'{f}=0 {100 * (X[f] == 0).mean():.0f}%' for f in FL) +
              f' | disocc=1 {100 * (X.disocc == 1).mean():.0f}% | hits med {X.hits.median():.0f}, dyn_ticks med {X.dyn_ticks.median():.0f}, '
              f'confirm_ticks med {X.confirm_ticks.median():.0f} | class: ' + ', '.join(f'{k} {v}' for k, v in X.tcls.value_counts().head(3).items()) +
              f' | size med {X.sx.median():.1f}x{X.sy.median():.1f}x{X.sz.median():.1f} | speed med {X.speed.median():.2f}')
