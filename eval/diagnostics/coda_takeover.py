#!/usr/bin/env python3
"""CODa person ID switches where a PRE-EXISTING track takes the person over (23/11, DM0 vs DC0). Same GT/filters/matching as
coda_person_funnel.py (e3, Hungarian 1 m on the published output). For each switch A -> B on a person GT track, with B first
published >= 0.5 s before the switch, B is described at the last tracker tick BEFORE it took the person (/sins/tracker/tracks):
  class (tracker class hypothesis or none), state, box size (max(l,w), h), age (since B first appears on the tracker topic),
  distance from B to the person's GT position at that tick; and the fused detection nearest to B then: its label (L / F / S
  combinations; F = YOLO-refined) and size ("big" = max(l,w) > 1.2 m: a merged/absorbed object rather than a person).
usage: coda_takeover.py --tags TAG [...] [--windows ...]"""
import argparse, sys, collections
from pathlib import Path
import numpy as np, pandas as pd
from scipy.optimize import linear_sum_assignment
from rosbags.highlevel import AnyReader
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import coda_eval as ce   # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--tags', nargs='+', required=True)
ap.add_argument('--windows', nargs='+', default='w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1'.split())
a = ap.parse_args()
B = Path(__file__).resolve().parents[2] / 'bags/coda'
ns = argparse.Namespace(range=20.0, ignore_dist=1.0, min_range=1.2, max_height=2.5, min_pts=1, pd_everywhere=False, camera_only=False,
                        cam_json=None, ignore_undefined=True, dyn_speed=0.3, match_dist=1.0, points_min_cluster=20, extra_ignore=None,
                        extra_ignore_dist=0.3, extra_ignore_dt=0.5, frames='all', coda_root='', ignore_gt=None, topic=None,
                        method='sins', ignore_box=False)
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
def bag(run):
    trk, det, born = {}, {}, {}
    with AnyReader([Path(run)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic in ('/sins/tracker/tracks', '/sins/dbscan_detector/fused/detections3d')]):
            m = r.deserialize(raw, c.msgtype); t = st(m.header); L = []
            for d in m.detections:
                b = d.bbox; h = [x.hypothesis.class_id for x in d.results]
                L.append((d.id, b.center.position.x, b.center.position.y, max(b.size.x, b.size.y), b.size.z, h[0] if h else '', h[1] if len(h) > 1 else ''))
                if 'tracker' in c.topic: born.setdefault(int(d.id.split('_')[1]), t)
            (trk if 'tracker' in c.topic else det)[t] = L
    return trk, det, born
for tag in a.tags:
    C = collections.Counter(); ages, dists = [], []; n = 0
    for w in a.windows:
        c = w if '_' in w else f'0_{w}'
        run = B / 'runs' / f'{tag}_v2_coda{c}'
        if not run.exists(): continue
        odom = ce.load_odometry(B / f'coda{c}_odom')
        gt, ign, frames, _ = ce.build_gt(str(B / f'coda{c}_gt.csv'), odom, ns)
        raw = pd.read_csv(B / f'coda{c}_gt.csv')
        gt = gt.assign(ped=gt.track_id.map(raw[['track_id', 'class']].drop_duplicates('track_id').set_index('track_id')['class']) == 'Pedestrian')
        pred, _, _ = ce.load_obstacles(str(run), None)
        pred, _, _ = ce.filter_pred(pred, ign, frames, odom, ce.odom_ground_z(B / f'coda{c}_odom'), ns)
        pred = pred[pred.track_id >= 0]; pt = np.unique(pred.t_ns.to_numpy())
        first = pred.groupby('track_id').t_ns.min().to_dict()
        trk, det, born = bag(run); tt, dt = np.array(sorted(trk)), np.array(sorted(det))
        last = {}
        for t, G in gt[gt.track_id >= 0].groupby('t_ns'):
            k = pt[np.argmin(np.abs(pt - t))]
            P = pred[pred.t_ns == k] if abs(k - t) < 0.06e9 else pred.iloc[:0]
            if not len(P): continue
            G = G.reset_index(drop=True); P = P.reset_index(drop=True)
            Dm = np.hypot(P.x.to_numpy()[:, None] - G.x.to_numpy()[None], P.y.to_numpy()[:, None] - G.y.to_numpy()[None])
            r_, c_ = linear_sum_assignment(np.where(Dm <= 1.0, Dm, 1e6))
            for i, j in zip(r_, c_):
                if Dm[i, j] > 1.0 or not G.ped[j]: continue
                gid, pid = int(G.track_id[j]), int(P.track_id[i])
                if gid in last and last[gid] != pid and first.get(pid, t) < t - 0.5e9:
                    n += 1
                    ti = np.searchsorted(tt, t) - 1
                    prev = None
                    for q in range(ti, max(-1, ti - 30), -1):           # last tick where B was not yet on the person (> 1 m) or B's entry 1 tick before
                        e = [x for x in trk[tt[q]] if x[0] == f'track_{pid}']
                        if e: prev = (tt[q], e[0]); break
                    if prev is None: C['B not on tracker topic'] += 1; continue
                    tq, e = prev
                    d = float(np.hypot(e[1] - G.x[j], e[2] - G.y[j])); dists.append(d); ages.append((tq - born.get(pid, tq)) / 1e9)
                    C['state ' + e[5]] += 1; C['class ' + (e[6] or 'none')] += 1
                    C['size big (>1.2 m)' if e[3] > 1.2 else 'size person-like'] += 1
                    C['dist <0.5' if d < 0.5 else 'dist 0.5-1' if d < 1 else 'dist 1-2' if d < 2 else 'dist >2'] += 1
                    Dd = det[dt[np.argmin(np.abs(dt - tq))]]
                    if Dd:
                        nd = min(Dd, key=lambda x: np.hypot(x[1] - e[1], x[2] - e[2]))
                        if np.hypot(nd[1] - e[1], nd[2] - e[2]) < 0.5:
                            C['B det label ' + nd[5]] += 1
                            if 'F' in nd[5] and nd[3] > 1.2: C['B = YOLO-refined big object (merged-fragment proxy)'] += 1
                        else: C['B had no own cluster (coasting)'] += 1
                last[gid] = pid
    print(f'{tag}: takeover events {n}; age median {np.median(ages) if ages else float("nan"):.1f} s; distance B->person median '
          f'{np.median(dists) if dists else float("nan"):.2f} m')
    for k in sorted(C): print(f'   {k}: {C[k]}')
