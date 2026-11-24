#!/usr/bin/env python3
"""Crowd swaps (24/11): is a D-D takeover preceded by merge_fragments absorbing the taking-over track's own cluster?
For each person takeover A -> B (B a pre-existing D/PD track, same detection of events as coda_takeover.py) in a DM-based run
(E0/EB), look at the 1-5 tracker ticks before the switch, at B's own position pB(tick):
  own      a fused detection (any label) within 0.5 m of pB in this run (B still had its own cluster)
  absorbed no own detection, but pB lies inside (+0.1 m) a fused detection labelled with F (YOLO-refined) whose centre is > 0.5 m
           away (B's cluster is inside a neighbour's refined object)
  gone     neither
and in the reference run without merge_fragments (DC0 = r1lcc, same window, same stamps): was there a fused detection within
0.5 m of pB at the same stamp (own cluster present without the merge)? "yes" = absorbed/gone here AND present in DC0.
Proxy for the YOLO 2D instance: whether B's own detection carried F (its own camera leaf) at any tick in the 2 s before.
usage: crowd_absorb.py --tags EB EBr2 ... --ref DC0"""
import argparse, sys, collections
from pathlib import Path
import numpy as np, pandas as pd
from scipy.optimize import linear_sum_assignment
from rosbags.highlevel import AnyReader
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import coda_eval as ce   # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--tags', nargs='+', required=True)
ap.add_argument('--ref', default='DC0')
ap.add_argument('--windows', nargs='+', default='w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1'.split())
a = ap.parse_args()
B = Path(__file__).resolve().parents[2] / 'bags/coda'
ns = argparse.Namespace(range=20.0, ignore_dist=1.0, min_range=1.2, max_height=2.5, min_pts=1, pd_everywhere=False, camera_only=False,
                        cam_json=None, ignore_undefined=True, dyn_speed=0.3, match_dist=1.0, points_min_cluster=20, extra_ignore=None,
                        extra_ignore_dist=0.3, extra_ignore_dt=0.5, frames='all', coda_root='', ignore_gt=None, topic=None,
                        method='sins', ignore_box=False)
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
def bag(run):
    trk, det = {}, {}
    with AnyReader([Path(run)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic in ('/sins/tracker/tracks', '/sins/dbscan_detector/fused/detections3d')]):
            m = r.deserialize(raw, c.msgtype)
            L = [(d.id, d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size.x, d.bbox.size.y,
                  d.results[0].hypothesis.class_id if d.results else '') for d in m.detections]
            (trk if 'tracker' in c.topic else det)[st(m.header)] = L
    return trk, det
def inside(px, py, d, m):
    return abs(px - d[1]) <= d[3] / 2 + m and abs(py - d[2]) <= d[4] / 2 + m   # axis-aligned approximation
refcache = {}
for tag in a.tags:
    C = collections.Counter()
    for w in a.windows:
        c = w if '_' in w else f'0_{w}'
        run = B / 'runs' / f'{tag}_v2_coda{c}'; ref = B / 'runs' / f'{a.ref}_v2_coda{c}'
        if not run.exists() or not ref.exists(): continue
        odom = ce.load_odometry(B / f'coda{c}_odom')
        gt, ign, frames, _ = ce.build_gt(str(B / f'coda{c}_gt.csv'), odom, ns)
        raw = pd.read_csv(B / f'coda{c}_gt.csv')
        gt = gt.assign(ped=gt.track_id.map(raw[['track_id', 'class']].drop_duplicates('track_id').set_index('track_id')['class']) == 'Pedestrian')
        pred, _, _ = ce.load_obstacles(str(run), None)
        pred, _, _ = ce.filter_pred(pred, ign, frames, odom, ce.odom_ground_z(B / f'coda{c}_odom'), ns)
        pred = pred[pred.track_id >= 0]; pt = np.unique(pred.t_ns.to_numpy()); first = pred.groupby('track_id').t_ns.min().to_dict()
        trk, det = bag(run)
        if c not in refcache: refcache[c] = bag(ref)[1]
        rdet = refcache[c]; tt = np.array(sorted(trk)); dt = np.array(sorted(det)); rt = np.array(sorted(rdet))
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
                    ti = np.searchsorted(tt, t) - 1
                    ent = [x for x in trk[tt[ti]] if x[0] == f'track_{pid}'] if ti >= 0 else []
                    if not ent or ent[0][5] == 'static': last[gid] = pid; continue
                    C['D/PD takeovers'] += 1
                    states = []; ref_present = False; hadF = False; gx, gy = G.x[j], G.y[j]
                    for q in range(max(0, ti - 4), ti + 1):
                        e = [x for x in trk[tt[q]] if x[0] == f'track_{pid}']
                        if not e: continue
                        px, py = e[0][1], e[0][2]
                        D = det[dt[np.argmin(np.abs(dt - tt[q]))]]
                        own = [d for d in D if np.hypot(d[1] - px, d[2] - py) <= 0.5]
                        dP = [np.hypot(d[1] - gx, d[2] - gy) for d in D]
                        pclu = int(np.argmin(dP)) if D and min(dP) <= 0.6 else -1          # the person's own cluster (at takeover)
                        own = [d for d in own if pclu < 0 or d is not D[pclu]]
                        if own: states.append('own')
                        elif pclu >= 0 and np.hypot(D[pclu][1] - px, D[pclu][2] - py) <= 0.5: states.append('on person')
                        elif any(inside(px, py, d, 0.1) and 'F' in d[5] for d in D): states.append('absorbed')
                        else: states.append('gone')
                        RD = rdet[rt[np.argmin(np.abs(rt - tt[q]))]]
                        rP = [np.hypot(d[1] - gx, d[2] - gy) for d in RD]; rp = int(np.argmin(rP)) if RD and min(rP) <= 0.6 else -1
                        if any(np.hypot(d[1] - px, d[2] - py) <= 0.5 and (rp < 0 or d is not RD[rp]) for d in RD): ref_present = True
                    for q in range(max(0, ti - 20), ti + 1):                  # 2 s at 10 Hz: B's own cluster had its own camera leaf?
                        e = [x for x in trk[tt[q]] if x[0] == f'track_{pid}']
                        if e:
                            D = det[dt[np.argmin(np.abs(dt - tt[q]))]]
                            if any(np.hypot(d[1] - e[0][1], d[2] - e[0][2]) <= 0.5 and 'F' in d[5] for d in D): hadF = True
                    s = 'absorbed' if 'absorbed' in states else 'own' if 'own' in states[-2:] else 'on person' if states and states[-1] == 'on person' else 'gone'
                    C['B before: ' + s] += 1
                    if s not in ('own',):
                        C[f'B before {s}, ref DC0 had its cluster: ' + ('yes' if ref_present else 'no')] += 1
                        if ref_present: C[f'  yes & B had its own camera leaf in prev 2 s: ' + ('yes' if hadF else 'no')] += 1
                last[gid] = pid
    print(tag + ': ' + '; '.join(f'{k} {v}' for k, v in sorted(C.items())), flush=True)
