#!/usr/bin/env python3
"""
Reference of MOVING objects for a recording made with the robot standing still (Marina di Pisa 14/10 "street": robot
on the kerb, cars and pedestrians passing), without manual labels. Offline and non-causal, so independent of SINS:

  1. background = voxels (0.2 m) occupied in >= --bg-frac of the scans of a centred --bg-window (default 180 s): with
     the sensor still, everything that stays put (buildings, parked cars, people sitting at the bar) is background;
     a car passing a voxel occupies it for a few scans only;
  2. ground = per 0.5 m cell, low percentile of the background heights; foreground = non-background points (background
     dilated by one voxel) between --min-h and --max-h above it, within the SINS outdoor range (+-15 m);
  3. per scan, DBSCAN on xy (eps 0.6 m) -> clusters; global nearest-neighbour tracking with constant velocity
     (Hungarian, gate 2.5 m: 14 m/s cars move 1.4 m per scan), forward over the whole recording;
  4. kept: tracks of >= 8 scans whose net displacement is >= --min-travel; positions smoothed (centred 5-scan median).
     Class by size: 'vehicle' if the 90th percentile of the cluster's extent along its motion is >= 2.5 m or its
     height >= 1.9 m with length >= 2 m, 'person' otherwise (pedestrians, cyclists).

Writes <out>_gt.csv in coda_eval's format (odom frame of the shared odometry bag; one row per scan, track_id -1 when no
mover is visible) and <out>_gt.json. Only MOVING objects are in it: score with coda_eval's dynamic_only mode.
usage: static_sensor_movers.py <sensor_bag> <odom_bag> <out prefix>
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader
from scipy.optimize import linear_sum_assignment
from sklearn.cluster import DBSCAN

V = 0.2


def quat_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def scans(bag):
    with AnyReader([Path(bag)]) as r:
        c = [c for c in r.connections if c.topic == '/velodyne_points']
        for cc, ts, raw in r.messages(c):
            m = r.deserialize(raw, cc.msgtype)
            a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
            off = {f.name: f.offset for f in m.fields}
            xyz = np.stack([a[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1)
            xyz = xyz[np.isfinite(xyz).all(1)]
            yield m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec, xyz


def keys(p):
    q = np.floor(p / V).astype(np.int64) + 4096
    return (q[:, 0] << 26) | (q[:, 1] << 13) | q[:, 2]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('odom')
    ap.add_argument('out')
    ap.add_argument('--bg-window', type=float, default=180.0)
    ap.add_argument('--bg-frac', type=float, default=0.10)
    ap.add_argument('--min-h', type=float, default=0.3)
    ap.add_argument('--max-h', type=float, default=3.5)
    ap.add_argument('--range', type=float, default=15.0)
    ap.add_argument('--min-travel', type=float, default=2.0)
    a = ap.parse_args()

    # static robot: one odom <- velodyne transform (first odometry pose, base_link -> velodyne from /tf_static)
    with AnyReader([Path(a.odom)]) as r:
        c = [c for c in r.connections if c.topic == '/odometry/filtered']
        m = r.deserialize(next(r.messages(c))[2], c[0].msgtype)
        P, Q = m.pose.pose.position, m.pose.pose.orientation
        T_ob = np.eye(4); T_ob[:3, :3] = quat_R(Q.x, Q.y, Q.z, Q.w); T_ob[:3, 3] = [P.x, P.y, P.z]
    T_bv = np.eye(4)
    with AnyReader([Path(a.bag)]) as r:
        for cc, _, raw in r.messages([c for c in r.connections if c.topic == '/tf_static']):
            for t in r.deserialize(raw, cc.msgtype).transforms:
                if t.header.frame_id == 'base_link' and t.child_frame_id == 'velodyne':
                    q, p = t.transform.rotation, t.transform.translation
                    T_bv[:3, :3] = quat_R(q.x, q.y, q.z, q.w); T_bv[:3, 3] = [p.x, p.y, p.z]
    T = T_ob @ T_bv
    sensor_xy = T[:2, 3]

    # pass 1: voxel occupancy per 10 s block
    stamps, blocks = [], {}
    for t, xyz in scans(a.bag):
        p = xyz @ T[:3, :3].T + T[:3, 3]
        p = p[(np.abs(p[:, 0] - sensor_xy[0]) <= a.range + 1) & (np.abs(p[:, 1] - sensor_xy[1]) <= a.range + 1)]
        b = len(stamps) // 100
        k = np.unique(keys(p))
        blocks.setdefault(b, []).append(k)
        stamps.append(t)
    nb = max(blocks) + 1
    cnt = []
    for b in range(nb):
        u, c = np.unique(np.concatenate(blocks[b]), return_counts=True)
        cnt.append((u, c, len(blocks[b])))
    del blocks
    half = int(round(a.bg_window / 2 / 10))
    bg = []
    for b in range(nb):
        lo, hi = max(0, b - half), min(nb, b + half + 1)
        allk = np.concatenate([cnt[i][0] for i in range(lo, hi)])
        allc = np.concatenate([cnt[i][1] for i in range(lo, hi)])
        n = sum(cnt[i][2] for i in range(lo, hi))
        uk, inv = np.unique(allk, return_inverse=True)
        s = np.bincount(inv, weights=allc)
        bk = uk[s >= a.bg_frac * n]
        # dilate by one voxel in x/y/z
        q = np.stack([(bk >> 26) & 8191, (bk >> 13) & 8191, bk & 8191], 1)
        d = np.array([[i, j, k] for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)])
        qq = (q[:, None, :] + d[None]).reshape(-1, 3)
        bg.append(np.unique((qq[:, 0] << 26) | (qq[:, 1] << 13) | qq[:, 2]))
    # ground from the whole-recording background (voxel centres)
    allk = np.unique(np.concatenate([c[0] for c in cnt]))
    zc = ((allk & 8191) - 4096 + 0.5) * V
    xc = (((allk >> 26) & 8191) - 4096 + 0.5) * V
    yc = (((allk >> 13) & 8191) - 4096 + 0.5) * V
    cell = lambda x, y: (np.floor(x / 0.5).astype(np.int64) << 20) ^ np.floor(y / 0.5).astype(np.int64)
    g = pd.DataFrame(dict(c=cell(xc, yc), z=zc)).groupby('c').z.quantile(0.05)
    sensor_z = T[2, 3]

    # pass 2: foreground clusters per scan
    det = []          # (scan index, t, cx, cy, cz, pts array xy, height)
    for i, (t, xyz) in enumerate(scans(a.bag)):
        p = xyz @ T[:3, :3].T + T[:3, 3]
        p = p[(np.abs(p[:, 0] - sensor_xy[0]) <= a.range) & (np.abs(p[:, 1] - sensor_xy[1]) <= a.range)]
        p = p[~np.isin(keys(p), bg[i // 100])]
        if not len(p):
            continue
        gz = g.reindex(cell(p[:, 0], p[:, 1])).to_numpy()
        gz = np.where(np.isnan(gz), sensor_z - 0.65, gz)
        h = p[:, 2] - gz
        p, h = p[(h >= a.min_h) & (h <= a.max_h)], h[(h >= a.min_h) & (h <= a.max_h)]
        if len(p) < 3:
            continue
        lab = DBSCAN(eps=0.6, min_samples=3).fit_predict(p[:, :2])
        for l in set(lab) - {-1}:
            q, hq = p[lab == l], h[lab == l]
            if len(q) < 4:
                continue
            det.append((i, t, *q[:, :2].mean(0), float(np.median(q[:, 2])), q[:, :2], float(hq.max()), len(q)))
        if i % 1000 == 0:
            print(f'{i} scans, {len(det)} clusters', flush=True)

    # tracking: constant velocity, Hungarian, gate 2.5 m, up to 3 missed scans
    by = {}
    for k, d in enumerate(det):
        by.setdefault(d[0], []).append(k)
    tracks, live = [], []      # track = dict(dets=[k...], pos, vel, last)
    for i in range(len(stamps)):
        ks = by.get(i, [])
        if live and ks:
            pred = np.array([tracks[j]['pos'] + tracks[j]['vel'] * (i - tracks[j]['last']) for j in live])
            obs = np.array([det[k][2:4] for k in ks])
            D = np.hypot(pred[:, None, 0] - obs[None, :, 0], pred[:, None, 1] - obs[None, :, 1])
            r_, c_ = linear_sum_assignment(np.where(D < 2.5, D, 1e6))
            used = set()
            for r, c in zip(r_, c_):
                if D[r, c] >= 2.5:
                    continue
                tr = tracks[live[r]]
                newp = obs[c]
                v = (newp - tr['pos']) / (i - tr['last'])
                tr['vel'] = v if len(tr['dets']) == 1 else 0.6 * tr['vel'] + 0.4 * v
                tr['pos'], tr['last'] = newp, i
                tr['dets'].append(ks[c]); used.add(c)
            ks = [k for n, k in enumerate(ks) if n not in used]
        for k in ks:
            tracks.append(dict(dets=[k], pos=np.array(det[k][2:4]), vel=np.zeros(2), last=i))
        live = [j for j in range(len(tracks)) if i - tracks[j]['last'] <= 3]

    rows, n_kept, cls_count = [], 0, {'vehicle': 0, 'person': 0}
    for tr in tracks:
        D = [det[k] for k in tr['dets']]
        if len(D) < 8:
            continue
        xy = np.array([d[2:4] for d in D])
        if np.hypot(*(xy[-1] - xy[0])) < a.min_travel:
            continue
        idx = np.array([d[0] for d in D])
        sm = np.stack([pd.Series(xy[:, j]).rolling(5, center=True, min_periods=1).median().to_numpy() for j in (0, 1)], 1)
        head = np.arctan2(*(sm[-1] - sm[0])[::-1])
        u = np.array([np.cos(head), np.sin(head)]); nrm = np.array([-u[1], u[0]])
        L = np.percentile([np.ptp(d[5] @ u) for d in D], 90)
        W = np.percentile([np.ptp(d[5] @ nrm) for d in D], 90)
        H = np.percentile([d[6] for d in D], 90)
        cls = 'vehicle' if (L >= 2.5 or (H >= 1.9 and L >= 2.0)) else 'person'
        cls_count[cls] += 1
        tid = n_kept; n_kept += 1
        for d, (x, y) in zip(D, sm):
            rel = np.array([x, y]) - T[:2, 3]
            # front camera (base x, ~89 deg HFOV): the robot looks along odom x up to its 3 deg start roll
            in_fov = bool(rel[0] > 0.5 and abs(np.degrees(np.arctan2(rel[1], rel[0] - 0.3))) < 44.5)
            rows.append(dict(t_ns=stamps[d[0]], track_id=tid, x=x, y=y, z=d[4], l=max(L, 0.3), w=max(W, 0.3), h=H,
                             yaw=head, state='', score=1.0, **{'class': cls}, occluded='', npts=d[7],
                             range=float(np.hypot(*rel)), in_fov=in_fov))
    seen = {r['t_ns'] for r in rows}
    rows += [dict(t_ns=t, track_id=-1) for t in stamps if t not in seen]
    df = pd.DataFrame(rows).sort_values(['t_ns', 'track_id'])
    df['npts'] = df.npts.astype('Int64')            # empty-frame rows have no count: keep the column integer
    df.to_csv(f'{a.out}_gt.csv', index=False, float_format='%.4f')
    json.dump(dict(source=a.bag, method=__doc__.split('\n')[1], tracks=n_kept, classes=cls_count,
                   scans=len(stamps), t0_ns=stamps[0], t1_ns=stamps[-1], n_color=1, params=vars(a)),
              open(f'{a.out}_gt.json', 'w'), indent=1)
    print(f'done: {len(stamps)} scans, {len(det)} clusters, {len(tracks)} tracks, kept {n_kept} movers {cls_count}, '
          f'{(df.track_id >= 0).sum()} rows')


if __name__ == '__main__':
    main()
