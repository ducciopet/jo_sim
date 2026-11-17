#!/usr/bin/env python3
"""
Free-space probe (17/11, diagnosis only, SINS untouched): would a memory of "space seen empty" separate moving objects
from SINS's residual false positives on CODa (tuning windows only)?

For one CODa window: every scan, in time order, (1) QUERY: for each object present at that scan, the fraction of its
LiDAR points (above the ground) that fall in voxels already seen free by an earlier ray, at least --lag s ago and at
least --min-free times; (2) UPDATE: cast the scan's rays (subsampled) and mark the traversed voxels free.
Causal: a scan never uses its own or later rays.

Object groups:
  gt_moving   annotated agents moving (> 0.3 m/s over +-0.5 s, same rule as coda_eval), points in the box;
  gt_still    annotated agents of movable classes standing still, points in the box;
  fp_struct   SINS I2P residual moving FP (eval/diagnostics/fp_unannotated/fprows_LVI2P_edge.csv: no_object, no raw
              annotation within 1.5 m, within 20 m): points within --fp-radius of the reported centre; per the visual
              check these are ~70 % static structure (walls, poles, railings).
Voxel 0.2 m, rays 1-25 m, free marking stops --stop m before the hit. Odometry = the window's /odometry/filtered.

usage: free_space_probe.py <window e.g. 10_w1> [--sub 20000] [--out eval/diagnostics/free_space/<w>.csv]
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader

E = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(E))
from harness.ros_io import typestore  # noqa: E402


def quat_rot(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_odom(bag):
    out = []
    with AnyReader([Path(bag)], default_typestore=typestore()) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
            m = r.deserialize(raw, c.msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            T = np.eye(4)
            T[:3, :3] = quat_rot(q.x, q.y, q.z, q.w)
            T[:3, 3] = [p.x, p.y, p.z]
            out.append((m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, T))
    out.sort(key=lambda x: x[0])
    return np.array([o[0] for o in out]), np.array([o[1] for o in out])


def scans(bag):
    with AnyReader([Path(bag)], default_typestore=typestore()) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == '/velodyne_points']):
            m = r.deserialize(raw, c.msgtype)
            a = np.frombuffer(m.data, np.float32).reshape(-1, m.point_step // 4)[:, :3]
            yield m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, a


def gt_groups(gt_csv, dyn=0.3, hw=0.5):
    g = pd.read_csv(gt_csv, keep_default_na=False)
    g = g[g['track_id'] >= 0].copy()
    for c in ('x', 'y', 'z', 'l', 'w', 'h', 'yaw'):
        g[c] = g[c].astype(float)
    g['t'] = g['t_ns'] / 1e9
    sp = np.full(len(g), np.nan)
    for _, d in g.groupby('track_id'):
        t, x, y = d['t'].to_numpy(), d['x'].to_numpy(), d['y'].to_numpy()
        for j, ix in enumerate(d.index):
            a, b = np.searchsorted(t, t[j] - hw), np.searchsorted(t, t[j] + hw, 'right') - 1
            if t[b] - t[a] >= 1.6 * hw:
                sp[g.index.get_loc(ix)] = np.hypot(x[b] - x[a], y[b] - y[a]) / (t[b] - t[a])
    g['speed'] = sp
    g = g[np.isfinite(g['speed'])]
    g['grp'] = np.where(g['speed'] > dyn, 'gt_moving', 'gt_still')
    return g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('window')
    ap.add_argument('--root', default='bags/coda')
    ap.add_argument('--fp', default='eval/diagnostics/fp_unannotated/fprows_LVI2P_edge.csv')
    ap.add_argument('--voxel', type=float, default=0.2)
    ap.add_argument('--sub', type=int, default=20000)
    ap.add_argument('--rmax', type=float, default=25.0)
    ap.add_argument('--stop', type=float, default=0.3)
    ap.add_argument('--lag', type=float, default=0.5)
    ap.add_argument('--min-free', type=int, default=5)
    ap.add_argument('--fp-radius', type=float, default=0.4)
    ap.add_argument('--out')
    a = ap.parse_args()
    w = a.window
    base = f'{a.root}/coda{w}'
    import json
    T_bv = np.array(json.load(open(f'{base}_gt.json'))['T_base_velodyne'])
    ot, oT = read_odom(f'{base}_odom')
    gt = gt_groups(f'{base}_gt.csv')
    gt_by_t = {t: d for t, d in gt.groupby('t_ns')}
    fp = pd.read_csv(a.fp)
    fp = fp[(fp['w'] == w) & (fp['b'] == 'no_object') & (~fp['raw_gt'].astype(bool)) & (fp['in20'].astype(bool))]
    fp = fp[fp['rep'].isna() | (fp['rep'].astype(str).isin(['', 'nan']))]
    fpt = fp['t'].to_numpy()

    # dense grid around the path
    lo = oT[:, :3, 3].min(0) - a.rmax
    hi = oT[:, :3, 3].max(0) + a.rmax
    lo[2], hi[2] = oT[:, 2, 3].min() - 1.0, oT[:, 2, 3].max() + 4.0
    dims = np.ceil((hi - lo) / a.voxel).astype(int)
    # Dynablox-like "ever free": a voxel becomes ever-free after --min-free consecutive free observations with no hit
    # in between (any hit resets the streak); `first` = stamp when it became ever-free
    first = np.full(dims, np.iinfo(np.int64).max, np.int64)
    cnt = np.zeros(dims, np.uint16)                            # current free streak
    rng = np.random.default_rng(0)
    steps = np.arange(0.0, a.rmax, a.voxel / 2)
    rows, t_ray = [], []
    for k, (t, p) in enumerate(scans(base)):
        i = min(np.searchsorted(ot, t), len(ot) - 1)
        if abs(int(ot[i]) - t) > 60_000_000:
            continue
        T = oT[i] @ T_bv
        r = np.linalg.norm(p, axis=1)
        p = p[np.isfinite(r) & (r > 1.0) & (r < a.rmax)]
        P = p @ T[:3, :3].T + T[:3, 3]
        gz = oT[i][2, 3]                                       # base_link z = ground under the robot
        above = P[:, 2] > gz + 0.25
        # QUERY (before this scan updates the map)
        def frac(m):
            q = P[m]
            if len(q) == 0:
                return np.nan, 0
            v = ((q - lo) / a.voxel).astype(int)
            ok = np.all((v >= 0) & (v < dims), axis=1)
            v = v[ok]
            f = first[v[:, 0], v[:, 1], v[:, 2]] <= t - int(a.lag * 1e9)
            return float(f.mean()) if len(f) else np.nan, int(len(f))
        d = gt_by_t.get(t)
        if d is not None:
            for o in d.itertuples():
                c, s = np.cos(o.yaw), np.sin(o.yaw)
                dx, dy = P[:, 0] - o.x, P[:, 1] - o.y
                m = above & (np.abs(dx * c + dy * s) <= o.l / 2) & (np.abs(-dx * s + dy * c) <= o.w / 2) & \
                    (np.abs(P[:, 2] - o.z) <= o.h / 2)
                fr, n = frac(m)
                rows.append(dict(w=w, t=t, grp=o.grp, id=o.track_id, cls=o._12, speed=o.speed, npts=n, free_frac=fr))
        for j in np.nonzero(np.abs(fpt - t / 1e9) < 0.05)[0]:
            o = fp.iloc[j]
            m = above & (np.hypot(P[:, 0] - o['x'], P[:, 1] - o['y']) <= a.fp_radius)
            fr, n = frac(m)
            rows.append(dict(w=w, t=t, grp='fp_struct', id=o['id'], cls='', speed=np.nan, npts=n, free_frac=fr))
        # UPDATE: rays of a subsample
        t0 = time.time()
        sel = rng.choice(len(P), min(a.sub, len(P)), replace=False)
        o3 = T[:3, 3]
        dvec = P[sel] - o3
        rr = np.linalg.norm(dvec, axis=1)
        u = dvec / rr[:, None]
        S = steps[None, :] < (rr[:, None] - a.stop)
        pts = o3 + u[:, None, :] * steps[None, :, None]
        pts = pts[S]
        pts = pts[pts[:, 2] > gz + 0.25]
        v = ((pts - lo) / a.voxel).astype(int)
        v = v[np.all((v >= 0) & (v < dims), axis=1)]
        flat = np.unique(np.ravel_multi_index(v.T, dims))
        hv = ((P[above] - lo) / a.voxel).astype(int)
        hv = hv[np.all((hv >= 0) & (hv < dims), axis=1)]
        hit = np.unique(np.ravel_multi_index(hv.T, dims))
        flat = np.setdiff1d(flat, hit, assume_unique=True)
        ff, cc = first.reshape(-1), cnt.reshape(-1)
        cc[hit] = 0
        cc[flat] = np.minimum(cc[flat].astype(np.int32) + 1, 65535).astype(np.uint16)
        new = flat[cc[flat] >= a.min_free]
        ff[new] = np.minimum(ff[new], t)
        t_ray.append(time.time() - t0)
        if k % 100 == 0:
            print(f'{w} scan {k}, ray update {1000 * np.median(t_ray):.0f} ms (python)', flush=True)
    df = pd.DataFrame(rows)
    out = a.out or f'eval/diagnostics/free_space/{w}.csv'
    df.to_csv(out, index=False)
    q = df[df['npts'] >= 5].groupby('grp')['free_frac'].describe(percentiles=[.25, .5, .75])
    print(w, f'ray update median {1000 * np.median(t_ray):.0f} ms (python, {a.sub} rays)')
    print(q.round(2).to_string())


if __name__ == '__main__':
    main()
