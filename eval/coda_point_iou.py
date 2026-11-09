#!/usr/bin/env python3
"""
Point-wise IoU on a CODa window, the metric of the Dynablox and M-detector papers, for any method.

GT points: the window's LiDAR points (/velodyne_points, moved to odom with the window odometry and T_base_velodyne)
inside an annotated box at that annotated frame. Each GT box keeps coda_eval.py's state (moving = DYNAMIC above
--dyn-speed) and filters: boxes with < --min-pts returns, farther than --range or closer than --min-range from the
robot are ignored (their points count neither as TP, FP nor FN), as are points beyond --range / inside --min-range.
Predicted points: the same LiDAR points inside a predicted box (SINS tracks, Dynablox clusters; nearest prediction
stamp within --dt) or, for M-detector, the LiDAR points within --pt-dist of one of its published dynamic points.
SINS: 'dyn' = boxes in state DYNAMIC, 'any' = every published box (moving or still); the baselines only publish
what they think moves, so dyn = any.

Labels (point sets of the GT): moving / still x pedestrian / vehicle / other (bikes, scooters, ...), plus 'moving'
(all classes) and 'all' (every annotated agent). For each label L and prediction set P:
  IoU_L = TP / (TP + FN + FP), TP = |P & L|, FN = |L - P|, FP = |P - any GT box| (predicted points on no annotated
  agent; points on an agent of another label are not counted against L: a motion detector does not classify).
  recall_L = TP / |L|. Counts are summed over the window's annotated frames (pooled, as the papers do).
usage: coda_point_iou.py --runs NAME=sins:<run> NAME=dynablox:<bag> NAME=mdetector:<bag> ... --window bags/coda/coda11_w1
       [--bag <sensor bag>] [--out-dir dir]   (the window's scans are read once for all runs)
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

E = Path(__file__).resolve().parent
sys.path.insert(0, str(E))
sys.path.insert(0, str(E / 'adapters'))
import coda_eval as ce                                              # noqa: E402
from harness.ros_io import load_obstacles, load_odometry            # noqa: E402
from harness.tracks import objects                                  # noqa: E402
from marker_adapter import load_marker_boxes                        # noqa: E402

VEHICLES = {'Car', 'Pickup Truck', 'Delivery Truck', 'Utility Vehicle', 'Service Vehicle', 'Bus', 'Cart', 'Golf Cart',
            'Motorcycle', 'Truck', 'Van'}
LABELS = ('moving', 'moving_pedestrian', 'moving_vehicle', 'moving_other', 'still_pedestrian', 'still_vehicle',
          'still_other', 'all')


def stamp(m):
    return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec


def read_clouds(bag, times_ns, topic='/velodyne_points'):
    """{annotated t_ns: Nx3 sensor-frame points} for the scan nearest to each annotated time."""
    want = np.array(sorted(times_ns))
    out = {}
    with AnyReader([Path(bag)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == topic]):
            m = r.deserialize(raw, c.msgtype)
            t = stamp(m)
            k = np.searchsorted(want, t)
            for j in (k - 1, k):
                if 0 <= j < len(want) and abs(want[j] - t) < 50_000_000 and want[j] not in out:
                    out[want[j]] = xyz(m)
    return out


def xyz(m):
    f = {x.name: x.offset for x in m.fields}
    a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
    p = np.stack([a[:, f[k]:f[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1)
    return p[np.isfinite(p).all(1) & (np.abs(p).sum(1) > 0)]


def odom_poses(bag):
    rows = []
    with AnyReader([Path(bag)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == '/odometry/filtered']):
            m = r.deserialize(raw, c.msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            rows.append((stamp(m), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    return np.array(sorted(rows))


def pose_at(poses, t):
    k = int(np.clip(np.searchsorted(poses[:, 0], t), 1, len(poses) - 1))
    a, b = poses[k - 1], poses[k]
    s = 0.0 if b[0] == a[0] else float(np.clip((t - a[0]) / (b[0] - a[0]), 0, 1))
    T = np.eye(4)
    T[:3, 3] = a[1:4] + s * (b[1:4] - a[1:4])
    T[:3, :3] = Rotation.from_quat(a[4:] if s < 0.5 else b[4:]).as_matrix()
    return T


def in_box(pts, b, pad=0.0):
    c, s = np.cos(-b['yaw']), np.sin(-b['yaw'])
    d = pts[:, :3] - [b['x'], b['y'], b['z']]
    lx = c * d[:, 0] - s * d[:, 1]
    ly = s * d[:, 0] + c * d[:, 1]
    return (np.abs(lx) <= b['l'] / 2 + pad) & (np.abs(ly) <= b['w'] / 2 + pad) & (np.abs(d[:, 2]) <= b['h'] / 2 + pad)


def gt_label(cls, moving):
    kind = 'pedestrian' if cls == 'Pedestrian' else 'vehicle' if cls in VEHICLES else 'other'
    return ('moving_' if moving else 'still_') + kind


def load_pred(method, run):
    """(kind, data): ('boxes', (stamps, {t: rows})) or ('points', (stamps, {t: Nx3}))."""
    if method == 'mdetector':
        mdet = {}
        with AnyReader([Path(run)]) as r:
            for c, _, rw in r.messages([c for c in r.connections if c.topic == '/m_detector/point_out']):
                m = r.deserialize(rw, c.msgtype)
                mdet[stamp(m)] = xyz(m)
        return 'points', (np.array(sorted(mdet)), mdet)
    if method == 'sins':
        pred, _, _ = load_obstacles(run, None)
        pobj = objects(pred)
    else:
        pobj = objects(load_marker_boxes(run, ce.BASELINES['dynablox'][0], persistent_ids=True)).assign(state='dynamic')
    return 'boxes', (np.unique(pobj['t_ns'].to_numpy()), {t: gr for t, gr in pobj.groupby('t_ns')})


def nearest(ts, t, dt):
    if not len(ts):
        return None
    k = int(np.clip(np.searchsorted(ts, t), 1, len(ts) - 1)) if len(ts) > 1 else 0
    tn = ts[k - 1] if k > 0 and abs(ts[k - 1] - t) < abs(ts[k] - t) else ts[k]
    return tn if abs(tn - t) <= dt * 1e9 else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', nargs='+', required=True, help='name=method:path, e.g. LWNC=sins:bags/coda/runs/LWNC_v2_coda0_w1')
    ap.add_argument('--window', required=True, help='bags/coda/coda<seq>_<w> (+ _odom, _gt.csv, _gt.json next to it)')
    ap.add_argument('--bag', help='sensor bag if not at --window (e.g. the NAS copy)')
    ap.add_argument('--range', type=float, default=25.0)
    ap.add_argument('--min-range', type=float, default=1.2)
    ap.add_argument('--min-pts', type=int, default=1)
    ap.add_argument('--dyn-speed', type=float, default=0.3)
    ap.add_argument('--dt', type=float, default=0.06, help='max |prediction stamp - scan stamp| (s)')
    ap.add_argument('--pt-dist', type=float, default=0.15, help='M-detector: LiDAR point = dynamic if this close (m)')
    ap.add_argument('--out-dir', help='one json per run: <out-dir>/<name>.json')
    a = ap.parse_args()
    W = Path(a.window)
    gt_csv, odom_bag = Path(f'{W}_gt.csv'), Path(f'{W}_odom')
    info = json.load(open(f'{W}_gt.json'))
    T_bv = np.array(json.loads(info['T_base_velodyne']) if isinstance(info['T_base_velodyne'], str) else info['T_base_velodyne'])

    # GT boxes with coda_eval's state and filters (same args)
    ea = argparse.Namespace(dyn_speed=a.dyn_speed, min_pts=a.min_pts, pd_everywhere=True, range=a.range,
                            min_range=a.min_range)
    odom = load_odometry(str(odom_bag))
    gt, _, frames, _ = ce.build_gt(gt_csv, odom, ea)
    g = objects(gt)
    state = {(int(t), int(i)): s for t, i, s in zip(g['t_ns'], g['track_id'], g['state'])}
    raw = pd.read_csv(gt_csv, keep_default_na=False)
    raw = raw[raw['track_id'] != -1].copy()
    for c in ('x', 'y', 'z', 'l', 'w', 'h', 'yaw'):
        raw[c] = raw[c].astype(float)
    boxes = {}
    for r in raw.itertuples():
        boxes.setdefault(int(r.t_ns), []).append(dict(x=r.x, y=r.y, z=r.z, l=r.l, w=r.w, h=r.h, yaw=r.yaw, cls=r._12,
                                                     st=state.get((int(r.t_ns), int(r.track_id)))))
    runs = []
    for spec in a.runs:
        name, rest = spec.split('=', 1)
        method, path = rest.split(':', 1)
        runs.append((name, method, path, *load_pred(method, path)))

    poses = odom_poses(odom_bag)
    clouds = read_clouds(a.bag or W, frames)
    cnt = {n: {k: dict(tp=0, fn=0, n=0) for p in ('dyn', 'any') for k in [f'{p}:{l}' for l in LABELS]} for n, *_ in runs}
    fp = {n: dict(dyn=0, any=0) for n, *_ in runs}
    used = 0
    for t in frames:
        if t not in clouds:
            continue
        T = pose_at(poses, t) @ T_bv
        P = clouds[t] @ T[:3, :3].T + T[:3, 3]
        rob = T[:2, 3]
        d = np.hypot(P[:, 0] - rob[0], P[:, 1] - rob[1])
        P = P[(d <= a.range) & (d >= a.min_range)]
        lab = np.full(len(P), '', object)          # '' = on no annotated agent
        ign = np.zeros(len(P), bool)
        for b in boxes.get(int(t), []):
            m = in_box(P, b)
            if b['st'] is None:                    # filtered by coda_eval (hidden / out of range): ignore its points
                ign |= m
            else:
                lab[m] = gt_label(b['cls'], b['st'] == 'dynamic')
        keep = ~ign
        Ls = {}
        for l in LABELS:
            if l == 'all':
                Ls[l] = (lab != '') & keep
            elif l == 'moving':
                Ls[l] = np.array([x.startswith('moving_') for x in lab], bool) & keep
            else:
                Ls[l] = (lab == l) & keep
        for name, method, path, kind, (ts, data) in runs:
            pd_ = {'dyn': np.zeros(len(P), bool), 'any': np.zeros(len(P), bool)}
            tn = nearest(ts, t, a.dt)
            if tn is not None and kind == 'boxes':
                for r in data[tn].itertuples():
                    m = in_box(P, dict(x=r.x, y=r.y, z=r.z, l=r.l, w=r.w, h=r.h, yaw=r.yaw))
                    pd_['any'] |= m
                    if getattr(r, 'state', 'dynamic') == 'dynamic':
                        pd_['dyn'] |= m
            elif tn is not None and len(data[tn]):
                dd, _ = cKDTree(data[tn]).query(P, distance_upper_bound=a.pt_dist)
                pd_['dyn'] = np.isfinite(dd)
                pd_['any'] = pd_['dyn'].copy()
            for p in ('dyn', 'any'):
                pr = pd_[p] & keep
                fp[name][p] += int((pr & (lab == '')).sum())
                for l, L in Ls.items():
                    c = cnt[name][f'{p}:{l}']
                    c['tp'] += int((L & pr).sum())
                    c['fn'] += int((L & ~pr).sum())
                    c['n'] += int(L.sum())
        used += 1
    for name, method, path, *_ in runs:
        res = {k: dict(v, fp=fp[name][k.split(':')[0]], iou=v['tp'] / max(1, v['tp'] + v['fn'] + fp[name][k.split(':')[0]]),
                       recall=v['tp'] / max(1, v['n'])) for k, v in cnt[name].items()}
        print(f"{name} {W.name}: frames {used}/{len(frames)} | " + ' '.join(
            f"{l} {res['dyn:' + l]['iou']:.3f}" for l in ('moving', 'moving_pedestrian', 'moving_vehicle', 'all')) +
            f" | any: all {res['any:all']['iou']:.3f}", flush=True)
        if a.out_dir:
            Path(a.out_dir).mkdir(parents=True, exist_ok=True)
            json.dump(dict(run=path, method=method, window=str(W), frames_used=used, frames=len(frames), config=vars(a),
                           counts=res), open(Path(a.out_dir) / f'{name}.json', 'w'), indent=1)


if __name__ == '__main__':
    main()
