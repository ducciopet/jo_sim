#!/usr/bin/env python3
"""Per-class / per-state recall and false-positive breakdown of one CODa run (diagnostics for tuning, not a metric).
usage: coda_diag.py <run bag> <window gt prefix, e.g. bags/coda/coda0_w1>"""
import sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import linear_sum_assignment
sys.path.insert(0, str(Path(__file__).resolve().parent))
from harness.gt_mocap import gt_tracks                               # noqa: E402
from harness.ros_io import load_obstacles, load_odometry             # noqa: E402
from harness.tracks import EMPTY_FRAME_ID, objects                   # noqa: E402

run, pre = sys.argv[1], sys.argv[2]
odom = load_odometry(pre + '_odom')
raw = pd.read_csv(pre + '_gt.csv', keep_default_na=False)
frames = np.unique(raw['t_ns'])
obj = raw[raw['track_id'] != EMPTY_FRAME_ID].copy()
for c in ('x', 'y', 'z', 'yaw', 'l', 'w', 'h', 'npts', 'range'):
    obj[c] = obj[c].astype(float)
bodies = {int(i): g.sort_values('t_ns')[['t_ns', 'x', 'y', 'z', 'yaw']].to_numpy(float) * [1e-9, 1, 1, 1, 1]
          for i, g in obj.groupby('track_id')}
gt, _ = gt_tracks(bodies, dyn_speed=0.3)
names = sorted(bodies)
gt['track_id'] = [names[i] for i in gt['track_id']]
# gt_tracks goes through float seconds: snap its stamps back onto the annotated frames' own
k = np.clip(np.searchsorted(frames, gt['t_ns'].to_numpy()), 1, len(frames) - 1)
gt['t_ns'] = np.where(np.abs(frames[k - 1] - gt['t_ns']) < np.abs(frames[k] - gt['t_ns']), frames[k - 1], frames[k])
gt = gt.drop(columns=[c for c in ('class', 'l', 'w', 'h') if c in gt]).merge(obj[['t_ns', 'track_id', 'class', 'npts', 'range', 'l', 'w', 'h']], on=['t_ns', 'track_id'], how='left')
pred = objects(load_obstacles(run)[0])
pt = np.unique(pred['t_ns'])
rows, fps = [], []
for t in frames:
    g = gt[gt['t_ns'] == t]
    tp_ = pt[np.argmin(np.abs(pt - t))]
    p = pred[pred['t_ns'] == tp_]
    m = set()
    if len(g) and len(p):
        D = np.hypot(g['x'].to_numpy()[:, None] - p['x'].to_numpy()[None], g['y'].to_numpy()[:, None] - p['y'].to_numpy()[None])
        r, c = linear_sum_assignment(D)
        m = {(i, j) for i, j in zip(r, c) if D[i, j] < 1.0}
    hit = {i for i, _ in m}
    rx, ry = np.interp(t / 1e9, odom[:, 0], odom[:, 1]), np.interp(t / 1e9, odom[:, 0], odom[:, 2])
    for k, (_, row) in enumerate(g.iterrows()):
        if row['range'] < 1.2:
            continue
        rows.append(dict(cls=row['class'], state=row['state'], hit=k in hit, range=row['range'], npts=row['npts']))
    used = {j for _, j in m}
    for j, (_, q) in enumerate(p.iterrows()):
        if j not in used and 1.2 <= np.hypot(q['x'] - rx, q['y'] - ry):
            dg = np.min(np.hypot(g['x'] - q['x'], g['y'] - q['y'])) if len(g) else np.inf
            fps.append(dict(x=q['x'] - rx, y=q['y'] - ry, h=q['h'], l=q['l'], w=q['w'], r=np.hypot(q['x'] - rx, q['y'] - ry), state=q.get('state', ''), cls=q.get('class', ''),
                            dgt=dg, z=q['z'], track=q['track_id']))
R = pd.DataFrame(rows)
print('recall by class/state:\n', R.groupby(['cls', 'state'])['hit'].agg(['mean', 'size']).round(2).to_string())
R['rb'] = pd.cut(R['range'], [0, 3, 6, 10, 15, 25])
print('recall by range:\n', R.groupby('rb', observed=True)['hit'].agg(['mean', 'size']).round(2).to_string())
F = pd.DataFrame(fps)
print(f'FP rows {len(F)} (per annotated frame {len(F) / len(frames):.2f}), tracks {F["track"].nunique()}')
F['rb'] = pd.cut(F['r'], [0, 3, 6, 10, 15, 25, 99])
print(F.groupby('rb', observed=True).size().to_string())
print('FP by state/class:\n', F.groupby(['state', 'cls']).size().to_string())
print('FP within 2 m of a GT box:', (F['dgt'] < 2).mean().round(2))
print(F.groupby('track').agg(n=('r', 'size'), r=('r', 'median'), l=('l', 'median'), w=('w', 'median'), h=('h', 'median'),
                             dgt=('dgt', 'median'), st=('state', 'first')).sort_values('n', ascending=False).head(10).round(2).to_string())
