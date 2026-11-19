#!/usr/bin/env python3
"""P6 (track_vehicle_lshape) false-positive diagnosis, 19/11, on existing CODa runs only (no new runs).
Same GT / prediction filtering and frame sampling as coda_eval.py e3 (--ignore-undefined, merged mode, 1 m gate). For
every false positive (prediction unmatched on a scored frame):
  - published class (tracker /sins/tracker/tracks), state (D / PD), range from the robot;
  - kind: 'shifted' = an UNMATCHED GT within 2.5 m (the box sits next to a real object but its centre is beyond the
    1 m gate: costs an FP and an FN), 'duplicate' = only MATCHED GTs within 2.5 m (a second box on an object already
    found), 'phantom' = no GT within 2.5 m; class of that nearest GT.
usage: p6_fp.py <tag A> <tag B> <window> [<window> ...]   (reps '', r2, r3 pooled)"""
import sys, argparse, collections
from pathlib import Path
import numpy as np, pandas as pd
sys.path.insert(0, 'eval')
import coda_eval as ce
from harness.metrics import EvalConfig, make_grid, sample_gt, sample_pred, _match
from harness.ros_io import load_obstacles, load_odometry, load_track_classes

tags, wins = sys.argv[1:3], sys.argv[3:]
A = argparse.Namespace(range=20.0, ignore_dist=1.0, min_range=1.2, max_height=2.5, min_pts=1, pd_everywhere=False,
                       camera_only=False, cam_json=None, ignore_gt=None, ignore_undefined=True, dyn_speed=0.3,
                       match_dist=1.0, extra_ignore=None, extra_ignore_dist=0.3, extra_ignore_dt=0.5)
cfg = EvalConfig(match_dist=1.0, label_mode='merged', skip_missing_gt=False)
NEAR = 2.5
rows = []
for w in wins:
    c = w if '_' in w else '0_' + w
    gtc, odb = f'bags/coda/coda{c}_gt.csv', f'bags/coda/coda{c}_odom'
    odom = load_odometry(odb); oz = ce.odom_ground_z(odb)
    gt, ign, frames, _ = ce.build_gt(gtc, odom, A)
    gcls = pd.read_csv(gtc, keep_default_na=False)
    gcls = dict(zip(gcls.track_id.astype(int), gcls['class']))
    grid = make_grid(gt, cfg); G = sample_gt(gt, grid, cfg)
    for tag in tags:
        for r in ('', 'r2', 'r3'):
            run = f'bags/coda/runs/{tag}{r}_v2_coda{c}'
            if not Path(run).exists(): continue
            pred, _, _ = load_obstacles(run)
            pred, _, _ = ce.filter_pred(pred, ign, frames, odom, oz, A)
            cls = load_track_classes(run)
            P = sample_pred(pred, grid, cfg)
            obj = ce.objects(pred)
            ft = np.unique(pred['t_ns'].to_numpy()); fts = ft / 1e9
            idx = np.searchsorted(fts, grid, side='right') - 1
            for k, (gts, ps) in enumerate(zip(G, P)):
                if not ps: continue
                m, _ = _match(gts, ps, 1.0)
                mg = {i for i, _, _ in m}; mp = {j for _, j, _ in m}
                t = ft[idx[k]]
                rx, ry = np.interp(grid[k], odom[:, 0], odom[:, 1]), np.interp(grid[k], odom[:, 0], odom[:, 2])
                gxy = np.array([[x, y] for _, x, y, _ in gts]) if gts else np.zeros((0, 2))
                for j, (pid, x, y, st) in enumerate(ps):
                    if j in mp: continue
                    d = np.hypot(gxy[:, 0] - x, gxy[:, 1] - y) if len(gxy) else np.array([])
                    near = [i for i in np.argsort(d) if d[i] <= NEAR] if len(d) else []
                    unm = [i for i in near if i not in mg]
                    kind = 'shifted' if unm else ('duplicate' if near else 'phantom')
                    gi = (unm or near or [None])[0]
                    rows.append(dict(tag=tag, rep=r, win=w, kind=kind, state=st, pcls=cls.get((t, int(pid)), '-'),
                                     gcls=gcls.get(int(gts[gi][0]), '-') if gi is not None else '-',
                                     dgt=d[gi] if gi is not None else np.nan, rng=np.hypot(x - rx, y - ry)))
D = pd.DataFrame(rows)
D['rbin'] = pd.cut(D.rng, [0, 5, 10, 15, 20.1], labels=['<5', '5-10', '10-15', '15-20'])
D['pc'] = np.where(D.pcls.isin(['car', 'truck', 'bus']), 'vehicle', np.where(D.pcls == '-', 'none', D.pcls))
print(f'windows {" ".join(wins)}: FP (3 reps pooled)')
for col in ('kind', 'pc', 'state', 'rbin', 'gcls'):
    t = D.groupby(['tag', col], observed=True).size().unstack(0).fillna(0).astype(int)
    t['B-A'] = t[tags[1]] - t[tags[0]]
    print(f'\n-- by {col}\n{t.sort_values("B-A", ascending=False).to_string()}')
t = D.groupby(['tag', 'kind', 'pc'], observed=True).size().unstack(0).fillna(0).astype(int)
t['B-A'] = t[tags[1]] - t[tags[0]]
print(f'\n-- kind x published class\n{t.sort_values("B-A", ascending=False).to_string()}')
s = D[D.kind == 'shifted'].groupby('tag').dgt.median()
print(f'\nshifted: median centre distance to the unmatched GT: ' + ', '.join(f'{k} {v:.2f} m' for k, v in s.items()))
