#!/usr/bin/env python3
"""Where do the CODa errors of a run come from? (19/11, plan revision step 1; analysis only, no new runs)

Same GT, ignore regions and prediction filtering as coda_eval.py (e3: --ignore-undefined), then per annotated frame a
one-to-one assignment (Hungarian, centre distance) at 1 m (the metric's gate) and, as a diagnostic, at 2 m.
Per class group (persons / vehicles / bikes+scooters / other):
  GT rows, matched at 1 m, matched only at 2 m (= "found but centre 1-2 m off": one FN + one FP in the metric), missed.
Unmatched predictions at 1 m, by cause:
  offset     an unmatched GT object of any class within 2 m (the same object, centre off)
  piece      inside the box (+0.3 m) of a GT object that IS matched (extra fragment of a found object)
  duplicate  within 0.6 m of another prediction of the same frame (two tracks / two cameras on one object)
  near       within 1.5 m of the box of an annotated object (a piece or a shifted box next to it)
  ghost      none of the above (no annotated object there; may be an unannotated real object)
  and the class group of the nearest GT object within 3 m.
usage: err_decomp.py --runs DIR [DIR ...] --windows 10_w1 ... [--method sins|<csv method>] [--frames testing]
       DIR may contain {c} for the window (e.g. bags/coda/runs/Q0_v2_coda{c}); CSV methods: a CSV path template.
"""
import argparse, sys, collections
from pathlib import Path
import numpy as np, pandas as pd
from scipy.optimize import linear_sum_assignment
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import coda_eval as ce   # noqa: E402

GROUP = {'Pedestrian': 'persons'}
for c in ['Car', 'Pickup Truck', 'Utility Vehicle', 'Service Vehicle', 'Delivery Truck', 'Bus', 'Truck', 'Van']: GROUP[c] = 'vehicles'
for c in ['Bike', 'Scooter', 'Motorcycle']: GROUP[c] = 'bikes'


def frame_pairs(P, G, gate):
    if not len(P) or not len(G): return []
    D = np.hypot(P.x.to_numpy()[:, None] - G.x.to_numpy()[None], P.y.to_numpy()[:, None] - G.y.to_numpy()[None])
    r, c = linear_sum_assignment(np.where(D <= gate, D, 1e6))
    return [(i, j) for i, j in zip(r, c) if D[i, j] <= gate]


def inside(px, py, g, m):
    c, s = np.cos(g.yaw), np.sin(g.yaw); dx, dy = px - g.x, py - g.y
    return abs(dx * c + dy * s) <= g.l / 2 + m and abs(-dx * s + dy * c) <= g.w / 2 + m


def one(run, c, a, B):
    ns = argparse.Namespace(range=20.0, ignore_dist=1.0, min_range=1.2, max_height=2.5, min_pts=1, pd_everywhere=False,
                            camera_only=False, cam_json=None, ignore_undefined=True, dyn_speed=0.3, match_dist=1.0,
                            points_min_cluster=20, extra_ignore=None, extra_ignore_dist=0.3, extra_ignore_dt=0.5,
                            frames='all', coda_root=a.coda_root, ignore_gt=None, topic=None, method=a.method)
    odom = ce.load_odometry(B / f'coda{c}_odom')
    gt, ign, frames, _ = ce.build_gt(str(B / f'coda{c}_gt.csv'), odom, ns)
    if a.method == 'sins':
        pred, _, _ = ce.load_obstacles(run, None)
    else:
        pred = ce.load_csv_tracks(run)
    pred, _, _ = ce.filter_pred(pred, ign, frames, odom, ce.odom_ground_z(B / f'coda{c}_odom'), ns)
    keep = None
    if a.frames != 'all':
        import json, re
        info = json.load(open(B / f'coda{c}_gt.json'))
        meta = json.load(open(Path(a.coda_root) / 'metadata' / f"{info['seq']}.json"))['ObjectTracking'][a.frames]
        split = {int(re.findall(r'_(\d+)\.json$', f)[0]) for f in meta}
        keep = {t for f, t in zip(info['annotated_frames'], info['annotated_t_ns']) if int(f) in split}
    raw = pd.read_csv(B / f'coda{c}_gt.csv')
    gcls = raw[['track_id', 'class']].drop_duplicates('track_id').set_index('track_id')['class']
    dims = raw.groupby('track_id')[['l', 'w']].median()   # the evaluator's GT carries no box size
    gt = gt.assign(grp=gt.track_id.map(gcls).map(GROUP).fillna('other'), l=gt.track_id.map(dims.l).fillna(0.5),
                   w=gt.track_id.map(dims.w).fillna(0.5), yaw=gt.yaw.fillna(0.0))
    pt = np.unique(pred.t_ns.to_numpy()) if len(pred) else np.array([0])
    G1 = collections.Counter(); FP = collections.Counter()
    for t, G in gt.groupby('t_ns'):
        if keep is not None and t not in keep: continue
        k = pt[np.argmin(np.abs(pt - t))]
        P = pred[pred.t_ns == k] if abs(k - t) < 0.06e9 else pred.iloc[:0]
        G = G.reset_index(drop=True); P = P.reset_index(drop=True)
        m1 = frame_pairs(P, G, 1.0); g1 = {j for _, j in m1}; p1 = {i for i, _ in m1}
        restP = P.drop(index=list(p1)); restG = G.drop(index=list(g1))
        m2 = frame_pairs(restP.reset_index(drop=True), restG.reset_index(drop=True), 2.0)
        g2 = {restG.index[j] for _, j in m2}
        for j, g in G.iterrows():
            G1[(g.grp, 'rows')] += 1
            G1[(g.grp, 'TP@1m' if j in g1 else ('only@2m' if j in g2 else 'missed'))] += 1
        for i, p in P.iterrows():
            if i in p1: continue
            d = np.hypot(G.x - p.x, G.y - p.y)
            near = G.grp[d.idxmin()] if len(G) and d.min() < 3.0 else 'none'
            if any(j not in g1 and d[j] < 2.0 for j in G.index): cause = 'offset'
            elif any(j in g1 and inside(p.x, p.y, G.loc[j], 0.3) for j in G.index): cause = 'piece'
            elif ((np.hypot(P.x - p.x, P.y - p.y) < 0.6).sum() > 1): cause = 'duplicate'
            elif any(inside(p.x, p.y, G.loc[j], 1.5) for j in G.index): cause = 'near'
            else: cause = 'ghost'
            FP[(near, cause)] += 1
    return G1, FP


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', nargs='+', required=True)
    ap.add_argument('--windows', nargs='+', required=True)
    ap.add_argument('--method', default='sins')
    ap.add_argument('--frames', default='all')
    ap.add_argument('--coda-root', default='/mnt/nas-imperatore/datasets_SINS/CODa')
    ap.add_argument('--bags', default=str(Path(__file__).resolve().parents[2] / 'bags/coda'))
    a = ap.parse_args()
    B = Path(a.bags); G1 = collections.Counter(); FP = collections.Counter()
    for tmpl in a.runs:
        for w in a.windows:
            c = w if '_' in w else f'0_{w}'
            run = tmpl.format(c=c)
            if not Path(run).exists(): print('missing', run, file=sys.stderr); continue
            g, f = one(run, c, a, B); G1.update(g); FP.update(f)
    print(f'runs: {a.runs}  windows: {a.windows}  frames: {a.frames}')
    print('\nGT rows by class group (TP at the 1 m gate / found only within 2 m / missed):')
    for grp in ['persons', 'vehicles', 'bikes', 'other']:
        n = G1[(grp, 'rows')]
        if n: print(f'  {grp:9s} {n:6d}  TP@1m {100*G1[(grp,"TP@1m")]/n:5.1f}%  only@2m {100*G1[(grp,"only@2m")]/n:5.1f}%  missed {100*G1[(grp,"missed")]/n:5.1f}%')
    tot = sum(FP.values())
    print(f'\nunmatched predictions at 1 m: {tot}')
    for near in ['persons', 'vehicles', 'bikes', 'other', 'none']:
        n = sum(v for (k, _), v in FP.items() if k == near)
        if n: print(f'  near {near:9s} {n:6d}: ' + ', '.join(f'{cause} {FP[(near, cause)]}' for cause in ['offset', 'piece', 'duplicate', 'near', 'ghost']))


if __name__ == '__main__':
    main()
