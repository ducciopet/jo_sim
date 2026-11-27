#!/usr/bin/env python3
"""Per-class recall and precision (27/11, analysis of existing outputs; same GT, ignore rules and 1 m matching as
coda_eval.py e3, class-agnostic matching).

recall(group)    = GT rows of that group matched to ANY prediction / GT rows of that group
                   (+ "with the right class": matched to a prediction whose class maps to the same group)
precision(group) = predictions labelled with a class of that group that are matched to a GT row OF THAT GROUP /
                   predictions labelled with that group (matched to another group = wrong class; unmatched = false)
Predictions without a class are counted separately (share of all predictions, and their precision). SINS's class
comes from /sins/tracker/tracks (harness.ros_io.load_track_classes; '' = none); CSV baselines from their 'class' column.
Groups: persons, vehicles, bikes (bikes+scooters+motorcycles), other.

usage: per_class.py --windows coda2_w1 ... --run-tpl <bag or csv with {w}> [--method sins|eagermot|simpletrack|...]
        --bags <dir with <w>_gt.csv, <w>_odom> --out x.json
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import linear_sum_assignment

E = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(E))
import coda_eval as ce                                                     # noqa: E402
from harness.ros_io import load_obstacles, load_odometry, load_track_classes   # noqa: E402
from harness.tracks import objects                                         # noqa: E402

GT_GROUP = {'Pedestrian': 'persons'}
for c in ['Car', 'Pickup Truck', 'Utility Vehicle', 'Service Vehicle', 'Delivery Truck', 'Bus', 'Truck', 'Van']:
    GT_GROUP[c] = 'vehicles'
for c in ['Bike', 'Scooter', 'Motorcycle']:
    GT_GROUP[c] = 'bikes'
PRED_GROUP = {'person': 'persons', 'pedestrian': 'persons', 'car': 'vehicles', 'truck': 'vehicles', 'bus': 'vehicles',
              'van': 'vehicles', 'vehicle': 'vehicles', 'bicycle': 'bikes', 'cyclist': 'bikes', 'motorbike': 'bikes',
              'motorcycle': 'bikes', 'bike': 'bikes', 'scooter': 'bikes'}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--windows', nargs='+', required=True)
    ap.add_argument('--run-tpl', required=True)
    ap.add_argument('--method', default='sins')
    ap.add_argument('--bags', required=True)
    ap.add_argument('--range', type=float, default=20.0)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    G, P = [], []
    for w in a.windows:
        gt_csv, odom_bag = f'{a.bags}/{w}_gt.csv', f'{a.bags}/{w}_odom'
        odom = load_odometry(odom_bag)
        ns = argparse.Namespace(camera_only=False, dyn_speed=0.3, ignore_undefined=True, min_pts=1, min_range=1.2,
                                pd_everywhere=False, pd_prev_observable=False, range=a.range, state_buffer=0.0)
        gt, ign, frames, _ = ce.build_gt(gt_csv, odom, ns)
        gt = objects(gt)
        raw = pd.read_csv(gt_csv, keep_default_na=False)
        raw = raw[pd.to_numeric(raw['track_id'], errors='coerce') >= 0]
        cls = {int(i): GT_GROUP.get(c, 'other') for i, c in zip(raw['track_id'], raw['class'])}
        run = a.run_tpl.format(w=w)
        if a.method == 'sins':
            pr, _, _ = load_obstacles(run)
            kc = load_track_classes(run)
        else:
            pr = ce.load_csv_tracks(run)
            kc = None
        fargs = argparse.Namespace(range=a.range, min_range=1.2, max_height=2.5, camera_only=False, ignore_dist=1.0)
        pr, _, _ = ce.filter_pred(pr, ign, frames, odom, ce.odom_ground_z(odom_bag), fargs)
        pr = objects(pr)
        pr = pr[pr['state'].isin(['dynamic', 'potentially_dynamic'])]
        pt = np.sort(pr['t_ns'].unique())
        by = dict(tuple(pr.groupby('t_ns')))
        for t, g in gt.groupby('t_ns'):
            g = g.reset_index(drop=True)
            p = None
            if len(pt):
                k = int(np.argmin(np.abs(pt - t)))
                if abs(int(pt[k]) - int(t)) <= 0.15e9:
                    p = by[pt[k]].reset_index(drop=True)
            m = {}
            if p is not None and len(p) and len(g):
                d = np.hypot(g['x'].to_numpy()[:, None] - p['x'].to_numpy()[None], g['y'].to_numpy()[:, None] - p['y'].to_numpy()[None])
                r, c = linear_sum_assignment(np.where(d <= 1.0, d, 1e6))
                m = {i: j for i, j in zip(r, c) if d[i, j] <= 1.0}
            pg = []
            if p is not None:
                for j in range(len(p)):
                    lab = kc.get((int(p['t_ns'][j]), int(p['track_id'][j])), '') if kc is not None else str(p['class'][j] or '')
                    pg.append(PRED_GROUP.get(str(lab).lower(), '' if not lab else 'other'))
            inv = {j: i for i, j in m.items()}
            for i in range(len(g)):
                G.append(dict(group=cls.get(int(g['track_id'][i]), 'other'), found=i in m,
                              right=(i in m) and pg[m[i]] == cls.get(int(g['track_id'][i]), 'other')))
            if p is not None:
                for j in range(len(p)):
                    P.append(dict(group=pg[j], matched=j in inv,
                                  right=(j in inv) and pg[j] == cls.get(int(g['track_id'][inv[j]]), 'other')))
    G, P = pd.DataFrame(G), pd.DataFrame(P)
    out = dict(method=a.method, windows=a.windows, gt_rows=len(G), predictions=len(P), recall={}, precision={})
    for grp in ('persons', 'vehicles', 'bikes', 'other'):
        g = G[G.group == grp]
        if len(g):
            out['recall'][grp] = dict(n=len(g), any=round(float(g.found.mean()), 3), right_class=round(float(g.right.mean()), 3))
        p = P[P.group == grp]
        if len(p):
            out['precision'][grp] = dict(n=len(p), right_class=round(float(p.right.mean()), 3),
                                         wrong_class=round(float((p.matched & ~p.right).mean()), 3),
                                         false=round(float((~p.matched).mean()), 3))
    nc = P[P.group == '']
    out['no_class'] = dict(share=round(len(nc) / max(1, len(P)), 3), precision=round(float(nc.matched.mean()), 3) if len(nc) else None)
    print(json.dumps(out, indent=1))
    json.dump(out, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
