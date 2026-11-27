#!/usr/bin/env python3
"""Where does SINS lose recall against a reference method (e.g. EMMC)? (27/11, analysis of existing outputs only)

For every GT row kept by coda_eval's e3 GT (same ignore rules), match per annotated frame (Hungarian, 1 m) against SINS
and against the reference. Each GT row is: both / sins_only / ref_only / neither. The "ref_only" rows (found by the
reference, missed by SINS) are broken down by class group, motion state (D / PD), inside/outside the camera FOV,
range bin, LiDAR points in the box (npts bin), and by what SINS had near the object:
  offset   a SINS prediction 1-2 m from the GT centre (found, centre off)
  nothing  no SINS prediction within 2 m (not detected / not published)
Plus the same breakdown of all SINS misses, for scale.

usage: recall_gap.py --windows coda2_w1 ... --sins-tpl <dir with {w}> --ref-tpl <csv with {w}> [--ref-method eagermot]
         --bags <dir with <w>_gt.csv and <w>_odom> --out gap.json
Pure analysis.
"""
import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import linear_sum_assignment

E = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(E))
import coda_eval as ce                                     # noqa: E402
from harness.ros_io import load_obstacles, load_odometry   # noqa: E402
from harness.tracks import objects                         # noqa: E402

GROUP = {'Pedestrian': 'persons'}
for c in ['Car', 'Pickup Truck', 'Utility Vehicle', 'Service Vehicle', 'Delivery Truck', 'Bus', 'Truck', 'Van']:
    GROUP[c] = 'vehicles'
for c in ['Bike', 'Scooter', 'Motorcycle']:
    GROUP[c] = 'bikes'


def matches(g, p, gate):
    if p is None or not len(p) or not len(g):
        return {}, np.full(len(g), np.inf)
    d = np.hypot(g['x'].to_numpy()[:, None] - p['x'].to_numpy()[None], g['y'].to_numpy()[:, None] - p['y'].to_numpy()[None])
    r, c = linear_sum_assignment(np.where(d <= gate, d, 1e6))
    m = {i: j for i, j in zip(r, c) if d[i, j] <= gate}
    return m, d.min(axis=1)


def near(pt, by_pt, t, tol=0.15e9):
    if not len(pt):
        return None
    k = int(np.argmin(np.abs(pt - t)))
    return by_pt[pt[k]] if abs(int(pt[k]) - int(t)) <= tol else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--windows', nargs='+', required=True)
    ap.add_argument('--sins-tpl', required=True)
    ap.add_argument('--ref-tpl', required=True)
    ap.add_argument('--ref-method', default='eagermot')
    ap.add_argument('--bags', required=True)
    ap.add_argument('--range', type=float, default=20.0)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    rows = []
    for w in a.windows:
        gt_csv, odom_bag = f'{a.bags}/{w}_gt.csv', f'{a.bags}/{w}_odom'
        odom = load_odometry(odom_bag)
        ns = argparse.Namespace(camera_only=False, dyn_speed=0.3, ignore_undefined=True, min_pts=1, min_range=1.2,
                                pd_everywhere=False, pd_prev_observable=False, range=a.range, state_buffer=0.0)
        gt, ign, frames, _ = ce.build_gt(gt_csv, odom, ns)
        gt = objects(gt)
        raw = pd.read_csv(gt_csv, keep_default_na=False)
        raw = raw[pd.to_numeric(raw['track_id'], errors='coerce') >= 0]
        info = {(int(t), int(i)): (c, str(f) == 'True', float(n) if n != '' else 0.0, float(rg) if rg != '' else np.nan)
                for t, i, c, f, n, rg in zip(raw['t_ns'], raw['track_id'], raw['class'], raw['in_fov'], raw['npts'], raw['range'])}
        sp, _, _ = load_obstacles(a.sins_tpl.format(w=w))
        rp = ce.load_csv_tracks(a.ref_tpl.format(w=w))
        odz = ce.odom_ground_z(odom_bag)
        fargs = argparse.Namespace(range=a.range, min_range=1.2, max_height=2.5, camera_only=False, ignore_dist=1.0)
        sp, _, _ = ce.filter_pred(sp, ign, frames, odom, odz, fargs)
        rp, _, _ = ce.filter_pred(rp, ign, frames, odom, odz, fargs)
        sp, rp = objects(sp), objects(rp)
        sp = sp[sp['state'].isin(['dynamic', 'potentially_dynamic'])]
        spt, rpt = np.sort(sp['t_ns'].unique()), np.sort(rp['t_ns'].unique())
        sby, rby = dict(tuple(sp.groupby('t_ns'))), dict(tuple(rp.groupby('t_ns')))
        for t, g in gt.groupby('t_ns'):
            g = g.reset_index(drop=True)
            ms, dmin = matches(g, near(spt, sby, t), 1.0)
            mr, _ = matches(g, near(rpt, rby, t), 1.0)
            for i in range(len(g)):
                c, fov, npts, rg = info.get((int(t), int(g['track_id'][i])), ('?', False, 0, np.nan))
                s, r = i in ms, i in mr
                rows.append(dict(w=w, group=GROUP.get(c, 'other'), state='D' if g['state'][i] == 'dynamic' else 'PD',
                                 fov='in_fov' if fov else 'out_fov',
                                 range='<10' if rg < 10 else ('10-15' if rg < 15 else '15-20'),
                                 npts='<20' if npts < 20 else ('20-100' if npts < 100 else '>=100'),
                                 sins=s, ref=r, sins_near='offset' if (not s and dmin[i] <= 2.0) else ('nothing' if not s else '')))
    df = pd.DataFrame(rows)
    out = dict(gt_rows=len(df), sins_recall=float(df['sins'].mean()), ref_recall=float(df['ref'].mean()),
               both=int((df.sins & df.ref).sum()), sins_only=int((df.sins & ~df.ref).sum()),
               ref_only=int((~df.sins & df.ref).sum()), neither=int((~df.sins & ~df.ref).sum()))
    ro = df[~df.sins & df.ref]
    for k in ('group', 'state', 'fov', 'range', 'npts', 'sins_near'):
        out[f'ref_only_by_{k}'] = dict(Counter(ro[k]))
        out[f'all_by_{k}'] = {v: dict(n=int((df[k] == v).sum()), sins_recall=round(float(df[df[k] == v]['sins'].mean()), 3),
                                      ref_recall=round(float(df[df[k] == v]['ref'].mean()), 3)) for v in df[k].unique()}
    combo = ro.groupby(['group', 'state', 'fov']).size().sort_values(ascending=False).head(12)
    out['ref_only_top_combos'] = {' / '.join(k): int(v) for k, v in combo.items()}
    print(json.dumps(out, indent=1))
    json.dump(out, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
