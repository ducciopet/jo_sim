#!/usr/bin/env python3
"""
Label S on the lab sequences: everything that is not the person is static.

The mocap GT has one object, the person (D or PD). Assumption (true in the Qualisys lab: nothing else moves
inside the capture volume): every other object there is S. On the method side, everything not published as D/PD
is S: v2 does not publish static tracks at all, v1 does but the protocol ignores them, so "labelled S" means "not
reported as dynamic" for both.

Per 10 Hz frame (same grid, ROI, gate and dropout skip as harness/metrics.evaluate):
  person -> S      the person matched to no D/PD prediction (the "missed" frames of the main evaluation)
  S -> D, S -> PD  a D/PD prediction matched to nobody and farther than --dup-dist from the person (closer ones are
                   duplicates of the person, already counted as FP, not static objects)
  S -> S           the static objects present minus those labelled D/PD. Their number is not in the GT: it is
                   estimated, the same for every method, as the fused detections of that sequence's v2 run (same
                   sensor bag for v1 and v2) inside the ROI and farther than --dup-dist from the person. One
                   object split in two clusters counts twice: S -> S is an order of magnitude, the S -> D/PD
                   counts are exact.

Writes eval/results/own_<C>_static.json and eval/tables/own_<C>_static.csv (per method: mean/std over reps).

Usage
  python3 eval/static_eval.py --campaign r9 --reps a b c
"""

import argparse
import csv
import json
import statistics
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader

sys.path.insert(0, str(Path(__file__).resolve().parent))
from harness.gt_mocap import tracks_to_mocap  # noqa: E402
from harness.metrics import EvalConfig, _match, make_grid, sample_gt, sample_pred  # noqa: E402
from harness.tracks import DYNAMIC, POTENTIALLY_DYNAMIC, objects, read_tracks  # noqa: E402
from harness.ros_io import stamp_ns  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
SEQS = ['20260519_1942', '20260519_1945', '20260519_1947', '20260519_1949', '20260519_1950', '20260519_1952']
FUSED = '/jo_detector_v2/dbscan_detector/fused/detections3d'


def load_fused(run):
    """fused detections of a v2 run -> unified-track-like frame (t_ns, x, y) in odom, one row per detection."""
    rows = []
    with AnyReader([run]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == FUSED]):
            m = r.deserialize(raw, c.msgtype)
            t = stamp_ns(m.header)
            for d in m.detections:
                p = d.bbox.center.position
                rows.append((t, p.x, p.y))
    df = pd.DataFrame(rows, columns=['t_ns', 'x', 'y'])
    df['track_id'], df['yaw'], df['vx'], df['vy'] = 0, 0.0, 0.0, 0.0
    return df


def count_seq(gt, pred, fused, cfg, dup):
    grid = make_grid(gt, cfg)
    G, P = sample_gt(gt, grid, cfg), sample_pred(pred, grid, cfg)
    spans = {gid: (g['t_ns'].min() / 1e9, g['t_ns'].max() / 1e9) for gid, g in objects(gt).groupby('track_id')}
    ft = np.unique(fused['t_ns'].to_numpy()) / 1e9
    by_t = {t / 1e9: g[['x', 'y']].to_numpy() for t, g in fused.groupby('t_ns')}
    xmin, xmax, ymin, ymax = cfg.roi
    n = dict(frames=0, person_S=0, S_D=0, S_PD=0, S_objects=0, frames_false_dyn=0)
    for k, t in enumerate(grid):
        present = {g[0] for g in G[k]}
        if not all(gid in present for gid, (a, b) in spans.items() if a <= t <= b) or not G[k]:
            continue
        n['frames'] += 1
        px, py = G[k][0][1], G[k][0][2]
        m, _ = _match(G[k], P[k], cfg.match_dist)
        if not m:
            n['person_S'] += 1
        matched = {j for _, j, _ in m}
        false_dyn = 0
        for j, (_, x, y, st) in enumerate(P[k]):
            if j in matched or np.hypot(x - px, y - py) <= dup:
                continue
            n['S_D' if st == DYNAMIC else 'S_PD'] += 1
            false_dyn += 1
        n['frames_false_dyn'] += false_dyn > 0
        i = np.searchsorted(ft, t, side='right') - 1
        if i >= 0 and t - ft[i] <= cfg.max_pred_age:
            xy = by_t[ft[i]]
            inside = (xy[:, 0] >= xmin) & (xy[:, 0] <= xmax) & (xy[:, 1] >= ymin) & (xy[:, 1] <= ymax)
            far = np.hypot(xy[:, 0] - px, xy[:, 1] - py) > dup
            n['S_objects'] += int((inside & far).sum())
    return n


def scores(n):
    s_s = max(0, n['S_objects'] - n['S_D'] - n['S_PD'])
    prec = s_s / (s_s + n['person_S']) if s_s + n['person_S'] else float('nan')
    rec = s_s / n['S_objects'] if n['S_objects'] else float('nan')
    minutes = n['frames'] / 10.0 / 60.0
    return dict(n, S_S=s_s, prec_S=prec, recall_S=rec,
                f1_S=2 * prec * rec / (prec + rec) if prec + rec > 0 else float('nan'),
                false_dyn_per_min=(n['S_D'] + n['S_PD']) / minutes,
                share_frames_false_dyn=n['frames_false_dyn'] / n['frames'])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default='r9')
    ap.add_argument('--reps', nargs='+', default=['a', 'b', 'c'])
    ap.add_argument('--methods', nargs='+', default=['v1', 'v2'])
    ap.add_argument('--dup-dist', type=float, default=1.0, help='[m] = the match gate')
    args = ap.parse_args()
    c = args.campaign
    out = {}
    for r in args.reps:
        res = REPO / 'eval' / 'results' / f'own_{c}{r}'
        S = json.loads((res / 'summary.json').read_text())
        cfg = EvalConfig(match_dist=S['match_dist'], roi=tuple(S['roi']), t_start=S['t_start'])
        al = json.loads((res / 'alignment_v2.json').read_text())
        for s in SEQS:
            gt = read_tracks(res / s / 'gt' / 'tracks.csv')
            fused = tracks_to_mocap(load_fused(REPO / 'bags' / 'replay' / f'{c}{r}_v2_{s}'),
                                    al['T_mocap_odom'][s], al['delta'][s])
            for m in args.methods:
                pred = read_tracks(res / s / m / 'tracks.csv')
                n = count_seq(gt, pred, fused, cfg, args.dup_dist)
                tot = out.setdefault(m, {}).setdefault(r, {k: 0 for k in n})
                for k, v in n.items():
                    tot[k] += v
                print(f'{c}{r} {s} {m}: {n}')
    rows, js = [], {}
    for m, per_rep in out.items():
        sc = {r: scores(n) for r, n in per_rep.items()}
        js[m] = sc
        row = {'method': m, 'reps': len(sc)}
        for k in next(iter(sc.values())):
            xs = [sc[r][k] for r in sc]
            row[f'{k}_mean'] = statistics.fmean(xs)
            row[f'{k}_std'] = statistics.stdev(xs) if len(xs) > 1 else 0.0
        rows.append(row)
        print(f"{m}: S precision {100*row['prec_S_mean']:.2f}  S recall {100*row['recall_S_mean']:.3f}  "
              f"S->D {row['S_D_mean']:.1f}  S->PD {row['S_PD_mean']:.1f}  person->S {row['person_S_mean']:.1f}  "
              f"S objects {row['S_objects_mean']:.0f}  false dyn/min {row['false_dyn_per_min_mean']:.2f}")
    (REPO / 'eval' / 'results' / f'own_{c}_static.json').write_text(json.dumps(js, indent=1))
    with open(REPO / 'eval' / 'tables' / f'own_{c}_static.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    main()
