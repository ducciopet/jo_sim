#!/usr/bin/env python3
"""
Top-down replay video of one lab sequence: mocap GT vs the methods' tracks, side by side, in the mocap frame.

Each panel: robot (mocap pose, front camera FOV wedge), GT person (ring + 3 s trail, GT state), the method's
published tracks (oriented boxes coloured by state, track id). Header: time, GT state, per-method running
MOTA-style counts are left to the tables -- this is for looking at behaviour, not for scoring.

Methods without a tracks.csv in --results (the literature baselines) are read from their recorded runs
(baseline_eval.predictions, rep = the last letter of --results). --map draws the static map (the no-rejection GLIM
map of eval/map_eval.py at body height, minus the person's own trail) as grey background.

Usage
  python3 eval/make_video.py --results eval/results/own_r8a --seq 20260519_1947 --methods v1 v2 \
      --out eval/videos/r8a_1947.mp4 [--speed 2]
  python3 eval/make_video.py --results eval/results/own_r9a --seq 20260519_1950 --methods v2 lvdot dynablox mdetector \
      --map --out eval/videos/compare_1950.mp4
"""

import argparse
import glob
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
from matplotlib.patches import Polygon, Wedge, Circle  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from harness.ros_io import load_mocap  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
COL = {'dynamic': '#d62728', 'potentially_dynamic': '#ff9f1c', 'static': '#8a8a8a'}
SHORT = {'dynamic': 'D', 'potentially_dynamic': 'PD', 'static': 'S'}
NAME = {'v1': 'v1', 'v2': 'SINS', 'v2g': 'v2 (greedy-Kuhn)', 'lvdot': 'LV-DOT', 'dynablox': 'Dynablox',
        'mdetector': 'M-detector', 'xurgbd': 'Xu et al.', 'xuyolo': 'Xu et al. + YOLO', 'unimt': 'UniMT + SimpleTrack'}
HFOV = np.deg2rad(87)


def box(x, y, l, w, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    pts = np.array([[l, w], [l, -w], [-l, -w], [-l, w]]) / 2
    return pts @ np.array([[c, s], [-s, c]]) + [x, y]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--results', required=True)
    ap.add_argument('--seq', required=True)
    ap.add_argument('--methods', nargs='+', default=['v1', 'v2'])
    ap.add_argument('--out', required=True)
    ap.add_argument('--speed', type=float, default=2.0, help='playback speed factor')
    ap.add_argument('--dpi', type=int, default=90)
    ap.add_argument('--names', nargs='+', default=None, help='panel titles, one per method (default: built-in names)')
    ap.add_argument('--map', action='store_true', help='static map background (bags/replay/map_none_<seq>)')
    args = ap.parse_args()

    res = Path(args.results)
    gt = pd.read_csv(res / args.seq / 'gt' / 'tracks.csv')
    names = dict(zip(args.methods, args.names)) if args.names else NAME
    al_pred = json.loads((res / 'alignment_v2.json').read_text())

    def load(m):
        f = res / args.seq / m / 'tracks.csv'
        if f.exists():
            return pd.read_csv(f)
        import baseline_eval as be
        return be.predictions(m, res.name[4:-1], res.name[-1], args.seq, res, al_pred)
    preds = {m: load(m) for m in args.methods}
    walls = None
    if args.map:
        import map_eval as me
        maps, _, _, _ = me.aligned_maps(args.seq, [f"none={REPO / 'bags' / 'replay' / f'map_none_{args.seq}'}"],
                                        gt_campaign=res.name)
        pm = maps['none']
        pm = pm[(pm[:, 2] > 0.2) & (pm[:, 2] < 1.8)]
        walls = np.unique(np.floor(pm[:, :2] / 0.05).astype(np.int64), axis=0) * 0.05 + 0.025
        from scipy.spatial import cKDTree  # the person's own trail is not structure: drop it (map_eval's ghost region)
        d, _ = cKDTree(gt[gt['track_id'] >= 0][['x', 'y']].to_numpy()).query(walls)
        walls = walls[d > 0.8]
    bodies = load_mocap(glob.glob(str(REPO / 'bags' / 'qualysis' / f'*{args.seq}_0'))[0])
    jo = bodies['JO']
    al = json.loads((res / f'alignment_{args.methods[-1] if args.methods[-1] in ("v1", "v2") else "v2"}.json').read_text())
    jo_yaw_off = al['T_base_JO'][2]

    times = np.sort(gt['t_ns'].unique())
    pts = np.concatenate([gt[['x', 'y']].to_numpy(), jo[:, 1:3]])
    lo, hi = pts.min(0) - 1.0, pts.max(0) + 1.0

    fig, axes = plt.subplots(1, len(args.methods), figsize=(5.2 * len(args.methods), 5.0), squeeze=False)
    axes = axes[0]
    fig.subplots_adjust(left=0.04, right=0.98, top=0.86, bottom=0.05, wspace=0.08)
    writer = FFMpegWriter(fps=10 * args.speed, codec='libx264', extra_args=['-pix_fmt', 'yuv420p', '-crf', '26'])
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    pred_by_t = {m: {t: g for t, g in p[p['track_id'] >= 0].groupby('t_ns')} for m, p in preds.items()}
    pred_ts = {m: np.array(sorted(d)) for m, d in pred_by_t.items()}

    with writer.saving(fig, args.out, args.dpi):
        for t in times:
            ts = t / 1e9
            g = gt[gt['t_ns'] == t].iloc[0]
            trail = gt[(gt['t_ns'] > t - 3e9) & (gt['t_ns'] <= t)]
            k = np.searchsorted(jo[:, 0], ts).clip(0, len(jo) - 1)
            rx, ry, ryaw = jo[k, 1], jo[k, 2], jo[k, 4] - jo_yaw_off
            for ax, m in zip(axes, args.methods):
                ax.clear()
                ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect('equal')
                ax.set_xticks([]); ax.set_yticks([])
                ax.set_title(names.get(m, m), fontsize=12)
                if walls is not None:
                    ax.scatter(walls[:, 0], walls[:, 1], s=0.6, c='#8f98a3', lw=0)
                ax.add_patch(Wedge((rx, ry), 6.0, np.rad2deg(ryaw - HFOV / 2), np.rad2deg(ryaw + HFOV / 2),
                                   color='#4c78a8', alpha=0.08, lw=0))
                ax.add_patch(Polygon(box(rx, ry, 0.55, 0.45, ryaw), closed=True, fc='#4c78a8', ec='#1f3b5c'))
                ax.arrow(rx, ry, 0.4 * np.cos(ryaw), 0.4 * np.sin(ryaw), width=0.03, color='#1f3b5c')
                ax.plot(trail['x'], trail['y'], '-', color='black', alpha=0.35, lw=1.5)
                ax.add_patch(Circle((g['x'], g['y']), 0.25, fill=False, ec='black', lw=2, ls='--'))
                # latest published output at most 0.25 s old (same rule as the evaluator)
                pt = pred_ts[m]
                j = np.searchsorted(pt, t, side='right') - 1
                if j >= 0 and t - pt[j] <= 0.25e9:
                    for _, p in pred_by_t[m][pt[j]].iterrows():
                        c = COL.get(p['state'], '#555')
                        l, w = (p['l'], p['w']) if np.isfinite(p['l']) else (0.4, 0.4)
                        ax.add_patch(Polygon(box(p['x'], p['y'], l, w, p['yaw'] if np.isfinite(p['yaw']) else 0),
                                             closed=True, fc=c, ec=c, alpha=0.55))
                        ax.text(p['x'], p['y'] + 0.35, f"{int(p['track_id'])} {SHORT.get(p['state'], '?')}",
                                fontsize=8, ha='center', color=c, weight='bold')
            fig.suptitle(f"{args.seq}   t = {ts - times[0] / 1e9:5.1f} s   GT: {SHORT[g['state']]}   "
                         f"(dashed ring = mocap person, boxes = tracks: D red, PD orange, S grey)   x{args.speed:g}",
                         fontsize=10)
            writer.grab_frame()
    print('wrote', args.out)


if __name__ == '__main__':
    main()
