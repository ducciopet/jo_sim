#!/usr/bin/env python3
"""
Inside vs outside the front camera's field of view: how the methods do on the person when only the LiDAR sees them.

Per 10 Hz frame (harness/metrics grid), the person's GT position is expressed in the robot's frame (mocap pose of
the robot body JO, heading corrected by alignment_v2's T_base_JO yaw) and called "in FOV" when its bearing is within
half the colour camera's horizontal FOV (from /front_camera/camera/color/camera_info: 2 atan(w / 2 fx)) and it is
within --max-range; otherwise "out of FOV". Both groups are scored with the person-level rules of the timeline
(harness/metrics matching, 1 m gate): found (any label), label correct (D for GT D, PD for GT PD), id switches
(consecutive matched frames with a different id), plus the person's turning rate while walking (how "zig-zag").

usage: python3 eval/fov_eval.py --campaign r9 --reps a b c [--methods v2 v1 ...]
writes eval/results/fov_<campaign>.json
"""

import argparse
import glob
import json
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

sys.path.insert(0, str(Path(__file__).resolve().parent))
import baseline_eval as be  # noqa: E402
from harness.metrics import EvalConfig, _match, make_grid, sample_gt, sample_pred  # noqa: E402
from harness.ros_io import load_mocap  # noqa: E402
from harness.tracks import read_tracks  # noqa: E402


def camera_hfov(seq):
    with AnyReader([be.REPO / 'bags' / 'bags_benchmark' / f'duccio_test_{seq}_0']) as r:
        con = [c for c in r.connections if c.topic == '/front_camera/camera/color/camera_info']
        for c, _, raw in r.messages(con):
            m = r.deserialize(raw, c.msgtype)
            return 2 * np.arctan(m.width / 2 / m.k[0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default='r9')
    ap.add_argument('--reps', nargs='+', default=['a', 'b', 'c'])
    ap.add_argument('--methods', nargs='+', default=['v2', 'v1', 'lvdot', 'dynablox', 'mdetector', 'unimt', 'xuyolo'])
    ap.add_argument('--max-range', type=float, default=6.0, help='[m] beyond this the camera does not count as seeing')
    args = ap.parse_args()
    out = {}
    hfov = camera_hfov(be.SEQS[0])
    print(f'colour camera HFOV {np.degrees(hfov):.1f} deg')
    geo = {}
    for r in args.reps:
        res = be.REPO / 'eval' / 'results' / f'own_{args.campaign}{r}'
        S = json.loads((res / 'summary.json').read_text())
        al = json.loads((res / 'alignment_v2.json').read_text())
        cfg = EvalConfig(match_dist=S['match_dist'], roi=tuple(S['roi']), t_start=S['t_start'])
        yaw_off = al['T_base_JO'][2]
        for s in be.SEQS:
            gt = read_tracks(res / s / 'gt' / 'tracks.csv')
            grid = make_grid(gt, cfg)
            G = sample_gt(gt, grid, cfg)
            if s not in geo:
                jo = load_mocap(glob.glob(str(be.REPO / 'bags' / 'qualysis' / f'*{s}_0'))[0])['JO']
                k = np.searchsorted(jo[:, 0], grid).clip(0, len(jo) - 1)
                rx, ry, ryaw = jo[k, 1], jo[k, 2], jo[k, 4] - yaw_off
                inf, turn = [], []
                for i, g in enumerate(G):
                    g = [x for x in g if x[0] >= 0]
                    if not g:
                        inf.append(None)
                        continue
                    dx, dy = g[0][1] - rx[i], g[0][2] - ry[i]
                    inf.append(bool(abs(wrap(np.arctan2(dy, dx) - ryaw[i])) <= hfov / 2 and np.hypot(dx, dy) <= args.max_range))
                # walking heading changes, per frame (zig-zag indicator)
                p = gt[gt['track_id'] >= 0].sort_values('t_ns')
                xy, st = p[['x', 'y']].to_numpy(), p['state'].to_numpy()
                h = np.arctan2(np.diff(xy[:, 1]), np.diff(xy[:, 0]))
                dt = np.diff(p['t_ns'].to_numpy()) / 1e9
                ok = (st[1:] == 'dynamic') & (st[:-1] == 'dynamic') & (dt > 0) & (dt < 0.3)
                rate = np.abs(wrap(np.diff(h))) / dt[1:]
                okr = ok[1:] & ok[:-1]
                geo[s] = dict(infov=inf, turn_rate_deg_s=float(np.degrees(np.median(rate[okr]))) if okr.any() else float('nan'))
            for m in args.methods:
                P = sample_pred(be.predictions(m, args.campaign, r, s, res, al), grid, cfg)
                acc = out.setdefault(m, {}).setdefault(r, {g: dict(frames=0, found=0, correct=0, idsw=0, D=0, D_found=0, D_ok=0, PD=0, PD_ok=0, PD_found=0)
                                                          for g in ('in', 'out')})
                last = None
                for g, p, f in zip(G, P, geo[s]['infov']):
                    g = [x for x in g if x[0] >= 0]
                    if f is None:
                        continue
                    a = acc['in' if f else 'out']
                    mt, _ = _match(g, p, cfg.match_dist)
                    a['frames'] += 1
                    a['D'] += g[0][3] == 'dynamic'
                    a['PD'] += g[0][3] == 'potentially_dynamic'
                    if mt:
                        pid, pst = p[mt[0][1]][0], p[mt[0][1]][3]
                        a['found'] += 1
                        a['D_found'] += g[0][3] == 'dynamic'
                        a['correct'] += pst == g[0][3]
                        a['D_ok'] += pst == g[0][3] == 'dynamic'
                        a['PD_ok'] += pst == g[0][3] == 'potentially_dynamic'
                        a['PD_found'] += g[0][3] == 'potentially_dynamic'
                        if be.IDENTITY.get(m, True) and last is not None and pid != last:
                            a['idsw'] += 1
                        last = pid
    summary = {}
    for m, reps in out.items():
        row = {}
        for g in ('in', 'out'):
            tot = {k: sum(reps[r][g][k] for r in reps) for k in reps[next(iter(reps))][g]}
            row[g] = dict(frames=tot['frames'] // len(reps), found=tot['found'] / max(1, tot['frames']),
                          correct=tot['correct'] / max(1, tot['frames']),
                          D_found=tot['D_found'] / max(1, tot['D']), D_ok=tot['D_ok'] / max(1, tot['D']),
                          PD_frames=tot['PD'] // len(reps), PD_found=tot['PD_found'] / max(1, tot['PD']),
                          PD_ok=tot['PD_ok'] / max(1, tot['PD']),
                          idsw_per_rep=(tot['idsw'] / len(reps)) if be.IDENTITY.get(m, True) else None)
        summary[m] = row
        def f(g):
            sw = row[g]['idsw_per_rep']
            return (f"found {100 * row[g]['found']:5.1f}%  label ok {100 * row[g]['correct']:5.1f}%  "
                    f"walking found {100 * row[g]['D_found']:5.1f}% as D {100 * row[g]['D_ok']:5.1f}%  "
                    f"standing ({row[g]['PD_frames']} fr) found {100 * row[g]['PD_found']:5.1f}% as PD {100 * row[g]['PD_ok']:5.1f}%  idsw/rep {'n/a' if sw is None else f'{sw:.1f}'}")
        print(f"{m:10s} IN  ({row['in']['frames']} fr) {f('in')}\n{'':10s} OUT ({row['out']['frames']} fr) {f('out')}")
    per_seq = {s: dict(out_of_fov=sum(1 for x in v['infov'] if x is False) / max(1, sum(1 for x in v['infov'] if x is not None)),
                       turn_rate_deg_s=v['turn_rate_deg_s']) for s, v in geo.items()}
    for s, v in per_seq.items():
        print(f"{s}: person out of the camera FOV {100 * v['out_of_fov']:5.1f}% of the time, walking turn rate (median) "
              f"{v['turn_rate_deg_s']:.0f} deg/s")
    (be.REPO / 'eval' / 'results' / f'fov_{args.campaign}.json').write_text(
        json.dumps(dict(hfov_deg=float(np.degrees(hfov)), methods=summary, per_seq=per_seq), indent=1))


if __name__ == '__main__':
    main()
