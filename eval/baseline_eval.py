#!/usr/bin/env python3
"""
Compare external baselines with v1/v2 on the lab sequences, at the levels a method without the PD state can be judged on.

Levels (harness/metrics.py): 'dynamic_only' = does it flag what moves (a standing person flagged as moving is a false
alarm), 'merged' = does it find the person, moving or standing (D and PD are one label). The full D/PD/S level needs
the PD state, which only v1 and v2 have.

Detection metrics are compared for every method: MODA = 1 - (FN + FP) / GT (MOTA without identity switches), recall,
false alarms per minute. Identity metrics (IDF1, ID switches) only for methods with persistent track ids: LV-DOT
publishes none; Dynablox does (cluster tracking, exported by an output-only patch).

Inputs: v1/v2 = eval/results/own_<campaign><rep>/<seq>/<m>/tracks.csv (already in the mocap frame); baselines =
bags/<method>_out/<method><rep>_<seq>.bag (eval/adapters/marker_adapter.py), mapped to the mocap frame with the same
alignment as v2 (same shared odometry).
Writes eval/results/baselines_<campaign>.json and eval/tables/baselines_<campaign>.csv.

Usage
  python3 eval/baseline_eval.py --campaign r9 --reps a b c
"""

import argparse
import csv
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / 'adapters'))
from harness.gt_mocap import tracks_to_mocap  # noqa: E402
from harness.metrics import EvalConfig, evaluate  # noqa: E402
from harness.tracks import read_tracks  # noqa: E402
from marker_adapter import load_marker_boxes  # noqa: E402
from points_adapter import load_dynamic_points  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
SEQS = ['20260519_1942', '20260519_1945', '20260519_1947', '20260519_1949', '20260519_1950', '20260519_1952']
LEVELS = ('dynamic_only', 'merged')
IDENTITY = {'v1': True, 'v2': True, 'lvdot': False, 'dynablox': True, 'mdetector': False, 'xurgbd': False,
            'xuyolo': False, 'unimt': True}
# baseline -> (output dir under bags/, run prefix, marker topic, persistent ids)
BASELINES = {'lvdot': ('lvdot_out', 'lvdot', '/onboard_detector/dynamic_bboxes', False),
             'dynablox': ('dynablox_out', 'dynablox', '/motion_detector/visualization/clusters', True),
             'xurgbd': ('xu_rgbd_out', 'xurgbd', '/onboard_detector/dynamic_bboxes', False),
             'xuyolo': ('xu_rgbd_out', 'xuyolo', '/onboard_detector/dynamic_bboxes', False),
             'unimt': ('unimt_out', 'unimt', '/unimt/tracks', True),
             'mdetector': ('mdetector_out', 'mdetector', '/m_detector/point_out', None)}  # None = dynamic points


def predictions(method, campaign, rep, seq, res, align):
    if method in ('v1', 'v2'):
        return read_tracks(res / seq / method / 'tracks.csv')
    d, prefix, topic, ids = BASELINES[method]
    out = REPO / 'bags' / d
    if not (out / f'{prefix}{rep}_{seq}.done').exists():
        return None
    bag = out / f'{prefix}{rep}_{seq}.bag'
    boxes = load_dynamic_points(bag, topic) if ids is None else load_marker_boxes(bag, topic, persistent_ids=ids)
    return tracks_to_mocap(boxes, align['T_mocap_odom'][seq], align['delta'][seq])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default='r9')
    ap.add_argument('--reps', nargs='+', default=['a', 'b', 'c'])
    ap.add_argument('--methods', nargs='+', default=['v2', 'v1', 'lvdot', 'dynablox', 'mdetector', 'xurgbd', 'xuyolo', 'unimt'])
    args = ap.parse_args()
    out = {}
    for r in args.reps:
        res = REPO / 'eval' / 'results' / f'own_{args.campaign}{r}'
        S = json.loads((res / 'summary.json').read_text())
        align = json.loads((res / 'alignment_v2.json').read_text())
        for m in args.methods:
            per_level = {lvl: dict(tp=0, fp=0, fn=0, idsw=0, idtp=0, num_gt=0, num_pred=0, frames=0) for lvl in LEVELS}
            complete = True
            for s in SEQS:
                gt = read_tracks(res / s / 'gt' / 'tracks.csv')
                pred = predictions(m, args.campaign, r, s, res, align)
                if pred is None:
                    complete = False
                    break
                for lvl in LEVELS:
                    e = evaluate(gt, pred, EvalConfig(match_dist=S['match_dist'], roi=tuple(S['roi']),
                                                      t_start=S['t_start'], label_mode=lvl)).to_dict()
                    for k in per_level[lvl]:
                        per_level[lvl][k] += e['num_frames'] if k == 'frames' else e[k]
            if not complete:
                print(f'rep {r} {m}: incomplete, skipped')
                continue
            for lvl, c in per_level.items():
                minutes = c['frames'] / 10.0 / 60.0
                out.setdefault(m, {}).setdefault(lvl, {})[r] = {
                    'MODA': 1 - (c['fn'] + c['fp']) / c['num_gt'], 'recall': c['tp'] / c['num_gt'],
                    'false_alarms_per_min': c['fp'] / minutes, 'FP': c['fp'], 'FN': c['fn'], 'GT': c['num_gt'],
                    'IDF1': 2 * c['idtp'] / (c['num_gt'] + c['num_pred']) if IDENTITY[m] else None,
                    'MOTA': 1 - (c['fn'] + c['fp'] + c['idsw']) / c['num_gt'] if IDENTITY[m] else None}
    rows = []
    for m, levels in out.items():
        for lvl, reps in levels.items():
            row = {'method': m, 'level': lvl, 'reps': len(reps)}
            for k in ('MODA', 'recall', 'false_alarms_per_min', 'IDF1', 'MOTA', 'FP', 'FN'):
                xs = [v[k] for v in reps.values() if v[k] is not None]
                row[f'{k}_mean'] = statistics.fmean(xs) if xs else None
                row[f'{k}_std'] = (statistics.stdev(xs) if len(xs) > 1 else 0.0) if xs else None
            rows.append(row)
            f = lambda k, pct=True: '–' if row[f'{k}_mean'] is None else (
                f"{100*row[f'{k}_mean']:.1f}±{100*row[f'{k}_std']:.1f}" if pct else f"{row[f'{k}_mean']:.1f}±{row[f'{k}_std']:.1f}")
            print(f"{lvl:13s} {m:6s} n={row['reps']}  MODA {f('MODA')}  recall {f('recall')}  "
                  f"false alarms/min {f('false_alarms_per_min', False)}  IDF1 {f('IDF1')}")
    (REPO / 'eval' / 'results' / f'baselines_{args.campaign}.json').write_text(json.dumps(out, indent=1))
    if rows:
        with open(REPO / 'eval' / 'tables' / f'baselines_{args.campaign}.csv', 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


if __name__ == '__main__':
    main()
