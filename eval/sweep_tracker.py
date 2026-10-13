#!/usr/bin/env python3
"""
Tracker parameter sweep with leave-one-sequence-out (LOSO) validation.

For every value of one tracker parameter, tracker_node is replayed alone on the
SAME recorded inputs of every sequence (eval/scripts/run_tracker_only.sh, which
is deterministic), and evaluated against the motion-capture GT. Then, holding
each sequence out in turn, the value that maximises the summed-count MOTA on
the other sequences is picked and scored on the held-out one. The LOSO total is
an honest estimate of what tuning this parameter on these bags buys on unseen
sequences; a large gap to the in-sample best flags overfitting.

Usage (host; the container must be running):
  python3 eval/sweep_tracker.py --param track_min_hits --values 2 4 6 --inputs r5
Results are cached in eval/results/sweeps/<param>=<value>/ so re-runs are cheap.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from harness.gt_mocap import gt_tracks, tracks_to_mocap  # noqa: E402
from harness.metrics import EvalConfig, evaluate  # noqa: E402
from harness.ros_io import load_mocap, load_obstacles  # noqa: E402
from harness.tracks import read_tracks  # noqa: E402
from report import aggregate  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
REPLAY_HOST = REPO / 'bags' / 'replay'
REPLAY_CONT = '/home/ros/bags/replay'
CONTAINER = 'jo_sim_lv-dot-jo-sim_lv-dot-1'
SEQS = ['20260519_1942', '20260519_1945', '20260519_1947', '20260519_1949', '20260519_1950', '20260519_1952']


def run_value(param, value, inputs, gt_dir, force=False):
    tag = f'sw_{param}={value}'
    out = REPO / 'eval' / 'results' / 'sweeps' / f'{param}={value}.json'
    if out.exists() and not force:
        return json.loads(out.read_text())
    for s in SEQS:
        bag = REPLAY_HOST / f'{tag}_v2_{s}'
        if not (bag / 'metadata.yaml').exists() or force:
            subprocess.run(['docker', 'exec', CONTAINER, 'bash', f'{REPLAY_CONT}/run_tracker_only.sh',
                            f'{REPLAY_CONT}/{inputs}_v2_{s}', f'{REPLAY_CONT}/{tag}_v2_{s}', '1.0', '--',
                            '-p', f'{param}:={value}'], check=True, capture_output=True)
    A = json.loads((gt_dir / 'alignment_v2.json').read_text())
    S = json.loads((gt_dir / 'summary.json').read_text())
    res = {}
    for s in SEQS:
        p, _, _ = load_obstacles(REPLAY_HOST / f'{tag}_v2_{s}')
        pred = tracks_to_mocap(p, np.array(A['T_mocap_odom'][s]), A['delta'][s])
        gt = read_tracks(gt_dir / s / 'gt' / 'tracks.csv')
        r = evaluate(gt, pred, EvalConfig(roi=tuple(S['roi']))).to_dict()
        res[s] = json.loads(json.dumps(r, default=float).replace('NaN', 'null'))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res))
    return res


_MOCAP = {}


def state_acc_by_gt_threshold(param, value, gt_dir, thresholds=(0.2, 0.3, 0.4)):
    """State accuracy of one sweep value under different GT dynamic thresholds (robustness check)."""
    tag = f'sw_{param}={value}'
    A = json.loads((gt_dir / 'alignment_v2.json').read_text())
    S = json.loads((gt_dir / 'summary.json').read_text())
    out = {}
    for thr in thresholds:
        res = []
        for s in SEQS:
            if s not in _MOCAP:
                _MOCAP[s] = load_mocap(next((REPO / 'bags' / 'qualysis').glob(f'*_{s}_0')))
            gt, _ = gt_tracks({'person': _MOCAP[s]['person']}, dyn_speed=thr)
            p, _, _ = load_obstacles(REPLAY_HOST / f'{tag}_v2_{s}')
            pred = tracks_to_mocap(p, np.array(A['T_mocap_odom'][s]), A['delta'][s])
            res.append(json.loads(json.dumps(evaluate(gt, pred, EvalConfig(roi=tuple(S['roi']))).to_dict(),
                                             default=float).replace('NaN', 'null')))
        out[thr] = state_acc(res)
    return out


def mota(results):
    return aggregate(results)['mota']


def state_acc(results):
    """Fraction of GT frames matched with the right state."""
    a = aggregate(results)
    c = a['confusion']
    ok = sum(c[s].get(s, 0) for s in ('dynamic', 'potentially_dynamic'))
    return ok / a['num_gt'] if a['num_gt'] else float('nan')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--param', required=True)
    ap.add_argument('--values', nargs='+', required=True)
    ap.add_argument('--inputs', default='r5', help='recorded full-pipeline runs <inputs>_v2_<seq> to replay')
    ap.add_argument('--gt-dir', default=str(REPO / 'eval' / 'results' / 'own'))
    ap.add_argument('--force', action='store_true')
    ap.add_argument('--gt-robustness', action='store_true',
                    help='also report state accuracy with GT dynamic thresholds 0.2 / 0.3 / 0.4 m/s')
    args = ap.parse_args()
    gt_dir = Path(args.gt_dir)

    R = {v: run_value(args.param, v, args.inputs, gt_dir, args.force) for v in args.values}
    print(f'== {args.param}')
    print(f'{"value":>8} {"MOTA":>7} {"IDF1":>7} {"stateAcc":>8} {"FP":>5} {"FN":>5} {"IDSW":>4} | per-sequence MOTA')
    for v, res in R.items():
        a = aggregate(list(res.values()))
        per = ' '.join(f'{res[s]["mota"] * 100:5.1f}' for s in SEQS)
        print(f'{v:>8} {a["mota"] * 100:7.2f} {a["idf1"] * 100:7.2f} {state_acc(list(res.values())) * 100:8.2f} '
              f'{a["fp"]:5d} {a["fn"]:5d} {a["idsw"]:4d} | {per}')
    if args.gt_robustness:
        print('state accuracy vs GT dynamic threshold (0.2 / 0.3 / 0.4 m/s):')
        for v in args.values:
            r = state_acc_by_gt_threshold(args.param, v, gt_dir)
            print(f'{v:>8} ' + ' '.join(f'{x * 100:6.2f}' for x in r.values()))
    # LOSO
    held = []
    picks = []
    for h in SEQS:
        train = [s for s in SEQS if s != h]
        best = max(args.values, key=lambda v: mota([R[v][s] for s in train]))
        picks.append(best)
        held.append(R[best][h])
    best_all = max(args.values, key=lambda v: mota(list(R[v].values())))
    print(f'LOSO picks per held-out sequence: {dict(zip([s[-4:] for s in SEQS], picks))}')
    print(f'LOSO MOTA {mota(held) * 100:.2f}  vs  in-sample best ({best_all}) {mota(list(R[best_all].values())) * 100:.2f}')


if __name__ == '__main__':
    main()
