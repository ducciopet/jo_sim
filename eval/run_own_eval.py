#!/usr/bin/env python3
"""
Evaluate detector runs on the lab sequences with Qualisys motion-capture GT.

Inputs
  --runs   directory with one recorded bag per (method, sequence), named
           <method>_<seq> (e.g. bags/smoke/v2_20260519_1945), each holding the
           method's ObstacleArray output and the /odometry/filtered it consumed
           (see eval/scripts/smoke_test.sh, eval/scripts/run_qualisys_batch.sh)
  --mocap  directory with the matching Qualisys bags, *_<seq>_0
Outputs (under --out, default eval/results/own)
  <seq>/gt/tracks.csv, <seq>/<method>/tracks.csv   unified format, mocap frame
  <seq>/<method>/metrics.json, manifest.json
  alignment_<method>.json                           mocap<->odom fit per method
  summary.json                                      everything report.py needs

Usage
  python3 eval/run_own_eval.py --runs bags/smoke --mocap bags/qualysis --methods v1 v2
"""

import argparse
import datetime
import json
import platform
import re
import subprocess
from pathlib import Path

import numpy as np

from harness.gt_mocap import fit_alignment, gt_tracks, tracks_to_mocap
from harness.metrics import EvalConfig, evaluate
from harness.ros_io import load_mocap, load_obstacles, load_odometry
from harness.tracks import write_tracks

REPO = Path(__file__).resolve().parents[1]
METHOD_REPO = {'v1': 'jo_detector', 'v2': 'jo_detector_v2'}


def git_state(path):
    try:
        h = subprocess.run(['git', '-C', str(path), 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(['git', '-C', str(path), 'status', '--porcelain', '--untracked-files=no'],
                               capture_output=True, text=True).stdout.strip()
        return {'commit': h, 'dirty': bool(dirty)}
    except Exception as e:
        return {'error': str(e)}


def jsonable(o):
    if isinstance(o, dict):
        return {str(k): jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [jsonable(v) for v in o]
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, float)):
        return None if not np.isfinite(o) else float(o)
    if isinstance(o, np.integer):
        return int(o)
    return o


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', required=True)
    ap.add_argument('--mocap', required=True)
    ap.add_argument('--methods', nargs='+', default=['v1', 'v2'])
    ap.add_argument('--out', default=str(REPO / 'eval' / 'results' / 'own'))
    ap.add_argument('--person-body', default='person')
    ap.add_argument('--robot-body', default='JO')
    ap.add_argument('--dyn-speed', type=float, nargs='+', default=[0.3],
                    help='GT dynamic threshold(s) [m/s]; the first is the main one, the rest a sensitivity sweep')
    ap.add_argument('--match-dist', type=float, default=1.0)
    ap.add_argument('--roi-margin', type=float, default=0.5)
    ap.add_argument('--t-start', type=float, default=None,
                    help='skip the first N s of every sequence (e.g. online-calibration warm-up)')
    ap.add_argument('--t-base-jo', type=float, nargs=2, default=[0.0, 0.0], metavar=('X', 'Y'),
                    help="robot rigid body origin in base_link [m]; only used if the data can't observe it "
                         '(robot never rotates), see harness/gt_mocap.fit_alignment')
    args = ap.parse_args()

    runs, out = Path(args.runs), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    mocap = {}
    for d in sorted(Path(args.mocap).iterdir()):
        m = re.search(r'(\d{8}_\d{4})_0$', d.name)
        if m and d.is_dir():
            mocap[m.group(1)] = load_mocap(d)

    # ROI = mocap capture volume actually observed (all bodies, all sequences) + margin
    allxy = np.concatenate([b[:, 1:3] for bodies in mocap.values() for b in bodies.values()])
    roi = (allxy[:, 0].min() - args.roi_margin, allxy[:, 0].max() + args.roi_margin,
           allxy[:, 1].min() - args.roi_margin, allxy[:, 1].max() + args.roi_margin)

    summary = {'created': datetime.datetime.now().isoformat(timespec='seconds'), 'roi': roi, 't_start': args.t_start,
               'dyn_speed': args.dyn_speed, 'match_dist': args.match_dist, 'methods': {}, 'sequences': sorted(mocap)}

    for method in args.methods:
        seq_runs = {s: runs / f'{method}_{s}' for s in mocap if (runs / f'{method}_{s}').is_dir()}
        if not seq_runs:
            print(f'[{method}] no runs found, skipping')
            continue
        odom = {s: load_odometry(p) for s, p in seq_runs.items()}
        align = fit_alignment({s: (odom[s], mocap[s][args.robot_body]) for s in seq_runs},
                              t_base_jo_prior=args.t_base_jo)
        (out / f'alignment_{method}.json').write_text(json.dumps(jsonable(align), indent=2))
        print(f'[{method}] T_base_JO={np.round(align["T_base_JO"], 3)} '
              f'(translation observable: {align["t_base_JO_observable"]}) delta={align["delta"]["median"]:+.3f}s '
              f'rms_xy={ {k: round(v, 3) for k, v in align["rms_xy"].items()} }')

        summary['methods'][method] = {'alignment_rms_xy': align['rms_xy'],
                                      't_base_JO_observable': align['t_base_JO_observable'], 'per_seq': {}}
        for s, run in seq_runs.items():
            pred_odom, frame_id, topic = load_obstacles(run)
            if frame_id and frame_id != 'odom':
                raise ValueError(f'{run}: obstacles in frame {frame_id}, expected odom')
            pred = tracks_to_mocap(pred_odom, align['T_mocap_odom'][s], align['delta'][s])
            seq_dir = out / s
            (seq_dir / method).mkdir(parents=True, exist_ok=True)
            write_tracks(pred, seq_dir / method / 'tracks.csv')

            per_thr = {}
            for thr in args.dyn_speed:
                gt, ids = gt_tracks({args.person_body: mocap[s][args.person_body]}, dyn_speed=thr)
                if thr == args.dyn_speed[0]:
                    (seq_dir / 'gt').mkdir(parents=True, exist_ok=True)
                    write_tracks(gt, seq_dir / 'gt' / 'tracks.csv')
                cfg = EvalConfig(match_dist=args.match_dist, roi=roi, t_start=args.t_start)
                per_thr[thr] = evaluate(gt, pred, cfg).to_dict()
            main_res = per_thr[args.dyn_speed[0]]
            (seq_dir / method / 'metrics.json').write_text(json.dumps(jsonable(
                {'main': main_res, 'sensitivity': {str(k): v for k, v in per_thr.items()}}), indent=2))
            manifest = {
                'method': method, 'sequence': s, 'run_bag': str(run), 'output_topic': topic,
                'run_recorded': datetime.datetime.fromtimestamp(run.stat().st_mtime).isoformat(timespec='seconds'),
                'method_git_at_eval': git_state(REPO / METHOD_REPO.get(method, '.')),
                'eval_git': git_state(REPO),
                'eval_config': {'match_dist': args.match_dist, 'roi': roi, 'dyn_speed': args.dyn_speed,
                                't_start': args.t_start},
                'alignment': {'T_mocap_odom': align['T_mocap_odom'][s], 'delta': align['delta'][s],
                              'T_base_JO': align['T_base_JO'], 'rms_xy': align['rms_xy'][s],
                              't_base_JO_observable': align['t_base_JO_observable']},
                'host': platform.node(), 'evaluated': datetime.datetime.now().isoformat(timespec='seconds'),
            }
            (seq_dir / method / 'manifest.json').write_text(json.dumps(jsonable(manifest), indent=2))
            summary['methods'][method]['per_seq'][s] = {str(k): v for k, v in per_thr.items()}
            print(f'  {s}: MOTA={main_res["mota"]:.3f} IDF1={main_res["idf1"]:.3f} '
                  f'recall D={main_res["state_recall"]["dynamic"]:.2f} PD={main_res["state_recall"]["potentially_dynamic"]:.2f} '
                  f'FP={main_res["fp"]} FN={main_res["fn"]} IDSW={main_res["idsw"]}')

    (out / 'summary.json').write_text(json.dumps(jsonable(summary), indent=2))
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
