#!/usr/bin/env python3
"""Score the baseline runs made on the laptop on 25-26/11 (tags F, F2, F3) without the NAS.

Lab (6 Qualisys sequences): same evaluator and levels as eval/baseline_eval.py, with the GT and the mocap<->odom
alignment of campaign own_r9a (same sequences, same shared odometry bags/replay/odom_<seq>); output bag names
<method><rep>_<seq>.bag in bags/<method>_out (xu -> xu_rgbd_out, prefix 'xu').
JRDB (6 held-out sequences): eval/coda_eval.py --ignore-undefined --range 15 --extra-ignore <seq>_pano.csv (as for SINS).

usage: score_laptop_baselines.py [--lab] [--jrdb]   -> eval/results/laptop_baselines/{lab.json,jrdb/<tag>_<seq>.json}
Pure evaluation of existing outputs.
"""
import argparse
import json
import statistics
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'eval'))
sys.path.insert(0, str(REPO / 'eval' / 'adapters'))
from harness.gt_mocap import tracks_to_mocap   # noqa: E402
from harness.metrics import EvalConfig, evaluate  # noqa: E402
from harness.tracks import read_tracks          # noqa: E402
from marker_adapter import load_marker_boxes    # noqa: E402
from points_adapter import load_dynamic_points  # noqa: E402

SEQS = ['20260519_1942', '20260519_1945', '20260519_1947', '20260519_1949', '20260519_1950', '20260519_1952']
JR = ['forbes', 'huangb', 'packard2', 'tressider2', 'clark0', 'clark1']
M = {'dynablox': ('dynablox_out', 'dynablox', '/motion_detector/visualization/clusters', True),
     'mdetector': ('mdetector_out', 'mdetector', '/m_detector/point_out', None),
     'lvdot': ('lvdot_out', 'lvdot', '/onboard_detector/dynamic_bboxes', False),
     'xu': ('xu_rgbd_out', 'xu', '/onboard_detector/dynamic_bboxes', False)}
OUT = REPO / 'eval' / 'results' / 'laptop_baselines'


def lab():
    res = REPO / 'eval' / 'results' / 'own_r9a'
    S = json.loads((res / 'summary.json').read_text())
    align = json.loads((res / 'alignment_v2.json').read_text())
    out = {}
    for m, (d, prefix, topic, ids) in M.items():
        for rep in ('F', 'F2', 'F3'):
            acc = {lvl: dict(tp=0, fp=0, fn=0, idsw=0, idtp=0, num_gt=0, num_pred=0, frames=0) for lvl in ('dynamic_only', 'merged')}
            for s in SEQS:
                bag = REPO / 'bags' / d / f'{prefix}{rep}_{s}.bag'
                if not bag.exists() or bag.stat().st_size < 10000:
                    raise SystemExit(f'missing {bag}')
                boxes = load_dynamic_points(bag, topic) if ids is None else load_marker_boxes(bag, topic, persistent_ids=ids)
                pred = tracks_to_mocap(boxes, align['T_mocap_odom'][s], align['delta'][s])
                gt = read_tracks(res / s / 'gt' / 'tracks.csv')
                for lvl in acc:
                    e = evaluate(gt, pred, EvalConfig(match_dist=S['match_dist'], roi=tuple(S['roi']), t_start=S['t_start'],
                                                      label_mode=lvl)).to_dict()
                    for k in acc[lvl]:
                        acc[lvl][k] += e['num_frames'] if k == 'frames' else e[k]
            for lvl, c in acc.items():
                minutes = c['frames'] / 10.0 / 60.0
                out.setdefault(m, {}).setdefault(lvl, {})[rep] = dict(
                    MODA=1 - (c['fn'] + c['fp']) / c['num_gt'], MOTA=1 - (c['fn'] + c['fp'] + c['idsw']) / c['num_gt'],
                    recall=c['tp'] / c['num_gt'], false_per_min=c['fp'] / minutes,
                    IDF1=2 * c['idtp'] / (c['num_gt'] + c['num_pred']) if ids else None, **c)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'lab.json').write_text(json.dumps(out, indent=1))
    for m, lv in out.items():
        for lvl, reps in lv.items():
            mo = [r['MOTA' if M[m][3] else 'MODA'] * 100 for r in reps.values()]
            print(f'lab {m:9s} {lvl:12s} {"MOTA" if M[m][3] else "MODA"} {statistics.fmean(mo):6.1f} [{min(mo):.1f}, {max(mo):.1f}]  '
                  f'recall {statistics.fmean(r["recall"] for r in reps.values()) * 100:.1f}')


def jrdb():
    (OUT / 'jrdb').mkdir(parents=True, exist_ok=True)
    for m in ('dynablox', 'mdetector'):
        d = M[m][0]
        for rep in ('F', 'F2', 'F3'):
            for s in JR:
                run = REPO / 'bags' / d / f'{m}{rep}_jrdb_{s}.bag'
                o = OUT / 'jrdb' / f'{m}{rep}_{s}.json'
                if o.exists():
                    continue
                cmd = [sys.executable, str(REPO / 'eval' / 'coda_eval.py'), '--ignore-undefined', '--range', '15',
                       '--method', m, '--run', str(run), '--gt', str(REPO / 'bags' / 'jrdb' / f'{s}_gt.csv'),
                       '--odom', str(REPO / 'bags' / 'jrdb' / f'{s}_odom'), '--points-min-cluster', '5', '--out', str(o)]
                pano = REPO / 'bags' / 'jrdb' / f'{s}_pano.csv'
                if pano.exists():
                    cmd += ['--extra-ignore', str(pano)]
                r = subprocess.run(cmd, capture_output=True, text=True)
                if r.returncode:
                    print('FAIL', m, rep, s, r.stderr[-300:])


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--lab', action='store_true')
    ap.add_argument('--jrdb', action='store_true')
    a = ap.parse_args()
    if a.lab:
        lab()
    if a.jrdb:
        jrdb()
