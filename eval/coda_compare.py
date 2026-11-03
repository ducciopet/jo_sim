#!/usr/bin/env python3
"""
SINS vs literature methods on the CODa windows: runs coda_eval.py for every (method run, window) not yet evaluated,
pools each method over the windows (coda_pool.pool), writes results/coda/compare.json and tables/coda_compare.csv.
--jrdb: the same on the six JRDB held-out sequences (range 15 m), results/jrdb/compare.json, tables/jrdb_compare.csv.
  --tune: the seven JRDB TUNING sequences (SINS runs TU0*) instead; --pano: predictions on movable non-person objects
  labelled by JRDB-PanoTrack (bags/jrdb/<seq>_pano.csv) are ignored, as JRDB's boxes cover people only.
--heldout: the CODa HELD-OUT windows of seqs 11/12; --seq10: seq 10 (second tuning sequence) (SINS runs H1*, frozen heldout-v5), compareH.json.
usage: coda_compare.py [--heldout | --jrdb [--tune] [--pano]] [--windows w1 w2 w3]
"""
import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

E = Path(__file__).resolve().parent
REPO = E.parent
sys.path.insert(0, str(E))
from coda_pool import pool  # noqa: E402

# name -> (coda_eval --method, run path template ({w} = window), sensors, note)
METHODS = {
    'SINS': [('sins', f'bags/coda/runs/{t}_v2_coda{{w}}', 'LiDAR + 2 cameras (ZED RGB-D, stereo cam0 with online depth)', 'outdoor profile (heldout-v5: bikes from YOLO score 0.3), one YOLO (instance seg) per camera')
             for t in ('T1', 'T1r2', 'T1r3')],
    'SINS (before)': [('sins', f'bags/coda/runs/{t}_v2_coda{{w}}', 'LiDAR + RGB-D', 'outdoor profile of 29/10')
                      for t in ('B1', 'B1r2', 'B1r3')],
    'Dynablox': [('dynablox', 'bags/dynablox_out/dynabloxa_coda{w}.bag', 'LiDAR', 'deterministic')],
    'M-detector': [('mdetector', 'bags/mdetector_out/mdetectora_coda{w}.bag', 'LiDAR', 'real time (1x)')],
    'M-detector 0.25x': [('mdetector', 'bags/mdetector_out/mdetectorslow_coda{w}.bag', 'LiDAR',
                          'played at 0.25x so that it processes every scan')],
    'LV-DOT': [('lvdot', 'bags/lvdot_out/lvdota_coda{w}.bag', 'LiDAR + RGB-D', '')],
    'CenterPoint + SimpleTrack': [('simpletrack', 'bags/unimt_out/unimt_coda{w}_tracks.csv', 'LiDAR',
                                   "UniMT authors' CODa weights and tracker; trained on CODa"),],
    'EagerMOT': [('eagermot', 'bags/eagermot_out/eagermot_coda{w}.csv', 'LiDAR + camera',
                  'CenterPoint (CODa) 3D + YOLO 2D, pedestrians')],
}

# JRDB held-out: nothing tuned on these sequences (SINS: jrdb_indoor / jrdb_outdoor profiles, chosen before any run)
CODA_TUNE10 = ['10_w1', '10_w2', '10_w3', '10_w4', '10_w5']   # second CODa tuning sequence since 03/11
CODA_HELDOUT = ['11_w1', '11_w2', '11_w3', '12_w1', '12_w2', '12_w3']
SINS_HELDOUT = [('sins', f'bags/coda/runs/{t}_v2_coda{{w}}', METHODS['SINS'][0][2], METHODS['SINS'][0][3])
                for t in ('H1', 'H1r2', 'H1r3')]
JRDB_SEQS = ['forbes', 'huangb', 'packard2', 'tressider2', 'clark0', 'clark1']
JRDB_TUNE = ['bytes', 'gateslab', 'nvidia', 'stlc', 'tressider1', 'memorial', 'hewlett']
SINS_TUNE = [('sins', f'bags/jrdb/runs/{t}_v2_jrdb_{{w}}', 'LiDAR (VLP-16) + 5 cameras, depth projected from the LiDAR',
              'heldout-v5 profiles (jrdb_indoor v3, jrdb_outdoor)') for t in ('TU0', 'TU0r2', 'TU0r3')]
METHODS_JRDB = {
    'SINS': [('sins', f'bags/jrdb/runs/{t}_v2_jrdb_{{w}}', 'LiDAR (VLP-16) + 5 cameras, depth projected from the LiDAR',
              'heldout-v5 (indoor runs J2 = v3 profile, outdoor runs J4), one YOLO (instance seg) per camera') for t in ('J5', 'J5r2', 'J5r3')],
    'Dynablox': [('dynablox', 'bags/dynablox_out/dynabloxa_jrdb_{w}.bag', 'LiDAR', 'deterministic')],
    'M-detector': [('mdetector', 'bags/mdetector_out/mdetectora_jrdb_{w}.bag', 'LiDAR', 'real time (1x)')],
}


def cw(w):
    """CODa window name -> '<seq>_<window>' (bare w1..w3 = seq 0)."""
    return w if '_' in w else f'0_{w}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--windows', nargs='+')
    ap.add_argument('--jrdb', action='store_true')
    ap.add_argument('--tune', action='store_true')
    ap.add_argument('--pano', action='store_true')
    ap.add_argument('--heldout', action='store_true')
    ap.add_argument('--seq10', action='store_true', help='CODa seq 10 (tuning), SINS runs H1*')
    a = ap.parse_args()
    ds = 'jrdb' if a.jrdb else 'coda'
    wins = a.windows or ((JRDB_TUNE if a.tune else JRDB_SEQS) if a.jrdb else (CODA_HELDOUT if a.heldout else CODA_TUNE10 if a.seq10 else ['w1', 'w2', 'w3']))
    sfx = ('H' if a.heldout else 'S10' if a.seq10 else '') + ('T' if a.tune else '') + ('pano' if a.pano else '')
    methods = dict(METHODS_JRDB, SINS=SINS_TUNE) if a.tune else (METHODS_JRDB if a.jrdb else METHODS)
    if a.heldout or a.seq10:
        methods = {k: v for k, v in dict(METHODS, SINS=SINS_HELDOUT).items() if k not in ('SINS (before)', 'M-detector 0.25x')}
    gt_of = (lambda w: REPO / f'bags/jrdb/{w}') if a.jrdb else (lambda w: REPO / f'bags/coda/coda{cw(w)}')
    out = {}
    for name, runs in methods.items():
        reps = []
        for k, (method, tpl, sensors, note) in enumerate(runs):
            tag = f"cmp{sfx}_{name.replace(' ', '').replace('.', '')}_{k}"
            for w in wins:
                run, res = REPO / tpl.format(w=w if a.jrdb else cw(w)), E / f'results/{ds}' / f'{tag}_{w}.json'
                if not run.exists():
                    print('missing', run)
                    continue
                if not res.exists():
                    subprocess.run([sys.executable, str(E / 'coda_eval.py'), '--method', method, '--run', str(run),
                                    '--gt', f'{gt_of(w)}_gt.csv', '--odom', f'{gt_of(w)}_odom', '--out', str(res),
                                    *(['--range', '15'] if a.jrdb else []),
                                    *(['--extra-ignore', f'{gt_of(w)}_pano.csv'] if a.pano else [])], check=True, capture_output=True)
            reps.append(pool(tag, ds))
        keys = [('merged', 'recall'), ('merged', 'precision'), ('merged', 'fp_min'), ('merged', 'mota'),
                ('merged', 'idf1'), ('dynamic_only', 'recall'), ('dynamic_only', 'precision'),
                ('dynamic_only', 'fp_min'), ('dynamic_only', 'mota'), ('dynamic_only', 'idf1')]
        row = dict(sensors=runs[0][2], note=runs[0][3], reps=len(reps))
        for m, k in keys:
            v = [r[m][k] for r in reps]
            row[f'{m}_{k}'], row[f'{m}_{k}_std'] = float(np.mean(v)), float(np.std(v))
        for s in ('dynamic', 'potentially_dynamic'):
            row[f'f1_{s}'] = float(np.mean([r['state_f1'][s] for r in reps]))
        for l in ('dynamic', 'potentially_dynamic'):
            for k in ('n', 'recall', 'other', 'missed', 'said', 'precision', 'false_min'):
                row[f'{l}_{k}'] = float(np.mean([r['per_label'][l][k] for r in reps]))
        out[name] = row
        print('   per label: ' + '  '.join(f"{l[:3]} n={row[f'{l}_n']:.0f} rec {row[f'{l}_recall']:.1%} other "
              f"{row[f'{l}_other']:.1%} missed {row[f'{l}_missed']:.1%} prec {row[f'{l}_precision']:.1%} "
              f"false/min {row[f'{l}_false_min']:.0f}" for l in ('dynamic', 'potentially_dynamic')))
        print(f"{name:18s} ALL rec {row['merged_recall']:.1%} FP/min {row['merged_fp_min']:.0f} MOTA {row['merged_mota']:.1%} "
              f"IDF1 {row['merged_idf1']:.1%} | MOVING rec {row['dynamic_only_recall']:.1%} prec "
              f"{row['dynamic_only_precision']:.1%} FP/min {row['dynamic_only_fp_min']:.0f} MOTA {row['dynamic_only_mota']:.1%}")
    json.dump(out, open(E / f'results/{ds}/compare{sfx}.json', 'w'), indent=1)
    with open(E / f'tables/{ds}_compare{sfx}.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['method', *next(iter(out.values())).keys()])
        w.writeheader()
        for k, v in out.items():
            w.writerow(dict(method=k, **v))


if __name__ == '__main__':
    main()
