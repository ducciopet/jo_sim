#!/usr/bin/env python3
"""
SINS vs literature methods on the CODa windows: runs coda_eval.py for every (method run, window) not yet evaluated,
pools each method over the windows (coda_pool.pool), writes results/coda/compare.json and tables/coda_compare.csv.
usage: coda_compare.py [--windows w1 w2 w3]
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
    'SINS': [('sins', f'bags/coda/runs/{t}_v2_coda0_{{w}}', 'LiDAR + RGB-D', '') for t in ('B1', 'B1r2', 'B1r3')],
    'Dynablox': [('dynablox', 'bags/dynablox_out/dynabloxa_coda0_{w}.bag', 'LiDAR', 'deterministic')],
    'M-detector': [('mdetector', 'bags/mdetector_out/mdetectora_coda0_{w}.bag', 'LiDAR', 'real time (1x)')],
    'M-detector 0.25x': [('mdetector', 'bags/mdetector_out/mdetectorslow_coda0_{w}.bag', 'LiDAR',
                          'played at 0.25x so that it processes every scan')],
    'LV-DOT': [('lvdot', 'bags/lvdot_out/lvdota_coda0_{w}.bag', 'LiDAR + RGB-D', '')],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--windows', nargs='+', default=['w1', 'w2', 'w3'])
    a = ap.parse_args()
    out = {}
    for name, runs in METHODS.items():
        reps = []
        for k, (method, tpl, sensors, note) in enumerate(runs):
            tag = f"cmp_{name.replace(' ', '').replace('.', '')}_{k}"
            for w in a.windows:
                run, res = REPO / tpl.format(w=w), E / 'results/coda' / f'{tag}_{w}.json'
                if not run.exists():
                    print('missing', run)
                    continue
                if not res.exists():
                    subprocess.run([sys.executable, str(E / 'coda_eval.py'), '--method', method, '--run', str(run),
                                    '--gt', str(REPO / f'bags/coda/coda0_{w}_gt.csv'),
                                    '--odom', str(REPO / f'bags/coda/coda0_{w}_odom'), '--out', str(res)],
                                   check=True, capture_output=True)
            reps.append(pool(tag))
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
    json.dump(out, open(E / 'results/coda/compare.json', 'w'), indent=1)
    with open(E / 'tables/coda_compare.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['method', *next(iter(out.values())).keys()])
        w.writeheader()
        for k, v in out.items():
            w.writerow(dict(method=k, **v))


if __name__ == '__main__':
    main()
