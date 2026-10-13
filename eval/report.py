#!/usr/bin/env python3
"""
Tables from run_own_eval.py's summary.json: CSV + LaTeX (booktabs).

Per-sequence rows, plus an 'all' row per method computed by SUMMING the raw
counts over sequences (TP/FP/FN/IDSW/IDTP/confusion) and recomputing the ratios
-- not by averaging per-sequence percentages, which would weight a 30 s and a
70 s sequence equally.

Usage
  python3 eval/report.py [--summary eval/results/own/summary.json] [--out eval/tables]
"""

import argparse
import csv
import json
import math
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
STATES = ('dynamic', 'potentially_dynamic', 'static')
LEVEL = {'full': 'D/PD/S labels', 'dynamic_only': 'moving objects only (D vs rest)',
         'merged': 'D and PD merged (people/movable objects, moving or not)'}
SHORT = {'dynamic': 'D', 'potentially_dynamic': 'PD', 'static': 'S'}


def nan(x):
    return float('nan') if x is None else x


def aggregate(results):
    """Sum raw counts of several EvalResult dicts and recompute the ratios."""
    a = {k: sum(r[k] for r in results) for k in ('num_frames', 'num_gt', 'num_pred', 'tp', 'fp', 'fn', 'idsw', 'idtp')}
    a['mota'] = 1 - (a['fn'] + a['fp'] + a['idsw']) / a['num_gt'] if a['num_gt'] else float('nan')
    d = sum(nan(r['motp']) * r['tp'] for r in results if r['tp'])
    a['motp'] = d / a['tp'] if a['tp'] else float('nan')
    a['idf1'] = 2 * a['idtp'] / (a['num_gt'] + a['num_pred']) if (a['num_gt'] + a['num_pred']) else float('nan')
    conf = {s: {} for s in STATES}
    for r in results:
        for g, row in r['confusion'].items():
            for p, n in row.items():
                conf[g][p] = conf[g].get(p, 0) + n
    a['confusion'] = conf
    a['state_recall'] = {s: (conf[s].get(s, 0) / sum(conf[s].values()) if sum(conf[s].values()) else float('nan'))
                         for s in STATES}
    lats = [l for r in results for l in r['latencies']]
    a['latency_mean'], a['latency_censored'] = {}, {}
    for to in ('dynamic', 'potentially_dynamic'):
        ls = [nan(l['latency']) for l in lats if l['to'] == to]
        ok = [x for x in ls if not math.isnan(x)]
        a['latency_mean'][f'to_{to}'] = sum(ok) / len(ok) if ok else float('nan')
        a['latency_censored'][f'to_{to}'] = f'{len(ls) - len(ok)}/{len(ls)}'
    return a


def row(method, seq, r):
    return {
        'method': method, 'sequence': seq, 'frames': r['num_frames'],
        'MOTA': r['mota'], 'MOTP_m': r['motp'], 'IDF1': r['idf1'],
        'FP': r['fp'], 'FN': r['fn'], 'IDSW': r['idsw'],
        'recall_D': r['state_recall']['dynamic'], 'recall_PD': r['state_recall']['potentially_dynamic'],
        'lat_to_D_s': r['latency_mean']['to_dynamic'], 'cens_to_D': r['latency_censored']['to_dynamic'],
        'lat_to_PD_s': r['latency_mean']['to_potentially_dynamic'],
        'cens_to_PD': r['latency_censored']['to_potentially_dynamic'],
    }


def fmt(v, pct=False, nd=2):
    if isinstance(v, str):
        return v
    v = nan(v)
    if isinstance(v, float) and math.isnan(v):
        return '--'
    if pct:
        return f'{100 * v:.1f}'
    return f'{v:.{nd}f}' if isinstance(v, float) else str(v)


def latex(rows, caption, label):
    cols = [('method', 'Method', None), ('sequence', 'Seq.', None), ('MOTA', 'MOTA\\,\\%', 'pct'),
            ('IDF1', 'IDF1\\,\\%', 'pct'), ('MOTP_m', 'MOTP [m]', 'f'), ('FP', 'FP', None), ('FN', 'FN', None),
            ('IDSW', 'IDSW', None), ('recall_D', 'Rec.\\ D\\,\\%', 'pct'), ('recall_PD', 'Rec.\\ PD\\,\\%', 'pct'),
            ('lat_to_D_s', '$\\to$D [s]', 'f'), ('lat_to_PD_s', '$\\to$PD [s]', 'f')]
    out = ['\\begin{table}[t]', '\\centering', f'\\caption{{{caption}}}', f'\\label{{{label}}}',
           '\\setlength{\\tabcolsep}{3pt}', '\\begin{tabular}{ll' + 'r' * (len(cols) - 2) + '}', '\\toprule',
           ' & '.join(c[1] for c in cols) + ' \\\\', '\\midrule']
    prev = None
    for r in rows:
        if prev is not None and r['method'] != prev:
            out.append('\\midrule')
        prev = r['method']
        cells = [fmt(r[k], pct=(f == 'pct')).replace('_', '\\_') for k, _, f in cols]
        if r['sequence'] == 'all':
            cells = [f'\\textbf{{{c}}}' for c in cells]
        out.append(' & '.join(cells) + ' \\\\')
    out += ['\\bottomrule', '\\end{tabular}', '\\end{table}']
    return '\n'.join(out) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--summary', default=str(REPO / 'eval' / 'results' / 'own' / 'summary.json'))
    ap.add_argument('--out', default=str(REPO / 'eval' / 'tables'))
    ap.add_argument('--name', default='own_mocap', help='output file stem')
    args = ap.parse_args()
    S = json.loads(Path(args.summary).read_text())
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    main_thr = str(S['dyn_speed'][0])

    rows, sens = [], []
    for method, M in S['methods'].items():
        per = {seq: v[main_thr] for seq, v in sorted(M['per_seq'].items())}
        for seq, r in per.items():
            rows.append(row(method, seq, r))
        rows.append(row(method, 'all', aggregate(list(per.values()))))
        for thr in S['dyn_speed']:
            a = aggregate([v[str(thr)] for v in M['per_seq'].values()])
            sens.append({'method': method, 'dyn_speed': thr, 'recall_D': a['state_recall']['dynamic'],
                         'recall_PD': a['state_recall']['potentially_dynamic'],
                         'lat_to_D_s': a['latency_mean']['to_dynamic'],
                         'lat_to_PD_s': a['latency_mean']['to_potentially_dynamic']})

    with open(out / f'{args.name}.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(out / f'{args.name}_sensitivity.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(sens[0]))
        w.writeheader()
        w.writerows(sens)
    (out / f'{args.name}.tex').write_text(latex(
        rows, f'Lab sequences with motion-capture ground truth, {LEVEL.get(S.get("label_mode", "full"), "")} '
              f'(GT dynamic threshold {main_thr}\\,m/s, '
              f'match gate {S["match_dist"]}\\,m'
              + (f', first {S["t_start"]:g}\\,s of each sequence excluded' if S.get('t_start') else '') + ').',
        f'tab:{args.name}'))

    print(' '.join(f'{k[:10]:>10}' for k in rows[0]))
    for r in rows:
        cells = {k: fmt(r[k], pct=k in ('MOTA', 'IDF1', 'recall_D', 'recall_PD')) for k in r}
        cells['sequence'] = cells['sequence'][-4:]   # 20260519_1945 -> 1945
        print(' '.join(f'{v[:10]:>10}' for v in cells.values()))
    print(f'\nsensitivity (GT dynamic threshold):')
    for s in sens:
        print(f"  {s['method']} thr={s['dyn_speed']}: recall D={fmt(s['recall_D'], True)}% "
              f"PD={fmt(s['recall_PD'], True)}% lat->D={fmt(s['lat_to_D_s'])}s lat->PD={fmt(s['lat_to_PD_s'])}s")
    print(f'wrote {out}/{args.name}.{{csv,tex}}, {args.name}_sensitivity.csv')


if __name__ == '__main__':
    main()
