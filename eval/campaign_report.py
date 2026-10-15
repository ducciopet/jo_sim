#!/usr/bin/env python3
"""
Evaluate a repeated benchmark campaign (eval/scripts/run_campaign.sh) and aggregate over repetitions.

For each repetition <rep> it links the finished runs of bags/replay/<C><rep>_{v1,v2}_<seq> (full pipeline) and
<C><rep>g_v2_<seq> (greedy_kuhn ablation, tracker-only replay) as v1 / v2 / v2g into bags/replay/eval_<C><rep>/,
runs run_own_eval.py into eval/results/own_<C><rep>/ and report.py into eval/tables/own_<C><rep>_<level>.
Then, per level, the overall metrics (counts summed over sequences, report.aggregate) of every repetition give
mean / std / min / max -> eval/tables/own_<C>_<level>_reps.{csv,tex}, plus per-sequence mean/std
-> own_<C>_<level>_reps_perseq.csv. Only repetitions where all sequences of a method finished are aggregated.

Usage
  python3 eval/campaign_report.py --campaign r8 --reps a b c [--skip-eval]
"""

import argparse
import csv
import json
import statistics
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from report import aggregate  # noqa: E402
from harness.ros_io import load_odometry  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
REPLAY = REPO / 'bags' / 'replay'
SEQS = ['20260519_1942', '20260519_1945', '20260519_1947', '20260519_1949', '20260519_1950', '20260519_1952']
METHODS = {'v1': '{c}{r}_v1_{s}', 'v2': '{c}{r}_v2_{s}', 'v2g': '{c}{r}g_v2_{s}'}
LEVELS = {'full': 'summary.json', 'dynamic_only': 'summary_dynamic_only.json', 'merged': 'summary_merged.json'}
MIN_COVERAGE = 0.97   # a run whose recorded odometry ends earlier than this fraction of the sensor bag was truncated


def coverage(run, seq):
    import yaml
    meta = yaml.safe_load(open(REPO / 'bags' / 'bags_benchmark' / f'duccio_test_{seq}_0' / 'metadata.yaml'))
    info = meta['rosbag2_bagfile_information']
    t0 = info['starting_time']['nanoseconds_since_epoch'] / 1e9
    return (load_odometry(run)[:, 0].max() - t0) / (info['duration']['nanoseconds'] / 1e9)


LABELS = {'dynamic': 'D', 'potentially_dynamic': 'PD', 'static': 'S'}
METRICS = [('MOTA', 'mota', True), ('IDF1', 'idf1', True), ('MOTP_m', 'motp', False), ('FP', 'fp', False),
           ('FN', 'fn', False), ('IDSW', 'idsw', False)]


def metrics_of(a):
    m = {k: a[src] for k, src, _ in METRICS}
    m['recall_D'] = a['state_recall']['dynamic']
    m['recall_PD'] = a['state_recall']['potentially_dynamic']
    m['recall_S'] = a['state_recall']['static']
    m['lat_to_D_s'] = a['latency_mean']['to_dynamic']
    # per-label scores on the matched frames (D/PD/S confusion summed over the sequences): precision = share of the
    # frames given label s whose GT is s, recall = share of the GT-s frames given s (unmatched GT counts as missed)
    conf = a['confusion']
    for s, k in LABELS.items():
        pred_s = sum(conf[g].get(s, 0) for g in conf)
        p = conf[s].get(s, 0) / pred_s if pred_s else float('nan')
        r = a['state_recall'][s]
        m[f'prec_{k}'], m[f'f1_{k}'] = p, (2 * p * r / (p + r) if p == p and r == r and p + r > 0 else float('nan'))
        for g, gk in LABELS.items():
            m[f'conf_{gk}_{k}'] = conf[g].get(s, 0)
        m[f'conf_{k}_miss'] = conf[s].get('missed', 0)
    return m


def stats(xs):
    xs = [x for x in xs if x is not None and x == x]
    if not xs:
        return {'mean': float('nan'), 'std': float('nan'), 'min': float('nan'), 'max': float('nan'), 'n': 0}
    return {'mean': statistics.fmean(xs), 'std': statistics.stdev(xs) if len(xs) > 1 else 0.0,
            'min': min(xs), 'max': max(xs), 'n': len(xs)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default='r8')
    ap.add_argument('--reps', nargs='+', default=['a', 'b', 'c'])
    ap.add_argument('--skip-eval', action='store_true', help='only aggregate existing results')
    args = ap.parse_args()
    c = args.campaign

    complete = {}   # rep -> methods with every sequence done
    for r in args.reps:
        d = REPLAY / f'eval_{c}{r}'
        d.mkdir(exist_ok=True)
        done = []
        for m, pat in METHODS.items():
            ok = all((REPLAY / (pat.format(c=c, r=r, s=s) + '.done')).exists() for s in SEQS)
            if ok:
                for s in SEQS:
                    cov = coverage(REPLAY / pat.format(c=c, r=r, s=s), s)
                    if cov < MIN_COVERAGE:
                        print(f'WARNING {pat.format(c=c, r=r, s=s)}: replay covered only {cov:.0%} of the bag, '
                              f'rep {r} of {m} left out (remove its .done and rerun run_campaign.sh)')
                        ok = False
            for s in SEQS:
                src, link = REPLAY / pat.format(c=c, r=r, s=s), d / f'{m}_{s}'
                if link.is_symlink():
                    link.unlink()
                if ok:
                    link.symlink_to(Path('..') / src.name)
            if ok:
                done.append(m)
        complete[r] = done
        print(f'rep {r}: complete methods {done}')
        if not done or args.skip_eval:
            continue
        out = REPO / 'eval' / 'results' / f'own_{c}{r}'
        subprocess.run([sys.executable, str(REPO / 'eval' / 'run_own_eval.py'), '--runs', str(d),
                        '--mocap', str(REPO / 'bags' / 'qualysis'), '--methods', *done,
                        '--dyn-speed', '0.3', '0.2', '0.4', '0.5', '--out', str(out)], check=True)
        for lvl, fn in LEVELS.items():
            subprocess.run([sys.executable, str(REPO / 'eval' / 'report.py'), '--summary', str(out / fn),
                            '--name', f'own_{c}{r}_{lvl}'], check=True, stdout=subprocess.DEVNULL)

    tables = REPO / 'eval' / 'tables'
    agg_json = {}
    for lvl, fn in LEVELS.items():
        rows, perseq = [], []
        agg_json[lvl] = {}
        for m in METHODS:
            reps = [r for r in args.reps if m in complete[r]
                    and (REPO / 'eval' / 'results' / f'own_{c}{r}' / fn).exists()]
            if not reps:
                continue
            per_rep, per_seq_rep = {}, {s: {} for s in SEQS}
            for r in reps:
                S = json.loads((REPO / 'eval' / 'results' / f'own_{c}{r}' / fn).read_text())
                ps = S['methods'][m]['per_seq']
                per_rep[r] = metrics_of(aggregate([ps[s]['0.3'] for s in SEQS]))
                for s in SEQS:
                    per_seq_rep[s][r] = metrics_of(aggregate([ps[s]['0.3']]))
            agg_json[lvl][m] = {'reps': reps, 'per_rep': per_rep, 'per_seq_rep': per_seq_rep}
            keys = list(next(iter(per_rep.values())))
            row = {'method': m, 'reps': len(reps)}
            for k in keys:
                st = stats([per_rep[r][k] for r in reps])
                row.update({f'{k}_mean': st['mean'], f'{k}_std': st['std'], f'{k}_min': st['min'],
                            f'{k}_max': st['max']})
            rows.append(row)
            for s in SEQS:
                prow = {'method': m, 'sequence': s, 'reps': len(reps)}
                for k in ('MOTA', 'IDF1', 'FP', 'FN', 'IDSW'):
                    st = stats([per_seq_rep[s][r][k] for r in reps])
                    prow.update({f'{k}_mean': st['mean'], f'{k}_std': st['std']})
                perseq.append(prow)
        if not rows:
            continue
        name = f'own_{c}_{lvl}_reps'
        with open(tables / f'{name}.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        with open(tables / f'{name}_perseq.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(perseq[0]))
            w.writeheader()
            w.writerows(perseq)
        (tables / f'{name}.tex').write_text(latex(rows, lvl, name))
        print(f'\n[{lvl}]  (mean ± std over repetitions)')
        for r in rows:
            print(f"  {r['method']:4s} n={r['reps']}  MOTA {100*r['MOTA_mean']:.2f}±{100*r['MOTA_std']:.2f}  "
                  f"IDF1 {100*r['IDF1_mean']:.2f}±{100*r['IDF1_std']:.2f}  IDSW {r['IDSW_mean']:.1f}±{r['IDSW_std']:.1f}  "
                  f"FP {r['FP_mean']:.1f}  FN {r['FN_mean']:.1f}  recD {100*r['recall_D_mean']:.1f}  "
                  f"recPD {100*r['recall_PD_mean']:.1f}  lat->D {r['lat_to_D_s_mean']:.2f}s")
    (REPO / 'eval' / 'results' / f'own_{c}_reps.json').write_text(json.dumps(agg_json, indent=1, default=str))


def latex(rows, lvl, name):
    def pm(r, k, pct):
        f = 100 if pct else 1
        nd = 1 if pct else (2 if k in ('MOTP_m', 'lat_to_D_s') else 1)
        return f"{f*r[k+'_mean']:.{nd}f}\\,$\\pm$\\,{f*r[k+'_std']:.{nd}f}"
    cols = [('MOTA', 'MOTA\\,\\%', True), ('IDF1', 'IDF1\\,\\%', True), ('MOTP_m', 'MOTP [m]', False),
            ('FP', 'FP', False), ('FN', 'FN', False), ('IDSW', 'IDSW', False),
            ('recall_D', 'Rec.\\ D\\,\\%', True), ('recall_PD', 'Rec.\\ PD\\,\\%', True),
            ('lat_to_D_s', '$\\to$D [s]', False)]
    label = {'v1': 'v1', 'v2': 'v2 (Hungarian)', 'v2g': 'v2 (greedy-Kuhn)'}
    out = ['\\begin{table}[t]', '\\centering',
           f'\\caption{{Lab sequences with motion-capture GT, level {lvl.replace("_", " ")}: mean $\\pm$ std over '
           f'{rows[0]["reps"]} full-pipeline repetitions (greedy-Kuhn: tracker-only replay on the same inputs).}}',
           f'\\label{{tab:{name}}}', '\\setlength{\\tabcolsep}{3pt}', '\\begin{tabular}{l' + 'r' * len(cols) + '}',
           '\\toprule', 'Method & ' + ' & '.join(c[1] for c in cols) + ' \\\\', '\\midrule']
    for r in rows:
        out.append(label.get(r['method'], r['method']) + ' & ' + ' & '.join(pm(r, k, p) for k, _, p in cols) + ' \\\\')
    out += ['\\bottomrule', '\\end{tabular}', '\\end{table}']
    return '\n'.join(out) + '\n'


if __name__ == '__main__':
    main()
