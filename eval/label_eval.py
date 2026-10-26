#!/usr/bin/env python3
"""
Per-label scores (D, PD, S) for every method, v1/v2 and the literature baselines, on the same footing.

D and PD (the person): harness/metrics.evaluate with label_mode 'full', confusion summed over the 6 sequences.
Same definitions as the r9 report (campaign_report.metrics_of): precision = share of the matched frames given label X
whose GT is X; recall = share of the GT-X frames given X (unmatched GT counts as missed); F1 of the two. Also where
the GT-X frames went: labelled D, labelled PD, missed.
S (everything that is not the person): eval/static_eval.py (count_seq/scores): S->D/PD = false dynamics on static
objects, S precision/recall/F1, false dynamics per minute.

The baselines publish only what they flag as moving (all 'dynamic'): none has a PD state, so their PD recall is 0
by construction and a person standing still counts as "missed" (or as D when the method keeps it moving).

Writes eval/results/labels_<C>.json and eval/tables/labels_<C>.csv (mean/std over reps).
Usage
  python3 eval/label_eval.py --campaign r9 --reps a b c
"""

import argparse
import csv
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import baseline_eval as be  # noqa: E402
import static_eval as se  # noqa: E402
from harness.gt_mocap import tracks_to_mocap  # noqa: E402
from harness.metrics import EvalConfig, evaluate  # noqa: E402
from harness.tracks import read_tracks  # noqa: E402

LABELS = {'dynamic': 'D', 'potentially_dynamic': 'PD'}
METHODS = ['v2', 'v1', 'lvdot', 'dynablox', 'mdetector', 'xurgbd', 'xuyolo', 'unimt', 'eagermot']


def person_scores(conf):
    out = {}
    for s, k in LABELS.items():
        row = conf.get(s, {})
        n_gt = sum(row.values())
        pred_s = sum(conf[g].get(s, 0) for g in conf)
        p = conf.get(s, {}).get(s, 0) / pred_s if pred_s else float('nan')
        r = row.get(s, 0) / n_gt if n_gt else float('nan')
        out[f'prec_{k}'], out[f'recall_{k}'] = p, r
        out[f'f1_{k}'] = 2 * p * r / (p + r) if p == p and r == r and p + r > 0 else float('nan')
        for t, tk in (('dynamic', 'D'), ('potentially_dynamic', 'PD'), ('missed', 'miss')):
            out[f'{k}_as_{tk}'] = row.get(t, 0) / n_gt if n_gt else float('nan')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default='r9')
    ap.add_argument('--reps', nargs='+', default=['a', 'b', 'c'])
    ap.add_argument('--methods', nargs='+', default=METHODS)
    ap.add_argument('--dup-dist', type=float, default=1.0)
    args = ap.parse_args()
    c = args.campaign
    per = {}  # method -> rep -> scores
    for r in args.reps:
        res = be.REPO / 'eval' / 'results' / f'own_{c}{r}'
        S = json.loads((res / 'summary.json').read_text())
        al = json.loads((res / 'alignment_v2.json').read_text())
        cfg = EvalConfig(match_dist=S['match_dist'], roi=tuple(S['roi']), t_start=S['t_start'], label_mode='full')
        gts, fused = {}, {}
        for s in be.SEQS:
            gts[s] = read_tracks(res / s / 'gt' / 'tracks.csv')
            fused[s] = tracks_to_mocap(se.load_fused(be.REPO / 'bags' / 'replay' / f'{c}{r}_v2_{s}'),
                                       al['T_mocap_odom'][s], al['delta'][s])
        for m in args.methods:
            conf, n = {}, None
            preds = {s: be.predictions(m, c, r, s, res, al) for s in be.SEQS}
            if any(p is None for p in preds.values()):
                print(f'rep {r} {m}: incomplete, skipped')
                continue
            for s in be.SEQS:
                e = evaluate(gts[s], preds[s], cfg).to_dict()
                for g, row in e['confusion'].items():
                    for k, v in row.items():
                        conf.setdefault(g, {}).setdefault(k, 0)
                        conf[g][k] += v
                ns = se.count_seq(gts[s], preds[s], fused[s], cfg, args.dup_dist)
                n = ns if n is None else {k: n[k] + v for k, v in ns.items()}
            st = se.scores(n)
            per.setdefault(m, {})[r] = dict(person_scores(conf), prec_S=st['prec_S'], recall_S=st['recall_S'],
                                            f1_S=st['f1_S'], false_dyn_per_min=st['false_dyn_per_min'],
                                            S_D=st['S_D'], S_PD=st['S_PD'], S_objects=st['S_objects'])
            print(f'{c}{r} {m}: done')
    rows = []
    for m, reps in per.items():
        row = {'method': m, 'reps': len(reps)}
        for k in next(iter(reps.values())):
            xs = [v[k] for v in reps.values() if v[k] == v[k]]
            row[f'{k}_mean'] = statistics.fmean(xs) if xs else float('nan')
            row[f'{k}_std'] = (statistics.stdev(xs) if len(xs) > 1 else 0.0) if xs else float('nan')
        rows.append(row)
        f = lambda k: f"{100 * row[k + '_mean']:.1f}"
        print(f"{m:9s} D P/R/F1 {f('prec_D')}/{f('recall_D')}/{f('f1_D')}  PD P/R/F1 {f('prec_PD')}/{f('recall_PD')}/"
              f"{f('f1_PD')}  S P/R/F1 {f('prec_S')}/{f('recall_S')}/{f('f1_S')}  "
              f"false dyn/min {row['false_dyn_per_min_mean']:.1f}")
    (be.REPO / 'eval' / 'results' / f'labels_{c}.json').write_text(json.dumps(per, indent=1))
    with open(be.REPO / 'eval' / 'tables' / f'labels_{c}.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    main()
