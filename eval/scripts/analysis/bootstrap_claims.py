#!/usr/bin/env python3
"""Claims vs a baseline (FINAL_RUN_SPEC.md 3.3, paper Sec. 5 statistics), on pooled MOTA from coda_eval.py JSONs.

Rule (agreed 25/11 with the paper session, after the external review):
  - reps are AVERAGED inside each window first (replays of the same data are not independent samples): per window the
    error rate e = (fp+fn+idsw)/num_gt of each method is the mean over its reps;
  - exact paired sign test and exact paired permutation test (sign flips of the per-window differences of MOTA, weighted
    by num_gt so the statistic is the pooled-MOTA difference) on the window means, two-sided;
  - paired cluster bootstrap of the pooled-MOTA difference, cluster = --cluster (window or sequence), only if the number
    of clusters is >= 10 (otherwise no CI, no better/worse wording from this split alone);
  - verdict: "better"/"worse" only if the 95% CI excludes 0 on that side AND both exact tests have p < 0.05 with the same
    sign; otherwise "comparable".
Windows of several datasets can be pooled (pass --dir several times with --windows given as DIR:WINDOW or a common list).

usage: bootstrap_claims.py --dir eval/results/coda_e3/test --a SF SFr2 SFr3 --b EMMC \
          --windows w1 w2 w3 10_w1 ... [--cluster window|sequence] [--mode merged] [--n 10000]
Pure analysis of existing JSONs (no runs).
"""
import argparse
import itertools
import json
import re
from math import comb
from pathlib import Path

import numpy as np


def load(d, tag, w, mode):
    m = json.load(open(Path(d) / f'{tag}_{w}.json'))['metrics'][mode]
    return m['fp'] + m['fn'] + m['idsw'], m['num_gt']


def seq_of(w):
    m = re.match(r'(.+)_w\d+$', w)
    return m.group(1) if m else '0'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dir', required=True)
    ap.add_argument('--a', nargs='+', required=True, help='method tags (reps)')
    ap.add_argument('--b', nargs='+', required=True, help='baseline tags (reps; one if deterministic)')
    ap.add_argument('--windows', nargs='+', required=True)
    ap.add_argument('--cluster', choices=['window', 'sequence'], default='sequence')
    ap.add_argument('--mode', default='merged')
    ap.add_argument('--n', type=int, default=10000)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    W = a.windows
    gt = np.array([load(a.dir, a.a[0], w, a.mode)[1] for w in W], float)
    ea = np.array([np.mean([load(a.dir, t, w, a.mode)[0] for t in a.a]) for w in W])   # mean errors over reps
    eb = np.array([np.mean([load(a.dir, t, w, a.mode)[0] for t in a.b]) for w in W])

    def pooled(e, idx):
        return 100 * (1 - e[idx].sum() / gt[idx].sum())

    allw = np.arange(len(W))
    d0 = pooled(ea, allw) - pooled(eb, allw)
    # exact sign test on per-window MOTA differences (ties dropped)
    dm = (eb - ea) / gt                     # > 0: A better in that window
    wins, losses = int(np.sum(dm > 0)), int(np.sum(dm < 0))
    n = wins + losses
    k = max(wins, losses)
    p_sign = min(1.0, 2 * sum(comb(n, j) for j in range(k, n + 1)) / 2 ** n) if n else 1.0
    # exact permutation test (sign flips of per-window error differences; statistic = pooled-MOTA difference)
    diff = eb - ea
    stat = diff.sum()
    if len(W) <= 20:
        flips = np.array(list(itertools.product([1, -1], repeat=len(W))))
    else:
        flips = np.random.default_rng(a.seed).choice([1, -1], size=(100000, len(W)))
    p_perm = float(np.mean(np.abs(flips @ diff) >= abs(stat) - 1e-9))
    # cluster bootstrap
    clusters = [[k] for k in allw] if a.cluster == 'window' else \
        [list(g) for _, g in itertools.groupby(sorted(allw, key=lambda k: seq_of(W[k])), key=lambda k: seq_of(W[k]))]
    ci = None
    if len(clusters) >= 10:
        rng = np.random.default_rng(a.seed)
        boot = []
        for _ in range(a.n):
            idx = np.concatenate([clusters[c] for c in rng.integers(0, len(clusters), len(clusters))])
            boot.append(pooled(ea, idx) - pooled(eb, idx))
        ci = np.percentile(boot, [2.5, 97.5])
    if ci is None:
        verdict = 'comparable (fewer than 10 clusters: per-sequence values and exact tests only)'
    elif ci[0] > 0 and p_sign < 0.05 and p_perm < 0.05 and wins > losses:
        verdict = 'A better'
    elif ci[1] < 0 and p_sign < 0.05 and p_perm < 0.05 and losses > wins:
        verdict = 'A worse'
    else:
        verdict = 'comparable'
    print(f'pooled MOTA (reps averaged per window): A {pooled(ea, allw):.1f}  B {pooled(eb, allw):.1f}  diff {d0:+.1f}')
    print(f'clusters ({a.cluster}): {len(clusters)}  95% CI: ' + (f'[{ci[0]:+.1f}, {ci[1]:+.1f}]' if ci is not None else 'n/a (< 10)'))
    print(f'exact sign test on windows: A better in {wins}/{n}, two-sided p {p_sign:.3f};  permutation test p {p_perm:.3f}')
    print('verdict:', verdict)
    for k, w in enumerate(W):
        print(f'  {w:10s} gt {int(gt[k]):5d}  A {100 * (1 - ea[k] / gt[k]):6.1f}  B {100 * (1 - eb[k] / gt[k]):6.1f}')


if __name__ == '__main__':
    main()
