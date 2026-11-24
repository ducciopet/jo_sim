#!/usr/bin/env python3
"""Claims vs a baseline (PIPELINE_FINAL.md section 2, FINAL_RUN_SPEC.md 3.3): paired cluster bootstrap over windows of
the pooled-MOTA difference (method mean over reps - baseline mean over reps), plus a sign test over window x rep pairs.

Pooled MOTA = 1 - sum(fp+fn+idsw) / sum(num_gt) over the resampled windows (metrics.merged of coda_eval.py JSONs).
"better" only if the 95% CI excludes 0. Also used for fix adoption (--adopt): sign test >= 19/27-style threshold.

usage: bootstrap_claims.py --dir eval/results/coda_e3/test --a EB EBr2 EBr3 --b EMMC \
          --windows w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1 [--mode merged] [--n 10000] [--seed 0]
Pure analysis of existing JSONs (no runs).
"""
import argparse
import json
from math import comb
from pathlib import Path

import numpy as np


def load(d, tag, w, mode):
    m = json.load(open(Path(d) / f'{tag}_{w}.json'))['metrics'][mode]
    return np.array([m['fp'] + m['fn'] + m['idsw'], m['num_gt']], float)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dir', required=True)
    ap.add_argument('--a', nargs='+', required=True, help='method tags (reps)')
    ap.add_argument('--b', nargs='+', required=True, help='baseline / reference tags (reps; one if deterministic)')
    ap.add_argument('--windows', nargs='+', required=True)
    ap.add_argument('--mode', default='merged')
    ap.add_argument('--n', type=int, default=10000)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    W = a.windows
    A = np.array([[load(a.dir, t, w, a.mode) for w in W] for t in a.a])   # reps x windows x (err, gt)
    B = np.array([[load(a.dir, t, w, a.mode) for w in W] for t in a.b])

    def pooled(X, idx):           # mean over reps of the pooled MOTA on windows idx
        s = X[:, idx, :].sum(axis=1)
        return float(np.mean(100 * (1 - s[:, 0] / s[:, 1])))

    allw = np.arange(len(W))
    d0 = pooled(A, allw) - pooled(B, allw)
    rng = np.random.default_rng(a.seed)
    boot = np.array([pooled(A, i) - pooled(B, i) for i in (rng.integers(0, len(W), len(W)) for _ in range(a.n))])
    lo, hi = np.percentile(boot, [2.5, 97.5])
    # sign test: every method rep vs every baseline rep, per window, on the error count (same GT)
    wins = losses = 0
    for ra in A:
        for rb in B:
            for k in allw:
                wins += ra[k, 0] < rb[k, 0]
                losses += ra[k, 0] > rb[k, 0]
    n = wins + losses
    p = sum(comb(n, j) for j in range(wins, n + 1)) / 2 ** n if n else 1.0
    print(f'pooled MOTA: A {pooled(A, allw):.1f}  B {pooled(B, allw):.1f}  diff {d0:+.1f}  95% CI [{lo:+.1f}, {hi:+.1f}]  '
          f'P(diff<=0) {np.mean(boot <= 0):.3f}')
    print(f'sign test (window x rep pairs, ties dropped): A better in {wins}/{n}, one-sided p {p:.3f}')
    print('verdict:', 'A better' if lo > 0 else ('B better' if hi < 0 else 'comparable'))
    for k, w in enumerate(W):
        sa = A[:, k, :]
        sb = B[:, k, :]
        print(f'  {w:7s} gt {int(sa[0, 1]):5d}  A {np.mean(100 * (1 - sa[:, 0] / sa[:, 1])):6.1f}  '
              f'B {np.mean(100 * (1 - sb[:, 0] / sb[:, 1])):6.1f}')


if __name__ == '__main__':
    main()
