#!/usr/bin/env python3
"""
Map experiment table: eval/results/map_<C>_<seq>.json (eval/map_eval.py) -> eval/tables/map_<C>.csv and a printout.
Per variant: ghost voxels per sequence and summed, person removed = 1 - ghost / ghost of the no-rejection map
(summed over the sequences), static kept per sequence (min) and mean. The repeat of the no-rejection run (none2) is
GLIM's run-to-run noise on both numbers.

usage: python3 eval/map_table.py [--campaign r9a]
"""

import argparse
import csv
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SEQS = ['20260519_1942', '20260519_1945', '20260519_1947', '20260519_1949', '20260519_1950', '20260519_1952']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default='r9a')
    args = ap.parse_args()
    res = {s: json.loads((REPO / 'eval' / 'results' / f'map_{args.campaign}_{s}.json').read_text()) for s in SEQS}
    variants = list(res[SEQS[0]])
    rows = []
    for v in variants:
        g = {s[-4:]: res[s][v]['ghost_voxels'] for s in SEQS}
        k = {s[-4:]: res[s][v]['static_kept'] for s in SEQS}
        g0 = sum(res[s]['none']['ghost_voxels'] for s in SEQS)
        row = dict(variant=v, **{f'ghost_{s}': n for s, n in g.items()}, ghost_total=sum(g.values()),
                   person_removed=1 - sum(g.values()) / g0, **{f'static_kept_{s}': x for s, x in k.items()},
                   static_kept_mean=sum(k.values()) / len(k), static_kept_min=min(k.values()))
        rows.append(row)
        print(f"{v:9s} ghost {' '.join(f'{n:5d}' for n in g.values())}  total {row['ghost_total']:6d}  "
              f"removed {100 * row['person_removed']:5.1f}%  static kept mean {100 * row['static_kept_mean']:.1f}% "
              f"min {100 * row['static_kept_min']:.1f}%")
    with open(REPO / 'eval' / 'tables' / f'map_{args.campaign}.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    main()
