#!/usr/bin/env python3
"""Pool coda_eval.py results over windows: sums of TP/FP/FN/IDSW/IDTP per mode -> recall, precision, FP/min, MOTA, IDF1;
state F1 (D / PD) from the summed confusion of the full mode.
usage: coda_pool.py <tag> [<tag> ...]   (reads eval/results/coda/<tag>_w*.json)"""
import glob
import json
import sys
from pathlib import Path

import numpy as np

R = Path(__file__).resolve().parent / 'results/coda'


def pool(tag):
    files = sorted(glob.glob(str(R / f'{tag}_w*.json')))
    S = {m: dict(tp=0, fp=0, fn=0, idsw=0, idtp=0, num_gt=0, num_pred=0) for m in ('merged', 'dynamic_only')}
    conf, minutes, fps = None, 0.0, {}
    for f in files:
        d = json.load(open(f))
        for m in S:
            for k in S[m]:
                S[m][k] += d['metrics'][m][k]
        conf = conf or {g: dict.fromkeys(r, 0) for g, r in d['metrics']['full']['confusion'].items()}
        for g, r in d['metrics']['full']['confusion'].items():
            for k, v in r.items():
                conf[g][k] += v
        for k, v in d['metrics']['full'].get('fp_by_state', {}).items():
            fps[k] = fps.get(k, 0) + v
        minutes += d['gt']['annotated_frames'] / 600.0          # 10 Hz annotation
    out = {'windows': len(files)}
    for m, s in S.items():
        out[m] = dict(recall=s['tp'] / max(1, s['tp'] + s['fn']), precision=s['tp'] / max(1, s['tp'] + s['fp']),
                      fp_min=s['fp'] / minutes, mota=1 - (s['fn'] + s['fp'] + s['idsw']) / max(1, s['num_gt']),
                      idf1=2 * s['idtp'] / max(1, s['num_gt'] + s['num_pred']), idsw=s['idsw'])
    f1 = {}
    for lab in ('dynamic', 'potentially_dynamic'):       # among matched pairs, missed GT counted as wrong
        tp, p, g = conf[lab][lab], sum(r[lab] for r in conf.values()), sum(conf[lab].values())
        f1[lab] = 2 * tp / max(1, p + g)
    out['state_f1'] = f1
    # per label (full mode): of the GT rows with that label, found with it / found with the other / missed;
    # precision = predictions with that label on a GT with that label / all predictions with that label (unmatched too)
    lab = {}
    for l, other in (('dynamic', 'potentially_dynamic'), ('potentially_dynamic', 'dynamic')):
        n, said = sum(conf[l].values()), sum(r[l] for r in conf.values()) + fps.get(l, 0)
        lab[l] = dict(n=n, recall=conf[l][l] / max(1, n), other=conf[l][other] / max(1, n),
                      missed=conf[l]['missed'] / max(1, n), said=said,
                      precision=conf[l][l] / said if said else float('nan'), false_min=fps.get(l, 0) / minutes)
    out['per_label'] = lab
    return out


if __name__ == '__main__':
    for tag in sys.argv[1:]:
        o = pool(tag)
        m, d = o['merged'], o['dynamic_only']
        print(f"{tag:10s} ({o['windows']} win) ALL: rec {m['recall']:.1%} prec {m['precision']:.1%} FP/min {m['fp_min']:.0f} "
              f"MOTA {m['mota']:.1%} IDF1 {m['idf1']:.1%} IDSW {m['idsw']} | MOVING: rec {d['recall']:.1%} "
              f"prec {d['precision']:.1%} MOTA {d['mota']:.1%} IDF1 {d['idf1']:.1%} | F1 " +
              ' '.join(f'{k[:3]} {v:.1%}' for k, v in o['state_f1'].items()))
