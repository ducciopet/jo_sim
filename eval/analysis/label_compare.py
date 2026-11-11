#!/usr/bin/env python3
"""
Per-label comparison of SINS and the baselines on every dataset with box GT (CODa tuning / held-out, JRDB, street).

For each method, files are pooled over the windows of a group (sums of counts, as coda_pool.py), per repetition; the
table reports the mean (and min-max over repetitions when there are several).
Labels:
  - agents (any label, 'merged' mode): precision, recall, F1, MOTA, IDF1, MOTP, FP/min;
  - D (moving GT vs predictions labelled DYNAMIC, 'dynamic_only' mode): precision, recall, F1, MOTA, IDF1, FP/min;
  - PD (still movable GT, from the 'full' confusion): recall = GT PD labelled PD / GT PD; precision = GT PD labelled PD
    / every prediction labelled PD (matched to any GT state, or matched to nothing); F1;
  - label swaps: GT D labelled PD, GT PD labelled D (share of the matched GT of that state);
  - S: no dataset annotates static structure with boxes, so S is scored by its errors only: false D and false PD per
    minute = predictions with that label on no annotated agent (bollards, walls, trees, parked things not annotated).
The baselines publish no state: every prediction is D (PD precision undefined).
usage: label_compare.py [--out eval/results/label_compare.json]
"""
import argparse
import glob
import json
import os
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
R = REPO / 'eval' / 'results'
RL = REPO / 'eval' / 'results_lab'      # lab PC runs (NAS claude_handoff/results_1111): SINS x3 of 10-11/11
WIN = {0: ['w1', 'w2', 'w3'], 10: ['w1', 'w2', 'w3', 'w4', 'w5'], 16: ['w1'], 11: ['w1', 'w2', 'w3'], 12: ['w1', 'w2', 'w3']}
# per-class GT (coda_vehicle: vehicles only) exists only for the windows with annotated vehicles
VWIN = {0: ['w1'], 10: ['w1', 'w3', 'w4', 'w5'], 16: ['w1'], 11: ['w1', 'w2'], 12: ['w2', 'w3']}
GROUPS = {'CODa tuning (0+10+16)': ('coda', [0, 10, 16]), 'CODa held-out (11+12)': ('coda', [11, 12]),
          'CODa 0': ('coda', [0]), 'CODa 10': ('coda', [10]), 'CODa 16': ('coda', [16]), 'CODa 11': ('coda', [11]), 'CODa 12': ('coda', [12]),
          'CODa vehicles, tuning': ('veh', [0, 10, 16]), 'CODa vehicles, held-out': ('veh', [11, 12]),
          'Street (MDP), all': ('street', ''), 'Street (MDP), vehicles': ('street', '_vehicle'), 'Street (MDP), persons': ('street', '_person')}


def sins(tags_tune, tags_held):
    def f(kind, rep, s, w):
        if kind == 'street':
            return None
        t = tags_held[rep] if s in (11, 12) else tags_tune[rep]
        if t is None:
            return None
        if kind == 'veh':
            return RL / 'coda_vehicle' / f'{t}_{s}_{w}.json'
        return RL / 'coda' / (f'{t}_{w}.json' if s == 0 else f'{t}_{s}_{w}.json')
    return f, len(tags_tune)


def street(profile):
    def f(kind, rep, s, w):
        return RL / 'mdp_lab' / f'mdp_street_{profile}{s}_r{rep + 1}.json' if kind == 'street' else None
    return f, 3


def motion(name):     # Dynablox / M-detector: faithful runs of 09/11 on every window (+ per-class vehicle GT)
    def f(kind, rep, s, w):
        if kind == 'coda':
            return R / 'coda' / f'cmpFa_{name}_{s}_{w}.json'
        if kind == 'veh':
            return RL / 'coda_vehicle' / f'{name}_{s}_{w}.json'
        return None
    return f, 1


def legacy(name):   # LV-DOT / EagerMOT / CenterPoint+SimpleTrack: per-seq comparison campaigns (no seq 16)
    def f(kind, rep, s, w):
        if kind != 'coda':
            return None
        rel = {0: f'cmp_{name}_0_{w}.json', 10: f'cmpS10_{name}_0_10_{w}.json',
               11: f'cmpH_{name}_0_11_{w}.json', 12: f'cmpH_{name}_0_12_{w}.json'}.get(s)
        return R / 'coda' / rel if rel else None
    return f, 1


def both(sins_spec, street_spec):
    (fa, n), (fb, _) = sins_spec, street_spec
    return (lambda kind, rep, s, w: fb(kind, rep, s, w) if kind == 'street' else fa(kind, rep, s, w)), n


METHODS = {
    'SINS V+J+SY+E': both(sins(['LVE', 'LVEr2', 'LVEr3'], ['LHVE', 'LHVEr2', 'LHVEr3']), street('vjsye')),
    'SINS V+J+SY': both(sins(['LVS', 'LVSr2', 'LVSr3'], ['LHVS', 'LHVSr2', 'LHVSr3']), street('vjsy')),
    'SINS V+J': both(sins(['LVJ', 'LVJr2', 'LVJr3'], [None, None, None]), street('vj')),
    'SINS L3 (10/11 base)': both(sins(['LL3b', 'LL3br2', 'LL3br3'], ['LHL3', 'LHL3r2', 'LHL3r3']), street('lt2')),
    'Dynablox': motion('dynablox'),
    'M-detector': motion('mdetector'),
    'LV-DOT': legacy('LV-DOT'),
    'EagerMOT': legacy('EagerMOT'),
    'CenterPoint+SimpleTrack': legacy('CenterPoint+SimpleTrack'),
}


def pool(files, hz=10.0):
    S = {m: dict(tp=0, fp=0, fn=0, idsw=0, idtp=0, num_gt=0, num_pred=0, motp_w=0.0) for m in ('merged', 'dynamic_only')}
    conf, fps, minutes = {}, {}, 0.0
    for f in files:
        d = json.load(open(f))
        for m in S:
            mm = d['metrics'][m]
            for k in ('tp', 'fp', 'fn', 'idsw', 'idtp', 'num_gt', 'num_pred'):
                S[m][k] += mm[k]
            if mm['tp'] and mm['motp'] == mm['motp']:
                S[m]['motp_w'] += mm['motp'] * mm['tp']
        for g, r in d['metrics']['full']['confusion'].items():
            for k, v in r.items():
                conf.setdefault(g, {}).setdefault(k, 0)
                conf[g][k] += v
        for k, v in d['metrics']['full'].get('fp_by_state', {}).items():
            fps[k] = fps.get(k, 0) + v
        minutes += d['gt']['annotated_frames'] / (60.0 * hz)
    out = {}
    for m, s in S.items():
        p = s['tp'] / max(1, s['tp'] + s['fp'])
        r = s['tp'] / max(1, s['tp'] + s['fn'])
        out[m] = dict(P=p, R=r, F1=2 * p * r / max(1e-9, p + r), MOTA=1 - (s['fn'] + s['fp'] + s['idsw']) / max(1, s['num_gt']),
                      IDF1=2 * s['idtp'] / max(1, s['num_gt'] + s['num_pred']), MOTP=s['motp_w'] / max(1, s['tp']),
                      FPmin=s['fp'] / max(1e-9, minutes), GT=s['num_gt'])
    D, PD = 'dynamic', 'potentially_dynamic'
    n_pd = sum(conf.get(PD, {}).values())
    pd_tp = conf.get(PD, {}).get(PD, 0)
    pd_pred = sum(conf.get(g, {}).get(PD, 0) for g in conf) + fps.get(PD, 0)
    p = pd_tp / pd_pred if pd_pred else float('nan')
    r = pd_tp / n_pd if n_pd else float('nan')
    out['PD'] = dict(P=p, R=r, F1=2 * p * r / (p + r) if pd_pred and n_pd and p + r > 0 else float('nan'), GT=n_pd)
    md = sum(v for k, v in conf.get(D, {}).items() if k != 'missed')
    mp = sum(v for k, v in conf.get(PD, {}).items() if k != 'missed')
    out['swap'] = dict(D_as_PD=conf.get(D, {}).get(PD, 0) / md if md else float('nan'),
                       PD_as_D=conf.get(PD, {}).get(D, 0) / mp if mp else float('nan'),
                       any_as_S=(conf.get(D, {}).get('static', 0) + conf.get(PD, {}).get('static', 0)) / max(1, md + mp))
    out['S'] = dict(falseD_min=fps.get(D, 0) / max(1e-9, minutes), falsePD_min=fps.get(PD, 0) / max(1e-9, minutes))
    out['minutes'] = minutes
    out['files'] = len(files)
    return out


def method_group(fn, reps, kind, seqs):
    per_rep = []
    for rep in range(reps):
        if kind == 'street':
            p = fn(kind, rep, seqs, None)
            if p is not None and p.exists():
                per_rep.append(pool([str(p)]))
            continue
        files, missing = [], 0
        W = VWIN if kind == 'veh' else WIN
        for s in seqs:
            for w in W[s]:
                p = fn(kind, rep, s, w)
                if p is None or not p.exists():
                    missing += 1
                    continue
                files.append(str(p))
        expected = sum(len(W[s]) for s in seqs)
        if files and missing == 0:
            per_rep.append(pool(files))
        elif files:
            per_rep.append(dict(pool(files), incomplete=f'{len(files)}/{expected}'))
    return per_rep


def flatten(d, pre=''):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(flatten(v, f'{pre}{k}.'))
        elif isinstance(v, (int, float)):
            out[f'{pre}{k}'] = v
    return out


def summarize(per_rep):
    if not per_rep:
        return None
    flat = [flatten(p) for p in per_rep]
    keys = flat[0].keys()
    res = {k: dict(mean=float(np.nanmean([f[k] for f in flat])), min=float(np.nanmin([f[k] for f in flat])),
                   max=float(np.nanmax([f[k] for f in flat]))) if not all(np.isnan(f[k]) for f in flat)
           else dict(mean=float('nan'), min=float('nan'), max=float('nan')) for k in keys}
    res['reps'] = len(per_rep)
    inc = [p.get('incomplete') for p in per_rep if p.get('incomplete')]
    if inc:
        res['incomplete'] = inc[0]
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=str(R / 'label_compare.json'))
    a = ap.parse_args()
    table = {}
    for g, (kind, seqs) in GROUPS.items():
        table[g] = {}
        for name, (fn, reps) in METHODS.items():
            s = summarize(method_group(fn, reps, kind, seqs))
            if s:
                table[g][name] = s
    json.dump(table, open(a.out, 'w'), indent=1)
    cols = [('merged.P', 'Pag'), ('merged.R', 'Rag'), ('merged.F1', 'F1ag'), ('merged.MOTA', 'MOTAag'), ('merged.IDF1', 'IDF1'),
            ('dynamic_only.P', 'P_D'), ('dynamic_only.R', 'R_D'), ('dynamic_only.F1', 'F1_D'), ('dynamic_only.MOTA', 'MOTA_D'),
            ('PD.P', 'P_PD'), ('PD.R', 'R_PD'), ('PD.F1', 'F1_PD'), ('swap.D_as_PD', 'D>PD'), ('swap.PD_as_D', 'PD>D'),
            ('S.falseD_min', 'fD/min'), ('S.falsePD_min', 'fPD/min'), ('merged.MOTP', 'MOTP')]
    for g, ms in table.items():
        print(f'\n== {g}')
        print(f'{"method":26s}' + ''.join(f'{c:>8s}' for _, c in cols) + '  reps')
        for name, s in ms.items():
            row = ''
            for k, _ in cols:
                v = s[k]['mean']
                row += f'{v:8.1f}' if k.startswith('S.') else (f'{v:8.2f}' if k.endswith('MOTP') else (f'{100 * v:8.1f}' if v == v else f'{"-":>8s}'))
            print(f'{name:26s}{row}  {s["reps"]}{" (" + s["incomplete"] + ")" if s.get("incomplete") else ""}')


if __name__ == '__main__':
    main()


# ---- point-wise IoU (coda_point_iou.py), pooled over windows: IoU = sum TP / (sum TP + sum FN + sum FP) ----
IOU_LABELS = ['moving', 'moving_pedestrian', 'moving_vehicle', 'still_pedestrian', 'still_vehicle', 'all']


def iou_table(methods=(('SINS V+J+SY+E', ['LVE', 'LVEr2', 'LVEr3'], ['LHVE', 'LHVEr2', 'LHVEr3']),
                       ('SINS V+J+SY', ['LVS', 'LVSr2', 'LVSr3'], ['LHVS', 'LHVSr2', 'LHVSr3']),
                       ('Dynablox', ['dynablox'], ['dynablox']), ('M-detector', ['mdetector'], ['mdetector'])),
              dirs=(RL / 'coda_iou', R / 'coda_iou')):
    out = {}
    for gname, seqs in (('tuning', [0, 10, 16]), ('held-out', [11, 12]), *((f'seq {s}', [s]) for s in (0, 10, 16, 11, 12))):
        out[gname] = {}
        for name, tt, th in methods:
            reps = []
            for rep in range(len(tt)):
                acc, n = {}, 0
                for s in seqs:
                    tag = (th if s in (11, 12) else tt)[rep]
                    for w in WIN[s]:
                        f = next((d / f'{tag}_{s}_{w}.json' for d in dirs if (d / f'{tag}_{s}_{w}.json').exists()), None)
                        if f is None:
                            continue
                        n += 1
                        c = json.load(open(f))['counts']
                        for k, v in c.items():
                            a = acc.setdefault(k, [0, 0, 0])
                            a[0] += v['tp']; a[1] += v['fn']; a[2] += v['fp']
                if n:
                    reps.append({k: a[0] / max(1, sum(a)) for k, a in acc.items()} | {'_n': n})
            if reps:
                out[gname][name] = {k: float(np.mean([r[k] for r in reps if k in r])) for k in reps[0]} | {'_reps': len(reps)}
    return out


if __name__ == '__main__' and os.environ.get('IOU'):
    t = iou_table()
    json.dump(t, open(R / 'label_compare_iou.json', 'w'), indent=1)
    for g, ms in t.items():
        print(f'\n== point IoU, CODa {g}  (D = boxes labelled DYNAMIC, any = every published box)')
        print(f'{"method":18s}' + ''.join(f'{l:>19s}' for l in IOU_LABELS) + '  windows reps')
        for name, r in ms.items():
            pre = 'any:' if not name.startswith('SINS') else None
            row = ''
            for l in IOU_LABELS:
                kd, ka = f'dyn:{l}', f'any:{l}'
                d = r.get(kd, float('nan')); a = r.get(ka, d)
                row += f'{100 * d:9.1f}/{100 * a:<9.1f}'
            print(f'{name:18s}{row}  {r["_n"]:.0f} {r["_reps"]}')
