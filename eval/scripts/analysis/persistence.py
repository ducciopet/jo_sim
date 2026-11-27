#!/usr/bin/env python3
"""Persistence outside the camera FOV and identity across events (paper view C + identity; review 25/11).

For one run on one window (CODa, JRDB: GT CSV with an in_fov column) and for any method (SINS run bag, or a CSV baseline
with the debounced state conversion of coda_eval.py):
  - GT agents and their motion state come from coda_eval.build_gt (all states kept everywhere: --pd-everywhere), the
    in_fov flag from the GT CSV; predictions are the method's published outputs (D or PD), matched per annotated frame to
    the GT by centre distance <= 1 m (Hungarian), prediction frame nearest in time within 0.15 s.
  - EXIT events: a GT agent goes from inside a camera FOV to outside. Counted only if, in the last 1 s before the exit,
    it was matched inside the FOV (the method "had" it). Persistence = time from the exit until the first frame the
    agent is no longer matched (any prediction / the same prediction id as before the exit); censored if the agent
    re-enters a FOV, leaves the range or its annotation ends first. Kaplan-Meier survival S(t), separately for agents
    still (PD) and moving (D) at the exit. This isolates the persistence mechanism from the detector's quality.
    --gap G: persistence ends only after more than G s without a match (short flicker tolerated), same for all methods.
  - STOP events: GT D -> PD. Identity across an event (exit or stop): the id matched in the last 1 s before it versus
    the id of the next match after it: same = success, different = failure, never matched again = censored.

usage: persistence.py --run <bag|csv> [--method sins|<csv method>] --gt <window>_gt.csv --odom <window>_odom --out x.json
       persistence.py --pool a.json b.json ... [--label NAME]       (pooled KM and identity over windows/reps)
Pure evaluation of existing outputs.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import linear_sum_assignment

E = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(E))
import coda_eval as ce                                  # noqa: E402
from harness.ros_io import load_obstacles, load_odometry   # noqa: E402
from harness.tracks import objects                     # noqa: E402

GRID = [0.5, 1, 2, 3, 5, 10, 20]


def km(events):
    """events: list of (time, observed 1/0) -> S(t) at GRID, median."""
    ev = sorted(events)
    s, out, med = 1.0, {}, None
    times = sorted({t for t, o in ev if o})
    curve = []
    for t in times:
        n = sum(1 for x, _ in ev if x >= t)
        d = sum(1 for x, o in ev if x == t and o)
        if n:
            s *= 1 - d / n
        curve.append((t, s))
        if med is None and s <= 0.5:
            med = t
    for g in GRID:
        v = 1.0
        for t, sv in curve:
            if t <= g:
                v = sv
        out[str(g)] = round(v, 3)
    return out, med


def analyse(run, method, gt_csv, odom_bag, a):
    odom = load_odometry(odom_bag)
    ns = argparse.Namespace(camera_only=False, dyn_speed=0.3, ignore_undefined=False, min_pts=1, min_range=1.2,
                            pd_everywhere=True, pd_prev_observable=False, range=a.range, state_buffer=0.0)
    gt, _, frames, _ = ce.build_gt(gt_csv, odom, ns)
    gt = objects(gt)
    raw = pd.read_csv(gt_csv, keep_default_na=False)
    raw = raw[raw['track_id'].astype(int) >= 0]
    fov = {(int(t), int(i)): str(f) == 'True' for t, i, f in zip(raw['t_ns'], raw['track_id'], raw['in_fov'])}
    if method == 'sins':
        pred, _, _ = load_obstacles(run)
    else:
        pred = ce.convert_states(ce.load_csv_tracks(run), 'debounced')
    pred = objects(pred)
    pred = pred[pred['state'].isin(['dynamic', 'potentially_dynamic'])]
    pt = np.sort(pred['t_ns'].unique())
    by_pt = {t: g for t, g in pred.groupby('t_ns')}
    frames = np.sort(np.unique(gt['t_ns']))
    # per frame: {gt_id: (matched pred id or None, in_fov, state)}
    rows = []
    for t in frames:
        g = gt[gt['t_ns'] == t]
        p = None
        if len(pt):
            k = int(np.argmin(np.abs(pt - t)))
            if abs(int(pt[k]) - int(t)) <= 0.15e9:
                p = by_pt[pt[k]]
        m = {}
        if p is not None and len(p) and len(g):
            d = np.hypot(g['x'].to_numpy()[:, None] - p['x'].to_numpy()[None], g['y'].to_numpy()[:, None] - p['y'].to_numpy()[None])
            r, c = linear_sum_assignment(np.where(d <= 1.0, d, 1e6))
            for i, j in zip(r, c):
                if d[i, j] <= 1.0:
                    m[int(g['track_id'].iloc[i])] = int(p['track_id'].iloc[j])
        rows.append({int(i): (m.get(int(i)), fov.get((int(t), int(i)), False), s)
                     for i, s in zip(g['track_id'], g['state'])})
    tt = frames / 1e9
    ids = sorted({i for r in rows for i in r})
    exits, ident = [], []
    for gid in ids:
        seq = [(tt[k], rows[k][gid]) for k in range(len(rows)) if gid in rows[k]]
        for k in range(1, len(seq)):
            t, (mid, inf, st) = seq[k]
            tp, (mid_p, inf_p, st_p) = seq[k - 1]
            if t - tp > 0.35:
                continue          # annotation gap
            ev = None
            if inf_p and not inf:
                ev = 'exit'
            elif st_p == 'dynamic' and st == 'potentially_dynamic':
                ev = 'stop'
            if ev is None:
                continue
            pre = [x[1][0] for x in seq[:k] if t - x[0] <= 1.0 and x[1][0] is not None]
            if not pre:
                continue
            pre_id = pre[-1]
            # identity: next match after the event
            nxt = next((x[1][0] for x in seq[k:] if x[1][0] is not None), None)
            ident.append(dict(event=ev, t=float(t), state_at_event=st,
                              outcome='censored' if nxt is None else ('same' if nxt == pre_id else 'switch')))
            if ev != 'exit':
                continue
            # persistence after the exit
            for mode in ('any', 'same_id'):
                dur, obs, last_ok = 0.0, 0, t
                for x in seq[k:]:
                    m_, inf_, _ = x[1]
                    if inf_:
                        break     # back in a FOV: censored
                    ok = m_ is not None and (mode == 'any' or m_ == pre_id)
                    if ok:
                        last_ok = x[0]
                        dur = x[0] - t
                    elif a.gap <= 0:
                        obs = 1
                        dur = x[0] - t
                        break
                    elif x[0] - last_ok > a.gap:      # lost for longer than the tolerated gap: ends at the last match
                        obs = 1
                        dur = last_ok - t
                        break
                exits.append(dict(mode=mode, still=st == 'potentially_dynamic', time=float(dur), observed=obs))
    return dict(run=str(run), method=method, gt=str(gt_csv), exits=exits, identity=ident)


def summarize(exits, ident):
    out = {}
    for mode in ('any', 'same_id'):
        for still in (True, False):
            ev = [(e['time'], e['observed']) for e in exits if e['mode'] == mode and e['still'] == still]
            if ev:
                s, med = km(ev)
                out[f'persistence_{mode}_{"still" if still else "moving"}'] = dict(n=len(ev), S=s, median_s=med)
    for evn in ('exit', 'stop'):
        x = [i for i in ident if i['event'] == evn]
        dec = [i for i in x if i['outcome'] != 'censored']
        if x:
            out[f'identity_{evn}'] = dict(n=len(x), censored=len(x) - len(dec),
                                          same_rate=round(sum(i['outcome'] == 'same' for i in dec) / len(dec), 3) if dec else None)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run')
    ap.add_argument('--method', default='sins')
    ap.add_argument('--gt')
    ap.add_argument('--odom')
    ap.add_argument('--range', type=float, default=20.0)
    ap.add_argument('--gap', type=float, default=0.0,
                    help='tolerated unmatched gap in s before persistence ends (0 = ends at the first unmatched frame, as paper-v1)')
    ap.add_argument('--out')
    ap.add_argument('--pool', nargs='+')
    ap.add_argument('--label', default='')
    a = ap.parse_args()
    if a.pool:
        ex, idn = [], []
        for f in a.pool:
            d = json.load(open(f))
            ex += d['exits']
            idn += d['identity']
        res = dict(label=a.label, files=len(a.pool), **summarize(ex, idn))
    else:
        d = analyse(a.run, a.method, a.gt, a.odom, a)
        res = dict(d, summary=summarize(d['exits'], d['identity']))
    print(json.dumps(res.get('summary', res), indent=1))
    if a.out:
        json.dump(res, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
