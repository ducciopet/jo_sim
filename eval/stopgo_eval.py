#!/usr/bin/env python3
"""
Stop-and-go: does the person keep the SAME track id when they stop and when they start walking again, and are they
still tracked while standing?

Events from the GT state of the person (mocap): phases of DYNAMIC / POTENTIALLY_DYNAMIC, consecutive samples of the
same state merged across mocap gaps shorter than --merge-gap, phases shorter than --min-phase dropped (merged into
their neighbours). Every boundary between a D and a PD phase is an event: "stop" (D -> PD) or "restart" (PD -> D).

Per frame the person is matched to the predictions as in harness/metrics.evaluate (10 Hz grid, 1 m gate, same ROI,
any published state counts: this is about identity, the label is scored elsewhere).
  id kept      the last id matched in the --before window before the boundary equals the first id matched in the
               --after window after it; an event where either side has no match counts as NOT kept
  tracked      share of the PD-phase frames (person standing) where the person is matched at all
Methods without a stable id (LV-DOT, M-detector, Xu) only get "tracked".

usage: python3 eval/stopgo_eval.py --campaign r9 --reps a b c       (methods of eval/baseline_eval.py)
       python3 eval/stopgo_eval.py --bags name=<bag prefix ending in _v2_> ...  (tracker replays, rep a only)
writes eval/results/stopgo_<campaign>.json
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import baseline_eval as be  # noqa: E402
from harness.gt_mocap import tracks_to_mocap  # noqa: E402
from harness.metrics import EvalConfig, _match, make_grid, sample_gt, sample_pred  # noqa: E402
from harness.ros_io import load_obstacles  # noqa: E402
from harness.tracks import read_tracks  # noqa: E402


def phases(gt, merge_gap, min_phase):
    g = gt[gt['track_id'] >= 0].sort_values('t_ns')
    t, st = g['t_ns'].to_numpy() / 1e9, g['state'].to_numpy()
    ph = []  # [state, t0, t1]
    for ti, si in zip(t, st):
        if ph and ph[-1][0] == si and ti - ph[-1][2] < merge_gap:
            ph[-1][2] = ti
        else:
            ph.append([si, ti, ti])
    changed = True
    while changed:  # drop short phases: the neighbours absorb them
        changed = False
        for i, p in enumerate(ph):
            if p[2] - p[1] < min_phase and len(ph) > 1:
                ph.pop(i)
                changed = True
                break
        merged = []
        for p in ph:
            if merged and merged[-1][0] == p[0]:
                merged[-1][2] = p[2]
            else:
                merged.append(p)
        ph = merged
    return ph


def matched_ids(gt, pred, cfg):
    grid = make_grid(gt, cfg)
    G, P = sample_gt(gt, grid, cfg), sample_pred(pred, grid, cfg)
    ids = []
    for gts, preds in zip(G, P):
        gts = [g for g in gts if g[0] >= 0]
        m, _ = _match(gts, preds, cfg.match_dist)
        ids.append(preds[m[0][1]][0] if m else None)
    return grid, ids


def score_seq(gt, pred, cfg, args):
    ph = phases(gt, args.merge_gap, args.min_phase)
    grid, ids = matched_ids(gt, pred, cfg)
    ev = []
    for a, b in zip(ph, ph[1:]):
        kind = {('dynamic', 'potentially_dynamic'): 'stop', ('potentially_dynamic', 'dynamic'): 'restart'}.get((a[0], b[0]))
        if kind is None:
            continue
        tb = b[1]
        if tb < grid[0] + args.before:  # before the scored window (t_start)
            continue
        before = [i for t, i in zip(grid, ids) if tb - args.before <= t < tb and i is not None]
        after = [i for t, i in zip(grid, ids) if tb <= t < tb + args.after and i is not None]
        ev.append(dict(kind=kind, t=float(tb - grid[0]), kept=bool(before and after and before[-1] == after[0]),
                       before=before[-1] if before else None, after=after[0] if after else None))
    pd_frames = [i for t, i in zip(grid, ids) if any(p[0] == 'potentially_dynamic' and p[1] <= t <= p[2] for p in ph)]
    return ev, sum(i is not None for i in pd_frames), len(pd_frames)


def summarize(per_seq):
    ev = [e for s in per_seq.values() for e in s['events']]
    out = dict(events=len(ev), stops=sum(e['kind'] == 'stop' for e in ev), restarts=sum(e['kind'] == 'restart' for e in ev),
               id_kept=sum(e['kept'] for e in ev) / len(ev) if ev else float('nan'),
               tracked_standing=sum(s['pd_tracked'] for s in per_seq.values()) / max(1, sum(s['pd_frames'] for s in per_seq.values())))
    for k in ('stop', 'restart'):
        e = [x for x in ev if x['kind'] == k]
        out[f'id_kept_{k}'] = sum(x['kept'] for x in e) / len(e) if e else float('nan')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--campaign', default='r9')
    ap.add_argument('--reps', nargs='*', default=['a', 'b', 'c'])
    ap.add_argument('--methods', nargs='*', default=['v2', 'v1', 'dynablox', 'unimt', 'eagermot', 'lvdot', 'mdetector', 'xuyolo'])
    ap.add_argument('--bags', nargs='*', default=[], help='name=<bags/replay prefix>, e.g. nosem=semab_r9a_nosem_v2_')
    ap.add_argument('--merge-gap', type=float, default=2.0)
    ap.add_argument('--min-phase', type=float, default=1.0)
    ap.add_argument('--before', type=float, default=1.0)
    ap.add_argument('--after', type=float, default=2.0)
    ap.add_argument('--out')
    args = ap.parse_args()
    jobs = [(m, r, None) for r in args.reps for m in args.methods] + \
           [(spec.split('=')[0], 'a', spec.split('=')[1]) for spec in args.bags]
    res_all = {}
    for name, r, prefix in jobs:
        res = be.REPO / 'eval' / 'results' / f'own_{args.campaign}{r}'
        S = json.loads((res / 'summary.json').read_text())
        al = json.loads((res / 'alignment_v2.json').read_text())
        cfg = EvalConfig(match_dist=S['match_dist'], roi=tuple(S['roi']), t_start=S['t_start'])
        per_seq = {}
        for s in be.SEQS:
            gt = read_tracks(res / s / 'gt' / 'tracks.csv')
            if prefix:
                p, _, _ = load_obstacles(be.REPO / 'bags' / 'replay' / f'{prefix}{s}')
                pred = tracks_to_mocap(p, np.array(al['T_mocap_odom'][s]), al['delta'][s])
            else:
                pred = be.predictions(name, args.campaign, r, s, res, al)
            ev, tr, n = score_seq(gt, pred, cfg, args)
            per_seq[s] = dict(events=ev, pd_tracked=tr, pd_frames=n)
        sm = summarize(per_seq)
        if not prefix and not be.IDENTITY.get(name, True):
            for k in ('id_kept', 'id_kept_stop', 'id_kept_restart'):
                sm[k] = None
        res_all.setdefault(name, {})[r] = dict(sm, per_seq=per_seq)
        ik = '   n/a' if sm['id_kept'] is None else f"{100 * sm['id_kept']:5.1f}%"
        print(f"{name:11s} {r}: events {sm['events']} (stop {sm['stops']}, restart {sm['restarts']})  id kept {ik}  "
              f"tracked while standing {100 * sm['tracked_standing']:5.1f}%", flush=True)
    out = args.out or be.REPO / 'eval' / 'results' / f'stopgo_{args.campaign}.json'
    Path(out).write_text(json.dumps(res_all, indent=1, default=str))


if __name__ == '__main__':
    main()
