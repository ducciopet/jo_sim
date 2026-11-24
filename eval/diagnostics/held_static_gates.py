#!/usr/bin/env python3
"""Held-static moving persons (24/11, laptop request): which motion gate holds them. Inputs: HDBG run (EB + track_debug_csv)
per window, bags/coda/runs/<TAG>_trk_<c>.csv. GT: coda_eval e3 GT, moving persons (CODa class Pedestrian, state dynamic).
Each GT row is paired with the nearest CONFIRMED debug-track row within 1 m at the same tick (same sim stamps, <= 60 ms).
A pair is "held" when the track's status is not 'dynamic' (static / potentially_dynamic / missed / suppressed / unconfirmed)
and that track has been non-dynamic for >= 3 consecutive ticks on this person. For held rows: the failing gates
(left_ok=0 net footprint from birth, cons_ok=0 consistency window, kf_conf=0, vote=0 point vote, size_ok=0, disocc=1 blocked by
disocclusion, speed_ok=0), confirm_ticks vs dyn_ticks, in/out of camera FOV, person class yes/no, and the GT speed.
usage: held_static_gates.py [TAG]"""
import argparse, sys, collections
from pathlib import Path
import numpy as np, pandas as pd
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import coda_eval as ce   # noqa: E402

TAG = sys.argv[1] if len(sys.argv) > 1 else 'HDBG'
B = Path(__file__).resolve().parents[2] / 'bags/coda'
ns = argparse.Namespace(range=20.0, ignore_dist=1.0, min_range=1.2, max_height=2.5, min_pts=1, pd_everywhere=False, camera_only=False,
                        cam_json=None, ignore_undefined=True, dyn_speed=0.3, match_dist=1.0, points_min_cluster=20, extra_ignore=None,
                        extra_ignore_dist=0.3, extra_ignore_dt=0.5, frames='all', coda_root='', ignore_gt=None, topic=None,
                        method='sins', ignore_box=False)
C = collections.Counter(); spd = []; ct = []; dyt = []; n_mov = 0; n_paired = 0
for w in 'w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1'.split():
    c = w if '_' in w else f'0_{w}'
    f = B / 'runs' / f'{TAG}_trk_{c}.csv'
    if not f.exists(): print('missing', f, file=sys.stderr); continue
    d = pd.read_csv(f); d = d[d.confirmed == 1]; d['tn'] = (d.t * 1e9).round().astype('int64')
    tick = np.sort(d.tn.unique())
    odom = ce.load_odometry(B / f'coda{c}_odom')
    gt, _, _, _ = ce.build_gt(str(B / f'coda{c}_gt.csv'), odom, ns)
    raw = pd.read_csv(B / f'coda{c}_gt.csv')
    gt = gt[(gt.track_id >= 0)]
    gt = gt[gt.track_id.map(raw[['track_id', 'class']].drop_duplicates('track_id').set_index('track_id')['class']) == 'Pedestrian']
    gt = gt[gt.state == 'dynamic'].copy()
    gt = gt.sort_values(['track_id', 't_ns']).copy(); sp = np.zeros(len(gt))
    for _, idx in gt.groupby('track_id').indices.items():
        if len(idx) > 1:
            q = gt.iloc[idx]; tt_ = q.t_ns.to_numpy(float) / 1e9
            sp[idx] = np.hypot(np.gradient(q.x.to_numpy(float), tt_), np.gradient(q.y.to_numpy(float), tt_))
    gt['spd'] = sp
    by_t = dict(tuple(d.groupby('tn')))
    run = collections.defaultdict(int)       # (gt id, track id) -> consecutive non-dynamic ticks
    for t, G in gt.groupby('t_ns'):
        k = tick[np.argmin(np.abs(tick - t))]
        if abs(k - t) > 0.06e9: continue
        D = by_t[k]
        for _, g in G.iterrows():
            n_mov += 1
            dd = np.hypot(D.x - g.x, D.y - g.y)
            if not len(dd) or dd.min() > 1.0: continue
            n_paired += 1; r = D.loc[dd.idxmin()]
            key = (int(g.track_id), int(r.id))
            if r.status in ('dynamic', 'coasting_pub'): run[key] = 0; C['paired dynamic'] += 1; continue
            run[key] += 1
            if run[key] < 3: C['paired non-dynamic < 3 ticks'] += 1; continue
            C['held'] += 1; C['held status ' + r.status] += 1
            fail = [nm for nm, bad in (('left_ok', r.left_ok == 0), ('cons_ok', r.cons_ok == 0), ('kf_conf', r.kf_conf == 0),
                                        ('vote', r.vote == 0), ('size_ok', r.size_ok == 0), ('disocc', r.disocc == 1),
                                        ('speed_ok', r.speed_ok == 0)) if bad]
            for nm in fail: C['gate ' + nm] += 1
            C['combo ' + ('+'.join(fail) or 'none')] += 1
            C[f'status {r.status} combo ' + ('+'.join(fail) or 'none')] += 1
            C['fov in' if r.fov == 1 else 'fov out'] += 1
            C['class person' if r.cls == 'person' else 'class ' + ('none' if r.cls == '-' else 'other')] += 1
            spd.append(g.spd); ct.append(r.confirm_ticks); dyt.append(r.dyn_ticks)
h = C['held']
print(f'{TAG}: moving person GT rows {n_mov}, paired with a confirmed track within 1 m {n_paired}, held (non-dynamic >= 3 ticks) {h} '
      f'({100 * h / max(1, n_mov):.1f}% of moving rows)')
for k in sorted(C):
    if k.startswith(('held status', 'gate', 'fov', 'class', 'paired')): print(f'  {k}: {C[k]} ({100 * C[k] / max(1, h):.0f}% of held)' if not k.startswith('paired') else f'  {k}: {C[k]}')
print('  top gate combinations: ' + '; '.join(f'{k[6:]} {v}' for k, v in sorted(((k, v) for k, v in C.items() if k.startswith('combo')), key=lambda x: -x[1])[:8]))
for st_ in ('static', 'potentially_dynamic', 'missed'):
    print(f'  {st_}: ' + '; '.join(f'{k.split(" combo ")[1]} {v}' for k, v in sorted(((k, v) for k, v in C.items() if k.startswith(f'status {st_} combo')), key=lambda x: -x[1])[:6]))
if spd:
    print(f'  GT speed of held persons: median {np.median(spd):.2f} m/s, p25 {np.percentile(spd, 25):.2f}, p75 {np.percentile(spd, 75):.2f}')
    print(f'  confirm_ticks median {np.median(ct):.0f} (p90 {np.percentile(ct, 90):.0f}); dyn_ticks median {np.median(dyt):.0f} (p90 {np.percentile(dyt, 90):.0f})')
