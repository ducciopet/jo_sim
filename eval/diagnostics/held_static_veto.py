#!/usr/bin/env python3
"""Follow-up of held_static_gates.py (24/11): motion evidence and the point-to-surface veto on held walking persons (HDBG).
motion_ev (CSV) = left_ok && cons && speed_ok && !vetoed && (kf_conf || vote) for unclassified tracks; vetoed is recovered as
(pm_frac >= 0 && pm_frac < 0.5 && cls empty). 'missed' rows use the track's last matched row. Compares held rows with walking
persons (GT speed > 0.5 m/s) correctly paired with a dynamic track; splits by range from the robot; for held episodes (consecutive
held ticks of one GT person on one track) counts dyn_ticks resets (dyn_ticks drops) and the gates failing at each reset tick.
usage: held_static_veto.py [TAG]"""
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
held, dyn, resets = [], [], collections.Counter(); ep_resets = []
def gates(r):
    return [nm for nm, bad in (('left_ok', r.left_ok == 0), ('cons_ok', r.cons_ok == 0), ('kf_conf', r.kf_conf == 0), ('vote', r.vote == 0),
                               ('size_ok', r.size_ok == 0), ('disocc', r.disocc == 1), ('speed_ok', r.speed_ok == 0),
                               ('veto', 0 <= r.pm_frac < 0.5 and r.cls == '-')) if bad]
for w in 'w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1'.split():
    c = w if '_' in w else f'0_{w}'
    f = B / 'runs' / f'{TAG}_trk_{c}.csv'
    if not f.exists(): continue
    d = pd.read_csv(f); d['tn'] = (d.t * 1e9).round().astype('int64'); d = d.sort_values(['id', 'tn'])
    lastm = {}                                            # last matched row per track id, for 'missed' rows
    d['lm_idx'] = -1
    for tid, q in d.groupby('id'):
        li = -1
        for ix, m in zip(q.index, q.matched):
            if m == 1: li = ix
            d.at[ix, 'lm_idx'] = li
    dc = d[d.confirmed == 1]; tick = np.sort(dc.tn.unique()); by_t = dict(tuple(dc.groupby('tn')))
    odom = ce.load_odometry(B / f'coda{c}_odom')
    gt, _, _, _ = ce.build_gt(str(B / f'coda{c}_gt.csv'), odom, ns)
    raw = pd.read_csv(B / f'coda{c}_gt.csv')
    gt = gt[(gt.track_id >= 0)]
    gt = gt[gt.track_id.map(raw[['track_id', 'class']].drop_duplicates('track_id').set_index('track_id')['class']) == 'Pedestrian']
    gt = gt[gt.state == 'dynamic'].sort_values(['track_id', 't_ns']).copy(); sp = np.zeros(len(gt))
    for _, idx in gt.groupby('track_id').indices.items():
        if len(idx) > 1:
            q = gt.iloc[idx]; tt_ = q.t_ns.to_numpy(float) / 1e9
            sp[idx] = np.hypot(np.gradient(q.x.to_numpy(float), tt_), np.gradient(q.y.to_numpy(float), tt_))
    gt['spd'] = sp; gt['rng'] = ce.robot_dist(gt, odom)
    run = collections.defaultdict(int); prev_dyn = {}; ep = {}
    for t, G in gt.groupby('t_ns'):
        k = tick[np.argmin(np.abs(tick - t))]
        if abs(k - t) > 0.06e9: continue
        D = by_t[k]
        for _, g in G.iterrows():
            dd = np.hypot(D.x - g.x, D.y - g.y)
            if not len(dd) or dd.min() > 1.0: continue
            r = D.loc[dd.idxmin()]; key = (w, int(g.track_id), int(r.id))
            rr = d.loc[r.lm_idx] if (r.matched == 0 and r.lm_idx >= 0) else r
            rec = dict(rng=g.rng, spd=g.spd, motion_ev=rr.motion_ev, pm_frac=rr.pm_frac, pm_dist=rr.pm_dist, pm_dt=rr.pm_dt, pm_n=rr.pm_n,
                       veto=bool(0 <= rr.pm_frac < 0.5 and rr.cls == '-'), status=r.status, dyn_ticks=r.dyn_ticks, gates=gates(rr))
            pd_ = prev_dyn.get(key)
            if pd_ is not None and r.dyn_ticks < pd_ and pd_ > 0 and key in ep:
                ep[key] += 1
                for gname in gates(rr) or ['none']: resets[gname] += 1
            prev_dyn[key] = r.dyn_ticks
            if r.status in ('dynamic', 'coasting_pub'):
                if key in ep and run[key] >= 3: ep_resets.append(ep.pop(key))
                else: ep.pop(key, None)
                run[key] = 0
                if g.spd > 0.5: dyn.append(rec)
                continue
            run[key] += 1; ep.setdefault(key, 0)
            if run[key] >= 3: held.append(rec)
    ep_resets += [v for k_, v in ep.items() if run[k_] >= 3]
H = pd.DataFrame(held); Dy = pd.DataFrame(dyn)
def summ(X, name):
    if not len(X): print(f'{name}: none'); return
    me0 = X.motion_ev == 0
    print(f'{name}: n {len(X)} | motion_ev=0 {100 * me0.mean():.0f}%, of those vetoed (pm_frac<0.5, unclassified) {100 * X[me0].veto.mean():.0f}% '
          f'| vetoed overall {100 * X.veto.mean():.0f}% | pm_frac p10/50/90 ' + '/'.join(f'{v:.2f}' for v in np.nanpercentile(X.pm_frac.where(X.pm_frac >= 0), [10, 50, 90]))
          + f' (pm_frac<0 = no baseline: {100 * (X.pm_frac < 0).mean():.0f}%) | pm_dist med {X.pm_dist.where(X.pm_dist >= 0).median():.2f} m, '
          f'pm_dt med {X.pm_dt.where(X.pm_dt > 0).median():.2f} s, pm_n med {X.pm_n.median():.0f} | speed med {X.spd.median():.2f}')
print(f'{TAG}: held walking persons vs dynamic walking persons (GT speed > 0.5 m/s for the dynamic set)')
summ(H, 'held'); summ(H[H.spd > 0.5], 'held, speed > 0.5'); summ(Dy, 'dynamic')
for lo, hi in ((0, 10), (10, 15), (15, 20.5)):
    summ(H[(H.rng >= lo) & (H.rng < hi)], f'held {lo}-{hi:.0f} m'); summ(Dy[(Dy.rng >= lo) & (Dy.rng < hi)], f'dynamic {lo}-{hi:.0f} m')
nog = H[H.gates.apply(len) == 0]
print(f'no gate failing (incl. veto): {len(nog)} rows, dyn_ticks p10/50/90 ' + '/'.join(f'{v:.0f}' for v in np.percentile(nog.dyn_ticks, [10, 50, 90])) if len(nog) else 'no gate failing: none')
print(f'held episodes {len(ep_resets)}: resets of the dyn_ticks streak per episode mean {np.mean(ep_resets):.2f}, episodes with >= 1 reset '
      f'{100 * np.mean(np.array(ep_resets) > 0):.0f}%; gates failing at reset ticks: ' + ', '.join(f'{k} {v}' for k, v in resets.most_common()))

M1 = H[H.motion_ev == 1]
print(f'held with motion_ev=1: {len(M1)} ({100 * len(M1) / max(1, len(H)):.0f}%) | status ' + ', '.join(f'{k} {v}' for k, v in M1.status.value_counts().items())
      + f' | disocc {100 * M1.gates.apply(lambda g: "disocc" in g).mean():.0f}% | dyn_ticks p10/50/90 ' + '/'.join(f'{v:.0f}' for v in np.percentile(M1.dyn_ticks, [10, 50, 90]))
      + f' | dyn_ticks < 5: {100 * (M1.dyn_ticks < 5).mean():.0f}%')
