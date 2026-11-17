#!/usr/bin/env python3
"""Moving-FP / moving-FN funnel on CODa, same pipeline as coda_eval.py dynamic_only (build_gt, filter_pred, 10 Hz grid,
Hungarian 1.0 m). Every unmatched DYNAMIC prediction is put in one bucket, using the FULL GT of that frame
(moving + still, ignored boxes too):
  dup_moving   within 2.5 m of a moving GT that another prediction already matched (fragment / duplicate)
  off_moving   within 2.5 m of a moving GT left unmatched (centre 1-2.5 m off: costs a FP and a FN)
  still_obj    within 1.5 m of a still GT (parked car, standing person: real object called moving)
  ignored_obj  within 1.5 m of an ignored GT box (hidden / out of FOV still) that survived filter_pred
  no_object    nothing annotated near: split by the prediction track's net travel over its life (< 0.5 m = static
               structure called moving; else a moving ghost), footprint and range
Unmatched moving GT (FN): pred of any state within 1 m (wrong state), pred within 2.5 m (offset), none.
usage: fp_funnel.py TAG win [win ...]     (run dirs bags/coda/runs/<TAG>[r2|r3]_v2_coda<win>, all 3 reps)
"""
import sys, argparse
from pathlib import Path
import numpy as np, pandas as pd
sys.path.insert(0, '/home/punk-opc/Desktop/jo_sim_sins/eval')
import coda_eval as ce
from harness.metrics import EvalConfig, make_grid, sample_gt, sample_pred, _match
from harness.tracks import objects, DYNAMIC
from harness.ros_io import load_obstacles, load_odometry

R = '/home/punk-opc/Desktop/jo_sim_sins/'
tag, wins = sys.argv[1], sys.argv[2:]
a = argparse.Namespace(range=25.0, min_range=1.2, ignore_dist=1.0, max_height=2.5, min_pts=1, pd_everywhere=False,
                       camera_only=False, cam_json=None, dyn_speed=0.3, extra_ignore=None)
cfg = EvalConfig(match_dist=1.0, skip_missing_gt=False)
fp_rows, fn_rows, tp_rows = [], [], []
for w in wins:
    for rep in ('', 'r2', 'r3'):
        run = f'{R}bags/coda/runs/{tag}{rep}_v2_coda{w}'
        if not Path(run, 'metadata.yaml').exists():
            continue
        odom = load_odometry(f'{R}bags/coda/coda{w}_odom')
        gt, ign, frames, _ = ce.build_gt(f'{R}bags/coda/coda{w}_gt.csv', odom, a)
        pred, _, _ = load_obstacles(run, None)
        oz = ce.odom_ground_z(f'{R}bags/coda/coda{w}_odom')
        raw = objects(pred); raw_t = raw.t_ns.to_numpy() / 1e9
        pred, _, _ = ce.filter_pred(pred, ign, frames, odom, ce.odom_ground_z(f'{R}bags/coda/coda{w}_odom'), a)
        po = objects(pred)
        # net travel and size of each prediction track over its whole (kept) life
        life = po.sort_values('t_ns').groupby('track_id').agg(x0=('x', 'first'), y0=('y', 'first'), x1=('x', 'last'),
                                                               y1=('y', 'last'), n=('x', 'size'))
        travel = np.hypot(life.x1 - life.x0, life.y1 - life.y0).to_dict()
        grid = make_grid(gt, cfg)
        G = sample_gt(gt, grid, cfg)
        P = sample_pred(pred, grid, cfg)
        # ignored GT positions per annotated time
        ign_t = {t: g[['x', 'y']].to_numpy() for t, g in ign.groupby('t_ns')}
        ft = frames / 1e9
        # size lookup: nearest pred row by (track, time)
        po_by = {tid: g.sort_values('t_ns') for tid, g in po.groupby('track_id')}
        rpos = odom
        for k, t in enumerate(grid):
            gm = [g for g in G[k] if g[3] == DYNAMIC]
            pm = [p for p in P[k] if p[3] == DYNAMIC]
            m, d = _match(gm, pm, 1.0)
            mg = {i for i, _, _ in m}; mp = {j for _, j, _ in m}
            allg = np.array([[x, y] for _, x, y, _ in G[k]]) if G[k] else np.zeros((0, 2))
            alls = np.array([s for *_, s in G[k]])
            kk = int(np.argmin(np.abs(ft - t))); ig = ign_t.get(frames[kk], np.zeros((0, 2)))
            for j, (pid, x, y, _) in enumerate(pm):
                if j in mp:
                    tp_rows.append(dict(w=w, rep=rep, t=t, id=pid, x=x, y=y))
                    continue
                dm = [np.hypot(gx - x, gy - y) for _, gx, gy, _ in gm]
                near_m = [i for i, dd in enumerate(dm) if dd < 2.5]
                if near_m:
                    b = 'dup_moving' if any(i in mg for i in near_m) else 'off_moving'
                else:
                    st = allg[alls != DYNAMIC] if len(allg) else allg
                    if len(st) and np.min(np.hypot(st[:, 0] - x, st[:, 1] - y)) < 1.5:
                        b = 'still_obj'
                    elif len(ig) and np.min(np.hypot(ig[:, 0] - x, ig[:, 1] - y)) < 1.5:
                        b = 'ignored_obj'
                    else:
                        b = 'no_object'
                g = po_by[pid]; r = g.iloc[int(np.argmin(np.abs(g.t_ns.to_numpy() / 1e9 - t)))]
                rd = float(ce.robot_dist(pd.DataFrame(dict(t_ns=[int(t * 1e9)], x=[x], y=[y])), odom)[0])
                fp_rows.append(dict(w=w, rep=rep, t=t, id=pid, b=b, travel=travel.get(pid, 0), fp=max(r.l, r.w), h=r.h,
                                    rng=rd, x=x, y=y, age=r.score))
            pa = np.array([[x, y] for _, x, y, _ in P[k]]) if P[k] else np.zeros((0, 2))
            for i, (gid, x, y, _) in enumerate(gm):
                if i in mg:
                    continue
                dd = np.min(np.hypot(pa[:, 0] - x, pa[:, 1] - y)) if len(pa) else 99
                dmv = np.min([np.hypot(px - x, py - y) for _, px, py, _ in pm]) if pm else 99
                c = 'wrong_state' if dd < 1.0 else 'offset' if min(dd, dmv) < 2.5 else 'none'
                if c == 'none':
                    rr = raw[np.abs(raw_t - t) < 0.15]
                    if len(rr) and np.min(np.hypot(rr.x - x, rr.y - y)) < 1.0:
                        q = rr.iloc[[int(np.argmin(np.hypot(rr.x - x, rr.y - y)))]]
                        qd = float(ce.robot_dist(q, odom)[0])
                        qz = float(q.z.iloc[0] - np.interp(t, oz[:, 0], oz[:, 1]))
                        c = ('drop_far' if qd > 25 else 'drop_near' if qd < 1.2 else 'drop_height' if qz > 2.5
                             else 'drop_ignore_box')
                rd = float(ce.robot_dist(pd.DataFrame(dict(t_ns=[int(t * 1e9)], x=[x], y=[y])), odom)[0])
                fn_rows.append(dict(w=w, rep=rep, t=t, gid=gid, b=c, rng=rd))
F, N = pd.DataFrame(fp_rows), pd.DataFrame(fn_rows)
_S = str(Path(__file__).resolve().parent) + '/'
F.to_csv(f'{_S}fprows_{tag}.csv', index=False); pd.DataFrame(tp_rows).to_csv(f'{_S}tprows_{tag}.csv', index=False)
N.to_csv(f'{_S}fnrows_{tag}.csv', index=False)
print(f'== {tag} {" ".join(wins)}: moving FP {len(F)}  moving FN {len(N)}  (3 reps summed)')
if len(F):
    F['seq'] = F.w.str.split('_').str[0]
    print('FP buckets by seq (%):'); print((pd.crosstab(F.b, F.seq, normalize='columns') * 100).round(1).to_string())
    print('FP counts by seq:'); print(pd.crosstab(F.b, F.seq, margins=True).to_string())
    no = F[F.b == 'no_object']
    if len(no):
        print('no_object: static structure (life travel < 0.5 m) %.0f%%, footprint median %.2f m, h %.2f m' %
              (100 * (no.travel < .5).mean(), no.fp.median(), no.h.median()))
        print('  by footprint:', no.groupby(pd.cut(no.fp, [0, .8, 1.5, 3, 99]), observed=True).size().to_dict())
        print('  by range    :', no.groupby(pd.cut(no.rng, [0, 5, 10, 15, 20, 25]), observed=True).size().to_dict())
        print('  by height   :', no.groupby(pd.cut(no.h, [0, .5, 1.0, 1.5, 2.2, 99]), observed=True).size().to_dict())
    tops = F.groupby(['w', 'rep', 'id', 'b']).agg(n=('t', 'size'), fp=('fp', 'median'), h=('h', 'median'),
                                                 travel=('travel', 'first'), x=('x', 'median'), y=('y', 'median')).reset_index()
    print('track-level: %d FP tracks; top 12 by frames:' % len(tops))
    print(tops.sort_values('n', ascending=False).head(12).round(2).to_string(index=False))
    print('FP frames from tracks with >= 20 FP frames: %.0f%%' % (100 * tops[tops.n >= 20].n.sum() / max(1, tops.n.sum())))
if len(N):
    N['seq'] = N.w.str.split('_').str[0]
    print('FN buckets by seq:'); print(pd.crosstab(N.b, N.seq, margins=True).to_string())
    print('FN none by range:', N[N.b == 'none'].groupby(pd.cut(N[N.b == 'none'].rng, [0, 5, 10, 15, 20, 25]), observed=True).size().to_dict())
