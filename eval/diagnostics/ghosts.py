#!/usr/bin/env python3
"""What are the 'ghost' false positives of err_decomp.py (unmatched at 1 m, no annotated object within 3 m)? (20/11)
Per ghost row: published state, box size, speed, robot distance, height above ground, track age; per track: how much
of its published life is ghost (a whole ghost track vs a real track that drifted away).
usage: ghosts.py --runs DIR [...] --windows w1 10_w1 [--frames testing]
"""
import argparse, sys
from pathlib import Path
import numpy as np, pandas as pd
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import coda_eval as ce        # noqa: E402
import err_decomp as ed       # noqa: E402


def rows(run, c, a, B):
    ns = argparse.Namespace(range=20.0, ignore_dist=1.0, min_range=1.2, max_height=2.5, min_pts=1, pd_everywhere=False,
                            camera_only=False, cam_json=None, ignore_undefined=True, dyn_speed=0.3, match_dist=1.0,
                            points_min_cluster=20, extra_ignore=None, extra_ignore_dist=0.3, extra_ignore_dt=0.5,
                            frames='all', coda_root=a.coda_root, ignore_gt=None, topic=None, method='sins')
    odom = ce.load_odometry(B / f'coda{c}_odom'); oz = ce.odom_ground_z(B / f'coda{c}_odom')
    gt, ign, frames, _ = ce.build_gt(str(B / f'coda{c}_gt.csv'), odom, ns)
    pred, _, _ = ce.load_obstacles(run, None)
    pred, _, _ = ce.filter_pred(pred, ign, frames, odom, oz, ns)
    pred = ce.objects(pred).copy()
    pred['rd'] = ce.robot_dist(pred, odom)
    pred['hz'] = pred.z - np.interp(pred.t_ns / 1e9, oz[:, 0], oz[:, 1])
    pt = np.unique(pred.t_ns.to_numpy()); out = []
    for t, G in gt.groupby('t_ns'):
        k = pt[np.argmin(np.abs(pt - t))]
        if abs(k - t) >= 0.06e9: continue
        P = pred[pred.t_ns == k].reset_index(drop=True); G = G.reset_index(drop=True)
        p1 = {i for i, _ in ed.frame_pairs(P, G, 1.0)}
        for i, p in P.iterrows():
            d = np.hypot(G.x - p.x, G.y - p.y).min() if len(G) else 99
            out.append(dict(win=c, run=run, t=t, tid=p.track_id, x=p.x, y=p.y, state=p.state, l=p.l, w=p.w, h=p.h,
                            v=np.hypot(p.vx, p.vy), rd=p.rd, hz=p.hz, age=p.score,
                            kind='TP' if i in p1 else ('ghost' if d >= 3.0 else 'otherFP')))
    return pd.DataFrame(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', nargs='+', required=True); ap.add_argument('--windows', nargs='+', required=True)
    ap.add_argument('--coda-root', default='/mnt/nas-imperatore/datasets_SINS/CODa')
    ap.add_argument('--bags', default=str(Path(__file__).resolve().parents[2] / 'bags/coda'))
    ap.add_argument('--csv', default=None)
    a = ap.parse_args(); B = Path(a.bags)
    df = pd.concat([rows(t.format(c=w if '_' in w else f'0_{w}'), w if '_' in w else f'0_{w}', a, B)
                    for t in a.runs for w in a.windows])
    if a.csv: df.to_csv(a.csv, index=False)
    print('rows by kind:', df.kind.value_counts().to_dict())
    print('\nstate x kind (rows):'); print(pd.crosstab(df.state, df.kind))
    q = lambda s: s.quantile([.1, .5, .9]).round(2).tolist()
    for k in ['TP', 'ghost']:
        g = df[df.kind == k]
        print(f'\n{k}: l {q(g.l)}  w {q(g.w)}  h {q(g.h)}  speed {q(g.v)}  robot dist {q(g.rd)}  height above ground {q(g.hz)}  age {q(g.age)}')
    # per track: share of rows that are ghost
    tr = df.groupby(['run', 'win', 'tid']).agg(n=('kind', 'size'), ghost=('kind', lambda s: (s == 'ghost').mean()),
                                               state=('state', lambda s: s.mode().iat[0]))
    gtr = tr[tr.ghost > 0]
    print(f'\ntracks with ghost rows: {len(gtr)} of {len(tr)};  ghost rows from tracks that are >=90% ghost: '
          f'{int((df.kind.eq("ghost") & df.set_index(["run","win","tid"]).index.isin(gtr[gtr.ghost>=.9].index)).sum())} of {int(df.kind.eq("ghost").sum())}')
    print('rows per ghost-only track (>=90%):', q(gtr[gtr.ghost >= .9].n), ' by state:', gtr[gtr.ghost >= .9].state.value_counts().to_dict())


if __name__ == '__main__':
    main()
