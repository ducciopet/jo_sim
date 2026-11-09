"""Does a motion measure separate moving objects from still ones? On every matched, confirmed track row of the debug
CSV: label from the GT (moving > 0.5 m/s, still < 0.1 m/s, 'no GT' = nothing annotated within 2 m: structure,
unannotated parked cars), then per measure the share of moving rows above a threshold at a fixed share of 'no GT'
rows above it (and AUC). Measures: KF speed, the v1 point vote ratio, the new point-to-surface displacement
(pm_dist / pm_dt, and the share of points > 0.2 m off the old surface).
usage: pm_sep.py coda <tag> <win> [...] | street <csv>"""
import sys
from pathlib import Path
import numpy as np
import pandas as pd
sys.path.insert(0, str(Path(__file__).parent))
from funnel import gt_speed, REPO   # noqa: E402


def label(K, g):
    gt_t = np.unique(g.t_ns); G = {t: x for t, x in g.groupby('t_ns')}
    lab = []
    for t, x, y in zip(K.tn, K.ox, K.oy):
        k = gt_t[np.argmin(np.abs(gt_t - t))]
        if abs(k - t) > 0.06e9:
            lab.append('?'); continue
        q = G[k]; d = np.hypot(q.x - x, q.y - y)
        if d.min() < 1.0:
            s = q.spd.to_numpy()[np.argmin(d)]
            lab.append('moving' if s > 0.5 else 'still' if s < 0.1 else 'slow')
        else:
            lab.append('no GT' if d.min() > 2.0 else '?')
    return lab


def auc(pos, neg):
    if not len(pos) or not len(neg):
        return np.nan
    a = np.concatenate([pos, neg]); r = pd.Series(a).rank().to_numpy()
    return (r[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg))


def table(A):
    A = A[A.lab.isin(['moving', 'still', 'no GT'])].copy()
    ok = A.pm_dist >= 0
    A['pm_speed'] = np.where(ok, A.pm_dist / A.pm_dt.clip(lower=0.3), np.nan)
    print(f"rows: {A.lab.value_counts().to_dict()}; with the point measure: {ok.mean():.0%}")
    for fov in (None, 1, 0):
        S = A if fov is None else A[A.fov == fov]
        print(f"\n-- {'all' if fov is None else 'in FOV' if fov else 'out of FOV'} --")
        for m in ('speed', 'vote_ratio', 'pm_speed', 'pm_frac'):
            T = S.dropna(subset=[m]); T = T[T[m] >= 0] if m in ('vote_ratio', 'pm_frac') else T
            pos, still, bg = (T[T.lab == l][m].to_numpy() for l in ('moving', 'still', 'no GT'))
            if not len(pos) or not len(bg):
                continue
            line = f"{m:10s} n={len(pos):5d}/{len(still):5d}/{len(bg):6d} AUC mov-vs-noGT {auc(pos, bg):.3f} mov-vs-still {auc(pos, still):.3f} |"
            for fpr in (0.01, 0.05):
                th = np.quantile(bg, 1 - fpr)
                line += f" @noGT {fpr:.0%}: th {th:.2f} moving {np.mean(pos > th):.0%} still {np.mean(still > th) if len(still) else np.nan:.0%} |"
            print(line)
            if m == 'pm_speed':
                print('   median pm_speed  moving %.2f  still %.2f  noGT %.2f' % (np.median(pos), np.median(still) if len(still) else np.nan, np.median(bg)))


if __name__ == '__main__':
    out = []
    if sys.argv[1] == 'coda':
        tag = sys.argv[2]
        for w in sys.argv[3:]:
            c = w if '_' in w else f'0_{w}'
            g = pd.read_csv(REPO / f'bags/coda/coda{c}_gt.csv', keep_default_na=False)
            g = g[g.track_id.astype(int) >= 0].copy()
            for col in ('x', 'y'):
                g[col] = g[col].astype(float)
            g['track_id'] = g.track_id.astype(int); g['t_ns'] = g.t_ns.astype('int64'); g = g.sort_values('t_ns')
            g['spd'] = gt_speed(g)
            K = pd.read_csv(REPO / f'bags/coda/runs/{tag}_trk_{c}.csv')
            K = K[(K.matched == 1) & (K.confirmed == 1)].copy()
            K['tn'] = (K.t * 1e9).round().astype('int64')
            K['lab'] = label(K, g); K['win'] = c
            out.append(K)
    else:
        g = pd.read_csv(REPO / 'bags/mdp/mdp_street_gt.csv'); g = g[g.track_id >= 0].sort_values('t_ns').copy()
        g['spd'] = gt_speed(g)
        K = pd.read_csv(sys.argv[2]); K = K[(K.matched == 1) & (K.confirmed == 1)].copy()
        K['tn'] = (K.t * 1e9).round().astype('int64'); K['lab'] = label(K, g)
        out.append(K)
    table(pd.concat(out))
