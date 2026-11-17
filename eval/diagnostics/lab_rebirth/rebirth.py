"""Why do tracks of moving objects die and get re-born? DBG2 per-tick tracker CSV vs GT (tuning only).
For each moving GT object (GT speed >= 0.5 m/s over the frame), each GT frame -> nearest tracker track within 1 m.
Consecutive distinct track ids A->B on the same object = a handover. Classify:
  overlap   B born while A still alive (A coasting or matched elsewhere) -> association failure (detection not given to A)
  gap       B born after A deleted -> A ran out of coast; dt = B birth - A last matched; was A missed>0 at the end?
Also A state at its end: speed, last |innovation| (x-ox), hits, age."""
import sys, numpy as np, pandas as pd
def gt_moving(f, vmin=0.5):
    g = pd.read_csv(f, keep_default_na=False); g = g[g.track_id.astype(int) >= 0].copy(); g[['x','y']] = g[['x','y']].astype(float)
    g['t'] = g.t_ns.astype('int64') / 1e9; g = g.sort_values(['track_id', 't'])
    for c in 'xy': g['v' + c] = g.groupby('track_id')[c].diff() / g.groupby('track_id').t.diff()
    g['v'] = np.hypot(g.vx, g.vy); g['v'] = g.groupby('track_id').v.transform(lambda s: s.rolling(5, center=True, min_periods=1).median())
    return g[g.v >= vmin]
def run(trk, gtf, name):
    T = pd.read_csv(trk); G = gt_moving(gtf)
    T['t'] = T.t.round(2); ts = np.array(sorted(T.t.unique()))
    byt = {t: g for t, g in T.groupby('t')}
    life = T.groupby('id').agg(t0=('t', 'min'), t1=('t', 'max'))
    lastm = T[T.matched == 1].groupby('id').t.max()
    assoc, dists = [], []
    for _, r in G.iterrows():
        k = ts[np.argmin(np.abs(ts - r.t))]
        if abs(k - r.t) > 0.06: continue
        g = byt[k]; d = np.hypot(g.x - r.x, g.y - r.y)
        dists.append(d.min())
        if d.min() < 1.0: assoc.append((r.track_id, r['class'], r.t, int(g.id.iloc[int(np.argmin(d.values))]), r.v))
    A = pd.DataFrame(assoc, columns=['gid', 'cls', 't', 'tid', 'v'])
    rows = []
    for gid, a in A.groupby('gid'):
        a = a.sort_values('t'); seq = a.tid.to_numpy()
        for i in range(1, len(seq)):
            if seq[i] == seq[i - 1]: continue
            pa, b = seq[i - 1], seq[i]
            if b in seq[:i]: rows.append(dict(name=name, gid=gid, cls=a['class'].iloc[0] if 'class' in a else a.cls.iloc[0], kind='switch_back')); continue
            tb = life.t0[b]; ta1 = life.t1[pa]; tam = lastm.get(pa, life.t0[pa])
            ea = T[(T.id == pa)].sort_values('t'); e = ea.iloc[-1]
            # A's state at the time B was born (or at its end)
            sa = ea[ea.t <= tb].iloc[-1] if (ea.t <= tb).any() else e
            kind = 'overlap' if tb <= ta1 else 'gap'
            bo = T[(T.id == b)].sort_values('t').iloc[0]
            rows.append(dict(name=name, gid=gid, a=pa, b=b, cls=a.cls.iloc[0], kind=kind, v=a.v.iloc[i], tb_minus_lastmatch=tb - tam,
                             a_missed=sa.missed, a_matched=sa.matched, a_age=sa.t - life.t0[pa], a_spd=sa.speed, a_hits=sa.hits,
                             a_status=sa.status, dist_ab=np.hypot(bo.x - sa.x, bo.y - sa.y), a_alive_after=ta1 - tb))
    return pd.DataFrame(rows), A, np.array(dists)
R = '/home/punk-opc/Desktop/jo_sim_sins/'
out = []
P = sys.argv[2] if len(sys.argv) > 2 else 'DBG2'
W = sys.argv[3].split(',') if len(sys.argv) > 3 else ['0_w1', '0_w2', '0_w3', '10_w1', '10_w2', '10_w3', '10_w4', '10_w5', '16_w1']
for w in W:
    if w == 'street': h, A, d = run(f'{R}bags/mdp/runs/{P}_trk_street.csv', f'{R}bags/mdp/mdp_street_gt.csv', w)
    else: h, A, d = run(f'{R}bags/coda/runs/{P}_trk_{w}.csv', f'{R}bags/coda/coda{w}_gt.csv', w)
    print(w, 'moving GT rows', len(d), 'with track<1m %.0f%%' % (100 * (d < 1).mean()), 'objects', A.gid.nunique(), 'handovers', len(h))
    out.append(h)
H = pd.concat(out).reset_index(drop=True); H.to_csv(sys.argv[1] if len(sys.argv) > 1 else 'handovers.csv', index=False)
H['cls2'] = np.where(H.cls.str.lower().isin(['pedestrian','person']), 'ped', 'other')
print(pd.crosstab(H.kind, H.cls2, margins=True))
for k, s in H[H.kind != 'switch_back'].groupby('kind'):
    print('==', k, len(s)); print(s[['tb_minus_lastmatch', 'a_missed', 'a_age', 'a_spd', 'a_hits', 'dist_ab', 'a_alive_after', 'v']].describe(percentiles=[.25, .5, .75]).round(2).to_string())
    print(' A status:', s.a_status.value_counts().to_dict()); print(' A matched at B birth:', s.a_matched.value_counts().to_dict())
