"""Where does SINS lose the D (moving) and PD (still, movable class) objects? Every annotated object row (CODa windows
or the Marina di Pisa street) is followed through the pipeline using a diagnostic run with the per-track CSV
(profiles coda_dbg / mdp_street_dbg, tracker_node track_debug_csv) and given ONE cause, first failing stage wins:

  OK            output obstacle within 1 m with the right state (D -> dynamic, PD -> potentially_dynamic)
  wrong label   output obstacle within 1 m, other state
  no cluster    no fused detection within 1 m (1-2 m: 'cluster offset/merged')
  no track      a fused detection but no track within 1 m
  unconfirmed / missed (coasting) / suppressed (died still)
  for a D row held 'static' or 'PD': the first motion rule that failed on this tick (KF speed, N, C, corroboration,
  disocclusion, size gate, debounce)
  for a PD row held 'static': no class (in / out of camera FOV)

usage: funnel.py coda <tag> <win> [...]      (bags/coda/runs/<tag>_v2_coda<win>, <tag>_trk_<win>.csv)
       funnel.py street <run> <csv>
writes a per-row CSV next to this script (rows_<name>.csv) for the plots/tables.
"""
import sys
import collections
from pathlib import Path
import numpy as np
import pandas as pd
sys.path.insert(0, '/home/duccio/Desktop/jo_sim_sins/eval')
sys.path.insert(0, '/home/duccio/Desktop/jo_sim_sins/eval/diagnostics/coda_walkers')
from harness.ros_io import load_obstacles, load_odometry   # noqa: E402
from harness.tracks import objects                         # noqa: E402
from ped_funnel_lib import det3d, nearest                  # noqa: E402

REPO = Path('/home/duccio/Desktop/jo_sim_sins')
DYN = 0.3          # [m/s] GT state threshold, as coda_eval --dyn-speed
GATE = 1.0


def gt_speed(g):
    sp = []
    for _, gg in g.groupby('track_id'):
        tt = gg.t_ns.to_numpy() / 1e9; x, y = gg.x.to_numpy(), gg.y.to_numpy()
        v = [np.hypot(np.interp(a + .5, tt, x) - np.interp(a - .5, tt, x), np.interp(a + .5, tt, y) - np.interp(a - .5, tt, y))
             / max(1e-3, min(a + .5, tt[-1]) - max(a - .5, tt[0])) for a in tt]
        sp.append(pd.Series(v, index=gg.index))
    return pd.concat(sp)


def cause(q, K, KG, Kt, KI, D, Dt, O, Ot, gate):
    t = q.t_ns
    o = O[O.t_ns == Ot[np.argmin(np.abs(Ot - t))]]
    want = 'dynamic' if q.D else 'potentially_dynamic'
    if len(o):
        dd = np.hypot(o.x - q.x, o.y - q.y)
        if dd.min() < gate:
            st = o.state.to_numpy()[np.argmin(dd)]
            if st == want or (want in set(o.state[dd < gate])):
                return 'OK', None
            out_state = st
        else:
            out_state = None
    else:
        out_state = None
    kt = Kt[np.argmin(np.abs(Kt - t))]
    k = KG[kt]
    dk = np.minimum(np.hypot(k.x - q.x, k.y - q.y), np.hypot(k.ox - q.x, k.oy - q.y))
    k = k.assign(d=dk)[dk < gate]
    if len(k) == 0:
        de = [x for x in nearest(D, Dt, t) if np.hypot(x[0] - q.x, x[1] - q.y) < gate]
        if de:
            return 'no track (cluster there)', None
        de2 = [x for x in nearest(D, Dt, t) if np.hypot(x[0] - q.x, x[1] - q.y) < 2 * gate]
        return ('cluster offset/merged' if de2 else 'no cluster'), None
    tr = k.sort_values('d').iloc[0]
    st = tr.status
    if out_state is not None and st in ('dynamic', 'potentially_dynamic'):
        return f'wrong label ({out_state})', tr
    if st == 'unconfirmed':
        return 'track unconfirmed', tr
    if st == 'missed':
        de = [x for x in nearest(D, Dt, t) if np.hypot(x[0] - q.x, x[1] - q.y) < gate]
        return ('track coasting: cluster there, not associated' if de else 'track coasting: no cluster'), tr
    if st == 'suppressed':
        return 'track suppressed (still, no class)', tr
    if not q.D:   # PD row, track exists but not PD
        if st == 'dynamic':
            return 'wrong label (dynamic)', tr
        return 'PD: track held static (no class)', tr
    # D row, track static or PD
    if st == 'potentially_dynamic' and tr.cls != '-':
        pre = 'D held PD (classified)'
    else:
        pre = 'D held static' if st == 'static' else 'D held PD (motion-earned)'
    if not tr.speed_ok:
        return f'{pre}: KF speed < 0.35', tr
    if not tr.left_ok:
        return f'{pre}: N (not left birth spot)', tr
    if not tr.cons_ok:
        return f'{pre}: C (displacement < 0.5 KF)', tr
    if tr.disocc:
        return f'{pre}: disocclusion', tr
    if tr.cls == '-' and not (tr.kf_conf or tr.vote):
        return f'{pre}: no corroboration (KF std / vote)', tr
    if not tr.size_ok:
        return f'{pre}: size gate', tr
    return f'{pre}: debounce (dyn_ticks {min(int(tr.dyn_ticks), 9)}<{int(tr.confirm_ticks)})', tr


def run_one(run, csv, gt, odom=None, rng=(1.2, 25.0), name=''):
    K = pd.read_csv(csv)
    K['tn'] = (K.t * 1e9).round().astype('int64')
    Kt = np.unique(K.tn); KG = {t: g for t, g in K.groupby('tn')}
    D = det3d(run, '/sins/dbscan_detector/fused/detections3d'); Dt = np.array(sorted(D))
    O = objects(load_obstacles(run)[0]); Ot = np.unique(O.t_ns)
    rows = []
    for _, q in gt.iterrows():
        if odom is not None:
            rx, ry = np.interp(q.t_ns / 1e9, odom[:, 0], odom[:, 1]), np.interp(q.t_ns / 1e9, odom[:, 0], odom[:, 2])
            r = np.hypot(q.x - rx, q.y - ry)
        else:
            r = q.range
        if not rng[0] <= r <= rng[1]:
            continue
        gate = max(GATE, q.l / 2) if q.get('cls') == 'vehicle' else GATE
        c, tr = cause(q, K, KG, Kt, None, D, Dt, O, Ot, gate)
        rows.append(dict(win=name, t_ns=q.t_ns, gid=q.track_id, cls=q.cls, D=q.D, spd=q.spd, fov=bool(q.in_fov), rng=r,
                         npts=q.npts, cause=c,
                         tid=-1 if tr is None else int(tr.id), tspeed=np.nan if tr is None else tr.speed,
                         pm_dist=np.nan if tr is None else tr.pm_dist, pm_dt=np.nan if tr is None else tr.pm_dt))
    return pd.DataFrame(rows)


def coda(tag, wins):
    out = []
    for w in wins:
        c = w if '_' in w else f'0_{w}'
        pre = REPO / f'bags/coda/coda{c}'
        g = pd.read_csv(f'{pre}_gt.csv', keep_default_na=False)
        g = g[g.track_id.astype(int) >= 0].copy()
        for col in ('x', 'y', 'z', 'l', 'w', 'h', 'yaw', 'range'):
            g[col] = g[col].astype(float)
        g['track_id'] = g.track_id.astype(int); g['npts'] = g.npts.astype(int); g['t_ns'] = g.t_ns.astype('int64')
        g = g.sort_values('t_ns')
        g['spd'] = gt_speed(g); g['D'] = g.spd > DYN; g['cls'] = g['class']
        g['in_fov'] = g.in_fov.astype(str) == 'True'
        g = g[g.npts >= 1]
        odom = load_odometry(f'{pre}_odom')
        out.append(run_one(REPO / f'bags/coda/runs/{tag}_v2_coda{c}', REPO / f'bags/coda/runs/{tag}_trk_{c}.csv', g, odom,
                           name=c))
        print(c, len(out[-1]), file=sys.stderr)
    return pd.concat(out)


def street(run, csv):
    g = pd.read_csv(REPO / 'bags/mdp/mdp_street_gt.csv')
    g = g[g.track_id >= 0].sort_values('t_ns').copy()
    g['spd'] = gt_speed(g); g['D'] = True; g['cls'] = g['class']        # the street reference holds movers only
    return run_one(run, csv, g, None, rng=(1.2, 15.0), name='street')


def report(R, title):
    print(f'\n=== {title}: {len(R)} rows ===')
    for lab, S in (('D (moving)', R[R.D]), ('PD (still)', R[~R.D])):
        if not len(S):
            continue
        tab = pd.crosstab(S.cause, S.fov.map({True: 'in FOV', False: 'out FOV'}), normalize='columns').mul(100).round(1)
        tab['all %'] = (S.cause.value_counts(normalize=True) * 100).round(1)
        tab['rows'] = S.cause.value_counts()
        print(f'\n{lab}: {len(S)} rows, in FOV {S.fov.mean():.0%}')
        print(tab.sort_values('rows', ascending=False).to_string())
        S = S.assign(band=pd.cut(S.rng, [0, 5, 10, 15, 25]), ok=S.cause == 'OK')
        print('  OK by range band (in FOV / out):', {str(b): f"{x[x.fov].ok.mean():.0%} / {x[~x.fov].ok.mean():.0%} (n {len(x)})"
                                                  for b, x in S.groupby('band', observed=True)})


if __name__ == '__main__':
    if sys.argv[1] == 'coda':
        R = coda(sys.argv[2], sys.argv[3:]); name = f'coda_{sys.argv[2]}'
    else:
        R = street(sys.argv[2], sys.argv[3]); name = 'street'
    R.to_csv(Path(__file__).parent / f'rows_{name}.csv', index=False)
    report(R, name)
    if name.startswith('coda'):
        for s, S in R.groupby(R.win.str.split('_').str[0]):
            report(S, f'seq {s}')
