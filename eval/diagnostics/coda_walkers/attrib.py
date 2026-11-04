"""Where does SINS lose a moving annotated pedestrian? D7 runs (per-track CSV). usage: attrib.py <tag> <win> [...]"""
import sys, collections
sys.path.insert(0, '/home/duccio/Desktop/jo_sim_sins/eval'); sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
import numpy as np, pandas as pd
from harness.ros_io import load_obstacles, load_odometry
from harness.tracks import objects
from ped_funnel_lib import det3d, nearest
tag = sys.argv[1]
C = collections.Counter(); EX = collections.defaultdict(list); N = 0; NOK = 0
for w in sys.argv[2:]:
    run = f'bags/coda/runs/{tag}_v2_coda{w}'; pre = f'bags/coda/coda{w}'
    K = pd.read_csv(f'bags/coda/runs/{tag}_trk_{w}.csv'); K['tn'] = (K.t * 1e9).round().astype('int64')
    Kt = np.unique(K.tn); KG = {t: g for t, g in K.groupby('tn')}
    KI = {i: g.sort_values('tn') for i, g in K.groupby('id')}
    odom = load_odometry(pre + '_odom')
    g = pd.read_csv(pre + '_gt.csv'); g = g[(g.track_id >= 0) & (g['class'] == 'Pedestrian')].sort_values('t_ns')
    sp = []
    for _, gg in g.groupby('track_id'):
        tt = gg.t_ns.to_numpy() / 1e9; x, y = gg.x.to_numpy(), gg.y.to_numpy()
        v = [np.hypot(np.interp(a + .5, tt, x) - np.interp(a - .5, tt, x), np.interp(a + .5, tt, y) - np.interp(a - .5, tt, y)) / (min(a + .5, tt[-1]) - max(a - .5, tt[0]) + 1e-9) for a in tt]
        sp.append(pd.Series(v, index=gg.index))
    g['spd'] = pd.concat(sp); g = g[(g.npts >= 20) & (g.spd > 0.5)]
    D = det3d(run, '/sins/dbscan_detector/fused/detections3d'); Dt = np.array(sorted(D))
    O = objects(load_obstacles(run)[0]); Ot = np.unique(O.t_ns)
    for _, q in g.iterrows():
        t = q.t_ns; rx, ry = np.interp(t / 1e9, odom[:, 0], odom[:, 1]), np.interp(t / 1e9, odom[:, 0], odom[:, 2])
        if not 1.2 < np.hypot(q.x - rx, q.y - ry) < 25: continue
        N += 1
        o = O[O.t_ns == Ot[np.argmin(np.abs(Ot - t))]]
        o = o[o.state == 'dynamic']
        if len(o) and np.min(np.hypot(o.x - q.x, o.y - q.y)) < 1.0: NOK += 1; continue
        fov = 'FOV' if q.in_fov else 'out'
        kt = Kt[np.argmin(np.abs(Kt - t))]; k = KG[kt]
        dk = np.minimum(np.hypot(k.x - q.x, k.y - q.y), np.hypot(k.ox - q.x, k.oy - q.y)); k = k.assign(d=dk)[dk < 1.0]
        if len(k) == 0:
            de = [x for x in nearest(D, Dt, t) if np.hypot(x[0] - q.x, x[1] - q.y) < 1.0]
            de2 = [x for x in nearest(D, Dt, t) if np.hypot(x[0] - q.x, x[1] - q.y) < 2.0]
            r = 'A cluster, no track at all' if de else ('A cluster 1-2 m off (merged)' if de2 else 'A no cluster')
        else:
            tr = k.sort_values('d').iloc[0]; st = tr.status
            if st == 'potentially_dynamic': r = 'B published PD (classified person, judged slow)'
            elif st == 'dynamic': r = 'B dynamic but >1 m off / other'
            elif st == 'unconfirmed': r = f'C unconfirmed track (hits {min(tr.hits, 5)})'
            elif st == 'missed': r = 'C track coasting (unmatched this tick)'
            elif st == 'suppressed': r = 'D KF speed < 0.07 (suppressed)'
            elif tr.cls != '-': r = f'E static with class {tr.cls}'
            elif tr.speed < 0.25: r = 'F KF speed 0.07-0.25'
            else:
                h = KI[tr.id]; h = h[(h.tn <= kt) & (h.tn > kt - 3e9) & (h.status != 'missed')]
                if not tr.size_ok: r = 'G size gate'
                else:
                    z = h[(h.dyn_ticks == 0)]
                    if len(z) == 0: r = 'H debounce: never reset in 3 s, too young'
                    else:
                        b = z.iloc[-1]; age = (kt - b.tn) / 1e9
                        if b.speed < 0.25: why = 'KF speed dip <0.25'
                        elif b.disocc: why = 'disocclusion'
                        elif not (b.kf_conf or b.vote): why = 'no corroboration (KF std & point vote)'
                        else: why = 'other'
                        r = f'H debounce: last reset by {why}'
            EX[r].append((tr.speed, tr.hits, tr.sx, tr.sy, tr.sz, tr.dyn_ticks, tr.confirm_ticks))
        C[(r, fov)] += 1
print(f'moving GT rows {N}, found (dynamic within 1 m) {NOK / N:.1%}')
R = pd.Series(C).unstack(fill_value=0); R['tot'] = R.sum(1); R = R.sort_values('tot', ascending=False)
R['% of all'] = (100 * R.tot / N).round(1)
print(R.to_string())
for r, v in sorted(EX.items()):
    a = np.array(v, float); print(f'{r:55s} n={len(a):5d} speed {np.median(a[:,0]):.2f} hits {np.median(a[:,1]):.0f} size {np.median(a[:,2]):.2f}x{np.median(a[:,3]):.2f}x{np.median(a[:,4]):.2f} dyn {np.median(a[:,5]):.0f}/{np.median(a[:,6]):.0f}')
