"""Disocclusion hits on D7: who is the occluder? Hits on tracks that are on a moving annotated pedestrian (victim ok)."""
import sys, re, collections
sys.path.insert(0, '/home/duccio/Desktop/jo_sim_sins/eval')
import numpy as np, pandas as pd
from harness.ros_io import load_odometry
pat = re.compile(r"\[WARN\] \[([\d.]+)\].*disocc_hit: track #(\d+) \(hits=(\d+)\) cand=\[([-\d.]+),([-\d.]+)\] by #(\d+) \(hits=(-?\d+) dyn_ticks=(-?\d+) best_class='([^']*)' size=\[([\d.]+),([\d.]+)\]\) at P=\[([-\d.]+),([-\d.]+)\] lidar=\[([-\d.]+),([-\d.]+)\]")
rows = []
for w in sys.argv[1:]:
    odom = load_odometry(f'bags/coda/coda{w}_odom')
    g = pd.read_csv(f'bags/coda/coda{w}_gt.csv'); g = g[(g.track_id >= 0) & (g['class'] == 'Pedestrian')]
    gt_t = np.unique(g.t_ns); GG = {t: x for t, x in g.groupby('t_ns')}
    K = pd.read_csv(f'bags/coda/runs/D7_trk_{w}.csv'); K['tn'] = (K.t * 1e9).round().astype('int64')
    L = [m.groups() for m in (pat.search(l) for l in open(f'bags/coda/runs/D7_v2_coda{w}_logs/det.log') if 'disocc_hit' in l) if m]
    A = np.array([[float(x) if i != 8 else 0 for i, x in enumerate(r)] for r in L]); cls = [r[8] for r in L]
    off = []
    for a in A[::max(1, len(A) // 400)]:
        d = np.hypot(odom[:, 1] - a[13], odom[:, 2] - a[14]); k = np.argmin(d)
        if d[k] < 0.05: off.append(odom[k, 0] - a[0])
    off = np.median(off)
    # occluder current position from the track CSV
    KO = {i: x.sort_values('tn') for i, x in K.groupby('id')}
    for a, c in zip(A, cls):
        t = (a[0] + off) * 1e9
        if t < gt_t[0] or t > gt_t[-1]: continue
        gg = GG[gt_t[np.argmin(np.abs(gt_t - t))]]
        dv = np.min(np.hypot(gg.x - a[3], gg.y - a[4])) if len(gg) else 9
        dP = np.min(np.hypot(gg.x - a[11], gg.y - a[12])) if len(gg) else 9
        ko = KO.get(int(a[5])); oc = None
        if ko is not None:
            j = np.argmin(np.abs(ko.tn.to_numpy() - t)); oc = ko.iloc[j]
            do = np.min(np.hypot(gg.x - oc.x, gg.y - oc.y)) if len(gg) else 9
        else: do = 9
        kv = KO.get(int(a[1]))
        if kv is not None:
            jv = np.argmin(np.abs(kv.tn.to_numpy() - t)); vc = kv.iloc[jv]
            vflags = dict(v_ev=bool(vc.speed_ok and (vc.kf_conf or vc.vote)), v_speed=vc.speed, v_sz=max(vc.sx, vc.sy), v_h=vc.sz, v_hits=vc.hits, v_fov=vc.fov)
        else: vflags = dict(v_ev=False, v_speed=np.nan, v_sz=np.nan, v_h=np.nan, v_hits=np.nan, v_fov=0)
        rL = np.hypot(a[3] - a[13], a[4] - a[14]); rP = np.hypot(a[11] - a[13], a[12] - a[14])
        occ_now_r = np.hypot(oc.x - a[13], oc.y - a[14]) if oc is not None else np.nan
        # angle between the occluder's CURRENT position and the victim, seen from the lidar
        ang = np.degrees(abs(np.angle(complex(a[3] - a[13], a[4] - a[14]) / complex(oc.x - a[13], oc.y - a[14])))) if oc is not None else np.nan
        rows.append(dict(**vflags, w=w, victim_on_ped=dv < 0.8, occ_P_on_ped=dP < 0.8, occ_now_on_ped=do < 0.8, rV=rL, rP=rP,
                         gap=rL - rP, occ_cls=c or '-', occ_dyn=a[7], occ_hits=a[6], occ_sz=max(a[9], a[10]),
                         occ_status=oc.status if oc is not None else '?', occ_missed=oc.missed if oc is not None else -1, occ_ts=(t-oc.tn)/1e9 if oc is not None else 0, ang_now=ang, occ_moved=np.hypot(oc.x - a[11], oc.y - a[12]) if oc is not None else np.nan))
R = pd.DataFrame(rows)

R.to_pickle('/tmp/claude-1000/-home-duccio-Desktop-jo-sim-sins/6b5f28fe-4bd1-41ab-95fb-b1ee18ac2c62/scratchpad/occ.pkl')
print('saved', len(R))
