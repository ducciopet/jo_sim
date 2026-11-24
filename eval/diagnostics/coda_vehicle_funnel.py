#!/usr/bin/env python3
"""CODa vehicle-loss funnel (24/11; offline, existing runs). Same GT / ignore / prediction filtering as coda_eval e3.
Per annotated frame, Hungarian at 1 m between ALL GT objects and published obstacles (as the metric). For each vehicle GT row:
  TP          matched at 1 m
  offset      not matched; a published obstacle within 2 m of the GT centre (found, centre 1-2 m off: FN + FP)
  taken       not matched; an obstacle within 1 m was assigned to another GT object
  eval drop   a published obstacle within 1 m before the e3 filter (ignore zone / range / height)
  held static a fused detection inside the GT box (+0.3 m) and a tracker track inside it in state 'static'
  unconfirmed fused detection + a non-static tracker track inside the box that is not published
  cluster only fused detection inside the box, no tracker track inside it
  no cluster  no fused detection inside the box
Split by: GT moving (state dynamic) or still; range 0-10 / 10-15 / 15-20 m; bearing |b| <= 35 deg (front camera) or not;
GT length < 6 m or >= 6 m. Also per row: fused detections inside the box (fragmentation) and, for TP/offset rows, the
published centre error along the robot->GT ray (negative = short, towards the sensor).
Unmatched published obstacles near vehicles (within 1.5 m of a vehicle box) are counted per window as vehicle FP.
usage: coda_vehicle_funnel.py --tags TAG [...] [--windows ...] [--frames all|testing] [--method sins|eagermot --csv TMPL]"""
import argparse, json, re, sys, collections
from pathlib import Path
import numpy as np, pandas as pd
from scipy.optimize import linear_sum_assignment
from rosbags.highlevel import AnyReader
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import coda_eval as ce   # noqa: E402

VEH = ['Car', 'Pickup Truck', 'Utility Vehicle', 'Service Vehicle', 'Delivery Truck', 'Bus', 'Truck', 'Van']
ap = argparse.ArgumentParser()
ap.add_argument('--tags', nargs='+', required=True)
ap.add_argument('--windows', nargs='+', default='w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1'.split())
ap.add_argument('--frames', default='all')
ap.add_argument('--method', default='sins')
ap.add_argument('--csv', default='/mnt/nas-imperatore/datasets_SINS/derived/baselines_out/eagermot_mc_out/eagermot_coda{c}.csv')
ap.add_argument('--coda-root', default='/mnt/nas-imperatore/datasets_SINS/CODa')
a = ap.parse_args()
B = Path(__file__).resolve().parents[2] / 'bags/coda'
ns = argparse.Namespace(range=20.0, ignore_dist=1.0, min_range=1.2, max_height=2.5, min_pts=1, pd_everywhere=False, camera_only=False,
                        cam_json=None, ignore_undefined=True, dyn_speed=0.3, match_dist=1.0, points_min_cluster=20, extra_ignore=None,
                        extra_ignore_dist=0.3, extra_ignore_dt=0.5, frames='all', coda_root=a.coda_root, ignore_gt=None, topic=None,
                        method=a.method, ignore_box=False)
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
CAUSES = ['offset', 'taken', 'eval drop', 'held static', 'unconfirmed', 'cluster only', 'no cluster']


def inside(px, py, g, m):
    c, s = np.cos(g.yaw), np.sin(g.yaw); dx, dy = px - g.x, py - g.y
    return (np.abs(dx * c + dy * s) <= g.l / 2 + m) & (np.abs(-dx * s + dy * c) <= g.w / 2 + m)


def window(c):
    odom = ce.load_odometry(B / f'coda{c}_odom')
    gt, ign, frames, _ = ce.build_gt(str(B / f'coda{c}_gt.csv'), odom, ns)
    raw = pd.read_csv(B / f'coda{c}_gt.csv')
    g1 = raw.drop_duplicates('track_id').set_index('track_id')
    dims = raw.groupby('track_id')[['l', 'w']].median()
    gt = gt.assign(cls=gt.track_id.map(g1['class']), l=gt.track_id.map(dims.l).fillna(0.5), w=gt.track_id.map(dims.w).fillna(0.5),
                   yaw=gt.yaw.fillna(0.0))
    gt['veh'] = gt.cls.isin(VEH)
    keep = None
    if a.frames != 'all':
        info = json.load(open(B / f'coda{c}_gt.json'))
        meta = json.load(open(Path(a.coda_root) / 'metadata' / f"{info['seq']}.json"))['ObjectTracking'][a.frames]
        split = {int(re.findall(r'_(\d+)\.json$', f)[0]) for f in meta}
        keep = {t for f, t in zip(info['annotated_frames'], info['annotated_t_ns']) if int(f) in split}
    return odom, gt, ign, frames, keep


def bagmsgs(run):
    det, trk = {}, {}
    with AnyReader([Path(run)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic in ('/sins/dbscan_detector/fused/detections3d', '/sins/tracker/tracks')]):
            m = r.deserialize(raw, c.msgtype)
            L = np.array([(d.bbox.center.position.x, d.bbox.center.position.y) for d in m.detections]).reshape(-1, 2)
            S = [d.results[0].hypothesis.class_id if d.results else '' for d in m.detections]
            (det if 'dbscan' in c.topic else trk)[st(m.header)] = (L, S)
    return det, trk


print(f'frames {a.frames}: vehicle GT rows by outcome (sum over windows) and vehicle FP')
for tag in a.tags:
    F = collections.Counter(); frag = collections.defaultdict(list); rad = collections.defaultdict(list); FPv = collections.Counter()
    for w in a.windows:
        c = w if '_' in w else f'0_{w}'
        odom, gt, ign, frames, keep = window(c)
        if not gt.veh.any(): continue
        if a.method == 'sins':
            run = B / 'runs' / f'{tag}_v2_coda{c}'
            if not run.exists(): print('missing', run, file=sys.stderr); continue
            pred0, _, _ = ce.load_obstacles(str(run), None)
            det, trk = bagmsgs(run); dt, tt = np.array(sorted(det)), np.array(sorted(trk))
        else:
            pred0 = ce.load_csv_tracks(a.csv.format(c=c)); det = None
        gz = ce.odom_ground_z(B / f'coda{c}_odom')
        raw = pred0[pred0.track_id >= 0]; rt = np.unique(raw.t_ns.to_numpy()) if len(raw) else np.array([0])
        pred, _, _ = ce.filter_pred(pred0, ign, frames, odom, gz, ns)
        pred = pred[pred.track_id >= 0]
        pt = np.unique(pred.t_ns.to_numpy()) if len(pred) else np.array([0])
        for t, G in gt[gt.track_id >= 0].groupby('t_ns'):
            if keep is not None and t not in keep: continue
            k = pt[np.argmin(np.abs(pt - t))]
            P = pred[pred.t_ns == k] if abs(k - t) < 0.06e9 else pred.iloc[:0]
            G = G.reset_index(drop=True); P = P.reset_index(drop=True); m = {}
            if len(P):
                Dm = np.hypot(P.x.to_numpy()[:, None] - G.x.to_numpy()[None], P.y.to_numpy()[:, None] - G.y.to_numpy()[None])
                r_, c_ = linear_sum_assignment(np.where(Dm <= 1.0, Dm, 1e6))
                m = {j: i for i, j in zip(r_, c_) if Dm[i, j] <= 1.0}
            mp = set(m.values())
            ti = np.argmin(np.abs(odom[:, 0] - t / 1e9)); rx, ry, ryaw = odom[ti, 1], odom[ti, 2], odom[ti, 3] if odom.shape[1] > 3 else 0.0
            if det is not None:
                Ld, _ = det[dt[np.argmin(np.abs(dt - t))]]; Lt, St = trk[tt[np.argmin(np.abs(tt - t))]]
            V = G[G.veh]
            for i, p in P.iterrows():   # vehicle FP: unmatched obstacle within 1.5 m of a vehicle box
                if i in mp or not len(V): continue
                if inside(p.x, p.y, V, 1.5).any(): FPv['fp'] += 1
            for j, g in V.iterrows():
                ray = np.array([g.x - rx, g.y - ry]); rng = np.linalg.norm(ray); ray /= max(rng, 1e-6)
                key = dict(mov='moving' if g.state == 'dynamic' else 'still', rng='0-10' if rng < 10 else '10-15' if rng < 15 else '15-20',
                           len='L<6' if g.l < 6 else 'L>=6', cls=g.cls,
                           fov='front' if abs(np.degrees(np.arctan2(np.sin(np.arctan2(ray[1], ray[0]) - ryaw), np.cos(np.arctan2(ray[1], ray[0]) - ryaw)))) <= 35 else 'side/back')
                if j in m: cause = 'TP'
                elif len(P) and np.hypot(P.x - g.x, P.y - g.y).min() <= 1.0: cause = 'taken'
                elif len(P) and np.hypot(P.x - g.x, P.y - g.y).min() <= 2.0: cause = 'offset'
                else:
                    kr = rt[np.argmin(np.abs(rt - t))]; RP = raw[raw.t_ns == kr]
                    if abs(kr - t) < 0.06e9 and len(RP) and np.hypot(RP.x - g.x, RP.y - g.y).min() <= 1.0: cause = 'eval drop'
                    elif det is None: cause = 'no cluster'
                    else:
                        din = inside(Ld[:, 0], Ld[:, 1], g, 0.3) if len(Ld) else np.zeros(0, bool)
                        tin = inside(Lt[:, 0], Lt[:, 1], g, 0.3) if len(Lt) else np.zeros(0, bool)
                        if din.any():
                            if tin.any(): cause = 'held static' if all(St[q] == 'static' for q in np.where(tin)[0]) else 'unconfirmed'
                            else: cause = 'cluster only'
                        else: cause = 'no cluster'
                for kk in ['all'] + [f'{n}={v}' for n, v in key.items()]:
                    F[(kk, 'rows')] += 1; F[(kk, cause)] += 1
                if det is not None and len(Ld):
                    frag[cause].append(int(inside(Ld[:, 0], Ld[:, 1], g, 0.3).sum()))
                if cause in ('TP', 'offset') and len(P):
                    q = P.iloc[int(np.argmin(np.hypot(P.x - g.x, P.y - g.y)))]
                    rad[key['len']].append(float(np.dot([q.x - g.x, q.y - g.y], ray)))
    keys = ['all'] + sorted({k for k, _ in F if k != 'all'})
    for kk in keys:
        n = F[(kk, 'rows')]
        if not n: continue
        print(f'{tag:6s} {kk:24s} rows {n:5d} TP {100 * F[(kk, "TP")] / n:5.1f}% | ' +
              ', '.join(f'{cz} {100 * F[(kk, cz)] / n:.1f}' for cz in CAUSES))
    print(f'{tag:6s} vehicle FP (unmatched obstacles within 1.5 m of a vehicle box): {FPv["fp"]}')
    if frag:
        print(f'{tag:6s} fused detections inside the GT box, mean by outcome: ' +
              ', '.join(f'{cz} {np.mean(v):.2f} (n {len(v)})' for cz, v in sorted(frag.items())))
    for L, v in sorted(rad.items()):
        print(f'{tag:6s} radial centre error {L} (TP+offset): median {np.median(v):+.2f} m, |err|>1 m {100 * np.mean(np.abs(v) > 1):.0f}%')
    sys.stdout.flush()
