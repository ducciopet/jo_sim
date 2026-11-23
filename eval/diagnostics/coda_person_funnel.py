#!/usr/bin/env python3
"""CODa person-loss funnel and person ID-switch causes from existing runs (23/11; offline, no per-track debug CSV).
Same GT / ignore / prediction filtering as coda_eval e3. Per annotated frame, Hungarian at 1 m between GT and published obstacles.
Each UNMATCHED person GT row (moving = GT state dynamic; also all) is attributed, first from the published output:
  taken by other GT  a published obstacle within 1 m exists but the assignment gave it to another GT object (crowd)
  eval: ignore zone  a published obstacle within 1 m is dropped by the e3 filter because it lies within ignore_dist (1 m) of an
                     ignored object (undefined state / not visible), although it sits on this counted person
  eval: range/height dropped by the range (> 20 m / < 1.2 m from the robot) or height filter
then from the run bag at the nearest tick:
  held static     a fused detection within 1 m AND a tracker track within 1 m in state 'static'
  unconfirmed     a fused detection within 1 m AND a tracker track within 1 m in D/PD that is not published
  coast+cluster   a fused detection within 1 m, no tracker track within 1 m but one within 2.5 m (coasting next to it)
  cluster only    a fused detection within 1 m, no tracker track within 2.5 m
  coast no clus.  no fused detection within 1 m, a tracker track within 1 m (coasting / held by prediction)
  no cluster      no fused detection and no tracker track within 1 m
(An approximation of the laptop's 20/11 funnel, which used track_debug_csv: coasting is inferred from cluster absence.)
Person ID switches (matched published id changes along a GT person track): the new id is
  takeover   it was already published >= 0.5 s before the switch (a pre-existing track takes the person over), split by
             whether it had been matched to another GT object before
  re-birth   it is new (first published < 0.5 s before)
usage: coda_person_funnel.py --tags TAG [...] [--windows ...] [--frames all|testing]"""
import argparse, json, re, sys, collections
from pathlib import Path
import numpy as np, pandas as pd
from scipy.optimize import linear_sum_assignment
from rosbags.highlevel import AnyReader
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import coda_eval as ce   # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--tags', nargs='+', required=True)
ap.add_argument('--windows', nargs='+', default='w1 w2 w3 10_w1 10_w2 10_w3 10_w4 10_w5 16_w1'.split())
ap.add_argument('--frames', default='all')
ap.add_argument('--coda-root', default='/mnt/nas-imperatore/datasets_SINS/CODa')
a = ap.parse_args()
B = Path(__file__).resolve().parents[2] / 'bags/coda'
ns = argparse.Namespace(range=20.0, ignore_dist=1.0, min_range=1.2, max_height=2.5, min_pts=1, pd_everywhere=False, camera_only=False,
                        cam_json=None, ignore_undefined=True, dyn_speed=0.3, match_dist=1.0, points_min_cluster=20, extra_ignore=None,
                        extra_ignore_dist=0.3, extra_ignore_dt=0.5, frames='all', coda_root=a.coda_root, ignore_gt=None, topic=None,
                        method='sins', ignore_box=False)
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
CAUSES = ['taken by other GT', 'eval: ignore zone', 'eval: range/height', 'held static', 'unconfirmed', 'coast+cluster', 'cluster only', 'coast no clus.', 'no cluster']
cache = {}
def window(c):
    if c not in cache:
        odom = ce.load_odometry(B / f'coda{c}_odom')
        gt, ign, frames, _ = ce.build_gt(str(B / f'coda{c}_gt.csv'), odom, ns)
        raw = pd.read_csv(B / f'coda{c}_gt.csv')
        cls = gt.track_id.map(raw[['track_id', 'class']].drop_duplicates('track_id').set_index('track_id')['class'])
        gt = gt.assign(ped=(cls == 'Pedestrian'))
        keep = None
        if a.frames != 'all':
            info = json.load(open(B / f'coda{c}_gt.json'))
            meta = json.load(open(Path(a.coda_root) / 'metadata' / f"{info['seq']}.json"))['ObjectTracking'][a.frames]
            split = {int(re.findall(r'_(\d+)\.json$', f)[0]) for f in meta}
            keep = {t for f, t in zip(info['annotated_frames'], info['annotated_t_ns']) if int(f) in split}
        cache[c] = (odom, gt, ign, frames, keep)
    return cache[c]
def bagmsgs(run):
    det, trk = {}, {}
    with AnyReader([Path(run)]) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic in ('/sins/dbscan_detector/fused/detections3d', '/sins/tracker/tracks')]):
            m = r.deserialize(raw, c.msgtype)
            L = np.array([(d.bbox.center.position.x, d.bbox.center.position.y) for d in m.detections]).reshape(-1, 2)
            S = [d.results[0].hypothesis.class_id if d.results else '' for d in m.detections]
            (det if 'dbscan' in c.topic else trk)[st(m.header)] = (L, S)
    return det, trk
print(f'frames {a.frames}: person GT rows not found at 1 m, by cause (sum over windows); person ID switches by cause')
for tag in a.tags:
    F = collections.Counter(); I = collections.Counter()
    for w in a.windows:
        c = w if '_' in w else f'0_{w}'
        run = B / 'runs' / f'{tag}_v2_coda{c}'
        if not run.exists(): print('missing', run, file=sys.stderr); continue
        odom, gt, ign, frames, keep = window(c)
        pred, _, _ = ce.load_obstacles(str(run), None)
        raw = pred[pred.track_id >= 0]; rt = np.unique(raw.t_ns.to_numpy()) if len(raw) else np.array([0])
        noig, _, _ = ce.filter_pred(pred, ign.iloc[:0], frames, odom, ce.odom_ground_z(B / f'coda{c}_odom'), ns)
        noig = noig[noig.track_id >= 0]; nt = np.unique(noig.t_ns.to_numpy()) if len(noig) else np.array([0])
        pred, _, _ = ce.filter_pred(pred, ign, frames, odom, ce.odom_ground_z(B / f'coda{c}_odom'), ns)
        pred = pred[pred.track_id >= 0]
        det, trk = bagmsgs(run); dt, tt = np.array(sorted(det)), np.array(sorted(trk))
        pt = np.unique(pred.t_ns.to_numpy()) if len(pred) else np.array([0])
        first = pred.groupby('track_id').t_ns.min().to_dict()
        lastmatch = {}; matched_other = collections.defaultdict(set)
        for t, G in gt[gt.track_id >= 0].groupby('t_ns'):
            if keep is not None and t not in keep: continue
            k = pt[np.argmin(np.abs(pt - t))]
            P = pred[pred.t_ns == k] if abs(k - t) < 0.06e9 else pred.iloc[:0]
            G = G.reset_index(drop=True); P = P.reset_index(drop=True); m = {}
            if len(P):
                Dm = np.hypot(P.x.to_numpy()[:, None] - G.x.to_numpy()[None], P.y.to_numpy()[:, None] - G.y.to_numpy()[None])
                r_, c_ = linear_sum_assignment(np.where(Dm <= 1.0, Dm, 1e6))
                m = {j: int(P.track_id[i]) for i, j in zip(r_, c_) if Dm[i, j] <= 1.0}
            for j, pid in m.items():
                gid = int(G.track_id[j])
                if G.ped[j] and gid in lastmatch and lastmatch[gid] != pid:
                    pre = first.get(pid, t) < t - 0.5e9
                    I['takeover from another GT' if pre and (matched_other[pid] - {gid}) else 'takeover' if pre else 're-birth'] += 1
                if G.ped[j]: lastmatch[gid] = pid
                matched_other[pid].add(gid)
            Ld, _ = det[dt[np.argmin(np.abs(dt - t))]]; Lt, St = trk[tt[np.argmin(np.abs(tt - t))]]
            for j, g in G.iterrows():
                if not g.ped: continue
                mv = g.state == 'dynamic'
                F[('all', 'rows')] += 1; F[('mov', 'rows')] += mv
                if j in m: continue
                if len(P) and np.hypot(P.x - g.x, P.y - g.y).min() <= 1.0:
                    F[('all', 'taken by other GT')] += 1; F[('mov', 'taken by other GT')] += mv; continue
                kr = rt[np.argmin(np.abs(rt - t))]; RP = raw[raw.t_ns == kr]
                if abs(kr - t) < 0.06e9 and len(RP) and np.hypot(RP.x - g.x, RP.y - g.y).min() <= 1.0:
                    kn = nt[np.argmin(np.abs(nt - t))]; NP = noig[noig.t_ns == kn]
                    why = 'eval: ignore zone' if abs(kn - t) < 0.06e9 and len(NP) and np.hypot(NP.x - g.x, NP.y - g.y).min() <= 1.0 else 'eval: range/height'
                    F[('all', why)] += 1; F[('mov', why)] += mv; continue
                dd = np.hypot(Ld[:, 0] - g.x, Ld[:, 1] - g.y) if len(Ld) else np.array([9.0])
                td = np.hypot(Lt[:, 0] - g.x, Lt[:, 1] - g.y) if len(Lt) else np.array([9.0])
                if dd.min() <= 1.0:
                    if td.min() <= 1.0: cause = 'held static' if St[int(td.argmin())] == 'static' else 'unconfirmed'
                    else: cause = 'coast+cluster' if td.min() <= 2.5 else 'cluster only'
                else: cause = 'coast no clus.' if td.min() <= 1.0 else 'no cluster'
                F[('all', cause)] += 1; F[('mov', cause)] += mv
    for s in ('mov', 'all'):
        n = F[(s, 'rows')]; lost = sum(F[(s, c)] for c in CAUSES)
        print(f'{tag:7s} {s}: rows {n} lost {100 * lost / max(1, n):.1f}% | ' + ', '.join(f'{c} {100 * F[(s, c)] / max(1, n):.1f}' for c in CAUSES))
    print(f'{tag:7s} person IDSW: ' + ', '.join(f'{k} {v}' for k, v in sorted(I.items())), flush=True)
