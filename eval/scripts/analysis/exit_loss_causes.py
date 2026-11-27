#!/usr/bin/env python3
"""Why does SINS lose an object after it leaves the camera FOV? (27/11, analysis of existing final-run bags only)

Same exit events as persistence.py (GT agent goes from inside to outside the camera FOV, matched by a published D/PD
SINS box in the last 1 s before). At the FIRST annotated frame after the exit where no published D/PD box is within
1 m (the loss), look at everything SINS had there:
  same_track_static      the track matched before the exit is still alive near the object (<= 1 m) but labelled static
  same_track_unpub       ... alive near the object but not published (label 'tracked' / unconfirmed / coasting)
  same_track_far         ... alive but > 1 m from the object (drifted / predicted away / box centre off)
  other_track_static     another track near the object, labelled static (e.g. a re-born track without class)
  other_track_unpub      another track near the object, not published
  detection_no_track     a fused LiDAR detection within 1 m but no track there
  nothing                no detection within 1 m (occluded, too sparse, out of range): the legitimate loss
plus, for the track matched before the exit: did it carry a class at the exit, and the GT state (moving / still) at
the loss. With --gap G only losses longer than G s are counted (short flicker skipped), as persistence.py --gap.

usage: exit_loss_causes.py --run <SINS run bag> --gt <w>_gt.csv --odom <w>_odom --out x.json
       exit_loss_causes.py --pool a.json b.json ...
"""
import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader
from scipy.optimize import linear_sum_assignment

E = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(E))
import coda_eval as ce                                                     # noqa: E402
from harness.ros_io import load_obstacles, load_odometry, stamp_ns, typestore   # noqa: E402
from harness.tracks import objects                                         # noqa: E402

STATES = {'static', 'dynamic', 'potentially_dynamic', 'tracked'}


def load_det3d(bag, topic):
    """Detection3DArray -> {t_ns: DataFrame(id, x, y, label, cls)}."""
    out = {}
    with AnyReader([Path(bag)], default_typestore=typestore()) as r:
        cs = [c for c in r.connections if c.topic == topic]
        for c, _, raw in r.messages(cs):
            m = r.deserialize(raw, c.msgtype)
            rows = []
            for d in m.detections:
                lab = [h.hypothesis.class_id for h in d.results if h.hypothesis.class_id in STATES]
                cls = [h.hypothesis.class_id for h in d.results if h.hypothesis.class_id not in STATES]
                try:
                    tid = int(d.id.split('_')[-1])
                except ValueError:
                    tid = -1
                rows.append(dict(id=tid, x=d.bbox.center.position.x, y=d.bbox.center.position.y,
                                 label=lab[0] if lab else '', cls=cls[0] if cls else ''))
            out[stamp_ns(m.header)] = pd.DataFrame(rows, columns=['id', 'x', 'y', 'label', 'cls'])
    return out


def nearest(d, t, tol=0.15e9):
    if not d:
        return None
    ks = np.fromiter(d.keys(), dtype=np.int64)
    k = int(ks[np.argmin(np.abs(ks - t))])
    return d[k] if abs(k - int(t)) <= tol else None


def analyse(a):
    odom = load_odometry(a.odom)
    ns = argparse.Namespace(camera_only=False, dyn_speed=0.3, ignore_undefined=False, min_pts=1, min_range=1.2,
                            pd_everywhere=True, pd_prev_observable=False, range=a.range, state_buffer=0.0)
    gt, _, _, _ = ce.build_gt(a.gt, odom, ns)
    gt = objects(gt)
    raw = pd.read_csv(a.gt, keep_default_na=False)
    raw = raw[raw['track_id'].astype(int) >= 0]
    fov = {(int(t), int(i)): str(f) == 'True' for t, i, f in zip(raw['t_ns'], raw['track_id'], raw['in_fov'])}
    pred, _, _ = load_obstacles(a.run)
    pred = objects(pred)
    pred = pred[pred['state'].isin(['dynamic', 'potentially_dynamic'])]
    pby = {t: g for t, g in pred.groupby('t_ns')}
    tracks = load_det3d(a.run, '/sins/tracker/tracks')
    dets = load_det3d(a.run, '/sins/dbscan_detector/fused/detections3d')
    frames = np.sort(np.unique(gt['t_ns']))
    per = {}                                     # gid -> list of (t, matched id, in_fov, state, x, y)
    for t in frames:
        g = gt[gt['t_ns'] == t]
        p = nearest(pby, t)
        m = {}
        if p is not None and len(p) and len(g):
            d = np.hypot(g['x'].to_numpy()[:, None] - p['x'].to_numpy()[None], g['y'].to_numpy()[:, None] - p['y'].to_numpy()[None])
            r, c = linear_sum_assignment(np.where(d <= 1.0, d, 1e6))
            m = {int(g['track_id'].iloc[i]): int(p['track_id'].iloc[j]) for i, j in zip(r, c) if d[i, j] <= 1.0}
        for gid, s, x, y in zip(g['track_id'], g['state'], g['x'], g['y']):
            per.setdefault(int(gid), []).append((t, m.get(int(gid)), fov.get((int(t), int(gid)), False), s, x, y))
    ev = []
    for gid, seq in per.items():
        for k in range(1, len(seq)):
            t, _, inf, _, _, _ = seq[k]
            tp, _, inf_p, _, _, _ = seq[k - 1]
            if (t - tp) / 1e9 > 0.35 or not (inf_p and not inf):
                continue
            pre = [x[1] for x in seq[:k] if (t - x[0]) / 1e9 <= 1.0 and x[1] is not None]
            if not pre:
                continue
            pre_id = pre[-1]
            tr0 = nearest(tracks, seq[k - 1][0])
            had_cls = bool(tr0 is not None and len(tr0[(tr0.id == pre_id) & (tr0.cls != '')]))
            loss, last_ok = None, t
            for j in range(k, len(seq)):
                tj, mj, infj = seq[j][0], seq[j][1], seq[j][2]
                if infj:
                    break
                if mj is not None:
                    last_ok = tj
                    continue
                if a.gap <= 0 or (tj - last_ok) / 1e9 > a.gap:
                    # cause at the first unmatched frame of this loss
                    j0 = j if a.gap <= 0 else next(q for q in range(k, j + 1) if seq[q][0] > last_ok)
                    loss = seq[j0]
                    break
            if loss is None:
                continue
            tl, _, _, st, x, y = loss
            tr, de = nearest(tracks, tl), nearest(dets, tl)
            cause = 'nothing'
            if tr is not None and len(tr):
                dd = np.hypot(tr.x - x, tr.y - y)
                same = tr[tr.id == pre_id]
                if len(same) and float(np.hypot(same.x.iloc[0] - x, same.y.iloc[0] - y)) <= 1.0:
                    cause = 'same_track_static' if same.label.iloc[0] == 'static' else 'same_track_unpub'
                elif (dd <= 1.0).any():
                    near = tr[dd <= 1.0]
                    cause = 'other_track_static' if (near.label == 'static').any() else 'other_track_unpub'
                elif len(same):
                    cause = 'same_track_far'
            if cause == 'nothing' and de is not None and len(de) and (np.hypot(de.x - x, de.y - y) <= 1.0).any():
                cause = 'detection_no_track'
            ev.append(dict(gid=gid, t_exit=float(t / 1e9), t_loss=float(tl / 1e9), after_s=round((tl - t) / 1e9, 2),
                           state=st, had_class=had_cls, cause=cause))
    return ev


def summary(ev):
    df = pd.DataFrame(ev)
    if df.empty:
        return dict(n=0)
    out = dict(n=len(df), cause=dict(Counter(df.cause)))
    for k, g in df.groupby('state'):
        out[f'cause_{k}'] = dict(Counter(g.cause))
    for k, g in df.groupby('had_class'):
        out[f'cause_had_class_{k}'] = dict(Counter(g.cause))
    out['median_after_s'] = float(df.after_s.median())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run')
    ap.add_argument('--gt')
    ap.add_argument('--odom')
    ap.add_argument('--range', type=float, default=20.0)
    ap.add_argument('--gap', type=float, default=0.0)
    ap.add_argument('--out')
    ap.add_argument('--pool', nargs='+')
    a = ap.parse_args()
    if a.pool:
        ev = [e for f in a.pool for e in json.load(open(f))['events']]
        res = dict(files=len(a.pool), summary=summary(ev))
    else:
        ev = analyse(a)
        res = dict(run=a.run, gap=a.gap, events=ev, summary=summary(ev))
    print(json.dumps(res['summary'], indent=1))
    if a.out:
        json.dump(res, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
