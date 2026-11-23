#!/usr/bin/env python3
"""Lab: where do a run's person FNs come from? (23/11, DM vs DC on seq 1950). Uses the evaluated outputs of run_own_eval
(<out>/<seq>/gt/tracks.csv and <out>/<seq>/<run>/tracks.csv, mocap frame) and the run bag (fused detections and tracker tracks,
odom frame, mapped to mocap with the run's alignment). A GT person row is a FN when no published track is within 1 m at the
nearest published frame (<= 60 ms). Each FN is attributed, at the nearest detector/tracker message:
  merged     the GT position lies inside a fused detection larger than a person (max(l,w) > 1.2 m): absorbed into a bigger object
  bus-cap    ... and that box's extent exceeds the old lab cap (3.0 x 3.0 x 2.5 m / 12 m3): it exists only thanks to the bus cap
  static     a person-size detection is there and a tracker track within 1 m is 'static' (held static by the motion rules)
  unpub      a person-size detection is there, a tracker track within 1 m is D/PD but not published (or none) (not confirmed)
  nodet      no fused detection within 1 m (detector loss), split by whether a tracker track is within 1 m (coasting) or not
usage: lab_fn_seq.py <eval out dir> <seq> <runs dir> RUN [RUN ...]"""
import json, sys, collections
from pathlib import Path
import numpy as np, pandas as pd
from rosbags.highlevel import AnyReader
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from harness.gt_mocap import compose   # noqa: E402

O, seq, R = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
gt = pd.read_csv(O / seq / 'gt' / 'tracks.csv'); gt = gt[gt.track_id >= 0]
st = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
for run in sys.argv[4:]:
    al = json.load(open(O / f'alignment_{run}.json')); T = np.array(al['T_mocap_odom'][seq]); dl = int(round(al['delta'][seq] * 1e9))
    pr = pd.read_csv(O / seq / run / 'tracks.csv'); pr = pr[pr.track_id >= 0]
    pts = np.sort(pd.read_csv(O / seq / run / 'tracks.csv').t_ns.unique())
    det, trk = {}, {}
    with AnyReader([R / f'{run}_{seq}']) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic in ('/sins/dbscan_detector/fused/detections3d', '/sins/tracker/tracks')]):
            m = r.deserialize(raw, c.msgtype); L = []
            for d in m.detections:
                b = d.bbox; q = compose(T, np.array([b.center.position.x, b.center.position.y, 0.0]))
                L.append((q[0], q[1], b.size.x, b.size.y, b.size.z, d.results[0].hypothesis.class_id if d.results else ''))
            (det if 'dbscan' in c.topic else trk)[st(m.header) + dl] = L
    dt, tt = np.array(sorted(det)), np.array(sorted(trk))
    near = lambda arr, t: arr[np.argmin(np.abs(arr - t))]
    C = collections.Counter(); ex = collections.defaultdict(list); n = 0
    for _, g in gt.iterrows():
        k = near(pts, g.t_ns)
        P = pr[pr.t_ns == k] if abs(k - g.t_ns) < 0.06e9 else pr.iloc[:0]
        n += 1
        if len(P) and np.hypot(P.x - g.x, P.y - g.y).min() <= 1.0: C['found'] += 1; continue
        D = det[near(dt, g.t_ns)]; K = trk[near(tt, g.t_ns)]
        ins = [d for d in D if abs(d[0] - g.x) <= d[2] / 2 + 0.2 and abs(d[1] - g.y) <= d[3] / 2 + 0.2]   # axis-aligned approx
        big = [d for d in ins if max(d[2], d[3]) > 1.2]
        close = [d for d in D if np.hypot(d[0] - g.x, d[1] - g.y) <= 1.0]
        tk = [k_ for k_ in K if np.hypot(k_[0] - g.x, k_[1] - g.y) <= 1.0]
        if big and not any(max(d[2], d[3]) <= 1.2 for d in close):
            cap = any(max(d[2], d[3]) > 3.0 or d[4] > 2.5 or d[2] * d[3] * d[4] > 12.0 for d in big)
            cause = 'bus-cap' if cap else 'merged'
        elif close:
            cause = 'static' if any(k_[5] == 'static' for k_ in tk) else 'unpub'
        else:
            cause = 'nodet+track' if tk else 'nodet'
        C[cause] += 1
        if len(ex[cause]) < 3 or (len(ex[cause]) < 50 and g.t_ns - ex[cause][-1] > 3e9): ex[cause].append(g.t_ns)
    t0 = gt.t_ns.min()
    print(f'{run}: GT rows {n}, found {C["found"]}, FN {n - C["found"]}: ' + ', '.join(f'{k} {C[k]}' for k in
          ('merged', 'bus-cap', 'static', 'unpub', 'nodet+track', 'nodet')) + ' | examples [s from seq start]: ' +
          '; '.join(f'{k} ' + ','.join(f'{(t - t0) / 1e9:.1f}' for t in v[:3]) for k, v in ex.items()))
