#!/usr/bin/env python3
"""
Point-level moving-object methods (M-detector) -> unified tracks, by one fixed clustering of their dynamic points.

These methods label points, not objects. As LV-DOT's authors did, the dynamic points of every output message are
clustered into boxes with a single, fixed rule shared by every point-level method: DBSCAN in the ground plane
(eps 0.5 m, at least 5 points), axis-aligned box of each cluster, all labelled 'dynamic'. No tracking is added, so
every box gets its own id and only detection metrics apply (identity would measure our clustering, not the method).

The points are expected in the world (odometry) frame, as M-detector publishes them; time = header stamp.
"""

from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from sklearn.cluster import DBSCAN

from harness.tracks import EMPTY_FRAME_ID, make_tracks

EPS, MIN_POINTS = 0.5, 5


def xyz_of(msg):
    f = {fl.name: fl for fl in msg.fields}
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    return np.stack([buf[:, f[k].offset:f[k].offset + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], axis=1)


def load_dynamic_points(bag, topic='/m_detector/point_out'):
    rows, nid = [], 0
    with AnyReader([Path(bag)]) as r:
        cs = [c for c in r.connections if c.topic == topic]
        if not cs:
            raise ValueError(f'{bag}: no {topic}')
        for c, t, raw in r.messages(cs):
            m = r.deserialize(raw, c.msgtype)
            ts = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec or t
            p = xyz_of(m)
            p = p[np.isfinite(p).all(axis=1)]
            labels = DBSCAN(eps=EPS, min_samples=MIN_POINTS).fit_predict(p[:, :2]) if len(p) >= MIN_POINTS else []
            ks = [k for k in set(labels) if k >= 0]
            if not ks:
                rows.append(dict(t_ns=ts, track_id=EMPTY_FRAME_ID))
                continue
            for k in ks:
                q = p[labels == k]
                lo, hi = q.min(axis=0), q.max(axis=0)
                c3 = (lo + hi) / 2
                rows.append(dict(t_ns=ts, track_id=nid, x=c3[0], y=c3[1], z=c3[2], l=hi[0] - lo[0], w=hi[1] - lo[1],
                                 h=hi[2] - lo[2], yaw=0.0, vx=0.0, vy=0.0, vz=0.0, state='dynamic', score=float(len(q)),
                                 **{'class': ''}))
                nid += 1
    return make_tracks(rows)
