"""Moving-pedestrian funnel on CODa windows: for each annotated pedestrian (visible, 1.2-25 m), is there within 1 m
(a) a detector cluster, (b) a tracker track (and its state), (c) an output obstacle. usage: <run> <prefix> [...]"""
import sys
sys.path.insert(0, '/home/duccio/Desktop/jo_sim_sins/eval')
import collections, numpy as np, pandas as pd
from pathlib import Path
from rosbags.highlevel import AnyReader
from harness.ros_io import typestore, load_obstacles, load_odometry
from harness.tracks import objects

def det3d(run, tp):
    out = collections.defaultdict(list)
    with AnyReader([Path(run)], default_typestore=typestore()) as r:
        for c, _, raw in r.messages([c for c in r.connections if c.topic == tp]):
            m = r.deserialize(raw, c.msgtype); t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            for d in m.detections:
                p, s = d.bbox.center.position, d.bbox.size
                st = d.results[0].hypothesis.class_id if d.results else ''
                out[t].append((p.x, p.y, max(s.x, s.y), s.z, st, d.id, d.results[0].hypothesis.score if d.results else 0, [h.hypothesis.class_id for h in d.results]))
    return out

def nearest(store, ts, t):
    k = ts[np.argmin(np.abs(ts - t))]
    return store[k] if abs(k - t) < 0.06e9 else []

